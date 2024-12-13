#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_crc.h>
#include <cstring>
#include <nvs_handle.hpp>
#include <mbedtls/sha256.h>
#include <esp_ota_ops.h>
#include <esp_mac.h>
#include <esp_flash.h>
#include <esp_timer.h>
#include <soc/rtc_cntl_reg.h>
#include "tcfg_client.hpp"

esp_err_t tcfg_client::init(tcfg_wire_if *_wire_if)
{
    wire_if = _wire_if;
    if (wire_if == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xTaskCreateWithCaps(rx_task, "tcfg_wire_rx", 32768, this, tskIDLE_PRIORITY + 1, &rx_task_handle, MALLOC_CAP_SPIRAM) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to create receive task");
        return ESP_ERR_NO_MEM;
    }

    state_evt_group = xEventGroupCreate();
    if (state_evt_group == nullptr) {
        ESP_LOGE(TAG, "Failed to create state event group");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void tcfg_client::rx_task(void *_ctx)
{
    auto *ctx = (tcfg_client *)_ctx;

    while (true) {
        uint8_t *pkt_ptr = nullptr;
        size_t read_len = 0;
        if (!ctx->wire_if->begin_read(&pkt_ptr, &read_len, portMAX_DELAY)) {
            ESP_LOGE(TAG, "Rx: read fail");
            vTaskDelay(1);
            continue;
        }

        if (pkt_ptr == nullptr) {
            ESP_LOGE(TAG, "Rx: ringbuf returned null");
            vTaskDelay(1);
            continue;
        }

        ctx->handle_rx_pkt(pkt_ptr, read_len);
        ctx->wire_if->finalise_read(pkt_ptr);
    }
}

void tcfg_client::handle_rx_pkt(const uint8_t *buf, size_t decoded_len)
{
    if (buf == nullptr || decoded_len < sizeof(tcfg_client::header)) {
        return;
    }

    auto *header = (tcfg_client::header *)buf;

    uint16_t expected_crc = header->crc;
    header->crc = 0;

    size_t pkt_len_with_hdr = header->len + sizeof(tcfg_client::header);
    uint16_t actual_crc = get_crc16(buf, pkt_len_with_hdr);
    if (actual_crc != expected_crc) {
        ESP_LOGE(TAG, "Incoming packet CRC corrupted, expect 0x%x, actual 0x%x decode len %u", expected_crc, actual_crc, pkt_len_with_hdr);
        send_nack();
        return;
    }

    switch (header->type) {
        case PKT_GET_DEVICE_INFO: {
            send_dev_info();
            break;
        }

        case PKT_GET_CONFIG: {
            auto *payload = (tcfg_client::cfg_pkt *)(buf + sizeof(tcfg_client::header));
            get_cfg_from_nvs(payload->ns, payload->key, payload->type);
            break;
        }

        case PKT_SET_CONFIG: {
            auto *payload = (tcfg_client::cfg_pkt *)(buf + sizeof(tcfg_client::header));
            set_cfg_to_nvs(payload->ns, payload->key, payload->type, payload->value, payload->val_len);
            break;
        }

        case PKT_DEL_CONFIG: {
            auto *payload = (tcfg_client::del_cfg_pkt *)(buf + sizeof(tcfg_client::header));
            delete_cfg(payload->ns, payload->key);
            break;
        }

        case PKT_NUKE_CONFIG: {
            auto *payload = (tcfg_client::del_cfg_pkt *)(buf + sizeof(tcfg_client::header));
            nuke_cfg(payload->ns);
            break;
        }

        case PKT_PING: {
            ESP_LOGI(TAG, "Got PING!");
            send_ack();
            break;
        }

        case PKT_GET_UPTIME: {
            auto *pkt = (tcfg_client::uptime_req_pkt *)(buf + sizeof(tcfg_client::header));
            handle_uptime(pkt->realtime_ms);
            break;
        }

        case PKT_REBOOT: {
            ESP_LOGW(TAG, "Reboot requested!");
            send_ack();
            vTaskDelay(pdMS_TO_TICKS(3500)); // Wait for a while to get the ACK sent...
            esp_restart();
            break;
        }

        case PKT_REBOOT_BOOTLOADER: {
            ESP_LOGW(TAG, "Reboot to BL requested!");
            send_ack();
            vTaskDelay(pdMS_TO_TICKS(3500)); // Wait for a while to get the ACK sent...
            REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
            esp_restart();
            break;
        }

        case PKT_BEGIN_FILE_WRITE: {
            auto *payload = (tcfg_client::path_pkt *)(buf + sizeof(tcfg_client::header));
            handle_begin_file_write(payload->path, payload->len);
            break;
        }

        case PKT_FILE_CHUNK: {
            auto *payload = (uint8_t *)(buf + sizeof(tcfg_client::header));
            handle_file_chunk(payload, header->len);
            break;
        }

        case PKT_DELETE_FILE: {
            auto *payload = (tcfg_client::path_pkt *)(buf + sizeof(tcfg_client::header));
            handle_file_delete(payload->path);
            break;
        }

        case PKT_GET_FILE_INFO: {
            auto *payload = (tcfg_client::path_pkt *)(buf + sizeof(tcfg_client::header));
            handle_get_file_info(payload->path);
            break;
        }

        case PKT_BEGIN_OTA: {
            handle_ota_begin();
            break;
        }

        case PKT_OTA_CHUNK: {
            auto *chunk = (uint8_t *)(buf + sizeof(tcfg_client::header));
            handle_ota_chunk(chunk, header->len);
            break;
        }

        case PKT_OTA_COMMIT: {
            handle_ota_commit();
            break;
        }

        default: {
            ESP_LOGW(TAG, "Unknown packet type 0x%x received", header->type);
            send_nack();
            break;
        }
    }
}

uint16_t tcfg_client::get_crc16(const uint8_t *buf, size_t len, uint16_t init)
{
//  * CRC-16/XMODEM, poly= 0x1021, init = 0x0000, refin = false, refout = false, xorout = 0x0000
// *     crc = ~crc16_be((uint16_t)~0x0000, buf, length);
    if (buf == nullptr || len < 1) {
        return 0;
    }

    return ~esp_crc16_be((uint16_t)~init, buf, len);
}

esp_err_t tcfg_client::send_pkt(tcfg_client::pkt_type type, const uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    if (buf == nullptr && len > 0) return ESP_ERR_INVALID_ARG;

    tcfg_client::header header = {};
    header.type = type;
    header.len = len;
    header.crc = 0; // Set later
    uint16_t crc = get_crc16((uint8_t *) &header, sizeof(header));

    // When packet has no data body, just send header (e.g. ACK)
    if (buf == nullptr || len < 1) {
        header.crc = crc;
        return encode_and_tx((uint8_t *)&header, sizeof(header), nullptr, 0, timeout_ms);
    } else {
        crc = get_crc16(buf, len, crc);
        header.crc = crc;
        return encode_and_tx((uint8_t *)&header, sizeof(header), buf, len, timeout_ms);
    }
}

esp_err_t tcfg_client::encode_and_tx(const uint8_t *header_buf, size_t header_len, const uint8_t *buf, size_t len, uint32_t timeout_ticks)
{
    ESP_LOGD(TAG, "EncodeAndTx: len=%u + %u", header_len, len);
    if (!wire_if->write_response(header_buf, header_len, buf, len, timeout_ticks)) {
        ESP_LOGE(TAG, "Write failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t tcfg_client::send_ack(uint32_t timeout_ticks)
{
    return send_pkt(PKT_ACK, nullptr, 0, timeout_ticks);
}

esp_err_t tcfg_client::send_nack(int32_t ret, uint32_t timeout_ticks)
{
    tcfg_client::nack_pkt nack = {};
    nack.ret = ret;

    return send_pkt(PKT_NACK, (uint8_t *)&nack, sizeof(nack), timeout_ticks);
}

esp_err_t tcfg_client::send_dev_info(uint32_t timeout_ticks)
{
    tcfg_client::device_info_pkt dev_info = {};
    auto *desc = esp_app_get_description();
    if (desc->magic_word != ESP_APP_DESC_MAGIC_WORD) {
        ESP_LOGW(TAG, "DevInfo: invalid magic"); // Should we NACK here???
    }

    strncpy(dev_info.comp_date, desc->date, sizeof(device_info_pkt::comp_date));
    strncpy(dev_info.comp_time, desc->time, sizeof(device_info_pkt::comp_time));
    strncpy(dev_info.fw_ver, desc->version, sizeof(device_info_pkt::fw_ver));
    strncpy(dev_info.sdk_ver, desc->idf_ver, sizeof(device_info_pkt::sdk_ver));
    strncpy(dev_info.model_name, desc->project_name, sizeof(device_info_pkt::model_name));
    memcpy(dev_info.fw_hash, desc->app_elf_sha256, sizeof(device_info_pkt::fw_hash));

    auto ret = esp_efuse_mac_get_default(dev_info.mac_addr);
    ret = ret ?: esp_flash_read_unique_chip_id(esp_flash_default_chip, (uint64_t *)dev_info.flash_id);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read UID! ret=%d %s", ret, esp_err_to_name(ret));
        send_nack(ret);
        return ret;
    }

    return send_pkt(PKT_DEV_INFO, (uint8_t *)&dev_info, sizeof(dev_info), timeout_ticks);;
}

esp_err_t tcfg_client::send_chunk_ack(tcfg_client::chunk_state state, uint32_t aux, uint32_t timeout_ticks)
{
    tcfg_client::chunk_ack_pkt pkt = {};
    pkt.state = state;
    pkt.aux_info = aux;

    return send_pkt(PKT_CHUNK_ACK, (uint8_t *)&pkt, sizeof(pkt), timeout_ticks);;
}

esp_err_t tcfg_client::set_cfg_to_nvs(const char *ns, const char *key, nvs_type_t type, const void *value, size_t value_len)
{
    if (ns == nullptr || key == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;
    auto nv = nvs::open_nvs_handle(ns, NVS_READWRITE, &ret);
    if (!nv || ret != ESP_OK) {
        ESP_LOGE(TAG, "SetCfg: failed to set cfg, ret=%s", esp_err_to_name(ret));
        send_nack(ret);
        return ret;
    }

    switch (type) {
        case NVS_TYPE_U8: {
            uint8_t val = 0;
            memcpy(&val, value, sizeof(val));
            ret = ret ?: nv->set_item(key, val);
            break;
        }

        case NVS_TYPE_I8: {
            int8_t val = 0;
            if (sizeof(val) < value_len) {
                ESP_LOGE(TAG, "SetCfg: unexpected length: %u < %u", sizeof(val), value_len);
                return ESP_ERR_INVALID_SIZE;
            }

            memcpy(&val, value, sizeof(val));
            ret = ret ?: nv->set_item(key, val);
            break;
        }

        case NVS_TYPE_U16: {
            uint16_t val = 0;
            if (sizeof(val) < value_len) {
                ESP_LOGE(TAG, "SetCfg: unexpected length: %u < %u", sizeof(val), value_len);
                return ESP_ERR_INVALID_SIZE;
            }

            memcpy(&val, value, sizeof(val));
            ret = ret ?: nv->set_item(key, val);
            break;
        }

        case NVS_TYPE_I16: {
            int16_t val = 0;
            if (sizeof(val) < value_len) {
                ESP_LOGE(TAG, "SetCfg: unexpected length: %u < %u", sizeof(val), value_len);
                return ESP_ERR_INVALID_SIZE;
            }

            memcpy(&val, value, sizeof(val));
            ret = ret ?: nv->set_item(key, val);
            break;
        }

        case NVS_TYPE_U32: {
            uint32_t val = 0;
            if (sizeof(val) < value_len) {
                ESP_LOGE(TAG, "SetCfg: unexpected length: %u < %u", sizeof(val), value_len);
                return ESP_ERR_INVALID_SIZE;
            }

            memcpy(&val, value, sizeof(val));
            ret = ret ?: nv->set_item(key, val);
            break;
        }

        case NVS_TYPE_I32: {
            int32_t val = 0;
            if (sizeof(val) < value_len) {
                ESP_LOGE(TAG, "SetCfg: unexpected length: %u < %u", sizeof(val), value_len);
                return ESP_ERR_INVALID_SIZE;
            }

            memcpy(&val, value, sizeof(val));
            ret = ret ?: nv->set_item(key, val);
            break;
        }

        case NVS_TYPE_U64: {
            uint64_t val = 0;
            if (sizeof(val) < value_len) {
                ESP_LOGE(TAG, "SetCfg: unexpected length: %u < %u", sizeof(val), value_len);
                return ESP_ERR_INVALID_SIZE;
            }

            memcpy(&val, value, sizeof(val));
            ret = ret ?: nv->set_item(key, val);
            break;
        }
        case NVS_TYPE_I64: {
            int64_t val = 0;
            if (sizeof(val) < value_len) {
                ESP_LOGE(TAG, "SetCfg: unexpected length: %u < %u", sizeof(val), value_len);
                return ESP_ERR_INVALID_SIZE;
            }

            memcpy(&val, value, sizeof(val));
            ret = ret ?: nv->set_item(key, val);
            break;
        }

        case NVS_TYPE_STR: {
            if (value == nullptr) {
                ret = ESP_ERR_INVALID_ARG;
                break;
            }

            ret = ret ?: nv->set_string(key, (const char *)value);
            break;
        }

        case NVS_TYPE_BLOB: {
            if (value == nullptr || value_len < 1) {
                ret = ESP_ERR_INVALID_ARG;
                break;
            }

            ret = ret ?: nv->set_blob(key, value, value_len);
            break;
        }

        case NVS_TYPE_ANY: {
            ret = ESP_ERR_INVALID_ARG;
        }
    }

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SetCfg: %s:%s set OK", ns, key);
        send_ack();
    } else {
        ESP_LOGE(TAG, "SetCfg: %s:%s set fail: %d %s", ns, key, ret, esp_err_to_name(ret));
        send_nack(ret);
    }

    return ret;
}

esp_err_t tcfg_client::get_cfg_from_nvs(const char *ns, const char *key, nvs_type_t type)
{
    if (ns == nullptr || key == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;
    auto nv = nvs::open_nvs_handle(ns, NVS_READONLY, &ret);
    if (!nv || ret != ESP_OK) {
        ESP_LOGE(TAG, "SetCfg: failed to set cfg, ret=%s", esp_err_to_name(ret));
        send_nack(ret);
        return ret;
    }

    uint8_t tx_buf[TCFG_WIRE_MAX_PACKET_SIZE] = { 0 };
    auto *pkt = (tcfg_client::cfg_pkt *)tx_buf;

    switch (type) {
        case NVS_TYPE_U8: {
            uint8_t val = 0;
            size_t len = 0;
            ret = nv->get_item_size((nvs::ItemType)type, key, len);
            ret = ret ?: nv->get_item(key, val);
            pkt->val_len = len;
            memcpy(pkt->value, &val, sizeof(uint8_t));
            break;
        }
        case NVS_TYPE_I8: {
            int8_t val = 0;
            size_t len = 0;
            ret = nv->get_item_size((nvs::ItemType)type, key, len);
            ret = ret ?: nv->get_item(key, val);
            pkt->val_len = len;
            memcpy(pkt->value, &val, sizeof(int8_t));
            break;
        }
        case NVS_TYPE_U16: {
            uint16_t val = 0;
            size_t len = 0;
            ret = nv->get_item_size((nvs::ItemType)type, key, len);
            ret = ret ?: nv->get_item(key, val);
            pkt->val_len = len;
            memcpy(pkt->value, &val, sizeof(uint16_t));
            break;
        }
        case NVS_TYPE_I16: {
            int16_t val = 0;
            size_t len = 0;
            ret = nv->get_item_size((nvs::ItemType)type, key, len);
            ret = ret ?: nv->get_item(key, val);
            pkt->val_len = len;
            memcpy(pkt->value, &val, sizeof(int16_t));
            break;
        }
        case NVS_TYPE_U32: {
            uint32_t val = 0;
            size_t len = 0;
            ret = nv->get_item_size((nvs::ItemType)type, key, len);
            ret = ret ?: nv->get_item(key, val);
            pkt->val_len = len;
            memcpy(pkt->value, &val, sizeof(uint32_t));
            break;
        }
        case NVS_TYPE_I32: {
            int32_t val = 0;
            size_t len = 0;
            ret = nv->get_item_size((nvs::ItemType)type, key, len);
            ret = ret ?: nv->get_item(key, val);
            pkt->val_len = len;
            memcpy(pkt->value, &val, sizeof(int32_t));
            break;
        }

        case NVS_TYPE_U64: {
            uint64_t val = 0;
            size_t len = 0;
            ret = nv->get_item_size((nvs::ItemType)type, key, len);
            ret = ret ?: nv->get_item(key, val);
            pkt->val_len = len;
            memcpy(pkt->value, &val, sizeof(uint64_t));
            break;
        }

        case NVS_TYPE_I64: {
            int64_t val = 0;
            size_t len = 0;
            ret = nv->get_item_size((nvs::ItemType)type, key, len);
            ret = ret ?: nv->get_item(key, val);
            pkt->val_len = len;
            memcpy(pkt->value, &val, sizeof(int64_t));
            break;
        }

        case NVS_TYPE_STR: {
            size_t len = 0;
            ret = nv->get_item_size((nvs::ItemType)type, key, len);
            ret = ret ?: nv->get_string(key, (char *)pkt->value, sizeof(tx_buf));
            pkt->val_len = len;
            break;
        }

        case NVS_TYPE_BLOB: {
            size_t len = 0;
            ret = nv->get_item_size((nvs::ItemType)type, key, len);
            ret = ret ?: nv->get_blob(key, (void *)pkt->value, sizeof(tx_buf));
            pkt->val_len = len;
            break;
        }

        default: {
            ret = ESP_ERR_INVALID_ARG;
            break;
        }
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GetConfig: can't read config, ret=%d %s", ret, esp_err_to_name(ret));
        send_nack(ret);
    } else {
        size_t tx_len = sizeof(tcfg_client::cfg_pkt) + pkt->val_len;
        ESP_LOGI(TAG, "GetConfig: send cfg %s:%s len=%u", ns, key, tx_len);
        ret = send_pkt(PKT_CONFIG_RESULT, tx_buf, tx_len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "GetConfig: can't send config, ret=%d %s", ret, esp_err_to_name(ret));
        }
    }

    return ret;
}

esp_err_t tcfg_client::delete_cfg(const char *ns, const char *key)
{
    esp_err_t ret = ESP_OK;
    auto nv = nvs::open_nvs_handle(ns, NVS_READWRITE, &ret);
    if (!nv || ret != ESP_OK) {
        ESP_LOGE(TAG, "DeleteConfig: failed to delete cfg, ret=%s", esp_err_to_name(ret));
        send_nack(ret);
        return ret;
    }

    ret = nv->erase_item(key);
    ret = ret ?: nv->commit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DeleteConfig: failed to delete cfg, ret=%s", esp_err_to_name(ret));
        send_nack(ret);
        return ret;
    }

    return send_ack();
}

esp_err_t tcfg_client::nuke_cfg(const char *ns)
{
    esp_err_t ret = ESP_OK;
    auto nv = nvs::open_nvs_handle(ns, NVS_READWRITE, &ret);
    if (!nv || ret != ESP_OK) {
        ESP_LOGE(TAG, "NukeCfg: failed to nuke cfg namespace %s, ret=%s", ns, esp_err_to_name(ret));
        send_nack(ret);
        return ret;
    }

    ret = nv->erase_all();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NukeCfg: failed to nuke cfg, ret=%s", esp_err_to_name(ret));
        send_nack(ret);
        return ret;
    }

    return send_ack();
}

esp_err_t tcfg_client::handle_begin_file_write(const char *path, size_t expect_len)
{
    if (path == nullptr || expect_len < 1) {
        return ESP_ERR_INVALID_ARG;
    }

    file_expect_len = expect_len;
    fp = fopen(path, "wb");

    if (fp == nullptr) {
        ESP_LOGE(TAG, "BeginFileWrite: fopen() failed!");
        send_nack(-1);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t tcfg_client::handle_file_chunk(const uint8_t *buf, uint16_t len)
{
    if (fp == nullptr) {
        ESP_LOGE(TAG, "FileChunk: not started yet!");
        send_nack(ESP_ERR_INVALID_STATE);
        return ESP_ERR_INVALID_STATE;
    }

    if (buf == nullptr || len == 0) {
        ESP_LOGW(TAG, "FileChunk: abort requested");
        send_chunk_ack(CHUNK_ERR_ABORT_REQUESTED);
        fclose(fp);
        fp = nullptr;
        return ESP_OK;
    }

    if (ftell(fp) > file_expect_len) {
        ESP_LOGE(TAG, "FileChunk: file written more than it supposed to: %ld < %d", ftell(fp), file_expect_len);
        send_chunk_ack(chunk_state::CHUNK_ERR_INTERNAL, ESP_ERR_INVALID_STATE);
        return ESP_ERR_INVALID_STATE;
    }

    auto ret_len = fwrite(buf, 1, len, fp);
    if (ret_len < len) {
        ESP_LOGE(TAG, "FileChunk: can't write in full! ret_len=%d < %d", ret_len, len);
        send_chunk_ack(chunk_state::CHUNK_ERR_INTERNAL, ESP_ERR_INVALID_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }

    if (ftell(fp) == file_expect_len) {
        ESP_LOGE(TAG, "FileChunk: file written more than it supposed to: %ld < %d", ftell(fp), file_expect_len);
        send_chunk_ack(chunk_state::CHUNK_XFER_DONE, ftell(fp));
        fflush(fp);
        fclose(fp);
        fp = nullptr;
        return ESP_OK;
    }

    send_chunk_ack(chunk_state::CHUNK_XFER_NEXT, ftell(fp));
    return ESP_OK;
}

esp_err_t tcfg_client::handle_file_delete(const char *path)
{
    if (unlink(path) < 0) {
        send_nack(ESP_FAIL);
    }

    return send_ack();
}

esp_err_t tcfg_client::handle_get_file_info(const char *path)
{
    FILE *file_info_fp = fopen(path, "r");
    if (file_info_fp == nullptr) {
        ESP_LOGE(TAG, "GetFileInfo: Can't open");
        send_nack(ESP_ERR_NOT_FOUND);
        return ESP_ERR_NOT_FOUND;
    }

    if (fseek(file_info_fp, 0, SEEK_END) < 1) {
        ESP_LOGE(TAG, "GetFileInfo: Can't estimate length");
        send_nack(ESP_FAIL);
        return ESP_FAIL;
    }

    int32_t file_len = ftell(file_info_fp);
    if (file_len < 0) {
        ESP_LOGE(TAG, "GetFileInfo: Can't ftell() length");
        send_nack(ESP_ERR_INVALID_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }

    if (fseek(file_info_fp, 0, SEEK_SET) < 1) {
        ESP_LOGE(TAG, "GetFileInfo: Can't estimate length");
        send_nack(ESP_FAIL);
        return ESP_FAIL;
    }

    if (file_len == 0) {
        tcfg_client::file_info_pkt info_pkt = {};
        info_pkt.size = 0;

        ESP_LOGW(TAG, "GetFileInfo: file size 0, skip SHA256");
        return send_pkt(PKT_FILE_INFO, (uint8_t *)&info_pkt, sizeof(info_pkt));
    }

    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, /*is224=*/0);

    uint8_t buf[64] = { 0 };
    size_t read_len = 0;
    while ((read_len = fread(buf, 1, sizeof(buf), fp)) > 0) {
        mbedtls_sha256_update(&ctx, buf, read_len);
    }

    tcfg_client::file_info_pkt info_pkt = {};
    info_pkt.size = 0;
    if (mbedtls_sha256_finish(&ctx, info_pkt.hash) < 0) {
        ESP_LOGE(TAG, "GetFileInfo: Can't finalise SHA256");
        send_nack(ESP_FAIL);
        return ESP_FAIL;
    }

    return send_pkt(PKT_FILE_INFO, (uint8_t *)&info_pkt, sizeof(info_pkt));
}

esp_err_t tcfg_client::handle_ota_begin()
{
    if (ota_handle != 0) {
        ESP_LOGW(TAG, "OTA already started!");
        send_nack(ESP_ERR_INVALID_STATE);
        return ESP_ERR_INVALID_STATE;
    } else {
        curr_ota_part = esp_ota_get_next_update_partition(nullptr);
        if (curr_ota_part == nullptr) {
            ESP_LOGW(TAG, "OTA partition not present!");
            send_nack(ESP_ERR_NOT_SUPPORTED);
            return ESP_ERR_NOT_SUPPORTED;
        }

        auto ota_ret = esp_ota_begin(curr_ota_part, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
        if (ota_ret != ESP_OK) {
            ESP_LOGE(TAG, "OTA begin failed; ret=%d %s", ota_ret, esp_err_to_name(ota_ret));
            send_nack(ota_ret);
        } else {
            ESP_LOGW(TAG, "OTA begin");
        }
    }

    return send_ack();
}

esp_err_t tcfg_client::handle_ota_chunk(const uint8_t *buf, uint16_t len)
{
    if (ota_handle == 0) {
        ESP_LOGE(TAG, "OTA not started yet!");
        return send_nack(ESP_ERR_INVALID_STATE);;
    }

    if (len == 0) {
        ESP_LOGW(TAG, "OTA abort requested!");
        auto ret = esp_ota_abort(ota_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "OTA failed to abort! ret=%d %s", ret, esp_err_to_name(ret));
            send_chunk_ack(CHUNK_ERR_INTERNAL, ret);
            return ret;
        }

        curr_ota_chunk_offset += len;
        ota_handle = 0;
        return send_chunk_ack(CHUNK_ERR_ABORT_REQUESTED, curr_ota_chunk_offset);
    } else {
        auto ret = esp_ota_write(ota_handle, buf, len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "OTA failed to write chunk! ret=%d %s", ret, esp_err_to_name(ret));
            send_chunk_ack(CHUNK_ERR_INTERNAL, ret);
            return ret;
        }

        curr_ota_chunk_offset += len;
        return send_chunk_ack(CHUNK_XFER_NEXT, curr_ota_chunk_offset);
    }

    return ESP_OK;
}

esp_err_t tcfg_client::handle_ota_commit()
{
    if (ota_handle == 0) {
        ESP_LOGE(TAG, "OTA commit requested but not started!");
        send_nack(ESP_ERR_INVALID_STATE);

        ota_handle = 0;
        curr_ota_part = nullptr;
        curr_ota_chunk_offset = 0;
        return ESP_ERR_INVALID_STATE;
    }

    auto ret = esp_ota_end(ota_handle);
    ret = ret ?: esp_ota_set_boot_partition(curr_ota_part);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OTA failed to end! ret=%d %s", ret, esp_err_to_name(ret));
        send_nack(ret);

        ota_handle = 0;
        curr_ota_part = nullptr;
        curr_ota_chunk_offset = 0;
        return ret;
    }

    ota_handle = 0;
    curr_ota_part = nullptr;
    curr_ota_chunk_offset = 0;
    return send_ack();
}

esp_err_t tcfg_client::handle_uptime(uint64_t realtime_ms)
{
    if (realtime_ms != 0 && realtime_ms != UINT64_MAX) {
        struct timeval tv = {};
        tv.tv_sec = (time_t)(realtime_ms / 1000ULL);
        tv.tv_usec = (suseconds_t)((realtime_ms % 1000ULL) * 1000ULL);

        ESP_LOGI(TAG, "Uptime: got epoch: %llu", realtime_ms);
        settimeofday(&tv, nullptr);
    }

    tcfg_client::uptime_pkt pkt = {};
    pkt.last_rst_reason = esp_reset_reason();
    pkt.uptime = esp_timer_get_time();

    return send_pkt(PKT_UPTIME, (uint8_t *)&pkt, sizeof(pkt));;
}
