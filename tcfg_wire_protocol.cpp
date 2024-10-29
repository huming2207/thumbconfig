#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_crc.h>
#include <cstring>
#include <nvs_handle.hpp>
#include "tcfg_wire_protocol.hpp"

esp_err_t tcfg_wire_protocol::init(tcfg_wire_if *_wire_if)
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

void tcfg_wire_protocol::rx_task(void *_ctx)
{
    auto *ctx = (tcfg_wire_protocol *)_ctx;
    uint8_t decoded_buf[TCFG_WIRE_MAX_PACKET_SIZE] = { 0 };
    bool begin_read = false, esc = false;
    size_t decode_idx = 0;
    while (true) {
        uint8_t *next_ptr = nullptr;
        size_t read_len = 0;
        if (!ctx->wire_if->begin_read(&next_ptr, 1, &read_len, portMAX_DELAY)) {
            ESP_LOGE(TAG, "Rx: read fail");
            vTaskDelay(1);
            continue;
        }

        if (next_ptr == nullptr) {
            ESP_LOGE(TAG, "Rx: ringbuf returned null");
            vTaskDelay(1);
            continue;
        }

        uint8_t next_byte = *next_ptr;
        ctx->wire_if->finalise_read(next_ptr);

        switch (next_byte) {
            case SLIP_START: {
                begin_read = true;
                memset(decoded_buf, 0, sizeof(decoded_buf));
                decode_idx = 0;
                break;
            }

            case SLIP_END: {
                if (begin_read) {
                    ctx->handle_rx_pkt(decoded_buf, decode_idx);
                }

                begin_read = false; // Force it to stop anyway
                break;
            }

            case SLIP_ESC: {
                if (!begin_read) {
                    continue;
                }

                esc = true;
                break;
            }

            case SLIP_ESC_END: {
                if (!begin_read) {
                    continue;
                }

                if (esc) {
                    decoded_buf[decode_idx++] = SLIP_END;
                    esc = false;
                } else {
                    decoded_buf[decode_idx++] = SLIP_ESC_END;
                }

                break;
            }

            case SLIP_ESC_ESC: {
                if (!begin_read) {
                    continue;
                }

                if (esc) {
                    decoded_buf[decode_idx++] = SLIP_ESC;
                    esc = false;
                } else {
                    decoded_buf[decode_idx++] = SLIP_ESC_ESC;
                }

                break;
            }

            case SLIP_ESC_START: {
                if (!begin_read) {
                    continue;
                }

                if (esc) {
                    decoded_buf[decode_idx++] = SLIP_START;
                    esc = false;
                } else {
                    decoded_buf[decode_idx++] = SLIP_ESC_START;
                }

                break;
            }

            default: {
                decoded_buf[decode_idx++] = next_byte;
                break;
            }
        }
    }
}

void tcfg_wire_protocol::handle_rx_pkt(const uint8_t *buf, size_t decoded_len)
{
    if (buf == nullptr || decoded_len < sizeof(tcfg_wire_protocol::header)) {
        return;
    }

    auto *header = (tcfg_wire_protocol::header *)buf;

    uint16_t expected_crc = header->crc;
    header->crc = 0;

    uint16_t actual_crc = get_crc16(buf, decoded_len);
    if (actual_crc != expected_crc) {
        ESP_LOGW(TAG, "Incoming packet CRC corrupted, expect 0x%x, actual 0x%x", expected_crc, actual_crc);
        send_nack();
        return;
    }

    switch (header->type) {
        case PKT_DEVICE_INFO: {
            break;
        }

        case PKT_GET_CONFIG: {
            auto *payload = (tcfg_wire_protocol::cfg_pkt *)(buf + sizeof(tcfg_wire_protocol::header));
            get_cfg_from_nvs(payload->ns, payload->key, payload->type);
            break;
        }

        case PKT_SET_CONFIG: {
            auto *payload = (tcfg_wire_protocol::cfg_pkt *)(buf + sizeof(tcfg_wire_protocol::header));
            set_cfg_to_nvs(payload->ns, payload->key, payload->type, payload->value, payload->val_len);
            break;
        }

        case PKT_PING: {
            send_ack();
            break;
        }

        case PKT_BEGIN_FILE_WRITE: {
            break;
        }

        case PKT_FILE_CHUNK: {
            break;
        }

        default: {
            ESP_LOGW(TAG, "Unknown packet type 0x%x received", header->type);
            send_nack();
            break;
        }
    }
}

uint16_t tcfg_wire_protocol::get_crc16(const uint8_t *buf, size_t len, uint16_t init)
{
//  * CRC-16/XMODEM, poly= 0x1021, init = 0x0000, refin = false, refout = false, xorout = 0x0000
// *     crc = ~crc16_be((uint16_t)~0x0000, buf, length);
    if (buf == nullptr || len < 1) {
        return 0;
    }

    return ~esp_crc16_be((uint16_t)~init, buf, len);
}

esp_err_t tcfg_wire_protocol::send_pkt(tcfg_wire_protocol::pkt_type type, const uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    if (buf == nullptr && len > 0) return ESP_ERR_INVALID_ARG;

    tcfg_wire_protocol::header header = {};
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

esp_err_t tcfg_wire_protocol::encode_and_tx(const uint8_t *header_buf, size_t header_len, const uint8_t *buf, size_t len, uint32_t timeout_ticks)
{
    const uint8_t slip_esc_end[] = { SLIP_ESC, SLIP_ESC_END };
    const uint8_t slip_esc_esc[] = { SLIP_ESC, SLIP_ESC_ESC };
    const uint8_t slip_esc_start[] = { SLIP_ESC, SLIP_ESC_START };

    if (header_buf == nullptr || header_len < 1) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t slip_start = SLIP_START;
    if (!wire_if->write_response(&slip_start, 1, 1)) {
        ESP_LOGE(TAG, "Encode SLIP START failed");
        return ESP_FAIL;
    }

    for (size_t idx = 0; idx < header_len; idx += 1) {
        switch (header_buf[idx]) {
            case SLIP_START: {
                if (!wire_if->write_response(slip_esc_start, sizeof(slip_esc_start), 1)) {
                    ESP_LOGE(TAG, "Encode header SLIP_START failed, pos=%u", idx);
                    return ESP_FAIL;
                }

                break;
            }

            case SLIP_END: {
                if (!wire_if->write_response(slip_esc_end, sizeof(slip_esc_end), 1)) {
                    ESP_LOGE(TAG, "Encode header SLIP_END failed, pos=%u", idx);
                    return ESP_FAIL;
                }

                break;
            }

            case SLIP_ESC: {
                if (!wire_if->write_response(slip_esc_esc, sizeof(slip_esc_esc), 1)) {
                    ESP_LOGE(TAG, "Encode header SLIP_ESC failed, pos=%u", idx);
                    return ESP_FAIL;
                }

                break;
            }

            default: {
                if (!wire_if->write_response(&header_buf[idx], 1, 1)) {
                    ESP_LOGE(TAG, "Encode header failed, pos=%u", idx);
                    return ESP_FAIL;
                }
            }
        }
    }

    const uint8_t slip_end = SLIP_END;

    // If no payload, then end here
    if (buf == nullptr || len < 1) {
        if (!wire_if->write_response(&slip_end, sizeof(slip_end), 1)) {
            ESP_LOGE(TAG, "Encode END failed");
            return ESP_FAIL;
        }

        return ESP_OK;
    }

    for (size_t idx = 0; idx < len; idx += 1) {
        switch (buf[idx]) {
            case SLIP_START: {
                if (!wire_if->write_response(slip_esc_start, sizeof(slip_esc_start), 1)) {
                    ESP_LOGE(TAG, "Encode header SLIP_START failed, pos=%u", idx);
                    return ESP_FAIL;
                }

                break;
            }

            case SLIP_END: {
                if (!wire_if->write_response(slip_esc_end, sizeof(slip_esc_end), 1)) {
                    ESP_LOGE(TAG, "Encode header SLIP_END failed, pos=%u", idx);
                    return ESP_FAIL;
                }

                break;
            }

            case SLIP_ESC: {
                if (!wire_if->write_response(slip_esc_esc, sizeof(slip_esc_esc), 1)) {
                    ESP_LOGE(TAG, "Encode header SLIP_ESC failed, pos=%u", idx);
                    return ESP_FAIL;
                }

                break;
            }

            default: {
                if (!wire_if->write_response(&buf[idx], 1, 1)) {
                    ESP_LOGE(TAG, "Encode header failed, pos=%u", idx);
                    return ESP_FAIL;
                }
            }
        }
    }

    if (!wire_if->write_response(&slip_end, sizeof(slip_end), 1)) {
        ESP_LOGE(TAG, "Encode END failed");
        return ESP_FAIL;
    }

    if (!wire_if->flush(timeout_ticks)) {
        ESP_LOGE(TAG, "Flush failed");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t tcfg_wire_protocol::send_ack( uint32_t timeout_ticks)
{
    return send_pkt(PKT_ACK, nullptr, 0, timeout_ticks);
}

esp_err_t tcfg_wire_protocol::send_nack(int32_t ret, uint32_t timeout_ticks)
{
    tcfg_wire_protocol::nack_pkt nack = {};
    nack.ret = ret;

    return send_pkt(PKT_NACK, (uint8_t *)&nack, sizeof(nack), timeout_ticks);
}

esp_err_t tcfg_wire_protocol::send_dev_info(uint32_t timeout_ticks)
{
    return 0;
}

esp_err_t tcfg_wire_protocol::send_chunk_ack(tcfg_wire_protocol::chunk_ack state, uint32_t aux, uint32_t timeout_ticks)
{
    return 0;
}

esp_err_t tcfg_wire_protocol::set_cfg_to_nvs(const char *ns, const char *key, nvs_type_t type, const void *value, size_t value_len)
{
    if (ns == nullptr || key == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;
    auto nv = nvs::open_nvs_handle(ns, NVS_READWRITE, &ret);
    if (!nv || ret != ESP_OK) {
        ESP_LOGE(TAG, "SetCfg: failed to set cfg, ret=%s", esp_err_to_name(ret));
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

esp_err_t tcfg_wire_protocol::get_cfg_from_nvs(const char *ns, const char *key, nvs_type_t type)
{
    if (ns == nullptr || key == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;
    auto nv = nvs::open_nvs_handle(ns, NVS_READONLY, &ret);
    if (!nv || ret != ESP_OK) {
        ESP_LOGE(TAG, "SetCfg: failed to set cfg, ret=%s", esp_err_to_name(ret));
        return ret;
    }

    uint8_t tx_buf[TCFG_WIRE_MAX_PACKET_SIZE] = { 0 };
    auto *pkt = (tcfg_wire_protocol::cfg_pkt *)tx_buf;

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
        size_t tx_len = sizeof(tcfg_wire_protocol::cfg_pkt) + pkt->val_len;
        ESP_LOGI(TAG, "GetConfig: send cfg %s:%s len=%u", ns, key, tx_len);
        ret = send_pkt(PKT_CONFIG_RESULT, tx_buf, tx_len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "GetConfig: can't send config, ret=%d %s", ret, esp_err_to_name(ret));
        }
    }

    return ret;
}

esp_err_t tcfg_wire_protocol::handle_begin_file_write(const char *path, size_t expect_len)
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

esp_err_t tcfg_wire_protocol::handle_file_chunk(const uint8_t *buf, uint16_t len)
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
        send_chunk_ack(chunk_ack::CHUNK_ERR_INTERNAL, ESP_ERR_INVALID_STATE);
        return ESP_ERR_INVALID_STATE;
    }

    auto ret_len = fwrite(buf, 1, len, fp);
    if (ret_len < len) {
        ESP_LOGE(TAG, "FileChunk: can't write in full! ret_len=%d < %d", ret_len, len);
        send_chunk_ack(chunk_ack::CHUNK_ERR_INTERNAL, ESP_ERR_INVALID_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }

    if (ftell(fp) == file_expect_len) {
        ESP_LOGE(TAG, "FileChunk: file written more than it supposed to: %ld < %d", ftell(fp), file_expect_len);
        send_chunk_ack(chunk_ack::CHUNK_XFER_DONE, ftell(fp));
        fflush(fp);
        fclose(fp);
        fp = nullptr;
        return ESP_OK;
    }

    send_chunk_ack(chunk_ack::CHUNK_XFER_NEXT, ftell(fp));
    return ESP_OK;
}


