#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_crc.h>
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
                break;
            }

            case SLIP_END: {
                if (begin_read) {
                    begin_read = false;
                }

                ctx->handle_rx_pkt(decoded_buf, decode_idx);
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
            break;
        }

        case PKT_SET_CONFIG: {
            break;
        }

        case PKT_PING: {
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

esp_err_t tcfg_wire_protocol::send_ack(uint16_t crc, uint32_t timeout_ticks)
{
    return send_pkt(PKT_ACK, nullptr, 0, timeout_ticks);
}

esp_err_t tcfg_wire_protocol::send_nack(uint32_t timeout_ticks)
{
    return send_pkt(PKT_NACK, nullptr, 0, timeout_ticks);
}

esp_err_t tcfg_wire_protocol::send_dev_info(uint32_t timeout_ticks)
{
    return 0;
}

esp_err_t tcfg_wire_protocol::send_chunk_ack(tcfg_wire_protocol::chunk_ack state, uint32_t aux, uint32_t timeout_ticks)
{
    return 0;
}
