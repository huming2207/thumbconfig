#include <cstring>
#include <esp_random.h>
#include <esp_log.h>
#include "tcfg_wire_usb_cdc.hpp"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

esp_err_t tcfg_wire_usb_cdc::init(const char *serial_num, tinyusb_cdcacm_itf_t channel)
{
    if (serial_num == nullptr) {
        strncpy(sn_str, "1145141919810893", sizeof(sn_str));
    } else {
        strncpy(sn_str, serial_num, sizeof(sn_str));
    }

    cdc_channel = channel;
    static char lang[2] = {0x09, 0x04};
    static const char *desc_str[5] = {
            lang,                // 0: is supported language is English (0x0409)
            const_cast<char *>(CONFIG_TINYUSB_DESC_MANUFACTURER_STRING), // 1: Manufacturer
            const_cast<char *>(CONFIG_TINYUSB_DESC_PRODUCT_STRING),      // 2: Product
            sn_str,       // 3: Serials, should use chip ID
            const_cast<char *>(CONFIG_TINYUSB_DESC_PRODUCT_STRING),          // 4: CDC Interface
    };

    tinyusb_config_t tusb_cfg = {}; // the configuration using default values
    tusb_cfg.string_descriptor = (const char **)desc_str;
    tusb_cfg.device_descriptor = nullptr;
    tusb_cfg.self_powered = false;
    tusb_cfg.external_phy = false;
    tusb_cfg.string_descriptor_count = sizeof(desc_str) / sizeof(desc_str[0]);

    auto ret = tinyusb_driver_install(&tusb_cfg);


    acm_cfg.usb_dev = TINYUSB_USBDEV_0;
    acm_cfg.cdc_port = channel;
    acm_cfg.callback_rx = &serial_rx_cb;
    acm_cfg.callback_rx_wanted_char = nullptr;
    acm_cfg.callback_line_state_changed = nullptr;
    acm_cfg.callback_line_coding_changed = nullptr;

    ret = ret ?: tusb_cdc_acm_init(&acm_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TinyUSB driver install failed");
        return ret;
    }

    // TODO: put size in Kconfig later
    rx_rb = xRingbufferCreateWithCaps(65536, RINGBUF_TYPE_NOSPLIT, MALLOC_CAP_SPIRAM);
    if (rx_rb == nullptr) {
        ESP_LOGE(TAG, "Failed to create Rx ring buffer");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}


bool tcfg_wire_usb_cdc::begin_read(uint8_t **data_out, size_t buf_len, size_t *len_written, uint32_t wait_ticks)
{
    if (data_out == nullptr) {
        return false;
    }

    auto *ptr = (uint8_t *)xRingbufferReceiveUpTo(rx_rb, len_written, wait_ticks, buf_len);
    if (ptr == nullptr) {
        return false;
    }

    *data_out = ptr;
    return true;
}

bool tcfg_wire_usb_cdc::finalise_read(uint8_t *ret_ptr)
{
    if (ret_ptr == nullptr) {
        return false;
    }

    vRingbufferReturnItem(rx_rb, ret_ptr);
    return true;
}

bool tcfg_wire_usb_cdc::write_response(const uint8_t *data_in, size_t buf_len, uint32_t wait_ticks)
{
    return tinyusb_cdcacm_write_queue(cdc_channel, data_in, buf_len) > 0;
}

bool tcfg_wire_usb_cdc::flush(uint32_t wait_ticks)
{
    return tinyusb_cdcacm_write_flush(cdc_channel, wait_ticks) == ESP_OK;
}

bool tcfg_wire_usb_cdc::pause(bool force)
{
    has_force_paused = force;
    if (has_force_paused) {
        return tusb_cdc_acm_deinit(cdc_channel) == ESP_OK;
    } else {
        return tinyusb_cdcacm_unregister_callback(cdc_channel, CDC_EVENT_RX);
    }
}

bool tcfg_wire_usb_cdc::resume()
{
    if (has_force_paused) {
        return tusb_cdc_acm_init(&acm_cfg) == ESP_OK;
    } else {
        return tinyusb_cdcacm_register_callback(cdc_channel, CDC_EVENT_RX, serial_rx_cb);
    }
}

void tcfg_wire_usb_cdc::serial_rx_cb(int itf, cdcacm_event_t *event)
{
    auto *ctx = tcfg_wire_usb_cdc::instance();
    if (itf != ctx->cdc_channel || event == nullptr) {
        return;
    }

    if (event->type == CDC_EVENT_RX) {
        size_t rx_len_out = 0;
        do {
            uint8_t rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE] = { 0 };
            tinyusb_cdcacm_read(static_cast<tinyusb_cdcacm_itf_t>(itf), rx_buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_len_out);
            xRingbufferSend(ctx->rx_rb, rx_buf, rx_len_out, pdMS_TO_TICKS(1000)); // TODO: ticks should be in Kconfig
        } while (rx_len_out != 0);
    }
}
