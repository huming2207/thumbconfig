#pragma once

#include <esp_err.h>
#include "tcfg_wire_interface.hpp"
#include <tinyusb.h>
#include <tusb_cdc_acm.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"

class tcfg_wire_usb_cdc : public tcfg_wire_if
{
public:
    static tcfg_wire_usb_cdc *instance()
    {
        static tcfg_wire_usb_cdc _instance;
        return &_instance;
    }

    tcfg_wire_usb_cdc(tcfg_wire_usb_cdc const &) = delete;
    void operator=(tcfg_wire_usb_cdc const &) = delete;

public:
    enum slip_byte : uint8_t {
        SLIP_START = 0x5a,
        SLIP_END = 0xc0,
        SLIP_ESC = 0xdb,
        SLIP_ESC_END = 0xdc,
        SLIP_ESC_ESC = 0xdd,
        SLIP_ESC_START = 0xde,
    };

public:
    esp_err_t init(const char *serial_num, tinyusb_cdcacm_itf_t channel = TINYUSB_CDC_ACM_0);
    bool begin_read(uint8_t **data_out, size_t *len_written, uint32_t wait_ticks) override;
    bool finalise_read(uint8_t *ret_ptr) override;
    bool write_response(const uint8_t *header_out, size_t header_len, const uint8_t *payload_out, size_t payload_len, uint32_t wait_ticks) override;
    bool flush(uint32_t wait_ticks) override;
    bool pause(bool force) override;
    bool resume() override;
    size_t max_packet_size() override;
    bool ditch_read() override;

private:
    tcfg_wire_usb_cdc() = default;
    static void serial_rx_cb(int itf, cdcacm_event_t *event);

private:
    static const constexpr size_t MAX_PACKET_SIZE = 8192;
    static const constexpr char TAG[] = "tcfg_usbcdc";
    bool has_force_paused = false;
    RingbufHandle_t rx_rb = nullptr;
    bool slip_esc = false;
    tinyusb_cdcacm_itf_t cdc_channel = TINYUSB_CDC_ACM_MAX;
    uint8_t *curr_decoded_buf = nullptr;
    size_t decode_idx = 0;
    tinyusb_config_cdcacm_t acm_cfg = {};
    char sn_str[32] = { 0 };
};

