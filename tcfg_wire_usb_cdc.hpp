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

    enum event : uint32_t {
        EVT_NEW_PACKET = BIT(0),
        EVT_READING_PKT = BIT(1),
        EVT_SLIP_ERROR = BIT(2),
    };

    enum pkt_type : uint8_t {
        PKT_ACK = 0,
        PKT_DEVICE_INFO = 1,
        PKT_CURR_CONFIG = 2,
        PKT_SET_CONFIG = 3,
        PKT_GET_ALGO_METADATA = 4,
        PKT_SET_ALGO_METADATA = 5,
        PKT_GET_FW_METADATA = 6,
        PKT_SET_FW_METADATA = 7,
        PKT_PING = 8,
        PKT_DATA_CHUNK = 9,
        PKT_CHUNK_ACK = 10,
        PKT_NACK = 0xff,
    };

    enum chunk_ack : uint8_t {
        CHUNK_XFER_DONE = 0,
        CHUNK_XFER_NEXT = 1,
        CHUNK_ERR_CRC32_FAIL = 2,
        CHUNK_ERR_INTERNAL = 3,
        CHUNK_ERR_ABORT_REQUESTED = 4,
        CHUNK_ERR_NAME_TOO_LONG = 5,
    };

    struct __attribute__((packed)) chunk_ack_pkt {
        chunk_ack state;

        // Can be:
        // 1. Next chunk offset (when state == 1)
        // 2. Expected CRC32 (when state == 2)
        // 3. Max length allowed (when state == 3)
        // 4. Just 0 (when state == anything else?)
        uint32_t aux_info;
    };

    struct __attribute__((packed)) header {
        pkt_type type;
        uint8_t len;
        uint16_t crc;
    };

    struct __attribute__((packed)) ack_pkt {
        pkt_type type;
        uint8_t len;
        uint16_t crc;
    };

    struct __attribute__((packed)) device_info {
        uint8_t mac_addr[6];
        uint8_t flash_id[8];
        char esp_idf_ver[32];
        char dev_model[32];
        char dev_build[32];
    };

    struct __attribute__((packed)) file_info {
        uint32_t crc; // 4
        uint32_t len; // 4
        char path[UINT8_MAX];
    }; // 8 bytes

    struct __attribute__((packed)) chunk_pkt {
        uint8_t len;
        uint8_t buf[UINT8_MAX];
    };

public:
    esp_err_t init(const char *serial_num, tinyusb_cdcacm_itf_t channel = TINYUSB_CDC_ACM_0);
    bool begin_read(uint8_t **data_out, size_t buf_len, size_t *len_written, uint32_t wait_ticks) override;
    bool finalise_read(uint8_t *ret_ptr) override;
    bool write_response(const uint8_t *data_in, size_t buf_len, uint32_t wait_ticks) override;
    bool flush(uint32_t wait_ticks) override;
    bool pause(bool force) override;
    bool resume() override;

private:
    tcfg_wire_usb_cdc() = default;
    static void serial_rx_cb(int itf, cdcacm_event_t *event);

private:
    static const constexpr char TAG[] = "tcfg_usbcdc";
    bool has_force_paused = false;
    RingbufHandle_t rx_rb = nullptr;
    tinyusb_cdcacm_itf_t cdc_channel = TINYUSB_CDC_ACM_MAX;
    tinyusb_config_cdcacm_t acm_cfg = {};
    char sn_str[32] = { 0 };
};

