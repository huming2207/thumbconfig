#pragma once

#include "tcfg_wire_protocol.hpp"
#include "tcfg_wire_interface.hpp"
#include <nvs.h>
#include <nvs_flash.h>

#define TCFG_WIRE_MAX_PACKET_SIZE 4096

class tcfg_wire_protocol
{
public:
    static tcfg_wire_protocol *instance()
    {
        static tcfg_wire_protocol _instance;
        return &_instance;
    }

    tcfg_wire_protocol(tcfg_wire_protocol const &) = delete;
    void operator=(tcfg_wire_protocol const &) = delete;

public:
    enum slip_byte : uint8_t {
        SLIP_START = 0x5a,
        SLIP_END = 0xc0,
        SLIP_ESC = 0xdb,
        SLIP_ESC_END = 0xdc,
        SLIP_ESC_ESC = 0xdd,
        SLIP_ESC_START = 0xde,
    };

    enum event : uint32_t {
        EVT_NEW_PACKET = BIT(0),
        EVT_READING_PKT = BIT(1),
        EVT_SLIP_ERROR = BIT(2),
    };

    enum pkt_type : uint8_t {
        PKT_DEVICE_INFO = 1,
        PKT_PING = 2,
        PKT_GET_CONFIG = 0x10,
        PKT_SET_CONFIG = 0x11,
        PKT_DEL_CONFIG = 0x12,
        PKT_BEGIN_FILE_WRITE = 0x20,
        PKT_FILE_CHUNK = 0x21,
        PKT_END_FILE_WRITE = 0x22,
        PKT_GET_FILE_INFO = 0x23,
        PKT_DELETE_FILE = 0x24,
        PKT_ACK = 0x80,
        PKT_CHUNK_ACK = 0x81,
        PKT_CONFIG_RESULT = 0x82,
        PKT_FILE_INFO = 0x83,
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

    struct __attribute__((packed)) nack_pkt {
        int32_t ret;
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
        uint16_t len;
        uint8_t buf[];
    };

    struct __attribute__((packed)) cfg_pkt {
        nvs_type_t type : 8;
        uint16_t val_len;
        char ns[16];
        char key[16];
        uint8_t value[];
    };

public:
    esp_err_t init(tcfg_wire_if *_wire_if);

private:
    tcfg_wire_protocol() = default;
    static void rx_task(void *_ctx);
    void handle_rx_pkt(const uint8_t *buf, size_t decoded_len);

private:
    static uint16_t get_crc16(const uint8_t *buf, size_t len, uint16_t init = 0);
    esp_err_t send_pkt(pkt_type type, const uint8_t *buf = nullptr, size_t len = 0, uint32_t timeout_ticks = portMAX_DELAY);
    esp_err_t send_ack(uint32_t timeout_ticks = portMAX_DELAY);
    esp_err_t send_nack(int32_t ret = 0, uint32_t timeout_ticks = portMAX_DELAY);
    esp_err_t send_dev_info(uint32_t timeout_ticks = portMAX_DELAY);
    esp_err_t send_chunk_ack(tcfg_wire_protocol::chunk_ack state, uint32_t aux = 0, uint32_t timeout_ticks = portMAX_DELAY);
    esp_err_t encode_and_tx(const uint8_t *header_buf, size_t header_len, const uint8_t *buf, size_t len, uint32_t timeout_ticks = portMAX_DELAY);

private:
    esp_err_t set_cfg_to_nvs(const char *ns, const char *key, nvs_type_t type, const void *value, size_t value_len);
    esp_err_t get_cfg_from_nvs(const char *ns, const char *key, nvs_type_t type);
    esp_err_t handle_begin_file_write(const char *path, size_t expect_len);

private:
    FILE *fp = nullptr;
    size_t file_expect_len = 0;
    tcfg_wire_if *wire_if = nullptr;
    EventGroupHandle_t state_evt_group = nullptr;
    TaskHandle_t rx_task_handle = nullptr;

private:
    static const constexpr char TAG[] = "tcfg_wire";
};

