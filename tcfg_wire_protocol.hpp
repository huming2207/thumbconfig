#pragma once

#include "tcfg_wire_protocol.hpp"
#include "tcfg_wire_interface.hpp"
#include <nvs.h>
#include <nvs_flash.h>
#include <esp_ota_ops.h>

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
    enum event : uint32_t {
        EVT_NEW_PACKET = BIT(0),
        EVT_READING_PKT = BIT(1),
        EVT_SLIP_ERROR = BIT(2),
    };

    enum pkt_type : uint8_t {
        PKT_GET_DEVICE_INFO = 1,
        PKT_PING = 2,
        PKT_GET_UPTIME = 3,
        PKT_REBOOT = 4,
        PKT_REBOOT_BOOTLOADER = 5,
        PKT_GET_CONFIG = 0x10,
        PKT_SET_CONFIG = 0x11,
        PKT_DEL_CONFIG = 0x12,
        PKT_NUKE_CONFIG = 0x13,
        PKT_BEGIN_FILE_WRITE = 0x20,
        PKT_FILE_CHUNK = 0x21,
        PKT_GET_FILE_INFO = 0x22,
        PKT_DELETE_FILE = 0x23,
        PKT_BEGIN_OTA = 0x30,
        PKT_OTA_CHUNK = 0x31,
        PKT_OTA_COMMIT = 0x32,
        PKT_BIN_RPC_REQUEST = 0x70,
        PKT_ACK = 0x80,
        PKT_CHUNK_ACK = 0x81,
        PKT_CONFIG_RESULT = 0x82,
        PKT_FILE_INFO = 0x83,
        PKT_UPTIME = 0x84,
        PKT_DEV_INFO = 0x85,
        PKT_BIN_RPC_REPLY = 0x86,
        PKT_JSON_RPC_REPLY = 0x87,
        PKT_NACK = 0xff,
    };

    enum chunk_state : uint8_t {
        CHUNK_XFER_DONE = 0,
        CHUNK_XFER_NEXT = 1,
        CHUNK_ERR_CRC32_FAIL = 2,
        CHUNK_ERR_INTERNAL = 3,
        CHUNK_ERR_ABORT_REQUESTED = 4,
        CHUNK_ERR_NAME_TOO_LONG = 5,
    };

    struct __attribute__((packed)) chunk_ack_pkt {
        chunk_state state;

        // Can be:
        // 1. Next chunk offset (when state == 1)
        // 2. Expected CRC32 (when state == 2)
        // 3. Max length allowed (when state == 3)
        // 4. Just 0 (when state == anything else?)
        uint32_t aux_info;
    };

    struct __attribute__((packed)) header {
        pkt_type type;
        uint16_t crc;
        uint16_t len;
    };

    struct __attribute__((packed)) nack_pkt {
        int32_t ret;
    };

    struct __attribute__((packed)) uptime_req_pkt {
        uint64_t realtime_ms;
    };

    struct __attribute__((packed)) uptime_pkt {
        esp_reset_reason_t last_rst_reason : 8;
        uint64_t uptime;
    };


    struct __attribute__((packed)) device_info_pkt {
        uint8_t mac_addr[6];
        uint8_t flash_id[8];
        char sdk_ver[16];
        char comp_time[16];
        char comp_date[16];
        char model_name[32];
        char fw_ver[32];
        uint8_t fw_hash[32];
    };

    struct __attribute__((packed)) path_pkt {
        uint32_t len; // 4
        char path[UINT8_MAX];
    }; // 8 bytes

    struct __attribute__((packed)) chunk_pkt {
        uint8_t buf[];
    };

    struct __attribute__((packed)) cfg_pkt {
        nvs_type_t type : 8;
        uint16_t val_len;
        char ns[16];
        char key[16];
        uint8_t value[];
    };

    struct __attribute__((packed)) del_cfg_pkt {
        char ns[16];
        char key[16];
    };

    struct __attribute__((packed)) file_info_pkt {
        uint32_t size;
        uint8_t hash[32];
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
    esp_err_t send_chunk_ack(tcfg_wire_protocol::chunk_state state, uint32_t aux = 0, uint32_t timeout_ticks = portMAX_DELAY);
    esp_err_t encode_and_tx(const uint8_t *header_buf, size_t header_len, const uint8_t *buf, size_t len, uint32_t timeout_ticks = portMAX_DELAY);

private:
    esp_err_t set_cfg_to_nvs(const char *ns, const char *key, nvs_type_t type, const void *value, size_t value_len);
    esp_err_t get_cfg_from_nvs(const char *ns, const char *key, nvs_type_t type);
    esp_err_t delete_cfg(const char *ns, const char *key);
    esp_err_t nuke_cfg(const char *ns);
    esp_err_t handle_begin_file_write(const char *path, size_t expect_len);
    esp_err_t handle_file_chunk(const uint8_t *buf, uint16_t len);
    esp_err_t handle_file_delete(const char *path);
    esp_err_t handle_get_file_info(const char *path);
    esp_err_t handle_ota_begin();
    esp_err_t handle_ota_chunk(const uint8_t *buf, uint16_t len);
    esp_err_t handle_ota_commit();
    esp_err_t handle_uptime(uint64_t realtime_ms);

private:
    FILE *fp = nullptr;
    size_t file_expect_len = 0;
    tcfg_wire_if *wire_if = nullptr;
    EventGroupHandle_t state_evt_group = nullptr;
    TaskHandle_t rx_task_handle = nullptr;
    esp_ota_handle_t ota_handle = 0;
    uint32_t curr_ota_chunk_offset = 0;
    const esp_partition_t *curr_ota_part = nullptr;

private:
    static const constexpr char TAG[] = "tcfg_wire";
};

