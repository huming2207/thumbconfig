#pragma once

#include <esp_err.h>
#include <esp_partition.h>
#include <wear_levelling.h>
#include "tusb_msc_storage.h"

class tcfg_msc_dealer
{
public:
    enum state : uint32_t {
        MSC_MOUNTED = BIT(0),
    };

public:
    tcfg_msc_dealer() = default;
    esp_err_t init(const char *part_name);
    esp_err_t mount(const char *path);
    esp_err_t unmount();

private:
    esp_err_t try_setup_part(const char *part_name);

private:
    wl_handle_t wl_handle = 0;
    const esp_partition_t *data_part = nullptr;
    EventGroupHandle_t msc_evt_group = nullptr;
    tinyusb_msc_spiflash_config_t spiflash_cfg = {
            .wl_handle = wl_handle,
            .callback_mount_changed = nullptr,
            .callback_premount_changed = nullptr,
            .mount_config = {
                    .format_if_mount_failed = true,
                    .max_files = 5,
                    .allocation_unit_size = 0,
                    .disk_status_check_enable = false,
                    .use_one_fat = false,
            },
    };

    char sn_str[32] = {};

private:
    static const constexpr char TAG[] = "tcfg_msc";
    static const constexpr char USB_DESC_MANUFACTURER[] = "Jackson M Hu";
    static const constexpr char USB_DESC_PRODUCT[] = "Soul Injector";
    static const constexpr char USB_DESC_CDC_NAME[] = "Soul Injector Programmer";
};
