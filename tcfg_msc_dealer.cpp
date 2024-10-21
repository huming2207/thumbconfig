#include <cstring>
#include <esp_mac.h>
#include <esp_rom_spiflash.h>
#include <esp_flash.h>
#include "tcfg_msc_dealer.hpp"
#include "tinyusb.h"
#include "tusb_msc_storage.h"

esp_err_t tcfg_msc_dealer::init(const char *part_name)
{
    data_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, part_name);
    if (data_part == nullptr) {
        ESP_LOGE(TAG, "Failed to find partition: %s", part_name);
        return ESP_ERR_NOT_FOUND;
    }

    wl_handle = WL_INVALID_HANDLE;
    msc_evt_group = xEventGroupCreate();
    if (msc_evt_group == nullptr) {
        ESP_LOGE(TAG, "MSC event group init fail");
        return ESP_ERR_NO_MEM;
    }

    auto ret = wl_mount(data_part, &wl_handle);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Wear level mount error: 0x%x", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Mount data partition, wl handle = %ld", wl_handle);

    ret = tinyusb_msc_storage_init_spiflash(&spiflash_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI flash init fail");
        return ret;
    }

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

    uint8_t sn_buf[16] = { 0 };
    ret = ret ?: esp_efuse_mac_get_default(sn_buf);
    ret = ret ?: esp_flash_read_unique_chip_id(esp_flash_default_chip, reinterpret_cast<uint64_t *>(sn_buf + 6));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Can't read UID: 0x%x", ret);
        return ret;
    }

    snprintf(sn_str, 32, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
             sn_buf[0], sn_buf[1], sn_buf[2], sn_buf[3], sn_buf[4], sn_buf[5], sn_buf[6], sn_buf[7],
             sn_buf[8], sn_buf[9], sn_buf[10], sn_buf[11], sn_buf[12], sn_buf[13]);

    ESP_LOGI(TAG, "Initialised with SN: %s", sn_str);

    ret = tinyusb_driver_install(&tusb_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB driver install failed: 0x%x", ret);
        return ret;
    }

    ret = mount(CONFIG_TC_MOUNT_PATH);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI flash mount fail");
        return ret;
    }

    return ret;
}

esp_err_t tcfg_msc_dealer::mount(const char *path)
{
    ESP_LOGI(TAG, "Start mount");
    auto ret = tinyusb_msc_storage_mount(path);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Mount failed: 0x%x", ret);
        return ret;
    }

    xEventGroupSetBits(msc_evt_group, MSC_MOUNTED);
    return ret;
}

esp_err_t tcfg_msc_dealer::unmount()
{
    ESP_LOGI(TAG, "Unmount & expose to USB now");
    auto ret = tinyusb_msc_storage_unmount();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Unmount/Expose failed");
        return ret;
    }

    xEventGroupClearBits(msc_evt_group, MSC_MOUNTED);
    return ret;
}

esp_err_t tcfg_msc_dealer::try_setup_part(const char *part_name)
{
    const esp_vfs_fat_mount_config_t mount_config = {
            .format_if_mount_failed = true, // Format this partition if previously not formatted
            .max_files = 4,
            .allocation_unit_size = CONFIG_WL_SECTOR_SIZE,
            .disk_status_check_enable = false,
            .use_one_fat = false,
    };

    esp_err_t ret = esp_vfs_fat_spiflash_mount_rw_wl("/tcfg_tmp", part_name, &mount_config, &wl_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "try_setup: Failed to mount (%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_vfs_fat_spiflash_unmount_rw_wl("/tcfg_tmp", wl_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "try_setup: Failed to unmount FATFS (%s)", esp_err_to_name(ret));
        return ret;
    }

    return ret;
}
