idf_component_register(
        SRCS
            "tcfg_manager.cpp" "tcfg_manager.hpp"
            "tcfg_msc_dealer.cpp" "tcfg_msc_dealer.hpp"
        INCLUDE_DIRS "."
        PRIV_REQUIRES "usb" "tinyusb" "spi_flash" "esp_tinyusb" "esp_partition"
)