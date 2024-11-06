idf_component_register(
        SRCS
            "tcfg_manager.cpp" "tcfg_manager.hpp"
            "tcfg_wire_protocol.cpp" "tcfg_wire_protocol.hpp" "tcfg_wire_interface.hpp"
            "tcfg_wire_usb_cdc.cpp" "tcfg_wire_usb_cdc.hpp"
        INCLUDE_DIRS "."
        PRIV_REQUIRES "usb" "tinyusb" "spi_flash" "esp_tinyusb" "esp_partition" "esp_ringbuf" "nvs_flash" "mbedtls" "app_update"
)