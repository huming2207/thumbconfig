#include "tcfg_manager.hpp"

esp_err_t tcfg_manager::init()
{
    return msc.init(CONFIG_TC_PART_NAME);
}
