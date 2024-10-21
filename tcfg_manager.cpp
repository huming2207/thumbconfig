#include "tcfg_manager.hpp"

esp_err_t tcfg_manager::init()
{
    auto ret = msc.init(CONFIG_TC_PART_NAME);
    if (ret != ESP_OK) {
        return ret;
    }

    return ret;
}

void tcfg_manager::tcfg_checker_task(void *_ctx)
{

}
