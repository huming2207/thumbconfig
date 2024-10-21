#pragma once

#include <esp_err.h>
#include "tcfg_msc_dealer.hpp"

class tcfg_manager
{
public:
    static tcfg_manager *instance()
    {
        static tcfg_manager _instance;
        return &_instance;
    }

    tcfg_manager(tcfg_manager const &) = delete;
    void operator=(tcfg_manager const &) = delete;

public:
    esp_err_t init();

private:
    static void tcfg_checker_task(void *_ctx);

private:
    tcfg_manager() = default;

private:
    tcfg_msc_dealer msc = {};
};

