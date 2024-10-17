#pragma once

#include <esp_err.h>

class thumbcfg_msc_dealer
{
public:
    static thumbcfg_msc_dealer *instance()
    {
        static thumbcfg_msc_dealer _instance;
        return &_instance;
    }

    thumbcfg_msc_dealer(thumbcfg_msc_dealer const &) = delete;
    void operator=(thumbcfg_msc_dealer const &) = delete;

public:
    esp_err_t init();


private:
    thumbcfg_msc_dealer() = default;
};

