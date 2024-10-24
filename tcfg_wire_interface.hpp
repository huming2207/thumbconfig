#pragma once

#include <freertos/FreeRTOS.h>
#include <cstdint>
#include <cstddef>

class tcfg_wire_if
{
public:
    virtual bool begin_read(uint8_t **data_out, size_t buf_len, size_t *len_written, uint32_t wait_ticks) = 0;
    virtual bool finalise_read(uint8_t *ret_ptr) = 0;
    virtual bool write_response(const uint8_t *data_in, size_t buf_len, uint32_t wait_ticks) = 0;
    virtual bool flush(uint32_t wait_ticks) = 0;
    virtual bool pause(bool force) = 0;
    virtual bool resume() = 0;
};