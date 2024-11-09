#pragma once

#include <freertos/FreeRTOS.h>
#include <cstdint>
#include <cstddef>

class tcfg_wire_if
{
public:
    virtual bool begin_read(uint8_t **data_out, size_t *len_written, uint32_t wait_ticks) = 0;
    virtual bool finalise_read(uint8_t *ret_ptr) = 0;
    virtual bool write_response(const uint8_t *header_out, size_t header_len, const uint8_t *payload_out, size_t payload_len, uint32_t wait_ticks) = 0;
    virtual bool flush(uint32_t wait_ticks) = 0;
    virtual bool ditch_read() = 0;
    virtual bool pause(bool force) = 0;
    virtual bool resume() = 0;
    virtual size_t max_packet_size() = 0;
};