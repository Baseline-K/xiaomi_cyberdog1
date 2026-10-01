#include <assert.h>
#include <stdio.h>
#include "as5047_pipeline.h"

static uint16_t response(uint16_t data)
{
    uint16_t p = data;
    p ^= p >> 8; p ^= p >> 4; p ^= p >> 2; p ^= p >> 1;
    return data | ((p & 1U) << 15);
}

int main(void)
{
    AS5047_Pipeline p;
    uint16_t value = 999U;
    uint32_t stamp = 0U;
    AS5047_PipelineReset(&p);
    assert(AS5047_PipelineCommand(&p) == 0xFFFFU);
    assert(AS5047_PipelineAccept(&p, 0xFFFFU, response(17), 100U,
        &value, &stamp) == AS5047_PIPE_WARMUP);
    assert(value == 999U); /* Never publish the startup/diagnostic response. */
    assert(AS5047_PipelineAccept(&p, 0xFFFFU, response(123), 200U,
        &value, &stamp) == AS5047_PIPE_ANGLE);
    assert(value == 123U && stamp == 100U); /* timestamp of previous command */
    /* Corrupt response: invalidate ownership and schedule ERRFL, not an angle. */
    assert(AS5047_PipelineAccept(&p, 0xFFFFU, response(123)^1U, 300U,
        &value, &stamp) == AS5047_PIPE_PARITY);
    assert(AS5047_PipelineCommand(&p) == 0x4001U);
    assert(AS5047_PipelineAccept(&p, 0x4001U, response(0x4000U), 400U,
        &value, &stamp) == AS5047_PIPE_EF);
    assert(AS5047_PipelineCommand(&p) == 0xFFFFU);
    assert(AS5047_PipelineAccept(&p, 0xFFFFU, response(0x4004U), 500U,
        &value, &stamp) == AS5047_PIPE_ERROR_FLAGS);
    assert(value == 4U);
    assert(AS5047_PipelineAccept(&p, 0xFFFFU, response(250), 600U,
        &value, &stamp) == AS5047_PIPE_ANGLE);
    assert(value == 250U && stamp == 500U);
    /* A clean EF frame must not be treated as a valid angle either. */
    assert(AS5047_PipelineAccept(&p, 0xFFFFU, response(0x4123U), 700U,
        &value, &stamp) == AS5047_PIPE_EF);
    assert(AS5047_PipelineCommand(&p) == 0x4001U);
    /* Arbitrary register access/reset loses stream ownership. */
    AS5047_PipelineReset(&p);
    assert(AS5047_PipelineAccept(&p, 0xFFFFU, response(8192), 0xFFFFFFF0U,
        &value, &stamp) == AS5047_PIPE_WARMUP);
    assert(AS5047_PipelineAccept(&p, 0xFFFFU, response(16383), 0x10U,
        &value, &stamp) == AS5047_PIPE_ANGLE);
    assert(value == 16383U && stamp == 0xFFFFFFF0U);
    assert(!AS5047_Elapsed(0x10U, 0xFFFFFFF0U, 33U));
    assert(AS5047_Elapsed(0x10U, 0xFFFFFFF0U, 32U));
    /* Exhaust all 14-bit positions, including mechanical wrap to zero. */
    for (uint32_t raw = 0U; raw < 16384U; ++raw) {
        uint32_t previous = p.pending_cycles;
        assert(AS5047_PipelineAccept(&p, 0xFFFFU, response((uint16_t)raw),
            raw + 1000U, &value, &stamp) == AS5047_PIPE_ANGLE);
        assert(value == raw && stamp == previous);
    }
    puts("AS5047 pipeline: warmup, timestamps, parity, EF, recovery, reset, wrap PASS");
    return 0;
}
