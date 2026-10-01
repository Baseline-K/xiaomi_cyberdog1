#ifndef AS5047_PIPELINE_H
#define AS5047_PIPELINE_H

#include <stdbool.h>
#include <stdint.h>

/* Protocol state, owned by the consumer; DMA only transports complete frames. */
typedef struct {
    uint16_t pending_address;
    uint32_t pending_cycles;
    bool pending_valid;
    bool recover;
} AS5047_Pipeline;

enum { AS5047_PIPE_WARMUP = 0, AS5047_PIPE_ANGLE = 1,
       AS5047_PIPE_ERROR_FLAGS = 2, AS5047_PIPE_PARITY = -1,
       AS5047_PIPE_EF = -2 };

static inline void AS5047_PipelineReset(AS5047_Pipeline *p)
{
    p->pending_address = 0U;
    p->pending_cycles = 0U;
    p->pending_valid = false;
    p->recover = false;
}

static inline uint16_t AS5047_PipelineCommand(const AS5047_Pipeline *p)
{
    /* Precomputed read commands, including even parity. */
    return p->recover ? 0x4001U : 0xFFFFU;
}

static inline int AS5047_PipelineAccept(AS5047_Pipeline *p,
    uint16_t tx, uint16_t rx, uint32_t completed_cycles,
    uint16_t *value, uint32_t *sample_cycles)
{
    uint16_t parity = rx;
    uint16_t previous_address = p->pending_address;
    bool previous_valid = p->pending_valid;
    *sample_cycles = p->pending_cycles;
    p->pending_address = tx & 0x3FFFU;
    p->pending_cycles = completed_cycles; /* CS rising edge of THIS command. */
    p->pending_valid = true;
    p->recover = false;
    parity ^= parity >> 8;
    parity ^= parity >> 4;
    parity ^= parity >> 2;
    parity ^= parity >> 1;
    if ((parity & 1U) != 0U) {
        p->pending_valid = false;
        p->recover = true;
        return AS5047_PIPE_PARITY;
    }
    if ((rx & 0x4000U) != 0U) {
        /* ERRFL clears sticky command errors; its returned data may carry EF. */
        p->recover = p->pending_address != 1U;
        if (previous_valid && previous_address == 1U) {
            *value = rx & 0x3FFFU;
            p->recover = false;
            return AS5047_PIPE_ERROR_FLAGS;
        }
        return AS5047_PIPE_EF;
    }
    if (!previous_valid) return AS5047_PIPE_WARMUP;
    *value = rx & 0x3FFFU;
    if (previous_address == 0x3FFFU) return AS5047_PIPE_ANGLE;
    if (previous_address == 1U) return AS5047_PIPE_ERROR_FLAGS;
    return AS5047_PIPE_WARMUP;
}

/* Unsigned subtraction is intentional: DWT wraps roughly every 36 s at 120 MHz. */
static inline bool AS5047_Elapsed(uint32_t now, uint32_t then, uint32_t limit)
{
    return (uint32_t)(now - then) >= limit;
}
#endif
