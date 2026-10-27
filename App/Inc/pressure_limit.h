#ifndef PRESSURE_LIMIT_H
#define PRESSURE_LIMIT_H

#include <stdbool.h>
#include <stdint.h>
#include "app_config.h"

typedef struct {
    bool timing;
    uint32_t started_ms;
} PressureLimit;

static inline void PressureLimit_Reset(PressureLimit *limit)
{
    limit->timing = false;
    limit->started_ms = 0U;
}

/* Call on each fresh, valid sample; explicitly reset on acquisition failure.
 * Confirmation starts at the first high sample, not one sample period earlier. */
static inline bool PressureLimit_Update(PressureLimit *limit, float pressure,
                                        uint32_t sampled_ms)
{
    if (!(pressure >= APP_MAX_SAFE_PRESSURE_MMHG)) {
        PressureLimit_Reset(limit);
        return false;
    }
    if (!limit->timing) {
        limit->timing = true;
        limit->started_ms = sampled_ms;
        return false;
    }
    return sampled_ms - limit->started_ms >= APP_OVER_PRESSURE_CONFIRM_MS;
}

#endif
