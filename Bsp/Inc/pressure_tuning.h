#ifndef PRESSURE_TUNING_H
#define PRESSURE_TUNING_H

#include <stdbool.h>
#include <stdint.h>

#define PRESSURE_TUNING_PROFILE_COUNT 5U

typedef struct {
    float sensitivity_mv_v;
    float offset_mmhg;
    float fast_speed;
    float approach_speed;
    float retract_speed;
    float approach_threshold;
    float hold_threshold;
    uint32_t hold_ms;
    uint32_t retract_ms;
    float kp;
    float ki;
} PressureTuningProfile;

void PressureTuning_Init(void);
const PressureTuningProfile *PressureTuning_ProfileForTarget(float target_mmhg);
bool PressureTuning_Get(uint8_t index, PressureTuningProfile *profile);
bool PressureTuning_Set(uint8_t index, const PressureTuningProfile *profile);
void PressureTuning_ResetDefaults(void);
bool PressureTuning_Save(void);

#endif
