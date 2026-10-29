#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#include "pressure_tuning.h"

static unsigned storage_reads;
static unsigned storage_writes;

void AppLog_Write(const char *level, const char *format, ...)
{
    (void)level;
    (void)format;
}

bool StorageDriver_ReadBytes(uint8_t address, uint8_t *data, size_t length)
{
    (void)address; (void)data; (void)length;
    storage_reads++;
    return false;
}

bool StorageDriver_WriteBytes(uint8_t address, const uint8_t *data, size_t length)
{
    (void)address; (void)data; (void)length;
    storage_writes++;
    return true;
}

int main(void)
{
    const float targets[] = {150.0f, 250.0f, 350.0f, 450.0f, 550.0f};
    PressureTuning_Init();
    assert(storage_reads == 0U);
    for (uint8_t i = 0U; i < PRESSURE_TUNING_PROFILE_COUNT; ++i) {
        PressureTuningProfile baseline;
        assert(PressureTuning_Get(i, &baseline));
        const PressureTuningProfile *active = PressureTuning_ProfileForTarget(targets[i]);
        assert(active->retract_speed == baseline.retract_speed);
        assert(active->kp == baseline.kp);
        /* Even a valid host profile cannot override the locked code table. */
        PressureTuningProfile override = baseline;
        override.kp = 10.0f;
        override.retract_speed = baseline.retract_speed == 600U ? 700U : 600U;
        assert(!PressureTuning_Set(i, &override));
        assert(PressureTuning_ProfileForTarget(targets[i])->retract_speed == baseline.retract_speed);
        assert(PressureTuning_ProfileForTarget(targets[i])->kp == baseline.kp);
        PressureTuningProfile reported;
        assert(PressureTuning_Get(i, &reported));
        assert(reported.retract_speed == baseline.retract_speed);
        assert(reported.kp == baseline.kp);
    }
    assert(!PressureTuning_Save());
    assert(storage_writes == 0U);
    return 0;
}
