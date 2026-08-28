#ifndef EYE_DRIVER_H
#define EYE_DRIVER_H

#include <stdbool.h>

#include "app_types.h"

void EyeDriver_Init(void);
bool EyeDriver_ReadTemperature(float *temperature_c);
bool EyeDriver_ReadTemperatureTelemetry(float *temperature_c);
AppEyeState EyeDriver_ReadState(void);
bool EyeDriver_InitialStateConfirmed(void);
bool EyeDriver_MarkConsumed(void);
void EyeDriver_Erase(void);

#endif
