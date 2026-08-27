#ifndef ADS1220_DRIVER_H
#define ADS1220_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

void Ads1220Driver_PreparePins(void);
bool Ads1220Driver_Init(void);
bool Ads1220Driver_RetryInit(void);
bool Ads1220Driver_Ready(void);
bool Ads1220Driver_ReadRaw(int32_t *raw);
bool Ads1220Driver_ZeroCalibrate(void);
bool Ads1220Driver_ZeroValid(void);
int32_t Ads1220Driver_ZeroRaw(void);
float Ads1220Driver_PressureFromRaw(int32_t raw, float target_mmhg);

#endif
