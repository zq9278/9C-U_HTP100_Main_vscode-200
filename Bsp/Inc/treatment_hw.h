#ifndef TREATMENT_HW_H
#define TREATMENT_HW_H

#include <stdbool.h>

#include "app_port.h"

void TreatmentHw_Init(void);
void TreatmentHw_SafeOutputsOff(void);
bool TreatmentHw_HeaterControl(float target_c, float *measured_c);
bool TreatmentHw_HeaterTestMax(float *measured_c);
bool TreatmentHw_SetHeatPid(float kp, float ki, float kd);
void TreatmentHw_GetHeatPid(float *kp, float *ki, float *kd);
void TreatmentHw_HeatTelemetry(float *power_percent, float *integral_output);
bool TreatmentHw_PressureStart(float target_mmhg);
bool TreatmentHw_PressureStep(float target_mmhg, float *measured_mmhg);
void TreatmentHw_PressureStop(void);
bool TreatmentHw_PressureTelemetry(int32_t *raw, float *pressure_mmhg,
                                   uint8_t *stage, bool *active,
                                   int32_t *speed_command,
                                   float *control_error);
bool TreatmentHw_HomeBegin(void);
AppAsyncResult TreatmentHw_HomePoll(void);
void TreatmentHw_HomeCancel(void);

#endif
