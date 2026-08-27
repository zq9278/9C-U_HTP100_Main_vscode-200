#ifndef APP_CONTROLLER_H
#define APP_CONTROLLER_H

#include "app_port.h"

void AppController_Init(const AppPort *port);
void AppController_Tick(void);

bool AppController_Prepare(AppMode mode, float pressure_mmhg);
bool AppController_Start(void);
void AppController_Stop(AppStopReason reason);
void AppController_RaiseFault(AppFault fault);
void AppController_SetEyeState(AppEyeState eye);
void AppController_SetPower(bool charging, bool full, uint16_t soc, uint16_t millivolts);
void AppController_NotifyPowerLoss(void);
void AppController_SetTemperature(float temperature_c);
void AppController_SetPressure(float pressure_mmhg);
void AppController_SetRuntime(uint16_t minutes);
void AppController_SetDebugMode(bool enabled);
bool AppController_DebugMode(void);
void AppController_ScreenBoot(void);
uint16_t AppController_StorageRead(uint8_t address, uint16_t default_value);
bool AppController_StorageWrite(uint8_t address, uint16_t value);
void AppController_StorageEraseMain(void);
void AppController_StorageEraseEye(void);

const AppSnapshot *AppController_Status(void);

#endif
