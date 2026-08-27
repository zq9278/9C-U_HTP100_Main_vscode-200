#ifndef TMC5130_DRIVER_H
#define TMC5130_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32g0xx_hal.h"

void Tmc5130Driver_Init(void);
void Tmc5130Driver_Enable(bool enable);
HAL_StatusTypeDef Tmc5130Driver_Write(uint8_t address, uint32_t value);
HAL_StatusTypeDef Tmc5130Driver_Read(uint8_t address, uint32_t *value);
bool Tmc5130Driver_Configure(void);
bool Tmc5130Driver_SetSpeed(int32_t speed);
void Tmc5130Driver_Stop(void);

#endif
