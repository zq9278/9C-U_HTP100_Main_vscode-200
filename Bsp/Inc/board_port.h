#ifndef BOARD_PORT_H
#define BOARD_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_port.h"

bool Board_Init(void);
void Board_Tick(void);
const AppPort *Board_AppPort(void);

size_t Board_ScreenRead(uint8_t *destination, size_t capacity);
bool Board_ScreenWrite(const uint8_t *data, uint16_t length);
bool Board_TakeButtonEvent(void);
bool Board_TakePowerLossEvent(void);

void Board_UartRxEventFromIsr(uint16_t length);
void Board_UartErrorFromIsr(void);
void Board_ButtonFromIsr(void);
void Board_PowerLossFromIsr(void);
void Board_Ws2812FinishedFromIsr(void);

#endif
