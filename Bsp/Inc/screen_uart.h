#ifndef SCREEN_UART_H
#define SCREEN_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool ScreenUart_Init(void);
void ScreenUart_SetPower(bool enabled);
bool ScreenUart_Write(const uint8_t *data, uint16_t length);
size_t ScreenUart_Read(uint8_t *destination, size_t capacity);
void ScreenUart_RxEventFromIsr(uint16_t length);
void ScreenUart_ErrorFromIsr(void);

#endif
