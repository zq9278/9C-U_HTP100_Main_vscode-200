#ifndef SCREEN_PROTOCOL_H
#define SCREEN_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef bool (*ScreenWriteFn)(const uint8_t *data, uint16_t length);

void ScreenProtocol_Init(ScreenWriteFn write_fn);
void ScreenProtocol_Feed(const uint8_t *data, size_t length);
void ScreenProtocol_SendFloat(uint16_t command, float value);
void ScreenProtocol_SendU16(uint16_t command, uint16_t value);
void ScreenProtocol_SendU32(uint16_t command, uint32_t value);
uint16_t ScreenProtocol_Crc16(const uint8_t *data, size_t length);

#endif
