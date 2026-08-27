#ifndef TUNING_PROTOCOL_H
#define TUNING_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef bool (*TuningWriteFn)(const uint8_t *data, uint16_t length);

void TuningProtocol_Init(TuningWriteFn write_fn);
void TuningProtocol_Feed(const uint8_t *data, size_t length);
void TuningProtocol_Tick(void);

#endif
