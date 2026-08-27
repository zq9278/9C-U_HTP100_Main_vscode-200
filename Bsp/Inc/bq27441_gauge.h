#ifndef BQ27441_GAUGE_H
#define BQ27441_GAUGE_H

#include <stdbool.h>
#include <stdint.h>

bool Bq27441Gauge_Init(void);
bool Bq27441Gauge_Read(uint16_t *soc, uint16_t *millivolts);

#endif
