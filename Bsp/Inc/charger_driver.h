#ifndef CHARGER_DRIVER_H
#define CHARGER_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

bool ChargerDriver_Init(void);
bool ChargerDriver_Read(bool *charging, bool *full, uint16_t *soc,
                        uint16_t *millivolts);
void ChargerDriver_PowerLatchOff(void);

#endif
