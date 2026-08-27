#ifndef WS2812_DRIVER_H
#define WS2812_DRIVER_H

#include <stdbool.h>

#include "app_port.h"

void Ws2812Driver_Init(void);
void Ws2812Driver_Set(AppLedState state, bool blink);
void Ws2812Driver_Tick(void);
void Ws2812Driver_FinishedFromIsr(void);

#endif
