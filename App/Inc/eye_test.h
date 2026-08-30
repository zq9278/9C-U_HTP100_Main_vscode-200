#ifndef EYE_TEST_H
#define EYE_TEST_H

#include "app_port.h"

void EyeTest_Init(const AppPort *port);
void EyeTest_SetEyeState(AppEyeState eye);
void EyeTest_Tick(void);
void EyeTest_Abort(void);

#endif
