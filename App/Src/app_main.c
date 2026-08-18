#include "app_main.h"

#include "FreeRTOS.h"
#include "app_config.h"
#include "app_controller.h"
#include "board_port.h"
#include "main.h"
#include "screen_protocol.h"
#include "task.h"

static void handle_button(void)
{
    const AppSnapshot *status = AppController_Status();

    if (status->state == APP_STATE_PREHEAT || status->state == APP_STATE_READY) {
        (void)AppController_Start();
    } else if (status->state == APP_STATE_RUNNING) {
        AppController_Stop(APP_STOP_USER);
    }
}

void AppMain_Task(void *argument)
{
    uint8_t serial_data[128];
    uint32_t last_eye_ms = 0U;
    uint32_t last_power_ms = 0U;

    (void)argument;
    if (!Board_Init()) {
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000U));
        }
    }
    ScreenProtocol_Init(Board_ScreenWrite);
    AppController_Init(Board_AppPort());

    for (;;) {
        uint32_t now = HAL_GetTick();
        size_t received = Board_ScreenRead(serial_data, sizeof(serial_data));
        if (received != 0U) {
            ScreenProtocol_Feed(serial_data, received);
        }
        if (Board_TakeButtonEvent()) {
            handle_button();
        }
        if (Board_TakePowerLossEvent()) {
            AppController_NotifyPowerLoss();
        }
        if (now - last_eye_ms >= APP_EYE_POLL_PERIOD_MS) {
            last_eye_ms = now;
            AppController_SetEyeState(Board_AppPort()->eye_read_state());
        }
        if (now - last_power_ms >= APP_BATTERY_POLL_PERIOD_MS) {
            bool charging = false;
            bool full = false;
            uint16_t soc = 0U;
            uint16_t millivolts = 0U;
            last_power_ms = now;
            if (Board_AppPort()->power_read(&charging, &full, &soc, &millivolts)) {
                (void)full;
                AppController_SetPower(charging, soc, millivolts);
            }
        }
        AppController_Tick();
        Board_Tick();
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}
