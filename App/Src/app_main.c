#include "app_main.h"

#include "FreeRTOS.h"
#include "app_config.h"
#include "app_controller.h"
#include "app_log.h"
#include "board_port.h"
#include "eye_driver.h"
#include "main.h"
#include "product_config.h"
#include "screen_protocol.h"
#include "storage_driver.h"
#include "task.h"
#include "tuning_protocol.h"

#define SCREEN_EARLY_BOOT_WINDOW_MS 250U

static void reply_language_before_board_init(void)
{
    uint8_t serial_data[128];
    uint32_t started_ms = HAL_GetTick();

    StorageDriver_PreparePins();
    do {
        size_t received = Board_ScreenRead(serial_data, sizeof(serial_data));
        if (received != 0U &&
            ScreenProtocol_EarlyBootReply(serial_data, received)) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    } while (HAL_GetTick() - started_ms < SCREEN_EARLY_BOOT_WINDOW_MS);
}

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
    bool external_power_seen = false;
    bool previous_external_power = false;

    (void)argument;
    ScreenProtocol_Init(Board_ScreenWrite);
    reply_language_before_board_init();
    AppLog_Init();
    LOGI("%s firmware boot, version=%lu, eye_fuse=%u",
         PRODUCT_MODEL_NAME, (unsigned long)PRODUCT_VERSION_NUMBER,
         (unsigned)PRODUCT_EYE_FUSE_ENABLED);
    if (!Board_Init()) {
        LOGE("Board initialization failed");
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000U));
        }
    }
    LOGI("Board initialization complete");
    /* Reset the stream parser after the early, language-only handshake. */
    ScreenProtocol_Init(Board_ScreenWrite);
    /* The tuning protocol shares USART2 with the real screen. Its 7A A7
     * header is deliberately distinct from the screen's 5A A5 / 6A A6. */
    TuningProtocol_Init(Board_ScreenWrite);
    AppController_Init(Board_AppPort());
    /* Make the first charger check immediate instead of leaving a one-second
     * window in which PWR_SENSE can be mistaken for a battery-only shutdown. */
    last_power_ms = HAL_GetTick() - APP_BATTERY_POLL_PERIOD_MS;

    for (;;) {
        uint32_t now = HAL_GetTick();
        size_t received = Board_ScreenRead(serial_data, sizeof(serial_data));
        if (received != 0U) {
            ScreenProtocol_Feed(serial_data, received);
            TuningProtocol_Feed(serial_data, received);
        }
        if (Board_TakeButtonEvent()) {
            handle_button();
        }
        if (now - last_eye_ms >= APP_EYE_POLL_PERIOD_MS) {
            last_eye_ms = now;
            AppEyeState eye = Board_AppPort()->eye_read_state();
            if (EyeDriver_InitialStateConfirmed()) {
                AppController_SetEyeLinkHealthy(EyeDriver_LastProbeValid());
                AppController_SetEyeState(eye);
            }
        }
        if (now - last_power_ms >= APP_BATTERY_POLL_PERIOD_MS) {
            bool charging = false;
            bool full = false;
            uint16_t soc = 0U;
            uint16_t millivolts = 0U;
            last_power_ms = now;
            if (Board_AppPort()->power_read(&charging, &full, &soc, &millivolts)) {
                /* Reset only on an observed external-power falling edge.
                 * A battery-only boot (or a failed status read) must never
                 * request another reset. Disable actuators before resetting;
                 * the normal startup path will configure and home again. */
                if (external_power_seen && previous_external_power && !charging) {
                    Board_AppPort()->safe_outputs_off();
                    LOGW("[Power] USB removed: outputs off, software reset requested");
                    NVIC_SystemReset();
                }
                previous_external_power = charging;
                external_power_seen = true;
                AppController_SetPower(charging, full, soc, millivolts);
            }
        }
        if (Board_TakePowerLossEvent()) {
            AppController_NotifyPowerLoss();
        }
        AppController_Tick();
        TuningProtocol_Tick();
        Board_Tick();
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}
