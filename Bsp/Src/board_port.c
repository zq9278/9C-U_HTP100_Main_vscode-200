#include "board_port.h"

#include "FreeRTOS.h"
#include "ads1220_driver.h"
#include "app_config.h"
#include "app_controller.h"
#include "app_log.h"
#include "charger_driver.h"
#include "eye_driver.h"
#include "main.h"
#include "pressure_tuning.h"
#include "screen_protocol.h"
#include "screen_uart.h"
#include "storage_driver.h"
#include "task.h"
#include "treatment_hw.h"
#include "ws2812_driver.h"

#define BOARD_ADS_RETRY_PERIOD_MS 2000U

static volatile bool g_button_event;
static volatile bool g_power_loss_event;
static bool g_ads_ready;
static uint32_t g_ads_last_retry_ms;

static uint32_t board_now_ms(void)
{
    return HAL_GetTick();
}

static bool board_eye_mark_consumed(void)
{
    if (!EyeDriver_MarkConsumed()) {
        return false;
    }
    StorageDriver_IncrementEyeReplacement();
    return true;
}

static bool board_pressure_zero_calibrate(void)
{
    if (!Ads1220Driver_Ready() && !Ads1220Driver_Init()) {
        LOGE("[ADS1220] Zero calibration skipped: ADC initialization unavailable");
        return false;
    }
    g_ads_ready = true;
    return Ads1220Driver_ZeroCalibrate();
}

static bool board_power_read(bool *charging, bool *full, uint16_t *soc,
                             uint16_t *millivolts)
{
    if (!ChargerDriver_Read(charging, full, soc, millivolts)) {
        return false;
    }
    /* External USB power blocks treatment and also blanks the screen. */
    ScreenUart_SetPower(!*charging || AppController_DebugMode());
    return true;
}

static void board_power_latch_off(void)
{
    LOGW("[Power] BATFET shutdown requested");
    TreatmentHw_SafeOutputsOff();
    ChargerDriver_PowerLatchOff();
}

static void board_fault_report(AppFault fault)
{
#if APP_ENABLE_SCREEN_FAULT_REPORT
    ScreenProtocol_SendU32(0x2070U, (uint32_t)fault);
    LOGE("[Screen TX] Fault code=0x%08lX", (unsigned long)fault);
#else
    (void)fault;
#endif
}

static const AppPort g_port = {
    .now_ms = board_now_ms,
    .safe_outputs_off = TreatmentHw_SafeOutputsOff,
    .heater_control = TreatmentHw_HeaterControl,
    .pressure_control_start = TreatmentHw_PressureStart,
    .pressure_control_step = TreatmentHw_PressureStep,
    .pressure_control_stop = TreatmentHw_PressureStop,
    .home_begin = TreatmentHw_HomeBegin,
    .home_poll = TreatmentHw_HomePoll,
    .home_cancel = TreatmentHw_HomeCancel,
    .pressure_zero_calibrate = board_pressure_zero_calibrate,
    .eye_read_state = EyeDriver_ReadState,
    .eye_mark_consumed = board_eye_mark_consumed,
    .power_read = board_power_read,
    .power_latch_off = board_power_latch_off,
    .led_set = Ws2812Driver_Set,
    .counter_increment = StorageDriver_IncrementCounter,
    .screen_boot_sync = StorageDriver_ScreenBootSync,
    .storage_read_u16 = StorageDriver_ReadU16,
    .storage_write_u16 = StorageDriver_WriteU16,
    .storage_erase_main = StorageDriver_EraseMain,
    .storage_erase_eye = EyeDriver_Erase,
    .screen_float = ScreenProtocol_SendFloat,
    .screen_u16 = ScreenProtocol_SendU16,
    .screen_u32 = ScreenProtocol_SendU32,
    .fault_report = board_fault_report,
    .heater_test_max = TreatmentHw_HeaterTestMax
};

bool Board_Init(void)
{
    bool charging;
    bool full;
    uint16_t soc;
    uint16_t millivolts;

    TreatmentHw_Init();
    Ads1220Driver_PreparePins();
    StorageDriver_PreparePins();
    EyeDriver_Init();
    Ws2812Driver_Init();
    if (!ChargerDriver_Init()) {
        return false;
    }
    /* Apply the USB/charging screen policy before starting the screen UART so
     * a USB cold start does not visibly flash the display for one poll cycle. */
    if (!board_power_read(&charging, &full, &soc, &millivolts)) {
        ScreenUart_SetPower(true);
    }
    /* Load validated external pressure profiles (or compiled defaults) before
     * the ADC can perform conversion-to-pressure validation. */
    StorageDriver_Init();
    PressureTuning_Init();
    g_ads_ready = Ads1220Driver_Init();
    if (!g_ads_ready) {
        LOGW("[ADS1220] Startup degraded; UI, charging and non-pressure functions remain available");
        g_ads_last_retry_ms = HAL_GetTick();
    }
    return ScreenUart_Init();
}

const AppPort *Board_AppPort(void)
{
    return &g_port;
}

void Board_Tick(void)
{
    Ws2812Driver_Tick();
    if (!g_ads_ready &&
        HAL_GetTick() - g_ads_last_retry_ms >= BOARD_ADS_RETRY_PERIOD_MS) {
        g_ads_last_retry_ms = HAL_GetTick();
        LOGW("[ADS1220] Background initialization retry");
        g_ads_ready = Ads1220Driver_RetryInit();
        if (g_ads_ready) {
            LOGI("[ADS1220] Background initialization recovered");
        }
    }
}

bool Board_ScreenWrite(const uint8_t *data, uint16_t length)
{
    return ScreenUart_Write(data, length);
}

size_t Board_ScreenRead(uint8_t *destination, size_t capacity)
{
    return ScreenUart_Read(destination, capacity);
}

bool Board_TakeButtonEvent(void)
{
    bool value;

    taskENTER_CRITICAL();
    value = g_button_event;
    g_button_event = false;
    taskEXIT_CRITICAL();
    return value;
}

bool Board_TakePowerLossEvent(void)
{
    bool value;

    taskENTER_CRITICAL();
    value = g_power_loss_event;
    g_power_loss_event = false;
    taskEXIT_CRITICAL();
    return value;
}

void Board_UartRxEventFromIsr(uint16_t length)
{
    ScreenUart_RxEventFromIsr(length);
}

void Board_UartErrorFromIsr(void)
{
    ScreenUart_ErrorFromIsr();
}

void Board_ButtonFromIsr(void)
{
    g_button_event = true;
}

void Board_PowerLossFromIsr(void)
{
    g_power_loss_event = true;
}

void Board_Ws2812FinishedFromIsr(void)
{
    Ws2812Driver_FinishedFromIsr();
}
