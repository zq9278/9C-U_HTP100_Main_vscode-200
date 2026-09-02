#include <assert.h>
#include <string.h>

#include "app_config.h"
#include "app_controller.h"
#include "product_config.h"

void AppLog_Write(const char *level, const char *format, ...)
{
    (void)level;
    (void)format;
}

static uint32_t fake_now;
static AppAsyncResult fake_home_result;
static AppEyeState fake_eye;
static bool fake_heat_ok;
static unsigned home_begin_count;
static unsigned home_poll_count;
static unsigned home_cancel_count;
static unsigned eye_consume_count;
static unsigned treatment_count;
static unsigned power_off_count;
static unsigned pressure_start_count;
static unsigned eye_screen_count;
static unsigned new_eye_screen_count;
static unsigned temperature_screen_count;
static unsigned fault_status_count;
static unsigned immediate_fault_count;
static uint32_t last_fault_status_value;
static uint16_t last_screen_command;
static float last_eye_screen_value;
static float last_heat_target;
static float last_temperature_screen_value;
static AppLedState last_led;

static uint32_t now_ms(void) { return fake_now; }
static void outputs_off(void) { }
static bool heat(float target, float *measured)
{
    last_heat_target = target;
    *measured = target;
    return fake_heat_ok;
}
static bool pressure_start(float target) { (void)target; pressure_start_count++; return true; }
static bool pressure_step(float target, float *measured) { *measured = target; return true; }
static void pressure_stop(void) { }
static bool home_begin(void) { home_begin_count++; return true; }
static AppAsyncResult home_poll(void) { home_poll_count++; return fake_home_result; }
static void home_cancel(void) { home_cancel_count++; }
static bool zero_calibrate(void) { return true; }
static AppEyeState eye_read(void) { return fake_eye; }
static bool eye_consume(void) { eye_consume_count++; return true; }
static bool power_read(bool *charging, bool *full, uint16_t *soc, uint16_t *mv)
{ (void)charging; (void)full; (void)soc; (void)mv; return true; }
static void power_off(void) { power_off_count++; }
static void led_set(AppLedState state, bool blink) { last_led = state; (void)blink; }
static void count(AppMode mode) { if (mode != APP_MODE_NONE) treatment_count++; }
static void screen_boot_sync(void) { }
static uint16_t storage_read(uint8_t address, uint16_t fallback) { (void)address; return fallback; }
static bool storage_write(uint8_t address, uint16_t value) { (void)address; (void)value; return true; }
static void storage_erase(void) { }
static void screen_float(uint16_t command, float value)
{
    if (command == 0x2055U) {
        eye_screen_count++;
        last_eye_screen_value = value;
        last_screen_command = command;
    } else if (command == 0x2057U) {
        new_eye_screen_count++;
    } else if (command == 0x2037U || command == 0x2041U) {
        temperature_screen_count++;
        last_temperature_screen_value = value;
    }
}
static void screen_u16(uint16_t command, uint16_t value) { (void)command; (void)value; }
static void screen_u32(uint16_t command, uint32_t value)
{
    if (command == 0x2F01U) {
        fault_status_count++;
        last_fault_status_value = value;
        last_screen_command = command;
    }
}
static void fault(AppFault value) { (void)value; immediate_fault_count++; }

static const AppPort port = {
    now_ms, outputs_off, heat, pressure_start, pressure_step, pressure_stop,
    home_begin, home_poll, home_cancel, zero_calibrate, eye_read, eye_consume, power_read,
    power_off, led_set, count, screen_boot_sync, storage_read, storage_write,
    storage_erase, storage_erase, screen_float, screen_u16, screen_u32, fault
};

static void reset_fixture(void)
{
    fake_now = 0U;
    fake_home_result = APP_ASYNC_BUSY;
    fake_eye = APP_EYE_NEW;
    fake_heat_ok = true;
    home_begin_count = home_poll_count = home_cancel_count = eye_consume_count = 0U;
    treatment_count = power_off_count = pressure_start_count = 0U;
    eye_screen_count = 0U;
    new_eye_screen_count = 0U;
    temperature_screen_count = 0U;
    fault_status_count = 0U;
    immediate_fault_count = 0U;
    last_fault_status_value = UINT32_MAX;
    last_screen_command = 0U;
    last_eye_screen_value = -1.0f;
    last_heat_target = -1.0f;
    last_temperature_screen_value = -1.0f;
    last_led = APP_LED_IDLE;
    AppController_Init(&port);
    fake_home_result = APP_ASYNC_OK;
    AppController_Tick();
    AppController_SetEyeState(APP_EYE_NEW);
    assert(AppController_Status()->state == APP_STATE_IDLE);
}

static void test_eye_is_consumed_only_at_formal_start(void)
{
    reset_fixture();
    assert(new_eye_screen_count == 1U);
    assert(AppController_Prepare(APP_MODE_HEAT, 0.0f));
    assert(eye_consume_count == 0U);
    assert(AppController_Start());
    assert(new_eye_screen_count == 1U);
#if PRODUCT_EYE_FUSE_ENABLED
    assert(eye_consume_count == 1U);
#else
    assert(eye_consume_count == 0U);
#endif
    AppController_Stop(APP_STOP_NATURAL);
    assert(treatment_count == 1U);
    assert(AppController_Status()->eye == APP_EYE_IN_USE);

    /* EEPROM polling now sees the marker, but the uninterrupted insertion stays usable. */
    AppController_SetEyeState(APP_EYE_CONSUMED);
    assert(AppController_Status()->eye == APP_EYE_IN_USE);
    fake_home_result = APP_ASYNC_OK;
    AppController_Tick();
    assert(AppController_Prepare(APP_MODE_HEAT, 0.0f));
    assert(AppController_Start());
#if PRODUCT_EYE_FUSE_ENABLED
    assert(eye_consume_count == 1U);
#else
    assert(eye_consume_count == 0U);
#endif
}

static void test_reinserted_consumed_eye_is_rejected(void)
{
    reset_fixture();
    assert(AppController_Prepare(APP_MODE_HEAT, 0.0f));
    assert(AppController_Start());
    AppController_Stop(APP_STOP_USER);
    fake_home_result = APP_ASYNC_OK;
    AppController_Tick();

    AppController_SetEyeState(APP_EYE_ABSENT);
    AppController_SetEyeState(APP_EYE_CONSUMED);
    assert(AppController_Status()->eye == APP_EYE_CONSUMED);
    assert(eye_screen_count >= 2U);
    assert(last_eye_screen_value == 0.0f);
    assert(!AppController_Prepare(APP_MODE_HEAT, 0.0f));
}

static void test_eye_offline_state_is_repeated_to_screen(void)
{
    unsigned before;

    reset_fixture();
    AppController_SetEyeState(APP_EYE_ABSENT);
    before = eye_screen_count;
    AppController_SetEyeState(APP_EYE_ABSENT);
    AppController_SetEyeState(APP_EYE_ABSENT);
    assert(eye_screen_count == before + 2U);
    assert(last_eye_screen_value == 0.0f);
}

static void test_eye_offline_clears_and_suppresses_nonzero_fault_frames(void)
{
    reset_fixture();
    AppController_RaiseFault(APP_FAULT_TMP112_COMM);
    assert(immediate_fault_count == 1U);

    fault_status_count = 0U;
    last_screen_command = 0U;
    AppController_SetEyeState(APP_EYE_ABSENT);
    assert(fault_status_count == 1U);
    assert(last_fault_status_value == 0U);
    /* The clear frame must precede the eye-offline frame. */
    assert(last_screen_command == 0x2055U);

    AppController_RaiseFault(APP_FAULT_TMP112_COMM);
    AppController_SetPower(false, false, 50U, 3800U);
    assert(immediate_fault_count == 1U);
    assert(fault_status_count == 1U);

    AppController_SetEyeState(APP_EYE_SERVICE);
    AppController_SetPower(false, false, 50U, 3800U);
    assert(fault_status_count == 2U);
    assert(last_fault_status_value == APP_FAULT_TMP112_COMM);
}

static void test_temperature_failure_then_eye_removal_does_not_raise_0101(void)
{
    reset_fixture();
    assert(AppController_Prepare(APP_MODE_HEAT, 0.0f));
    fake_heat_ok = false;
    fake_now = APP_HEAT_CONTROL_PERIOD_MS;
    AppController_Tick();
    assert(AppController_Status()->fault == APP_FAULT_NONE);
    assert(immediate_fault_count == 0U);

    AppController_SetEyeState(APP_EYE_ABSENT);
    fake_now += APP_TEMP_COMM_FAULT_CONFIRM_MS;
    AppController_Tick();
    assert(AppController_Status()->fault == APP_FAULT_NONE);
    assert(immediate_fault_count == 0U);
    assert(AppController_Status()->stop_reason == APP_STOP_EYE_REMOVED);
}

static void test_temperature_failure_raises_0101_only_while_eye_online(void)
{
    reset_fixture();
    assert(AppController_Prepare(APP_MODE_HEAT, 0.0f));
    fake_heat_ok = false;
    fake_now = APP_HEAT_CONTROL_PERIOD_MS;
    AppController_Tick();
    fake_now += APP_TEMP_COMM_FAULT_CONFIRM_MS;
    AppController_Tick();
    assert(AppController_Status()->eye == APP_EYE_NEW);
    assert(AppController_Status()->fault == APP_FAULT_TMP112_COMM);
    assert(immediate_fault_count == 1U);
}

static void test_only_natural_finish_counts(void)
{
    reset_fixture();
    assert(AppController_Prepare(APP_MODE_PRESSURE, 350.0f));
    assert(AppController_Start());
    AppController_Stop(APP_STOP_USER);
    assert(treatment_count == 0U);
    assert(home_begin_count == 2U);
}

static void test_temperature_compensation_controls_and_offsets_lcd(void)
{
    reset_fixture();
    assert(AppController_Prepare(APP_MODE_AUTO, 350.0f));
    assert(!AppController_Status()->temperature_valid);
    fake_now = 300U;
    AppController_Tick();
    assert(AppController_Status()->temperature_valid);
    assert(last_heat_target ==
           APP_PREHEAT_TARGET_C + PRODUCT_TEMPERATURE_CONTROL_COMPENSATION_C);
    assert(AppController_Status()->measured_temperature_c == last_heat_target);
    assert(temperature_screen_count == 1U);
    assert(last_temperature_screen_value == APP_PREHEAT_TARGET_C);

    assert(AppController_Start());
    fake_now = 450U;
    AppController_Tick();
    assert(AppController_Status()->settings.temperature_c ==
           APP_DEFAULT_TREATMENT_TEMP_C);
    assert(last_heat_target == APP_DEFAULT_TREATMENT_TEMP_C +
                               PRODUCT_TEMPERATURE_CONTROL_COMPENSATION_C);
    AppController_Stop(APP_STOP_USER);
    assert(!AppController_Status()->temperature_valid);
}

static void test_charging_blocks_and_interrupts_treatment(void)
{
    reset_fixture();
    AppController_SetPower(true, false, 50U, 3800U);
    assert(!AppController_Prepare(APP_MODE_AUTO, 350.0f));

    reset_fixture();
    assert(AppController_Prepare(APP_MODE_AUTO, 350.0f));
    assert(AppController_Start());
    AppController_SetPower(true, false, 50U, 3800U);
    assert(AppController_Status()->state == APP_STATE_HOMING);
    assert(AppController_Status()->stop_reason == APP_STOP_CHARGING);
}

static void test_charge_done_uses_full_led(void)
{
    reset_fixture();
    AppController_SetPower(true, false, 50U, 3800U);
    assert(last_led == APP_LED_CHARGING);
    AppController_SetPower(true, true, 100U, 4200U);
    assert(AppController_Status()->charging);
    assert(AppController_Status()->charge_full);
    assert(last_led == APP_LED_FULL);
}

static void test_low_voltage_homes_then_cuts_power(void)
{
    reset_fixture();
    for (unsigned i = 0U; i < 5U; ++i) {
        AppController_SetPower(false, false, 1U, 3150U);
    }
    assert(AppController_Status()->state == APP_STATE_HOMING);
    fake_home_result = APP_ASYNC_OK;
    AppController_Tick();
    assert(power_off_count == 1U);
    assert(AppController_Status()->state == APP_STATE_SHUTDOWN);
}

static void test_fault_and_power_loss_always_home(void)
{
    reset_fixture();
    AppController_RaiseFault(APP_FAULT_PRESSURE_COMM);
    assert(AppController_Status()->state == APP_STATE_HOMING);
    assert(AppController_Status()->stop_reason == APP_STOP_FAULT);

    reset_fixture();
    AppController_NotifyPowerLoss();
    assert(AppController_Status()->state == APP_STATE_HOMING);
    assert(AppController_Status()->stop_reason == APP_STOP_POWER_LOSS);
}

static void test_external_power_prevents_batfet_shutdown(void)
{
    reset_fixture();
    AppController_SetPower(true, false, 50U, 3800U);
    AppController_NotifyPowerLoss();
    assert(AppController_Status()->state == APP_STATE_IDLE);
    assert(power_off_count == 0U);

    /* Also cover the startup race where the edge is handled immediately
     * before the first successful external-power sample. */
    reset_fixture();
    AppController_NotifyPowerLoss();
    assert(AppController_Status()->state == APP_STATE_HOMING);
    AppController_SetPower(true, false, 50U, 3800U);
    fake_home_result = APP_ASYNC_OK;
    AppController_Tick();
    assert(AppController_Status()->state == APP_STATE_IDLE);
    assert(power_off_count == 0U);
}

static void test_normal_stop_can_quick_resume_with_previous_zero(void)
{
    reset_fixture();
    assert(AppController_Prepare(APP_MODE_PRESSURE, 350.0f));
    assert(AppController_Start());
    AppController_Stop(APP_STOP_USER);
    assert(AppController_Status()->state == APP_STATE_HOMING);
    assert(!AppController_Status()->home_valid);
    assert(!AppController_Status()->pressure_zero_valid);

    assert(AppController_Prepare(APP_MODE_PRESSURE, 350.0f));
    assert(home_cancel_count == 1U);
    assert(AppController_Status()->state == APP_STATE_READY);
    assert(!AppController_Status()->home_valid);
    assert(AppController_Status()->pressure_zero_valid);
    assert(AppController_Start());
}

static void test_fault_stop_cannot_quick_resume(void)
{
    reset_fixture();
    assert(AppController_Prepare(APP_MODE_PRESSURE, 350.0f));
    assert(AppController_Start());
    AppController_RaiseFault(APP_FAULT_PRESSURE_COMM);
    assert(AppController_Status()->state == APP_STATE_HOMING);
    assert(!AppController_Prepare(APP_MODE_PRESSURE, 350.0f));
    assert(home_cancel_count == 0U);
}

int main(void)
{
    test_eye_is_consumed_only_at_formal_start();
    test_reinserted_consumed_eye_is_rejected();
    test_eye_offline_state_is_repeated_to_screen();
    test_eye_offline_clears_and_suppresses_nonzero_fault_frames();
    test_temperature_failure_then_eye_removal_does_not_raise_0101();
    test_temperature_failure_raises_0101_only_while_eye_online();
    test_only_natural_finish_counts();
    test_temperature_compensation_controls_and_offsets_lcd();
    test_charging_blocks_and_interrupts_treatment();
    test_charge_done_uses_full_led();
    test_low_voltage_homes_then_cuts_power();
    test_fault_and_power_loss_always_home();
    test_external_power_prevents_batfet_shutdown();
    test_normal_stop_can_quick_resume_with_previous_zero();
    test_fault_stop_cannot_quick_resume();
    return 0;
}
