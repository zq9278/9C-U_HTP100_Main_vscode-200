#include <assert.h>
#include <string.h>

#include "app_controller.h"

static uint32_t fake_now;
static AppAsyncResult fake_home_result;
static AppEyeState fake_eye;
static unsigned home_begin_count;
static unsigned home_poll_count;
static unsigned eye_consume_count;
static unsigned treatment_count;
static unsigned power_off_count;
static unsigned pressure_start_count;

static uint32_t now_ms(void) { return fake_now; }
static void outputs_off(void) { }
static bool heat(float target, float *measured) { *measured = target; return true; }
static bool pressure_start(float target) { (void)target; pressure_start_count++; return true; }
static bool pressure_step(float target, float *measured) { *measured = target; return true; }
static void pressure_stop(void) { }
static bool home_begin(void) { home_begin_count++; return true; }
static AppAsyncResult home_poll(void) { home_poll_count++; return fake_home_result; }
static bool zero_calibrate(void) { return true; }
static AppEyeState eye_read(void) { return fake_eye; }
static bool eye_consume(void) { eye_consume_count++; return true; }
static bool power_read(bool *charging, bool *full, uint16_t *soc, uint16_t *mv)
{ (void)charging; (void)full; (void)soc; (void)mv; return true; }
static void power_off(void) { power_off_count++; }
static void led_set(AppLedState state, bool blink) { (void)state; (void)blink; }
static void count(AppMode mode) { if (mode != APP_MODE_NONE) treatment_count++; }
static void screen_boot_sync(void) { }
static uint16_t storage_read(uint8_t address, uint16_t fallback) { (void)address; return fallback; }
static bool storage_write(uint8_t address, uint16_t value) { (void)address; (void)value; return true; }
static void storage_erase(void) { }
static void screen_float(uint16_t command, float value) { (void)command; (void)value; }
static void screen_u16(uint16_t command, uint16_t value) { (void)command; (void)value; }
static void screen_u32(uint16_t command, uint32_t value) { (void)command; (void)value; }
static void fault(AppFault value) { (void)value; }

static const AppPort port = {
    now_ms, outputs_off, heat, pressure_start, pressure_step, pressure_stop,
    home_begin, home_poll, zero_calibrate, eye_read, eye_consume, power_read,
    power_off, led_set, count, screen_boot_sync, storage_read, storage_write,
    storage_erase, storage_erase, screen_float, screen_u16, screen_u32, fault
};

static void reset_fixture(void)
{
    fake_now = 0U;
    fake_home_result = APP_ASYNC_BUSY;
    fake_eye = APP_EYE_NEW;
    home_begin_count = home_poll_count = eye_consume_count = 0U;
    treatment_count = power_off_count = pressure_start_count = 0U;
    AppController_Init(&port);
    fake_home_result = APP_ASYNC_OK;
    AppController_Tick();
    AppController_SetEyeState(APP_EYE_NEW);
    assert(AppController_Status()->state == APP_STATE_IDLE);
}

static void test_eye_is_consumed_only_at_formal_start(void)
{
    reset_fixture();
    assert(AppController_Prepare(APP_MODE_HEAT, 0.0f));
    assert(eye_consume_count == 0U);
    assert(AppController_Start());
    assert(eye_consume_count == 1U);
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
    assert(eye_consume_count == 1U);
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
    assert(!AppController_Prepare(APP_MODE_HEAT, 0.0f));
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

static void test_charging_blocks_and_interrupts_treatment(void)
{
    reset_fixture();
    AppController_SetPower(true, 50U, 3800U);
    assert(!AppController_Prepare(APP_MODE_AUTO, 350.0f));

    reset_fixture();
    assert(AppController_Prepare(APP_MODE_AUTO, 350.0f));
    assert(AppController_Start());
    AppController_SetPower(true, 50U, 3800U);
    assert(AppController_Status()->state == APP_STATE_HOMING);
    assert(AppController_Status()->stop_reason == APP_STOP_CHARGING);
}

static void test_low_voltage_homes_then_cuts_power(void)
{
    reset_fixture();
    for (unsigned i = 0U; i < 5U; ++i) {
        AppController_SetPower(false, 1U, 3150U);
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

int main(void)
{
    test_eye_is_consumed_only_at_formal_start();
    test_reinserted_consumed_eye_is_rejected();
    test_only_natural_finish_counts();
    test_charging_blocks_and_interrupts_treatment();
    test_low_voltage_homes_then_cuts_power();
    test_fault_and_power_loss_always_home();
    return 0;
}
