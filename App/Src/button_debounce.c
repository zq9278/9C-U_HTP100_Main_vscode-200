#include "button_debounce.h"

#include "app_config.h"

bool ButtonDebounce_Update(ButtonDebounce *button, bool pressed,
                           bool falling_edge, uint32_t now_ms)
{
    if (pressed != button->candidate_pressed ||
        (falling_edge && pressed && !button->press_latched)) {
        button->candidate_pressed = pressed;
        button->candidate_started_ms = now_ms;
    }
    if (now_ms - button->candidate_started_ms < APP_BUTTON_DEBOUNCE_MS) {
        return false;
    }
    if (!pressed) {
        button->press_latched = false;
        return false;
    }
    if (!button->press_latched) {
        button->press_latched = true;
        return true;
    }
    return false;
}
