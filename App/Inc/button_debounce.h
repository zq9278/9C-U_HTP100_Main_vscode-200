#ifndef BUTTON_DEBOUNCE_H
#define BUTTON_DEBOUNCE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool candidate_pressed;
    bool press_latched;
    uint32_t candidate_started_ms;
} ButtonDebounce;

/* Emit one event after a stable press. Require a stable release to rearm.
 * Falling-edge hints restart press confirmation even when polling missed
 * the intervening high level. Call regularly in task context. */
bool ButtonDebounce_Update(ButtonDebounce *button, bool pressed,
                           bool falling_edge, uint32_t now_ms);

#endif
