#include <assert.h>
#include <stdint.h>

#include "app_config.h"
#include "button_debounce.h"

int main(void)
{
    ButtonDebounce button = {0};
    assert(!ButtonDebounce_Update(&button, false, false, 100U));
    /* Press bounces across task polls, then emits exactly one event. */
    assert(!ButtonDebounce_Update(&button, true, true, 110U));
    assert(!ButtonDebounce_Update(&button, false, false, 115U));
    assert(!ButtonDebounce_Update(&button, true, true, 120U));
    assert(!ButtonDebounce_Update(&button, true, false, 149U));
    assert(ButtonDebounce_Update(&button, true, false, 150U));
    assert(!ButtonDebounce_Update(&button, true, true, 170U));
    assert(!ButtonDebounce_Update(&button, true, false, 5000U));
    /* Release bounce cannot turn the same physical press into stop. */
    assert(!ButtonDebounce_Update(&button, false, false, 5010U));
    assert(!ButtonDebounce_Update(&button, true, true, 5020U));
    assert(!ButtonDebounce_Update(&button, true, false, 5100U));
    assert(!ButtonDebounce_Update(&button, false, false, 5110U));
    assert(!ButtonDebounce_Update(&button, false, false, 5140U));
    assert(!ButtonDebounce_Update(&button, true, true, 5150U));
    assert(ButtonDebounce_Update(&button, true, false, 5180U));

    /* An edge between polls restarts confirmation even if release was missed. */
    button = (ButtonDebounce){0};
    assert(!ButtonDebounce_Update(&button, true, true, 100U));
    assert(!ButtonDebounce_Update(&button, true, true, 125U));
    assert(!ButtonDebounce_Update(&button, true, false, 130U));
    assert(ButtonDebounce_Update(&button, true, false, 155U));

    /* A glitch shorter than the confirmation interval never emits an event. */
    button = (ButtonDebounce){0};
    assert(!ButtonDebounce_Update(&button, true, true, 100U));
    assert(!ButtonDebounce_Update(&button, false, false, 110U));
    assert(!ButtonDebounce_Update(&button, false, false, 200U));

    /* Unsigned elapsed-time arithmetic also works across tick rollover. */
    button = (ButtonDebounce){0};
    assert(!ButtonDebounce_Update(&button, true, true, UINT32_MAX - 10U));
    assert(!ButtonDebounce_Update(&button, true, false, 18U));
    assert(ButtonDebounce_Update(&button, true, false, 19U));
    return 0;
}
