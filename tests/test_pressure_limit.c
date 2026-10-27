#include <assert.h>
#include <stdint.h>
#include "pressure_limit.h"

int main(void)
{
    PressureLimit limit = {0};
    assert(!PressureLimit_Update(&limit, 650.0f, 100U));
    assert(!PressureLimit_Update(&limit, 699.9f, 500U));
    assert(!PressureLimit_Update(&limit, 700.0f, 1000U));
    assert(!PressureLimit_Update(&limit, 800.0f, 1020U));
    assert(!PressureLimit_Update(&limit, 800.0f, 1060U));
    assert(!PressureLimit_Update(&limit, 700.0f, 1899U));
    assert(PressureLimit_Update(&limit, 700.0f, 1900U));

    /* Below-limit recovery cancels all elapsed over-limit time. */
    assert(!PressureLimit_Update(&limit, 699.0f, 1901U));
    assert(!PressureLimit_Update(&limit, 800.0f, 2000U));
    assert(!PressureLimit_Update(&limit, 800.0f, 2899U));
    assert(PressureLimit_Update(&limit, 800.0f, 2900U));

    /* Acquisition failure or stopping explicitly resets the monitor. */
    PressureLimit_Reset(&limit);
    assert(!PressureLimit_Update(&limit, 900.0f, 5000U));
    assert(!PressureLimit_Update(&limit, 900.0f, 5899U));
    assert(PressureLimit_Update(&limit, 900.0f, 5900U));

    PressureLimit_Reset(&limit);
    assert(!PressureLimit_Update(&limit, 700.0f, UINT32_MAX - 99U));
    assert(!PressureLimit_Update(&limit, 700.0f, 799U));
    assert(PressureLimit_Update(&limit, 700.0f, 800U));
    return 0;
}
