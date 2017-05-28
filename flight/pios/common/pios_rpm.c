#include "pios.h"

#include "pios_rpm.h"
#include <pios_led.h>

static uint32_t last_timestamp_us = 0;
static uint32_t last_dT_us = 0;
static float rpm_filtered = 0;
static float alpha = 1.0f;
bool PIOS_RPM_IRQHandler()
{
    PIOS_LED_Toggle(PIOS_LED_HEARTBEAT);
    last_dT_us = PIOS_DELAY_GetuSSince(last_timestamp_us);
    last_timestamp_us += last_dT_us;

    float rpm_new = 60000000.0f / last_dT_us;
    rpm_filtered = rpm_filtered * (1 - alpha) + rpm_new * alpha;
    return true;
}

float PIOS_RPM_GetRPM()
{
    static const uint32_t deadtime_us = 1000000;
    if (PIOS_DELAY_GetuSSince(last_timestamp_us) < deadtime_us) {
        return rpm_filtered;
    } else {
        return 0;
    }
}

float PIOS_RPM_GetTimestamp()
{
    return last_timestamp_us;
}

void PIOS_RPM_SetAlpha(float alpha_new)
{
    alpha = alpha_new;
}

