#include "openpilot.h"
#include <pios_math.h>
#include "stabilization.h"
#include "stabilizationsettings.h"

typedef struct{
	int a;
}OBarPIDData;

static const pid_scaler scaler_1 = {.p=1.0f, .i=1.0f, .d=1.0f,};

float stabilization_obar_axes(float command, float gyro, float dT, bool measuredDterm_enabled, bool reinit, uint32_t axis)
{
    (void)reinit;
    command = boundf(command,
        -StabilizationBankMaximumRateToArray(stabSettings.stabBank.MaximumRate)[axis],
        StabilizationBankMaximumRateToArray(stabSettings.stabBank.MaximumRate)[axis]
    );
    float output = pid_apply_setpoint(&stabSettings.innerPids[axis], &scaler_1, command, gyro, dT, measuredDterm_enabled);
    return output;
}

float stabilization_obar_governor(float command, float sensor, bool reinit)
{
    (void)reinit;
    (void)sensor;
    return command;
}
