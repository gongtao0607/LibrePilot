#include "openpilot.h"
#include <pios_math.h>
#include "stabilization.h"
#include "stabilizationsettings.h"

typedef struct{
	int a;
}OBarPIDData;

static float tau_to_alpha(float tau)
{
    const float fakeDt = 0.0025f;
    if(tau < 0.0001f) {
        return 0;
    } else {
        return expf(-fakeDt / stabSettings.settings.OBarCyclic.Tau);
    }
}
/*static float low_pass_filter(float*history, float value, float tau)
{
    float alpha = tau_to_alpha(tau);
    return (*history=((*history*alpha)+value*(1-alpha));
}*/

static const pid_scaler scaler_1 = {.p=1.0f, .i=1.0f, .d=1.0f,};

static float stabilization_obar_cyclic(float command, float gyro, float dT, bool measuredDterm_enabled, bool reinit, uint32_t axis)
{
    (void)reinit;
    float rate=StabilizationBankManualRateToArray(stabSettings.stabBank.ManualRate)[axis];
    float decay_alpha = tau_to_alpha(stabSettings.settings.OBarCyclic.Tau);
    stabSettings.innerPids[axis].iAccumulator*=decay_alpha;
    float setpoint_weight = stabSettings.settings.OBarCyclic.SetpointWeight;
    float output_forward = command * setpoint_weight;
    float output_back = pid_apply_setpoint(&stabSettings.innerPids[axis], &scaler_1, (command - output_forward) * rate,
         gyro, dT, measuredDterm_enabled);
    return output_forward + output_back;
}

static float stabilization_obar_tail(float command, float gyro, float dT, bool measuredDterm_enabled, bool reinit, uint32_t axis)
{
    (void)reinit;
    float rate=StabilizationBankManualRateToArray(stabSettings.stabBank.ManualRate)[axis];
    float decay_alpha = tau_to_alpha(stabSettings.settings.OBarTail.Tau);
    stabSettings.innerPids[axis].iAccumulator*=decay_alpha;
    float setpoint_weight = stabSettings.settings.OBarTail.SetpointWeight;
    float output_forward = command * setpoint_weight;
    float output_back = pid_apply_setpoint(&stabSettings.innerPids[axis], &scaler_1, (command - output_forward) * rate, gyro, dT, measuredDterm_enabled);
    return output_forward + output_back;
}

float stabilization_obar_axes(float command, float gyro, float dT, bool measuredDterm_enabled, bool reinit, uint32_t axis)
{
    switch (axis) {
    case 0:
    case 1:
        return stabilization_obar_cyclic(command, gyro, dT, measuredDterm_enabled, reinit, axis);
    case 2:
        return stabilization_obar_tail(command, gyro, dT, measuredDterm_enabled, reinit, axis);
    default:
        return 0;
    }
}

float stabilization_obar_governor(float command, float sensor, bool reinit)
{
    (void)reinit;
    (void)sensor;
    return command;
}
