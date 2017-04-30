#include "openpilot.h"
#include <pios_math.h>
#include "stabilization.h"
#include "stabilizationsettings.h"
#include "manualcontrolcommand.h"

typedef struct{
	int a;
}OBarPIDData;

static float low_pass_filter(float*history, float value, float alpha)
{
    return (*history=(*history)*(1-alpha)+value*alpha);
}

static const pid_scaler scaler_1 = {.p=1.0f, .i=1.0f, .d=1.0f,};

static float stabilization_obar_cyclic(float command, float gyro, float dT, bool measuredDterm_enabled, bool reinit, uint32_t axis)
{
    (void)reinit;
    float rate=StabilizationBankManualRateToArray(stabSettings.stabBank.ManualRate)[axis];
    stabSettings.innerPids[axis].iAccumulator*=stabSettings.settings.OBarCyclic.Decay;
    float setpoint_weight = stabSettings.settings.OBarCyclic.SetpointWeight;
    float command_forward = command * setpoint_weight;
    float output_back = pid_apply_setpoint(&stabSettings.innerPids[axis], &scaler_1, (command - command_forward) * rate,
         gyro, dT, measuredDterm_enabled);
    return command_forward + output_back;
}

static float stabilization_obar_tail(float command, float gyro, float dT, bool measuredDterm_enabled, bool reinit, uint32_t axis)
{
    (void)reinit;
    (void)low_pass_filter;
    static float command_lpf=0;
    command = low_pass_filter(&command_lpf, command, stabSettings.settings.OBarTail.LowPassFilter);
    float rate=StabilizationBankManualRateToArray(stabSettings.stabBank.ManualRate)[axis];
    stabSettings.innerPids[axis].iAccumulator*=stabSettings.settings.OBarTail.Decay;
    float setpoint_weight = stabSettings.settings.OBarTail.SetpointWeight;

    float command_forward = command * setpoint_weight;

    float output_back = pid_apply_setpoint(&stabSettings.innerPids[axis], &scaler_1, (command - command_forward) * rate, gyro, dT, measuredDterm_enabled);

    float collective, roll, pitch;
    ManualControlCommandCollectiveGet(&collective);
    ManualControlCommandRollGet(&roll);
    ManualControlCommandPitchGet(&pitch);
    float output_forward = fabsf(collective) * stabSettings.settings.OBarTail.CollectiveFeedforward;
    //output_forward += (fabsf(roll) + fabsf(pitch)) * stabSettings.settings.OBarTail.CyclicFeedforward;
    return output_forward + output_back + command_forward;
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
