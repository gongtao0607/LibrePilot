#include "openpilot.h"
#include <pios_math.h>
#include "stabilization.h"
#include "stabilizationsettings.h"
#include "manualcontrolcommand.h"
#include <pios_rpm.h>

static inline float low_pass_filter(float*history, float value, float alpha)
{
    return (*history=(*history)*(1-alpha)+value*alpha);
}

static const pid_scaler scaler_111 = {.p=1.0f, .i=1.0f, .d=1.0f,};
static const pid_scaler scaler_101 = {.p=1.0f, .i=0.0f, .d=1.0f,};

static inline float stabilization_obar_cyclic(float command, float gyro, float dT, bool measuredDterm_enabled, bool reinit, uint32_t axis)
{
    (void)reinit;
    float rate=StabilizationBankManualRateToArray(stabSettings.stabBank.ManualRate)[axis];
    stabSettings.innerPids[axis].iAccumulator*=stabSettings.settings.OBarCyclic.Decay;
    float setpoint_weight = stabSettings.settings.OBarCyclic.SetpointWeight;
    float command_forward = command * setpoint_weight;
    float output_back = pid_apply_setpoint(&stabSettings.innerPids[axis], &scaler_111, (command - command_forward) * rate,
         gyro, dT, measuredDterm_enabled);
    return command_forward + output_back;
}

static inline float stabilization_obar_tail(float command, float gyro, float dT, bool measuredDterm_enabled, bool reinit, uint32_t axis)
{
    (void)reinit;
    const pid_scaler * p_scaler = &scaler_111;
    static float command_lpf=0;
    low_pass_filter(&command_lpf, command, stabSettings.settings.OBarTail.LowPassFilter);
    if (fabsf(command_lpf-command)>stabSettings.settings.OBarExtra.A){
        //damping, don't accumulate I
	p_scaler = &scaler_101;
    }
    command = command_lpf;
    float rate=StabilizationBankManualRateToArray(stabSettings.stabBank.ManualRate)[axis];
    stabSettings.innerPids[axis].iAccumulator*=stabSettings.settings.OBarTail.Decay;
    float setpoint_weight = stabSettings.settings.OBarTail.SetpointWeight;

    float command_forward = command * setpoint_weight;

    float output_back = pid_apply_setpoint(&stabSettings.innerPids[axis], p_scaler, (command - command_forward) * rate, gyro, dT, measuredDterm_enabled);

    float collective, roll, pitch;
    ManualControlCommandCollectiveGet(&collective);
    ManualControlCommandRollGet(&roll);
    ManualControlCommandPitchGet(&pitch);
    float output_forward = fabsf(collective) * stabSettings.settings.OBarTail.CollectiveFeedforward;
    output_forward += (fabsf(roll) + fabsf(pitch)) * stabSettings.settings.OBarTail.CyclicFeedforward;
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

static struct pid governorPid;
void stabilization_obar_init()
{
    pid_zero(&governorPid);
}

float stabilization_obar_governor(float command, float dT, bool measuredDterm_enabled, bool reinit)
{
    (void)reinit;
    float currentRpm = PIOS_RPM_GetRPM();
    if( command < 0.5f || currentRpm < 100.0f) return command;
    StabilizationSettingsOBarGovernorData *obargovernor = &stabSettings.settings.OBarGovernor;
    pid_configure(&governorPid, obargovernor->Kp, obargovernor->Ki, obargovernor->Kd, 1.0f);
    float desiredRpm = obargovernor->HeadSpeed * obargovernor->GearRatio * obargovernor->SensorScale;

    float output = command + pid_apply_setpoint(&governorPid, &scaler_111, desiredRpm, currentRpm, dT, measuredDterm_enabled);
    return output;
}
