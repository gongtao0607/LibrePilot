#include "openpilot.h"
#include <pios_math.h>
#include "stabilization.h"
#include "stabilizationsettings.h"
#include "manualcontrolcommand.h"
#include <pios_rpm.h>

static struct pid governorPid;
static const pid_scaler scaler_111 = {.p=1.0f, .i=1.0f, .d=1.0f,};

static inline void obar_axes_init(uint32_t axis)
{
    pid_zero(&stabSettings.innerPids[axis]);
}
static inline void obar_governor_init()
{
    pid_zero(&governorPid);
}

static inline float obar_cyclic(float command, float gyro, float dT, bool measuredDterm_enabled, uint32_t axis)
{
    float rate=StabilizationBankManualRateToArray(stabSettings.stabBank.ManualRate)[axis];
    stabSettings.innerPids[axis].iAccumulator*=stabSettings.settings.OBarCyclic.Decay;
    float setpoint_weight = stabSettings.settings.OBarCyclic.SetpointWeight;
    float command_forward = command * setpoint_weight;
    float output_back = pid_apply_setpoint(&stabSettings.innerPids[axis], &scaler_111, (command - command_forward) * rate,
         gyro, dT, measuredDterm_enabled);
    return command_forward + output_back;
}
static inline float tail_prefilter(float command, float dT)
{
    StabilizationSettingsOBarTailData *const obartail = &stabSettings.settings.OBarTail;
    static float command0 = 0;
    static float command_rate0 = 0;

    //rate cap; value in unit degree/sec;
    //90degree = [-1,+1]. Angle = command * 45
    // (command_rate * 45) = degree/sec
    float command_rate = (command - command0) /dT;
    const float max_command_rate = obartail->ServoRateLimit / 45;
    if( command_rate > max_command_rate){
        command_rate = max_command_rate;
    }
    if( command_rate < -max_command_rate){
        command_rate = -max_command_rate;
    }

    //accel cap; value in unit degree/sec^2;
    // (command_diff_diff * 45)/dT = degree/sec^2
    //just cap the accel, don't cap decel (not necessary)
    float command_accel = (command_rate - command_rate0) / dT;
    const float max_command_accel = obartail->ServoAccelLimit / 45;
    //filter out deceleration:
    // rate and rate0 has same direction and |rate| < |rate0|
    if(fabsf(command_rate) > fabsf(command_rate0) || command_rate * command_rate0 < 0) {
        if( command_accel > max_command_accel){
            command_accel = max_command_accel;
        }
        if( command_accel < -max_command_accel){
            command_accel = -max_command_accel;
        }
    }
    command_rate = command_rate0 + command_accel * dT;
    command = command0 + command_rate * dT;
    command0 = command;
    command_rate0 = command_rate;

    return command;
}
static inline float obar_tail(float command, float gyro, float dT, bool measuredDterm_enabled, uint32_t axis)
{
    command = tail_prefilter(command, dT);
    StabilizationSettingsOBarTailData *const obartail = &stabSettings.settings.OBarTail;

    float rate=StabilizationBankManualRateToArray(stabSettings.stabBank.ManualRate)[axis];
    stabSettings.innerPids[axis].iAccumulator*=obartail->Decay;
    float setpoint_weight = obartail->SetpointWeight;

    float command_forward = command * setpoint_weight;

    float output_back = pid_apply_setpoint(&stabSettings.innerPids[axis], &scaler_111, (command - command_forward) * rate, gyro, dT, measuredDterm_enabled);

    float collective, roll, pitch;
    ManualControlCommandCollectiveGet(&collective);
    ManualControlCommandRollGet(&roll);
    ManualControlCommandPitchGet(&pitch);
    float output_forward = fabsf(collective) * obartail->CollectiveFeedforward;
    output_forward += (fabsf(roll) + fabsf(pitch)) * obartail->CyclicFeedforward;
    return output_forward + output_back + command_forward;
}

float stabilization_obar_axes(float command, float gyro, float dT, bool measuredDterm_enabled, bool reinit, uint32_t axis)
{
    if (reinit) {
        obar_axes_init(axis);
    }
    switch (axis) {
    case 0:
    case 1:
        return obar_cyclic(command, gyro, dT, measuredDterm_enabled, axis);
    case 2:
        return obar_tail(command, gyro, dT, measuredDterm_enabled, axis);
    default:
        return 0;
    }
}

float stabilization_obar_governor(float command, float dT, bool measuredDterm_enabled, bool reinit)
{
    if (reinit) {
        obar_governor_init();
    }
    float currentRpm = PIOS_RPM_GetRPM();
    //safe mode either throttle < 50% or RPM reads out < 100
    if( command < 0.5f || currentRpm < 100.0f) return command;
    StabilizationSettingsOBarGovernorData *const obargovernor = &stabSettings.settings.OBarGovernor;
    pid_configure(&governorPid, obargovernor->Kp, obargovernor->Ki, obargovernor->Kd, 1.0f);
    float desiredRpm = obargovernor->HeadSpeed * obargovernor->GearRatio * obargovernor->SensorScale;
    float output = command + pid_apply_setpoint(&governorPid, &scaler_111, desiredRpm, currentRpm, dT, measuredDterm_enabled);
    return output;
}
