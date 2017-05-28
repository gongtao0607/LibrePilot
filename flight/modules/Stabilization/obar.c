#include "openpilot.h"
#include <pios_math.h>
#include "stabilization.h"
#include "stabilizationsettings.h"
#include "stabilizationstatus.h"
#include "manualcontrolcommand.h"
#include <pios_rpm.h>
#include <pios_led.h>

static struct pid governorPid;
static const pid_scaler scaler_111 = {.p=1.0f, .i=1.0f, .d=1.0f,};

struct lowpass {
    float y0;
};
struct highpass {
    float x0;
    float y0;
};
static inline float lowpass_apply(struct lowpass*lp, float x, float dT, float rc)
{
    float a = dT/(rc+dT);
    float y = lp->y0 * (1.0f-a) + a * x;
    return lp->y0 = y;
}
static inline float highpass_apply(struct highpass*hp, float x, float dT, float rc)
{
    float a = rc/(rc+dT);
    float y = a*(hp->y0 +x -hp->x0);
    hp->x0=x;
    return hp->y0=y;
}

static inline void obar_axes_init(uint32_t axis)
{
    pid_zero(&stabSettings.innerPids[axis]);
}
static inline void obar_governor_init()
{
    pid_zero(&governorPid);
}

static inline float obar_cyclic(float command, float gyro, float dT, uint32_t axis)
{
    static StabilizationSettingsOBarCyclicData *const obarcyclic = &stabSettings.settings.OBarCyclic;
    static pid_scaler scaler;
    float rate=StabilizationBankManualRateToArray(stabSettings.stabBank.ManualRate)[axis];
    float alpha = obarcyclic->Decay/(obarcyclic->Decay+dT);
    stabSettings.innerPids[axis].iAccumulator*=alpha;
    float suppress = 1.0f - obarcyclic->A * fabsf(command);
    suppress = boundf(suppress, 0, 1.0f);
    scaler.p=scaler.i=scaler.d=suppress;
    float feedforward = command * obarcyclic->SetpointWeight;
    float feedback = pid_apply_setpoint(&stabSettings.innerPids[axis], &scaler, command * rate, gyro, dT, true);
    return feedforward + feedback;
}

static inline bool detect_saturation(float *output, float dT)
{
    static StabilizationSettingsOBarTailData *const obartail = &stabSettings.settings.OBarTail;
    static float output0 = 0;
    static float output_rate0 = 0;
    bool detected = false;

    //rate cap; value in unit degree/sec;
    //90degree = [-1,+1]. Angle = output * 45
    // (output_rate * 45) = degree/sec
    float output_rate = (*output - output0) /dT;
    const float max_output_rate = obartail->ServoRateLimit / 45;
    if (output_rate > max_output_rate) {
        output_rate = max_output_rate;
        detected = true;
    }
    if (output_rate < -max_output_rate) {
        output_rate = -max_output_rate;
        detected = true;
    }

    //accel cap; value in unit degree/sec^2;
    // (output_diff_diff * 45)/dT = degree/sec^2
    //just cap the accel, don't cap decel (not necessary)
    float output_accel = (output_rate - output_rate0) / dT;
    const float max_output_accel = obartail->ServoAccelLimit / 45;
    //filter out deceleration:
    //|rate| < |rate0|
    if (fabsf(output_rate) > fabsf(output_rate0)) {
        if( output_accel > max_output_accel){
            output_accel = max_output_accel;
	    detected = true;
        }

        if( output_accel < -max_output_accel){
            output_accel = -max_output_accel;
	    detected = true;
        }
    }
    
    //however, we only store the bounded output here statically. 
    //It doesn't apply to the real output.
    //Which is good, so user could always get fast response.
    output_rate = output_rate0 + output_accel * dT;
    output_rate0 = output_rate;
    *output = output0 + output_rate * dT;
    output0 = *output;

    return detected;
}

static inline float obar_tail(float command, float gyro, float dT, uint32_t axis)
{
    static StabilizationSettingsOBarTailData *const obartail = &stabSettings.settings.OBarTail;

    float rate=StabilizationBankManualRateToArray(stabSettings.stabBank.ManualRate)[axis];

    //store old integral in case of saturation detected
    //float iAccumulator0 = stabSettings.innerPids[axis].iAccumulator;
    static pid_scaler scaler = {.p=1.0f, .i=1.0f, .d=1.0f,};
 
    if(detect_saturation(&command,dT)){
        //stabSettings.innerPids[axis].iAccumulator = iAccumulator0;
        scaler.p=obartail->StopGain;scaler.i=scaler.d=0;
    } else {
        scaler.p=scaler.i=scaler.d=1.0f;
    }
    
    float feedback = pid_apply_setpoint(&stabSettings.innerPids[axis], &scaler, command * rate, gyro, dT, true);

    float collective;//, roll, pitch;
    ManualControlCommandCollectiveGet(&collective);
//    ManualControlCommandRollGet(&roll);
//    ManualControlCommandPitchGet(&pitch);
    float feedforward = fabsf(collective) * obartail->CollectiveFeedforward;
//    feedforward += (fabsf(roll) + fabsf(pitch)) * obartail->CyclicFeedforward;
    feedforward += obartail->SetpointWeight * command;
    float output = feedforward + feedback;

    return output;
}

float stabilization_obar_axes(float command, float gyro, float dT, bool reinit, uint32_t axis)
{
/*    static bool first_init = true;
    if (first_init) {
        static float d0=0;
        d0 += dT;
        //twitch elevator and rudder for 0.2 s
        if (d0 > 0.2f) first_init = false;
        if (axis != 0) {
            return command + 0.5f;
        } else {
            return command;
        }
    }*/
    static StabilizationStatusData status;
    StabilizationStatusGet(&status);
    float output = 0;
    if (reinit) {
        obar_axes_init(axis);
    }
    switch (axis) {
    case 0:
        output = obar_cyclic(command, gyro, dT, axis);
        status.OBarIntegral.Aileron = stabSettings.innerPids[axis].iAccumulator;
        break;
    case 1:
        output = obar_cyclic(command, gyro, dT, axis);
        status.OBarIntegral.Elevator = stabSettings.innerPids[axis].iAccumulator;
        break;
    case 2:
        output = obar_tail(command, gyro, dT, axis);
        status.OBarIntegral.Rudder = stabSettings.innerPids[axis].iAccumulator;
        break;
    }
    StabilizationStatusSet(&status);
    return output;
}

float stabilization_obar_governor(float command, float dT, bool reinit)
{
    static StabilizationSettingsOBarGovernorData *const obargovernor = &stabSettings.settings.OBarGovernor;
    StabilizationStatusData status;
    StabilizationStatusGet(&status);
    float output = command;
    if (reinit) {
        obar_governor_init();
    }
    float currentHS = PIOS_RPM_GetRPM() / obargovernor->GearRatio / obargovernor->SensorScale;
    float desiredHS = obargovernor->HeadSpeed;
    //safe mode list:
    //desiredHS unset;
    //throttle < 50%;
    //RPM reads out < 70% setspeed;
    if (desiredHS < 0.0001f || command < 0.50f || (currentHS / desiredHS) < 0.7f) {

    } else {
    //here we have governor engaged
        pid_configure(&governorPid, obargovernor->Kp, obargovernor->Ki, obargovernor->Kd, 1.0f);
        output = command + pid_apply_setpoint(&governorPid, &scaler_111, desiredHS, currentHS, dT, true);
        //no less than 0.3
        output = boundf(output, 0.3f, 1.0f);
    }
    status.OBarHeadSpeed = currentHS;
    status.OBarIntegral.Throttle = governorPid.iAccumulator;
    StabilizationStatusSet(&status);
    return output;
}
