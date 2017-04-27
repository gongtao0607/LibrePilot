 #ifndef OBAR_H
 #define OBAR_H

#include "openpilot.h"
#include "stabilizationsettings.h"

float stabilization_obar_axes(float command, float gyro, float dT, bool measuredDterm_enabled, bool reinit, uint32_t axis);
float stabilization_obar_governor(float command, float sensor, bool reinit);

 #endif /* OBAR_H */
