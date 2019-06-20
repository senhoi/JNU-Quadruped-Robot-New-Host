#ifndef PID_MATH_H
#define PID_MATH_H

#include <math.h>

#include "pid_math.h"

float abslimit(float limit, float input);

float limit(float upper, float lower, float input);

float deadband(float bandwidth, float input);

#endif