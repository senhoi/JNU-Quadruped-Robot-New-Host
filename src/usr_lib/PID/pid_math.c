#include "pid_math.h"

float abslimit(float limit, float input)
{
	float output;

	limit = fabs(limit);

	if (input > limit)
		output = limit;
	else if (input < -limit)
		output = -limit;
	else
		output = input;

	return output;
}

float limit(float upper, float lower, float input)
{
	float output;

	if (input > upper)
		output = upper;
	else if (input < lower)
		output = lower;
	else
		output = input;

	return output;
}

float deadband(float bandwidth, float input)
{
	float output;

	bandwidth = fabs(bandwidth);

	if (input > bandwidth)
		output = input - bandwidth;
	else if (input < -bandwidth)
		output = input + bandwidth;
	else
		output = 0;

	return output;
}
