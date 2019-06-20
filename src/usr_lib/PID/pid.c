#include "pid.h"
#include "pid_math.h" 

void PID_Init(PID_t* pid, PID_Mode_t mode, float Kp, float Ki, float Kd,
	float deadband, float max_out, float min_out, float max_inc)
{
	pid->mode = mode;

	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;

	pid->deadband = (float)fabs(deadband);
	pid->max_out = max_out;
	pid->min_out = min_out;
	pid->max_inc = (float)fabs(max_inc);

	pid->int_sp.enable = false;
	pid->int_sp.threshold = 0;

	pid->out = 0;
	pid->err = 0;
	pid->ref = 0;
	pid->fbk = 0;
	pid->err_sum = 0;
	pid->prev_err = 0;
	pid->prev_out = 0;
}

void PID_SetIntSp(PID_t* pid, bool enable, float threshold)
{
	pid->int_sp.enable = enable;
	pid->int_sp.threshold = threshold;
}

void PID_SetRef(PID_t* pid, float ref)
{
	pid->ref = limit(pid->max_out, pid->min_out, ref);
}

float PID_Calc(PID_t* pid, float fbk)
{
	float inc;

	pid->fbk = fbk;

	pid->last_err = pid->prev_err;
	pid->prev_err = pid->err;
	pid->prev_out = pid->out;

	pid->err = deadband(pid->deadband, pid->ref - pid->fbk);

	switch (pid->mode)
	{
	case Regular:
		if (pid->int_sp.enable == true && pid->err > pid->int_sp.threshold)
		{
			pid->out = pid->Kp * pid->err + pid->Kd * (pid->prev_err - pid->err);
		}
		else
		{
			if ((pid->fbk > pid->max_out && pid->err < 0) ||
				(pid->fbk < pid->min_out && pid->err > 0) ||
				(pid->fbk > pid->min_out && pid->fbk < pid->max_out))
				pid->err_sum += pid->err;
			pid->out = pid->Kp * pid->err + pid->Ki * pid->err_sum + pid->Kd * (pid->prev_err - pid->err);
		}

		pid->out = limit(pid->max_out, pid->min_out, pid->out);

		if (pid->out - pid->prev_out > pid->max_inc)
			pid->out = pid->prev_out + pid->max_inc;
		else if (pid->out - pid->prev_out < -pid->max_inc)
			pid->out = pid->prev_out - pid->max_inc;
		else
			pid->out = pid->out;
		break;

	case Increment:
		inc = pid->Kp * (pid->err - pid->prev_err) + pid->Ki * pid->err + pid->Kd * (pid->err - 2 * pid->prev_err + pid->last_err);
		inc = abslimit(pid->max_inc, inc);
		pid->out = pid->prev_out + inc;
		break;

	default:
		break;
	}

	pid->out = limit(pid->max_out, pid->min_out, pid->out);

	return pid->out;
}

