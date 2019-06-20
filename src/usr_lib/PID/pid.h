#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>

typedef enum PID_Mode_t
{
	Regular,
	Increment
}PID_Mode_t;

/*integral separating*/
typedef struct PID_IntSp_t
{
	uint8_t enable;
	float threshold;
}PID_IntSp_t;

typedef struct PID_t
{
	PID_Mode_t mode;

	float Kp;
	float Ki;
	float Kd;

	float deadband;	
	/*	When the error is within the deadband,
		the PID controller will not work.*/
	
	float max_out;
	float min_out;
	/*	When the feedback value is beyond the limit, 
		the PID controller will automatically work 
		with integral anti-satuation. Only useful to 
		regular pid.*/

	float max_inc;
	/*	The increment of output will be limited to
		avoid a big step. This is also effective to
		the regular mode.*/
	
	PID_IntSp_t int_sp;
	/*	If the integral anti-satuation is not effective 
		enough, you can use intergral separating to reduce 
		the overshoot. When the reference val is far form
		the feedback(|ref - fbk| > fbk), the intergral loop 
		will not work. Only useful to 
		regular pid.*/
	
	float out;		/*E(K)*/
	float ref;
	float fbk;
	float err;

	float err_sum;
	float prev_err;	/*E(K-1)*/
	float prev_out;
	float last_err;	/*E(K-2)*/

}PID_t;

void PID_Init(PID_t* pid, PID_Mode_t mode, float Kp, float Ki, float Kd, float deadband, float max_out, float min_out, float max_inc);

void PID_SetIntSp(PID_t* pid, bool enable, float threshold);

void PID_SetRef(PID_t* pid, float ref);

float PID_Calc(PID_t* pid, float fbk);

#endif