#include "pid.h"
#include <stdio.h>

float PID_PhySys(float force)
{
	float spring = 1.0f;
	float damping = 1.0f / 1000.0f;
	float mass = 1.0f / 1000.0f;

	static float last_result = 0;
	static float prev_result = 0;
	static float result = 0;

	last_result = prev_result;
	prev_result = result;
	result = (force + 2 * mass * prev_result - mass * last_result + damping * prev_result) / (mass + damping + spring);

	return result;
}

/* int main()
{
	PID_t pid;

	FILE* fp;

	fopen_s(&fp, "PID_Test_Log.txt", "w+");

	float force, pos;

	PID_Init(&pid, Regular, 0.0f, 0.2f, 0.0f, 0.0f, 10000.0f, -10000.0f, 10000.0f);

	PID_SetRef(&pid, 5000);

	force = 0.0f;

	for (int i = 0; i < 50; i++)
	{
		pos = PID_PhySys(force);
		force = PID_Calc(&pid, pos);
		fprintf(fp, "%7.3f\t%7.3f\t\n", pos, force);
	}
}*/
