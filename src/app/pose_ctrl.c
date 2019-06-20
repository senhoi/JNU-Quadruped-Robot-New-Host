#include "pose_ctrl.h"

extern GYRO_t Gyro;

PID_t pid_Roll;
PID_t pid_Pitch;

GYRO_t RefGyro;

void PoseCtrlInit(void)
{
	RefGyro = Gyro;

	PID_Init(&pid_Roll, Regular, 0.005f, 0.0f, 0.0f, 0.0f, 12.0f, -12.0f, 1.0f);
	PID_Init(&pid_Pitch, Regular, 0.005f, 0.0f, 0.0f, 0.0f, 12.0f, -12.0f, 1.0f);

	PID_SetRef(&pid_Roll, RefGyro.Angle_ft.roll);
	PID_SetRef(&pid_Pitch, RefGyro.Angle_ft.pitch);

	printf("r-ro:%f\tr-pi:%f\n", RefGyro.Angle_ft.roll, RefGyro.Angle_ft.pitch);
}

void PoseCtrlTask(RC_Robot_t *robot, int ctrl_method)
{
	float delta_roll = PID_Calc(&pid_Roll, GYRO_GetFilt(&Gyro, GYRO_DATATYPE_ANGLE, GYRO_DATASUBTYPE_X));
	float delta_pitch = PID_Calc(&pid_Pitch, GYRO_GetFilt(&Gyro, GYRO_DATATYPE_ANGLE, GYRO_DATASUBTYPE_Y));

	switch (ctrl_method)
	{
	case CTRL_NONE:
		robot->Pose.body_ro = 0;
		robot->Pose.body_pi = 0;

		//robot->Pose.pos_ro = robot->Pose.body_ro;
		//robot->Pose.pos_pi = robot->Pose.body_pi;
		break;
	case CTRL_BODY:
		robot->Pose.body_ro -= delta_roll / 180.0f * pi;
		robot->Pose.body_pi += delta_pitch / 180.0f * pi;

		printf("inc-ro:%f\tinc-pi:%f\n", delta_roll, delta_pitch);

		robot->Pose.body_ro = abslimit(pi / 10, robot->Pose.body_ro);
		robot->Pose.body_pi = abslimit(pi / 10, robot->Pose.body_pi);

		printf("body c-ro:%f\tc-pi:%f\n", robot->Pose.body_ro, robot->Pose.body_pi);

		break;
	case CTRL_POS:
		robot->Pose.body_ro = 0;
		robot->Pose.body_pi = 0;

		robot->Pose.pos_ro += delta_roll / 180.0f * pi;
		robot->Pose.pos_pi -= delta_pitch / 180.0f * pi;

		printf("inc-ro:%f\tinc-pi:%f\n", delta_roll, delta_pitch);

		robot->Pose.pos_ro = abslimit(pi / 12, robot->Pose.pos_ro);
		robot->Pose.pos_pi = abslimit(pi / 12, robot->Pose.pos_pi);

		printf("pos c-ro:%f\tc-pi:%f\n", robot->Pose.pos_ro, robot->Pose.pos_pi);
		break;
	case CTRL_BOTH:
		robot->Pose.body_ro = -(GYRO_GetFilt(&Gyro, GYRO_DATATYPE_ANGLE, GYRO_DATASUBTYPE_X) - RefGyro.Angle_ft.roll) / 180.0f * pi;
		robot->Pose.body_pi = (GYRO_GetFilt(&Gyro, GYRO_DATATYPE_ANGLE, GYRO_DATASUBTYPE_Y) - RefGyro.Angle_ft.pitch) / 180.0f * pi;

		robot->Pose.pos_ro = robot->Pose.body_ro;
		robot->Pose.pos_pi = robot->Pose.body_pi;

		robot->Pose.body_ro = abslimit(pi / 12, robot->Pose.body_ro);
		robot->Pose.body_pi = abslimit(pi / 12, robot->Pose.body_pi);

		robot->Pose.pos_ro = abslimit(pi / 12, robot->Pose.pos_ro);
		robot->Pose.pos_pi = abslimit(pi / 12, robot->Pose.pos_pi);

		printf("both c-ro:%f\tc-pi:%f\n", robot->Pose.body_ro, robot->Pose.body_pi);

	default:
		break;
	}
}
/* 
void PoseCtrlTask(RC_Robot_t *robot, int ctrl_method)
{
	switch (ctrl_method)
	{
	case CTRL_NONE:
		//robot->Pose.body_ro += delta_roll;
		robot->Pose.body_pi = 0;

		//robot->Pose.pos_ro = robot->Pose.body_ro;
		robot->Pose.pos_pi = robot->Pose.body_pi;
		break;
	case CTRL_BODY:
		//robot->Pose.body_ro += delta_roll / 180.0f * pi;
		robot->Pose.body_pi = -(GYRO_GetFilt(&Gyro, GYRO_DATATYPE_ANGLE, GYRO_DATASUBTYPE_X) - RefGyro.Angle_ft.pitch) / 180.0f * pi;

		//robot->Pose.body_ro = abslimit(pi / 10, robot->Pose.body_ro);
		robot->Pose.body_pi = abslimit(pi / 10, robot->Pose.body_pi);

		printf("body c-ro:%f\tc-pi:%f\n", robot->Pose.body_ro, robot->Pose.body_pi);

		break;
	case CTRL_BOTH:
		//robot->Pose.body_ro += delta_roll;
		robot->Pose.body_pi = -(GYRO_GetFilt(&Gyro, GYRO_DATATYPE_ANGLE, GYRO_DATASUBTYPE_X) - RefGyro.Angle_ft.pitch) / 180.0f * pi;

		//robot->Pose.pos_ro = robot->Pose.body_ro;
		robot->Pose.pos_pi = robot->Pose.body_pi;

		robot->Pose.body_ro = abslimit(pi / 10, robot->Pose.body_ro);
		robot->Pose.body_pi = abslimit(pi / 10, robot->Pose.body_pi);

		robot->Pose.pos_ro = abslimit(pi / 10, robot->Pose.pos_ro);
		robot->Pose.pos_pi = abslimit(pi / 10, robot->Pose.pos_pi);

		printf("both c-ro:%f\tc-pi:%f\n", robot->Pose.body_ro, robot->Pose.body_pi);

	default:
		break;
	}
}*/
