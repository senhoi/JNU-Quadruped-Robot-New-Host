#include "tasks.h"

#define FRAME_HEAD 0x55AA
#define FRAMETYPE_REMOTE 0x11
#define FRAMETYPE_CAMERA 0x21
#define FRAMETYPE_GYROSCOPE 0x22
#define FRAMETYPE_FOOTGROUNDING 0x23

RC_Robot_t QuadrupedRobot;

matrix_t *m_pos;
matrix_t *m_rad;

float Phase;

PID_Regular_t PID_yaw;
float Locked_yaw;
struct
{
	float span_x;
	float span_y;
	float span_z;
	float span_w;

	float cycle_max;
	float cycle_min;
	float duty_ratio_max;
	float duty_ratio_min;

	float body_x;
	float body_y;
	float body_z;
	float roll;
	float pitch;
	float yaw;

	float zero_x;
	float zero_y;
} Range = {200.0f, 100.0f, 100.0f, pi / 12, 2.0f, 0.6f, 0.5f, 0.5f, 100.0f, 100.0f, 150.0f, pi / 15, pi / 15, pi / 15, 100.0f, 100.0f};

XBOX_t Xbox;
GYRO_t Gyro;

void InitTask(void)
{
	XBOX_Init(&Xbox);
	GYRO_Init(&Gyro, 115200);
	GYRO_ConfigLog(&Gyro, GYRO_LOG_ENABLE, GYRO_LOG_ACC | GYRO_LOG_ACC_FT | GYRO_LOG_ANGLE | GYRO_LOG_ANGLE_FT);
	GYRO_ConfigFilt(&Gyro, GYRO_DATATYPE_ACC, 0x07, 100);
	GYRO_ConfigFilt(&Gyro, GYRO_DATATYPE_ANGLE, 0x07, 100);

	m_pos = cmat_malloc(3, 4);
	m_rad = cmat_malloc(3, 4);

	RC_Init_Robot(&QuadrupedRobot, "elbow-elbow", 72, 300, 230, 150, 500);
	RC_Init_MovPara(&QuadrupedRobot, "trot", 0.7f, 0.01f, 0.55f,
					0.0f, 0.0f, 0.0f, 0.0f,
					0.0f, 0.0f, 425.0f, 0.0f, 0.0f, 0.0f,
					0.0f, 0.0f, 294.0f, 480.0f, 0.0f, 0.0f);

	PID_Regular_Reset(&PID_yaw, 0.02f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f);
}

void TimerTask(void)
{
	Phase += QuadrupedRobot.Move.interval / QuadrupedRobot.Move.cycle;
	if (Phase >= 1.0f)
		Phase -= 1.0f;
}

void SendTask(void)
{
	float send_buffer[12];

	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 3; j++)
			send_buffer[i * 3 + j] = cmat_get(m_rad, j, i);

	serialSendFrameHead(setting.fd_uart, FRAME_HEAD);
	serialSendFloatArr(setting.fd_uart, 12, send_buffer, 1);
}

/* void YawLockedTask(void)
{
	static int prev_remote_rs;

	if (prev_remote_rs == 0 && RemoteData.Coordinate == 1)
	{
		printf("AutoYaw\n");

		Locked_yaw = GetGyro_FilterYaw();

		printf("Yaw axis locked. Angle is %f\n", Locked_yaw);

		PID_yaw.Ref = Locked_yaw;
	}

	PID_yaw.Feedback = GetGyro_FilterYaw();

	printf("Yaw -ref:%f -fdb:%f\n", PID_yaw.Ref, PID_yaw.Feedback);

	PID_Regular_Cacl(&PID_yaw);

	prev_remote_rs = RemoteData.Coordinate;
}*/

void KeyPressTask(void)
{
	static int mode = 0;

	XBOX_Edge(&Xbox);
	XBOX_DispAll(&Xbox);

	if (Xbox.xx_edge == -1)
		QuadrupedRobot.Pose.body_z += 10;
	if (Xbox.xx_edge == 1)
		QuadrupedRobot.Pose.body_z -= 10;
	if (Xbox.yy_edge == -1)
		QuadrupedRobot.Move.span_z += 10;
	if (Xbox.yy_edge == 1)
		QuadrupedRobot.Move.span_z -= 10;

	if (QuadrupedRobot.Move.span_z > Range.span_z)
		QuadrupedRobot.Move.span_z = Range.span_z;
	if (QuadrupedRobot.Move.span_z < 0)
		QuadrupedRobot.Move.span_z = 0;

	if (Xbox.a_edge == 1)
		mode = 0;
	if (Xbox.b_edge == 1)
		mode = 1;

	switch (mode)
	{
	case 0:
		QuadrupedRobot.Move.span_x = -Xbox.ly_f * Range.span_x;
		QuadrupedRobot.Move.span_y = -Xbox.lx_f * Range.span_y;
		QuadrupedRobot.Move.span_w = (Xbox.lt_f - Xbox.rt_f) * Range.span_w;
		RC_ThreeDivided_Optimization(&QuadrupedRobot);
		break;
	case 1:
		QuadrupedRobot.Move.span_x = 0;
		QuadrupedRobot.Move.span_y = 0;
		QuadrupedRobot.Move.span_z = 0;

		QuadrupedRobot.Pose.body_x = Xbox.ly_f * Range.body_x;
		QuadrupedRobot.Pose.body_y = Xbox.lx_f * Range.body_y;
		QuadrupedRobot.Pose.body_ya = Xbox.rx_f * Range.yaw;
		QuadrupedRobot.Pose.body_pi = Xbox.ry_f * Range.pitch;
		break;

	default:
		break;
	}
}

void InterruptTask(void)
{
	static int prev_remote_ls = 0;
	static int locked_body_z = 350;

	//if (prev_remote_ls != RemoteData.Gait)
	//	locked_body_z = QuadrupedRobot.Pose.body_z;

	TimerTask();

	//YawLockedTask();

	KeyPressTask();

	RC_Update_ZeroPara(&QuadrupedRobot, QuadrupedRobot.Zero.width, QuadrupedRobot.Zero.length, QuadrupedRobot.Zero.centre_x, QuadrupedRobot.Zero.centre_y);
	RC_Update_BodyPose(&QuadrupedRobot, QuadrupedRobot.Pose.body_x, QuadrupedRobot.Pose.body_y, QuadrupedRobot.Pose.body_z, 0, QuadrupedRobot.Pose.body_pi, QuadrupedRobot.Pose.body_ya);
	RC_Update_PosPose(&QuadrupedRobot, 0, 0);

	RC_Calc_FootTraj(&QuadrupedRobot, Phase, m_pos);
	//cmat_display(m_pos);
	RC_InvKine(&QuadrupedRobot, m_pos, m_rad);
	//cmat_display(m_rad);
	RC_AngleCorrect(&QuadrupedRobot, m_rad);
	//cmat_display(m_rad);

	//prev_remote_ls = RemoteData.Gait;

	SendTask();
}

void DisplayTask(void)
{
	//DispRemoteData();
	//DispGyroData();
	//DispFootGroundingData(FootGrounding);
}

/* void RevTask(void)
{
	serial_frame_t sFrame;
	if (!serialRevFrame(&sFrame, setting.fd_uart, FRAME_HEAD))
	{
		switch (sFrame.type)
		{
		case FRAMETYPE_FOOTGROUNDING:
			AnalysisFootGroundingData(&sFrame);
			break;

		case FRAMETYPE_REMOTE:
			AnalysisRemoteData(&sFrame);
			break;

		case FRAMETYPE_GYROSCOPE:
			//AnalysisGyroData(&sFrame);
			break;

		default:
			PRINTF_WARNING("Unknown frame type.");
			break;
		}
		free(sFrame.pdata);
	}
	ReadGyro_RPY();
}*/

void LowPriorityTask(void)
{

	XBOX_Read(&Xbox);
	XBOX_Normal(&Xbox);

	GYRO_Read(&Gyro, GYRO_DATATYPE_ACC);
	GYRO_Read(&Gyro, GYRO_DATATYPE_ANGLE);

	/* switch (GYRO_Read(&Gyro))
	{
	case GYRO_REV_ACC:
		printf("r_a_x:%f,r_a_y:%f,r_a_z:%f,f_a_x:%f,f_a_y:%f,f_a_z:%f\n", Gyro.Acc.acc_x, Gyro.Acc.acc_y, Gyro.Acc.acc_z,
			   GYRO_GetFilt(&Gyro, GYRO_DATATYPE_ACC, GYRO_DATASUBTYPE_X),
			   GYRO_GetFilt(&Gyro, GYRO_DATATYPE_ACC, GYRO_DATASUBTYPE_Y),
			   GYRO_GetFilt(&Gyro, GYRO_DATATYPE_ACC, GYRO_DATASUBTYPE_Z));
		break;

	case GYRO_REV_ANGLE:
		printf("r_ro:%f,r_pi:%f,r_ya:%f,f_ro:%f,f_pi:%f,f_ya:%f\n", Gyro.Angle.roll, Gyro.Angle.pitch, Gyro.Angle.yaw,
			   GYRO_GetFilt(&Gyro, GYRO_DATATYPE_ANGLE, GYRO_DATASUBTYPE_X),
			   GYRO_GetFilt(&Gyro, GYRO_DATATYPE_ANGLE, GYRO_DATASUBTYPE_Y),
			   GYRO_GetFilt(&Gyro, GYRO_DATATYPE_ANGLE, GYRO_DATASUBTYPE_Z));
		break;

	default:
		break;
	}*/

	GYRO_RecLog(&Gyro);
	//DisplayTask();
	//RevTask();
}

void ExitTask(void)
{
	GYRO_Close(&Gyro);
}
