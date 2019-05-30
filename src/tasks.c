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
} Range = {150.0f, 100.0f, 100.0f, pi / 16, 2.0f, 0.6f, 0.5f, 0.5f, 100.0f, 100.0f, 100.0f, pi / 15, pi / 15, pi / 15, 100.0f, 100.0f};

void InitTask(void)
{
	m_pos = cmat_malloc(3, 4);
	m_rad = cmat_malloc(3, 4);

	RC_Init_Robot(&QuadrupedRobot, "elbow-elbow", 70, 300, 250, 260, 570);
	RC_Init_MovPara(&QuadrupedRobot, "trot", 0.8f, 0.01f, 0.5f,
					0.0f, 0.0f, 100.0f, 0.0f,
					0.0f, 0.0f, 450.0f, 0.0f, 0.0f, 0.0f,
					0.0f, 0.0f, 400.0f, 570.0f, 0.0f, 0.0f);
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

void InterruptTask(void)
{
	TimerTask();

	switch (RemoteData.Gait)
	{
	case 0:
		printf("Gait:0\n");
		QuadrupedRobot.Move.span_x = RemoteData.LY_Factor * Range.span_x;
		QuadrupedRobot.Move.span_y = RemoteData.LX_Factor * Range.span_y;
		QuadrupedRobot.Move.span_w = RemoteData.RX_Factor * Range.span_w;
		QuadrupedRobot.Move.span_z = RemoteData.Dial_Factor * Range.span_z;
		break;
	case 1:
		printf("Gait:1\n");
		QuadrupedRobot.Pose.body_x = RemoteData.LY_Factor * Range.body_x;
		QuadrupedRobot.Pose.body_y = RemoteData.LX_Factor * Range.body_y;
		QuadrupedRobot.Pose.body_ya = RemoteData.RX_Factor * Range.yaw;
		QuadrupedRobot.Pose.body_pi = RemoteData.RY_Factor * Range.pitch;
		QuadrupedRobot.Pose.body_z = 350 + RemoteData.Dial_Factor * Range.body_z;
		break;
	case 2:
		QuadrupedRobot.Zero.centre_x = RemoteData.RY_Factor * Range.zero_x;
		QuadrupedRobot.Zero.centre_y = RemoteData.RX_Factor * Range.zero_y;
		break;
	}

	RC_Update_ZeroPara(&QuadrupedRobot, 400, 700, QuadrupedRobot.Zero.centre_x, QuadrupedRobot.Zero.centre_y);
	RC_Update_BodyPose(&QuadrupedRobot, QuadrupedRobot.Pose.body_x, QuadrupedRobot.Pose.body_y, QuadrupedRobot.Pose.body_z, 0, QuadrupedRobot.Pose.body_pi, QuadrupedRobot.Pose.body_ya);
	RC_Update_PosPose(&QuadrupedRobot, 0, 0);

	RC_Calc_FootTraj(&QuadrupedRobot, Phase, m_pos);
	//cmat_display(m_pos);
	RC_InvKine(&QuadrupedRobot, m_pos, m_rad);
	cmat_display(m_rad);
	RC_AngleCorrect(&QuadrupedRobot, m_rad);
	//cmat_display(m_rad);

	SendTask();
}

void DisplayTask(void)
{
	//DispRemoteData();
	DispGyroData();
	//DispFootGroundingData(FootGrounding);
}

void RevTask(void)
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
			AnalysisGyroData(&sFrame);
			break;

		default:
			PRINTF_WARNING("Unknown frame type.");
			break;
		}
		free(sFrame.pdata);
	}
}

void LowPriorityTask(void)
{
	//DisplayTask();
	RevTask();
}
