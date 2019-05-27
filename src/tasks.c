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

void InitTask(void)
{
	m_pos = cmat_malloc(3, 4);
	m_rad = cmat_malloc(3, 4);

	RC_Init_Robot(&QuadrupedRobot, "elbow-elbow", 70, 300, 300, 400, 600);
	RC_Init_MovPara(&QuadrupedRobot, "trot", 1.0f, 0.01f, 0.5f,
					0.0f, 0.0f, 100.0f, 0.0f,
					0.0f, 0.0f, 400.0f, 0.0f, 0.0f, 0.0f,
					0.0f, 0.0f, 420.0f, 600.0f, 0.0f, 0.0f);
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

	RC_Update_ZeroPara(&QuadrupedRobot, 400, 600, 0, 0);
	RC_Update_BodyPose(&QuadrupedRobot, 0, 0, 400, 0, 0, 0);
	RC_Update_PosPose(&QuadrupedRobot, 0, 0);

	RC_Calc_FootTraj(&QuadrupedRobot, Phase, m_pos);
	//cmat_display(m_pos);
	RC_InvKine(&QuadrupedRobot, m_pos, m_rad);
	//cmat_display(m_rad);
	RC_AngleCorrect(&QuadrupedRobot, m_rad);
	cmat_display(m_rad);

	SendTask();
}

void DisplayTask(void)
{
	DispRemoteData();
	DispGyroData();
	DispFootGroundingData(FootGrounding);
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
			AnalysisGyroData(&sFrame, 1);
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
