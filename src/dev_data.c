#include "dev_data.h"

/********************FootGrounding*******************/

uint8_t FootGrounding;

void AnalysisFootGroundingData(serial_frame_t *pFrame)
{
	FootGrounding = *pFrame->pdata;
}

void DispFootGroundingData(FootStatus_t FootGrounding)
{
	if (FootGrounding & 0x01)
		printf("[LF]");
	else
		printf(" LF ");

	if (FootGrounding & 0x02)
		printf("[RH]");
	else
		printf(" RH ");

	if (FootGrounding & 0x04)
		printf("[RF]");
	else
		printf(" RF ");

	if (FootGrounding & 0x08)
		printf("[LH]");
	else
		printf(" LH ");

	printf("\n");
}

/********************Remote*******************/

Remote_t RemoteData;

void AnalysisRemoteData(serial_frame_t *pFrame)
{
	RemoteData.Joystick_LX = (int8_t)pFrame->pdata[0];
	RemoteData.Joystick_LY = (int8_t)pFrame->pdata[1];
	RemoteData.Joystick_RX = (int8_t)pFrame->pdata[2];
	RemoteData.Joystick_RY = (int8_t)pFrame->pdata[3];
	RemoteData.Gait = pFrame->pdata[4];
	RemoteData.Coordinate = pFrame->pdata[5];
	RemoteData.Dial = pFrame->pdata[6];

	RemoteData.LX_Factor = RemoteData.Joystick_LX / 128.0f;
	RemoteData.LY_Factor = RemoteData.Joystick_LY / 128.0f;
	RemoteData.RX_Factor = RemoteData.Joystick_RX / 128.0f;
	RemoteData.RY_Factor = RemoteData.Joystick_RY / 128.0f;
	RemoteData.Dial_Factor = RemoteData.Dial / 256.0f;
}

void DispRemoteData(void)
{
	printf("Remote Data:\n");
	printf("LX:%d\t", RemoteData.Joystick_LX);
	printf("LY:%d\t", RemoteData.Joystick_LY);
	printf("RX:%d\t", RemoteData.Joystick_RX);
	printf("RY:%d\t", RemoteData.Joystick_RY);
	printf("\n");
	printf("LS:%d\t", RemoteData.Gait);
	printf("RS:%d\t", RemoteData.Coordinate);
	printf("VA:%d\t", RemoteData.Dial);
	printf("\n");
}

/********************Gyroscope*******************/

/*Gyro_t GyroData;

void ConfigGyroFilter(int window_size)
{
	SildingAvrgFilter_Init(GyroData.Pitch_Filter, window_size);
	SildingAvrgFilter_Init(GyroData.Roll_Filter, window_size);
	SildingAvrgFilter_Init(GyroData.Yaw_Filter, window_size);
	SildingAvrgFilter_Init(GyroData.X_Filter, window_size);
	SildingAvrgFilter_Init(GyroData.Y_Filter, window_size);
	SildingAvrgFilter_Init(GyroData.Z_Filter, window_size);
}

void CloseGyroFilter(void)
{
	SildingAvrgFilter_Kill(GyroData.Pitch_Filter);
	SildingAvrgFilter_Kill(GyroData.Roll_Filter);
	SildingAvrgFilter_Kill(GyroData.Yaw_Filter);
	SildingAvrgFilter_Kill(GyroData.X_Filter);
	SildingAvrgFilter_Kill(GyroData.Y_Filter);
	SildingAvrgFilter_Kill(GyroData.Z_Filter);
}

void AnalysisGyroData(serial_frame_t *pFrame)
{
	float_uint8_t gyro_temp_arr[6];

	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			gyro_temp_arr[i].u8[j] = pFrame->pdata[i * 4 + j];
		}
	}

	GyroData.Pitch = gyro_temp_arr[0].fl - 2.43f;
	GyroData.Roll = gyro_temp_arr[1].fl;
	GyroData.Yaw = gyro_temp_arr[2].fl;
	GyroData.X = gyro_temp_arr[3].fl;
	GyroData.Y = gyro_temp_arr[4].fl;
	GyroData.Z = gyro_temp_arr[5].fl;

	if (GyroData.en_filter == FILTER_ON) //使用滤波器
	{
		GyroData.Pitch_ = SildingAvrgFilter_Calc(GyroData.Pitch_Filter, GyroData.Pitch);
		GyroData.Roll_ = SildingAvrgFilter_Calc(GyroData.Roll_Filter, GyroData.Roll);
		GyroData.Yaw_ = SildingAvrgFilter_Calc(GyroData.Yaw_Filter, GyroData.Yaw);
		GyroData.X_ = SildingAvrgFilter_Calc(GyroData.X_Filter, GyroData.X);
		GyroData.Y_ = SildingAvrgFilter_Calc(GyroData.Y_Filter, GyroData.Y);
		GyroData.Z_ = SildingAvrgFilter_Calc(GyroData.Z_Filter, GyroData.Z);
	}
}

void DispGyroData(void)
{
	printf("Gyro Data:\n");
	printf("Pitch:%f\t", GyroData.Pitch);
	printf("Roll:%f\t", GyroData.Roll);
	printf("Yaw:%f\t", GyroData.Yaw);
	printf("X:%f\t", GyroData.X);
	printf("Y:%f\t", GyroData.Y);
	printf("Z:%f\t", GyroData.Z);
	printf("\n");
}*/

GYRO_t WT901C;

queue_t *Window_yaw;
queue_t *Window_pitch;
queue_t *Window_roll;

void InitGyro(void)
{
	GYRO_Init(&WT901C, 115200);

	Window_yaw = SildingAvrgFilter_Init(100);
	Window_pitch = SildingAvrgFilter_Init(100);
	Window_roll = SildingAvrgFilter_Init(100);
}

float Filter_RPY[3];

void ReadGyro_RPY(void)
{
	if (GYRO_Read(&WT901C) == GYRO_REV_ANGLE)
	{
		Filter_RPY[0] = SildingAvrgFilter_Calc(Window_roll, WT901C.Angle.roll);
		Filter_RPY[1] = SildingAvrgFilter_Calc(Window_pitch, WT901C.Angle.pitch);
		Filter_RPY[2] = SildingAvrgFilter_Calc(Window_yaw, WT901C.Angle.yaw);

		//printf("Y:\t%f\t%f\n", WT901C.Angle.yaw, SildingAvrgFilter_Calc(Window_yaw, WT901C.Angle.yaw));
		//printf("P:\t%f\t%f\n", WT901C.Angle.pitch, SildingAvrgFilter_Calc(Window_pitch, WT901C.Angle.pitch));
		//printf("R:\t%f\t%f\n", WT901C.Angle.roll, SildingAvrgFilter_Calc(Window_roll, WT901C.Angle.roll));
	}
}

float GetGyro_FilterRoll(void)
{
	return Filter_RPY[0];
}

float GetGyro_FilterPitch(void)
{
	return Filter_RPY[1];
}

float GetGyro_FilterYaw(void)
{
	return Filter_RPY[2];
}
/*
void CreateGyroLogFile(void)
{
	char str_data[64];
	char str_cwd[64];
	time_t timer = time(NULL);

	strftime(str_data, sizeof(str_data), "%Y-%m-%d[%H:%M:%S][GYRO]", localtime(&timer));
	strcat(GyroData.Log.filename, str_data);
	strcat(GyroData.Log.filename, ".txt");
	GyroData.Log.fd = fopen(GyroData.Log.filename, "w");

	if (GyroData.Log.fd == NULL)
		printf("Failed to create gyro log file.\n");
}

void WriteGyroLogFile(void)
{
	fprintf(GyroData.Log.fd, "%f\t%f\t%f\t%f\t%f\t%f\n",
			GyroData.Pitch, GyroData.Roll, GyroData.Yaw,
			GyroData.X, GyroData.Y, GyroData.Z);
}

void SaveGyroLogFile(void)
{
	if (!fclose(GyroData.Log.fd))
		printf("Gyro log file saved. Filename is %s.\n", GyroData.Log.filename);
}

void ConfigGyro(Gyro_EnFilter_t en_filter, int window_size, Gyro_EnLog_t en_log)
{
	GyroData.en_filter = en_filter;
	GyroData.Log.enable = en_log;

	if (GyroData.en_filter == FILTER_ON)
		ConfigGyroFilter(window_size);

	if (GyroData.Log.enable == LOG_ON)
		CreateGyroLogFile();
}

void CloseGyro(void)
{
	CloseGyroFilter();
	SaveGyroLogFile();
}
*/
