#include "gyro.h"

void GYRO_ConfigLog(GYRO_t *gyro, int en_log, uint8_t type)
{
	if (en_log == GYRO_LOG_ENABLE)
	{
		char str_data[64];
		time_t timer = time(NULL);

		strftime(str_data, sizeof(str_data), "%Y-%m-%d[%H:%M:%S][GYRO]", localtime(&timer));

		strcat(gyro->log_filename, str_data);
		strcat(gyro->log_filename, ".txt");

		gyro->log_fp = fopen(gyro->log_filename, "w");

		if (gyro->log_fp == NULL)
			printf("Failed to create gyro log file.\n");

		gyro->log_ctrl = type;

		if (gyro->log_ctrl & GYRO_LOG_ACC)
			fprintf(gyro->log_fp, "acce_x\tacce_y\tacce_z\t");
		if (gyro->log_ctrl & GYRO_LOG_ANGLE)
			fprintf(gyro->log_fp, "ang_ro\tang_pi\tang_ya\t");
		if (gyro->log_ctrl & GYRO_LOG_ANGVEL)
			fprintf(gyro->log_fp, "omeg_x\tomeg_y\tomeg_z\t");
		if (gyro->log_ctrl & GYRO_LOG_MANGE)
			fprintf(gyro->log_fp, "mang_x\tmang_y\tmang_z\t");
		if (gyro->log_ctrl & GYRO_LOG_ACC_FT)
			fprintf(gyro->log_fp, "facce_x\tfacce_y\tfacce_z\t");
		if (gyro->log_ctrl & GYRO_LOG_ANGLE_FT)
			fprintf(gyro->log_fp, "fang_ro\tfang_pi\tfang_ya\t");
		if (gyro->log_ctrl & GYRO_LOG_ANGVEL_FT)
			fprintf(gyro->log_fp, "fomeg_x\tfomeg_y\tfomeg_z\t");
		if (gyro->log_ctrl & GYRO_LOG_MANGE_FT)
			fprintf(gyro->log_fp, "fmang_x\tfmang_x\tfmang_x\t");

		fprintf(gyro->log_fp, "\n");
	}
	else
	{
		gyro->log_fp = NULL;
	}
}

void GYRO_RecLog(GYRO_t *gyro)
{
	if (gyro->log_ctrl & GYRO_LOG_ACC)
		fprintf(gyro->log_fp, "%7.3f\t%7.3f\t%7.3f\t", gyro->Acc.acc_x, gyro->Acc.acc_y, gyro->Acc.acc_z);
	if (gyro->log_ctrl & GYRO_LOG_ANGLE)
		fprintf(gyro->log_fp, "%7.3f\t%7.3f\t%7.2f\t", gyro->Angle.roll, gyro->Angle.pitch, gyro->Angle.yaw);
	if (gyro->log_ctrl & GYRO_LOG_ANGVEL)
		fprintf(gyro->log_fp, "%7.3f\t%7.3f\t%7.3f\t", gyro->AngVel.w_x, gyro->AngVel.w_y, gyro->AngVel.w_z);
	if (gyro->log_ctrl & GYRO_LOG_MANGE)
		fprintf(gyro->log_fp, "%7.3f\t%7.3f\t%7.3f\t", gyro->Magne.h_x, gyro->Magne.h_y, gyro->Magne.h_z);
	if (gyro->log_ctrl & GYRO_LOG_ACC_FT)
		fprintf(gyro->log_fp, "%7.3f\t%7.3f\t%7.3f\t", gyro->Acc_ft.acc_x, gyro->Acc_ft.acc_y, gyro->Acc_ft.acc_z);
	if (gyro->log_ctrl & GYRO_LOG_ANGLE_FT)
		fprintf(gyro->log_fp, "%7.3f\t%7.3f\t%7.2f\t", gyro->Angle_ft.roll, gyro->Angle_ft.pitch, gyro->Angle_ft.yaw);
	if (gyro->log_ctrl & GYRO_LOG_ANGVEL_FT)
		fprintf(gyro->log_fp, "%7.3f\t%7.3f\t%7.3f\t", gyro->AngVel_ft.w_x, gyro->AngVel_ft.w_y, gyro->AngVel_ft.w_z);
	if (gyro->log_ctrl & GYRO_LOG_MANGE_FT)
		fprintf(gyro->log_fp, "%7.3f\t%7.3f\t%7.3f\t", gyro->Magne_ft.h_x, gyro->Magne_ft.h_y, gyro->Magne_ft.h_z);

	fprintf(gyro->log_fp, "\n");
}

int GYRO_Init(GYRO_t *gyro, int baud)
{
	int file_index;
	int res_val;

	for (file_index = 0; file_index < 4; file_index++)
	{
		sprintf(gyro->filename, "/dev/ttyUSB%d", file_index);
		if (access(gyro->filename, F_OK) != -1)
			gyro->fd = serialOpen(gyro->filename, baud);
		else
			continue;

		if (gyro->fd == -1)
		{
			printf("Dev File: %s\n", gyro->filename);
			perror("Failed to open.\n");
		}
		else
		{
			printf("Success to open %s.\n", gyro->filename);
			printf("Baud rate: %d.\tFile descriptor: %d.\n", baud, gyro->fd);
			printf("Wating to confirm the connection...\n");
			for (int i = 0; i < 5; i++)
			{
				res_val = GYRO_Read(gyro, GYRO_DATATYPE_ANYONE);
				switch (res_val)
				{
				case GYRO_REV_ACC:
				case GYRO_REV_ANGLE:
				case GYRO_REV_ANGVEL:
				case GYRO_REV_MANGE:
				case GYRO_REV_TIME:
					printf("Gyro connected!\n");
					return 0;
					break;

				default:
					break;
				}
				if (i == 5)
				{
					printf("Gyro response overtime.\n");
				}
			}
		}
	}
	printf("Failed to connect with gyro.");
	return -1;
}

void GYRO_CalcFilt(GYRO_t *gyro, GYRO_DataType_t type)
{
	switch (type)
	{
	case GYRO_DATATYPE_ACC:
		if (gyro->Acc_ft.filter_ctrl & GYRO_DATASUBTYPE_X)
			gyro->Acc_ft.acc_x = SildingAvrgFilter_Calc(gyro->Acc_ft.window_acc_x, gyro->Acc.acc_x);
		if (gyro->Acc_ft.filter_ctrl & GYRO_DATASUBTYPE_Y)
			gyro->Acc_ft.acc_y = SildingAvrgFilter_Calc(gyro->Acc_ft.window_acc_y, gyro->Acc.acc_y);
		if (gyro->Acc_ft.filter_ctrl & GYRO_DATASUBTYPE_Z)
			gyro->Acc_ft.acc_z = SildingAvrgFilter_Calc(gyro->Acc_ft.window_acc_z, gyro->Acc.acc_z);

		break;

	case GYRO_DATATYPE_ANGLE:
		if (gyro->Angle_ft.filter_ctrl & GYRO_DATASUBTYPE_X)
			gyro->Angle_ft.roll = SildingAvrgFilter_Calc(gyro->Angle_ft.window_roll, gyro->Angle.roll);
		if (gyro->Angle_ft.filter_ctrl & GYRO_DATASUBTYPE_Y)
			gyro->Angle_ft.pitch = SildingAvrgFilter_Calc(gyro->Angle_ft.window_pitch, gyro->Angle.pitch);
		if (gyro->Angle_ft.filter_ctrl & GYRO_DATASUBTYPE_Z)
			gyro->Angle_ft.yaw = SildingAvrgFilter_Calc(gyro->Angle_ft.window_yaw, gyro->Angle.yaw);

		break;

	case GYRO_DATATYPE_ANGVEL:
		if (gyro->AngVel_ft.filter_ctrl & GYRO_DATASUBTYPE_X)
			gyro->AngVel_ft.w_x = SildingAvrgFilter_Calc(gyro->AngVel_ft.window_w_x, gyro->AngVel.w_x);
		if (gyro->AngVel_ft.filter_ctrl & GYRO_DATASUBTYPE_Y)
			gyro->AngVel_ft.w_y = SildingAvrgFilter_Calc(gyro->AngVel_ft.window_w_y, gyro->AngVel.w_y);
		if (gyro->AngVel_ft.filter_ctrl & GYRO_DATASUBTYPE_Z)
			gyro->AngVel_ft.w_z = SildingAvrgFilter_Calc(gyro->AngVel_ft.window_w_z, gyro->AngVel.w_z);

		break;

	case GYRO_DATATYPE_MAGNE:
		if (gyro->Magne_ft.filter_ctrl & GYRO_DATASUBTYPE_X)
			gyro->Magne_ft.h_x = SildingAvrgFilter_Calc(gyro->Magne_ft.window_h_x, gyro->Magne.h_x);
		if (gyro->Magne_ft.filter_ctrl & GYRO_DATASUBTYPE_Y)
			gyro->Magne_ft.h_y = SildingAvrgFilter_Calc(gyro->Magne_ft.window_h_y, gyro->Magne.h_y);
		if (gyro->Magne_ft.filter_ctrl & GYRO_DATASUBTYPE_Z)
			gyro->Magne_ft.h_z = SildingAvrgFilter_Calc(gyro->Magne_ft.window_h_z, gyro->Magne.h_z);

		break;

	default:
		break;
	}
}

int GYRO_Read(GYRO_t *gyro, GYRO_DataType_t data_type)
{
	static uint8_t byte = 0;
	uint16_t sum = 0, sum_ = 0;

	uint8_t data[9];

	for (int i = 0; i < 128; i++)
	{
		if ((serialDataAvail(gyro->fd)) > 0)
		{
			byte = serialGetchar(gyro->fd);
			//printf("byte: %x\n", byte);
			if (byte == GYRO_FRAME_HEAD)
			{
				byte = serialGetchar(gyro->fd);

				if (byte != data_type && data_type != GYRO_DATATYPE_ANYONE)
					continue;

				switch (byte)
				{
				case Time:
				case Acc:
				case AngVel:
				case Angle:
				case Magne:
					for (int i = 0; i < 8; i++)
					{
						data[i] = serialGetchar(gyro->fd);
						sum += data[i];
					}
					sum += byte;
					sum += GYRO_FRAME_HEAD;

					sum_ = serialGetchar(gyro->fd);

					//printf("sum:%x sum_:%x\n", (uint8_t)sum, sum_);

					if (sum_ == (uint8_t)sum)
					{
						switch (byte)
						{
						case Time:
							gyro->Time.year = data[0];
							gyro->Time.month = data[1];
							gyro->Time.day = data[2];
							gyro->Time.hour = data[3];
							gyro->Time.minute = data[4];
							gyro->Time.second = data[5];
							gyro->Time.m_second = ((data[7] << 8) | data[6]);
							break;
						case Acc:
							gyro->Acc.acc_x = (int16_t)((data[1] << 8) | data[0]) / 32768.0f * 16 * 9.8;
							gyro->Acc.acc_y = (int16_t)((data[3] << 8) | data[2]) / 32768.0f * 16 * 9.8;
							gyro->Acc.acc_z = (int16_t)((data[5] << 8) | data[4]) / 32768.0f * 16 * 9.8;
							gyro->Acc.temp = (int16_t)((data[7] << 8) | data[6]) / 100.0f;
							GYRO_CalcFilt(gyro, GYRO_DATATYPE_ACC);
							break;
						case AngVel:
							gyro->AngVel.w_x = (int16_t)((data[1] << 8) | data[0]) / 32768.0f * 2000;
							gyro->AngVel.w_y = (int16_t)((data[3] << 8) | data[2]) / 32768.0f * 2000;
							gyro->AngVel.w_z = (int16_t)((data[5] << 8) | data[4]) / 32768.0f * 2000;
							gyro->AngVel.temp = (int16_t)((data[7] << 8) | data[6]) / 100.0f;
							GYRO_CalcFilt(gyro, GYRO_DATATYPE_ANGVEL);
							break;
						case Angle:
							gyro->Angle.roll = (int16_t)((data[1] << 8) | data[0]) / 32768.0f * 180;
							gyro->Angle.pitch = (int16_t)((data[3] << 8) | data[2]) / 32768.0f * 180;
							gyro->Angle.yaw = (int16_t)((data[5] << 8) | data[4]) / 32768.0f * 180;
							gyro->Angle.temp = (int16_t)((data[7] << 8) | data[6]) / 100.0f;
							GYRO_CalcFilt(gyro, GYRO_DATATYPE_ANGLE);
							break;
						case Magne:
							gyro->Magne.h_x = (int16_t)(data[1] << 8) | data[0];
							gyro->Magne.h_y = (int16_t)(data[3] << 8) | data[2];
							gyro->Magne.h_z = (int16_t)(data[5] << 8) | data[4];
							gyro->Magne.temp = (int16_t)((data[7] << 8) | data[6]) / 100.0f;
							GYRO_CalcFilt(gyro, GYRO_DATATYPE_MAGNE);
							break;
						}
						//if (serialDataAvail(gyro->fd) >= 128)
						//	serialFlush(gyro->fd);
						switch (byte)
						{
						case Time:
							return GYRO_REV_TIME;
						case Acc:
							return GYRO_REV_ACC;
						case AngVel:
							return GYRO_REV_ANGVEL;
						case Angle:
							return GYRO_REV_ANGLE;
						case Magne:
							return GYRO_REV_MANGE;
						}
						break;
					}
					else
					{
						return GYRO_ERROR;
					}
				default:
					break;
				}
			}
		}
	}
	return GYRO_NODATA;
}

void GYRO_ConfigFilt(GYRO_t *gyro, GYRO_DataType_t type, uint8_t val, uint8_t window_size)
{
	switch (type)
	{
	case GYRO_DATATYPE_ACC:
		if ((gyro->Acc_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_X) && (gyro->Acc_ft.filter_ctrl & GYRO_DATASUBTYPE_X))
			SildingAvrgFilter_Kill(gyro->Acc_ft.window_acc_x);
		if ((gyro->Acc_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_Y) && (gyro->Acc_ft.filter_ctrl & GYRO_DATASUBTYPE_Y))
			SildingAvrgFilter_Kill(gyro->Acc_ft.window_acc_y);
		if ((gyro->Acc_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_Z) && (gyro->Acc_ft.filter_ctrl & GYRO_DATASUBTYPE_Z))
			SildingAvrgFilter_Kill(gyro->Acc_ft.window_acc_z);

		if ((gyro->Acc_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_X) && (val & GYRO_DATASUBTYPE_X))
			gyro->Acc_ft.window_acc_x = SildingAvrgFilter_Init(window_size);
		if ((gyro->Acc_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_Y) && (val & GYRO_DATASUBTYPE_Y))
			gyro->Acc_ft.window_acc_y = SildingAvrgFilter_Init(window_size);
		if ((gyro->Acc_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_Z) && (val & GYRO_DATASUBTYPE_Z))
			gyro->Acc_ft.window_acc_z = SildingAvrgFilter_Init(window_size);

		gyro->Acc_ft.filter_ctrl = val;

		break;

	case GYRO_DATATYPE_ANGLE:
		if ((gyro->Angle_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_X) && (gyro->Angle_ft.filter_ctrl & GYRO_DATASUBTYPE_X))
			SildingAvrgFilter_Kill(gyro->Angle_ft.window_roll);
		if ((gyro->Angle_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_Y) && (gyro->Angle_ft.filter_ctrl & GYRO_DATASUBTYPE_Y))
			SildingAvrgFilter_Kill(gyro->Angle_ft.window_pitch);
		if ((gyro->Angle_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_Z) && (gyro->Angle_ft.filter_ctrl & GYRO_DATASUBTYPE_Z))
			SildingAvrgFilter_Kill(gyro->Angle_ft.window_yaw);

		if ((gyro->Angle_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_X) && (val & GYRO_DATASUBTYPE_X))
			gyro->Angle_ft.window_roll = SildingAvrgFilter_Init(window_size);
		if ((gyro->Angle_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_Y) && (val & GYRO_DATASUBTYPE_Y))
			gyro->Angle_ft.window_pitch = SildingAvrgFilter_Init(window_size);
		if ((gyro->Angle_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_Z) && (val & GYRO_DATASUBTYPE_Z))
			gyro->Angle_ft.window_yaw = SildingAvrgFilter_Init(window_size);

		gyro->Angle_ft.filter_ctrl = val;
		break;

	case GYRO_DATATYPE_ANGVEL:
		if ((gyro->AngVel_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_X) && (gyro->AngVel_ft.filter_ctrl & GYRO_DATASUBTYPE_X))
			SildingAvrgFilter_Kill(gyro->AngVel_ft.window_w_x);
		if ((gyro->AngVel_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_Y) && (gyro->AngVel_ft.filter_ctrl & GYRO_DATASUBTYPE_Y))
			SildingAvrgFilter_Kill(gyro->AngVel_ft.window_w_y);
		if ((gyro->AngVel_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_Z) && (gyro->AngVel_ft.filter_ctrl & GYRO_DATASUBTYPE_Z))
			SildingAvrgFilter_Kill(gyro->AngVel_ft.window_w_z);

		if ((gyro->AngVel_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_X) && (val & GYRO_DATASUBTYPE_X))
			gyro->AngVel_ft.window_w_x = SildingAvrgFilter_Init(window_size);
		if ((gyro->AngVel_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_Y) && (val & GYRO_DATASUBTYPE_Y))
			gyro->AngVel_ft.window_w_y = SildingAvrgFilter_Init(window_size);
		if ((gyro->AngVel_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_Z) && (val & GYRO_DATASUBTYPE_Z))
			gyro->AngVel_ft.window_w_z = SildingAvrgFilter_Init(window_size);

		gyro->AngVel_ft.filter_ctrl = val;
		break;

	case GYRO_DATATYPE_MAGNE:
		if ((gyro->Magne_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_X) && (gyro->Magne_ft.filter_ctrl & GYRO_DATASUBTYPE_X))
			SildingAvrgFilter_Kill(gyro->Magne_ft.window_h_x);
		if ((gyro->Magne_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_Y) && (gyro->Magne_ft.filter_ctrl & GYRO_DATASUBTYPE_Y))
			SildingAvrgFilter_Kill(gyro->Magne_ft.window_h_y);
		if ((gyro->Magne_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_Z) && (gyro->Magne_ft.filter_ctrl & GYRO_DATASUBTYPE_Z))
			SildingAvrgFilter_Kill(gyro->Magne_ft.window_h_z);

		if ((gyro->Magne_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_X) && (val & GYRO_DATASUBTYPE_X))
			gyro->Magne_ft.window_h_x = SildingAvrgFilter_Init(window_size);
		if ((gyro->Magne_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_Y) && (val & GYRO_DATASUBTYPE_Y))
			gyro->Magne_ft.window_h_y = SildingAvrgFilter_Init(window_size);
		if ((gyro->Magne_ft.filter_ctrl ^ val & GYRO_DATASUBTYPE_Z) && (val & GYRO_DATASUBTYPE_Z))
			gyro->Magne_ft.window_h_z = SildingAvrgFilter_Init(window_size);

		gyro->Magne_ft.filter_ctrl = val;
		break;

	default:
		break;
	}
}

float GYRO_GetRaw(GYRO_t *gyro, GYRO_DataType_t type, uint8_t sub_type)
{
	switch (type)
	{
	case GYRO_DATATYPE_ACC:
		switch (sub_type)
		{
		case GYRO_DATASUBTYPE_X:
			return gyro->Acc.acc_x;

		case GYRO_DATASUBTYPE_Y:
			return gyro->Acc.acc_y;

		case GYRO_DATASUBTYPE_Z:
			return gyro->Acc.acc_z;

		default:
			return 0;
		}
		break;

	case GYRO_DATATYPE_ANGLE:
		switch (sub_type)
		{
		case GYRO_DATASUBTYPE_X:
			return gyro->Angle.roll;

		case GYRO_DATASUBTYPE_Y:
			return gyro->Angle.pitch;

		case GYRO_DATASUBTYPE_Z:
			return gyro->Angle.yaw;

		default:
			return 0;
		}
		break;

	case GYRO_DATATYPE_ANGVEL:
		switch (sub_type)
		{
		case GYRO_DATASUBTYPE_X:
			return gyro->AngVel.w_x;

		case GYRO_DATASUBTYPE_Y:
			return gyro->AngVel.w_y;

		case GYRO_DATASUBTYPE_Z:
			return gyro->AngVel.w_z;

		default:
			return 0;
		}
		break;

	case GYRO_DATATYPE_MAGNE:
		switch (sub_type)
		{
		case GYRO_DATASUBTYPE_X:
			return gyro->Magne.h_x;

		case GYRO_DATASUBTYPE_Y:
			return gyro->Magne.h_y;

		case GYRO_DATASUBTYPE_Z:
			return gyro->Magne.h_z;

		default:
			return 0;
		}
		break;

	default:
		break;
	}
}

float GYRO_GetFilt(GYRO_t *gyro, GYRO_DataType_t type, uint8_t sub_type)
{
	switch (type)
	{
	case GYRO_DATATYPE_ACC:
		switch (sub_type)
		{
		case GYRO_DATASUBTYPE_X:
			return gyro->Acc_ft.acc_x;

		case GYRO_DATASUBTYPE_Y:
			return gyro->Acc_ft.acc_y;

		case GYRO_DATASUBTYPE_Z:
			return gyro->Acc_ft.acc_z;

		default:
			return 0;
		}
		break;

	case GYRO_DATATYPE_ANGLE:
		switch (sub_type)
		{
		case GYRO_DATASUBTYPE_X:
			return gyro->Angle_ft.roll;

		case GYRO_DATASUBTYPE_Y:
			return gyro->Angle_ft.pitch;

		case GYRO_DATASUBTYPE_Z:
			return gyro->Angle_ft.yaw;

		default:
			return 0;
		}
		break;

	case GYRO_DATATYPE_ANGVEL:
		switch (sub_type)
		{
		case GYRO_DATASUBTYPE_X:
			return gyro->AngVel_ft.w_x;

		case GYRO_DATASUBTYPE_Y:
			return gyro->AngVel_ft.w_y;

		case GYRO_DATASUBTYPE_Z:
			return gyro->AngVel_ft.w_z;

		default:
			return 0;
		}
		break;

	case GYRO_DATATYPE_MAGNE:
		switch (sub_type)
		{
		case GYRO_DATASUBTYPE_X:
			return gyro->Magne_ft.h_x;

		case GYRO_DATASUBTYPE_Y:
			return gyro->Magne_ft.h_y;

		case GYRO_DATASUBTYPE_Z:
			return gyro->Magne_ft.h_z;

		default:
			return 0;
		}
		break;

	default:
		break;
	}
}

void GYRO_DispAll(GYRO_t *gyro)
{
	printf("GYRO File descriptor:%d\n", gyro->fd);
	printf("Time:\n");
	printf("\ty:%d\n", gyro->Time.year);
	printf("\tm:%d\n", gyro->Time.month);
	printf("\td:%d\n", gyro->Time.day);
	printf("\th:%d\n", gyro->Time.hour);
	printf("\tm:%d\n", gyro->Time.minute);
	printf("\ts:%f\n", gyro->Time.second + gyro->Time.m_second / 1000.0f);
	printf("ACC:\n");
	printf("\tx:%f\n", gyro->Acc.acc_x);
	printf("\ty:%f\n", gyro->Acc.acc_y);
	printf("\tz:%f\n", gyro->Acc.acc_z);
	printf("\tt:%f\n", gyro->Acc.temp);
	printf("AngVel:\n");
	printf("\tx:%f\n", gyro->AngVel.w_x);
	printf("\ty:%f\n", gyro->AngVel.w_y);
	printf("\tz:%f\n", gyro->AngVel.w_z);
	printf("\ttemp:%f\n", gyro->AngVel.temp);
	printf("Angle:\n");
	printf("\tr:%f\n", gyro->Angle.roll);
	printf("\tp:%f\n", gyro->Angle.pitch);
	printf("\ty:%f\n", gyro->Angle.yaw);
	printf("\ttemp:%f\n", gyro->Angle.temp);
	printf("Magne:\n");
	printf("\tx:%f\n", gyro->Magne.h_x);
	printf("\ty:%f\n", gyro->Magne.h_y);
	printf("\tz:%f\n", gyro->Magne.h_z);
	printf("\ttemp:%f\n", gyro->Magne.temp);
}

void GYRO_Close(GYRO_t *gyro)
{
	if (gyro->log_fp != NULL)
	{
		fclose(gyro->log_fp);
		printf("Gyro log file saved. Filename is %s.\n", gyro->log_filename);
	}

	if (gyro->Acc_ft.filter_ctrl & GYRO_DATASUBTYPE_X)
		SildingAvrgFilter_Kill(gyro->Acc_ft.window_acc_x);
	if (gyro->Acc_ft.filter_ctrl & GYRO_DATASUBTYPE_Y)
		SildingAvrgFilter_Kill(gyro->Acc_ft.window_acc_y);
	if (gyro->Acc_ft.filter_ctrl & GYRO_DATASUBTYPE_Z)
		SildingAvrgFilter_Kill(gyro->Acc_ft.window_acc_z);

	if (gyro->Angle_ft.filter_ctrl & GYRO_DATASUBTYPE_X)
		SildingAvrgFilter_Kill(gyro->Angle_ft.window_roll);
	if (gyro->Angle_ft.filter_ctrl & GYRO_DATASUBTYPE_Y)
		SildingAvrgFilter_Kill(gyro->Angle_ft.window_pitch);
	if (gyro->Angle_ft.filter_ctrl & GYRO_DATASUBTYPE_Z)
		SildingAvrgFilter_Kill(gyro->Angle_ft.window_yaw);

	if (gyro->AngVel_ft.filter_ctrl & GYRO_DATASUBTYPE_X)
		SildingAvrgFilter_Kill(gyro->AngVel_ft.window_w_x);
	if (gyro->AngVel_ft.filter_ctrl & GYRO_DATASUBTYPE_Y)
		SildingAvrgFilter_Kill(gyro->AngVel_ft.window_w_y);
	if (gyro->AngVel_ft.filter_ctrl & GYRO_DATASUBTYPE_Z)
		SildingAvrgFilter_Kill(gyro->AngVel_ft.window_w_z);

	if (gyro->Magne_ft.filter_ctrl & GYRO_DATASUBTYPE_X)
		SildingAvrgFilter_Kill(gyro->Magne_ft.window_h_x);
	if (gyro->Magne_ft.filter_ctrl & GYRO_DATASUBTYPE_Y)
		SildingAvrgFilter_Kill(gyro->Magne_ft.window_h_y);
	if (gyro->Magne_ft.filter_ctrl & GYRO_DATASUBTYPE_Z)
		SildingAvrgFilter_Kill(gyro->Magne_ft.window_h_z);
}
