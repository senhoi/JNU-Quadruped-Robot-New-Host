#include "gyro.h"

void GYRO_Init(GYRO_t *gyro, int baud)
{
	gyro->fd = serialOpen("/dev/ttyUSB1", baud);

	if (gyro->fd == -1)
		perror("Failed to open /dev/ttyUSB1.\n");
	else
	{
		printf("Success to open /dev/ttyUSB1.\n");
		printf("Baud rate: %d.\tFile descriptor: %d.\n", baud, gyro->fd);
		getchar();
	}
}

int GYRO_Read(GYRO_t *gyro)
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
							break;
						case AngVel:
							gyro->AngVel.w_x = (int16_t)((data[1] << 8) | data[0]) / 32768.0f * 2000;
							gyro->AngVel.w_y = (int16_t)((data[3] << 8) | data[2]) / 32768.0f * 2000;
							gyro->AngVel.w_z = (int16_t)((data[5] << 8) | data[4]) / 32768.0f * 2000;
							gyro->AngVel.temp = (int16_t)((data[7] << 8) | data[6]) / 100.0f;
							break;
						case Angle:
							gyro->Angle.roll = (int16_t)((data[1] << 8) | data[0]) / 32768.0f * 180;
							gyro->Angle.pitch = (int16_t)((data[3] << 8) | data[2]) / 32768.0f * 180;
							gyro->Angle.yaw = (int16_t)((data[5] << 8) | data[4]) / 32768.0f * 180;
							gyro->Angle.temp = (int16_t)((data[7] << 8) | data[6]) / 100.0f;
							break;
						case Magne:
							gyro->Magne.h_x = (int16_t)(data[1] << 8) | data[0];
							gyro->Magne.h_y = (int16_t)(data[3] << 8) | data[2];
							gyro->Magne.h_z = (int16_t)(data[5] << 8) | data[4];
							gyro->Magne.temp = (int16_t)((data[7] << 8) | data[6]) / 100.0f;
							break;
						}
						if (serialDataAvail(gyro->fd) >= 256)
							serialFlush(gyro->fd);
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
					return GYRO_ERROR;
					break;
				}
			}
		}
	}
	return GYRO_NODATA;
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
