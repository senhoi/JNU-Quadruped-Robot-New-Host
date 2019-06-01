#ifndef GYRO_H
#define GYRO_H

#include "stdio.h"
#include "stdint.h"
#include "uart.h"

#define GYRO_WT901CTTL

#ifdef GYRO_WT901CTTL

#define GYRO_FRAME_LEN 10
#define GYRO_FRAME_HEAD 0x55

typedef enum GYRO_FrameType_t
{
	Time = 0x50,
	Acc = 0x51,
	AngVel = 0x52,
	Angle = 0x53,
	Magne = 0x54
} GYRO_FrameType_t;

typedef struct GYRO_Time_t
{
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint16_t m_second;
} GYRO_Time_t;

typedef struct GYRO_Acc_t
{
	float acc_x;
	float acc_y;
	float acc_z;
	float temp;
} GYRO_Acc_t;

typedef struct GYRO_AngVel_t
{
	float w_x;
	float w_y;
	float w_z;
	float temp;
} GYRO_AngVel_t;

typedef struct GYRO_Angle_t
{
	float roll;
	float pitch;
	float yaw;
	float temp;
} GYRO_Angle_t;

typedef struct GYRO_Magne_t
{
	float h_x;
	float h_y;
	float h_z;
	float temp;
} GYRO_Magne_t;

typedef struct GYRO_t
{
	int fd;
	GYRO_Time_t Time;
	GYRO_Acc_t Acc;
	GYRO_AngVel_t AngVel;
	GYRO_Angle_t Angle;
	GYRO_Magne_t Magne;
} GYRO_t;

#define GYRO_NODATA -2
#define GYRO_OVERTIME -1
#define GYRO_ERROR 0
#define GYRO_REV_TIME 1
#define GYRO_REV_ACC 2
#define GYRO_REV_ANGVEL 3
#define GYRO_REV_ANGLE 4
#define GYRO_REV_MANGE 5

void GYRO_Init(GYRO_t *gyro, int baud);
int GYRO_Read(GYRO_t *gyro);
void GYRO_DispAll(GYRO_t *gyro);

#endif

#endif
