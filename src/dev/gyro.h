#ifndef GYRO_H
#define GYRO_H

#include "stdio.h"
#include "stdint.h"
#include "uart.h"
#include "time.h"

#include "../usr_lib/FILTER/filter.h"
#include "../usr_lib/QUEUE/queue.h"

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

typedef struct GYRO_Acc_Filter_t
{
	float acc_x;
	float acc_y;
	float acc_z;
	uint8_t filter_ctrl;
	queue_t *window_acc_x;
	queue_t *window_acc_y;
	queue_t *window_acc_z;
} GYRO_Acc_Filter_t;

typedef struct GYRO_AngVel_Filter_t
{
	float w_x;
	float w_y;
	float w_z;
	uint8_t filter_ctrl;
	queue_t *window_w_x;
	queue_t *window_w_y;
	queue_t *window_w_z;
} GYRO_AngVel_Filter_t;

typedef struct GYRO_Angle_Filter_t
{
	float roll;
	float pitch;
	float yaw;
	uint8_t filter_ctrl;
	queue_t *window_roll;
	queue_t *window_pitch;
	queue_t *window_yaw;
} GYRO_Angle_Filter_t;

typedef struct GYRO_Magne_Filter_t
{
	float h_x;
	float h_y;
	float h_z;
	uint8_t filter_ctrl;
	queue_t *window_h_x;
	queue_t *window_h_y;
	queue_t *window_h_z;
} GYRO_Magne_Filter_t;

typedef struct GYRO_t
{
	int fd;
	char filename[20];

	FILE *log_fp;
	char log_filename[64];
	uint8_t log_ctrl;

	GYRO_Time_t Time;
	GYRO_Acc_t Acc;
	GYRO_AngVel_t AngVel;
	GYRO_Angle_t Angle;
	GYRO_Magne_t Magne;

	GYRO_Acc_Filter_t Acc_ft;
	GYRO_AngVel_Filter_t AngVel_ft;
	GYRO_Angle_Filter_t Angle_ft;
	GYRO_Magne_Filter_t Magne_ft;

} GYRO_t;

#define GYRO_NODATA -2
#define GYRO_OVERTIME -1
#define GYRO_ERROR 0
#define GYRO_REV_TIME 1
#define GYRO_REV_ACC 2
#define GYRO_REV_ANGVEL 3
#define GYRO_REV_ANGLE 4
#define GYRO_REV_MANGE 5

typedef enum GYRO_DataType_t
{
	GYRO_DATATYPE_ANYONE = 0x00,
	GYRO_DATATYPE_ACC = 0x51,
	GYRO_DATATYPE_ANGVEL = 0x52,
	GYRO_DATATYPE_ANGLE = 0x53,
	GYRO_DATATYPE_MAGNE = 0x54
} GYRO_DataType_t;

#define GYRO_DATASUBTYPE_X 0x01
#define GYRO_DATASUBTYPE_Y 0x02
#define GYRO_DATASUBTYPE_Z 0x04
#define GYRO_DATASUBTYPE_T 0x08

#define GYRO_LOG_DISABLE 0x00
#define GYRO_LOG_ENABLE 0x01

#define GYRO_LOG_ACC (0x01 << 0)
#define GYRO_LOG_ANGVEL (0x01 << 1)
#define GYRO_LOG_ANGLE (0x01 << 2)
#define GYRO_LOG_MANGE (0x01 << 3)
#define GYRO_LOG_ACC_FT (0x01 << 4)
#define GYRO_LOG_ANGVEL_FT (0x01 << 5)
#define GYRO_LOG_ANGLE_FT (0x01 << 6)
#define GYRO_LOG_MANGE_FT (0x01 << 7)

int GYRO_Init(GYRO_t *gyro, int baud);
void GYRO_ConfigLog(GYRO_t *gyro, int en_log, uint8_t type);
void GYRO_ConfigFilt(GYRO_t *gyro, GYRO_DataType_t type, uint8_t val, uint8_t window_size);

void GYRO_RecLog(GYRO_t *gyro);
int GYRO_Read(GYRO_t *gyro, GYRO_DataType_t data_type);

float GYRO_GetRaw(GYRO_t *gyro, GYRO_DataType_t type, uint8_t sub_type);
float GYRO_GetFilt(GYRO_t *gyro, GYRO_DataType_t type, uint8_t sub_type);

void GYRO_DispAll(GYRO_t *gyro);
void GYRO_Close(GYRO_t *gyro);

#endif

#endif
