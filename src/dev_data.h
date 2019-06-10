#ifndef _DEV_DATA_H
#define _DEV_DATA_H

#include "time.h"
#include "dev/uart.h"
#include "dev/gyro.h"
#include "dev/xbox.h"
#include "usr_lib/FILTER/filter.h"

enum Gait_t
{
	STANDING = 0,
	TROTTING,
	WALKING
};

enum Coordinate_t
{
	UNIVERSE_XYPY,
	POSITION_YZPY,
	POSITION_XZRY
};

typedef struct Remote_t
{
	int Joystick_LX;
	int Joystick_LY;
	int Joystick_RX;
	int Joystick_RY;
	unsigned char Dial;
	enum Gait_t Gait;
	enum Coordinate_t Coordinate;

	float LX_Factor;
	float LY_Factor;
	float RX_Factor;
	float RY_Factor;
	float Dial_Factor;
} Remote_t;

typedef enum Gyro_EnLog_t
{
	LOG_OFF,
	LOG_ON
} Gyro_EnLog_t;

typedef enum Gyro_EnFilter_t
{
	FILTER_OFF,
	FILTER_ON
} Gyro_EnFilter_t;

typedef struct Gyro_Log_t
{
	Gyro_EnLog_t enable;
	char filename[32];
	FILE *fd;
} Gyro_Log_t;

typedef struct Gyro_t
{
	Gyro_EnFilter_t en_filter;

	float Pitch;
	float Roll;
	float Yaw;
	float X;
	float Y;
	float Z;

	queue_t *Pitch_Filter;
	queue_t *Roll_Filter;
	queue_t *Yaw_Filter;
	queue_t *X_Filter;
	queue_t *Y_Filter;
	queue_t *Z_Filter;

	float Pitch_;
	float Roll_;
	float Yaw_;
	float X_;
	float Y_;
	float Z_;

	Gyro_Log_t Log;
} Gyro_t;

typedef uint8_t FootStatus_t;

extern Remote_t RemoteData;
extern Gyro_t GyroData;

extern uint8_t FootGrounding;

void AnalysisFootGroundingData(serial_frame_t *pFrame);
void AnalysisRemoteData(serial_frame_t *pFrame);
void AnalysisGyroData(serial_frame_t *pFrame);

void DispRemoteData(void);
void DispGyroData(void);
void DispFootGroundingData(FootStatus_t FootGrounding);

void CreateGyroLogFile(void);
void WriteGyroLogFile(void);
void SaveGyroLogFile(void);

void InitGyro(void);
void ReadGyro_RPY(void);

float GetGyro_FilterRoll(void);
float GetGyro_FilterPitch(void);
float GetGyro_FilterYaw(void);

void DevInit(void);

#endif
