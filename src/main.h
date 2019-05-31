#ifndef _MAIN_H
#define _MAIN_H

#include <sys/time.h>
#include <sys/unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include "tasks.h"
#include "dev/gyro.h"

#define PRINTF_ERROR(contents, args...) \
	{                                   \
		printf("[ERROR]:");             \
		printf(contents, ##args);       \
		printf("\n");                   \
	}

#define PRINTF_WARNING(contents, args...) \
	{                                     \
		printf("[WARNING]:");             \
		printf(contents, ##args);         \
		printf("\n");                     \
	}

#define PRINTF_TIPS(contents, args...) \
	{                                  \
		printf("[TIPS]:");             \
		printf(contents, ##args);      \
		printf("\n");                  \
	}

#define PRINTF_DEBUG(contents, args...) \
	{                                   \
		printf("[DEBUG]:");             \
		printf(contents, ##args);       \
		printf("\n");                   \
	}

#define LOG(fd, contents, args...)         \
	{                                      \
		if (fd != NULL)                    \
		{                                  \
			fprintf(fd, contents, ##args); \
			fprintf(fd, "\n");             \
		}                                  \
	}

typedef enum Sys_Interface_t
{
	GPIO,
	USB
} Sys_Interface_t;

typedef enum Sys_SynSig_t
{
	TIMER,
	INTERRUPT
} Sys_SynSig_t;

typedef enum Sys_EnLog_t
{
	OFF,
	ON
} Sys_EnLog_t;

typedef struct Sys_Log_t
{
	Sys_EnLog_t enable;
	char filename[32];
	FILE *fd;
} Sys_Log_t;

typedef struct Sys_Setting_t
{
	Sys_Interface_t interface;
	Sys_SynSig_t syn_sig;
	Sys_Log_t log;

	int fd_uart;
} Sys_Setting_t;

extern Sys_Setting_t setting;

#endif
