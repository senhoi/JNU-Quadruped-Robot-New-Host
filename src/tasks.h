#ifndef _TASK_H
#define _TASK_H

#include "main.h"
#include "time.h"
#include "errno.h"
#include "dev_data.h"
#include "dev/uart.h"
#include "dev/priority.h"
#include "usr_lib/RC/RC.h"

void InitTask(void);
void InterruptTask(void);
void LowPriorityTask(void);
void DisplayTask(void);

#endif
