#ifndef POSE_CTRL_H
#define POSE_CTRL_H

#include "../tasks.h"
#include "../usr_lib/PID/pid.h"
#include "../usr_lib/PID/pid_math.h"
#include "../usr_lib/RC/RC.h"

#define CTRL_NONE 2
#define CTRL_BODY 1
#define CTRL_POS 0
#define CTRL_BOTH 3

void PoseCtrlInit(void);
void PoseCtrlTask(RC_Robot_t *robot, int ctrl_method);

#endif
