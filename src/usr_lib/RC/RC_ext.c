#include "RC.h"

void RC_ThreeDivided_Optimization(RC_Robot_t *pRoobt)
{
	pRoobt->Zero.centre_x = -3.0f * pRoobt->Move.span_x / 20.0f;
}
