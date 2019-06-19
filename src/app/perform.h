#ifndef PERFORM_H
#define PERFORM_H

#include "../usr_lib/TP/TP_polynomial.h"
#include "../usr_lib/RC/RC.h"
#include <stdint.h>

typedef struct perform_item_t
{
	float pos;
	float spd;
	float acc;
	float cycle;

} perform_item_t;

typedef struct perform_t
{
	TP_QuinticPoly_t tp;

	uint16_t end_idx;

	int16_t seg_idx;
	int16_t _seg_idx;

	uint16_t seg_time;

	perform_item_t motion[50];

} perform_t;

void PerformInit(void);
void PerformTask(RC_Robot_t *robot);

#endif
