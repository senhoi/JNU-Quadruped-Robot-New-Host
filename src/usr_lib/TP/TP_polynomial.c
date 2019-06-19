#include "TP_polynomial.h"

static void TP_CalcCoeff_QuinticPoly(TP_QuinticPoly_t *pTP_QuinticPoly)
{
	pTP_QuinticPoly->a[0] = pTP_QuinticPoly->init_pos;
	pTP_QuinticPoly->a[1] = pTP_QuinticPoly->init_spd;
	pTP_QuinticPoly->a[2] = pTP_QuinticPoly->init_acc / 2;
	pTP_QuinticPoly->a[3] = (20 * pTP_QuinticPoly->final_pos - 20 * pTP_QuinticPoly->init_pos - (8 * pTP_QuinticPoly->final_spd + 12 * pTP_QuinticPoly->init_spd) * pTP_QuinticPoly->cycle - (3 * pTP_QuinticPoly->init_acc - pTP_QuinticPoly->final_acc) * pTP_QuinticPoly->cycle * pTP_QuinticPoly->cycle) / (2 * pow(pTP_QuinticPoly->cycle, 3));
	pTP_QuinticPoly->a[4] = (-30 * pTP_QuinticPoly->final_pos + 30 * pTP_QuinticPoly->init_pos + (14 * pTP_QuinticPoly->final_spd + 16 * pTP_QuinticPoly->init_spd) * pTP_QuinticPoly->cycle + (3 * pTP_QuinticPoly->init_acc - 2 * pTP_QuinticPoly->final_acc) * pTP_QuinticPoly->cycle * pTP_QuinticPoly->cycle) / (2 * pow(pTP_QuinticPoly->cycle, 4));
	pTP_QuinticPoly->a[5] = (12 * pTP_QuinticPoly->final_pos - 12 * pTP_QuinticPoly->init_pos - (6 * pTP_QuinticPoly->final_spd + 6 * pTP_QuinticPoly->init_spd) * pTP_QuinticPoly->cycle - (pTP_QuinticPoly->init_acc - pTP_QuinticPoly->final_acc) * pTP_QuinticPoly->cycle * pTP_QuinticPoly->cycle) / (2 * pow(pTP_QuinticPoly->cycle, 5));
}

void TP_Init_QuinticPoly(TP_QuinticPoly_t *pTP_QuinticPoly, float init_pos, float init_spd, float init_acc, float final_pos, float final_spd, float final_acc, float cycle, float interval)
{
	pTP_QuinticPoly->cycle = cycle;
	pTP_QuinticPoly->interval = interval;
	pTP_QuinticPoly->init_pos = init_pos;
	pTP_QuinticPoly->init_spd = init_spd;
	pTP_QuinticPoly->init_acc = init_acc;
	pTP_QuinticPoly->final_pos = final_pos;
	pTP_QuinticPoly->final_spd = final_spd;
	pTP_QuinticPoly->final_acc = final_acc;

	TP_CalcCoeff_QuinticPoly(pTP_QuinticPoly);
}

void TP_Update_QuinticPoly(TP_QuinticPoly_t *pTP_QuinticPoly, float final_pos, float final_spd, float final_acc, float cycle)
{
	pTP_QuinticPoly->cycle = cycle;
	pTP_QuinticPoly->init_pos = pTP_QuinticPoly->current_pos;
	pTP_QuinticPoly->init_spd = pTP_QuinticPoly->current_spd;
	pTP_QuinticPoly->init_acc = pTP_QuinticPoly->current_acc;
	pTP_QuinticPoly->final_pos = final_pos;
	pTP_QuinticPoly->final_spd = final_spd;
	pTP_QuinticPoly->final_acc = final_acc;

	TP_CalcCoeff_QuinticPoly(pTP_QuinticPoly);
}

uint8_t TP_Calc_QuinticPoly(TP_QuinticPoly_t *pTP_QuinticPoly)
{
	if (pTP_QuinticPoly->cycle - (pTP_QuinticPoly->time + pTP_QuinticPoly->interval) < -0.00001f)
		return 0;

	pTP_QuinticPoly->time += pTP_QuinticPoly->interval;

	pTP_QuinticPoly->current_acc = 2 * pTP_QuinticPoly->a[2] + 6 * pTP_QuinticPoly->a[3] * pTP_QuinticPoly->time +
								   12 * pTP_QuinticPoly->a[4] * pow(pTP_QuinticPoly->time, 2) + 20 * pTP_QuinticPoly->a[5] * pow(pTP_QuinticPoly->time, 3);

	pTP_QuinticPoly->current_spd = pTP_QuinticPoly->a[1] + 2 * pTP_QuinticPoly->a[2] * pTP_QuinticPoly->time + 3 * pTP_QuinticPoly->a[3] * pow(pTP_QuinticPoly->time, 2) +
								   4 * pTP_QuinticPoly->a[4] * pow(pTP_QuinticPoly->time, 3) + 5 * pTP_QuinticPoly->a[5] * pow(pTP_QuinticPoly->time, 4);

	pTP_QuinticPoly->current_pos = pTP_QuinticPoly->a[0] + pTP_QuinticPoly->a[1] * pTP_QuinticPoly->time + pTP_QuinticPoly->a[2] * pow(pTP_QuinticPoly->time, 2) +
								   pTP_QuinticPoly->a[3] * pow(pTP_QuinticPoly->time, 3) + pTP_QuinticPoly->a[4] * pow(pTP_QuinticPoly->time, 4) + pTP_QuinticPoly->a[5] * pow(pTP_QuinticPoly->time, 5);

	return 1;
}

float TP_Calc_QuinticPoly__(TP_QuinticPoly_t *pTP_QuinticPoly, float t)
{
	if (t > pTP_QuinticPoly->cycle)
		return pTP_QuinticPoly->current_pos;

	pTP_QuinticPoly->current_acc = 2 * pTP_QuinticPoly->a[2] + 6 * pTP_QuinticPoly->a[3] * t +
								   12 * pTP_QuinticPoly->a[4] * pow(t, 2) + 20 * pTP_QuinticPoly->a[5] * pow(t, 3);

	pTP_QuinticPoly->current_spd = pTP_QuinticPoly->a[1] + 2 * pTP_QuinticPoly->a[2] * t + 3 * pTP_QuinticPoly->a[3] * pow(t, 2) +
								   4 * pTP_QuinticPoly->a[4] * pow(t, 3) + 5 * pTP_QuinticPoly->a[5] * pow(t, 4);

	pTP_QuinticPoly->current_pos = pTP_QuinticPoly->a[0] + pTP_QuinticPoly->a[1] * t + pTP_QuinticPoly->a[2] * pow(t, 2) +
								   pTP_QuinticPoly->a[3] * pow(t, 3) + pTP_QuinticPoly->a[4] * pow(t, 4) + pTP_QuinticPoly->a[5] * pow(t, 5);

	return pTP_QuinticPoly->current_pos;
}

void TP_Disp_QuinticPoly(TP_QuinticPoly_t *pTP_QuinticPoly)
{
	printf("Equation:\n\tpos = %8.4f + %8.4ft + %8.4ft^2 + %8.4ft^3 + %8.4ft^4 + %8.4ft^5\n", pTP_QuinticPoly->a[0], pTP_QuinticPoly->a[1], pTP_QuinticPoly->a[2], pTP_QuinticPoly->a[3], pTP_QuinticPoly->a[4], pTP_QuinticPoly->a[5]);
}

static void TP_CalcCoeff_CubicPoly(TP_CubicPoly_t *pTP_CubicPoly)
{
	pTP_CubicPoly->a[0] = pTP_CubicPoly->init_pos;
	pTP_CubicPoly->a[1] = pTP_CubicPoly->init_spd;
	pTP_CubicPoly->a[2] = 3.0f / pow(pTP_CubicPoly->cycle, 2) * (pTP_CubicPoly->final_pos - pTP_CubicPoly->init_pos) - 2.0f / pTP_CubicPoly->cycle * pTP_CubicPoly->init_spd - 1.0f / pTP_CubicPoly->cycle * pTP_CubicPoly->final_spd;
	pTP_CubicPoly->a[3] = -2.0f / pow(pTP_CubicPoly->cycle, 3) * (pTP_CubicPoly->final_pos - pTP_CubicPoly->init_pos) + 1.0f / pow(pTP_CubicPoly->cycle, 2) * (pTP_CubicPoly->init_spd + pTP_CubicPoly->final_spd);
}

void TP_Init_CubicPoly(TP_CubicPoly_t *pTP_CubicPoly, float init_pos, float init_spd, float final_pos, float final_spd, float cycle, float interval)
{
	pTP_CubicPoly->cycle = cycle;
	pTP_CubicPoly->interval = interval;
	pTP_CubicPoly->init_pos = init_pos;
	pTP_CubicPoly->init_spd = init_spd;
	pTP_CubicPoly->final_pos = final_pos;
	pTP_CubicPoly->final_spd = final_spd;

	TP_CalcCoeff_CubicPoly(pTP_CubicPoly);
}

void TP_Update_CubicPoly(TP_CubicPoly_t *pTP_CubicPoly, float final_pos, float final_spd, float cycle)
{
	pTP_CubicPoly->cycle = cycle;
	pTP_CubicPoly->init_pos = pTP_CubicPoly->current_pos;
	pTP_CubicPoly->init_spd = pTP_CubicPoly->current_spd;
	pTP_CubicPoly->final_pos = final_pos;
	pTP_CubicPoly->final_spd = final_spd;

	TP_CalcCoeff_CubicPoly(pTP_CubicPoly);
}

uint8_t TP_Calc_CubicPoly(TP_CubicPoly_t *pTP_CubicPoly)
{
	if (pTP_CubicPoly->cycle - (pTP_CubicPoly->time + pTP_CubicPoly->interval) < -0.00001f)
		return 0;

	pTP_CubicPoly->time += pTP_CubicPoly->interval;

	pTP_CubicPoly->current_acc = 2 * pTP_CubicPoly->a[2] + 6 * pTP_CubicPoly->a[3] * pTP_CubicPoly->time;

	pTP_CubicPoly->current_spd = pTP_CubicPoly->a[1] + 2 * pTP_CubicPoly->a[2] * pTP_CubicPoly->time + 3 * pTP_CubicPoly->a[3] * pow(pTP_CubicPoly->time, 2);

	pTP_CubicPoly->current_pos = pTP_CubicPoly->a[0] + pTP_CubicPoly->a[1] * pTP_CubicPoly->time + pTP_CubicPoly->a[2] * pow(pTP_CubicPoly->time, 2) + pTP_CubicPoly->a[3] * pow(pTP_CubicPoly->time, 3);

	return 1;
}

float TP_Calc_CubicPoly__(TP_CubicPoly_t *pTP_CubicPoly, float t)
{

	pTP_CubicPoly->current_acc = 2 * pTP_CubicPoly->a[2] + 6 * pTP_CubicPoly->a[3] * t;

	pTP_CubicPoly->current_spd = pTP_CubicPoly->a[1] + 2 * pTP_CubicPoly->a[2] * t + 3 * pTP_CubicPoly->a[3] * pow(t, 2);

	pTP_CubicPoly->current_pos = pTP_CubicPoly->a[0] + pTP_CubicPoly->a[1] * t + pTP_CubicPoly->a[2] * pow(t, 2) + pTP_CubicPoly->a[3] * pow(t, 3);

	return pTP_CubicPoly->current_pos;
}

void TP_Disp_CubicPoly(TP_CubicPoly_t *pTP_CubicPoly)
{
	printf("Equation:\n\tpos = %8.4f + %8.4ft + %8.4ft^2 + %8.4ft^3\n", pTP_CubicPoly->a[0], pTP_CubicPoly->a[1], pTP_CubicPoly->a[2], pTP_CubicPoly->a[3]);
}
