#ifndef TP_POLYNOMIAL_H
#define TP_POLYNOMIAL_H

#include<stdint.h>
#include<stdio.h>
#include<math.h>

typedef struct TP_QuinticPoly_t
{
	float a[6];

	float cycle;
	float interval;
	float init_pos;
	float init_spd;
	float init_acc;
	float final_pos;
	float final_spd;
	float final_acc;

	float time;
	float current_pos;
	float current_spd;
	float current_acc;
}TP_QuinticPoly_t;

typedef struct TP_CubicPoly_t
{
	float a[4];

	float cycle;
	float interval;
	float init_pos;
	float init_spd;
	float final_pos;
	float final_spd;

	float time;
	float current_pos;
	float current_spd;
	float current_acc;
}TP_CubicPoly_t;

void TP_Init_QuinticPoly(TP_QuinticPoly_t *pTP_QuinticPoly, float init_pos, float init_spd, float init_acc, float final_pos, float final_spd, float final_acc, float cycle, float interval);
void TP_Update_QuinticPoly(TP_QuinticPoly_t *pTP_QuinticPoly, float final_pos, float final_spd, float final_acc, float cycle);
uint8_t TP_Calc_QuinticPoly(TP_QuinticPoly_t *pTP_QuinticPoly);
float TP_Calc_QuinticPoly__(TP_QuinticPoly_t *pTP_QuinticPoly, float t);
void TP_Disp_QuinticPoly(TP_QuinticPoly_t *pTP_QuinticPoly);

void TP_Init_CubicPoly(TP_CubicPoly_t *pTP_CubicPoly, float init_pos, float init_spd, float final_pos, float final_spd, float cycle, float interval);
void TP_Update_CubicPoly(TP_CubicPoly_t *pTP_CubicPoly, float final_pos, float final_spd, float cycle);
uint8_t TP_Calc_CubicPoly(TP_CubicPoly_t *pTP_CubicPoly);
float TP_Calc_CubicPoly__(TP_CubicPoly_t *pTP_CubicPoly, float t);
void TP_Disp_CubicPoly(TP_CubicPoly_t *pTP_CubicPoly);


#endif
