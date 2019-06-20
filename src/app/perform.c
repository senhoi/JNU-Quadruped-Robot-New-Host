#include "perform.h"

void PerformUpdate(perform_t *performace)
{
	TP_Update_QuinticPoly(&performace->tp,
						  performace->motion[performace->seg_idx].pos,
						  performace->motion[performace->seg_idx].spd,
						  performace->motion[performace->seg_idx].acc,
						  performace->motion[performace->seg_idx].cycle / 1000.0f);
}

void PerformCalc(perform_t *performace)
{
	if (performace->seg_idx != performace->end_idx)
	{
		if (performace->_seg_idx != performace->seg_idx)
		{
			PerformUpdate(performace);
		}
		else
		{
			TP_Calc_QuinticPoly__(&performace->tp, performace->seg_time / 1000.0f);
		}

		performace->_seg_idx = performace->seg_idx;

		if (performace->seg_time > performace->motion[performace->seg_idx].cycle)
		{
			if (performace->seg_idx < performace->end_idx)
			{
				performace->seg_idx++;
				performace->seg_time = 0;
			}
		}

		performace->seg_time += 5;
	}
}

/******************************************************************************** */

perform_t perform_span_x;
perform_t perform_span_y;
perform_t perform_span_z;
perform_t perform_span_w;
perform_t perform_body_x;
perform_t perform_body_z;
perform_t perform_body_ro;
perform_t perform_body_pi;
perform_t perform_body_ya;

static void set_span_x(void)
{
	perform_span_x.end_idx = 5;
	perform_span_x._seg_idx = -1;
	perform_span_x.seg_idx = 0;
	perform_span_x.seg_time = 0;

	perform_span_x.motion[0].pos = -100;
	perform_span_x.motion[0].cycle = 500;

	perform_span_x.motion[1].pos = -100;
	perform_span_x.motion[1].cycle = 2500;

	perform_span_x.motion[2].pos = 100;
	perform_span_x.motion[2].cycle = 1000;

	perform_span_x.motion[3].pos = 100;
	perform_span_x.motion[3].cycle = 2500;

	perform_span_x.motion[4].pos = 0;
	perform_span_x.motion[4].cycle = 500;

	TP_Init_QuinticPoly(&perform_span_x.tp, 0, 0, 0, 0, 0, 0, 0, 0.005);
}

static void set_span_y(void)
{
	perform_span_y.end_idx = 5;
	perform_span_y._seg_idx = -1;
	perform_span_y.seg_idx = 0;
	perform_span_y.seg_time = 0;

	perform_span_y.motion[0].pos = 0;
	perform_span_y.motion[0].cycle = 22000;

	perform_span_y.motion[1].pos = 50;
	perform_span_y.motion[1].cycle = 100;

	perform_span_y.motion[2].pos = 50;
	perform_span_y.motion[2].cycle = 1400;

	perform_span_y.motion[3].pos = -50;
	perform_span_y.motion[3].cycle = 1400;

	perform_span_y.motion[4].pos = 0;
	perform_span_y.motion[4].cycle = 100;

	TP_Init_QuinticPoly(&perform_span_y.tp, 0, 0, 0, 0, 0, 0, 0, 0.005);
}

static void set_span_z(void)
{
	perform_span_z.end_idx = 7;
	perform_span_z._seg_idx = -1;
	perform_span_z.seg_idx = 0;
	perform_span_z.seg_time = 0;

	perform_span_z.motion[0].pos = 35;
	perform_span_z.motion[0].cycle = 500;

	perform_span_z.motion[1].pos = 35;
	perform_span_z.motion[1].cycle = 6000;

	perform_span_z.motion[2].pos = 0;
	perform_span_z.motion[2].cycle = 500;

	perform_span_z.motion[3].pos = 0;
	perform_span_z.motion[3].cycle = 14500;

	perform_span_z.motion[4].pos = 35;
	perform_span_z.motion[4].cycle = 500;

	perform_span_z.motion[5].pos = 35;
	perform_span_z.motion[5].cycle = 13500;

	perform_span_z.motion[6].pos = 0;
	perform_span_z.motion[6].cycle = 500;

	TP_Init_QuinticPoly(&perform_span_z.tp, 0, 0, 0, 0, 0, 0, 0, 0.005);
}

static void set_span_w(void)
{
	perform_span_w.end_idx = 4;
	perform_span_w._seg_idx = -1;
	perform_span_w.seg_idx = 0;
	perform_span_w.seg_time = 0;

	perform_span_w.motion[0].pos = 0;
	perform_span_w.motion[0].cycle = 29000;

	perform_span_w.motion[1].pos = pi / 17;
	perform_span_w.motion[1].cycle = 200;

	perform_span_w.motion[2].pos = pi / 17;
	perform_span_w.motion[2].cycle = 6600;

	perform_span_w.motion[3].pos = 0;
	perform_span_w.motion[3].cycle = 200;

	TP_Init_QuinticPoly(&perform_span_w.tp, 0, 0, 0, 0, 0, 0, 0, 0.005);
}

static void set_body_x(void)
{
	perform_body_x.end_idx = 5;
	perform_body_x._seg_idx = -1;
	perform_body_x.seg_idx = 0;
	perform_body_x.seg_time = 0;

	perform_body_x.motion[0].pos = 0;
	perform_body_x.motion[0].cycle = 36000;

	//sexy
	perform_body_x.motion[1].pos = 50;
	perform_body_x.motion[1].cycle = 500;

	perform_body_x.motion[2].pos = -50;
	perform_body_x.motion[2].cycle = 1000;

	perform_body_x.motion[3].pos = 50;
	perform_body_x.motion[3].cycle = 1000;

	perform_body_x.motion[4].pos = 0;
	perform_body_x.motion[4].cycle = 500;

	TP_Init_QuinticPoly(&perform_body_x.tp, 0, 0, 0, 0, 0, 0, 0, 0.005);
}

static void set_body_z(void)
{
	perform_body_z.end_idx = 12;
	perform_body_z._seg_idx = -1;
	perform_body_z.seg_idx = 0;
	perform_body_z.seg_time = 0;

	perform_body_z.motion[0].pos = 400 + 50;
	perform_body_z.motion[0].cycle = 500;

	perform_body_z.motion[1].pos = 400 - 50;
	perform_body_z.motion[1].cycle = 2000;

	perform_body_z.motion[2].pos = 400 + 50;
	perform_body_z.motion[2].cycle = 2000;

	perform_body_z.motion[3].pos = 400 - 50;
	perform_body_z.motion[3].cycle = 2000;

	perform_body_z.motion[4].pos = 400;
	perform_body_z.motion[4].cycle = 500;

	perform_body_z.motion[5].pos = 400;
	perform_body_z.motion[5].cycle = 18000;

	//sexy
	perform_body_z.motion[6].pos = 400 + 50;
	perform_body_z.motion[6].cycle = 500;

	perform_body_z.motion[7].pos = 400 - 50;
	perform_body_z.motion[7].cycle = 500;

	perform_body_z.motion[8].pos = 400 + 50;
	perform_body_z.motion[8].cycle = 500;

	perform_body_z.motion[9].pos = 400 - 50;
	perform_body_z.motion[9].cycle = 500;

	perform_body_z.motion[10].pos = 400 + 50;
	perform_body_z.motion[10].cycle = 500;

	perform_body_z.motion[11].pos = 400;
	perform_body_z.motion[11].cycle = 500;

	TP_Init_QuinticPoly(&perform_body_z.tp, 0, 0, 0, 0, 0, 0, 0, 0.005);
	perform_body_z.tp.current_pos = 400;
}

static void set_body_ro(void)
{
	perform_body_ro.end_idx = 2;
	perform_body_ro._seg_idx = -1;
	perform_body_ro.seg_idx = 0;
	perform_body_ro.seg_time = 0;

	perform_body_ro.motion[0].pos = 0;
	perform_body_ro.motion[0].cycle = 3000;

	perform_body_ro.motion[1].pos = 0;
	perform_body_ro.motion[1].cycle = 3000;

	TP_Init_QuinticPoly(&perform_body_ro.tp, 0, 0, 0, 0, 0, 0, 0, 0.005);
}

static void set_body_pi(void)
{
	perform_body_pi.end_idx = 51;
	perform_body_pi._seg_idx = -1;
	perform_body_pi.seg_idx = 0;
	perform_body_pi.seg_time = 0;

	perform_body_pi.motion[0].pos = 0;
	perform_body_pi.motion[0].cycle = 7000;

	perform_body_pi.motion[1].pos = pi / 15;
	perform_body_pi.motion[1].cycle = 200;

	perform_body_pi.motion[2].pos = -pi / 15;
	perform_body_pi.motion[2].cycle = 400;

	perform_body_pi.motion[3].pos = pi / 15;
	perform_body_pi.motion[3].cycle = 400;

	perform_body_pi.motion[4].pos = -pi / 15;
	perform_body_pi.motion[4].cycle = 400;

	perform_body_pi.motion[5].pos = pi / 15;
	perform_body_pi.motion[5].cycle = 400;

	perform_body_pi.motion[6].pos = -pi / 15;
	perform_body_pi.motion[6].cycle = 400;

	perform_body_pi.motion[7].pos = pi / 15;
	perform_body_pi.motion[7].cycle = 400;

	perform_body_pi.motion[8].pos = -pi / 15;
	perform_body_pi.motion[8].cycle = 400;

	perform_body_pi.motion[9].pos = pi / 15;
	perform_body_pi.motion[9].cycle = 400;

	perform_body_pi.motion[10].pos = -pi / 15;
	perform_body_pi.motion[10].cycle = 400;

	perform_body_pi.motion[11].pos = pi / 15;
	perform_body_pi.motion[11].cycle = 400;

	perform_body_pi.motion[12].pos = -pi / 15;
	perform_body_pi.motion[12].cycle = 400;

	perform_body_pi.motion[13].pos = pi / 15;
	perform_body_pi.motion[13].cycle = 400;

	perform_body_pi.motion[14].pos = -pi / 15;
	perform_body_pi.motion[14].cycle = 400;

	perform_body_pi.motion[15].pos = -pi / 15;
	perform_body_pi.motion[15].cycle = 400;

	perform_body_pi.motion[16].pos = +pi / 15;
	perform_body_pi.motion[16].cycle = 200;

	perform_body_pi.motion[17].pos = 0;
	perform_body_pi.motion[17].cycle = 2500;

	//全向yaw+pitch
	perform_body_pi.motion[18].pos = pi / 15;
	perform_body_pi.motion[18].cycle = 500;

	perform_body_pi.motion[19].pos = 0;
	perform_body_pi.motion[19].cycle = 500;

	perform_body_pi.motion[20].pos = -pi / 15;
	perform_body_pi.motion[20].cycle = 500;

	perform_body_pi.motion[21].pos = 0;
	perform_body_pi.motion[21].cycle = 500;

	perform_body_pi.motion[22].pos = pi / 15;
	perform_body_pi.motion[22].cycle = 500;

	perform_body_pi.motion[23].pos = 0;
	perform_body_pi.motion[23].cycle = 500;

	perform_body_pi.motion[24].pos = -pi / 15;
	perform_body_pi.motion[24].cycle = 500;

	perform_body_pi.motion[25].pos = 0;
	perform_body_pi.motion[25].cycle = 500;

	//摆头点头
	perform_body_pi.motion[26].pos = pi / 15;
	perform_body_pi.motion[26].cycle = 250;

	perform_body_pi.motion[27].pos = 0;
	perform_body_pi.motion[27].cycle = 250;

	perform_body_pi.motion[28].pos = pi / 15;
	perform_body_pi.motion[28].cycle = 250;

	perform_body_pi.motion[29].pos = 0;
	perform_body_pi.motion[29].cycle = 250;

	perform_body_pi.motion[30].pos = pi / 15;
	perform_body_pi.motion[30].cycle = 250;

	perform_body_pi.motion[31].pos = 0;
	perform_body_pi.motion[31].cycle = 250;

	perform_body_pi.motion[32].pos = pi / 15;
	perform_body_pi.motion[32].cycle = 250;

	perform_body_pi.motion[33].pos = 0;
	perform_body_pi.motion[33].cycle = 250;

	perform_body_pi.motion[34].pos = pi / 15;
	perform_body_pi.motion[34].cycle = 250;

	perform_body_pi.motion[35].pos = 0;
	perform_body_pi.motion[35].cycle = 250;

	perform_body_pi.motion[36].pos = pi / 15;
	perform_body_pi.motion[36].cycle = 250;

	perform_body_pi.motion[37].pos = 0;
	perform_body_pi.motion[37].cycle = 250;

	perform_body_pi.motion[38].pos = 0;
	perform_body_pi.motion[38].cycle = 14000;

	//sexy 36s+
	perform_body_pi.motion[39].pos = pi / 15;
	perform_body_pi.motion[39].cycle = 500;

	perform_body_pi.motion[40].pos = -pi / 15;
	perform_body_pi.motion[40].cycle = 1000;

	perform_body_pi.motion[41].pos = pi / 15;
	perform_body_pi.motion[41].cycle = 1000;

	perform_body_pi.motion[42].pos = 0;
	perform_body_pi.motion[42].cycle = 500;

	//sexy 39s+
	perform_body_pi.motion[43].pos = 0;
	perform_body_pi.motion[43].cycle = 300;

	perform_body_pi.motion[44].pos = pi / 15;
	perform_body_pi.motion[44].cycle = 225;

	perform_body_pi.motion[45].pos = -pi / 15;
	perform_body_pi.motion[45].cycle = 450;

	perform_body_pi.motion[46].pos = 0;
	perform_body_pi.motion[46].cycle = 225;

	perform_body_pi.motion[47].pos = 0;
	perform_body_pi.motion[47].cycle = 600;

	perform_body_pi.motion[48].pos = pi / 15;
	perform_body_pi.motion[48].cycle = 225;

	perform_body_pi.motion[49].pos = -pi / 15;
	perform_body_pi.motion[49].cycle = 450;

	perform_body_pi.motion[50].pos = 0;
	perform_body_pi.motion[50].cycle = 225;

	TP_Init_QuinticPoly(&perform_body_pi.tp, 0, 0, 0, 0, 0, 0, 0, 0.005);
}

static void set_body_ya(void)
{
	perform_body_ya.end_idx = 24;
	perform_body_ya._seg_idx = -1;
	perform_body_ya.seg_idx = 0;
	perform_body_ya.seg_time = 0;

	perform_body_ya.motion[0].pos = 0;
	perform_body_ya.motion[0].cycle = 13500;

	//全向yaw+pitch
	perform_body_ya.motion[1].pos = pi / 15;
	perform_body_ya.motion[1].cycle = 500;

	perform_body_ya.motion[2].pos = 0;
	perform_body_ya.motion[2].cycle = 500;

	perform_body_ya.motion[3].pos = -pi / 15;
	perform_body_ya.motion[3].cycle = 500;

	perform_body_ya.motion[4].pos = 0;
	perform_body_ya.motion[4].cycle = 500;

	perform_body_ya.motion[5].pos = pi / 15;
	perform_body_ya.motion[5].cycle = 500;

	perform_body_ya.motion[6].pos = 0;
	perform_body_ya.motion[6].cycle = 500;

	perform_body_ya.motion[7].pos = -pi / 15;
	perform_body_ya.motion[7].cycle = 500;

	perform_body_ya.motion[8].pos = 0;
	perform_body_ya.motion[8].cycle = 500;

	perform_body_ya.motion[9].pos = pi / 15;
	perform_body_ya.motion[9].cycle = 500;

	perform_body_ya.motion[10].pos = 0;
	perform_body_ya.motion[10].cycle = 500;

	perform_body_ya.motion[11].pos = -pi / 15;
	perform_body_ya.motion[11].cycle = 500;

	perform_body_ya.motion[12].pos = 0;
	perform_body_ya.motion[12].cycle = 500;

	//摆头点头 19.5s+
	perform_body_ya.motion[13].pos = pi / 15;
	perform_body_ya.motion[13].cycle = 500;

	perform_body_ya.motion[14].pos = pi / 15; //在pi/15处停顿500ms
	perform_body_ya.motion[14].cycle = 500;

	perform_body_ya.motion[15].pos = -pi / 15;
	perform_body_ya.motion[15].cycle = 1000;

	perform_body_ya.motion[16].pos = -pi / 15; //在-pi/15处停顿500ms
	perform_body_ya.motion[16].cycle = 500;

	perform_body_ya.motion[17].pos = 0;
	perform_body_ya.motion[17].cycle = 500;

	perform_body_ya.motion[18].pos = 0;
	perform_body_ya.motion[18].cycle = 16500;

	//sexy2
	perform_body_ya.motion[19].pos = pi / 15;
	perform_body_ya.motion[19].cycle = 300;

	perform_body_ya.motion[20].pos = pi / 15; //在pi/15处停顿900ms
	perform_body_ya.motion[20].cycle = 900;

	perform_body_ya.motion[21].pos = -pi / 15;
	perform_body_ya.motion[21].cycle = 600;

	perform_body_ya.motion[22].pos = -pi / 15; //在-pi/15处停顿900ms
	perform_body_ya.motion[22].cycle = 900;

	perform_body_ya.motion[23].pos = 0;
	perform_body_ya.motion[23].cycle = 300;

	TP_Init_QuinticPoly(&perform_body_ya.tp, 0, 0, 0, 0, 0, 0, 0, 0.005);
}

static float calc_span_x(void)
{
	PerformCalc(&perform_span_x);
	return perform_span_x.tp.current_pos;
}

static float calc_span_y(void)
{
	PerformCalc(&perform_span_y);
	return perform_span_y.tp.current_pos;
}

static float calc_span_z(void)
{
	PerformCalc(&perform_span_z);
	return perform_span_z.tp.current_pos;
}

static float calc_span_w(void)
{
	PerformCalc(&perform_span_w);
	return perform_span_w.tp.current_pos;
}

static float calc_body_x(void)
{
	PerformCalc(&perform_body_x);
	return perform_body_x.tp.current_pos;
}

static float calc_body_z(void)
{
	PerformCalc(&perform_body_z);
	return perform_body_z.tp.current_pos;
}

static float calc_body_ro(void)
{
	PerformCalc(&perform_body_ro);
	return perform_body_ro.tp.current_pos;
}

static float calc_body_pi(void)
{
	PerformCalc(&perform_body_pi);
	return perform_body_pi.tp.current_pos;
}

static float calc_body_ya(void)
{
	PerformCalc(&perform_body_ya);
	return perform_body_ya.tp.current_pos;
}

/******************************************************************************** */

void PerformInit(void)
{
	set_span_x();
	set_span_y();
	set_span_z();
	set_span_w();
	set_body_x();
	set_body_z();
	set_body_ro();
	set_body_pi();
	set_body_ya();
}

void PerformTask(RC_Robot_t *robot)
{
	robot->Move.span_x = calc_span_x();
	robot->Move.span_y = calc_span_y();
	robot->Move.span_z = calc_span_z();
	robot->Move.span_w = calc_span_w();

	robot->Pose.body_x = calc_body_x();
	robot->Pose.body_z = calc_body_z();
	robot->Pose.body_ro = calc_body_ro();
	robot->Pose.body_pi = calc_body_pi();
	robot->Pose.body_ya = calc_body_ya();

	//printf("span_x:%7.2f\tspan_y:%7.2f\tspan_z:%7.2f\tspan_w:%7.2f\n", robot->Move.span_x, robot->Move.span_y, robot->Move.span_z, robot->Move.span_w);
	//printf("body_x:%7.2f\tbody_z:%7.2f\tbody_ro:%7.2f\tbody_pi:%7.2f\tbody_ya:%7.2f\n", robot->Pose.body_x, robot->Pose.body_z, robot->Pose.body_ro, robot->Pose.body_pi, robot->Pose.body_ya);
}
