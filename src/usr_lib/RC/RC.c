#include "RC.h"

RC_Robot_t QuadrupedRobot;

void RC_Init_Robot(RC_Robot_t *pRoobt, char *type, float offset, float length1, float length2, float body_width, float body_length)
{
	matrix_t *m_t, *m_ry, *m_rz;

	strcpy(pRoobt->Mech.type, type);
	pRoobt->Mech.leg_d = offset;
	pRoobt->Mech.leg_a1 = length1;
	pRoobt->Mech.leg_a2 = length2;
	pRoobt->Mech.body_w = body_width;
	pRoobt->Mech.body_l = body_length;

	pRoobt->Mat.Base[RC_LEG_LF] = cmat_malloc(4, 4);
	pRoobt->Mat.Base[RC_LEG_LH] = cmat_malloc(4, 4);
	pRoobt->Mat.Base[RC_LEG_RF] = cmat_malloc(4, 4);
	pRoobt->Mat.Base[RC_LEG_RH] = cmat_malloc(4, 4);

	m_ry = cmat_se3_ry(-pi / 2);
	m_rz = cmat_se3_rz(pi);

	m_t = cmat_se3(body_length / 2, body_width / 2, 0);
	cmat_multiply_multi(pRoobt->Mat.Base[RC_LEG_LF], 3, m_t, m_ry, m_rz);
	cmat_free(m_t);
	m_t = cmat_se3(-body_length / 2, body_width / 2, 0);
	cmat_multiply_multi(pRoobt->Mat.Base[RC_LEG_LH], 3, m_t, m_ry, m_rz);
	cmat_free(m_t);
	m_t = cmat_se3(body_length / 2, -body_width / 2, 0);
	cmat_multiply_multi(pRoobt->Mat.Base[RC_LEG_RF], 3, m_t, m_ry, m_rz);
	cmat_free(m_t);
	m_t = cmat_se3(-body_length / 2, -body_width / 2, 0);
	cmat_multiply_multi(pRoobt->Mat.Base[RC_LEG_RH], 3, m_t, m_ry, m_rz);
	cmat_free(m_t);

	cmat_free_multi(2, m_ry, m_rz);
}

void RC_Init_MovPara(RC_Robot_t *pRoobt, char *gait, double cycle, double interval, double dutyratio,
					 double span_x, double span_y, double span_z, double span_w,
					 double body_x, double body_y, double body_z,
					 double body_roll, double body_pitch, double body_yaw,
					 double pos_roll, double pos_pitch,
					 double zero_w, double zero_l, double zero_x, double zero_y)
{
	matrix_t *m_t, *m_rx, *m_ry, *m_rz;

	strcpy(pRoobt->Move.gait, gait);
	pRoobt->Move.cycle = cycle;
	pRoobt->Move.interval = interval;
	pRoobt->Move.duty_ratio = dutyratio;
	pRoobt->Move.span_x = span_x;
	pRoobt->Move.span_y = span_y;
	pRoobt->Move.span_z = span_z;
	pRoobt->Move.span_w = span_w;

	pRoobt->Pose.body_x = body_x;
	pRoobt->Pose.body_y = body_y;
	pRoobt->Pose.body_z = body_z;
	pRoobt->Pose.body_ro = body_roll;
	pRoobt->Pose.body_pi = body_pitch;
	pRoobt->Pose.body_ya = body_yaw;

	pRoobt->Pose.pos_ro = pos_roll;
	pRoobt->Pose.pos_pi = pos_pitch;

	pRoobt->Zero.width = zero_w;
	pRoobt->Zero.length = zero_l;
	pRoobt->Zero.centre_x = zero_x;
	pRoobt->Zero.centre_y = zero_y;

	m_rx = cmat_se3_rx(body_roll);
	m_ry = cmat_se3_ry(body_pitch);
	m_rz = cmat_se3_rz(body_yaw);
	m_t = cmat_se3(body_x, body_y, body_z);
	pRoobt->Mat.RefFrm2BodyFrm = cmat_malloc(4, 4);
	cmat_multiply_multi(pRoobt->Mat.RefFrm2BodyFrm, 4, m_t, m_rx, m_ry, m_rz);
	cmat_free_multi(4, m_t, m_rx, m_ry, m_rz);

	m_rx = cmat_se3_rx(pos_roll);
	m_ry = cmat_se3_ry(pos_pitch);
	pRoobt->Mat.RefFrm2PosFrm = cmat_malloc(4, 4);
	cmat_multiply_multi(pRoobt->Mat.RefFrm2PosFrm, 2, m_rx, m_ry);
	cmat_free_multi(2, m_rx, m_ry);

	pRoobt->Mat.BodyFrm2LegFrm[RC_LEG_LF] = pRoobt->Mat.Base[RC_LEG_LF];
	pRoobt->Mat.BodyFrm2LegFrm[RC_LEG_LH] = pRoobt->Mat.Base[RC_LEG_LH];
	pRoobt->Mat.BodyFrm2LegFrm[RC_LEG_RF] = pRoobt->Mat.Base[RC_LEG_RF];
	pRoobt->Mat.BodyFrm2LegFrm[RC_LEG_RH] = pRoobt->Mat.Base[RC_LEG_RH];

	pRoobt->Mat.PosFrm2ZeroFrm[RC_LEG_LF] = cmat_se3(zero_x + zero_l / 2, zero_y + zero_w / 2, 0);
	pRoobt->Mat.PosFrm2ZeroFrm[RC_LEG_LH] = cmat_se3(zero_x - zero_l / 2, zero_y + zero_w / 2, 0);
	pRoobt->Mat.PosFrm2ZeroFrm[RC_LEG_RF] = cmat_se3(zero_x + zero_l / 2, zero_y - zero_w / 2, 0);
	pRoobt->Mat.PosFrm2ZeroFrm[RC_LEG_RH] = cmat_se3(zero_x - zero_l / 2, zero_y - zero_w / 2, 0);

	for (int i = 0; i < 4; i++)
	{
		pRoobt->Mat.ZeroFrm2p[i] = cmat_zeros(4, 4);
		pRoobt->Mat.PosFrm2p[i] = cmat_zeros(4, 4);
		pRoobt->Mat.RefFrm2p[i] = cmat_zeros(4, 4);
		pRoobt->Mat.BodyFrm2p[i] = cmat_zeros(4, 4);
		pRoobt->Mat.LegFrm2p[i] = cmat_zeros(4, 4);
	}
}

void RC_Update_BodyPose(RC_Robot_t *pRoobt,
						double body_x, double body_y, double body_z,
						double body_roll, double body_pitch, double body_yaw)
{
	matrix_t *m_t, *m_rx, *m_ry, *m_rz;

	pRoobt->Pose.body_x = body_x;
	pRoobt->Pose.body_y = body_y;
	pRoobt->Pose.body_z = body_z;
	pRoobt->Pose.body_ro = body_roll;
	pRoobt->Pose.body_pi = body_pitch;
	pRoobt->Pose.body_ya = body_yaw;

	m_rx = cmat_se3_rx(body_roll);
	m_ry = cmat_se3_ry(body_pitch);
	m_rz = cmat_se3_rz(body_yaw);
	m_t = cmat_se3(body_x, body_y, body_z);
	cmat_multiply_multi(pRoobt->Mat.RefFrm2BodyFrm, 4, m_t, m_rx, m_ry, m_rz);
	cmat_free_multi(4, m_t, m_rx, m_ry, m_rz);
}

void RC_Update_PosPose(RC_Robot_t *pRoobt,
					   double pos_roll, double pos_pitch)
{
	matrix_t *m_rx, *m_ry;

	pRoobt->Pose.pos_ro = pos_roll;
	pRoobt->Pose.pos_pi = pos_pitch;

	m_rx = cmat_se3_rx(pos_roll);
	m_ry = cmat_se3_ry(pos_pitch);
	cmat_multiply_multi(pRoobt->Mat.RefFrm2PosFrm, 2, m_rx, m_ry);
	cmat_free_multi(2, m_rx, m_ry);
}

void RC_Update_ZeroPara(RC_Robot_t *pRoobt,
						double zero_w, double zero_l, double zero_x, double zero_y)
{
	pRoobt->Zero.width = zero_w;
	pRoobt->Zero.length = zero_l;
	pRoobt->Zero.centre_x = zero_x;
	pRoobt->Zero.centre_y = zero_y;

	for (int i = 0; i < 4; i++)
		cmat_free(pRoobt->Mat.PosFrm2ZeroFrm[i]);

	pRoobt->Mat.PosFrm2ZeroFrm[RC_LEG_LF] = cmat_se3(zero_x + zero_l / 2, zero_y + zero_w / 2, 0);
	pRoobt->Mat.PosFrm2ZeroFrm[RC_LEG_LH] = cmat_se3(zero_x - zero_l / 2, zero_y + zero_w / 2, 0);
	pRoobt->Mat.PosFrm2ZeroFrm[RC_LEG_RF] = cmat_se3(zero_x + zero_l / 2, zero_y - zero_w / 2, 0);
	pRoobt->Mat.PosFrm2ZeroFrm[RC_LEG_RH] = cmat_se3(zero_x - zero_l / 2, zero_y - zero_w / 2, 0);
}

void RC_Calc_FootTraj__(RC_Robot_t *pRoobt, double phase, double *pos_x, double *pos_y, double *pos_z, double *omega)
{
	float span_x = pRoobt->Move.span_x;
	float span_y = pRoobt->Move.span_y;
	float span_z = pRoobt->Move.span_z;
	float span_w = pRoobt->Move.span_w;
	float spd_x = pRoobt->Move.span_x / pRoobt->Move.duty_ratio;
	float spd_y = pRoobt->Move.span_y / pRoobt->Move.duty_ratio;
	float spd_w = pRoobt->Move.span_w / pRoobt->Move.duty_ratio;
	float interval = pRoobt->Move.interval;

	TP_QuinticPoly_t DirX_Seg[3];
	TP_QuinticPoly_t DirY_Seg[3];
	TP_QuinticPoly_t DirW_Seg[3];
	TP_QuinticPoly_t DirZ_Seg[3];

	TP_Init_QuinticPoly(&DirX_Seg[0], -span_x / 2, -spd_x, 0, 0, 2 * spd_x, 0, (1 - pRoobt->Move.duty_ratio) / 2, interval);
	TP_Init_QuinticPoly(&DirX_Seg[1], 0, 2 * spd_x, 0, span_x / 2, -spd_x, 0, (1 - pRoobt->Move.duty_ratio) / 2, interval);
	TP_Init_QuinticPoly(&DirX_Seg[2], span_x / 2, -spd_x, 0, -span_x / 2, -spd_x, 0, pRoobt->Move.duty_ratio, interval);
	TP_Init_QuinticPoly(&DirY_Seg[0], -span_y / 2, -spd_y, 0, 0, 2 * spd_y, 0, (1 - pRoobt->Move.duty_ratio) / 2, interval);
	TP_Init_QuinticPoly(&DirY_Seg[1], 0, 2 * spd_y, 0, span_y / 2, -spd_y, 0, (1 - pRoobt->Move.duty_ratio) / 2, interval);
	TP_Init_QuinticPoly(&DirY_Seg[2], span_y / 2, -spd_y, 0, -span_y / 2, -spd_y, 0, pRoobt->Move.duty_ratio, interval);
	TP_Init_QuinticPoly(&DirZ_Seg[0], 0, 0, 0, span_z, 0, 0, (1 - pRoobt->Move.duty_ratio) / 2, interval);
	TP_Init_QuinticPoly(&DirZ_Seg[1], span_z, 0, 0, 0, 0, 0, (1 - pRoobt->Move.duty_ratio) / 2, interval);
	TP_Init_QuinticPoly(&DirZ_Seg[2], 0, 0, 0, 0, 0, 0, pRoobt->Move.duty_ratio, interval);
	TP_Init_QuinticPoly(&DirW_Seg[0], -span_w / 2, -spd_w, 0, span_w / 2, -spd_w, 0, (1 - pRoobt->Move.duty_ratio) / 2, interval);
	TP_Init_QuinticPoly(&DirW_Seg[1], 0, 2 * spd_w, 0, span_w / 2, -spd_w, 0, (1 - pRoobt->Move.duty_ratio) / 2, interval);
	TP_Init_QuinticPoly(&DirW_Seg[2], span_w / 2, -spd_w, 0, -span_w / 2, -spd_w, 0, pRoobt->Move.duty_ratio, interval);

	if (0 <= phase && phase <= (1 - pRoobt->Move.duty_ratio) / 2)
	{
		*pos_x = TP_Calc_QuinticPoly__(&DirX_Seg[0], phase);
		*pos_y = TP_Calc_QuinticPoly__(&DirY_Seg[0], phase);
		*pos_z = TP_Calc_QuinticPoly__(&DirZ_Seg[0], phase);
		*omega = TP_Calc_QuinticPoly__(&DirW_Seg[0], phase);
	}
	else if ((1 - pRoobt->Move.duty_ratio) / 2 < phase && phase <= 1 - pRoobt->Move.duty_ratio)
	{
		*pos_x = TP_Calc_QuinticPoly__(&DirX_Seg[1], phase - (1 - pRoobt->Move.duty_ratio) / 2);
		*pos_y = TP_Calc_QuinticPoly__(&DirY_Seg[1], phase - (1 - pRoobt->Move.duty_ratio) / 2);
		*pos_z = TP_Calc_QuinticPoly__(&DirZ_Seg[1], phase - (1 - pRoobt->Move.duty_ratio) / 2);
		*omega = TP_Calc_QuinticPoly__(&DirW_Seg[1], phase - (1 - pRoobt->Move.duty_ratio) / 2);
	}
	else if (1 - pRoobt->Move.duty_ratio < phase && phase <= 1.0)
	{
		*pos_x = TP_Calc_QuinticPoly__(&DirX_Seg[2], phase - (1 - pRoobt->Move.duty_ratio));
		*pos_y = TP_Calc_QuinticPoly__(&DirY_Seg[2], phase - (1 - pRoobt->Move.duty_ratio));
		*pos_z = TP_Calc_QuinticPoly__(&DirZ_Seg[2], phase - (1 - pRoobt->Move.duty_ratio));
		*omega = TP_Calc_QuinticPoly__(&DirW_Seg[2], phase - (1 - pRoobt->Move.duty_ratio));
	}
}

void RC_Calc_FootTraj(RC_Robot_t *pRoobt, double phase_, matrix_t *m_angle)
{
	float span_w = pRoobt->Move.span_w;

	float phase[4];

	double pos_x = 0, pos_y = 0, pos_z = 0, omega[4] = {0};
	matrix_t *rz, *inv;

	/*if (0 <= phase_ && phase_ < 0.5)
	{
		TP_Init_QuinticPoly(&DirW_Seg, 0, 0, 0, span_w, 0, 0, 0.5, pRoobt->Move.interval);
		omega = TP_Calc_QuinticPoly__(&DirW_Seg, phase_);
	}
	else if (0.5 <= phase_ && phase_ <= 1)
	{
		TP_Init_QuinticPoly(&DirW_Seg, span_w, 0, 0, 0, 0, 0, 0.5, pRoobt->Move.interval);
		omega = TP_Calc_QuinticPoly__(&DirW_Seg, phase_ - 0.5);
	}*/

	if (strcmp(pRoobt->Move.gait, "trot") == 0)
	{
		phase[0] = phase_;		 //LF
		phase[1] = phase_ + 0.5; //LH
		phase[2] = phase_ + 0.5; //RF
		phase[3] = phase_;		 //RH
	}
	else
		printf("Wrong type of gait.\n");

	for (int i = 0; i < 4; i++)
	{
		if (phase[i] >= 1.0f)
			phase[i] -= 1;
	}

	for (int i = 0; i < 4; i++)
		cmat_free(pRoobt->Mat.ZeroFrm2p[i]);
	RC_Calc_FootTraj__(pRoobt, phase[0], &pos_x, &pos_y, &pos_z, &omega[0]);
	pRoobt->Mat.ZeroFrm2p[RC_LEG_LF] = cmat_se3(pos_x, pos_y, pos_z);
	RC_Calc_FootTraj__(pRoobt, phase[1], &pos_x, &pos_y, &pos_z, &omega[1]);
	pRoobt->Mat.ZeroFrm2p[RC_LEG_LH] = cmat_se3(pos_x, pos_y, pos_z);
	RC_Calc_FootTraj__(pRoobt, phase[2], &pos_x, &pos_y, &pos_z, &omega[2]);
	pRoobt->Mat.ZeroFrm2p[RC_LEG_RF] = cmat_se3(pos_x, pos_y, pos_z);
	RC_Calc_FootTraj__(pRoobt, phase[3], &pos_x, &pos_y, &pos_z, &omega[3]);
	pRoobt->Mat.ZeroFrm2p[RC_LEG_RH] = cmat_se3(pos_x, pos_y, pos_z);

	for (int i = 0; i < 4; i++)
	{
		rz = cmat_se3_rz(omega[i]);
		cmat_multiply_multi(pRoobt->Mat.PosFrm2p[i], 3, rz, pRoobt->Mat.PosFrm2ZeroFrm[i], pRoobt->Mat.ZeroFrm2p[i]);
		cmat_free(rz);
	}

	for (int i = 0; i < 4; i++)
		cmat_multiply(pRoobt->Mat.RefFrm2PosFrm, pRoobt->Mat.PosFrm2p[i], pRoobt->Mat.RefFrm2p[i]);

	inv = cmat_se3_homo_inv(pRoobt->Mat.RefFrm2BodyFrm);
	for (int i = 0; i < 4; i++)
		cmat_multiply(inv, pRoobt->Mat.RefFrm2p[i], pRoobt->Mat.BodyFrm2p[i]);
	cmat_free(inv);

	for (int i = 0; i < 4; i++)
	{
		inv = cmat_se3_homo_inv(pRoobt->Mat.BodyFrm2LegFrm[i]);
		cmat_multiply(inv, pRoobt->Mat.BodyFrm2p[i], pRoobt->Mat.LegFrm2p[i]);
		cmat_free(inv);
	}

	matrix_t *m_temp;
	for (int i = 0; i < 4; i++)
	{
		m_temp = cmat_se3_ext_t(pRoobt->Mat.LegFrm2p[i]);
		for (int j = 0; j < 3; j++)
			cmat_set(m_angle, j, i, cmat_get(m_temp, j, 0));
		cmat_free(m_temp);
	}
}

void RC_InvKine(RC_Robot_t *pRoobt, matrix_t *m_pos, matrix_t *m_angle)
{
	double temp1, temp2;
	if (m_pos->cols != 4 || m_pos->rows != 3)
	{
		printf("Abnormal pos matrix. Program will stop to aviod accident.\n");
		while (1)
			;
	}

	double rho, C3, S3;
	if (strcmp(pRoobt->Mech.type, "elbow-elbow") == 0)
	{
		for (int i = 0; i < 4; i++)
		{
			rho = sqrt(pow(cmat_get(m_pos, 0, i), 2) + pow(cmat_get(m_pos, 1, i), 2));
			if (i < 2)
				cmat_set(m_angle, 0, i, atan2(pRoobt->Mech.leg_d / rho, sqrt(1 - pow((pRoobt->Mech.leg_d / rho), 2))) + atan2(cmat_get(m_pos, 1, i), cmat_get(m_pos, 0, i)));
			//cmat_set(m_angle, 0, i, atan2(pRoobt->Mech.leg_d / rho, -sqrt(1 - pow((pRoobt->Mech.leg_d / rho), 2))) + atan2(cmat_get(m_pos, 1, i), cmat_get(m_pos, 0, i)));
			else
				cmat_set(m_angle, 0, i, atan2(-pRoobt->Mech.leg_d / rho, sqrt(1 - pow((pRoobt->Mech.leg_d / rho), 2))) + atan2(cmat_get(m_pos, 1, i), cmat_get(m_pos, 0, i)));
			//cmat_set(m_angle, 0, i, atan2(-pRoobt->Mech.leg_d / rho, -sqrt(1 - pow((pRoobt->Mech.leg_d / rho), 2))) + atan2(cmat_get(m_pos, 1, i), cmat_get(m_pos, 0, i)));

			C3 = (pow(cmat_get(m_pos, 0, i), 2) + pow(cmat_get(m_pos, 1, i), 2) + pow(cmat_get(m_pos, 2, i), 2) - pow(pRoobt->Mech.leg_a1, 2) - pow(pRoobt->Mech.leg_a2, 2) - pow(pRoobt->Mech.leg_d, 2)) / (2 * pRoobt->Mech.leg_a1 * pRoobt->Mech.leg_a2);
			if (i < 2)
				S3 = -sqrt(1 - pow(C3, 2));
			else
				S3 = sqrt(1 - pow(C3, 2));

			cmat_set(m_angle, 2, i, atan2(S3, C3));

			rho = sqrt(pow(pRoobt->Mech.leg_a1 + pRoobt->Mech.leg_a2 * C3, 2) + pow((pRoobt->Mech.leg_a2 * S3), 2));
			if (i < 2)
				cmat_set(m_angle, 1, i, atan2(pRoobt->Mech.leg_a1 + pRoobt->Mech.leg_a2 * C3, pRoobt->Mech.leg_a2 * S3) - atan2(sqrt(1 - pow(cmat_get(m_pos, 2, i) / rho, 2)), cmat_get(m_pos, 2, i) / rho));
			//cmat_set(m_angle, 1, i, atan2(pRoobt->Mech.leg_a1 + pRoobt->Mech.leg_a2*C3, pRoobt->Mech.leg_a2*S3) - atan2(-sqrt(1 - pow(cmat_get(m_pos, 2, i) / rho, 2)), cmat_get(m_pos, 2, i) / rho));
			else
				cmat_set(m_angle, 1, i, atan2(pRoobt->Mech.leg_a1 + pRoobt->Mech.leg_a2 * C3, pRoobt->Mech.leg_a2 * S3) - atan2(sqrt(1 - pow(cmat_get(m_pos, 2, i) / rho, 2)), -cmat_get(m_pos, 2, i) / rho));
			//cmat_set(m_angle, 1, i, atan2(pRoobt->Mech.leg_a1 + pRoobt->Mech.leg_a2*C3, pRoobt->Mech.leg_a2*S3) - atan2(-sqrt(1 - pow(cmat_get(m_pos, 2, i) / rho, 2)), -cmat_get(m_pos, 2, i) / rho));
		}
	}
}

void RC_AngleCorrect(RC_Robot_t *pRoobt, matrix_t *m_angle)
{
	if (strcmp(pRoobt->Mech.type, "elbow-elbow") == 0)
	{
		//因为实际电机零位定义与建模零位不一致，需要将计算角度修正为实际角度，本函数需要根据电机零位的确定方式而进行修改
		//当前零位定义为“L L”
		cmat_set(m_angle, 0, RC_LEG_LF, cmat_get(m_angle, 0, RC_LEG_LF));
		cmat_set(m_angle, 1, RC_LEG_LF, -cmat_get(m_angle, 1, RC_LEG_LF));
		cmat_set(m_angle, 2, RC_LEG_LF, pi / 2 + cmat_get(m_angle, 2, RC_LEG_LF));

		cmat_set(m_angle, 0, RC_LEG_LH, -cmat_get(m_angle, 0, RC_LEG_LH));
		cmat_set(m_angle, 1, RC_LEG_LH, -cmat_get(m_angle, 1, RC_LEG_LH));
		cmat_set(m_angle, 2, RC_LEG_LH, pi / 2 + cmat_get(m_angle, 2, RC_LEG_LH));

		cmat_set(m_angle, 0, RC_LEG_RF, cmat_get(m_angle, 0, RC_LEG_RF));
		cmat_set(m_angle, 1, RC_LEG_RF, -cmat_get(m_angle, 1, RC_LEG_RF));
		cmat_set(m_angle, 2, RC_LEG_RF, cmat_get(m_angle, 2, RC_LEG_RF) - pi / 2);

		cmat_set(m_angle, 0, RC_LEG_RH, -cmat_get(m_angle, 0, RC_LEG_RH));
		cmat_set(m_angle, 1, RC_LEG_RH, -cmat_get(m_angle, 1, RC_LEG_RH));
		cmat_set(m_angle, 2, RC_LEG_RH, cmat_get(m_angle, 2, RC_LEG_RH) - pi / 2);
	}
}

void RC_DispPara(RC_Robot_t *pRoobt)
{
	for (int i = 0; i < 4; i++)
	{
		switch (i)
		{
		case RC_LEG_LF:
			printf("\n[LEG_LF]\n");
			break;
		case RC_LEG_LH:
			printf("\n[LEG_LH]\n");
			break;
		case RC_LEG_RF:
			printf("\n[LEG_RF]\n");
			break;
		case RC_LEG_RH:
			printf("\n[LEG_RH]\n");
			break;
		}
		printf("Mat.ZeroFrm2p:\n");
		cmat_display(pRoobt->Mat.ZeroFrm2p[i]);
		printf("Mat.PosFrm2p:\n");
		cmat_display(pRoobt->Mat.PosFrm2p[i]);
		printf("Mat.RefFrm2p:\n");
		cmat_display(pRoobt->Mat.RefFrm2p[i]);
		printf("Mat.BodyFrm2p:\n");
		cmat_display(pRoobt->Mat.BodyFrm2p[i]);
		printf("Mat.LegFrm2p:\n");
		cmat_display(pRoobt->Mat.LegFrm2p[i]);
	}
}
