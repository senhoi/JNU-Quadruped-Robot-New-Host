#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "cmat.h"

matrix_t* cmat_se3(double px, double py, double pz)
{
	matrix_t *m = cmat_identity(4);

	cmat_set(m, 0, 3, px);
	cmat_set(m, 1, 3, py);
	cmat_set(m, 2, 3, pz);
	return m;
}

matrix_t* cmat_se3_rx(double rad)
{
	matrix_t *m = cmat_identity(4);

	cmat_set(m, 1, 1, cos(rad));
	cmat_set(m, 1, 2, sin(rad));
	cmat_set(m, 2, 1, -sin(rad));
	cmat_set(m, 2, 2, cos(rad));
	return m;
}

matrix_t* cmat_se3_ry(double rad)
{
	matrix_t *m = cmat_identity(4);

	cmat_set(m, 0, 0, cos(rad));
	cmat_set(m, 0, 2, sin(rad));
	cmat_set(m, 2, 0, -sin(rad));
	cmat_set(m, 2, 2, cos(rad));
	return m;
}

matrix_t* cmat_se3_rz(double rad)
{
	matrix_t *m = cmat_identity(4);

	cmat_set(m, 0, 0, cos(rad));
	cmat_set(m, 0, 1, sin(rad));
	cmat_set(m, 1, 0, -sin(rad));
	cmat_set(m, 1, 1, cos(rad));
	return m;
}

matrix_t* cmat_se3_ext_r(matrix_t* m)
{
	matrix_t *result = cmat_malloc(3, 3);

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cmat_set(result, i, j, cmat_get(m, i, j));

	return result;
}

matrix_t* cmat_se3_ext_t(matrix_t* m)
{
	matrix_t *result = cmat_malloc(3, 1);

	for (int i = 0; i < 3; i++)
		cmat_set(result, i, 0, cmat_get(m, i, 3));

	return result;
}

matrix_t* cmat_se3_homo_inv(matrix_t* m)
{
	matrix_t *R = cmat_se3_ext_r(m);
	matrix_t *P = cmat_se3_ext_t(m);
	matrix_t *R_ = cmat_malloc(3, 3);
	matrix_t *R__ = cmat_malloc(3, 3);
	matrix_t *P_ = cmat_malloc(3, 1);
	matrix_t *result = cmat_malloc(4, 4);

	cmat_transpose(R, R_);
	cmat_multiply_const(R_, -1, R__);
	cmat_multiply(R__, P, P_);

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cmat_set(result, i, j, cmat_get(R_, i, j));

	for (int j = 0; j < 3; j++)
		cmat_set(result, j, 3, cmat_get(P_, j, 0));

	for (int j = 0; j < 3; j++)
		cmat_set(result, 3, j, 0);
	cmat_set(result, 3, 3, 1);

	cmat_free_multi(5, R, P, R_, P_, R__);

	return result;
}