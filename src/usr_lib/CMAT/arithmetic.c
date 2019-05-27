#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "cmat.h"

int cmat_add_const(matrix_t* m, double a, matrix_t* result) {
	int i, j;

	if (!cmat_has_same_dimensions(m, result))
		return CMAT_FAIL;

	for (i = 0; i < m->rows; i++) {
		for (j = 0; j < m->cols; j++)
			cmat_set(result, i, j, cmat_get(m, i, j) + a);
	}
	return CMAT_SUCCESS;
}

int cmat_multiply_const(matrix_t* m, double a, matrix_t* result) {
	int i, j;

	if (!cmat_has_same_dimensions(m, result))
		return CMAT_FAIL;

	for (i = 0; i < m->rows; i++) {
		for (j = 0; j < m->cols; j++)
			cmat_set(result, i, j, cmat_get(m, i, j) * a);
	}
	return CMAT_SUCCESS;
}

int cmat_add(matrix_t* m, matrix_t* n, matrix_t* result) {
	int i, j;

	if (!cmat_has_same_dimensions(m, n) || !cmat_has_same_dimensions(m, result))
		return CMAT_FAIL;

	for (i = 0; i < m->rows; i++) {
		for (j = 0; j < m->cols; j++)
			cmat_set(result, i, j, cmat_get(m, i, j) + cmat_get(n, i, j));
	}
	return CMAT_SUCCESS;
}

int cmat_add_to_rows(matrix_t* m, matrix_t* n, matrix_t* result) {
	int i, j;

	// N has to be a row matrix with same columns as M
	if (!cmat_has_same_dimensions(m, result) || m->cols != n->cols || n->rows != 1)
		return CMAT_FAIL;

	for (i = 0; i < m->rows; i++) {
		for (j = 0; j < m->cols; j++)
			cmat_set(result, i, j, cmat_get(m, i, j) + cmat_get(n, 0, j));
	}
	return CMAT_SUCCESS;
}

int cmat_subtract(matrix_t* m, matrix_t* n, matrix_t* result) {
	int i, j;

	if (!cmat_has_same_dimensions(m, n) || !cmat_has_same_dimensions(m, result))
		return CMAT_FAIL;

	for (i = 0; i < m->rows; i++) {
		for (j = 0; j < m->cols; j++)
			cmat_set(result, i, j, cmat_get(m, i, j) - cmat_get(n, i, j));
	}
	return CMAT_SUCCESS;
}

int cmat_divide_each(matrix_t* m, matrix_t* n, matrix_t* result) {
	int i, j;

	if (!cmat_has_same_dimensions(m, n) || !cmat_has_same_dimensions(m, result))
		return CMAT_FAIL;

	for (i = 0; i < m->rows; i++) {
		for (j = 0; j < m->cols; j++)
			cmat_set(result, i, j, cmat_get(m, i, j) / cmat_get(n, i, j));
	}
	return CMAT_SUCCESS;
}

int cmat_multiply_each(matrix_t* m, matrix_t* n, matrix_t* result) {
	int i, j;

	if (!cmat_has_same_dimensions(m, n) || !cmat_has_same_dimensions(m, result))
		return CMAT_FAIL;

	for (i = 0; i < m->rows; i++) {
		for (j = 0; j < m->cols; j++)
			cmat_set(result, i, j, cmat_get(m, i, j) * cmat_get(n, i, j));
	}
	return CMAT_SUCCESS;
}

int cmat_multiply(matrix_t* m, matrix_t* n, matrix_t* result) {
	int i, j, k;
	double tmp;

	if (m->cols != n->rows)
		return CMAT_FAIL;

	// TODO: check if result is of proper dimension

	for (i = 0; i < m->cols; i++){
		for (j = 0; j < n->rows; j++) {
			tmp = 0;
			for (k = 0; k < m->rows; k++) {
				tmp += cmat_get(m, i, k) * cmat_get(n, k, j);
			}
			cmat_set(result, i, j, tmp);
		}
	}

	return CMAT_SUCCESS;
}

int cmat_multiply_multi(matrix_t* result, int num, ...) {
	// TODO: check if result is right
	float sum = 0;
	matrix_t *m_para[10];
	matrix_t *m_prev, *m_late;

	va_list argp;
	va_start(argp, num);
	if (num < 2)
		return CMAT_FAIL;
	for (int i = 0; i < num; i++) 
		m_para[i] = va_arg(argp, matrix_t*);
	va_end(argp);

	m_prev = m_para[0];
	for (int i = 1; i < num; i++)
	{
		m_late = m_para[i];
		cmat_multiply(m_prev, m_late, result);
		m_prev = result;
	}

	return CMAT_SUCCESS;
}

matrix_t* cmat_sum_x(matrix_t* m) {
	matrix_t* result = cmat_malloc(m->rows, 1);
	int i, j, t;

	for (i = 0; i < m->rows; i++) {
		t = 0;
		for (j = 0; j < m->cols; j++) {
			t += cmat_get(m, i, j);
		}
		cmat_set(result, i, 0, t);
	}

	return result;
}

matrix_t* cmat_sum_y(matrix_t* m) {
	matrix_t* result = cmat_malloc(1, m->cols);
	int i, j, t;

	for (i = 0; i < m->cols; i++) {
		t = 0;
		for (j = 0; j < m->rows; j++) {
			t += cmat_get(m, j, i);
		}
		cmat_set(result, 0, i, t);
	}

	return result;
}