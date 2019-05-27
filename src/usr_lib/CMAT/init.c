#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "cmat.h"

matrix_t* cmat_malloc(int rows, int cols) {
	matrix_t *m = (matrix_t*)malloc(sizeof(matrix_t));
	m->rows = rows;
	m->cols = cols;
	m->data = (double*)malloc(sizeof(double)*rows*cols);
	if (m->data == NULL)
	{
		printf("cmat malloc failed.\n");
		return NULL;
	}
	return m;
}

matrix_t* cmat_malloc_shape(matrix_t* m) {
	return cmat_malloc(m->rows, m->cols);
}

matrix_t* cmat_from_file(int rows, int cols, const char* file) {
	int i, j;
	FILE* fp;
	double t;
	matrix_t *m = cmat_malloc(rows, cols);

	fp = fopen(file, "r");
	for (i = 0; i < rows; i++) {
		for (j = 0; j < cols; j++) {
			fscanf(fp, "%lf,", &t);
			cmat_set(m, i, j, t);
		}
	}
	fclose(fp);

	return m;
}

void cmat_free(matrix_t* m) {
	free(m->data);
	free(m);	//结构体指针也需要被释放！！！
}

void cmat_free_multi(int num, ...) {
	va_list argp;
	va_start(argp, num);
	for (int i = 0; i < num; i++) {
		cmat_free(va_arg(argp, matrix_t*));
	}
	va_end(argp);
}

int cmat_set(matrix_t* m, int row, int col, double data) {
	if (row < m->rows && col < m->cols) {
		m->data[col + (row * m->cols)] = data;
		return CMAT_SUCCESS;
	}
	return CMAT_FAIL;
}

double cmat_get(matrix_t* m, int row, int col) {
	if (row < m->rows && col < m->cols)
		return m->data[col + (row * m->cols)];
	else
		return 0;
}
