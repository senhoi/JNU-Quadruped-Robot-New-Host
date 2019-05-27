#include <stdio.h>
#include <stdlib.h>

#include "cmat.h"

int cmat_has_same_dimensions(matrix_t* m, matrix_t* n){
	return (m->rows == n->rows && m->cols == n->cols);
}

void cmat_for_each(matrix_t* m, double (*repeat)(double)){
	int i, j;

	for(i = 0; i < m->rows; i++){
		for(j = 0; j < m->cols; j++){
			cmat_set(m, i, j, (*repeat)(cmat_get(m, i, j)));
		}
	}
}

void cmat_for_each_row(matrix_t* m, void (*repeat)(matrix_t*, int)){
	int i, j;
	matrix_t* row = cmat_malloc(1, m->cols);

	for(i = 0; i < m->rows; i++){
		for(j = 0; j < m->cols; j++){
			cmat_set(row, 0, j, cmat_get(m, i, j));
		}
		(*repeat)(row, i);
		for(j = 0; j < m->cols; j++){
			cmat_set(m, i, j, cmat_get(row, 0, j));
		}
	}
}

int cmat_to_file(matrix_t* m, const char* file){
	int i, j;
	FILE* fp;

	fp = fopen(file, "w");
	if(fp == NULL)
		return CMAT_FAIL;
	// fprintf(fp, "%d %d\n", m->rows, m->cols);
	for(i = 0; i < m->rows; i++){
		for(j = 0; j < m->cols; j++){
			fprintf(fp, "%f", cmat_get(m, i, j));
			if(j != m->cols - 1)
				fprintf(fp, ",");
		}
		fprintf(fp, "\n");
	}
	fclose(fp);
	return CMAT_SUCCESS;
}

void cmat_abort(char* msg){
	fprintf(stderr, "%s\n", msg);
	exit(1);
}

void cmat_row_exchange(matrix_t *m, int row1, int row2) {
	double temp;
	for (int i = 0; i < m->cols; i++) {
		temp = cmat_get(m, row1, i);
		cmat_set(m, row1, i, cmat_get(m, row2, i));
		cmat_set(m, row2, i, temp);
	}
}

void cmat_col_exchange(matrix_t *m, int col1, int col2) {
	double temp;
	for (int i = 0; i < m->rows; i++) {
		temp = cmat_get(m, i, col1);
		cmat_set(m, i, col1, cmat_get(m, i, col2));
		cmat_set(m, i, col2, temp);
	}
}