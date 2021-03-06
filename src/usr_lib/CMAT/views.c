#include <stdio.h>

#include "cmat.h"

void cmat_display(matrix_t* m){
	int i, j;
	printf("%dx%d matrix\n", m->rows, m->cols);
	for(i = 0; i < m->rows; i++){
		for(j = 0; j < m->cols; j++){
			printf(" %7.3f\t", cmat_get(m, i, j));
		}
		printf("\n");
	}
}