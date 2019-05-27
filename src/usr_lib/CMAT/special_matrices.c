#include <stdio.h>
#include <stdlib.h>

#include "cmat.h"

matrix_t* cmat_zeros(int rows, int cols){
    matrix_t *m = (matrix_t*) malloc(sizeof(matrix_t));
    m->rows = rows;
    m->cols = cols;
    m->data = (double*) calloc(rows*cols, sizeof(double));
    return m;
}

matrix_t* cmat_ones(int rows, int cols){
    int i, j;
    matrix_t *m = cmat_malloc(rows, cols);

    for(i = 0; i < rows; i++){
        for(j = 0; j < cols; j++){
            cmat_set(m, i, j, 1.0);
        }
    }
    return m;
}

matrix_t* cmat_identity(int dim){
    int i;
    matrix_t* identity = cmat_zeros(dim, dim);

    for(i = 0; i < dim; i++){
        cmat_set(identity, i, i, 1);
    }
    return identity;
}

matrix_t* cmat_rand(int rows, int cols){
    int i, j;
    matrix_t* m = cmat_malloc(rows, cols);

    for(i = 0; i < rows; i++){
        for(j = 0; j < cols; j++){

        }
    }
    return m;
}

matrix_t* cmat_upper_trig(int dim){
    int i, j;
    matrix_t* result = cmat_zeros(dim, dim);

    for(i = 0; i < dim; i++){
        for(j = i; j < dim; j++){
            cmat_set(result, i, j, 1);
        }
    }
    return result;
}

matrix_t* cmat_lower_trig(int dim){
    int i, j;
    matrix_t* result = cmat_zeros(dim, dim);

    for(i = 0; i < dim; i++){
        for(j = 0; j <= i; j++){
            cmat_set(result, i, j, 1);
        }
    }
    return result;
}

matrix_t* cmat_n(int rows, int cols){
    int i, j;
    matrix_t* result = cmat_malloc(rows, cols);

    for(i = 0; i < rows; i++){
        for(j = 0; j < cols; j++){
            cmat_set(result, i, j, j + (i * 3));
        }
    }
    return result;
}
