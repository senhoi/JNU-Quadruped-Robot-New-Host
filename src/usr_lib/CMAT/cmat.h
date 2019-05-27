#ifndef CMAT_H
#define CMAT_H

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CMAT_FAIL, CMAT_SUCCESS, CMAT_DIMENSION_MISMATCHED
};

typedef struct matrix_t {
    double *data; /**< Array where data is stored */
    int rows, /**< Number of rows in the matrix */
    	cols; /**< Number of columns in the matrix */
}matrix_t;

/** 
 * @brief Allocates memory for a matrix
 *
 * Matrices allocated with this method should be freed with
 * cmat_free().
 *
 * @param rows Number of rows in the matrix
 * @param cols Number of columns in the matrix
 *
 * @return Pointer to the allocated matrix
 */
matrix_t *cmat_malloc(int rows, int cols);

/** 
 * @brief Allocates memory for a matrix with same dimensions as `m`
 *
 * Matrices allocated with this method should be freed with
 * cmat_free().
 *
 * @param m pointer to the matrix
 *
 * @return Pointer to the allocated matrix
 */
matrix_t *cmat_malloc_shape(matrix_t *m);


/** 
 * @brief Creates a matrix from file
 *
 * It scans the `file` for `rows`x`cols` matrix, creates
 * the matrix and returns the pointer to it.
 *
 * Each element must be saperated by comma and each row must]
 * be saperated by a new line in the `file`.
 *
 * @param rows Number of rows in the matrix
 * @param cols Number of columns in the matrix
 * @param file Name of the input file
 *
 * @return Pointer to the allocated matrix
 */
matrix_t *cmat_from_file(int rows, int cols, const char *file);

void cmat_free(matrix_t *);

void cmat_free_multi(int, ...);

int cmat_set(matrix_t *, int, int, double);

double cmat_get(matrix_t *, int, int);

// Special matrices
matrix_t *cmat_zeros(int, int);

matrix_t *cmat_ones(int, int);

matrix_t *cmat_n(int, int);

matrix_t *cmat_rand(int, int);

matrix_t *cmat_identity(int);

matrix_t *cmat_upper_trig(int);

matrix_t *cmat_lower_trig(int);

// Arithmetic
int cmat_add_const(matrix_t *, double, matrix_t *);

int cmat_multiply_const(matrix_t *, double, matrix_t *);

int cmat_add(matrix_t *, matrix_t *, matrix_t *);

int cmat_add_to_rows(matrix_t *, matrix_t *, matrix_t *);

int cmat_subtract(matrix_t *, matrix_t *, matrix_t *);

int cmat_divide_each(matrix_t *, matrix_t *, matrix_t *);

int cmat_multiply_each(matrix_t *, matrix_t *, matrix_t *);

int cmat_multiply(matrix_t *, matrix_t *, matrix_t *);

int cmat_multiply_multi(matrix_t* result, int num, ...);

// Operations
int cmat_transpose(matrix_t *, matrix_t *);

int cmat_normalize(matrix_t *, matrix_t *);

matrix_t *cmat_sum_x(matrix_t *);

matrix_t *cmat_sum_y(matrix_t *);

matrix_t *cmat_submatrix(matrix_t *, int, int, int, int);

int cmat_determinant(matrix_t *, double *);

int cmat_lu_decompose(matrix_t *, matrix_t *, matrix_t *);

// Views
void cmat_display(matrix_t *);

// Utils
void cmat_abort(char *);

void cmat_for_each(matrix_t *, double (*)(double));

void cmat_for_each_row(matrix_t *, void (*)(matrix_t *, int));

int cmat_has_same_dimensions(matrix_t *, matrix_t *);

int cmat_to_file(matrix_t *, const char *);

void cmat_row_exchange(matrix_t*, int, int);

void cmat_col_exchange(matrix_t*, int, int);

//Robotic tools
matrix_t* cmat_se3(double px, double py, double pz);

matrix_t* cmat_se3_rx(double rad);

matrix_t* cmat_se3_ry(double rad);

matrix_t* cmat_se3_rz(double rad);

matrix_t* cmat_se3_ext_r(matrix_t* m);

matrix_t* cmat_se3_ext_t(matrix_t* m);

matrix_t* cmat_se3_homo_inv(matrix_t* m);

#ifdef __cplusplus
}
#endif

#endif
