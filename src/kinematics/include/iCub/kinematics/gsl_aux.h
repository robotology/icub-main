
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>

#ifndef __GSL_AUX_H__
#define __GSL_AUX_H__

void gsl_vector_inv_elements( gsl_vector* V );
void gsl_matrix_set_diagonal( gsl_matrix* M, gsl_vector* V);
void gsl_matrix_sum( gsl_matrix* M1, gsl_matrix* M2, gsl_matrix* Mt);
void gsl_vector_setfromfloat( gsl_vector* V, double* vf);
void gsl_vector_setfromdouble( gsl_vector* V, double* vf);
void gsl_vector_gettofloat( gsl_vector* V, double* vf);
void gsl_copy_submatrix( gsl_matrix *dest, const gsl_matrix *orig, int lin, int col, int nlin, int ncol);
void gsl_print_vector( gsl_vector* M, const char *name);
void gsl_print_matrix( gsl_matrix* M, const char *name);

float gsl_matrix_pseudoinv( gsl_matrix* M, gsl_matrix* Minv, double tol);

#endif /* __GSL_AUX_H__ */
