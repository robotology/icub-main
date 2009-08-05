#include <stdio.h>
#include <stdlib.h>

#include <iCub/kinematics/gsl_aux.h>

void gsl_vector_inv_elements( gsl_vector* V )
{
unsigned int i;
     
for (i = 0; i < V->size; i++)
        gsl_vector_set(V,i, 1/gsl_vector_get(V,i));
}

void gsl_matrix_set_diagonal( gsl_matrix* M, gsl_vector* V)
{
unsigned int cnt;

 for( cnt=0; cnt<V->size; cnt++ )    
         gsl_matrix_set( M, cnt, cnt, gsl_vector_get( V, cnt) );
         
}

void gsl_matrix_sum( gsl_matrix* M1, gsl_matrix* M2, gsl_matrix* Mt)
{
 for(unsigned int c1=0; c1<M1->size1; c1++ )    
 for(unsigned int c2=0; c2<M2->size2; c2++ )    
          gsl_matrix_set( Mt, c1, c2, gsl_matrix_get( M1, c1, c2)+gsl_matrix_get( M2, c1, c2) );
     
}

void gsl_vector_setfromfloat( gsl_vector* V, float* vf)
{
unsigned int cc;

    for(cc=0;cc<V->size;cc++)
      gsl_vector_set( V, cc, vf[cc]);
}

void gsl_vector_setfromdouble( gsl_vector* V, double* vf)
{
unsigned int cc;

    for(cc=0;cc<V->size;cc++)
      gsl_vector_set( V, cc, vf[cc]);
}

void gsl_vector_gettofloat( gsl_vector* V, double* vf)
{
unsigned int cc;

    for(cc=0;cc<V->size;cc++)
      vf[cc] = gsl_vector_get( V, cc);
}

void gsl_print_vector( gsl_vector* M, char *name)
{
unsigned int j;
     
printf("%s = \n", name);
for (j = 0; j < M->size; j++)
     printf("%7.4f ", gsl_vector_get(M, j));

printf("\n\n");

}

void gsl_print_matrix( gsl_matrix* M, char *name)
{
unsigned int i,j;
     
printf("%s = \n", name);
for (i = 0; i < M->size1; i++) {
    for (j = 0; j < M->size2; j++)
        printf("%7.4f ", gsl_matrix_get(M, i, j));
    printf("\n");
}
printf("\n");
}

void gsl_copy_submatrix( gsl_matrix *dest, const gsl_matrix *orig, int lin, int col, int nlin, int ncol)
{
double val;

	for(int cntl=0; cntl<nlin; cntl++)
		for(int cntc=0; cntc<ncol; cntc++)
		{
			val = gsl_matrix_get( orig, cntl+lin, cntc+col);
			gsl_matrix_set( dest, cntl, cntc, val);
		}
}

/******************************************************************
* void gsl_matrix_pseudoinv( gsl_matrix* M, gsl_matrix* Minv, float tol)
*
*   tol any singular values less than tol are treated as zero.
*   if tol<0 |tol| is a percentage of the larger singular value
*
*   M is ok after exiting the function
*                                                              
*   Minv should be initialized befores as:
*   gsl_matrix* Minv = gsl_matrix_calloc( NC, NL);
*******************************************************************/
float gsl_matrix_pseudoinv( gsl_matrix* M, gsl_matrix* Minv, double tol)
{
int NC,NL;
int cnt;
gsl_matrix* Maux;
float cond = 1;

  NL = M->size1;
  NC = M->size2;  
  
  if (M->size1 < M->size2)
  {
     gsl_matrix* Minvaux = gsl_matrix_alloc( NL, NC);
     
     Maux = gsl_matrix_alloc( NC, NL);
     gsl_matrix_transpose_memcpy( Maux, M);
     
     cond = gsl_matrix_pseudoinv( Maux, Minvaux, tol);

     gsl_matrix_transpose_memcpy( Minv, Minvaux);
     
     gsl_matrix_free( Maux );
     gsl_matrix_free( Minvaux );
     
     return cond;   
  }
  else
  {
    Maux = gsl_matrix_alloc( NL, NC);    
    gsl_matrix_memcpy( Maux, M);
  }
  
  gsl_matrix* S = gsl_matrix_calloc( NC, NC);    
  gsl_vector* Sv = gsl_vector_alloc( NC );
  gsl_matrix* V = gsl_matrix_calloc( NC, NC);     

  gsl_vector* work = gsl_vector_alloc( NC );  
  gsl_matrix* X = gsl_matrix_calloc( NC, NC);
  
  gsl_linalg_SV_decomp_mod( Maux, X, V, Sv, work);

  if(tol<0)
    tol = -tol * gsl_vector_get( Sv, 0);

  cond = (float) (gsl_vector_get( Sv, 0) / gsl_vector_get( Sv, NC-1));
  //printf("cond> %e \n", cond);

  gsl_vector_inv_elements( Sv );
  
  //delete small principal values
  for(cnt=0;cnt<NC;cnt++) 
  {
     if( gsl_vector_get( Sv, cnt) > (1/tol) )
       gsl_vector_set( Sv, cnt, 0);
  } 
  
  gsl_matrix_set_diagonal( S, Sv);

  gsl_matrix* AUX = gsl_matrix_calloc( NC, NL);
  gsl_blas_dgemm(CblasNoTrans, CblasTrans,
                    1.0, S, Maux,
                    0.0, AUX);
  
  gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                    1.0, V, AUX,
                    0.0, Minv);

  gsl_matrix_free( Maux );
  gsl_matrix_free( S );
  gsl_matrix_free( V );
  gsl_matrix_free( X );
  gsl_vector_free( Sv );
  gsl_vector_free( work );
  gsl_matrix_free( AUX );

  return cond;  
 }


