#include "cqpSolve.h"

void display_gsl_matrix(gsl_matrix *C)
{
	for(int i = 0; i < C->size1; i++)
	  {
	  for(int j = 0; j < C->size2; j++)
	    fprintf(stderr, "%.2f ", gsl_matrix_get(C, i, j));
	  fprintf(stderr, ";\n");
	  }
}

void display_gsl_vector(gsl_vector *v)
{
  for(int i = 0; i < v->size; i++)
	fprintf(stderr, "%.2f ", gsl_vector_get(v, i));
      fprintf(stderr, ";\n");

}

int solveProblem(cqpProblem *tp, gsl_cqpminimizer *s)
{
	const size_t max_iter = 1000;
	
	size_t iter=1;
	
	int status;

	
	//fprintf(stderr, "Q is: \n");
	//display_gsl_matrix(tp->cqp->Q);
	//fprintf(stderr, "q is: \n");
	//display_gsl_vector(tp->cqp->q);
	//fprintf(stderr, "A is: \n");
	//display_gsl_matrix(tp->cqp->A);
	//fprintf(stderr, "b is: \n");
	//display_gsl_vector(tp->cqp->b);
	//fprintf(stderr, "C is: \n");
	//display_gsl_matrix(tp->cqp->C);
	//fprintf(stderr, "d is: \n");
	//display_gsl_vector(tp->cqp->d);
	
	
	//printf("********************  %s  ********************\n\n",tp->name);
		
	//printf("== Itn ======= f ======== ||gap|| ==== ||residual||\n\n");

	
	do
	{
		
		status = gsl_cqpminimizer_iterate(s);
		status = gsl_cqpminimizer_test_convergence(s, 1e-10, 1e-10);
		  
		//printf("%4d   %14.8f  %13.6e  %13.6e\n", iter, gsl_cqpminimizer_f(s), gsl_cqpminimizer_gap(s), gsl_cqpminimizer_residuals_norm(s));

		if(status == GSL_SUCCESS)
		{
			size_t j;
			//printf("\nMinimum is found at\n");
			//for(j=0; j<gsl_cqpminimizer_x(s)->size; j++)
			//	printf("%9.6f ",gsl_vector_get(gsl_cqpminimizer_x(s), j));
			//printf("\n\n");
			
			//printf("\nLagrange-multipliers for Ax=b\n");
			//for(j=0; j<gsl_cqpminimizer_lm_eq(s)->size; j++)
			//	printf("%9.6f ",gsl_vector_get(gsl_cqpminimizer_lm_eq(s), j));
			//printf("\n\n");
			
			//printf("\nLagrange-multipliers for Cx>=d\n");
			//for(j=0; j<gsl_cqpminimizer_lm_ineq(s)->size; j++)
			//	printf("%9.6f ",gsl_vector_get(gsl_cqpminimizer_lm_ineq(s), j));
			//printf("\n\n");
		}  
		else
		{
			iter++;
		}  
	
	}
	while(status == GSL_CONTINUE && iter<=max_iter);  
	
	return GSL_SUCCESS;
}

  int defineProblemData(gsl_cqp_data * problem_cqp, Matrix *C, Vector *d)
{	
	problem_cqp->Q = gsl_matrix_alloc(4, 4);
	problem_cqp->q = gsl_vector_alloc(4);
	
			
	problem_cqp->A = gsl_matrix_alloc(1,4);
	problem_cqp->b = gsl_vector_alloc(1);
		
	//problem_cqp->C = gsl_matrix_alloc(11,3);
	//problem_cqp->d = gsl_vector_alloc(11);

	/* Q */
	gsl_matrix_set_identity(problem_cqp->Q);
	//gsl_matrix_set(problem_cqp->Q, 3, 3, 0.0);
	/* q = 0*/
	gsl_vector_set_zero(problem_cqp->q);
	/* A = 0*/
	gsl_matrix_set_zero(problem_cqp->A);
	gsl_matrix_set(problem_cqp->A, 0, 3, 1.0);
	/* b = 0*/
	gsl_vector_set_zero(problem_cqp->b);
	/* C */
	problem_cqp->C = (gsl_matrix*) C->getGslMatrix();
	//gsl_matrix_set_zero(problem_cqp->C);
	/* d */
	problem_cqp->d = (gsl_vector*) d->getGslVector();
	//gsl_vector_set_zero(problem_cqp->d);
	
	
	return GSL_SUCCESS;
}

int defineProblemSize(cqpProblem * lq, gsl_cqp_data* cqp_data)
{	
  //fprintf(stderr, "Dimensions of LQ problem will now be defined \n");
  lq->name 	= "Cable Length minimization";
  lq->n 	= 4;
  lq->me	= 1;
  lq->mi	=11;
  //fprintf(stderr, "Pointing to cqp_data...");
  lq->cqp	= cqp_data; 
  //fprintf(stderr, "done! \n");  
  lq->opt_value	= 100.0;
  //fprintf(stderr, "Dimensions defined \n");  
  return GSL_SUCCESS;
}

int freeProblemData(cqpProblem * lq)
{

  gsl_matrix_free(lq->cqp->Q);
  gsl_vector_free(lq->cqp->q);
		
  gsl_matrix_free(lq->cqp->A);
  gsl_vector_free(lq->cqp->b);
		
  //gsl_matrix_free(lq->cqp->C);
  //gsl_vector_free(lq->cqp->d);

  //fprintf(stderr, "cqp will be deleted \n");		
  free(lq->cqp);
  return GSL_SUCCESS;
}
