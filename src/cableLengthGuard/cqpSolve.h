#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_matrix_double.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>


#include <yarp/sig/Matrix.h> 
#include <yarp/sig/Vector.h> 
#include <yarp/math/Math.h>

extern "C" {
#include <gsl_cqp.h>
}

using namespace yarp::sig;
using namespace yarp::math; 

typedef struct
{
	const char *name;
	size_t n;
	size_t me;
	size_t mi;
	gsl_cqp_data *cqp;
	double opt_value;
}
cqpProblem;

int solveProblem(cqpProblem *tp, gsl_cqpminimizer *s);
int freeProblemData(cqpProblem *tp);
int defineProblemSize(cqpProblem *tp, gsl_cqp_data* cqp_data);
int defineProblemData(gsl_cqp_data *cqp_data, Matrix *C, Vector *d);

