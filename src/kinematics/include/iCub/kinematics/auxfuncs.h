
#ifndef _ICUB_AUXFUNCS_INC_
#define _ICUB_AUXFUNCS_INC_

#include <stdio.h>
#include <math.h>

#include <iCub/kinematics/robmatrix.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

// Guards added to avoid multiple redefinitions of macros.
// whether M_PI is available from math.h seems a bit platform dependent.

#ifndef M_PI_2
#define M_PI_2	((float)(asin(1.0)))
#endif

#ifndef M_PI
#define M_PI	((float)(2*M_PI_2))
#endif

int limitjoint(double *s, double *lim, double *th);
int myasbccsol(double aux1, double aux2, double aux3, double *s);
gsl_matrix *RobMatrix2Gsl(RobMatrix *M);
//double min( double a, double b);

#endif
