#include "mex.h"

void mexFunction( int nlhs,       mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] 
                 )
                
{   /* Sparse matrix-dense vector product 
     *       
     *  Inputs:
     *           
     *           A - m x n sparse matrix
     *           b - n x 1 dense vector
     *           
     *  Outputs:
     *   
     *           x = A*b
     *
     *  Darren Engwirda 2006.       
     */

    const double *h1, *h2;
    int *ir1, *ir2, i1, i2, nrow1, nrow2;
    /*int ncol1, ncol2;*/
    int maxc1, maxc2;
    double ret=0;
    
    /* Check I/O number */
    if (nlhs!=1) {
        mexErrMsgTxt("Incorrect number of outputs");
    }
    if (nrhs!=2) {
        mexErrMsgTxt("Incorrect number of inputs");
    }
    
    /* Brief error checking */
    
    /*ncol1 = mxGetN(prhs[0]);
    ncol2 = mxGetN(prhs[1]);*/
    nrow1 = mxGetM(prhs[0]);
    nrow2 = mxGetM(prhs[1]);
    
    /*if ((ncol!=mxGetM(prhs[1])) || (mxGetN(prhs[1])!=1)) {
        mexErrMsgTxt("Wrong input dimensions");
    }
    if (!mxIsSparse(prhs[0])) {
        mexErrMsgTxt("Matrix must be sparse");
    }*/
    
    
    /* Allocate output */
    /*plhs[0] = mxCreateDoubleMatrix(mxGetM(prhs[0]), 1, mxREAL);*/

    
    /* I/O pointers */
    ir1 = mxGetIr(prhs[0]);      /* Row indexing      */
    ir2 = mxGetIr(prhs[1]);      /* Row indexing      */    
    h1  = mxGetPr(prhs[0]);      /* Non-zero elements */
    h2  = mxGetPr(prhs[1]);      /* Rhs vector        */
    maxc1 = mxGetJc(prhs[0])[1];
    maxc2 = mxGetJc(prhs[1])[1];

    
    /* Multiplication */
    
    i1=0;
    i2=0;
    /*printf("%d %d\n",ncol1,ncol2);*/

    while(i1<maxc1 && i2<maxc2) {            /* Loop through columns */
        if (ir1[i1]==ir2[i2]) {
            ret=ret+h1[i1]*h2[i2]/(h1[i1]+h2[i2]);
            i1++;
            i2++;
        } else {
            if (ir1[i1]>ir2[i2])
                i2++;
            else
                i1++;
        }
    }

    plhs[0] = mxCreateDoubleScalar(2-4*ret);
    
    /* End */
}
