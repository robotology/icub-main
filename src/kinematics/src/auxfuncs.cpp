
#include <math.h>
#include <stdio.h>

#include <iCub/kinematics/auxfuncs.h>

/*********************************************************************
*
*
*
*
*
***********************************************************************/
int limitjoint(double *s, double *lim, double *th) {
int v[2];
    
    v[0] = (s[0]>lim[0]) & (s[0]<lim[1]);
    v[1] = (s[1]>lim[0]) & (s[1]<lim[1]);    

    if(v[0]==1)
        *th=s[1];
    else if(v[1]==1) {
        th[0]=s[1];
    }
    else {
        printf("fora dos limites fisicos %f\n",s[0]*180.0/M_PI);
        return -1;
    }

    return 0;    
}

/*********************************************************************
*
*
*
*
*
***********************************************************************/
int myasbccsol(double aux1, double aux2, double aux3, double *s)
{

    if((aux1+aux3)==0) 
    {
        s[0]=M_PI_2;
        s[1]=-M_PI_2;
    }
    else
    {
    double aux;    
        aux=aux2*aux2+aux1*aux1-aux3*aux3;    
        
        if(!aux) {
                s[0]=2*atan( (aux2) / (aux1+aux3));
                s[1]=s[0];
        }
        else if(aux<0)
                return -1;
        else {
                s[0] = 2*atan( (aux2 + sqrt( aux ) ) / (aux1+aux3));
                s[1] = 2*atan( (aux2 - sqrt( aux ) ) / (aux1+aux3));
        }
    }

    return 0;        
}

/*********************************************************************
*
*
*
*
*
***********************************************************************/
gsl_matrix *RobMatrix2Gsl(RobMatrix *RM)
{
	gsl_matrix *gslM = gsl_matrix_alloc( 4, 4);

	for(int cntl=0;cntl<4;cntl++)
		for(int cntc=0;cntc<4;cntc++)
			gsl_matrix_set( gslM, cntl, cntc, RM->M[cntl][cntc]);

	return gslM;
}


