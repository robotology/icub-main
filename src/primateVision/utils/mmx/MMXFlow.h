
#ifndef _MMX_FLOW_H
#define _MMX_FLOW_H

#include "MMXMatrix.h"

int MMXIIR(float tau, MMXMatrix *m, MMXMatrix *w[3], MMXMatrix *I, 
	   MMXMatrix *dt);

//int MMXLucas(MMXMatrix *I, float tau, 
//	     float sigma1, float sigma2, float sigmap,
//	     MMXMatrix *W[3], MMXMatrix *V[2], MMXMatrix *E, MMXMatrix *D);

int MMXLucas(MMXMatrix *I, float tau,
             float sigma1, float sigma2, float sigmap,
             MMXMatrix *W[3], MMXMatrix *V[2], MMXMatrix *E, MMXMatrix *D,
             MMXMatrix *dt2,MMXMatrix *I2,MMXMatrix *dx,MMXMatrix *dy,MMXMatrix *dxdx,
             MMXMatrix *dydy,MMXMatrix *dxdy,MMXMatrix *dxdt,MMXMatrix *dydt,
             MMXMatrix *dt,MMXMatrix *dxdx2,MMXMatrix *dydy2,MMXMatrix *dxdy2,
             MMXMatrix *dxdt2,MMXMatrix *dydt2,
             MMXMatrix *gKernel,MMXMatrix *dxKernel,MMXMatrix *dyKernel);


int MMXVirFlow(MMXMatrix *M[6], float omega[3], MMXMatrix *V[2]);

int MMXFlowSegmentation(float r, float c[2], float e1[2], float e2[2], 
			MMXMatrix *V[2], MMXMatrix *S);

unsigned int MMXIIR_S16(short *src, MMXMatrix *w[3], int width, int height, 
			MMXMatrix *I, MMXMatrix *dt);

#endif
