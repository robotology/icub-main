
#include <stdio.h>
#include <stdlib.h>
#include "MMXFlow.h"
#include "MMXConvolve.h"
#include "MMXKernel.h"
#include "MMXUtils.h"

int MMXIIR(float tau, MMXMatrix *m, MMXMatrix *w[3], MMXMatrix *I, 
	   MMXMatrix *dt)
{
	MMXMatrix *pm;
	sse_t *psrc,*pdt,*pw1,*pw2,*py,*pI;
	float q = tau/(tau+2);
	float r = (tau-2)/(tau+2);
	int i,j;
	float tmp[2];

	tmp[0] = tmp[1] = -r;
	movlps_m2r(*tmp,xmm0);
	movlhps_r2r(xmm0,xmm0);

	tmp[0] = tmp[1] = q;
	movlps_m2r(*tmp,xmm1);
	movlhps_r2r(xmm1,xmm1);

	tmp[0] = tmp[1] = 2;
	movlps_m2r(*tmp,xmm2);
	movlhps_r2r(xmm2,xmm2);

	tmp[0] = tmp[1] = tau;
	movlps_m2r(*tmp,xmm3);
	movlhps_r2r(xmm3,xmm3);

	for (j=0;j<m->rows;j++) {
		psrc = m->p[j].s;
		pw1 = w[0]->p[j].s;
		pw2 = w[1]->p[j].s;
		py = w[2]->p[j].s;
		pdt = dt->p[j].s;
		pI = I->p[j].s;

		for (i=0;i<(m->cols+3)/4;i++) {
			movaps_m2r(*(pw1++),xmm4); /* w1 in xmm4 */
			mulps_r2r(xmm2,xmm4); /* 2*w1 in xmm4 */
			movaps_m2r(*pw2,xmm5); /* w2 in xmm5 */
			movaps_r2r(xmm5,xmm6); 
			mulps_r2r(xmm0,xmm6);
			movaps_r2r(xmm4, xmm7);
			subps_r2r(xmm6,xmm7);
			mulps_r2r(xmm0,xmm7);
			addps_m2r(*(psrc++),xmm7); /* w0 in xmm7 */
			movaps_r2m(xmm7,*(pw2++));

			addps_r2r(xmm4,xmm7);
			addps_r2r(xmm5,xmm7);
			mulps_r2r(xmm1,xmm7); 
			mulps_r2r(xmm1,xmm7); /* R2 in xmm7 */

			movaps_m2r(*py,xmm4); /* y(t-1) in xmm4 */
			movaps_r2r(xmm4,xmm5);
			mulps_r2r(xmm0,xmm5);
			addps_r2r(xmm7,xmm5); /* y(t) in xmm5 */
			movaps_r2m(xmm5,*(py++));
      
			addps_r2r(xmm4,xmm5);
			mulps_r2r(xmm1,xmm5); /* R3 in xmm5 */
			movaps_r2m(xmm5,*(pI++));
      
			subps_r2r(xmm5,xmm7);
			mulps_r2r(xmm3,xmm7); /* dt in xmm7 */
			movaps_r2m(xmm7,*(pdt++));
		}
	}

	/* swap the w matrices */
	pm = w[0];
	w[0] = w[1];
	w[1] = pm;

	return 0;
}

int MMXDxDyDt(float sigma1, float sigma2,
	      MMXMatrix *dx, MMXMatrix *dy, MMXMatrix *dt, 
	      MMXMatrix *dxdx, MMXMatrix *dydy, MMXMatrix *dxdy, 
	      MMXMatrix *dxdt, MMXMatrix *dydt)
{
	int i,j;
	sse_t *pdx=NULL,*pdy=NULL,*pdt=NULL;
	sse_t *pdxdx=NULL,*pdydy=NULL,*pdxdy=NULL;
	sse_t *pdxdt=NULL,*pdydt=NULL;
	float tmp[2];
  
	tmp[0]=sigma1;
	tmp[1]=tmp[0];
	movlps_m2r(*tmp,xmm6);
	movlhps_r2r(xmm6,xmm6);

	tmp[0]=sigma2;
	tmp[1]=tmp[0];
	movlps_m2r(*tmp,xmm7);
	movlhps_r2r(xmm7,xmm7);
  
	for (j=0;j<dx->rows;j++) {
		pdx = dx->p[j].s;
		pdy = dy->p[j].s;
		pdt = dt->p[j].s;
		pdxdx = dxdx->p[j].s;
		pdydy = dydy->p[j].s;
		pdxdy = dxdy->p[j].s;
		pdxdt = dxdt->p[j].s;
		pdydt = dydt->p[j].s;
		for (i=0;i<(dx->cols+3)/4;i++) {
			movaps_m2r(*(pdx++),xmm0);
			movaps_m2r(*(pdy++),xmm1);
			movaps_m2r(*(pdt++),xmm2);

			movaps_r2r(xmm0,xmm3);
			mulps_r2r(xmm0,xmm3);

			movaps_r2r(xmm1,xmm4);
			mulps_r2r(xmm1,xmm4);
#if 1
			movaps_r2r(xmm4,xmm5);
			addps_r2r(xmm3,xmm5);
			mulps_r2r(xmm6,xmm5);
			addps_r2r(xmm7,xmm5);
			rcpps_r2r(xmm5,xmm5);
			mulps_r2r(xmm5,xmm3);
			mulps_r2r(xmm5,xmm4);
#endif
			movaps_r2m(xmm3,*(pdxdx++));
			movaps_r2m(xmm4,*(pdydy++));

			movaps_r2r(xmm0,xmm3);
			mulps_r2r(xmm1,xmm3);
			mulps_r2r(xmm5,xmm3);
			movaps_r2m(xmm3,*(pdxdy++));

			mulps_r2r(xmm2,xmm0);
			mulps_r2r(xmm5,xmm0);
			movaps_r2m(xmm0,*(pdxdt++));

			mulps_r2r(xmm2,xmm1);
			mulps_r2r(xmm5,xmm1);
			movaps_r2m(xmm1,*(pdydt++));
		}
	}

	return 0;
}

#if 0
int MMXDet(MMXMatrix *dxdx, MMXMatrix *dydy, MMXMatrix *dxdy,
	   MMXMatrix *dxdt, MMXMatrix *dydt,
	   MMXMatrix *V[2], MMXMatrix *L[2]) 
{
	int i,j;
	sse_t *pdxdx=NULL,*pdydy=NULL,*pdxdy=NULL;
	sse_t *pdxdt=NULL,*pdydt=NULL;
	sse_t *pvx=NULL,*pvy=NULL;
	sse_t *pl1=NULL,*pl2=NULL;

	for (j=0;j<dxdx->rows;j++) {
		pdxdx = dxdx->p[j].s;
		pdydy = dydy->p[j].s;
		pdxdy = dxdy->p[j].s;
		pdxdt = dxdt->p[j].s;
		pdydt = dydt->p[j].s;
		pvx = V[0]->p[j].s;
		pvy = V[1]->p[j].s;
		pl1 = L[0]->p[j].s;
		pl2 = L[1]->p[j].s;

		for (i=0;i<(dxdx->cols+3)/4;i++) {
			movaps_m2r(*(pdxdx++),xmm1);
			movaps_m2r(*(pdydy++),xmm2);
			movaps_m2r(*(pdxdy++),xmm3);
			movaps_m2r(*(pdxdt++),xmm4);
			movaps_m2r(*(pdydt++),xmm5);

			movaps_r2r(xmm1,xmm0);
			mulps_r2r(xmm2,xmm0);
    
			movaps_r2r(xmm3,xmm6);
			mulps_r2r(xmm6,xmm6);
			subps_r2r(xmm6,xmm0); /* determinant a*b-c*c in xmm0 */

			movaps_r2r(xmm1,xmm6);
			addps_r2r(xmm2,xmm6); /* trace in xmm6 */

			mulps_r2r(xmm4,xmm2);
			movaps_r2r(xmm3,xmm7);
			mulps_r2r(xmm5,xmm7);
			subps_r2r(xmm7,xmm2); /* vx in xmm2 */
			divps_r2r(xmm0,xmm2);
			movaps_r2m(xmm2,*(pvx++));
    
			mulps_r2r(xmm5,xmm1);
			mulps_r2r(xmm4,xmm3);
			subps_r2r(xmm3,xmm1); /* vy in xmm1 */
			divps_r2r(xmm0,xmm1);
			movaps_r2m(xmm1,*(pvy++));

			movaps_r2r(xmm6,xmm7);
			mulps_r2r(xmm6,xmm7);
			subps_r2r(xmm0,xmm7);
			sqrtps_r2r(xmm7,xmm7);
    
			movaps_r2r(xmm6,xmm1);
			addps_r2r(xmm7,xmm1); /* l1 in xmm1 */
			movups_r2m(xmm1,*(pl1++));
    
			subps_r2r(xmm7,xmm6); /* l2 in xmm6 */
			movups_r2m(xmm6,*(pl2++));
		}
	}
	return 0;
}
#else
/* -----------------
   |a0 a2| |u|= |b0|
   |a2 a1| |v|= |b1|
   ----------------- */
int MMXDet(MMXMatrix *dxdx, MMXMatrix *dydy, MMXMatrix *dxdy,
	   MMXMatrix *dxdt, MMXMatrix *dydt, float sigmap,
	   MMXMatrix *V[2], MMXMatrix *E) 
{
	int i,j;
	sse_t *pdxdx=NULL,*pdydy=NULL,*pdxdy=NULL;
	sse_t *pdxdt=NULL,*pdydt=NULL;
	sse_t *pvx=NULL,*pvy=NULL;
	sse_t *pe=NULL;
	float tmp[2];
  
	tmp[0]=1.0/sigmap;
	tmp[1]=tmp[0];
	movlps_m2r(*tmp,xmm7);
	movlhps_r2r(xmm7,xmm7);

	tmp[0]=4;
	tmp[1]=tmp[0];
	movlps_m2r(*tmp,xmm6);
	movlhps_r2r(xmm6,xmm6);

	for (j=0;j<dxdx->rows;j++) {
		pdxdx = dxdx->p[j].s;
		pdydy = dydy->p[j].s;
		pdxdy = dxdy->p[j].s;
		pdxdt = dxdt->p[j].s;
		pdydt = dydt->p[j].s;
		pvx = V[0]->p[j].s;
		pvy = V[1]->p[j].s;
		pe = E->p[j].s;

		for (i=0;i<(dxdx->cols+3)/4;i++) {
			movaps_m2r(*(pdxdx++),xmm1); /* a0 */
			movaps_m2r(*(pdydy++),xmm2); /* a1 */
			addps_r2r(xmm7,xmm1);        /* a0-sigmap */
			movaps_m2r(*(pdxdy++),xmm3); /* a2 */
			addps_r2r(xmm7,xmm2);        /* a1-sigmap */

			/* Compute determinant */
			movaps_r2r(xmm1,xmm0); /* a0 */
			mulps_r2r(xmm2,xmm0);  /* a0*a1 */
			movaps_r2r(xmm3,xmm4); /* a2 */
			mulps_r2r(xmm4,xmm4);  /* a2*a2 */
			subps_r2r(xmm4,xmm0);  /* det = a0*a1-a2*a2 */
			rcpps_r2r(xmm0,xmm0);  /* 1/det */

			/* Compute eigen value */
			movaps_r2r(xmm1,xmm4); /* a0 */
			subps_r2r(xmm2,xmm4);  /* a1-a0 */
			mulps_r2r(xmm4,xmm4);  /* (a1-a0)*(a1-a0) */
			movaps_r2r(xmm3,xmm5); /* a2 */
			mulps_r2r(xmm5,xmm5);  /* a2*a2 */
			mulps_r2r(xmm6,xmm5);  /* 4*a2*a2 */
			addps_r2r(xmm5,xmm4);  /* (a1-a0)*(a1-a0) - 4*a2*a2 */
			sqrtps_r2r(xmm4,xmm4); /* sqrt((a1-a0)*(a1-a0) - 4*a2*a2) */
			movaps_r2r(xmm1,xmm5); /* a0 */
			addps_r2r(xmm2,xmm5);  /* a0+a1 */
			subps_r2r(xmm4,xmm5);  /* a0+a1 - sqrt((a1-a0)*(a1-a0) - 4*a2*a2) */
			
			// test 
			sqrtps_r2r(xmm5,xmm5);
			//mulps_r2r(xmm0,xmm5);  /* min eigen value */
			//mulps_r2r(xmm5,xmm5); /* square */
			movaps_r2m(xmm5,*(pe++));

			movaps_m2r(*(pdxdt++),xmm4); /* b0 */
			//movaps_m2r(*(pdydt++),xmm5); /* b1 */

			mulps_r2r(xmm4,xmm2);   /* a1*b0 */
			movaps_r2r(xmm3,xmm5);  /* a2 */
			mulps_m2r(*pdydt,xmm5); /* a2*b1 */
			subps_r2r(xmm5,xmm2);   /* a1*b0-a2*b1 */
			mulps_r2r(xmm0,xmm2);   /* (a1*b0-a2*b1)/det */
			movaps_r2m(xmm2,*(pvx++)); /* u */
    
			mulps_m2r(*(pdydt++),xmm1); /* a0*b1 */
			mulps_r2r(xmm4,xmm3);       /* a2*b0 */
			subps_r2r(xmm3,xmm1);       /* a0*b1-a2*b0 */
			mulps_r2r(xmm0,xmm1);       /* (a0*b1-a2*b0)/det */ 
			movaps_r2m(xmm1,*(pvy++));  /* v */
		}
	}
	return 0;
}
#endif
      
int MMXLucas(MMXMatrix *I, float tau, 
	     float sigma1, float sigma2, float sigmap,
	     MMXMatrix *W[3], MMXMatrix *V[2], MMXMatrix *E, MMXMatrix *D,
	     MMXMatrix *dt2,MMXMatrix *I2,MMXMatrix *dx,MMXMatrix *dy,MMXMatrix *dxdx,
	     MMXMatrix *dydy,MMXMatrix *dxdy,MMXMatrix *dxdt,MMXMatrix *dydt,
	     MMXMatrix *dt,MMXMatrix *dxdx2,MMXMatrix *dydy2,MMXMatrix *dxdy2,
	     MMXMatrix *dxdt2,MMXMatrix *dydt2,
	     MMXMatrix *gKernel,MMXMatrix *dxKernel,MMXMatrix *dyKernel)
{
#if 0
	dx = D;
#endif

	MMXIIR(tau,I,W,I2,dt2);
	MMXStrip(dt2,2,2,2,2,dt);
	//printf("I2: %dx%d dx: %dx%d\n",I2->cols,I2->rows,dx->cols,dx->rows);
	MMXConvolve(I2,dxKernel,dx);
	MMXConvolve(I2,dyKernel,dy);
	MMXDxDyDt(sigma1,sigma2,dx,dy,dt,dxdx,dydy,dxdy,dxdt,dydt);
	MMXConvolve(dxdx,gKernel,dxdx2);
	MMXConvolve(dydy,gKernel,dydy2);
	MMXConvolve(dxdy,gKernel,dxdy2);
	MMXConvolve(dxdt,gKernel,dxdt2);
	MMXConvolve(dydt,gKernel,dydt2);
	MMXDet(dxdx2,dydy2,dxdy2,dxdt2,dydt2,sigmap,V,E);
	return 0;
}

int MMXVirFlow(MMXMatrix *M[6], float omega[3], MMXMatrix *V[2]) 
{
	int i,j;
	sse_t *ptr0,*ptr1,*ptr2,*ptr3,*ptr4,*ptr5,*pv1,*pv2;
	float tmp[2];
  
	tmp[0]=omega[0];
	tmp[1]=tmp[0];
	movlps_m2r(*tmp,xmm0);
	movlhps_r2r(xmm0,xmm0);

	tmp[0]=omega[1];
	tmp[1]=tmp[0];
	movlps_m2r(*tmp,xmm1);
	movlhps_r2r(xmm1,xmm1);

	tmp[0]=omega[2];
	tmp[1]=tmp[0];
	movlps_m2r(*tmp,xmm2);
	movlhps_r2r(xmm2,xmm2);
  
	for (j=0;j<V[0]->rows;j++) {
		ptr0 = M[0]->p[j].s;
		ptr1 = M[1]->p[j].s;
		ptr2 = M[2]->p[j].s;
		ptr3 = M[3]->p[j].s;
		ptr4 = M[4]->p[j].s;
		ptr5 = M[5]->p[j].s;
		pv1 = V[0]->p[j].s;
		pv2 = V[1]->p[j].s;

		for (i=0;i<(V[0]->cols+3)/4;i++) {
			movaps_m2r(*(ptr0++),xmm3);
			mulps_r2r(xmm0,xmm3);

			movaps_m2r(*(ptr1++),xmm4);
			mulps_r2r(xmm1,xmm4);
			addps_r2r(xmm4,xmm3);

			movaps_m2r(*(ptr2++),xmm4);
			mulps_r2r(xmm2,xmm4);
			addps_r2r(xmm4,xmm3);

			movaps_r2m(xmm3,*(pv1)++);

			movaps_m2r(*(ptr3++),xmm3);
			mulps_r2r(xmm0,xmm3);

			movaps_m2r(*(ptr4++),xmm4);
			mulps_r2r(xmm1,xmm4);
			addps_r2r(xmm4,xmm3);

			movaps_m2r(*(ptr5++),xmm4);
			mulps_r2r(xmm2,xmm4);
			addps_r2r(xmm4,xmm3);

			movaps_r2m(xmm3,*(pv2)++);
		}
	}
	return 0;
}

int MMXFlowSegmentation(float r, float c[2], float e1[2], float e2[2], 
			MMXMatrix *V[2], MMXMatrix *S) 
{
	int i,j;
	sse_t *pv1,*pv2,*ps,*p0,*p1,*p2,*p3,*p4,*p5;
	MMXMatrix *m = MMXMatrixAlloc(MMT_F_32,4,9);
	float *pf;

	for (i=0;i<4;i++) {
		((float *)m->p[0].s)[i] = c[0];
		((float *)m->p[1].s)[i] = c[1];
		((float *)m->p[2].s)[i] = e1[0];
		((float *)m->p[3].s)[i] = e1[1];
		((float *)m->p[4].s)[i] = e2[0];
		((float *)m->p[5].s)[i] = e2[1];
		((float *)m->p[6].s)[i] = 1.0/(e1[0]*e1[0]+e1[1]*e1[1]);
		((float *)m->p[7].s)[i] = 1.0/(e2[0]*e2[0]+e2[1]*e2[1]);
		((float *)m->p[8].s)[i] = -1.0/(2.0*r*r);
	}
  
	p0 = m->p[0].s;
	p1 = m->p[1].s;
	p2 = m->p[2].s;
	p3 = m->p[3].s;
	p4 = m->p[4].s;
	p5 = m->p[5].s;

	movaps_m2r(*(m->p[6].s),xmm5);
	movaps_m2r(*(m->p[7].s),xmm6);
	movaps_m2r(*(m->p[8].s),xmm7);

	for (j=0;j<V[0]->rows;j++) {
		pv1 = V[0]->p[j].s;
		pv2 = V[1]->p[j].s;
		ps = S->p[j].s;

		for (i=0;i<(V[0]->cols+3)/4;i++) {
			movaps_m2r(*(pv1++),xmm0); /* vx */
			subps_m2r(*p0,xmm0);       /* vx - mx */

			movaps_m2r(*(pv2++),xmm1); /* vy */
			subps_m2r(*p1,xmm1);       /* vy - my */

			movaps_r2r(xmm0,xmm2);     /* vx - mx */
			movaps_r2r(xmm1,xmm3);     /* vy - my */

			mulps_m2r(*p2,xmm0);       /* (vx-mx)*e0x */
			mulps_m2r(*p3,xmm1);       /* (vy-my)*e0y */

			mulps_m2r(*p4,xmm2);       /* (vx-mx)*e1x */
			mulps_m2r(*p5,xmm3);       /* (vy-my)*e1y */

			addps_r2r(xmm1,xmm0);      /* (v-m).e0 */
			addps_r2r(xmm3,xmm2);      /* (v-m).e1 */

			mulps_r2r(xmm5,xmm0);      /* (v-m).e0 / (e0.e0) */
			mulps_r2r(xmm6,xmm2);      /* (v-m).e1 / (e1.e1) */

			mulps_r2r(xmm0,xmm0);
			mulps_r2r(xmm2,xmm2);

			addps_r2r(xmm2,xmm0);
			mulps_r2r(xmm7,xmm0);

			movaps_r2m(xmm0,*(ps++));
		}
	}
	free(m);

	for (j=0;j<S->rows;j++) {
		pf = (float *)S->p[j].s;
		for (i=0;i<S->cols;i++) {
			*pf = 1-exp(*pf);
			pf++;
		}
	}
	return 0;
}
      
      
