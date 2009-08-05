
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "MMXKernel.h"
#include "MMXConvolve.h"

MMXMatrix *gaussMatrix(mmtype type, int w, int h, float sigmaX, float sigmaY) 
{
  MMXMatrix *m=NULL;
  int i,j;
  float x,y,sum=0;
  
  if (type!=MMT_F_32 && type!=MMT_S_16) {
    fprintf(stderr,"gaussMatrix(): type %d not supported yet\n",m->type);
    return NULL;
  }

  m = MMXMatrixAlloc(type, w, h);

  for (j=0;j<h;j++)
    for (i=0;i<w;i++) {
      x = (w-1)/2.0-i;
      y = (h-1)/2.0-j;
      switch(type) {
      case MMT_F_32:
	((MM_F_32 *)m->p[j].s)[i] = 
	  exp(-(x*x)/(2*sigmaX*sigmaX)) * exp(-(y*y)/(2*sigmaY*sigmaY));
	
	if (((MM_F_32 *)m->p[j].s)[i]<1e-40) ((MM_F_32 *)m->p[j].s)[i]=0;

	sum += ((MM_F_32 *)m->p[j].s)[i];
	break;
      case MMT_S_16:
	((MM_S_16 *)m->p[j].m)[i] = 
	  (short)(128*exp(-(x*x)/(2*sigmaX*sigmaX))*
		  exp(-(y*y)/(2*sigmaY*sigmaY)));
	sum += ((MM_S_16 *)m->p[j].m)[i];
	break;
      default:
	break;
      }
    }

  return m;
}

MMXMatrix *gaussKernel(mmtype type, int w, int h, float sigmaX, float sigmaY)
{
  MMXMatrix *m=NULL,*k=NULL;
  m = gaussMatrix(type,w,h,sigmaX,sigmaY);
  //MMXMatrixPrint("gauss kernel:",m);
  k = convKernel(m);
  free(m);
  return(k);
}
  
MMXMatrix *sobelKernel(mmtype type, int w, int h, 
		       float theta, float sigmaX, float sigmaY) 
{
  MMXMatrix *m=NULL,*k=NULL;
  int i,j;
  float x,y,X,Y,sum=0;
  float ctheta=cos(theta);
  float stheta=sin(theta);

  if (type!=MMT_F_32 && type!=MMT_S_16) {
    fprintf(stderr,"gaussMatrix(): type %d not supported yet\n",m->type);
    return NULL;
  }

  m = MMXMatrixAlloc(type, w, h);

  for (j=0;j<h;j++)
    for (i=0;i<w;i++) {
      x = (w-1)/2.0-i;
      y = (h-1)/2.0-j;
      X = ctheta*x+stheta*y;
      Y = -stheta*x+ctheta*y;

      switch(type) {
      case MMT_F_32:
	((MM_F_32 *)m->p[j].s)[i] = 
	  2*X*exp(-(X*X)/(2*sigmaX*sigmaX)) * exp(-(Y*Y)/(2*sigmaY*sigmaY))/
	  (2*sigmaX*sigmaX);
	
	sum += ((MM_F_32 *)m->p[j].s)[i];
	break;
      case MMT_S_16:
	((MM_S_16 *)m->p[j].m)[i] = 
	  (MM_S_16)(128*2*X*exp(-(X*X)/(2*sigmaX*sigmaX)) * 
		    exp(-(Y*Y)/(2*sigmaY*sigmaY))/(2*sigmaX*sigmaX));
	sum += ((MM_S_16 *)m->p[j].m)[i];
	break;
      default:
	break;
      }
    }
  
  //MMXMatrixPrint("sobel kernel:",m);

  k=convKernel(m);
  free(m);
  return k;
}




