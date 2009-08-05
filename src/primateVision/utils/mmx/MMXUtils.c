
#include <stdlib.h>
#include "MMXMatrix.h"

int MMXStrip(MMXMatrix *m, int argx0, int argx1, int argy0, int argy1, MMXMatrix *d) 
{
  int i,j;
  sse_t *psrc,*pdst;
  
  for (j=argy0;j<m->rows-argy1;j++) {
    psrc = (sse_t *)((float *)m->p[j].s + argx0);
    pdst = d->p[j-argy0].s;
    for (i=argx0;i<m->cols-argx1;i+=4) {
      movups_m2r(*(psrc++),xmm0);
      movaps_r2m(xmm0,*(pdst++));
    }
  }
  return 0;
}


int MMXGrav(MMXMatrix *src, int thresh, float *x, float *y)
{
  int i,j;
  float sumw=0,sumx=0,sumy=0;
  mmx_t m[2],*psrc;
  
  m[0].w[0]=1; m[0].w[1]=1; m[0].w[2]=1; m[0].w[3]=1;
  movq_m2r(*m,mm1);   
  
  pxor_r2r(mm7,mm7); /* sum of weight in mm7 */
  pxor_r2r(mm6,mm6); /* sum of w*y in mm6 */
  pxor_r2r(mm5,mm5); /* sum of w*x in mm5 */
  pxor_r2r(mm4,mm4); /* mm4 contains row index */
  m[0].w[0]=0; m[0].w[1]=1; m[0].w[2]=2; m[0].w[3]=3;
  movq_m2r(*m,mm3);   /* mm3 contains col index */

  for (j=0;j<src->rows;j++) {
    psrc = src->p[j].m;
    movq_m2r(*m,mm3);
    for (i=0;i<(src->cols+3)/4;i++) {
      movq_m2r(*(psrc++),mm0);
      
      /* compute sum of weights */
      movq_r2r(mm0,mm2);
      pmaddwd_r2r(mm1,mm2);
      paddd_r2r(mm2,mm7);
      
      /* compute sum of w*y */
      movq_r2r(mm0,mm2);
      pmaddwd_r2r(mm4,mm2);
      paddd_r2r(mm2,mm6);
      
      /* compute sum of w*x */
      pmaddwd_r2r(mm3,mm2);
      paddd_r2r(mm2,mm5);

      /* increment the col indexes */
      paddw_r2r(mm1,mm3);
    }
    /* increment the row indexes */
    paddw_r2r(mm1,mm4);
  }
  emms();

  movq_r2m(mm7,*m);
  sumw = m[0].d[0]+m[0].d[1];
  movq_r2m(mm6,*m);
  sumy = m[0].d[0]+m[0].d[1];
  movq_r2m(mm5,*m);
  sumx = m[0].d[0]+m[0].d[1];
  
  if (sumw>0) {
    *x = sumx/sumw;
    *y = sumy/sumw;
  }
  
  return (int)sumw;
}


int MMXThreshGrav(unsigned char *src, int width, int height,
		  unsigned char *dst, unsigned char thresh, float x[2])
{
  unsigned long sumw=0,sumx=0,sumy=0;
  mmx_t m[2],mxor[2],mand[2],mthresh[2],mzero[2],mone[2],mfour[2];
  unsigned char *psrc=src, *end=src+width*height, *endRow=0;
  unsigned char *pdst=dst;
  
  if (width%8) return -1;
  
  mxor[0].q = 0x8080808080808080LL;
  mand[0].q = 0xffffffffffffffffLL;
  mthresh[0].ub[0] = thresh ^ 0x80;
  mthresh[0].ub[1] = mthresh[0].ub[0];
  mthresh[0].ub[2] = mthresh[0].ub[0];
  mthresh[0].ub[3] = mthresh[0].ub[0];
  mthresh[0].ub[4] = mthresh[0].ub[0];
  mthresh[0].ub[5] = mthresh[0].ub[0];
  mthresh[0].ub[6] = mthresh[0].ub[0];
  mthresh[0].ub[7] = mthresh[0].ub[0];
  mzero[0].q = 0x0LL;
  mone[0].w[0] = mone[0].w[1] = mone[0].w[2] = mone[0].w[3] = 1;
  mfour[0].w[0] = mfour[0].w[1] = mfour[0].w[2] = mfour[0].w[3] = 4;
  
  pxor_r2r(mm7,mm7); /* sum of weight in mm7 */
  pxor_r2r(mm6,mm6); /* sum of w*y in mm6 */
  pxor_r2r(mm5,mm5); /* sum of w*x in mm5 */
  pxor_r2r(mm4,mm4); /* mm4 contains row index */
  m[0].w[0]=0; m[0].w[1]=1; m[0].w[2]=2; m[0].w[3]=3;
  
  while(psrc<end) {
    movq_m2r(*m,mm3);   /* mm3 contains col index */
    endRow = psrc+width;
    while(psrc<endRow) {
      movq_m2r(*psrc,mm0);
      
      /* thresholding */
      movq_r2r(mm0,mm1);
      pxor_m2r(*mxor,mm1);
      pcmpgtb_m2r(*mthresh, mm1);
      pandn_m2r(*mand, mm1);
      //pand_r2r(mm1,mm0); /* mm0 contains binarized data */
      movq_r2r(mm1, mm0);
      movq_r2m(mm0, *pdst);
      
      /* compute sum of weights for 1st 4 bytes */
      movq_r2r(mm0,mm1);
      punpcklbw_m2r(*mzero,mm0);
      
      movq_r2r(mm0,mm2);
      pmaddwd_m2r(*mone,mm2);
      paddd_r2r(mm2,mm7);
      
      /* compute sum of w*y */
      movq_r2r(mm0,mm2);
      pmaddwd_r2r(mm4,mm2);
      paddd_r2r(mm2,mm6);
      
      /* compute sum of w*x */
      pmaddwd_r2r(mm3,mm0);
      paddd_r2r(mm0,mm5);
      
      /* increment the col indexes */
      paddw_m2r(*mfour,mm3);
      
      /* compute sum of weights for last 4 bytes */
      punpckhbw_m2r(*mzero,mm1);
      
      movq_r2r(mm1,mm2);
      pmaddwd_m2r(*mone,mm2);
      paddd_r2r(mm2,mm7);
      
      /* compute sum of w*y */
      movq_r2r(mm1,mm2);
      pmaddwd_r2r(mm4,mm2);
      paddd_r2r(mm2,mm6);
      
      /* compute sum of w*x */
      pmaddwd_r2r(mm3,mm1);
      paddd_r2r(mm1,mm5);
      
      /* increment the col indexes */
      paddw_m2r(*mfour,mm3);
      
      psrc += 8;
      pdst += 8;
    }
    /* increment the row indexes */
    paddw_m2r(*mone,mm4);
  }
  movq_r2m(mm7,*m);
  sumw = m[0].ud[0]+m[0].ud[1];
  movq_r2m(mm6,*m);
  sumy = m[0].d[0]+m[0].d[1];
  movq_r2m(mm5,*m);
  sumx = m[0].d[0]+m[0].d[1];
  emms();
  
  if (sumw!=0) {
    x[0] = sumx/(float)sumw;
    x[1] = sumy/(float)sumw;
  }
  
  return (int)sumw;
}



// B = A - B + 128 U8 saturated math
int MMXSubtract_U8p128(unsigned char *A, unsigned char *B, int n) {

  mmx_t *psrc = (mmx_t *)A;
  mmx_t *end = (mmx_t *)(B+n);
  mmx_t *psrcdst = (mmx_t *)B;
  mmx_t ma;
  ma.q = 0x0080008000800080ULL;
  movq_m2r(ma,mm6);
  pxor_r2r(mm7,mm7);
  while (psrcdst<end) {
    movq_m2r(*psrc,mm0);
    movq_m2r(*psrcdst,mm2);
    // make U8->U16 A
    movq_r2r(mm0,mm1);
    punpcklbw_r2r(mm7,mm0);
    punpckhbw_r2r(mm7,mm1);
    // make U8->U16 B
    movq_r2r(mm2,mm3);
    punpcklbw_r2r(mm7,mm2);
    punpckhbw_r2r(mm7,mm3);
    // add 128
    paddw_r2r(mm6,mm0);
    paddw_r2r(mm6,mm1);
    // saturate subtract B from A
    psubusw_r2r(mm2,mm0);
    psubusw_r2r(mm3,mm1);
    // repack into U8
    packuswb_r2r(mm1,mm0);
    movq_r2m(mm0,*psrcdst);
    psrc++;
    psrcdst++;
  }
  emms();
  return 0;
}



// B = A + B - 128 U8 saturated math
int MMXAdd_U8m128(unsigned char *A, unsigned char *B, int n) {

  mmx_t *psrc = (mmx_t *)A;
  mmx_t *end = (mmx_t *)(B+n);
  mmx_t *psrcdst = (mmx_t *)B;
  mmx_t ma;
  ma.q = 0x0080008000800080ULL;
  movq_m2r(ma,mm6);
  pxor_r2r(mm7,mm7);
  while (psrcdst<end) {
    movq_m2r(*psrc,mm0);
    movq_m2r(*psrcdst,mm2);
    // make U8 to U16
    movq_r2r(mm0,mm1);
    punpcklbw_r2r(mm7,mm0);
    punpckhbw_r2r(mm7,mm1);
    // make U8 to U16
    movq_r2r(mm2,mm3);
    punpcklbw_r2r(mm7,mm2);
    punpckhbw_r2r(mm7,mm3);
    // add A B
    paddw_r2r(mm2,mm0);
    paddw_r2r(mm3,mm1);
    // subtract 128
    psubusw_r2r(mm6,mm0);
    psubusw_r2r(mm6,mm1);
    // pack U8
    packuswb_r2r(mm1,mm0);
    movq_r2m(mm0,*psrcdst);
    psrc++;
    psrcdst++;
  }
  emms();
  return 0;
}

