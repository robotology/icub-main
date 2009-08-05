
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "MMXConvert.h"

#ifdef _PIII
int MMXShrink4_4Float(unsigned char *src, int w, int h, MMXMatrix *dst) 
{
  sse_t factor;
  mmx_t one;
  int i,j;
  sse_t *pdst;
  unsigned char *psrc=src;

  one.w[0]=one.w[1]=one.w[2]=one.w[3]=1;
  movq_m2r(one,mm1);
  pxor_r2r(mm0,mm0);
  xorps_r2r(xmm0,xmm0);
  xorps_r2r(xmm1,xmm1);
  factor.sf[0]=1.0/16.0;
  factor.sf[1]=1.0/16.0;
  factor.sf[2]=1.0/16.0;
  factor.sf[3]=1.0/16.0;
  movups_m2r(factor,xmm2);

  for (j=0;j<dst->rows;j++) {
    pdst = dst->p[j].s;

    for (i=0;i<(dst->cols+3)/4;i++) {
      movq_m2r(*psrc,mm2);
      movq_r2r(mm2,mm3);
      punpcklbw_r2r(mm0,mm2);
      punpckhbw_r2r(mm0,mm3);
      paddw_r2r(mm3,mm2);

      movq_m2r(*(psrc+w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      paddw_r2r(mm4,mm3);
      paddw_r2r(mm3,mm2);

      movq_m2r(*(psrc+2*w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      paddw_r2r(mm4,mm3);
      paddw_r2r(mm3,mm2);

      movq_m2r(*(psrc+3*w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      paddw_r2r(mm4,mm3);
      paddw_r2r(mm3,mm2);

      pmaddwd_r2r(mm1,mm2);

      psrc+=8;
      
      movq_m2r(*psrc,mm5);
      movq_r2r(mm5,mm4);
      punpcklbw_r2r(mm0,mm5);
      punpckhbw_r2r(mm0,mm4);
      paddd_r2r(mm4,mm5);

      movq_m2r(*(psrc+w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      paddw_r2r(mm4,mm3);
      paddw_r2r(mm3,mm5);
      
      movq_m2r(*(psrc+2*w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      paddw_r2r(mm4,mm3);
      paddw_r2r(mm3,mm5);
      
      movq_m2r(*(psrc+3*w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      paddw_r2r(mm4,mm3);
      paddw_r2r(mm3,mm5);
      
      pmaddwd_r2r(mm1,mm5);
      
      psrc+=8;
      
      cvtpi2ps_r2r(mm2, xmm0);
      cvtpi2ps_r2r(mm5, xmm1);
      movlhps_r2r(xmm1, xmm0);
      
      mulps_r2r(xmm2,xmm0);
      movaps_r2m(xmm0,*(pdst++));
    }
    psrc += 3*w;
  }
  emms();
  return 0;
}

int MMXShrink8_4Float(unsigned char *src, int w, int h, MMXMatrix *dst) 
{
  sse_t factor;
  mmx_t one;
  int i,j;
  sse_t *pdst;
  unsigned char *psrc=src;

  one.w[0]=one.w[1]=one.w[2]=one.w[3]=1;
  movq_m2r(one,mm1);
  pxor_r2r(mm0,mm0);
  xorps_r2r(xmm0,xmm0);
  xorps_r2r(xmm1,xmm1);
  factor.sf[0]=1.0/32.0;
  factor.sf[1]=1.0/32.0;
  factor.sf[2]=1.0/32.0;
  factor.sf[3]=1.0/32.0;
  movups_m2r(factor,xmm2);

  for (j=0;j<dst->rows;j++) {
    pdst = dst->p[j].s;

    for (i=0;i<(dst->cols+3)/4;i++) {

      /* first 8x4 patch */
      movq_m2r(*psrc,mm2);
      movq_r2r(mm2,mm3);
      punpcklbw_r2r(mm0,mm2);
      punpckhbw_r2r(mm0,mm3);
      paddw_r2r(mm3,mm2);

      movq_m2r(*(psrc+w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      paddw_r2r(mm4,mm3);
      paddw_r2r(mm3,mm2);

      movq_m2r(*(psrc+2*w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      paddw_r2r(mm4,mm3);
      paddw_r2r(mm3,mm2);

      movq_m2r(*(psrc+3*w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      paddw_r2r(mm4,mm3);
      paddw_r2r(mm3,mm2);

      pmaddwd_r2r(mm1,mm2); /* result in mm2 */

      psrc+=8;
      
      /* second 8x4 patch */
      movq_m2r(*psrc,mm5);
      movq_r2r(mm5,mm4);
      punpcklbw_r2r(mm0,mm5);
      punpckhbw_r2r(mm0,mm4);
      paddd_r2r(mm4,mm5);

      movq_m2r(*(psrc+w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      paddw_r2r(mm4,mm3);
      paddw_r2r(mm3,mm5);
      
      movq_m2r(*(psrc+2*w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      paddw_r2r(mm4,mm3);
      paddw_r2r(mm3,mm5);
      
      movq_m2r(*(psrc+3*w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      paddw_r2r(mm4,mm3);
      paddw_r2r(mm3,mm5);
      
      pmaddwd_r2r(mm1,mm5); /* result in mm5 */

      movq_r2r(mm2,mm3);
      punpckldq_r2r(mm5,mm2);
      punpckhdq_r2r(mm5,mm3);
      paddd_r2r(mm3,mm2);
      cvtpi2ps_r2r(mm2, xmm0); /* first 2 pixels in low xmm0 */
      
      psrc+=8;

      /* third 8x4 patch */
      movq_m2r(*psrc,mm2);
      movq_r2r(mm2,mm3);
      punpcklbw_r2r(mm0,mm2);
      punpckhbw_r2r(mm0,mm3);
      paddw_r2r(mm3,mm2);

      movq_m2r(*(psrc+w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      paddw_r2r(mm4,mm3);
      paddw_r2r(mm3,mm2);

      movq_m2r(*(psrc+2*w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      paddw_r2r(mm4,mm3);
      paddw_r2r(mm3,mm2);

      movq_m2r(*(psrc+3*w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      paddw_r2r(mm4,mm3);
      paddw_r2r(mm3,mm2); /* result in mm2 */

      pmaddwd_r2r(mm1,mm2);

      psrc+=8;
      
      /* fourth 8x4 patch */
      movq_m2r(*psrc,mm5);
      movq_r2r(mm5,mm4);
      punpcklbw_r2r(mm0,mm5);
      punpckhbw_r2r(mm0,mm4);
      paddd_r2r(mm4,mm5);

      movq_m2r(*(psrc+w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      paddw_r2r(mm4,mm3);
      paddw_r2r(mm3,mm5);
      
      movq_m2r(*(psrc+2*w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      paddw_r2r(mm4,mm3);
      paddw_r2r(mm3,mm5);
      
      movq_m2r(*(psrc+3*w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      paddw_r2r(mm4,mm3);
      paddw_r2r(mm3,mm5); 
      
      pmaddwd_r2r(mm1,mm5); /* result in mm5 */
      

      psrc+=8;
      
      movq_r2r(mm2,mm3);
      punpckldq_r2r(mm5,mm2);
      punpckhdq_r2r(mm5,mm3);
      paddd_r2r(mm3,mm2);
      cvtpi2ps_r2r(mm2, xmm1); /* second 2 pixels in low xmm1 */

      movlhps_r2r(xmm1, xmm0);
      mulps_r2r(xmm2,xmm0);
      movaps_r2m(xmm0,*(pdst++));
    }
    psrc += 3*w;
  }
  emms();
  return 0;
}

int MMX_MMF32_MMU8(MMXMatrix *src, MMXMatrix *dst) 
{
  int i,j;
#if 0 /* the cvtps2pi instruction is not recognized by the assembler */
  sse_t *psrc;
  mmx_t *pdst,mask;
  mask.b[0]=0xff; mask.b[1]=0; mask.w[1]=0;
  mask.b[4]=0xff; mask.b[5]=0; mask.w[3]=0;
  movq_m2r(mask,mm7);

  for (j=0;j<src->rows;j++) {
    psrc = src->p[j].s;
    pdst = dst->p[j].m;
    for (i=0;i<(dst->cols+3)/4;i++) {
      movaps_m2r(*(psrc++),xmm0);
      cvtps2pi_r2r(xmm0,mm0);
      movhlps_r2r(xmm0,xmm1);
      cvtps2pi_r2r(xmm1,mm1);
      
      movaps_m2r(*(psrc++),xmm2);
      cvtps2pi_r2r(xmm2,mm2);
      movhlps_r2r(xmm2,xmm3);
      cvtps2pi_r2r(xmm3,mm3);
      
      pand_r2r(mm7,mm0);
      pand_r2r(mm7,mm1);
      pand_r2r(mm7,mm2);
      pand_r2r(mm7,mm3);
      
      movq_r2r(mm0,mm4);
      punpckldq_r2r(mm2,mm0);
      punpckldq_r2r(mm2,mm4);
      
      movq_r2r(mm1,mm5);
      punpckldq_r2r(mm3,mm1);
      punpckldq_r2r(mm3,mm5);
      
      pslld_i2r(8,mm4);
      pand_r2r(mm4,mm0);
      pslld_i2r(16,mm1);
      pand_r2r(mm1,mm0);
      pslld_i2r(24,mm5);
      pand_r2r(mm5,mm0);
      
      movq_r2m(mm0,*(pdst++));
    }
  }
  emms();
#else
  float *psrc;
  unsigned char *pdst;
  for (j=0;j<src->rows;j++) {
    psrc = (float *)src->p[j].s;
    pdst = (unsigned char *)dst->p[j].m;
    for (i=0;i<dst->cols;i++) {
      *(pdst++) = (unsigned char) ((*(psrc++)+256.0)/2.0);
    }
  }
#endif
  return 0;
}

int MMX_imgS16_F32(short *src, MMXMatrix *dst)
{
  mmx_t *psrc;
  sse_t *pdst;
  int i,j;

  for (j=0;j<dst->rows;j++) {
    pdst = dst->p[j].s;
    psrc = (mmx_t *)(src+j*dst->cols);
    for (i=0;i<dst->n;i++) {
      movq_m2r(*(psrc++),mm2);
      punpcklwd_r2r(mm2,mm0);
      punpckhwd_r2r(mm2,mm1);
      psrad_i2r(16,mm0);
      psrad_i2r(16,mm1);
      cvtpi2ps_r2r(mm0, xmm0);
      cvtpi2ps_r2r(mm1, xmm1);
      movlhps_r2r(xmm1, xmm0);
      //PRINT_REGMWD("mm2",mm2);
      //emms();
      //PRINT_REGF("xmm0",xmm0);
      //getchar();
      movaps_r2m(xmm0,*(pdst++));
    }
  }
  emms();
  return 0;
}


int MMX_S16_F32(MMXMatrix *src, MMXMatrix *dst)
{
  mmx_t *psrc;
  sse_t *pdst;
  int i,j;

  for (j=0;j<dst->rows;j++) {
    pdst = dst->p[j].s;
    psrc = src->p[j].m;
    for (i=0;i<dst->n;i++) {
      movq_m2r(*(psrc++),mm2);
      punpcklwd_r2r(mm2,mm0);
      punpckhwd_r2r(mm2,mm1);
      psrad_i2r(16,mm0);
      psrad_i2r(16,mm1);
      cvtpi2ps_r2r(mm0, xmm0);
      cvtpi2ps_r2r(mm1, xmm1);
      movlhps_r2r(xmm1, xmm0);
      //PRINT_REGMWD("mm2",mm2);
      //emms();
      //PRINT_REGF("xmm0",xmm0);
      //getchar();
      movaps_r2m(xmm0,*(pdst++));
    }
  }
  emms();
  return 0;
}

#endif /* _PIII */

// Convert from MMX Matrix to continuous unsigned char buffer
int MMX_MMU8_img(MMXMatrix *src, unsigned char *img) 
{
  int j;
  
  for (j=0;j<src->rows;j++)
    memcpy(img+j*src->cols,(char *)src->p[j].m,src->cols);
    
  return 0;
}

// Convert from continuous unsigned char buffer to MMX Matrix
int MMX_img_MMU8(unsigned char *img, MMXMatrix *dst) 
{
  int j;
  
  for (j=0;j<dst->rows;j++)
    memcpy((char *)dst->p[j].m,img+j*dst->cols,dst->cols);
    
  return 0;
}

// Convert from MMX Matrix to stepped unsigned char buffer
int MMX_MMU8_simg(MMXMatrix *src, unsigned char *img, unsigned int step) 
{
  int j;
  
  for (j=0;j<src->rows;j++)
    memcpy(img+j*step,(char *)src->p[j].m,src->cols);
    
  return 0;
}

// Convert from stepped unsigned char buffer to MMX Matrix
int MMX_simg_MMU8(unsigned char *img, unsigned int step, MMXMatrix *dst) 
{
  int j;
  
  for (j=0;j<dst->rows;j++)
    memcpy((char *)dst->p[j].m,img+j*step,dst->cols);
    
  return 0;
}


int MMX_img_MMF32(unsigned char *img,int w, int h, MMXMatrix *dst)
{
  int i,j;

  for (j=0;j<h;j++)
    for (i=0;i<w;i++)
      dst->p[j].s->sf[i]=*(img++);

  return 0;
}

int MMX_U8_S16(unsigned char *src, int width, int height,
	       short *dst)
{
  mmx_t *psrc,*pdst,*last,ma;
  
  psrc=(mmx_t *)src;
  pdst=(mmx_t *)dst;
  last=(mmx_t *)(src+width*height);
  ma.q=0x8080808080808080LL;

  pxor_r2r(mm0,mm0);
  movq_m2r(ma,mm1);  

  while (psrc<last) {
    movq_m2r(*(psrc++),mm2);
    pxor_r2r(mm1,mm2);
    punpcklbw_r2r(mm2, mm3);
    psraw_i2r(8, mm3);
    movq_r2m(mm3,*(pdst++));
    punpckhbw_r2r(mm2, mm3);
    psraw_i2r(8, mm3);
    movq_r2m(mm3,*(pdst++));
  }
  emms();
  return 0;
}

/* int MMX_U8_S16_sum(unsigned char *src, int width, int height,
		   short *dst)
{
  mmx_t *psrc,*pdst,*last,ma;
  
  psrc=(mmx_t *)src;
  pdst=(mmx_t *)dst;
  last=(mmx_t *)(src+width*height);
  ma.q=0x8080808080808080LL;
  movq_m2r(ma,mm1);  
  ma.w[0]=ma.w[1]=ma.w[2]=ma.w[3]=1;
  movq_m2r(ma,mm6);  
  pxor_r2r(mm7,mm7);
  pxor_r2r(mm0,mm0);
  
  while (psrc<last) {
    movq_m2r(*(psrc++),mm2);
    pxor_r2r(mm1,mm2);
    punpcklbw_r2r(mm2, mm3);
    psraw_i2r(8, mm3);
    movq_r2m(mm3,*(pdst++));
    pmaddwd_r2r(mm6,mm3);
    paddd_r2r(mm3,mm7);
    punpckhbw_r2r(mm2, mm3);
    psraw_i2r(8, mm3);
    movq_r2m(mm3,*(pdst++));
    pmaddwd_r2r(mm6,mm3);
    paddd_r2r(mm3,mm7);
  }
  movq_r2m(mm7,ma);
  emms();
  return ma.d[0]+ma.d[1];
} 
*/

int MMX_img_MMS16(unsigned char *src, int width, int height, MMXMatrix *dst)
{
  mmx_t *psrc,*pdst,*last,ma;
  int i,j;

  psrc=(mmx_t *)src;
  pdst=(mmx_t *)dst;
  last=(mmx_t *)(src+width*height);
  ma.q=0x8080808080808080LL;

  pxor_r2r(mm0,mm0);
  movq_m2r(ma,mm1);  

  for (j=0;j<dst->rows;j++) {
    pdst = dst->p[j].m;
    psrc = (mmx_t *)(src+j*width);
    for (i=0;i<dst->n/2;i++) {
      movq_m2r(*(psrc++),mm2);
      pxor_r2r(mm1,mm2);
      punpcklbw_r2r(mm2, mm3);
      psraw_i2r(8, mm3);
      movq_r2m(mm3,*(pdst++));
      punpckhbw_r2r(mm2, mm3);
      psraw_i2r(8, mm3);
      movq_r2m(mm3,*(pdst++));
    }
  }
  emms();
  return 0;
}

int MMX_U16_U8(unsigned short *src, int width, int height, unsigned char *dst)
{
  mmx_t *psrc,*pdst,*last;
  
  psrc=(mmx_t *)src;
  pdst=(mmx_t *)dst;
  last=(mmx_t *)(src+width*height-15);

  while (psrc<last) {
    movq_m2r(*(psrc++),mm0);
    packuswb_m2r(*(psrc++),mm0);
    movq_r2m(mm0,*(pdst++));
  }
  emms();
  return 0;
}

//--------------------------------
// luke's remakes
//--------------------------------
int MMX_U8_F32(MMXMatrix *src, MMXMatrix *dst) {

  mmx_t *psrc;
  sse_t *pdst;
  int i,j;

  pxor_r2r(mm7,mm7);

  for (j=0;j<dst->rows;j++) {
    psrc = src->p[j].m;
    pdst = dst->p[j].s;
    for (i=0;i<dst->cols;i+=8) {
      movq_m2r(*(psrc++),mm2);
      movq_r2r(mm2,mm3);
      punpcklbw_r2r(mm7,mm2);
      punpckhbw_r2r(mm7,mm3);
      movq_r2r(mm2,mm4);
      punpcklwd_r2r(mm7,mm2);
      punpckhwd_r2r(mm7,mm4);
      cvtpi2ps_r2r(mm2,xmm0);
      cvtpi2ps_r2r(mm4,xmm1);
      movlhps_r2r(xmm1,xmm0);
      movaps_r2m(xmm0,*(pdst++));

      movq_r2r(mm3,mm4);
      punpcklwd_r2r(mm7,mm3);
      punpckhwd_r2r(mm7,mm4);
      cvtpi2ps_r2r(mm3,xmm0);
      cvtpi2ps_r2r(mm4,xmm1);
      movlhps_r2r(xmm1,xmm0);
      movaps_r2m(xmm0,*(pdst++));
    }
  }


  return 0;
}
