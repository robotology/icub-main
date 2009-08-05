
#include <stdio.h>
#include "MMXMatrix.h"

#include "MMXShrink.h"


int MMXExpand_U8(int xfactor, int yfactor, unsigned char *src, int width, int height, unsigned char *dst)
{
   int rc;
   if ((xfactor==2)&&(yfactor==4)) {
     rc=MMXExpand_U8_H2_V4(src, width, height, dst);
   }
   else if ((xfactor==2)&&(yfactor==2)) {
     rc=MMXExpand_U8_H2_V2(src, width, height, dst);
   }
   else {
     fprintf(stderr,"MMXShrink_U8: cant shrink x/%d y/%d\n",xfactor, yfactor);
     rc=-1;
   }
   return rc;
}


int MMXShrink_U8_H2(unsigned char *src, int width, int height,
		    unsigned char *dst) {
  mmx_t *psrc=(mmx_t *)src,*pdst=(mmx_t *)dst;
  mmx_t *end = (mmx_t *)(src+width*height-7);
  mmx_t ma;
  ma.q=0x00FF00FF00FF00FFLL;
  movq_m2r(ma,mm6);
  pxor_r2r(mm0,mm0);
  pxor_r2r(mm7,mm7);
  while (psrc<end) {
    movq_m2r(*(psrc++),mm1);
    movq_m2r(*(psrc++),mm3);
    movq_r2r(mm1,mm2);
    movq_r2r(mm3,mm4);
    pand_r2r(mm6,mm1);
    psrlw_i2r(8,mm2);
    pand_r2r(mm6,mm3);
    psrlw_i2r(8,mm4);
    paddw_r2r(mm2,mm1);
    paddw_r2r(mm4,mm3);
    psraw_i2r(1,mm1);
    psraw_i2r(1,mm3);
    packuswb_r2r(mm3,mm1);
    movq_r2m(mm1,*(pdst++));
  }
  emms();
  return 0;
}


int MMXShrink_U8_H2_V2(unsigned char *src, int width, int height,
		    unsigned char *dst) {
  mmx_t *psrc=(mmx_t *)src,*pdst=(mmx_t *)dst;
  mmx_t *end = (mmx_t *)(src+width*height-7), *endRow;
  mmx_t ma;
  int w = width/8;

  ma.q=0x00FF00FF00FF00FFLL;
  movq_m2r(ma,mm6);
  pxor_r2r(mm0,mm0);
  pxor_r2r(mm7,mm7);

  while (psrc<end) {
    endRow = psrc+w;
    while (psrc<endRow) {
      movq_m2r(*psrc,mm1);
      pavgb_m2r(*(psrc+w),mm1);
      psrc++;

      movq_m2r(*(psrc),mm3);
      pavgb_m2r(*(psrc+w),mm3);
      psrc++;

      movq_r2r(mm1,mm2);
      movq_r2r(mm3,mm4);
      pand_r2r(mm6,mm1);
      psrlw_i2r(8,mm2);
      pand_r2r(mm6,mm3);
      psrlw_i2r(8,mm4);
      paddw_r2r(mm2,mm1);
      paddw_r2r(mm4,mm3);
      psraw_i2r(1,mm1);
      psraw_i2r(1,mm3);
      packuswb_r2r(mm3,mm1);
      movq_r2m(mm1,*(pdst++));
    }
    psrc +=w;
  }
  emms();
  return 0;
}


int MMXShrink_U8_V2(unsigned char *src, int width, int height,
		    unsigned char *dst) {
  mmx_t *psrc=(mmx_t *)src,*pdst=(mmx_t *)dst;
  mmx_t *end = (mmx_t *)(src+width*height-7), *endRow;
  int w = width/8;

  while (psrc<end) {
    endRow = psrc+w/2;
    while (psrc<endRow) {
      movq_m2r(*psrc,mm0);
      pavgb_m2r(*(psrc+w),mm0);
      movq_r2m(mm0,*pdst);
      pdst++;
      psrc++;
    }
    psrc +=w;
  }
  emms();
  return 0;
}

int MMXExpand_U8_H2_V4(unsigned char *src, int width, int height,
		       unsigned char *dst) {
  mmx_t *psrc=(mmx_t *)src,*pdst=(mmx_t *)dst;
  mmx_t *end = (mmx_t *)(src+width*height-7),*endRow;
  int w = width/4;
  
  while (psrc<end) {
    endRow = psrc+w/2;
    while (psrc<endRow) {
      movq_m2r(*(psrc++),mm0);
      movq_r2r(mm0,mm1);
      punpcklbw_r2r(mm0,mm0);
      punpckhbw_r2r(mm1,mm1);
      movq_r2m(mm0,*pdst);
      movq_r2m(mm0,*(pdst+w));
      movq_r2m(mm0,*(pdst+2*w));
      movq_r2m(mm0,*(pdst+3*w));
      pdst++;
      movq_r2m(mm1,*pdst);
      movq_r2m(mm1,*(pdst+w));
      movq_r2m(mm1,*(pdst+2*w));
      movq_r2m(mm1,*(pdst+3*w));
      pdst++;
    }
    pdst += 3*w;
  }
  emms();
  return 0;
}


int MMXExpand_U8_H2_V2(unsigned char *src, int width, int height,
		       unsigned char *dst) {
  mmx_t *psrc=(mmx_t *)src,*pdst=(mmx_t *)dst;
  mmx_t *end = (mmx_t *)(src+width*height-7),*endRow;
  int w = width/4;
  
  while (psrc<end) {
    endRow = psrc+w/2;
    while (psrc<endRow) {
      movq_m2r(*(psrc++),mm0);
      movq_r2r(mm0,mm1);
      punpcklbw_r2r(mm0,mm0);
      punpckhbw_r2r(mm1,mm1);
      movq_r2m(mm0,*pdst);
      movq_r2m(mm0,*(pdst+w));
      pdst++;
      movq_r2m(mm1,*pdst);
      movq_r2m(mm1,*(pdst+w));
      pdst++;
    }
    pdst += w;
  }
  emms();
  return 0;
}

#ifdef _PIII
int MMXShrink_U8_S16_H4_V4(unsigned char *src, int width, int height, short *dst) 
{
  mmx_t *psrc=(mmx_t *)src;
  mmx_t *pdst=(mmx_t *)dst;
  mmx_t *end=(mmx_t *)(src+width*height-15);
  mmx_t *endRow;
  mmx_t one;
  int w = width/8;

  one.w[0]=one.w[1]=one.w[2]=one.w[3]=1;
  movq_m2r(one,mm1);
  pxor_r2r(mm0,mm0);
  while (psrc<end) {
    endRow = psrc+w;
    while (psrc<endRow) {
      movq_m2r(*psrc,mm2);
      //PRINT_REGMUBW("mm2",mm2);
      //movq_m2r(*(psrc+w),mm6);
      //PRINT_REGMUBW("mm2",mm6);
      //movq_m2r(*(psrc+2*w),mm6);
      //PRINT_REGMUBW("mm2",mm6);
      //movq_m2r(*(psrc+3*w),mm6);
      //PRINT_REGMUBW("mm2",mm6);

      pavgb_m2r(*(psrc+w),mm2);
      movq_m2r(*(psrc+2*w),mm3);
      pavgb_m2r(*(psrc+3*w),mm3);
      pavgb_r2r(mm3,mm2);
      //PRINT_REGMUBW("res",mm2);
      movq_r2r(mm2,mm3);
      punpcklbw_r2r(mm0,mm2);
      punpckhbw_r2r(mm0,mm3);
      pmaddwd_r2r(mm1,mm2);
      pmaddwd_r2r(mm1,mm3);
      movq_r2r(mm2,mm4);
      punpckldq_r2r(mm3,mm2);
      punpckhdq_r2r(mm3,mm4);
      paddd_r2r(mm4,mm2);
      psrc++;

      movq_m2r(*psrc,mm3);
      pavgb_m2r(*(psrc+w),mm3);
      movq_m2r(*(psrc+2*w),mm4);
      pavgb_m2r(*(psrc+3*w),mm4);
      pavgb_r2r(mm4,mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      pmaddwd_r2r(mm1,mm3);
      pmaddwd_r2r(mm1,mm4);
      movq_r2r(mm3,mm5);
      punpckldq_r2r(mm4,mm3);
      punpckhdq_r2r(mm4,mm5);
      paddd_r2r(mm5,mm3);
      psrc++;
      
      movq_r2r(mm2,mm4);
      punpcklwd_r2r(mm3,mm2);
      punpckhwd_r2r(mm3,mm4);
      punpcklwd_r2r(mm4,mm2);

      psraw_i2r(2,mm2);
      //PRINT_REGMWD("res",mm2);
      //getchar();
      movq_r2m(mm2,*(pdst++));
    }
    psrc += 3*w;
  }
  emms();
  return 0;
}

int MMXShrink_U8_H4_V2(unsigned char *src, int width, int height, unsigned char *dst)
{
  mmx_t *psrc=(mmx_t *)src;
  mmx_t *pdst=(mmx_t *)dst;
  mmx_t *end=(mmx_t *)(src+width*height-15);
  mmx_t *endRow;
  mmx_t one;
  int w = width/8;

  one.w[0]=one.w[1]=one.w[2]=one.w[3]=1;
  movq_m2r(one,mm1);
  pxor_r2r(mm0,mm0);
  while (psrc<end) {
    endRow = psrc+w;
    while (psrc<endRow) {
      movq_m2r(*psrc,mm2);
      pavgb_m2r(*(psrc+w),mm2);
      movq_r2r(mm2,mm3);
      punpcklbw_r2r(mm0,mm2);
      punpckhbw_r2r(mm0,mm3);
      pmaddwd_r2r(mm1,mm2);
      pmaddwd_r2r(mm1,mm3);
      movq_r2r(mm2,mm4);
      punpckldq_r2r(mm3,mm2);
      punpckhdq_r2r(mm3,mm4);
      paddd_r2r(mm4,mm2);
      psrc++;

      movq_m2r(*psrc,mm3);
      pavgb_m2r(*(psrc+w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      pmaddwd_r2r(mm1,mm3);
      pmaddwd_r2r(mm1,mm4);
      movq_r2r(mm3,mm5);
      punpckldq_r2r(mm4,mm3);
      punpckhdq_r2r(mm4,mm5);
      paddd_r2r(mm5,mm3);
      psrc++;
      
      movq_r2r(mm2,mm4);
      punpcklwd_r2r(mm3,mm2);
      punpckhwd_r2r(mm3,mm4);
      punpcklwd_r2r(mm4,mm2);
      psraw_i2r(2,mm2);

      movq_r2r(mm2,mm6);
      movq_m2r(*psrc,mm2);
      pavgb_m2r(*(psrc+w),mm2);
      movq_r2r(mm2,mm3);
      punpcklbw_r2r(mm0,mm2);
      punpckhbw_r2r(mm0,mm3);
      pmaddwd_r2r(mm1,mm2);
      pmaddwd_r2r(mm1,mm3);
      movq_r2r(mm2,mm4);
      punpckldq_r2r(mm3,mm2);
      punpckhdq_r2r(mm3,mm4);
      paddd_r2r(mm4,mm2);
      psrc++;

      movq_m2r(*psrc,mm3);
      pavgb_m2r(*(psrc+w),mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      pmaddwd_r2r(mm1,mm3);
      pmaddwd_r2r(mm1,mm4);
      movq_r2r(mm3,mm5);
      punpckldq_r2r(mm4,mm3);
      punpckhdq_r2r(mm4,mm5);
      paddd_r2r(mm5,mm3);
      psrc++;
      
      movq_r2r(mm2,mm4);
      punpcklwd_r2r(mm3,mm2);
      punpckhwd_r2r(mm3,mm4);
      punpcklwd_r2r(mm4,mm2);
      psraw_i2r(2,mm2);

      packuswb_r2r(mm2,mm6);
      movq_r2m(mm6,*(pdst++));
    }
    psrc += w;
  }
  emms();
  return 0;
}



int MMXShrink_U8_H4_V4(unsigned char *src, int width, int height, unsigned char *dst)
{
  mmx_t *psrc=(mmx_t *)src;
  mmx_t *pdst=(mmx_t *)dst;
  mmx_t *end=(mmx_t *)(src+width*height-15);
  mmx_t *endRow;
  mmx_t one;
  int w = width/8;

  one.w[0]=one.w[1]=one.w[2]=one.w[3]=1;
  movq_m2r(one,mm1);
  pxor_r2r(mm0,mm0);
  while (psrc<end) {
    endRow = psrc+w;
    while (psrc<endRow) {
      movq_m2r(*psrc,mm2);
      pavgb_m2r(*(psrc+w),mm2);
      movq_m2r(*(psrc+2*w),mm3);
      pavgb_m2r(*(psrc+3*w),mm3);
      pavgb_r2r(mm3,mm2);
      movq_r2r(mm2,mm3);
      punpcklbw_r2r(mm0,mm2);
      punpckhbw_r2r(mm0,mm3);
      pmaddwd_r2r(mm1,mm2);
      pmaddwd_r2r(mm1,mm3);
      movq_r2r(mm2,mm4);
      punpckldq_r2r(mm3,mm2);
      punpckhdq_r2r(mm3,mm4);
      paddd_r2r(mm4,mm2);
      psrc++;

      movq_m2r(*psrc,mm3);
      pavgb_m2r(*(psrc+w),mm3);
      movq_m2r(*(psrc+2*w),mm4);
      pavgb_m2r(*(psrc+3*w),mm4);
      pavgb_r2r(mm4,mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      pmaddwd_r2r(mm1,mm3);
      pmaddwd_r2r(mm1,mm4);
      movq_r2r(mm3,mm5);
      punpckldq_r2r(mm4,mm3);
      punpckhdq_r2r(mm4,mm5);
      paddd_r2r(mm5,mm3);
      psrc++;

      movq_r2r(mm2,mm4);
      punpcklwd_r2r(mm3,mm2);
      punpckhwd_r2r(mm3,mm4);
      punpcklwd_r2r(mm4,mm2);
      psraw_i2r(2,mm2);

      movq_r2r(mm2,mm6);
      movq_m2r(*psrc,mm2);
      pavgb_m2r(*(psrc+w),mm2);
      movq_m2r(*(psrc+2*w),mm3);
      pavgb_m2r(*(psrc+3*w),mm3);
      pavgb_r2r(mm3,mm2);
      movq_r2r(mm2,mm3);
      punpcklbw_r2r(mm0,mm2);
      punpckhbw_r2r(mm0,mm3);
      pmaddwd_r2r(mm1,mm2);
      pmaddwd_r2r(mm1,mm3);
      movq_r2r(mm2,mm4);
      punpckldq_r2r(mm3,mm2);
      punpckhdq_r2r(mm3,mm4);
      paddd_r2r(mm4,mm2);
      psrc++;

      movq_m2r(*psrc,mm3);
      pavgb_m2r(*(psrc+w),mm3);
      movq_m2r(*(psrc+2*w),mm4);
      pavgb_m2r(*(psrc+3*w),mm4);
      pavgb_r2r(mm4,mm3);
      movq_r2r(mm3,mm4);
      punpcklbw_r2r(mm0,mm3);
      punpckhbw_r2r(mm0,mm4);
      pmaddwd_r2r(mm1,mm3);
      pmaddwd_r2r(mm1,mm4);
      movq_r2r(mm3,mm5);
      punpckldq_r2r(mm4,mm3);
      punpckhdq_r2r(mm4,mm5);
      paddd_r2r(mm5,mm3);
      psrc++;

      movq_r2r(mm2,mm4);
      punpcklwd_r2r(mm3,mm2);
      punpckhwd_r2r(mm3,mm4);
      punpcklwd_r2r(mm4,mm2);
      psraw_i2r(2,mm2);

      packuswb_r2r(mm2,mm6);
      movq_r2m(mm6,*(pdst++));
    }
    psrc += 3*w;
  }
  emms();
  return 0;
}


#endif /* _PIII */


int MMXShrink_U8(int xfactor, int yfactor, unsigned char *src, int width, int height, unsigned char *dst)
{
   int rc;
   if ((xfactor==2)&&(yfactor==1)) {
     rc=MMXShrink_U8_H2(src, width, height, dst);
   }
   else if ((xfactor==1)&&(yfactor==2)) {
     rc=MMXShrink_U8_V2(src, width, height, dst);
   }
   else if ((xfactor==2)&&(yfactor==2)) {
     rc=MMXShrink_U8_H2_V2(src, width, height, dst);
   }

#ifdef _PIII
   else if ((xfactor==4)&&(yfactor==4)) {
     rc=MMXShrink_U8_H4_V4(src, width, height, dst);
   }
   else if ((xfactor==4)&&(yfactor==2)) {
     rc=MMXShrink_U8_H4_V2(src, width, height, dst);
   }
#endif
   else {
 		  fprintf(stderr, "MMXShrink_U8: cant shrink x/%d y/%d\n",xfactor, yfactor);
 			 rc=-1;
   }
   return rc;
}
