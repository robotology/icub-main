
#include "MMXOp.h"

// -- luke remake
int MMXAvg(MMXMatrix *src, MMXMatrix *dst)
{
  int i,j;
  mmx_t *psrc;
  mmx_t *pdst;

  
  pxor_r2r(mm7,mm7);

  for (j=0; j<src->rows; j++) {
    psrc = src->p[j].m;
    pdst = dst->p[j].m;
    for (i=0; i<src->cols; i+=8) {
      movq_m2r(*psrc, mm2);
      movq_m2r(*pdst, mm4);
      movq_r2r(mm2, mm3);
      movq_r2r(mm4, mm5);
      punpcklbw_r2r(mm7, mm2);
      punpcklbw_r2r(mm7, mm4);
      punpckhbw_r2r(mm7, mm3);
      punpckhbw_r2r(mm7, mm5);
      paddw_r2r(mm2, mm4);
      paddw_r2r(mm3, mm5);		
      psrlw_i2r(1, mm4);
      psrlw_i2r(1, mm5);
      packuswb_r2r(mm5, mm4);
      movq_r2m(mm4, *pdst);
      psrc++;
      pdst++;
    }
  }
  emms();
  return 0;
}
