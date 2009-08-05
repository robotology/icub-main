#if 1
int dummy() {
  // dummy
  return 0;
}

#else
#include "MMXCorrel.h"


/*
 * Add arrays of n unsigned 32-bit ints element by element.
 * Output array must be an even number of elements long.
 */

inline void
add_rows(int n, MM_S_32 *res, MM_S_32 *add)
{
  for (; n > 0; n -= 2) {
    movq_m2r(*res,mm0);
    paddd_m2r(*add,mm0);
    add += 2;
    movq_r2m(mm0, *res);
    res += 2;
  }
  //emms();
}

/*
 * Subtract arrays of n unsigned 32-bit ints element by element.
 * Output array must be an even number of elements long.
 */

inline void
sub_rows(int n, MM_S_32 *res, MM_S_32 *sub)
{
  for (; n > 0; n -= 2) {
    movq_m2r(*res,mm0);
    psubd_m2r(*sub,mm0);
    sub += 2;
    movq_r2m(mm0, *res);
    res += 2;
  }
  //emms();
}

/*
 * Copy an array of n unsigned 32-bit ints.
 * Output array must be an even number of elements long.
 */

inline void
copy_row(int n, MM_S_32 *dst, MM_S_32 *src)
{
  for (; n > 0; n -= 2) {
    movq_m2r(*src,mm0);
    src += 2;
    movq_r2m(mm0, *dst);
    dst += 2;
  }
  //emms();
}

/*
 * Zero an array of n unsigned 32-bit ints.
 * Output array must be an even number of elements long.
 */

inline void
zero_row(int n, MM_S_32 *dst)
{
  pxor_r2r(mm0,mm0);
  for (; n > 0; n -= 2) {
    movq_r2m(mm0, *dst);
    dst += 2;
  }
  //emms();
}

/*
 * Zero extend an array of 8-bit values to 16-bit values.  A multiple
 * of 4 16-bit values will be written so the output array may need
 * to be padded to leave room for extra values.
 */

inline void
extend8_16(MM_U_8 *d8, MM_U_16 *d16, int cols)
{
  pxor_r2r(mm0,mm0);
  for (cols = (cols - 1) / 4; cols > 0; cols -= 2) {
    movq_m2r(*d8, 1);
    d8 += 8;
    movq_r2r(mm1, 2);
    punpcklbw_r2r(mm0, 1);
    punpckhbw_r2r(mm0, 2);
    movq_r2m(mm1, *d16);
    d16 += 4;
    movq_r2m(mm2, *d16);
    d16 += 4;
  }
  if (cols == 0) {
    movq_m2r(*d8, 1);
    punpcklbw_r2r(mm0, 1);
    movq_r2m(mm1, *d16);
  }
	//emms();
}

/*
 * Shift an vector of cols 16-bit values one place.
 * The destination vector must be a multiple of four elements long.
 */

inline void
shift16(MM_U_16 *src, MM_U_16 *dst, int cols)
{
  pxor_r2r(mm2, 2);
  for (; cols > 0; cols -= 4) {
    movq_m2r(*src,mm0);
    src += 4;
    movq_r2r(mm0, 1);
    psllq_i2r(16,mm0);
    psrlq_i2r(48, 1);
    por_r2r(mm2,mm0);
    movq_r2m(mm0, *dst);
    dst += 4;
    movq_r2r(mm1, 2);
  }
  //emms();
}

/*
 * Expand a template row so that Ts shifted copies appear, e.g.,
 * the template row `1234 56--' becomes (for Ts = 4) :
 * `1234 -123 --12 ---1 56-- 456- 3456 2345 ---- ---- ---- 6---'
 * Note that a multiple of Ts words is generated and that empty elements
 * (denoted `-' above) are set to 0.  Elements are 16-bit and words are
 * 64-bit.
 */
inline void
templshift(MM_U_16 *d, int cols, int wid)
{
  int i, j;
  MM_U_16 *e, *f;
  
  /* ensure sufficient zero-padding: d[cols]...d[wid-1] = 0 */
  for (i = wid - cols, e = d + cols; i > 0; i--, e++)
    *e = 0;
  
  /* spread incoming array in-place: d[0]->d[0], d[1]->d[4], d[2]->d[8], ...*/
  e = d + wid;
  f = d + wid * Ts;
  for (i = wid - Ts; i > 0; i -= Ts) {
    e -= Ts;
    f -= Ts * Ts;
    movq_m2r(*e,mm0);
    movq_r2m(mm0, *f);
  }
  
  /* calculate the shifted values */
  e = d;
  for (i = Ts - 1; i > 0; i--) {
    f = e;
    e += Ts;
    pxor_r2r(mm2, 2);
    for (j = wid; j > 0; j -= Ts) {
      movq_m2r(*f,mm0);
      movq_r2r(mm0, 1);
      psllq_i2r(16,mm0);
      psrlq_i2r(48, 1);
      por_r2r(mm2,mm0);
      movq_r2r(mm1, 2);
      movq_r2m(mm0, *(f + Ts));
      f += Ts * Ts;
    }
  }
  //emms();
}

/*
 * Calculate all sums of consecutive sequences of len values in
 * the input array containing n values giving an output array
 * of n - len + 1 values.  Also compute a second output array with
 * sums of squares.
 */

inline void
sum_row(MM_U_16 *d, int n, int len, MM_S_32 *r, MM_S_32 *r2)
{
  int i;
  MM_S_32 tmp, sum, sum2;
  
  sum = sum2 = 0;
  for (i = len - 1; i > 0; i--) {
    tmp = *d++;
    sum += tmp;
    sum2 += tmp * tmp;
  }
  for (i = n - len + 1; i > 0; i--) {
    tmp = *d++;
    sum += tmp;
    sum2 += tmp * tmp;
    *r++ = sum;
    *r2++ = sum2;
    tmp = *(d - len);
    sum -= tmp;
    sum2 -= tmp * tmp;
  }
}


void
MMXCorrelate_SAD(MMXMatrix *I, MMXMatrix *T, double * best_cor, double * all_cor, 
		 int * best_x, int * best_y)
{
  int i, j, k, l, x;
  int rrows, rcols;
  int sum, min_sum;
  unsigned char result[8];
  mmx_t mmxresult32;
  double tmp, cor;
  int tcols;
  rrows = I->rows - T->rows + 1;
  rcols = I->cols - T->cols + 1;
  
  min_sum = 0;
  tcols = T->cols/8;
  //  pxor_r2r (mm7,mm7);
  
  /* for each poss template in image */
  for (j = 0; j < rrows; j++) {
    for (i = 0; i < rcols; i++) {
      
      // fprintf(stderr,"IMAGE FROM PIXEL %d, %d\n",j,i)
      //   i = j = 0; 
      pxor_r2r (mm3,mm3);
      pxor_r2r (mm4,mm4); 
      pxor_r2r (mm7,mm7);
      /* correlate using MMX */
      for (l = 0; l < T->rows;) {
	for (k = 0; k < tcols;) {
	  //fprintf(stderr,"template pixel %d %d\n", k,l);
	  movq_m2r (((MM_U_8 *)I->p[l+j].m)[k*8+i],mm0);  
          //PRINTREG8(mm0,"mmx0");
	  
	 
	  psubusb_m2r (T->p[l][k],mm0); 
	  movq_m2r (T->p[l][k],mm1); 
	  //  PRINTREG8(mm1,"mmx1");
	  psubusb_m2r (((MM_U_8 *)I->p[l+j].m)[k*8+i], 1);
	  por_r2r (mm0, 1);
	  
	  
	  // 	  PRINTREG8(mm1,"abs diff");
	  movq_r2r (mm1,mm2);
	  punpcklbw_r2r(mm7,mm1);
	  punpckhbw_r2r(mm7,mm2);
	  //PRINTREG16(mm1,"lbw");
	  //PRINTREG16(mm2,"hbw");
	  
	  paddw_r2r (mm1,mm3);
	  k++;
	  paddw_r2r (mm2,mm3);
	  //PRINTREG16(mm3,"sum16");
	  
	}
	l++;
      }
      movq_r2r (mm3,mm4);
      punpcklwd_r2r (mm7,mm3);
      punpckhwd_r2r (mm7,mm4);
      paddd_r2r (mm3,mm4);
      //PRINTREG32 (mm4,"sum32");
      
      movq_r2m (mm4,mmxresult32);
      
      emms();
      sum = mmxresult32.ud[0]+mmxresult32.mmu32[1];
      cor = (double) (((double)(abs(sum-65536)))/65536.0);
      //      fprintf(stderr,"cor = %.2f\n",cor);
      all_cor[i+j*rrows] = cor;
      if (cor > *best_cor) {
	*best_cor = cor;
	*best_x = i;
	*best_y = j;
      }
      
    }
  }
  // fprintf(stderr,"best cor = %.2f, at %d %d\n",*best_cor, *best_x, *best_y);
}
  
int MMXCorrelate_sad_16(unsigned char * template, unsigned char * image) {
  
  
  int k, l;
  int sum; float t;
  mmx_t mmxresult32;
  
  image_to_matrix (image, image_matrix, 16, 16, 
		   0, 0, 16, 16);
  image_to_matrix (template, template_matrix, 
		   16, 16, 0,0, 16, 16);
  //   t = GetTime();
  //    fprintf(stderr,"setup t = %.4f\n",t);
  //ResetTime();
  pxor_r2r (mm3,mm3);
  pxor_r2r (mm4,mm4); 
  pxor_r2r (mm7,mm7);
  /* correlate using MMX */
  for (l = 0; l < 16;) {
    for (k = 0; k < 2;) {
      movq_m2r (image_matrix->p[l][k],mm0);  
      psubusb_m2r (template_matrix->p[l][k],mm0);
      movq_m2r (template_matrix->p[l][k],mm1);
      psubusb_m2r (image_matrix->p[l][k], 1);
      por_r2r (mm0, 1);
      
      movq_r2r (mm1,mm2);
      punpcklbw_r2r(mm7,mm1);
      punpckhbw_r2r(mm7,mm2);
      
      paddw_r2r (mm1,mm3);
      k++;
      paddw_r2r (mm2,mm3);emms();
    }
    l++;
  }
  movq_r2r (mm3,mm4);
  punpcklwd_r2r (mm7,mm3);
  punpckhwd_r2r (mm7,mm4);
  paddd_r2r (mm3,mm4);
  movq_r2m (mm4,mmxresult32);
  
  emms();
  sum = mmxresult32.ud[0]+mmxresult32.mmu32[1];
  return(sum);
}

void
MMXNormCorrelate(MMXMatrix *I, MMXMatrix *T, double * best_cor, double * all_cor, 
		 int * best_x, int * best_y)
{
  int i, j, k, l;
  int trow;
  MMXMatrix *R; /* result matrix */
  MMXMatrix *TI; /* temporary matrix for I */
  mmx_t **tip; /* extra long row pointer matrix for above */
  MMXMatrix *TSV; /* temporary matrix for SumV */
  MMXMatrix *TSV2; /* temporary matrix for SumVSqr */
  MMXMatrix *TT; /* temporary matrix for Ts shifted copies of T */
  int ttwid; /* template width in 64-bit words */
  int tmp;
  int SU, SUSqr;
  double SumU, SumUSqr;
  double UBar, VBar, N, USD,  term;
  
  
  
  *best_cor = -1.0;
  
  

  if (I->type != MMT_U_8 || T->type != MMT_U_8)
    Error(mm1, "Correlate: bad data type\n");
  if (I->rows < T->rows || I->cols < T->cols)
    Error(mm1, "Correlate: template too large\n");
  
  SU = SUSqr = 0;
  for (i = 0; i < T->rows; i++)
    for (j = 0; j < T->cols; j++)
      {
	tmp = ((MM_U_8 *)T->p[i].m)[j];
	SU += tmp;
	SUSqr += tmp * tmp;
      }
  SumU = (double)SU;
  SumUSqr = (double)SUSqr;
  N = (double)(T->rows * T->cols);
  UBar = SU / N;
  USD = SUSqr / N - UBar * UBar;
  
  ttwid = (T->cols + 2 * Ts - 2) / Ts;
  TT = MMXMatrixAlloc(MMT_U_16, T->rows, ttwid * Ts * Ts);
  R = MMXMatrixAlloc(sizeof(MM_S_32), I->rows - T->rows + 1,
		     I->cols - T->cols + 1);
  TSV = MMXMatrixAlloc(MMT_U_32, T->rows + 1, I->cols - T->cols + 1);
  TSV2 = MMXMatrixAlloc(MMT_U_32, T->rows + 1, I->cols - T->cols + 1);
  TI = MMXMatrixAlloc(MMT_U_16, T->rows, I->cols);
  tip = (mmx_t **)Alloc(sizeof(mmxdata *) * (T->rows * 2));
  
  
  //	fprintf(stderr,"I = %dx%d, T = %dx%d, R = %dx%d\n",I->rows,I->cols,T->rows,T->cols,R->rows,R->cols);
  
  emms();
  for (i = 0; i < T->rows; i++) {
    extend8_16((MM_U_8 *)T->p[i], (MM_U_16 *)TT->p[i], T->cols);
    templshift((MM_U_16 *)TT->p[i], T->cols, ttwid * Ts);
    tip[i] = tip[i + T->rows] = TI->p[i];
  }
  
  TI->p = tip;
  
  zero_row(TSV->cols, (MM_U_32 *)TSV->p[T->rows]);
  zero_row(TSV2->cols, (MM_U_32 *)TSV2->p[T->rows]);
  for (trow = 0; trow < TI->rows - 1; trow++) {
    extend8_16((MM_U_8 *)I->p[trow], (MM_U_16 *)TI->p[trow], TI->cols);
    sum_row((MM_U_16 *)TI->p[trow], TI->cols, T->cols,
	    (MM_U_32 *)TSV->p[trow], (MM_U_32 *)TSV2->p[trow]);
    add_rows(TSV->cols, (MM_U_32 *)TSV->p[T->rows],
	     (MM_U_32 *)TSV->p[trow]);
    add_rows(TSV2->cols, (MM_U_32 *)TSV2->p[T->rows],
	     (MM_U_32 *)TSV2->p[trow]);
  }
  for (i = 0;;) {
    extend8_16((MM_U_8 *)I->p[i + T->rows - 1],
	       (MM_U_16 *)TI->p[trow], TI->cols);
    TI->p += (i % T->rows);
    //		myCount++;
    conv16_32(TI, TT, (MM_U_32 *)R->p[i], R->cols, ttwid);
    TI->p = tip;
    sum_row((MM_U_16 *)TI->p[trow], TI->cols, T->cols,
	    (MM_U_32 *)TSV->p[trow], (MM_U_32 *)TSV2->p[trow]);
    add_rows(TSV->cols, (MM_U_32 *)TSV->p[T->rows],
	     (MM_U_32 *)TSV->p[trow]);
    add_rows(TSV2->cols, (MM_U_32 *)TSV2->p[T->rows],
	     (MM_U_32 *)TSV2->p[trow]);
    
    emms();
    
    for (j = 0; j < R->cols; j++) {
      
      VBar = ((MM_U_32 *)TSV->p[T->rows].m)[j] / N;
      
      term = (((MM_U_32 *)TSV2->p[T->rows].m)[j] / N - VBar * VBar) * USD;
      
      if (term>0) 
	all_cor[i*R->cols+j] = (double)((((MM_U_32 *)R->p[i].m)[j] / N - UBar * VBar) / sqrt(term));
      else {
	all_cor[i*R->cols+j] = 0;
	//fprintf(stderr,"0 correction(%d,%d)\n",i,j);
      }
      /*    ((float *)R->p[i].m)[j] = 
	    (float)((((MM_U_32 *)R->p[i].m)[j] / N - UBar * VBar) /
	    sqrt((((MM_U_32 *)TSV2->p[T->rows].m)[j] / 
	    N - VBar * VBar) * USD));
      */
      
      //	      	fprintf(stderr," [%d] = %.2f ",i*R->cols+j,all_cor[i*R->cols+j]);
      if (all_cor[i*R->cols+j] > *best_cor) {
	*best_y = i;
	*best_x = j;
	*best_cor = all_cor[i*R->cols+j];
      }
    }
    emms();
    
    if (++i >= R->rows)
      break;
    trow = (trow + 1) % T->rows;
    sub_rows(TSV->cols, (MM_U_32 *)TSV->p[T->rows],
	     (MM_U_32 *)TSV->p[trow]);
    sub_rows(TSV2->cols, (MM_U_32 *)TSV2->p[T->rows],
	     (MM_U_32 *)TSV2->p[trow]);
    
  }	
  
  free(TI);
  free(tip);
  free(TSV);
  free(TSV2);
  free(TT);
  free (R);
  /*
    debug = 1;
    PrintMMXMatrix(R,"result");
    debug = 0;
  */
  //	return (R);
  
  //fprintf(stderr,"mmx:%f  ",*best_cor);
  emms();
  
}

void
NormCorrelate(MMXMatrix *I, MMXMatrix *T,double * best_cor, double * all_cor, 
		 int * best_x, int * best_y)
{
  int i, j, k, l;
  MMXMatrix *R;
  MM_U_32 SumV, SumVSqr, SumUV, tmp;
  MM_U_32 SU, SUSqr;
  double SumU, SumUSqr;
  double UBar, VBar, N, USD, term;
  
  if (I->type != MMT_U_8 || T->type != MMT_U_8)
    Error(mm1, "Correlate: bad data type\n");
  if (I->rows < T->rows || I->cols < T->cols)
    Error(mm1, "Correlate: template too large\n");
  R = MMXMatrixAlloc(sizeof(MM_S_32), I->rows - T->rows + 1,
		     I->cols - T->cols + 1);
  
  SU = SUSqr = 0;
  for (i = 0; i < T->rows; i++)
    for (j = 0; j < T->cols; j++) {
      tmp = ((MM_U_8 *)T->p[i].m)[j];
      SU += tmp;
      SUSqr += tmp * tmp;
    }
  SumU = (double)SU;
  SumUSqr = (double)SUSqr;
  N = (double)(T->rows * T->cols);
  UBar = SU / N;
  USD = SUSqr / N - UBar * UBar;
  
  for (i = 0; i < R->rows; i++)
    for (j = 0; j < R->cols; j++) {
      SumV = SumVSqr = SumUV = 0;
      for (k = 0; k < T->rows; k++)
	for (l = 0; l < T->cols; l++) {
	  tmp = ((MM_U_8 *)I->p[i + k].m)[j + l];
	  SumV += tmp;
	  SumVSqr += tmp * tmp;
	  SumUV += tmp * ((MM_U_8 *)T->p[k].m)[l];
	}
      // ((MM_S_32 *)R->p[i].m)[j] = SumV;
      // ((MM_S_32 *)R->p[i].m)[j] = SumVSqr;
      // ((MM_S_32 *)R->p[i].m)[j] = SumUV;
      VBar = SumV / N;
      /*
	((float *)R->p[i].m)[j] = (float)((SumUV / N - UBar * VBar) /
	sqrt((SumVSqr / N - VBar * VBar) * USD));	
      */	
      term = (SumVSqr / N - VBar * VBar) * USD;
      if (term>0)
	all_cor[i*R->cols+j] = (double)((SumUV / N - UBar * VBar) /sqrt(term));
      else { 
	all_cor[i*R->cols+j] =0;
	//fprintf(stderr,"0 correction(%d,%d)\n",i,j);
      }
      if (all_cor[i*R->cols+j] > *best_cor) {
	*best_y = i;
	*best_x = j;
	*best_cor = all_cor[i*R->cols+j];
      }
    }
}


#endif
