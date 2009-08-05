
#include <stdio.h>
#include "MMXyuv.h"

int yuv422_2_y_mmx(unsigned char *src, unsigned char *dst, int n)
{
  //return yuv422_2_y_mmx_YUYV(src,dst,n);
  return yuv422_2_y_mmx_UYVY(src,dst,n);
}


int yuv422_2_y_mmx_UYVY(unsigned char *src, unsigned char *dst, int n)
{
	mmx_t *psrc = (mmx_t *)src;
	mmx_t *end = (mmx_t *)(src+n);
	mmx_t *pdst = (mmx_t *)dst;
	mmx_t ma[2];
	ma[0].q = 0x00ff00ff00ff00ffULL;
	movq_m2r(*ma,mm2);
	movq_r2r(mm2,mm3);
	while (psrc<end) {
		movq_m2r(*(psrc++),mm0);
		movq_m2r(*(psrc++),mm1);
		//psrlq_i2r(8, mm0);
		//psrlq_i2r(8, mm1);
		pand_r2r(mm2,mm0);
		pand_r2r(mm3,mm1);
		packuswb_r2r(mm1,mm0);
		movq_r2m(mm0,*(pdst++));
	}
	emms();
	return 0;
}

int yuv422_2_y_mmx_YUYV(unsigned char *src, unsigned char *dst, int n)
{
	mmx_t *psrc = (mmx_t *)src;
	mmx_t *end = (mmx_t *)(src+n);
	mmx_t *pdst = (mmx_t *)dst;
	mmx_t ma[2];
	ma[0].q = 0x00ff00ff00ff00ffULL;
	movq_m2r(*ma,mm2);
	movq_r2r(mm2,mm3);
	while (psrc<end) {
		movq_m2r(*(psrc++),mm0);
		movq_m2r(*(psrc++),mm1);
		psrlq_i2r(8, mm0);
		psrlq_i2r(8, mm1);
		pand_r2r(mm2,mm0);
		pand_r2r(mm3,mm1);
		packuswb_r2r(mm1,mm0);
		movq_r2m(mm0,*(pdst++));
	}
	emms();
	return 0;
}

int yuv422_2_y_mmx2(unsigned char *src, unsigned char *dst1, 
		    unsigned char *dst2, int w, int h)
{
	mmx_t *psrc = (mmx_t *)src;
	mmx_t *end = (mmx_t *)(src+2*w*h);
	mmx_t *endRow;
	mmx_t *pdst1 = (mmx_t *)dst1;
	mmx_t *pdst2 = (mmx_t *)dst2;
	mmx_t ma[2];
	ma[0].q = 0x00ff00ff00ff00ffULL;
	movq_m2r(*ma,mm2);
	movq_r2r(mm2,mm3);
	ma[0].q = 0x00000000ffffffffULL;
	movq_m2r(*ma,mm4);
	while (psrc<end) {
		endRow = psrc+w/4;
		while (psrc<endRow) {
			movq_m2r(*(psrc++), mm0);
			movq_m2r(*(psrc++), mm1);
			movq_r2r(mm0, mm6);
			movq_r2r(mm1, mm7);
			psrlq_i2r(8, mm0);
			psrlq_i2r(8, mm1);
			pand_r2r(mm2,mm0);
			pand_r2r(mm3,mm1);
			packuswb_r2r(mm1,mm0);
			movq_r2m(mm0,*(pdst1++));
			pand_r2r(mm4,mm6);
			psllq_i2r(32, mm7);
			por_r2r(mm7,mm6);
			movq_r2m(mm6,*(pdst2++));
		}
		endRow = psrc+w/4;
		while (psrc<endRow) {
			movq_m2r(*(psrc++), mm0);
			movq_m2r(*(psrc++), mm1);
			psrlq_i2r(8, mm0);
			psrlq_i2r(8, mm1);
			pand_r2r(mm2,mm0);
			pand_r2r(mm3,mm1);
			packuswb_r2r(mm1,mm0);
			movq_r2m(mm0,*(pdst1++));
		}
	}
	emms();
	return 0;
}

/* Shrink 2x2 a YUV422 image into a Y image */

int MMXShrink_YUV_U8_H2_V2(unsigned char *src, int width, int height,
			   unsigned char *dst) 
{
	int w2 = 2*width;
	unsigned char *psrc=src,*pdst=dst;
	unsigned char *end=src+w2*(height-1);
	unsigned char *endRow;
	mmx_t ma;

	if (width%16) {
		fprintf(stderr, 
			"MMXShrink_YUV_U8_H2_V2 error width=%d not supported\n",
			width);
		return -1;
	}
	ma.q=0x00FF00FF00FF00FFLL;
	movq_m2r(ma,mm6);
	movq_r2r(mm6,mm7);
	ma.w[0]=ma.w[1]=ma.w[2]=ma.w[3]=1;
	movq_m2r(ma,mm5);

	while (psrc<end) {
		endRow = psrc+w2;
		while (psrc<endRow) {

			movq_m2r(*psrc,mm0);
			movq_m2r(*(psrc+8),mm1);
			psrlq_i2r(8,mm0);
			psrlq_i2r(8,mm1);
			pand_r2r(mm6,mm0);
			pand_r2r(mm7,mm1);

			movq_m2r(*(psrc+w2),mm2);
			movq_m2r(*(psrc+w2+8),mm3);
			psrlq_i2r(8,mm2);
			psrlq_i2r(8,mm3);
			pand_r2r(mm6,mm2);
			pand_r2r(mm7,mm3);

			paddw_r2r(mm2,mm0);
			paddw_r2r(mm3,mm1);
			pmaddwd_r2r(mm5,mm0);
			pmaddwd_r2r(mm5,mm1);
			packssdw_r2r(mm1,mm0);


			movq_m2r(*(psrc+16),mm1);
			movq_m2r(*(psrc+24),mm2);
			psrlq_i2r(8,mm1);
			psrlq_i2r(8,mm2);
			pand_r2r(mm6,mm1);
			pand_r2r(mm7,mm2);

			movq_m2r(*(psrc+w2+16),mm3);
			movq_m2r(*(psrc+w2+24),mm4);
			psrlq_i2r(8,mm3);
			psrlq_i2r(8,mm4);
			pand_r2r(mm6,mm3);
			pand_r2r(mm7,mm4);

			paddw_r2r(mm3,mm1);
			paddw_r2r(mm4,mm2);
			pmaddwd_r2r(mm5,mm1);
			pmaddwd_r2r(mm5,mm2);
			packssdw_r2r(mm2,mm1);

			
			psraw_i2r(2,mm0);
			psraw_i2r(2,mm1);
			packuswb_r2r(mm1,mm0);
			movq_r2m(mm0,*pdst);
			
			psrc += 32;
			pdst += 8;
		}
		psrc += w2;
	}
	emms();
	return 0;
}


/* Expand 2x2 a YUV422 image into a Y image */
int MMXExpand_YUV_U8_H2_V2(unsigned char *src, int width, int height,
			   unsigned char *dst) 
{
	int w2 = 2*width;
	unsigned char *psrc=src,*pdst=dst;
	unsigned char *end=src+w2*height;
	unsigned char *endRow;
	mmx_t ma[2];

	if (width%8) return -1;

	ma[0].q=0xFF00FF00FF00FF00LL;
	movq_m2r(*ma,mm6);

	while (psrc<end) {
		endRow = psrc+w2;
		while (psrc<endRow) {
			movq_m2r(*psrc, mm0);
			pand_r2r(mm6,mm0);
			movq_r2r(mm0,mm1);
			psrlq_i2r(8,mm1);
			por_r2r(mm1,mm0);
			movq_r2m(mm0,*pdst);
			movq_r2m(mm0,*(pdst+w2));
			psrc+=8;
			pdst+=8;
		}
		pdst += w2;
	}
	emms();
	return 0;
}
			
/* Convert a YUV422 image into a Y image coded on 16 bits... return
   sum of Y pixels */


/* int MMX_YUV_S16_sum(unsigned char *src, short *dst, int n) 
{
	mmx_t *psrc = (mmx_t *)src;
	mmx_t *pdst = (mmx_t *)dst;
	mmx_t *end = (mmx_t *)(src+n);
	mmx_t ma;

	if (n%4) {
		fprintf(stderr,"MMX_YUV_S16_sum() size=%d unsupported\n",n);
		return -1;
	}

	ma.q=0xFF00FF00FF00FF00LL;
	movq_m2r(ma,mm7);
	ma.w[0] = ma.w[1] = ma.w[2] = ma.w[3] = 1;
	movq_m2r(ma,mm6);
	pxor_r2r(mm5,mm5);

	while (psrc<end) {
		movq_m2r(*(psrc++), mm0);
		pand_r2r(mm7,mm0);
		psrlq_i2r(8,mm0);
		movq_r2m(mm0,*(pdst++));
		
		pmaddwd_r2r(mm6,mm0);
		paddd_r2r(mm0,mm5);
	}
	movq_r2m(mm5, ma);
	emms();
	
	return ma.d[0]+ma.d[1];
}
*/		
	
/* Compute distance to a YUV color */
MMXMatrix *MMX_YUV_Dist(MMXMatrix *src, 
			unsigned char y, unsigned char u, unsigned char v)
{
	mmx_t *psrc, *pdst;
	MMXMatrix *dst;
	mmx_t ma[2];
	int i,j;

	dst = MMXMatrixAlloc(MMT_S_16, src->cols, src->rows);
	if (dst==NULL) return dst;
	
	pxor_r2r(mm0,mm0);
	ma[0].uw[0] = u;
	ma[0].uw[1] = y;
	ma[0].uw[2] = v;
	ma[0].uw[3] = y;
	movq_m2r(*ma,mm1);
	
	ma[0].q = 0x0000ffff0000ffffULL;
	movq_m2r(*ma,mm7);

	for (j=0;j<src->rows;j++) {
		psrc = src->p[j].m;
		pdst = dst->p[j].m;
		for (i=0;i<src->n;i++) {
#if 0
			movq_m2r(*psrc,mm2);
			movq_r2r(mm2,mm3);
			punpcklbw_r2r(mm0,mm2);
			psubw_r2r(mm1,mm2);

			/* test remove Y component */
			pand_r2r(mm7,mm2);

			movq_r2r(mm2,mm4);
			pmaddwd_r2r(mm2,mm2);
			pand_r2r(mm7,mm4);
			pmaddwd_r2r(mm4,mm4);
			movq_r2r(mm4,mm5);
			psllq_i2r(32,mm4);
			paddd_r2r(mm4,mm2);
			psrlq_i2r(32,mm5);
			paddd_r2r(mm5,mm2);

			punpcklbw_r2r(mm0,mm3);
			psubw_r2r(mm1,mm3);

			/* test remove Y component */
			pand_r2r(mm7,mm3);

			movq_r2r(mm3,mm4);
			pmaddwd_r2r(mm3,mm3);
			pand_r2r(mm7,mm4);
			pmaddwd_r2r(mm4,mm4);
			movq_r2r(mm4,mm5);
			psllq_i2r(32,mm4);
			paddd_r2r(mm4,mm3);
			psrlq_i2r(32,mm5);
			paddd_r2r(mm5,mm3);
		
			psrld_i2r(12,mm2); /* convert to 16 bits */
			psrld_i2r(12,mm3);

			packssdw_r2r(mm3,mm2);
			//psllw_i2r(8,mm2); /* test */
			movq_r2m(mm2,*pdst);
#else
			movq_m2r(*psrc,mm2);
			movq_r2r(mm2,mm3);
			punpcklbw_r2r(mm0,mm2);
			psubw_r2r(mm1,mm2);
			movq_r2r(mm2,mm4);
			pmaddwd_r2r(mm2,mm2);
			pand_r2r(mm7,mm4);
			pmaddwd_r2r(mm4,mm4);
			movq_r2r(mm4,mm5);
			psllq_i2r(32,mm4);
			paddd_r2r(mm4,mm2);
			psrlq_i2r(32,mm5);
			paddd_r2r(mm5,mm2);

			punpcklbw_r2r(mm0,mm3);
			psubw_r2r(mm1,mm3);
			movq_r2r(mm3,mm4);
			pmaddwd_r2r(mm3,mm3);
			pand_r2r(mm7,mm4);
			pmaddwd_r2r(mm4,mm4);
			movq_r2r(mm4,mm5);
			psllq_i2r(32,mm4);
			paddd_r2r(mm4,mm3);
			psrlq_i2r(32,mm5);
			paddd_r2r(mm5,mm3);
		
			psrld_i2r(8,mm2); /* convert to 8 bits */
			psrld_i2r(8,mm3);

			packssdw_r2r(mm3,mm2);
			//psllw_i2r(8,mm2); /* test */
			movq_r2m(mm2,*pdst);
#endif
			psrc++;
			pdst++;
		}
	}
	emms();
	return dst;
}

/* Compute distance to a YUV color */
int MMX_YUV_Dist2(unsigned char *src, int width, int height,
		 unsigned char *dst,
		 unsigned char y, unsigned char u, unsigned char v)
{
	unsigned char *psrc = src;
	unsigned char *end = src+2*width*height;
	unsigned char *endRow;
	unsigned char *pdst = dst;
	//MMXMatrix *dst;
	mmx_t ma[2];

	//if (width%2) return NULL;
	if (width%2) return -1;

	//dst = MMXMatrixAlloc(MMT_U_16, width, height);
	//if (dst==NULL) return dst;
	
	pxor_r2r(mm0,mm0);
	ma[0].uw[0] = u;
	ma[0].uw[1] = y;
	ma[0].uw[2] = v;
	ma[0].uw[3] = y;
	movq_m2r(*ma,mm1);

	ma[0].q = 0x0000ffff0000ffffULL;
	movq_m2r(*ma,mm7);

	while (psrc<end) {
		//pdst = dst->p[j++].m;
		endRow = psrc+2*width;
		while (psrc<endRow) {
			movq_m2r(*psrc,mm2);
			movq_r2r(mm2,mm3);
			punpcklbw_r2r(mm0,mm2);
			psubw_r2r(mm1,mm2);
			movq_r2r(mm2,mm4);
			pmaddwd_r2r(mm2,mm2);
			pand_r2r(mm7,mm4);
			pmaddwd_r2r(mm4,mm4);
			movq_r2r(mm4,mm5);
			psllq_i2r(32,mm4);
			paddd_r2r(mm4,mm2);
			psrlq_i2r(32,mm5);
			paddd_r2r(mm5,mm2);

			punpcklbw_r2r(mm0,mm3);
			psubw_r2r(mm1,mm3);
			movq_r2r(mm3,mm4);
			pmaddwd_r2r(mm3,mm3);
			pand_r2r(mm7,mm4);
			pmaddwd_r2r(mm4,mm4);
			movq_r2r(mm4,mm5);
			psllq_i2r(32,mm4);
			paddd_r2r(mm4,mm3);
			psrlq_i2r(32,mm5);
			paddd_r2r(mm5,mm3);
		
			psrld_i2r(8,mm2); /* convert to 8 bits */
			psrld_i2r(8,mm3);

			packssdw_r2r(mm3,mm2);
			psllw_i2r(8,mm2); /* test */
			movq_r2m(mm2,*pdst);

			psrc += 8;
			//pdst++;
			pdst += 8;
		}

	}
	emms();
	//return dst;
	return 0;
}



/* CAREFULL: no call to emms() at the end */
int MMX_YUV_Shrink_H2(unsigned char *src, unsigned char *dst, int n)
{
	mmx_t *psrc=(mmx_t *)src;
	mmx_t *pdst=(mmx_t *)dst;
	mmx_t *end=(mmx_t *)(src+n);
	mmx_t ma[2];

	ma[0].q = 0x00000000ffffffffLL;
	movq_m2r(*ma,mm2);
	while (psrc<end) {
		movq_m2r(*(psrc++),mm0);
		movq_m2r(*(psrc++),mm1);
		pand_r2r(mm2,mm0);
		psllq_i2r(32,mm1);
		por_r2r(mm1,mm0);
		movq_r2m(mm0,*(pdst++));
	}
	return 0;
}

/* CAREFULL: no call to emms() at the end */
int MMX_YUV_Shrink_H4(unsigned char *src, unsigned char *dst, int n)
{
	mmx_t *psrc=(mmx_t *)src;
	mmx_t *pdst=(mmx_t *)dst;
	mmx_t *end=(mmx_t *)(src+n);
	mmx_t ma[2];

	ma[0].q = 0x00000000ffffffffLL;
	movq_m2r(*ma,mm2);
	while (psrc<end) {
		movq_m2r(*psrc,mm0);
		psrc += 2;
		movq_m2r(*psrc,mm1);
		psrc += 2;
		pand_r2r(mm2,mm0);
		psllq_i2r(32,mm1);
		por_r2r(mm1,mm0);
		movq_r2m(mm0,*(pdst++));
	}
	return 0;
}

/* Compute distance to a YUV color */
float MMX_YUV_Grav(MMXMatrix *src,
		   unsigned char y, unsigned char u, unsigned char v,
		   float *px, float *py)
{
	int i,j;
	mmx_t *psrc;
	mmx_t ma[2];
	sse_t mx;
	float sumw, sumwx, sumwy;

	xorps_r2r(xmm7,xmm7); /* sum w */
	xorps_r2r(xmm6,xmm6); /* sum w*x */
	xorps_r2r(xmm5,xmm5); /* sum w*y */

	mx.sf[0]=mx.sf[1]=mx.sf[2]=mx.sf[3]=1;
	movups_m2r(mx,xmm2);
	mx.sf[0]=0;mx.sf[1]=1;mx.sf[2]=2;mx.sf[3]=3;
	
	xorps_r2r(xmm4,xmm4); /* y */

	pxor_r2r(mm0,mm0);
	ma[0].uw[0] = u;
	ma[0].uw[1] = y;
	ma[0].uw[2] = v;
	ma[0].uw[3] = y;
	movq_m2r(*ma,mm1);
	
	ma[0].q = 0x0000ffff0000ffffULL;
	movq_m2r(*ma,mm7);

	
	for (j=0;j<src->rows;j++) {
		psrc = src->p[j].m;
		movups_m2r(mx, xmm3); /* x */
		for (i=0;i<src->n;i++) {
			movq_m2r(*psrc,mm2);
			movq_r2r(mm2,mm3);
			punpcklbw_r2r(mm0,mm2);
			psubw_r2r(mm1,mm2);

			/* test remove Y component */
			pand_r2r(mm7,mm2);

			movq_r2r(mm2,mm4);
			pmaddwd_r2r(mm2,mm2);
			pand_r2r(mm7,mm4);
			pmaddwd_r2r(mm4,mm4);
			movq_r2r(mm4,mm5);
			psllq_i2r(32,mm4);
			paddd_r2r(mm4,mm2);
			psrlq_i2r(32,mm5);
			paddd_r2r(mm5,mm2);

			punpcklbw_r2r(mm0,mm3);
			psubw_r2r(mm1,mm3);

			/* test remove Y component */
			pand_r2r(mm7,mm3);

			movq_r2r(mm3,mm4);
			pmaddwd_r2r(mm3,mm3);
			pand_r2r(mm7,mm4);
			pmaddwd_r2r(mm4,mm4);
			movq_r2r(mm4,mm5);
			psllq_i2r(32,mm4);
			paddd_r2r(mm4,mm3);
			psrlq_i2r(32,mm5);
			paddd_r2r(mm5,mm3);

			psrld_i2r(4,mm2);
			psrld_i2r(4,mm3);

			//PRINT_REGM("mm2",mm2);
			//PRINT_REGM("mm3",mm3);
			cvtpi2ps_r2r(mm2, xmm0);
			//PRINT_REGF("xmm0",xmm0);
			cvtpi2ps_r2r(mm3, xmm1);
			movlhps_r2r(xmm1, xmm0);
			addps_r2r(xmm2,xmm0);
#if 0
			rsqrtps_r2r(xmm0,xmm0);
#else
			rcpps_r2r(xmm0,xmm0);
#endif
			
			addps_r2r(xmm0,xmm7);
			movaps_r2r(xmm0,xmm1);
			mulps_r2r(xmm3,xmm0);
			addps_r2r(xmm0,xmm6);
			mulps_r2r(xmm4,xmm1);
			addps_r2r(xmm1,xmm5);

			/* FIXME: add 4, not 4x1*/
			addps_r2r(xmm2,xmm3);
			addps_r2r(xmm2,xmm3);
			addps_r2r(xmm2,xmm3);
			addps_r2r(xmm2,xmm3);

			psrc++;
		}
		addps_r2r(xmm2,xmm4);
	}
	emms();
	movups_r2m(xmm7,mx);
	sumw = mx.sf[0]+mx.sf[1]+mx.sf[2]+mx.sf[3];
	movups_r2m(xmm6,mx);
	sumwx = mx.sf[0]+mx.sf[1]+mx.sf[2]+mx.sf[3];
	movups_r2m(xmm5,mx);
	sumwy = mx.sf[0]+mx.sf[1]+mx.sf[2]+mx.sf[3];

	if (sumw>0.00001) {
		*px = sumwx/sumw;
		*py = sumwy/sumw;
	}
	else {
		*px = src->cols/2;
		*py = src->rows/2;
	}
	return sumwx;
}


#if 0
MMXMatrix *MMX_YUV_RGB_Dist(MMXMatrix *src, 
			    unsigned char y, unsigned char u, unsigned char v)
{
	mmx_t *psrc, *pdst;
	MMXMatrix *dst;
	mmx_t ma[2];
	int i,j;

	dst = MMXMatrixAlloc(MMT_S_16, src->cols, src->rows);
	if (dst==NULL) return dst;
	
	pxor_r2r(mm0,mm0);
	ma[0].uw[0] = u;
	ma[0].uw[1] = y;
	ma[0].uw[2] = v;
	ma[0].uw[3] = y;
	movq_m2r(*ma,mm1);
	
	ma[0].q = 0x0000ffff0000ffffULL;
	movq_m2r(*ma,mm7);
	ma[0].q = 0xffff0000ffff0000ULL;
	movq_m2r(*ma,mm6);

	for (j=0;j<src->rows;j++) {
		psrc = src->p[j].m;
		pdst = dst->p[j].m;
		for (i=0;i<src->n;i++) {
			movq_m2r(*psrc,mm2);
			punpcklbw_r2r(mm0,mm2);
			psubw_r2r(mm1,mm2);
			movq_r2r(mm2,mm3);

			pand_r2r(mm6,mm2);
			psrlq_i2r(16,mm2); /* (Y1-Y)|(Y0-Y) */

			pand_r2r(mm7,mm3);
			movq_r2r(mm3,mm4);
			movq_r2r(mm3,mm5);
			
			psllq_i2r(32,mm4);
			packssdw_r2r(mm4,mm3); /* (U0-U)|(U0-V) */

			movq_r2r(mm5,mm4);
			psrlq_i2r(32,mm4);
			packssdw_r2r(mm5,mm4); /* (V0-V)|(V0-V) */



			movq_r2r(mm4,mm5);
			psllq_i2r(32,mm4);
			paddd_r2r(mm4,mm2);
			psrlq_i2r(32,mm5);
			paddd_r2r(mm5,mm2);

			punpcklbw_r2r(mm0,mm3);
			psubw_r2r(mm1,mm3);

			/* test remove Y component */
			pand_r2r(mm7,mm3);

			movq_r2r(mm3,mm4);
			pmaddwd_r2r(mm3,mm3);
			pand_r2r(mm7,mm4);
			pmaddwd_r2r(mm4,mm4);
			movq_r2r(mm4,mm5);
			psllq_i2r(32,mm4);
			paddd_r2r(mm4,mm3);
			psrlq_i2r(32,mm5);
			paddd_r2r(mm5,mm3);
		
			psrld_i2r(4,mm2); /* convert to 16 bits */
			psrld_i2r(4,mm3);

			packssdw_r2r(mm3,mm2);
			//psllw_i2r(8,mm2); /* test */
			movq_r2m(mm2,*pdst);

			psrc++;
			pdst++;
		}
	}
	emms();
	return dst;
}
#endif
