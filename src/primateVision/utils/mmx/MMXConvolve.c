
#include <stdio.h>
#include <stdlib.h>
#include "MMXConvolve.h"

MMXMatrix *convKernel(MMXMatrix *m) 
{
	int n,cols,i,j,k,l;
	MMXMatrix *d=NULL,*tmp=NULL;

	if (m->type!=MMT_F_32 && m->type!=MMT_S_16) {
		fprintf(stderr,"convKernel(): type %d not supported yet\n",m->type);
		return NULL;
	}

	tmp=MMXMatrixAlloc(m->type,m->cols+6,m->rows);

	for (j=0;j<tmp->rows;j++) {
		for (i=0;i<tmp->cols;i++) {
			switch (m->type) {
			case MMT_F_32:
				if (i<3 || i>=tmp->cols-3) ((MM_F_32 *)tmp->p[j].s)[i] = 0;
				else ((MM_F_32 *)tmp->p[j].s)[i] = ((MM_F_32 *)m->p[j].s)[i-3];
				break;
			case MMT_S_16:
				if (i<3 || i>=tmp->cols-3) ((MM_S_16 *)tmp->p[j].m)[i] = 0;
				else ((MM_S_16 *)tmp->p[j].m)[i] = ((MM_S_16 *)m->p[j].m)[i-3];
				break;
			default:
				break;
			}
		}
	}

	n = (m->cols+6)/4;
	cols = n*16;

	d = MMXMatrixAlloc(m->type, cols, m->rows);
  
	for (j=0;j<d->rows;j++) {
		for (i=0;i<n;i++) {
			for (k=0;k<4;k++) {
				switch (m->type) {
#if 0
				case MMT_F_32:
					movups_m2r(((MM_F_32 *)tmp->p[j].s)[(i+1)*4-k-1],xmm0);
					movaps_r2m(xmm0,((MM_F_32 *)d->p[j].s)[i*16+4*k]);
					break;
				case MMT_S_16:
					movq_m2r(((MM_S_16 *)tmp->p[j].m)[(i+1)*4-k-1],mm0);
					movq_r2m(mm0,((MM_S_16 *)d->p[j].m)[i*16+4*k]);
					break;
#else
				case MMT_F_32:
					for (l=0;l<4;l++)
						((MM_F_32 *)d->p[j].s)[i*16+4*k+l] = 
							((MM_F_32 *)tmp->p[j].s)[(i+1)*4-k-1+l];
					break;
				case MMT_S_16:
					for (l=0;l<4;l++)
						((MM_S_16 *)d->p[j].m)[i*16+4*k+l] = 
							((MM_S_16 *)tmp->p[j].m)[(i+1)*4-k-1+l];
					break;
#endif					
				default:
					break;
				}
			}
		}
	}

	if (m->type == MMT_S_16) emms();
	free(tmp);
	return d;
}

int conv_MM_F_32(MMXMatrix *src, MMXMatrix *ker, MMXMatrix *dst)
{
	int i,j,k,l;
	int n = ker->cols/16;
	sse_t *psrc,*pker,*pdst;

	if (dst->type != src->type) {
		fprintf(stderr,"conv_sse(): incompatible type between src and dst\n");
		return -1;
	}
	for (j=0;j<dst->rows;j++) {
		pdst = dst->p[j].s;
		for (i=0;i<(dst->cols+3)/4;i++) {
			xorps_r2r(xmm4,xmm4);
			xorps_r2r(xmm5,xmm5);
			xorps_r2r(xmm6,xmm6);
			xorps_r2r(xmm7,xmm7);
			for (l=0;l<ker->rows;l++) {
				psrc = src->p[j+l].s+i;
				pker = ker->p[l].s;
				for (k=0;k<n;k++) {
					movaps_m2r(*pker,xmm0);
					movaps_m2r(*(pker+1),xmm1);

					mulps_m2r(*psrc,xmm0);
					mulps_m2r(*psrc,xmm1);

					addps_r2r(xmm0,xmm4);
					addps_r2r(xmm1,xmm5);

					movaps_m2r(*(pker+2),xmm2);
					movaps_m2r(*(pker+3),xmm3);
					
					mulps_m2r(*psrc,xmm2);
					mulps_m2r(*psrc,xmm3);

					addps_r2r(xmm2,xmm6);
					addps_r2r(xmm3,xmm7);

					psrc++;
					pker += 4;
				}
			}
			movaps_r2r(xmm4,xmm0);
			unpcklps_r2r(xmm6,xmm0);
			unpckhps_r2r(xmm6,xmm4);

			movaps_r2r(xmm5,xmm1);
			unpcklps_r2r(xmm7,xmm1);
			unpckhps_r2r(xmm7,xmm5);

			movaps_r2r(xmm0,xmm2);
			unpcklps_r2r(xmm1,xmm0);
			unpckhps_r2r(xmm1,xmm2);

			addps_r2r(xmm2,xmm0);
	
			movaps_r2r(xmm4,xmm3);
			unpcklps_r2r(xmm5,xmm3);
			addps_r2r(xmm3,xmm0);
			unpckhps_r2r(xmm5,xmm4);
			addps_r2r(xmm4,xmm0);

			//movaps_r2m(xmm0,*(pdst++));
			movntps_r2m(xmm0,*(pdst++));
		}
	}
	return 0;
}

int conv_MM_S_16(MMXMatrix *src, MMXMatrix *ker, MMXMatrix *dst)
{
	int i,j,k,l;
	int n = ker->cols/16;
	mmx_t *psrc,*pker,*pdst;

	if (dst->type != MMT_S_32) {
		fprintf(stderr,"conv_MM_S_16(): incompatible type between src and dst\n");
		return -1;
	}
	for (j=0;j<dst->rows;j++) {
		pdst = dst->p[j].m;
		for (i=0;i<(dst->cols+3)/4;i++) {
			//prefetcht1(*(src->p[j+ker->rows].m+i));
			pxor_r2r(mm4,mm4);
			pxor_r2r(mm5,mm5);
			pxor_r2r(mm6,mm6);
			pxor_r2r(mm7,mm7);
			for (l=0;l<ker->rows;l++) {
				pker = ker->p[l].m;
				psrc = src->p[j+l].m+i;
				for (k=0;k<n;k++) {
					movq_m2r(*(psrc++),mm1);

					movq_m2r(*(pker++),mm0);
					pmaddwd_r2r(mm1,mm0);
					paddd_r2r(mm0,mm4);

					movq_m2r(*(pker++),mm0);
					pmaddwd_r2r(mm1,mm0);
					paddd_r2r(mm0,mm5);

					movq_m2r(*(pker++),mm0);
					pmaddwd_r2r(mm1,mm0);
					paddd_r2r(mm0,mm6);

					movq_m2r(*(pker++),mm0);
					pmaddwd_r2r(mm1,mm0);
					paddd_r2r(mm0,mm7);
				}
			}
			movq_r2r(mm4,mm0);
			punpckldq_r2r(mm5,mm0);
			punpckhdq_r2r(mm5,mm4);
			paddd_r2r(mm4,mm0);
			movq_r2m(mm0,*(pdst++));

			movq_r2r(mm6,mm1);
			punpckldq_r2r(mm7,mm1);
			punpckhdq_r2r(mm7,mm6);
			paddd_r2r(mm6,mm1);
			movq_r2m(mm1,*(pdst++));
		}
	}
	emms();

	return 0;
}

int MMXConvolve(MMXMatrix *src, MMXMatrix *ker, MMXMatrix *dst)
{
	if (src->type != ker->type) {
		fprintf(stderr,"MMXConvolve(): incompatible type between src and ker\n");
		return -1;
	}

	switch(src->type) {
	case MMT_F_32:
		return conv_MM_F_32(src,ker,dst);
		break;
	case MMT_S_16:
		return conv_MM_S_16(src,ker,dst);
		break;
	default:
		fprintf(stderr,"conv_sse(): type %d not supported\n",src->type);
		break;
	}
	return -1;
}
