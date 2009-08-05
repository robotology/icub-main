#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <assert.h>

#include "MMXMatrix.h"




void MMXMatrixPrint(char *msg, MMXMatrix *m) 
{
  int i,j;

  printf("%s type 0x%X [%dx%d]:\n",msg,m->type,m->cols,m->rows);

  for (j=0;j<m->rows;j++) {
    //printf("%p: ",m->p[j].s);
    for (i=0;i<m->cols;i++) {
      switch (m->type) {
      case MMT_F_32:
#if 0
	printf("%e ",((float *)m->p[j].s)[i]);
#else
	printf("%+#07.3f ",((float *)m->p[j].s)[i]);
#endif
	break;
      case MMT_U_64:
	printf("%llu ",((MM_U_64 *)m->p[j].m)[i]);
	break;
      case MMT_S_64:
	printf("%lld ",((MM_S_64 *)m->p[j].m)[i]);
	break;
      case MMT_U_32:
	printf("%d ",((MM_U_32 *)m->p[j].m)[i]);
	break;
      case MMT_S_32:
	printf("%d ",((MM_S_32 *)m->p[j].m)[i]);
	break;
      case MMT_U_16:
	printf("%03d ",((MM_U_16 *)m->p[j].m)[i]);
	break;
      case MMT_S_16:
	printf("%+04d ",((MM_S_16 *)m->p[j].m)[i]);
	break;
      case MMT_U_8:
	printf("%03d ",((MM_U_8 *)m->p[j].m)[i]);
	break;
      case MMT_S_8:
	printf("%03d ",((MM_S_8 *)m->p[j].m)[i]);
	break;
      }
    }
    printf("\n");
  }
  printf("\n");
}

void MMXMatrixFree(MMXMatrix * M)
{
	free(M);
}

MMXMatrix *MMXMatrixAlloc(mmtype type, int cols, int rows)
{
  int i, width;
  MMXMatrix *m;
  sse_t *ps;
  mmx_t *pm;

  if (rows<=0 || cols<=0) return NULL;

  //printf("alloc [%dx%d] MMXMatrix\n",cols,rows);

  /* bytes per row, padded for cache line alignment */
  width = ((type & MMT_MASK) * cols + CACHE_LINE_WIDTH - 1) &
    -CACHE_LINE_WIDTH;

  if ((m = (MMXMatrix *)malloc(sizeof(MMXMatrix) +
			       rows * (sizeof(void *) + width) + 
			       CACHE_ALIGN_PAD)) == NULL) {
    fprintf(stderr,"MMXMatrixAlloc: out of memory\n");
    return NULL;
  }
  memset(m,0,sizeof(MMXMatrix) +rows * (sizeof(void *) + width) + 
	 CACHE_ALIGN_PAD);

  m->type = type;
  m->size = type & MMT_MASK;
  m->rows = rows;
  m->cols = cols;
  m->pad_rows = rows;
  m->pad_cols = width / m->size;
  m->p = (mmptr *)((char *)m + sizeof(MMXMatrix));

  switch (type) {
  case MMT_U_64:
  case MMT_S_64:
    m->n = cols;
    break;
  case MMT_U_32:
  case MMT_S_32:
    m->n = (cols+1)/2;
    break;
  case MMT_F_32:
  case MMT_U_16:
  case MMT_S_16:
    m->n = (cols+3)/4;
    break;
  case MMT_U_8:
  case MMT_S_8:
    m->n = (cols+7)/8;
    break;
  }

  if (type == MMT_F_32) {
    //m->d.s = CACHE_ALIGN((void *)(m->p) + rows * sizeof(sse_t *));
    m->d.s = (sse_t *) CACHE_ALIGN((char *)(m->p) + rows * sizeof(sse_t *));
    m->off = width / sizeof(sse_t);
    for (i = 0, ps = m->d.s; i < m->pad_rows; i++, ps += m->off) {
      m->p[i].s = ps;
      //printf("p[%d].s: %p\n",i,m->p[i].s);
    }
  }
  else {
    //m->d.m = CACHE_ALIGN((void *)(m->p) + rows * sizeof(mmx_t *));
    m->d.m = (mmx_t *) CACHE_ALIGN((char *)(m->p) + rows * sizeof(mmx_t *));
    m->off = width / sizeof(mmx_t);
    for (i = 0, pm = m->d.m; i < m->pad_rows; i++, pm += m->off) {
      m->p[i].m = pm;
	  //printf("p[%d].m = %p\n",i,m->p[i].m);
    }
  }

#if 0
  printf("width: %d off: %d n: %d type %d size %d pad_rows %d pad_cols %d\n",
	 width, m->off, m->n, m->type,m->size,m->pad_rows,m->pad_cols);
#endif
  return m;
}

void MMXMatrixAssert(const MMXMatrix * m)
{
  int i, width;
  sse_t *ps;
  mmx_t *pm;

  assert(m->rows > 0);
  assert(m->cols > 0);

  /* bytes per row, padded for cache line alignment */
  width = ((m->type & MMT_MASK) * m->cols + CACHE_LINE_WIDTH - 1) &
    -CACHE_LINE_WIDTH;

  assert(m->size == (m->type & MMT_MASK));
  assert(m->pad_rows == m->rows);
  assert(m->pad_cols == width / m->size);
  assert(m->p == (mmptr*)((char *)m + sizeof(MMXMatrix)));

  switch (m->type) {
  case MMT_U_64:
  case MMT_S_64:
    assert(m->n == m->cols);
    break;
  case MMT_U_32:
  case MMT_S_32:
    assert(m->n == (m->cols+1)/2);
    break;
  case MMT_F_32:
  case MMT_U_16:
  case MMT_S_16:
    assert(m->n == (m->cols+3)/4);
    break;
  case MMT_U_8:
  case MMT_S_8:
    assert(m->n == (m->cols+7)/8);
    break;
  }

  if (m->type == MMT_F_32) {
    //assert(m->d.s == CACHE_ALIGN((void *)(m->p) + m->rows * sizeof(sse_t *)));
    assert(m->d.s == (sse_t *) CACHE_ALIGN((char *)(m->p) + m->rows * sizeof(sse_t *)));
    assert(m->off == width / sizeof(sse_t));
    for (i = 0, ps = m->d.s; i < m->pad_rows; i++, ps += m->off) {
      assert(m->p[i].s == ps);
    }
  }
  else {
    //assert(m->d.m == CACHE_ALIGN((void *)(m->p) + m->rows * sizeof(mmx_t *)));
    assert(m->d.m == (mmx_t *)CACHE_ALIGN((char *)(m->p) + m->rows * sizeof(mmx_t *)));
    assert(m->off == width / sizeof(mmx_t));
    for (i = 0, pm = m->d.m; i < m->pad_rows; i++, pm += m->off) {
      assert(m->p[i].m == pm);
    }
  }

}

void MMXMatrixZero(MMXMatrix * m)
{
	int j;
	unsigned int colsize=0;
	switch (m->type) {
		case MMT_F_32:
			colsize = m->cols*sizeof(MM_F_32);
			break;
		case MMT_U_64:
			colsize = m->cols*sizeof(MM_U_64);
			break;
		case MMT_S_64:
			colsize = m->cols*sizeof(MM_S_64);
			break;
		case MMT_U_32:
			colsize = m->cols*sizeof(MM_U_32);
			break;
		case MMT_S_32:
			colsize = m->cols*sizeof(MM_S_32);
			break;
		case MMT_U_16:
			colsize = m->cols*sizeof(MM_U_16);
			break;
		case MMT_S_16:
			colsize = m->cols*sizeof(MM_S_16);
			break;
		case MMT_U_8:
			colsize = m->cols*sizeof(MM_U_8);
			break;
		case MMT_S_8:
			colsize = m->cols*sizeof(MM_S_8);
			break;
	}


	for (j=0;j<m->rows;j++)
		bzero((char *)m->p[j].m,colsize);

}

int conv(MMXMatrix *src, MMXMatrix *ker, MMXMatrix *dst)
{
  int i,j,k,l;

  switch(src->type) {
  case MMT_F_32:
    for (j=0;j<dst->rows;j++) {
      for (i=0;i<dst->cols;i++) {
	((MM_F_32 *)dst->p[j].s)[i] = 0;
	for (l=0;l<ker->rows;l++) {
	  for (k=0;k<ker->cols;k++) {
	    ((MM_F_32 *)dst->p[j].s)[i] += ((MM_F_32 *)ker->p[l].s)[k] *
	      ((MM_F_32 *)src->p[j+l].s)[i+k];
	  }
	}
      }
    }
    break;
  case MMT_S_16:
    if (dst->type != MMT_S_32) {
      fprintf(stderr,"conv(): type %d not supported for dst\n",dst->type);
      return -1;
    }
    for (j=0;j<dst->rows;j++) {
      for (i=0;i<dst->cols;i++) {
	((MM_S_32 *)dst->p[j].m)[i] = 0;
	for (l=0;l<ker->rows;l++) {
	  for (k=0;k<ker->cols;k++) {
	    ((MM_S_32 *)dst->p[j].m)[i] += ((MM_S_16 *)ker->p[l].m)[k] *
	      ((MM_S_16 *)src->p[j+l].m)[i+k];
	  }
	}
      }
    }
    break;
  default:
    fprintf(stderr,"conv(): type %d not supported\n",src->type);
    return -1;
    break;
  }
    
  return 0;
}

MMXMatrix *MMXMatrixRandom(mmtype type, int w, int h) 
{
  int i,j;
  MMXMatrix *m=NULL;

  if (type!=MMT_F_32 && type!=MMT_S_16 && type!=MMT_U_8) {
    fprintf(stderr,"convKernel(): type %d not supported yet\n",type);
    return NULL;
  }

  if ((m=MMXMatrixAlloc(type,w,h))==NULL) return NULL;

  for (j=0;j<m->rows;j++) {
    for (i=0;i<m->cols;i++) {
      switch (type) {
      case MMT_F_32:
	((MM_F_32 *)m->p[j].s)[i] = random()/(float)RAND_MAX;
	break;
      case MMT_S_16:
	((MM_S_16 *)m->p[j].m)[i] = (MM_S_16)(128.9*random()/(float)RAND_MAX);
	break;
      case MMT_U_8:
	((MM_U_8 *)m->p[j].m)[i] = (MM_U_8)(256.0*random()/(float)RAND_MAX);
	break;
      default:
	break;
      }
    }
  }
  return m;
}

float MMXMatrixDiff(MMXMatrix *m0, MMXMatrix *m1) 
{
  int i,j;
  float sum=0;
  MM_F_32 tmp=0;

  if (m0->type != m1->type || m0->cols != m1->cols || m0->rows != m1->rows) {
    fprintf(stderr,"MMXMatrixDiff(): incompatible matrices\n");
    return -1;
  }

  if (m0->type!=MMT_F_32 && m0->type!=MMT_S_32) {
    fprintf(stderr,"MMXMatrixDiff(): type %d not supported yet\n",m0->type);
    return -1;
  }

  for (j=0;j<m0->rows;j++) {
    for (i=0;i<m0->cols;i++) {
      switch (m0->type) {
      case MMT_F_32:
	tmp = ((MM_F_32 *)m0->p[j].s)[i] - ((MM_F_32 *)m1->p[j].s)[i];
	break;
      case MMT_S_32:
	tmp = ((MM_S_32 *)m0->p[j].m)[i] - ((MM_S_32 *)m1->p[j].m)[i];
	break;
      default:
	break;
      }
      sum += tmp*tmp;
    }
  }
  return sum;
}

void MMXMatrixFPrint(char *filename, MMXMatrix *m) 
{
  int i,j;
  FILE *fp;

  fp = fopen(filename,"w");

  //printf("%s type 0x%X [%dx%d]:\n",msg,m->type,m->cols,m->rows);

  for (j=0;j<m->rows;j++) {
    //printf("%p: ",m->p[j].s);
    for (i=0;i<m->cols;i++) {
      switch (m->type) {
      case MMT_F_32:
	fprintf(fp,"%+#07.3f ",((float *)m->p[j].s)[i]);
	break;
      case MMT_U_64:
	fprintf(fp,"%llu ",((MM_U_64 *)m->p[j].m)[i]);
	break;
      case MMT_S_64:
	fprintf(fp,"%lld ",((MM_S_64 *)m->p[j].m)[i]);
	break;
      case MMT_U_32:
	fprintf(fp,"%d ",((MM_U_32 *)m->p[j].m)[i]);
	break;
      case MMT_S_32:
	fprintf(fp,"%d ",((MM_S_32 *)m->p[j].m)[i]);
	break;
      case MMT_U_16:
	fprintf(fp,"%03d ",((MM_U_16 *)m->p[j].m)[i]);
	break;
      case MMT_S_16:
	fprintf(fp,"%+04d ",((MM_S_16 *)m->p[j].m)[i]);
	break;
      case MMT_U_8:
	fprintf(fp,"%03d ",((MM_U_8 *)m->p[j].m)[i]);
	break;
      case MMT_S_8:
	fprintf(fp,"%03d ",((MM_S_8 *)m->p[j].m)[i]);
	break;
      }
    }
    fprintf(fp,"\n");
  }
  fclose(fp);
}


MMXMatrix *MMXMatrixCopy(MMXMatrix *m) {
  MMXMatrix *n;

  n = MMXMatrixAlloc(m->type, m->cols, m->rows);
  memcpy(n->d.m,m->d.m,(m->pad_cols*m->rows*m->size));
  return n;
}


