
#ifndef _MMX_MATRIX_H
#define _MMX_MATRIX_H

#include <math.h>

#include "sse.h"
#include "mmx.h"

#ifndef DEG2RAD
#define DEG2RAD(f) ((f)*M_PI/180.0)
#endif
#ifndef RAD2DEG
#define RAD2DEG(f) ((f)*180.0/M_PI)
#endif

/* The various sized operands used by MMX instructions, signed and unsigned */

#define MM_F_32 float
#define MM_S_64	long long signed int
#define MM_U_64	long long unsigned int
#define MM_S_32	signed int
#define MM_U_32	unsigned int
#define MM_S_16	short signed int
#define MM_U_16	short unsigned int
#define MM_S_8	signed char
#define MM_U_8	unsigned char

#define MMT_FLOAT	0xC0
#define MMT_SIGNED	0x80
#define MMT_UNSIGNED	0x00
#define MMT_MASK	0x3f

typedef enum
{
  MMT_F_32 = sizeof(MM_F_32) | MMT_FLOAT,
  MMT_S_64 = sizeof(MM_S_64) | MMT_SIGNED,
  MMT_U_64 = sizeof(MM_U_64) | MMT_UNSIGNED,
  MMT_S_32 = sizeof(MM_S_32) | MMT_SIGNED,
  MMT_U_32 = sizeof(MM_U_32) | MMT_UNSIGNED,
  MMT_S_16 = sizeof(MM_S_16) | MMT_SIGNED,
  MMT_U_16 = sizeof(MM_U_16) | MMT_UNSIGNED,
  MMT_S_8 =  sizeof(MM_S_8) | MMT_SIGNED,
  MMT_U_8 =  sizeof(MM_U_8) | MMT_UNSIGNED
} mmtype;

typedef union {
  mmx_t *m;
  sse_t *s;
} mmptr;

/* The basic data types in an MMX register */

#define BITS_PER_BYTE 8

#define CACHE_LINE_WIDTH 32

#define CACHE_ALIGN_PAD (CACHE_LINE_WIDTH - sizeof(long))

#define CACHE_ALIGN(mm) ((void *)((unsigned long)((mm) + CACHE_ALIGN_PAD) & \
				~(unsigned long)CACHE_ALIGN_PAD))

typedef struct
{
  mmtype type;                  /* what MMX data type is used */
  int size;                     /* bytes per value */
  int rows, cols;               /* matrix dimensions requested */
  int pad_rows, pad_cols;       /* padded dimensions allocated */
  int off;                      /* offset between rows */
  int n;                        /* number of blocks (mmx_t or sse_t) per row */
  mmptr d;                      /* pointer to the data */
  mmptr *p;                     /* pointer to vector of row pointers */
} MMXMatrix;



#define PRINT_REGF(msg,reg) \
{\
   sse_t dbg;\
   movups_r2m(reg,dbg);\
   printf("%s: %f %f %f %f\n",msg,dbg.sf[0],dbg.sf[1],dbg.sf[2],dbg.sf[3]);\
}

#define PRINT_REGM(msg,reg) \
{\
   mmx_t dbg;\
   movq_r2m(reg,dbg);\
   printf("%s: %d %d\n",msg,dbg.d[0],dbg.d[1]);\
}

#define PRINT_REGMWD(msg,reg) \
{\
   mmx_t dbg;\
   movq_r2m(reg,dbg);\
   printf("%s: %d %d %d %d\n",msg,dbg.w[0],dbg.w[1],dbg.w[2],dbg.w[3]);\
}

#define PRINT_REGMBW(msg,reg) \
{\
   mmx_t dbg;\
   movq_r2m(reg,dbg);\
   printf("%s: %d %d %d %d %d %d %d %d\n",msg,dbg.b[0],dbg.b[1],dbg.b[2],dbg.b[3],dbg.b[4],dbg.b[5],dbg.b[6],dbg.b[7]);\
}

#define PRINT_REGMUBW(msg,reg) \
{\
   mmx_t dbg;\
   movq_r2m(reg,dbg);\
   printf("%s: %d %d %d %d %d %d %d %d\n",msg,dbg.ub[0],dbg.ub[1],dbg.ub[2],dbg.ub[3],dbg.ub[4],dbg.ub[5],dbg.ub[6],dbg.ub[7]);\
}



MMXMatrix *MMXMatrixAlloc(mmtype type, int cols, int rows);
void MMXMatrixZero(MMXMatrix * M);
void MMXMatrixAssert(const MMXMatrix * M);
void MMXMatrixFree(MMXMatrix * M);

void MMXMatrixPrint(char *msg, MMXMatrix *m);
void MMXMatrixFPrint(char *filename, MMXMatrix *m);
MMXMatrix *MMXMatrixRandom(mmtype type, int w, int h);
float MMXMatrixDiff(MMXMatrix *m0, MMXMatrix *m1);

int conv(MMXMatrix *src, MMXMatrix *ker, MMXMatrix *dst);
MMXMatrix *MMXMatrixCopy(MMXMatrix *m);

#endif
