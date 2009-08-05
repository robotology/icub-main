

#ifndef DEPTHMAP_H
#define DEPTHMAP_H

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>
#include <math.h>
#include <time.h>  
#include <stdarg.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <getopt.h>
#include <sys/types.h>

#include "libmmx.h"

#ifdef __cplusplus
extern "C" {
#endif

// will become 0xfx scheme which will be unambiguous 
// but currently just gives 0x00
// used to mask disparties to detect failed estimates
#define DMAP_FAIL_MASK 0xfe
// pixel disparity not estimated due to consistency check fail
#define DMAP_FAIL_CCHECK 0xff
// pixel disparity not estimated due to flat correlation result (neighbours matched just as well)
#define DMAP_FAIL_FLAT 0xfe
#define DMAP_UNKNOWN 0x00


#define PRINTREG16(reg,lab) \
{\
   static mmx_t m;\
   movq_r2m(reg,m);\
   printf("%s: %u %u %u %u\n",lab,\
          m.uw[0],m.uw[1],m.uw[2],m.uw[3]);\
}
//   printf("%s: %d %d %d %d\n",lab,

#define PRINTREG16XMM(reg,lab) \
{\
   static sse_t m;\
   movdqu_r2m(reg,m);\
   printf("%s: %u %u %u %u %u %u %u %u\n",lab,\
          m.uw[0],m.uw[1],m.uw[2],m.uw[3],m.uw[4],m.uw[5],m.uw[6],m.uw[7]);\
}
//   printf("%s: %d %d %d %d\n",lab,

#define PRINTREG32(reg,lab) \
{\
   static mmx_t m;\
   movq_r2m(reg,m);\
   printf("%s: %ld %ld\n",lab,\
          m.ud[0],m.ud[1]);\
}

#define PRINTREG8(reg,lab) \
{\
   static mmx_t m;\
   movq_r2m(reg,m);\
   printf("%s: %d %d %d %d %d %d %d %d\n",lab,\
          m.ub[0],m.ub[1],m.ub[2],m.ub[3],m.ub[4],m.ub[5],m.ub[6],m.ub[7]);\
}


#define ERR ((errno < sys_nerr) ? sys_errlist[errno] : "")

#define ERROR Error("%s: %d: Internal Error", __FILE__, __LINE__)


/* How many MM_U_16 operands will we work on at once */
#define Ts (sizeof(mmx_t) / sizeof(MM_U_16))




typedef void* MMXMatricesCachePtr;

extern MMXMatricesCachePtr MMXMatricesCacheAlloc(unsigned int nmat,
		unsigned int nvect, unsigned int nvoid);

extern void MMXMatricesCacheFree(MMXMatricesCachePtr cache);

/* matrices for correlation computations */
//static MMXMatrix *image_matrix, *template_matrix, *result_matrix;
//static int MMX_is_allocated = 0;

 void image_to_matrix(unsigned char * image, MMXMatrix * matrix, int width, int height, int start_x, int start_y, int s_width, int s_height);

 void image_to_matrix_oversample(unsigned char * image, MMXMatrix * matrix, int width, int height, int s_width, int s_height);

 void matrix_to_image(MMXMatrix * matrix, unsigned char * image, int width, int height, int start_x, int start_y, int s_width, int s_height);

 void _exit(int status);
// int getopt(int argc, char * const argv[], const char *optstring);

 void *Alloc(size_t size);
 void *Realloc(void *p, size_t size);

 int dmap_blocker_getHeight(MMXMatrix * src, int dest_width, int D, int Doff, int E, int Eoff, int W);
 void dmap_blocker (MMXMatrix * mat, MMXMatrix * dest, int dest_width, int D, int Doff, int E, int Eoff, int W);
 void dmap_deblocker (MMXMatrix * mat, MMXMatrix * dest, int dest_width, int D, int Doff, int E, int Eoff, int W);

 void ResetTime(void);
 double GetTime(void);

 void median (MMXMatrix *I, MMXMatrix  *O);
 void MMXmedian (MMXMatrix *I, MMXMatrix  *O);
 void MMXrank_trans (MMXMatrix *I, MMXMatrix  *O);
 void MMXrow_filt(MMXMatrix *I, MMXMatrix  *K, MMXMatrix  *O);
 void MMXcol_filt(MMXMatrix *I, MMXMatrix  *K, MMXMatrix  *O);
 void MMXsubtract(MMXMatrix *I1, MMXMatrix  *I2, MMXMatrix  *O);
 void MMXLoG_filt(MMXMatrix *I, MMXMatrix  *O);
 void LoG_filt(MMXMatrix *I, MMXMatrix  *O);

 int Depthmap_SAD(const MMXMatrix *I1,const MMXMatrix *I2, MMXMatrix  *O,const int D,const int D_OFF, MMXMatricesCachePtr* vcache);
 int Depthmap_SAD_Hat(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *O, const int D, const int D_OFF, MMXMatricesCachePtr* vcache);
 int Recursive_Depthmap_SAD(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache);
 int Recursive_Depthmap_SAD3(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache);
 int MMXRecursive_Depthmap_SAD(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache);
 int MMXRecursive_Depthmap_SAD_U8(const MMXMatrix *I1,const MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache);
 int MMXRecursive_Depthmap_SAD2(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache);
 int MMXRecursive_Depthmap_SAD2a(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache);
 int MMXRecursive_Depthmap_SAD3(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache);
 int MMXRecursive_Depthmap_SAD4(const MMXMatrix *I1,const  MMXMatrix *I2, MMXMatrix  *Out,const  int D,const  int D_OFF, MMXMatricesCachePtr* vcache);
int MMXRecursive_Depthmap_SAD4_leanne(const MMXMatrix *I1,const  MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF,  const int W, const int lr_threshold, MMXMatricesCachePtr* vcache);
int MMXRecursive_Depthmap_SAD4point1(const MMXMatrix *I1,const  MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF,  const int W, const int lr_threshold, MMXMatricesCachePtr* vcache);
 int MMXRecursive_Depthmap_SAD5(const MMXMatrix *I1,const  MMXMatrix *I2, MMXMatrix  *Out,const  int D,const  int D_OFF, MMXMatricesCachePtr* vcache);
 int MMXRecursive_Depthmap_SAD32(const MMXMatrix *I1,const  MMXMatrix *I2, MMXMatrix  *Out, MMXMatricesCachePtr* vcache);
 int MMXRecursive_Depthmap_BLOCKER_SAD4(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, int offset);

 int MMXRecursive_Depthmap_SAD6(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache);


 int Depthmap_NCC(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *O);
 int Recursive_Depthmap_NCC(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out);
 int MMXRecursive_Depthmap_NCC(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out);
 int MMXRecursive_Depthmap_NCC2(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out);
 int MMXRecursive_Depthmap_NCC3(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out);
 int MMXRecursive_Depthmap_NCC4(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, int offset);
 int MMXRecursive_Depthmap_NCC4_leanne(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, int d_max, int offset, const int win, MMXMatricesCachePtr* vcache);

 int Depthmap_FLOW(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *O);
 int Recursive_Depthmap_FLOW(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *O);
 int MMXRecursive_Depthmap_FLOW(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *O);
 int MMXRecursive_Depthmap_FLOW2(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *O);
 int MMXRecursive_Depthmap_FLOW3(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *O);


 int Recursive_Depthmap_SAD_DYX(const MMXMatrix *I1,const MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache);
 int Recursive_Depthmap_SAD_DYX_U8(const MMXMatrix *I1,const MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache);
 int MMXRecursive_Depthmap_SAD_DYX(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache);
 int MMXRecursive_Depthmap_SAD_DYX_U8(const MMXMatrix *I1,const MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache);


 int MMXClip_1D(MMXMatrix *I, MMXMatrix  *O,
          unsigned char bmin, unsigned char bmax);

 int MMXClip_2D(MMXMatrix *I, MMXMatrix  *O,
          unsigned char gmin, unsigned char gmax,
          unsigned char bmin, unsigned char bmax);

 int MMXClip_3D(MMXMatrix *I, MMXMatrix  *O,
          unsigned char rmin, unsigned char rmax,
          unsigned char gmin, unsigned char gmax,
          unsigned char bmin, unsigned char bmax);

int dmap_RemapImage(unsigned char *src, unsigned char *dest, int *src_idx, int length);

int LocalUniqueness(MMXMatrix *I1, const int arg_x0, const int arg_y0, const int arg_wid, const int arg_hei,
                    MMXMatrix *Osum);


#ifdef __cplusplus
}
#endif

#endif
