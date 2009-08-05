
#ifndef _MMX_UTILS_H
#define _MMX_UTILS_H

#include "MMXMatrix.h"

int MMXStrip(MMXMatrix *m, int argx0, int argx1, int argy0, int argy1, MMXMatrix *d);

float MMXGrav(MMXMatrix *S, MMXMatrix *V[2], MMXMatrix *E, 
	    float x[2], float v[2]);

void MMXmerge(unsigned char *img1, unsigned char *img2, 
	      int width, int height, unsigned char *dst);

void MMX_F32_U8(MMXMatrix *src, float factor, MMXMatrix *dst);

void MMXMultiply_F32(MMXMatrix *src1, MMXMatrix *src2, MMXMatrix *dst);

void MMXConvert_U8_F32(MMXMatrix *src, MMXMatrix *dst);

int MMXEigen(MMXMatrix *dxdx, MMXMatrix *dydy, MMXMatrix *dxdy, MMXMatrix *E);

int MMX_Enhance(unsigned char *src, int width, int height,
		int x, int y, int w, int h);

int MMXCopy(unsigned char *src, unsigned char *dst, int n);

void MMXMultiply_R32_F32(float factor, MMXMatrix *src, MMXMatrix *dst);

int MMXNormF32(MMXMatrix *mx, MMXMatrix *my, MMXMatrix *dst);

//int MMXCopy(unsigned char *dst, unsigned char *src, int n);

unsigned int MMXSum_U8(unsigned char *dst, int width, int height,
		       int x, int y, int w, int h);

// unsigned char clipped (not wrap around) B = A - B + 128
int MMXSubtract_U8p128(unsigned char *A, unsigned char *B, int n);

// unsigned char clipped (not wrap around) B = A + B - 128
int MMXAdd_U8m128(unsigned char *A, unsigned char *B, int n);


#endif

