
#ifndef _MMX_YUV_H
#define _MMX_YUV_H

#include "MMXMatrix.h"

int yuv422_2_y_mmx_UYVY(unsigned char *src, unsigned char *dst, int n);
int yuv422_2_y_mmx_YUYV(unsigned char *src, unsigned char *dst, int n);

int yuv422_2_y_mmx(unsigned char *src, unsigned char *dst, int n);
int yuv422_2_y_mmx2(unsigned char *src, unsigned char *dst1, 
		    unsigned char *dst2, int w, int h);
int MMXShrink_YUV_U8_H2_V2(unsigned char *src, int width, int height,
			   unsigned char *dst);
int MMXExpand_YUV_U8_H2_V2(unsigned char *src, int width, int height,
			   unsigned char *dst);
int MMX_YUV_S16_sum(unsigned char *src, short *dst, int n);

MMXMatrix *MMX_YUV_Dist(MMXMatrix *src, 
			unsigned char y, unsigned char u, unsigned char v);

int MMX_YUV_Dist2(unsigned char *src, int width, int height,
		  unsigned char *dst,
		  unsigned char y, unsigned char u, unsigned char v);

int MMX_YUV_Shrink_H2(unsigned char *src, unsigned char *dst, int n);

int MMX_YUV_Shrink_H4(unsigned char *src, unsigned char *dst, int n);

float MMX_YUV_Grav(MMXMatrix *src,
		    unsigned char y, unsigned char u, unsigned char v,
		    float *px, float *py);

#endif
