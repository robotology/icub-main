
#ifndef _MMX_CONVERT_H
#define _MMX_CONVERT_H

#include "MMXMatrix.h"

int MMXShrink4_4Float(unsigned char *src, int w, int h, MMXMatrix *dst);
int MMXShrink8_4Float(unsigned char *src, int w, int h, MMXMatrix *dst);
int MMXShrink2_1Float(unsigned char *src, int w, int h, MMXMatrix *dst);
int MMX_MMF32_MMU8(MMXMatrix *src, MMXMatrix *dst);

// Convert from MMX Matrix to continuous unsigned char buffer
int MMX_MMU8_img(MMXMatrix *src, unsigned char *img);
// Convert from continuous unsigned char buffer to MMX Matrix
int MMX_img_MMU8(unsigned char *img, MMXMatrix *dst);

// Convert from MMX Matrix to stepped unsigned char buffer
int MMX_MMU8_simg(MMXMatrix *src, unsigned char *img, unsigned int step); 
// Convert from stepped unsigned char buffer to MMX Matrix
int MMX_simg_MMU8(unsigned char *img, unsigned int step, MMXMatrix *dst); 

int MMX_U8_S16(unsigned char *src, int width, int height,short *dst);
int MMX_U16_U8(unsigned short *src, int width, int height, unsigned char *dst);
int MMX_img_MMS16(unsigned char *src, int width, int height, MMXMatrix *dst);
int MMX_imgS16_F32(short *src, MMXMatrix *dst);
int MMX_S16_F32(MMXMatrix *src, MMXMatrix *dst);
int MMX_U8_S16_sum(unsigned char *src, int width, int height,
		   short *dst);
int MMX_U8_F32(MMXMatrix *src, MMXMatrix *dst);
int MMX_img_MMF32(unsigned char *src, int width, int height, MMXMatrix *dst);
int MMX_MMF32_img(float factor, float offset, 
		  MMXMatrix *src, unsigned char *dst);
int MMX_MMS32_img(float factor, float offset, 
		  MMXMatrix *src, unsigned char *dst);
#endif
