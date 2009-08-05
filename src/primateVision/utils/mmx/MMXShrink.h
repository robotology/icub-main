
#ifndef _MMX_SHRINK_H
#define _MMX_SHRINK_H

#include "MMXMatrix.h"

int MMXShrink_U8(int xfactor, int yfactor, unsigned char *src, 
		 int width, int height, unsigned char *dst);

int MMXShrink_U8_S16(int xfactor, int yfactor, unsigned char *src, 
		     int width, int height, short *dst);

//int MMXShrink_U8_60Hz(int xfactor, int yfactor, unsigned char *src,
//	      int width, int height,
//	      unsigned char *dst, int field);

int MMXShrink_H2_V2(MMXMatrix *src, MMXMatrix *dst);


int MMXExpand_U8(int xfactor, int yfactor, unsigned char *src, 
		 int width, int height, unsigned char *dst);

int MMXExpand_U8_H2_V4(unsigned char *src,int width, int height, unsigned char *dst);
int MMXExpand_U8_H2_V2(unsigned char *src,int width, int height, unsigned char *dst);



#endif

