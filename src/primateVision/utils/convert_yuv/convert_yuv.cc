/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <cstdio>
#include "convert_yuv.h"



iCub::contrib::primateVision::Convert_YUV::Convert_YUV(IppiSize sze)
{
  //image allocation:
  width=sze.width;
  height=sze.height;

  yuva_orig = ippiMalloc_8u_C1(width*4,height,&psb_4);
  y_orig    = ippiMalloc_8u_C1(width,height,&psb);
  uv_orig   = ippiMalloc_8u_C1(width,height,&psb);
  u_orig    = ippiMalloc_8u_C1(width,height,&psb);
  v_orig    = ippiMalloc_8u_C1(width,height,&psb);
  r_orig    = ippiMalloc_8u_C1(width,height,&psb);
  g_orig    = ippiMalloc_8u_C1(width,height,&psb);
  b_orig    = ippiMalloc_8u_C1(width,height,&psb);
  a_orig    = ippiMalloc_8u_C1(width,height,&psb);

  qrgb = new QImage(width,height,32,256);

  srcsize.width=width;
  srcsize.height=height;  

  pyuv=(Ipp8u**)malloc(3*sizeof(Ipp8u*));
  prgb=(Ipp8u**)malloc(3*sizeof(Ipp8u*));
  psrc=(const Ipp8u**) malloc(4*sizeof(const Ipp8u*));

}

iCub::contrib::primateVision::Convert_YUV::~Convert_YUV()
{
}

void iCub::contrib::primateVision::Convert_YUV::proc(Ipp8u* im)
{

  int p4 = width * height * 2;

  //make y channel image:
  yuv422_2_y_mmx_UYVY(&im[0], y_orig,p4);
  //make uvuv image:
  yuv422_2_y_mmx_UYVY(&im[1], uv_orig,p4);
  
  //convert uvuv to separate channels:
  //make uu:
  ippiCopy_8u_C4CR(&uv_orig[1],width,&u_orig[0],width,srcsize);
  ippiCopy_8u_C4CR(&uv_orig[1],width,&u_orig[1],width,srcsize);
  ippiCopy_8u_C4CR(&uv_orig[3],width,&u_orig[2],width,srcsize);
  ippiCopy_8u_C4CR(&uv_orig[3],width,&u_orig[3],width,srcsize);
  //make vv:
  ippiCopy_8u_C4CR(&uv_orig[0],width,&v_orig[0],width,srcsize);
  ippiCopy_8u_C4CR(&uv_orig[0],width,&v_orig[1],width,srcsize);
  ippiCopy_8u_C4CR(&uv_orig[2],width,&v_orig[2],width,srcsize);
  ippiCopy_8u_C4CR(&uv_orig[2],width,&v_orig[3],width,srcsize);
  
  //convert to planar rgb:
  pyuv[0]= y_orig;
  pyuv[1]= u_orig;
  pyuv[2]= v_orig;

  prgb[0]= r_orig;
  prgb[1]= g_orig;
  prgb[2]= b_orig;
  ippiYUVToRGB_8u_P3R((const Ipp8u**)pyuv,width,prgb,width,srcsize);

  //convert to pixel-order rgba:
  psrc[0]= r_orig;
  psrc[1]= g_orig;
  psrc[2]= b_orig;
  psrc[3]= a_orig;
  ippiCopy_8u_P4C4R((const Ipp8u**)psrc,width,qrgb->bits(),width*4,srcsize);
      
}
