/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */
 
#include <math.h>
#include <iostream>
#include <cstdio>
#include <stdlib.h>
#include <iCub/convert_rgb.h>

iCub::contrib::primateVision::Convert_RGB::Convert_RGB(IppiSize sze)
{
  //image allocation:
  width=sze.width;
  height=sze.height;

  yuva_orig = ippiMalloc_8u_C1(width*4,height,&psb_4);
  y_orig    = ippiMalloc_8u_C1(width,height,&psb);
  u_orig    = ippiMalloc_8u_C1(width,height,&psb);
  v_orig    = ippiMalloc_8u_C1(width,height,&psb);
  tmp       = ippiMalloc_8u_C1(width,height,&psb);
  //qrgb = new QImage(width,height,32,256);
  pyuva = (Ipp8u**) malloc(4*sizeof(Ipp8u*));

  srcsize.width=width;
  srcsize.height=height;  
}

iCub::contrib::primateVision::Convert_RGB::~Convert_RGB()
{

}

void iCub::contrib::primateVision::Convert_RGB::proc(Ipp8u *rgba_orig,int psb_o)
{
  //convert pixel order rgba to planar order yuv channels:
  //get pixel order yuva:
  ippiRGBToYUV_8u_AC4R(rgba_orig,psb_o,yuva_orig,psb_4,srcsize);
  //convert to planar yuva:
  pyuva[0]= y_orig;
  pyuva[1]= u_orig;
  pyuva[2]= v_orig; 
  pyuva[3]= tmp; 
  ippiCopy_8u_C4P4R(yuva_orig,psb_4,pyuva,psb,srcsize);
  //ippiCopy_8u_C4R(rgba_orig,psb_o,qrgb->bits(),width*4,srcsize);
}
