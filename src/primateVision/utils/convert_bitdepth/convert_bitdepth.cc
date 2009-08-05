/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include "convert_bitdepth.h"




void iCub::contrib::primateVision::conv_32f_to_8u(Ipp32f*im_i,int p4_,Ipp8u*im_o,int p1_,IppiSize srcsize_)
{
  Ipp32f min,max;
  ippiMinMax_32f_C1R(im_i,p4_,srcsize_,&min,&max);
  if (max==min){max=255.0;min=0.0;}
  ippiScale_32f8u_C1R(im_i,p4_,im_o,p1_,srcsize_,min,max);
}

void iCub::contrib::primateVision::conv_8u_to_32f(Ipp8u*im_ci,int psb_8u_,Ipp32f*im_co,int psb_32f_,IppiSize ssize_)
{
  
  ippiConvert_8u32f_C1R(im_ci,psb_8u_,im_co,psb_32f_,ssize_);

}

