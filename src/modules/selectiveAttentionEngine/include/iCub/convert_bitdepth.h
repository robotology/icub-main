/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef CONVERT_BITDEPTH_H
#define CONVERT_BITDEPTH_H

#include <ipp.h>

void conv_32f_to_8u(Ipp32f*im_in,int psb_3,Ipp8u* im_out,int psb_o,IppiSize isze);
void conv_8u_to_32f(Ipp8u* im_in,int psb_3,Ipp32f*im_out,int psb_o,IppiSize isze);
void norm255_32f(Ipp32f*im,int psb_i,Ipp32f*im_out,int psb_o,IppiSize ssize);
void norm1_32f(Ipp32f*im,int psb,IppiSize ssize);


#endif
