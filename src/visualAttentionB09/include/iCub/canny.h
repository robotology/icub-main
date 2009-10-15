
/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef __CANNY_H
#define __CANNY_H

#include <ipp.h>

class CANNY
{
 public:
  CANNY();
  CANNY(IppiSize imsize);
  ~CANNY();
  void proc(Ipp8u* im,int psb_i_,double TL,double TU);
  Ipp8u* get_edgemap(){return edge_map;}
  int get_psb(){return psb;}
  void setThresholdL(double value);
  void setThresholdU(double value);
  void setIppiSize(IppiSize imsize);	
 private:
  
  int psb_i,psb;
  int psb_s;
  Ipp8u* new_im;
  Ipp16s* pSDx;
  Ipp16s* pSDy;
  int bufsize;
  Ipp8u *buf_h,*buf_v,*pbuf;
  IppiSize isize;
  Ipp8u *edge_map;
  double thresholdU;
  double thresholdL;

};

#endif
