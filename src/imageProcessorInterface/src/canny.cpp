/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#include <iCub/canny.h> 
#include <ipp.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
  

CANNY::CANNY(IppiSize isize_)
{
  this->thresholdL=1.0;
  this->thresholdU=2.0;

  isize=isize_;

  pSDx = ippiMalloc_16s_C1(isize.width,isize.height,&psb_s);
  pSDy = ippiMalloc_16s_C1(isize.width,isize.height,&psb_s);
  //ippiCannyGetSize(isize, &bufsize);
  

  int sz_h,sz_v;
  ippiFilterSobelNegVertGetBufferSize_8u16s_C1R(isize,ippMskSize3x3, &sz_v);
  ippiFilterSobelHorizGetBufferSize_8u16s_C1R(isize,ippMskSize3x3, &sz_h);
  if (sz_h<sz_v) sz_h=sz_v;
  ippiCannyGetSize(isize, &bufsize);
  if (sz_h<sz_v) sz_h=sz_v;
  buf_h = (Ipp8u*) malloc(sz_h*sizeof(Ipp8u));
  buf_v = (Ipp8u*) malloc(sz_v*sizeof(Ipp8u));

  pbuf = (Ipp8u*) malloc(sizeof(Ipp8u)*bufsize);
  edge_map = ippiMalloc_8u_C1(isize.width,isize.height,&psb);


}


CANNY::~CANNY()
{

}

void CANNY::proc(Ipp8u* new_im_,int psb_i_,double TL_, double TU_)
{

  new_im = new_im_;
  psb_i=psb_i_;

  //---- interactive thresholds
  //TL_=this->thresholdL;
  //TU_=this->thresholdU;

  printf("Canny TL:%f, TU:%f",this->thresholdL,this->thresholdU);


  //SOBELS:
  ippiFilterSobelNegVertBorder_8u16s_C1R(new_im,psb_i,
				      pSDy,psb_s,isize, 
				      ippMskSize3x3,
				      ippBorderWrap, 
				      0,
				      buf_v);
  ippiFilterSobelHorizBorder_8u16s_C1R(new_im,psb_i,
                                       pSDx,psb_s,isize, 
					   ippMskSize3x3,
                                       ippBorderWrap, 
				       0, 
				       buf_h);
  
  //CANNY:
  ippiCanny_16s8u_C1R(pSDx, psb_s,
  		      pSDy, psb_s, 
  		      edge_map, psb,
			  isize,(Ipp32f)this->thresholdL,(Ipp32f)this->thresholdU,
  		      pbuf);
 
}

void CANNY::setIppiSize(IppiSize isize_){
  isize=isize_;

  pSDx = ippiMalloc_16s_C1(isize.width,isize.height,&psb_s);
  pSDy = ippiMalloc_16s_C1(isize.width,isize.height,&psb_s);
  ippiCannyGetSize(isize, &bufsize);
  pbuf = (Ipp8u*) malloc(sizeof(Ipp8u)*bufsize);
  edge_map = ippiMalloc_8u_C1(isize.width,isize.height,&psb);

  int sz_h,sz_v;
  ippiFilterSobelHorizGetBufferSize_8u16s_C1R(isize,ippMskSize3x3, &sz_h);
  ippiFilterSobelVertGetBufferSize_8u16s_C1R(isize,ippMskSize3x3, &sz_v);
  buf_h = (Ipp8u*) malloc(sz_h*sizeof(Ipp8u));
  buf_v = (Ipp8u*) malloc(sz_v*sizeof(Ipp8u));
}

void CANNY::setThresholdL(double value){
	//this->thresholdL=value;
	printf("SETTHRSHOLDL: %f",this->thresholdU);
}

void CANNY::setThresholdU(double value){
	//this->thresholdU=value;
	printf("SETTHRSHOLDU: %f",this->thresholdU);
}
  
