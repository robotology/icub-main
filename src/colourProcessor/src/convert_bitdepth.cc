/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include <iCub/convert_bitdepth.h>

void conv_32f_to_8u(Ipp32f*im_i,int p4_,Ipp8u*im_o,int p1_,IppiSize srcsize_)
{
  Ipp32f min,max;
  ippiMinMax_32f_C1R(im_i,p4_,srcsize_,&min,&max);
  if (max==min){max=255.0;min=0.0;}
  ippiScale_32f8u_C1R(im_i,p4_,im_o,p1_,srcsize_,min,max);
}

void conv_8u_to_32f(Ipp8u*im_ci,int psb_8u_,Ipp32f*im_co,int psb_32f_,IppiSize ssize_)
{
  
  ippiConvert_8u32f_C1R(im_ci,psb_8u_,im_co,psb_32f_,ssize_);

}


void norm255_32f(Ipp32f*im,int psb_i,Ipp32f*im_out,int psb_o,IppiSize ssize){

  Ipp32f min,max;
  Ipp32f sc;
  
  //get min,max:
  ippiMinMax_32f_C1R(im,psb_i,ssize,&min,&max);

  //shift min to 0.0:
  ippiSubC_32f_C1R(im,psb_i,min,im_out,psb_o,ssize); 

  //scale image so max is 255.0:
  if (max!=min){
    sc = (Ipp32f)(255.0/(max-min));
    ippiMulC_32f_C1IR(sc,im_out,psb_o,ssize); 
  }

}




void norm1_32f(Ipp32f*im,int psb_i,Ipp32f*im_out,int psb_o,IppiSize ssize){

  Ipp32f min,max;
  Ipp32f sc;
  
  //get min,max:
  ippiMinMax_32f_C1R(im,psb_i,ssize,&min,&max);

  //shift min to 0.0:
  ippiSubC_32f_C1R(im,psb_i,min,im_out,psb_o,ssize); 

  //scale image so max is 255.0:
  if (max!=min){
    sc = (Ipp32f)(1.0/(max-min));
    ippiMulC_32f_C1IR(sc,im_out,psb_o,ssize); 
  }
  

}

//----- end-of-file --- ( next line intentionally left blank ) ------------------
