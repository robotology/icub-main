/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include "norm.h"
#include <stdio.h>
#include <math.h>

void iCub::contrib::primateVision::normVal_32f(double val, Ipp32f*im,int psb_i,Ipp32f*im_out,int psb_o,IppiSize ssize){

  Ipp32f min,max;
  Ipp32f sc;
  
  //get min,max:
  ippiMinMax_32f_C1R(im,psb_i,ssize,&min,&max);

  //shift min to 0.0:
  ippiSubC_32f_C1R(im,psb_i,min,im_out,psb_o,ssize); 

  //scale image so max is val:
  if (max!=min){
    sc = (Ipp32f)(val/(max-min));
    ippiMulC_32f_C1IR(sc,im_out,psb_o,ssize); 
  }
}


void iCub::contrib::primateVision::normValI_32f(double val, Ipp32f*im,int psb,IppiSize ssize){

  Ipp32f min,max;
  Ipp32f sc;
  
  //get min,max:
  ippiMinMax_32f_C1R(im,psb,ssize,&min,&max);

  //shift min to 0.0:
  ippiSubC_32f_C1IR(min,im,psb,ssize); 

  //scale image so max is val:
  if (max!=min){
    sc = (Ipp32f)(val/(max-min));
    ippiMulC_32f_C1IR(sc,im,psb,ssize); 
  }
}




void iCub::contrib::primateVision::normVal_8u(int val, Ipp8u*im,int psb_i,Ipp8u*im_out,int psb_o,IppiSize ssize){

  Ipp8u min,max;
  float sc=1.0;
  
  //get min,max:
  ippiMinMax_8u_C1R(im,psb_i,ssize,&min,&max);

  //shift min to 0:
  ippiSubC_8u_C1RSfs(im,psb_i,min,im_out,psb_o,ssize,0); 

  //scale image so max is Val:
  if (max>min){
    sc = (float)(((float)val)/(((float)max)-((float)min)));
  }

  if (sc>=1.0){
    ippiMulC_8u_C1IRSfs((int)floor(sc),im_out,psb_o,ssize,0); 
  }
  else{
    ippiDivC_8u_C1IRSfs((int)ceil(1.0/sc),im_out,psb_o,ssize,0);
  }  

}

void iCub::contrib::primateVision::normValI_8u(int val, Ipp8u*im,int psb,IppiSize ssize){

  Ipp8u min,max;
  float sc=1.0;
  
  //get min,max:
  ippiMinMax_8u_C1R(im,psb,ssize,&min,&max);

  //shift min to 0:
  ippiSubC_8u_C1IRSfs(min,im,psb,ssize,0); 

  //scale image so max is Val:
  if (max>min){
    sc = (float)(((float)val)/(((float)max)-((float)min)));
  }

  if (sc>=1.0){
    ippiMulC_8u_C1IRSfs((int)floor(sc),im,psb,ssize,0); 
  }
  else{
    ippiDivC_8u_C1IRSfs((int)ceil(1.0/sc),im,psb,ssize,0);
  }
 
}

