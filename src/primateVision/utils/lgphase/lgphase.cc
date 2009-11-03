/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 * Implementation inspired by work of Peter Kovesi.
 *
 */

#include <math.h>
#include <iostream>
#include <cstdio>
#include <stdlib.h>
#include <algorithm>
#include "lgphase.h"


#define MAX(a,b) ((a) < (b) ? (b) : (a))



iCub::contrib::primateVision::LGPhase::LGPhase(IppiSize srcsize_, 
	       int psb_in_,
	       int nscales_, 
	       int norient_,
	       int thisorient_, 
	       int minWaveLength_, 
	       double mult_, 
	       double sigmaOnf_, 
	       double dThetaOnSigma_,
	       double k_,
	       double cutOff_,
	       int g_)
{
  psb_in = psb_in_;
  srcsize.width=srcsize_.width;
  srcsize.height=srcsize_.height;
  nscale=nscales_;
  norient=norient_;
  thisorient=thisorient_;
  g = g_;
  k = k_;
  cutOff = cutOff_;
  minWaveLength = minWaveLength_;
  mult = mult_;
  sigmaOnf = sigmaOnf_;
  dThetaOnSigma = dThetaOnSigma_;
  
  double thetaSigma = IPP_PI/norient/dThetaOnSigma;
  
  
  //****************MEMORY ALLOCATION********************
  radius     = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  theta      = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  sintheta   = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  costheta   = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  dtheta     = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  lp         = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  A2_E0      = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  tmp1_32f   = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  EstSumAiAj = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  EstSumAn2  = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  sumE       = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  sumO       = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  sumAn      = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  maxAn      = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  tmp_32fc   = ippiMalloc_32fc_C1(srcsize.width,srcsize.height,&psb_32fc);
  spread     = ippiMalloc_32fc_C1(srcsize.width,srcsize.height,&psb_32fc);  

  ifftFilterArray = (Ipp32f**)  malloc(nscale*sizeof(Ipp32f*));
  A_E0            = (Ipp32f**)  malloc(nscale*sizeof(Ipp32f*));
  R_E0            = (Ipp32f**)  malloc(nscale*sizeof(Ipp32f*));
  I_E0            = (Ipp32f**)  malloc(nscale*sizeof(Ipp32f*));
  E0              = (Ipp32fc**) malloc(nscale*sizeof(Ipp32fc*));
  logGabor        = (Ipp32fc**) malloc(nscale*sizeof(Ipp32fc*));
  filterArray     = (Ipp32fc**) malloc(nscale*sizeof(Ipp32fc*));
  for (int s=0;s<nscale;s++){
    ifftFilterArray[s] = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);    
    A_E0[s]            = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
    R_E0[s]            = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
    I_E0[s]            = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
    E0[s]              = ippiMalloc_32fc_C1(srcsize.width,srcsize.height,&psb_32fc);
    filterArray[s]     = ippiMalloc_32fc_C1(srcsize.width,srcsize.height,&psb_32fc);
    logGabor[s]        = ippiMalloc_32fc_C1(srcsize.width,srcsize.height,&psb_32fc);
  }


  //*************************************************************



  //CONSTRUCT FILTERS
  
  //Radial:
  for (int y=0;y<srcsize.height;y++){
    for (int x=0;x<srcsize.width;x++){
      radius[y*psb_32f/4 + x] = sqrt(((Ipp32f) (x-srcsize.width/2)/srcsize.width)*((Ipp32f) (x-srcsize.width/2)/srcsize.width) + ((Ipp32f) (y-srcsize.height/2)/srcsize.height)*((Ipp32f) (y-srcsize.height/2)/srcsize.height));
      theta[y*psb_32f/4 + x] = atan2(-((Ipp32f) (y-srcsize.height/2)/srcsize.height),((Ipp32f) (x-srcsize.width/2)/srcsize.width));
    }
  }   
  //remove zero:
  radius[(srcsize.height/2)*psb_32f/4 + (srcsize.width/2)] = 1.0;
  //quad shift:
  quadshift_32f(srcsize,radius);
  quadshift_32f(srcsize,theta);
  //sin & cos images:
  for (int y=0;y<srcsize.height;y++){  
    for (int x=0;x<srcsize.width;x++){
      sintheta[y*psb_32f/4 + x] = sin(theta[y*psb_32f/4 + x]);
      costheta[y*psb_32f/4 + x] = cos(theta[y*psb_32f/4 + x]);
    }
  }
  
  //Low Pass Filter:
  makelowpassfilter(srcsize,lp,0.45,15); //Radius .45, 'sharpness' 15
 
  //radial filter components:
  double wavelength,fo;
  for (int s=0;s<nscale;s++){
    wavelength = minWaveLength * pow(mult,s);
    fo = 1.0/wavelength;
    for (int y=0;y<srcsize.height;y++){  
      for (int x=0;x<srcsize.width;x++){
	logGabor[s][y*psb_32fc/8+x].re = lp[y*psb_32f/4+x] * exp((-log(radius[y*psb_32f/4+x]/fo)*log(radius[y*psb_32f/4+x]/fo))/(2*log(sigmaOnf)*log(sigmaOnf)));
      }
    }   
    //restore zero:
    logGabor[s][0].re=0.0;
  }
  

  //angular filter components:
  double angl = thisorient*IPP_PI/norient;
  printf("Preparing orient processing angle: %f\n",angl*180.0/IPP_PI);
  for (int y=0;y<srcsize.height;y++){ 
    for (int x=0;x<srcsize.width;x++){
      dtheta[y*psb_32f/4+x] = fabs(atan2(sintheta[y*psb_32f/4+x]*cos(angl)-costheta[y*psb_32f/4+x]*sin(angl),costheta[y*psb_32f/4+x]*cos(angl)+sintheta[y*psb_32f/4+x]*sin(angl)));
      spread[y*psb_32fc/8+x].re=exp((-dtheta[y*psb_32f/4+x]*dtheta[y*psb_32f/4+x])/(2*thetaSigma*thetaSigma));
    }
  }
  
  
  //complex filters and ifft of filters:
  int size;
  ippiDFTInitAlloc_C_32fc(&pDFTSpec,srcsize,IPP_FFT_DIV_INV_BY_N,ippAlgHintFast);
  ippiDFTGetBufSize_C_32fc(pDFTSpec,&size);
  pbuf = (Ipp8u*) malloc(size*sizeof(Ipp8u));

  Ipp32f sc=sqrt(srcsize.width*srcsize.height);

  for(int s=0;s<nscale;s++){
    //filter:
    ippiMul_32fc_C1R(logGabor[s],psb_32fc,spread,psb_32fc,filterArray[s],psb_32fc,srcsize);

    //ifft:
    ippiDFTInv_CToC_32fc_C1R(filterArray[s],psb_32fc,tmp_32fc,psb_32fc,pDFTSpec,pbuf);
    for (int y=0;y<srcsize.height;y++){ 
      for (int x=0;x<srcsize.width;x++){
	ifftFilterArray[s][y*psb_32f/4+x] = tmp_32fc[y*psb_32fc/8+x].re;
	}
      }
    ippiMulC_32f_C1IR(sc,ifftFilterArray[s],psb_32f,srcsize); 

    if (s==0){
      EM_n=0;
      for (int y=0;y<srcsize.height;y++){ 
	for (int x=0;x<srcsize.width;x++){
	  EM_n += (filterArray[s][y*psb_32fc/8+x].re * filterArray[s][y*psb_32fc/8+x].re) +
	    (filterArray[s][y*psb_32fc/8+x].im * filterArray[s][y*psb_32fc/8+x].im);
	}
      }
    }
    
  }




}



iCub::contrib::primateVision::LGPhase::~LGPhase()
{
  
}




void iCub::contrib::primateVision::LGPhase::proc(Ipp32fc *im_in)
{


  //printf("Processing orientation \n");
  
  //init accumulators:
  ippiSet_32f_C1R(0.0,sumAn,psb_32f,srcsize);
  ippiSet_32f_C1R(0.0,sumE,psb_32f,srcsize);
  ippiSet_32f_C1R(0.0,sumO,psb_32f,srcsize);


  for(int s=0;s<nscale;s++){
    //printf("   Processing scale %d\n",s);

    //FILTER:
    //convolve:
    ippiMul_32fc_C1R(filterArray[s],psb_32fc,im_in,psb_in,tmp_32fc,psb_32fc,srcsize);
    //idft:
    ippiDFTInv_CToC_32fc_C1R(tmp_32fc,psb_32fc,E0[s],psb_32fc,pDFTSpec,pbuf);
    
    //CONVERSION:
    //convert filtered image to real & imag. components:
    //for loop seems faster!!
    //comp_2_Re_Im(E0,width,height, R_E0, I_E0);
    for (int y=0;y<srcsize.height;y++){ 
      for (int x=0;x<srcsize.width;x++){
	R_E0[s][y*psb_32f/4+x] = E0[s][y*psb_32fc/8+x].re;
	I_E0[s][y*psb_32f/4+x] = E0[s][y*psb_32fc/8+x].im;
      }
    }

    //get Abs:
    ippiSqr_32f_C1R(R_E0[s],psb_32f,A2_E0,psb_32f,srcsize);
    ippiAddSquare_32f_C1IR(I_E0[s],psb_32f,A2_E0,psb_32f,srcsize);
    ippiSqrt_32f_C1R(A2_E0,psb_32f,A_E0[s],psb_32f,srcsize);
    

    //add to accums:
    ippiAdd_32f_C1IR(A_E0[s],psb_32f,sumAn,psb_32f,srcsize); //orientation energy
    ippiAdd_32f_C1IR(R_E0[s],psb_32f,sumE,psb_32f,srcsize); 
    ippiAdd_32f_C1IR(I_E0[s],psb_32f,sumO,psb_32f,srcsize); 

    if (s==0){
      ippiCopy_32f_C1R(A_E0[s],psb_32f,maxAn,psb_32f,srcsize);
    }
    else{
      for (int y=0;y<srcsize.height;y++){ 
	for (int x=0;x<srcsize.width;x++){
	  maxAn[y*psb_32f/4+x] = MAX(A_E0[s][y*psb_32f/4+x],maxAn[y*psb_32f/4+x]);
	}
      }
    }
    
  }
  
  
  
  
  //%%%%%%%%******** Comp noise:
  ippiSqr_32f_C1R(A_E0[0],psb_32f,tmp1_32f,psb_32f,srcsize);
  medianE2n = getMedian(tmp1_32f,psb_32f,srcsize);
  //printf("medianE2n: %f\n",medianE2n);
  meanE2n = -medianE2n/log(0.5);
  //printf("meanE2n: %f\n",meanE2n);
  //printf("EM_n: %f\n",EM_n);
  noisePower = meanE2n/EM_n; //% Estimate power.
  //printf("noisePower: %f\n",noisePower); 
  
  ippiSet_32f_C1R(0.0,EstSumAn2,psb_32f,srcsize);
  for (int s = 0;s<nscale;s++){
    ippiAddSquare_32f_C1IR(ifftFilterArray[s],psb_32f,EstSumAn2,psb_32f,srcsize);
  }
  
  ippiSet_32f_C1R(0.0,EstSumAiAj,psb_32f,srcsize);
  for (int si = 0;si<(nscale-1);si++){
    for (int sj = si;sj<nscale;sj++){     
      ippiMul_32f_C1R(ifftFilterArray[si],psb_32f,ifftFilterArray[sj],psb_32f,tmp1_32f,psb_32f,srcsize);
      ippiAdd_32f_C1IR(tmp1_32f,psb_32f,EstSumAiAj,psb_32f,srcsize);
    }
  }
  
  ippiSum_32f_C1R(EstSumAn2,psb_32f,srcsize,&sumEstSumAn2,ippAlgHintFast);
  //printf("sumEstSumAn2: %f\n",sumEstSumAn2);
  ippiSum_32f_C1R(EstSumAiAj,psb_32f,srcsize,&sumEstSumAiAj,ippAlgHintFast);
  //printf("sumEstSumAiAj: %f\n",sumEstSumAiAj);
  
  EstNoiseEnergy2 = 2*noisePower*sumEstSumAn2 + 4*noisePower*sumEstSumAiAj;
  //printf("EstNoiseEnergy2: %f\n",EstNoiseEnergy2);
  tau = sqrt(EstNoiseEnergy2/2);                     
  //printf("tau: %f\n",tau);
  EstNoiseEnergy = tau*sqrt(IPP_PI/2);               
  //printf("EstNoiseEnergy: %f\n",EstNoiseEnergy);
  EstNoiseEnergySigma = sqrt( (2-IPP_PI/2)*tau*tau );
  //printf("EstNoiseEnergySigma: %f\n",EstNoiseEnergySigma);
  
  T =  EstNoiseEnergy + k*EstNoiseEnergySigma;    
  //printf("T: %f\n",T);
  T = T/1.7;   
  //printf("T: %f\n",T);


}





void iCub::contrib::primateVision::LGPhase::quadshift_32f(IppiSize sroi, Ipp32f*im)
{
  int psb_32;
  Ipp32f* tmp = ippiMalloc_32f_C1(sroi.width,sroi.height,&psb_32);
  IppiSize roi;
  roi.width=sroi.width/2;
  roi.height=sroi.height/2;
  
  ippiCopy_32f_C1R(im,psb_32f,&tmp[(sroi.height*psb_32/4+sroi.width)/2],psb_32,roi);
  ippiCopy_32f_C1R(&im[(sroi.height*psb_32f/4+sroi.width)/2],psb_32f,tmp,psb_32,roi); 
  ippiCopy_32f_C1R(&im[(sroi.height*psb_32f/4)/2],psb_32f,&tmp[(psb_32/4)/2],psb_32,roi);
  ippiCopy_32f_C1R(&im[(psb_32f/4)/2],psb_32f,&tmp[(sroi.height*psb_32/4)/2],psb_32,roi);
  ippiCopy_32f_C1R(tmp,psb_32,im,psb_32f,sroi); 

  ippiFree(tmp); 
}



void iCub::contrib::primateVision::LGPhase::makelowpassfilter(IppiSize slp, Ipp32f*im,double cutoff,int n)
{
  
  for (int y=0;y<slp.height;y++){ 
    for (int x=0;x<slp.width;x++){
      
      im[y*psb_32f/4+x] = 1.0/(1.0 + pow(sqrt(((Ipp32f) (x-slp.width/2-1)/slp.width)*((Ipp32f) (x-slp.width/2-1)/slp.width) + ((Ipp32f) (y-slp.height/2-1)/slp.height)*((Ipp32f) (y-slp.height/2-1)/slp.height))/cutoff, 2*n));
    }
  }   
  
  quadshift_32f(slp,im);
  
}


double iCub::contrib::primateVision::LGPhase::getMedian(Ipp32f*im,int p,IppiSize isize) 
{

  std::sort(im, &im[isize.height*psb_32f/4 + isize.width]);
  
  return im[(isize.height*psb_32f/4 + isize.width)/2];

}
