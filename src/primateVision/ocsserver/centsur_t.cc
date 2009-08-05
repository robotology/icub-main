/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include "centsur_t.h"
#include <timing.h>
#include <convert_bitdepth.h>

iCub::contrib::primateVision::CentsurT::CentsurT(
	   IppiSize srcsize_,
	   int nscale_,
	   int ncsscale,
	   double sigma)
{

  srcsize=srcsize_;
  nscale = nscale_;

  //CREATE serial Processing class: 
  centsur = (CentSur*) new CentSur(srcsize,ncsscale,sigma);

  //im alloc
  tot_32f = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  tot_8u  = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb_8u);

}

iCub::contrib::primateVision::CentsurT::~CentsurT()
{
   
}


void iCub::contrib::primateVision::CentsurT::run()
{
  //printf("CENTSURT: THREAD PROCESSING 0x%08x\n",this);
  //TSTART
  //centsur processing:
  done=false;

  //clear total:
  ippiSet_32f_C1R(0.0,tot_32f,psb_32f,srcsize);
  
  //proc each im serially:
  for (int s=0;s<nscale;s++){ 
    centsur->proc_im_32f(images[s], psb_32f);
    //add to total:
    ippiAdd_32f_C1IR(centsur->get_centsur_32f(),centsur->get_psb_32f(),tot_32f,psb_32f,srcsize);
  }
  
  //convert tot to 8u:
  conv_32f_to_8u(tot_32f,psb_32f,tot_8u,psb_8u,srcsize);
  
  done=true; 
  //printf("CENTSURT: THREAD DONE 0x%08x\n",this);
  //TSTOP  
}


void iCub::contrib::primateVision::CentsurT::proc_im_32f(Ipp32f**newims, int psb_32f_)
{

  //update image pointer:
  images = newims;
  psb_32f=psb_32f_;

  run(); //for serial processing, best on few cores.
  //start(); //for parallel processing, best on quad cores, etc.


}
