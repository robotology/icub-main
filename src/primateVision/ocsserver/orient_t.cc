/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include "orient_t.h" 


iCub::contrib::primateVision::OrientT::OrientT(
	   //ORIENT:
	   IppiSize srcsize, 
	   int psb_in,
	   int nscales, 
	   int norients, 
	   int thisorient,
	   int minWaveLength, 
	   double mult, 
	   double sigmaOnf, 
	   double dThetaOnSigma,
	   double k,
	   double cutOff,
	   int g
	   )
{

  orient = new LGPhase(srcsize,psb_in,nscales,norients,thisorient,minWaveLength,mult,sigmaOnf,dThetaOnSigma,k,cutOff,g);

}

iCub::contrib::primateVision::OrientT::~OrientT()
{
   
}


void iCub::contrib::primateVision::OrientT::run()
{
  //printf("ORIENTT: THREAD PROCESSING 0x%08x\n",this);
  //TSTART
  //oreint processing:
  done=false;
  orient->proc(imagefft);
  done=true;
  //printf("ORIENTT: THREAD DONE 0x%08x\n",this);
  //TSTOP
}


void iCub::contrib::primateVision::OrientT::proc(Ipp32fc*newim)
{

  //update image pointer:
  imagefft = newim;

  run(); //for serial processing, best on few cores.
  //start(); //for parallel processing, best on quad cores, etc.
}
