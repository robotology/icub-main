/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.  
 *
 * An implementation inspired by the work and MATLAB 
 * implementation of Peter Kovesi.
 * 
 */

#include "ocs.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <qstring.h>
#include <qimage.h>


//#define FILE //leave commented out to run live
#define WINDOW 0
#define EPSILON 0.0001
#define MAX(a,b) ((a) < (b) ? (b) : (a))

//My includes
#include <timing.h>
#include <convert_bitdepth.h>
#include "orient_t.h"
#include "centsur_t.h"
// THIS APP IS A RECCLIENT!!!
#include <recio.h>





iCub::contrib::primateVision::OCSServer::OCSServer(string*c_)
{
  
  cfg = c_; 
  
  start();

}

iCub::contrib::primateVision::OCSServer::~OCSServer()
{
   
}


void iCub::contrib::primateVision::OCSServer::run(){
  
  Property prop;
  prop.fromConfigFile(cfg->c_str());

  int servernum = prop.findGroup("OCS").find("SERVERNUM").asInt();
  int input     = prop.findGroup("OCS").find("INPUT").asInt();
  int norient   = prop.findGroup("OCS").find("NORIENT").asInt();
  int nscale    = prop.findGroup("OCS").find("NSCALE").asInt();

  int minWaveLength    = prop.findGroup("ORIENT").find("MINWAVELENGTH").asInt();
  double mult          = prop.findGroup("ORIENT").find("MULT").asDouble();
  double k             = prop.findGroup("ORIENT").find("K").asDouble();
  double sigmaOnf      = prop.findGroup("ORIENT").find("SIGMAONF").asDouble();
  double dThetaOnSigma = prop.findGroup("ORIENT").find("DTHETAONSIGMA").asDouble();
  double cutOff        = prop.findGroup("ORIENT").find("CUTOFF").asDouble();
  int g                = prop.findGroup("ORIENT").find("G").asInt();
  int polarity         = prop.findGroup("ORIENT").find("SYM_POLARITY").asInt();

  int ncsscale = prop.findGroup("CENTSUR").find("NCSSCALE").asInt();
  double sigma = prop.findGroup("CENTSUR").find("SIGMA").asDouble();

  printf("%d orientations, %d scales (%d feature maps). \n",norient,nscale,nscale*norient);
  printf("%d centsur scales for each feature map. \n",ncsscale);
  printf("Symmetry Polarity:%d \n",polarity);

  int psb,psb_32f,psb_32fc;

  IppiSize srcsize;
  int psb_in;
  Ipp8u* rec_im_y;


#ifdef FILE
  QImage*im = new QImage("ad_g.jpg","JPEG");
  srcsize.width = im->width();
  srcsize.height = im->height();
  psb_in = im->width();
#else
  //IN PORTS:
  Port inPort_s;
  inPort_s.open("/ocsserver_"+QString::number(servernum)+"/input/serv_params");     // Give it a name on the network.
  Network::connect("/ocsserver_"+QString::number(servernum)+"/input/serv_params", "/recserver/output/serv_params");
  Network::connect("/recserver/output/serv_params", "/ocsserver_"+QString::number(servernum)+"/input/serv_params");
  BinPortable<RecServerParams> response; 
  Bottle empty;
  inPort_s.write(empty,response);
  RecServerParams rsp = response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl; 
  srcsize.width = rsp.width;
  srcsize.height = rsp.height; 
  psb_in = rsp.psb;
#endif
  


  // OUTPUT PORTS:
  BufferedPort<Bottle> outPort_s;
  outPort_s.open("/ocsserver_"+QString::number(servernum)+"/output/serv_params");

  BufferedPort<Bottle> outPort_m;
  outPort_m.open("/ocsserver_"+QString::number(servernum)+"/output/m");

  BufferedPort<Bottle> outPort_M;
  outPort_M.open("/ocsserver_"+QString::number(servernum)+"/output/M");

  BufferedPort<Bottle> outPort_phaseSym;
  outPort_phaseSym.open("/ocsserver_"+QString::number(servernum)+"/output/phaseSym");

  BufferedPort<Bottle> outPort_ocs;
  outPort_ocs.open("/ocsserver_"+QString::number(servernum)+"/output/ocs");




  //Make replier for INDIVIDUAL ORIENTATION RESPONSE requests:
  //*********************
  //*****ADDME***********
  //*********************




  //Allocate space for total ocs image:
  Ipp32f* tmp1_32f          = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* tmp2_32f          = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* Energy_ThisOrient = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* XEnergy           = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* Energy            = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* totalSumAn        = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* totalEnergy       = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* change            = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* not_change        = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* E                 = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* O                 = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* MeanE             = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* MeanO             = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* width             = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* weight            = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* phaseSym          = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* covx              = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* covy              = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* covx2             = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* covy2             = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* covxy             = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  //Ipp32f* maxEnergy         = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  //Ipp32f* or_               = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  //Ipp32f* neg               = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  //Ipp32f* not_neg           = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  //Ipp32f* orientation       = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* denom             = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* sin2theta         = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* cos2theta         = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* m                 = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* M                 = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* ocs_tot_32f       = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);

  Ipp8u* ocs_tot_8u         = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);
  Ipp8u* m_8u               = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);
  Ipp8u* M_8u               = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);
  Ipp8u* phaseSym_8u        = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);

  double angl;
 
  //complex filters and ifft of filters:
  Ipp32fc*im_32fc           = ippiMalloc_32fc_C1(srcsize.width,srcsize.height,&psb_32fc);
  Ipp32fc*imagefft          = ippiMalloc_32fc_C1(srcsize.width,srcsize.height,&psb_32fc);
  int size;
  IppiDFTSpec_C_32fc *pDFTSpec;
  ippiDFTInitAlloc_C_32fc(&pDFTSpec,srcsize,IPP_FFT_DIV_INV_BY_N,ippAlgHintFast);
  ippiDFTGetBufSize_C_32fc(pDFTSpec,&size);
  Ipp8u*pbuf = (Ipp8u*) malloc(size*sizeof(Ipp8u));


  //CREATE Processing classes: 
  int thisorient        = 0;
  OrientT**orientcs    = (OrientT**)  malloc(norient*sizeof(OrientT*));
  CentsurT**centsurcs  = (CentsurT**) malloc(norient*sizeof(CentsurT*));
  Ipp32f**PC            = (Ipp32f**)    malloc(norient*sizeof(Ipp32f*));
  Ipp32f***csims        = (Ipp32f***)   malloc(norient*sizeof(Ipp32f**));
  //Ipp32fc**featType   = (Ipp32fc**)   malloc(norient*sizeof(Ipp32fc*));
  for(int n=0;n<norient;n++){
    orientcs[n]  = (OrientT*) new OrientT(srcsize,psb_32fc,nscale,norient,thisorient,
					    minWaveLength,mult,sigmaOnf,dThetaOnSigma,k,cutOff,g);
    centsurcs[n] = (CentsurT*) new CentsurT(srcsize,nscale,ncsscale,sigma);
    csims[n]     = (Ipp32f**) malloc(nscale*sizeof(Ipp32f*));
    PC[n]        = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
    //featType[n]  = ippiMalloc_32fc_C1(srcsize.width,srcsize.height,&psb_32fc);
    thisorient++;
  } 
  

#if WINDOW
  //make ayers-rock windowing function:
  Ipp8u*ar_window = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);
  Ipp8u*im_8u_tmp = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);
  ippiSet_8u_C1R(255,ar_window,psb,srcsize);
  double sigma_w = 30.0;
  double val;
  for (int y=0;y<srcsize.height;y++){
    for (int x=0;x<(int)(2.0*sigma_w);x++){
      val = exp((float) ( -((x-2.0*sigma_w)*(x-2.0*sigma_w))/(2.0*sigma_w*sigma_w) ) );
      //left:
      ar_window[x+y*psb] = (int) ( ar_window[x+y*psb] * val );
      //right:    
      ar_window[srcsize.width-1-x+y*psb] = (int) ( ar_window[srcsize.width-1-x+y*psb] * val );
    }  
  }
  for (int x=0;x<srcsize.width;x++){
    for (int y=0;y<(int)(2.0*sigma_w);y++){
      val = exp((float) ( -((y-2.0*sigma_w)*(y-2.0*sigma_w))/(2.0*sigma_w*sigma_w) ) );
      //top:
      ar_window[x+y*psb] = (int) ( ar_window[x+y*psb] * val );
      //bottom:    
      ar_window[x+(srcsize.height-1-y)*psb] = (int) ( ar_window[x+(srcsize.height-1-y)*psb] * val );
    }  
  }
#endif


  // Make replier for server param probes on server port:
  //Set Params:
  OCSServerParams osp;
  osp.width=srcsize.width;
  osp.height=srcsize.height;
  osp.psb=psb;
  //Replier:
  OCSReplyParamProbe server_replier;
  server_replier.reply=osp;
  outPort_s.setReplier(server_replier);


#ifndef FILE  
  BufferedPort<Bottle> inPort_y;
  if (input==0){
    inPort_y.open("/ocsserver_"+QString::number(servernum)+"/input/rec_ly");     // Give it a name on the network.
    Network::connect("/recserver/output/left_ye" , "/ocsserver_"+QString::number(servernum)+"/input/rec_ly");
  }
  else{
    inPort_y.open("/ocsserver_"+QString::number(servernum)+"/input/rec_ry");     // Give it a name on the network.
    Network::connect("/recserver/output/right_ye" , "/ocsserver_"+QString::number(servernum)+"/input/rec_ry");
  }
  Bottle *inBot_y;
#endif








  TCREATE




  //MAIN EVENT LOOP:
  int done_cs=0,done_o=0;
  printf("Begin\n");
  while (1)
    {   
      
      
      TSTART

#ifdef FILE      
      if (1){
	rec_im_y = (Ipp8u*) im->bits();      
#else


      //use currently buffered image:
      inBot_y = inPort_y.read(); //false





      if (inBot_y!=NULL){
	rec_im_y = (Ipp8u*) inBot_y->get(0).asBlob();
#endif
	
	  
	  
#if WINDOW
	//mul image by ayers-rock windowing function:
	ippiMul_8u_C1RSfs(rec_im_y,psb,ar_window,psb,im_8u_tmp,psb,srcsize,8);
#endif
	
	//convert image to complex:
	//Re_2_comp(im_in,width,height,im_32fc);     //for loop seems faster!
	for (int y=0;y<srcsize.height;y++){ 
	  for (int x=0;x<srcsize.width;x++){
#if WINDOW
	    im_32fc[y*psb_32fc/8+x].re =  (double) im_8u_tmp[y*psb+x];
#else
	    im_32fc[y*psb_32fc/8+x].re =  (double) rec_im_y[y*psb+x];
#endif
	  }
	} 
	
	
	//take dft, this is what orient class requires:
	ippiDFTFwd_CToC_32fc_C1R(im_32fc,psb_32fc,imagefft,psb_32fc,pDFTSpec,pbuf);
	
	
	
	
	
	
		
	// FOR EACH OF THIS SERVER'S ORIENTATIONS, LAUNCH ORIENT PROC THREADS
	for(int n=0;n<norient;n++){
	  orientcs[n]->proc(imagefft);
	}
	//if using parallel orient threads, wait for them to finish:
	//done_o = 0;
	//while (done_o<norient){
	//  done_o = 0;
	//  usleep(1000);
	//  for (int n=0;n<norient;n++){ 
	//    done_o += (int) orientcs[n]->is_done();
	//  }
	//}






	//start n (4) centsur threads in parallel:
	for(int n=0;n<norient;n++){
	  for(int s=0;s<nscale;s++){
	    csims[n][s] = orientcs[n]->get_A_EO_32f(s);
	  } 
	}
	for(int n=0;n<norient;n++){
	  centsurcs[n]->proc_im_32f(csims[n],orientcs[n]->get_psb_32f());
	}
	//we wait for them to finish below, when they are required..




	//%%%%%CUE POST PROC!!!!


	
	//init Accums:
	//PC
	ippiSet_32f_C1R(0.0,covx2,psb_32f,srcsize);
	ippiSet_32f_C1R(0.0,covy2,psb_32f,srcsize);
	ippiSet_32f_C1R(0.0,covxy,psb_32f,srcsize);
	//ippiSet_32f_C1R(0.0,maxEnergy,psb_32f,srcsize);
	//PS
	ippiSet_32f_C1R(0.0,totalSumAn,psb_32f,srcsize);
	ippiSet_32f_C1R(0.0,totalEnergy,psb_32f,srcsize);
	//ippiSet_32f_C1R(0.0,orientation,psb_32f,srcsize);
	
	
	
	for(int n=0;n<norient;n++){
	  
	  
	  //%*****SYMMETRY
	  //phase symmetry measure
	  ippiSet_32f_C1R(0.0,Energy_ThisOrient,psb_32f,srcsize);
	  ippiSet_32f_C1R(0.0,change,psb_32f,srcsize);
	  if (polarity == 0){// % look for 'white' and 'black' spots
	    for (int s = 0;s<nscale;s++){
	      ippiAbs_32f_C1R(orientcs[n]->get_R_EO_32f(s),psb_32f,tmp1_32f,psb_32f,srcsize);
	      ippiAbs_32f_C1R(orientcs[n]->get_I_EO_32f(s),psb_32f,tmp2_32f,psb_32f,srcsize);
	      ippiSub_32f_C1IR(tmp2_32f,psb_32f,tmp1_32f,psb_32f,srcsize);
	      ippiAdd_32f_C1IR(tmp1_32f,psb_32f,Energy_ThisOrient,psb_32f,srcsize);
	    }
	  }
	  else if (polarity == 1){//  % Just look for 'white' spots
	    for (int s = 0;s<nscale;s++){                 
	      ippiAdd_32f_C1IR(orientcs[n]->get_R_EO_32f(s),psb_32f,Energy_ThisOrient,psb_32f,srcsize);
	      ippiAbs_32f_C1R(orientcs[n]->get_I_EO_32f(s),psb_32f,tmp2_32f,psb_32f,srcsize);
	      ippiSub_32f_C1IR(tmp2_32f,psb_32f,Energy_ThisOrient,psb_32f,srcsize);
	    }
	  }
	  else if (polarity == -1){//  % Just look for 'black' spots
	    for (int s=0;s<nscale;s++){   
	      ippiSub_32f_C1IR(orientcs[n]->get_R_EO_32f(s),psb_32f,Energy_ThisOrient,psb_32f,srcsize);
	      ippiAbs_32f_C1R(orientcs[n]->get_I_EO_32f(s),psb_32f,tmp2_32f,psb_32f,srcsize);
	      ippiSub_32f_C1IR(tmp2_32f,psb_32f,Energy_ThisOrient,psb_32f,srcsize);
	    }
	  }
	  
	  //% Apply noise threshold 
	  ippiSubC_32f_C1IR(orientcs[n]->get_T(),Energy_ThisOrient,psb_32f,srcsize);
	  ippiThreshold_LT_32f_C1IR(Energy_ThisOrient,psb_32f,srcsize,0.0);
	  
	  
	  //% Update accumulator matrix for sumAn and totalEnergy
	  ippiAdd_32f_C1IR(orientcs[n]->get_sumAn_32f(),psb_32f,totalSumAn,psb_32f,srcsize);
	  ippiAdd_32f_C1IR(Energy_ThisOrient,psb_32f,totalEnergy,psb_32f,srcsize);
	  
	  //*******SYMMETRY ORIENTATION NOT REQUIRED/USED AT THE MOMENT:
	  // 	  if(n == 0){
	  // 	    //maxEnergy = Energy_ThisOrient;
	  // 	    ippiCopy_32f_C1R(Energy_ThisOrient,psb_32f,maxEnergy,psb_32f,srcsize);
	  // 	  }
	  // 	  else{
	  // 	    for (int y=0;y<srcsize.height;y++){ 
	  // 	      for (int x=0;x<srcsize.width;x++){
	  // 		if (Energy_ThisOrient[y*srcsize.width+x]>maxEnergy[y*srcsize.width+x]){
	  // 		  change[y*srcsize.width+x] = 1.0;
	  // 		  not_change[y*srcsize.width+x] = 0.0;
	  // 		}
	  // 		else{
	  // 		  change[y*srcsize.width+x] = 0.0;
	  // 		  not_change[y*srcsize.width+x] = 1.0;
	  // 		}
	  // 	      }
	  // 	    }
	  //
	  // 	    ippiMulC_32f_C1IR(n,change,psb_32f,srcsize);
	  // 	    ippiMul_32f_C1IR(not_change,psb_32f,orientation,psb_32f,srcsize);
	  // 	    ippiAdd_32f_C1IR(change,psb_32f,orientation,psb_32f,srcsize);
	  //
	  // 	    //maxEnergy = max(maxEnergy, Energy_ThisOrient);
	  // 	    for (int y=0;y<srcsize.height;y++){ 
	  // 	      for (int x=0;x<srcsize.width;x++){
	  // 		maxEnergy[y*srcsize.width+x] = MAX(maxEnergy[y*srcsize.width+x], Energy_ThisOrient[y*srcsize.width+x]);
	  // 	      }
	  // 	    }
	  //      }
	  //*****

	  //%*****SYMMETRY
	  
	  
	  
	  
	  //%*****PHASECONG:
	  ippiSqr_32f_C1R(orientcs[n]->get_sumE_32f(),psb_32f,XEnergy,psb_32f,srcsize);
	  ippiAddSquare_32f_C1IR(orientcs[n]->get_sumO_32f(),psb_32f,XEnergy,psb_32f,srcsize);
	  ippiSqrt_32f_C1IR(XEnergy,psb_32f,srcsize);
	  ippiAddC_32f_C1IR(EPSILON,XEnergy,psb_32f,srcsize);
	  ippiDiv_32f_C1R(XEnergy,psb_32f,orientcs[n]->get_sumE_32f(),psb_32f,MeanE,psb_32f,srcsize);
	  ippiDiv_32f_C1R(XEnergy,psb_32f,orientcs[n]->get_sumO_32f(),psb_32f,MeanO,psb_32f,srcsize);
	  
	  ippiSet_32f_C1R(0.0,Energy,psb_32f,srcsize);	        //PhaseCong energy
	  for (int s = 0;s<nscale;s++){
	    ippiCopy_32f_C1R(orientcs[n]->get_R_EO_32f(s),psb_32f,E,psb_32f,srcsize);
	    ippiCopy_32f_C1R(orientcs[n]->get_I_EO_32f(s),psb_32f,O,psb_32f,srcsize);
	    ippiMul_32f_C1R(E,psb_32f,MeanE,psb_32f,tmp1_32f,psb_32f,srcsize);
	    ippiAdd_32f_C1IR(tmp1_32f,psb_32f,Energy,psb_32f,srcsize);
	    ippiMul_32f_C1R(O,psb_32f,MeanO,psb_32f,tmp1_32f,psb_32f,srcsize);
	    ippiAdd_32f_C1IR(tmp1_32f,psb_32f,Energy,psb_32f,srcsize);
	    ippiMul_32f_C1R(E,psb_32f,MeanO,psb_32f,tmp1_32f,psb_32f,srcsize);
	    ippiMul_32f_C1R(O,psb_32f,MeanE,psb_32f,tmp2_32f,psb_32f,srcsize);
	    ippiSub_32f_C1IR(tmp2_32f,psb_32f,tmp1_32f,psb_32f,srcsize);
	    ippiAbs_32f_C1IR(tmp1_32f,psb_32f,srcsize);
	    ippiSub_32f_C1IR(tmp1_32f,psb_32f,Energy,psb_32f,srcsize);	  
	  }
	  	  
	  ippiSubC_32f_C1IR(orientcs[n]->get_T(),Energy,psb_32f,srcsize);
	  ippiThreshold_LT_32f_C1IR(Energy,psb_32f,srcsize,0.0);
	  
	  ippiAddC_32f_C1R(orientcs[n]->get_maxAn_32f(),psb_32f,EPSILON,tmp1_32f,psb_32f,srcsize);
	  ippiDiv_32f_C1R(tmp1_32f,psb_32f,orientcs[n]->get_sumAn_32f(),psb_32f,width,psb_32f,srcsize);
	  ippiDivC_32f_C1IR(nscale,width,psb_32f,srcsize);
	  
	  ippiSet_32f_C1R(1.0,tmp1_32f,psb_32f,srcsize);
	  ippiMulC_32f_C1R(width,psb_32f,-1.0,tmp2_32f,psb_32f,srcsize);
	  ippiAddC_32f_C1IR(cutOff,tmp2_32f,psb_32f,srcsize);
	  ippiMulC_32f_C1IR(g,tmp2_32f,psb_32f,srcsize);
	  ippiExp_32f_C1IR(tmp2_32f,psb_32f,srcsize);
	  ippiAddC_32f_C1IR(1.0,tmp2_32f,psb_32f,srcsize);
	  ippiDiv_32f_C1R(tmp2_32f,psb_32f,tmp1_32f,psb_32f,weight,psb_32f,srcsize);
	  
	  ippiMul_32f_C1R(weight,psb_32f,Energy,psb_32f,PC[n],psb_32f,srcsize);
	  ippiDiv_32f_C1IR(orientcs[n]->get_sumAn_32f(),psb_32f,PC[n],psb_32f,srcsize);
	  
	  //*****featType NOT REQUIRED/USED AT THE MOMENT:
	  //for (int y=0;y<srcsize.height;y++){ 
	  //  for (int x=0;x<srcsize.width;x++){
	  //    featType[n][y*srcsize.width+x].re = E[y*srcsize.width+x];
	  //    featType[n][y*srcsize.width+x].im = O[y*srcsize.width+x];
	  //  }
	  //}
	  //**** 
	  
	  angl = n*IPP_PI/norient;
	  ippiMulC_32f_C1R(PC[n],psb_32f,cos(angl),covx,psb_32f,srcsize);
	  ippiMulC_32f_C1R(PC[n],psb_32f,sin(angl),covy,psb_32f,srcsize);
	  ippiAddSquare_32f_C1IR(covx,psb_32f,covx2,psb_32f,srcsize);
	  ippiAddSquare_32f_C1IR(covy,psb_32f,covy2,psb_32f,srcsize);
	  ippiMul_32f_C1R(covx,psb_32f,covy,psb_32f,tmp1_32f,psb_32f,srcsize);
	  ippiAdd_32f_C1IR(tmp1_32f,psb_32f,covxy,psb_32f,srcsize);
	  //*****PHASECONG
	  
	  
	}//end for each orientation
	
	
	
	
	//////POST-POST-PROCESSING:
	
	//****PHASECONG:
	ippiDivC_32f_C1IR(norient/2.0,covx2,psb_32f,srcsize);
	ippiDivC_32f_C1IR(norient/2.0,covy2,psb_32f,srcsize);
	ippiDivC_32f_C1IR(norient,covxy,psb_32f,srcsize);
	
	ippiSqr_32f_C1R(covxy,psb_32f,tmp1_32f,psb_32f,srcsize);
	ippiSub_32f_C1R(covy2,psb_32f,covx2,psb_32f,tmp2_32f,psb_32f,srcsize);
	ippiAddSquare_32f_C1IR(tmp2_32f,psb_32f,tmp1_32f,psb_32f,srcsize);
	ippiSqrt_32f_C1R(tmp1_32f,psb_32f,denom,psb_32f,srcsize);
	ippiAddC_32f_C1IR(EPSILON,denom,psb_32f,srcsize);
	
	ippiDiv_32f_C1R(denom,psb_32f,covxy,psb_32f,sin2theta,psb_32f,srcsize);
	ippiDiv_32f_C1R(denom,psb_32f,tmp2_32f,psb_32f,cos2theta,psb_32f,srcsize);	
	
	
	//**** FEATURE ORIENTATIONS NOT REQUIRED/USED AT THE MOMENT:
	//for (int y=0;y<srcsize.height;y++){ 
	//  for (int x=0;x<srcsize.width;x++){
	//    
	//    or_[y*srcsize.width+x] = atan2(sin2theta[y*srcsize.width+x],cos2theta[y*srcsize.width+x]);    
	//    
	//    //neg = or < 0;  
	//    if (or_[y*srcsize.width+x] < 0.0){
	//      neg[y*srcsize.width+x] = 1.0;
	//      not_neg[y*srcsize.width+x] = 0.0;
	//    }
	//    else{
	//      neg[y*srcsize.width+x] = 0.0;
	//      not_neg[y*srcsize.width+x] = 1.0;
	//    }
	//    
	//  }
	//}
	//ippiMulC_32f_C1IR(90.0/IPP_PI,or_,psb_32f,srcsize);
	//ippiAddC_32f_C1R(or_,psb_32f,180.0,tmp1_32f,psb_32f,srcsize);
	//ippiMul_32f_C1IR(neg,psb_32f,tmp1_32f,psb_32f,srcsize);
	//ippiMul_32f_C1R(not_neg,psb_32f,or_,psb_32f,tmp2_32f,psb_32f,srcsize);
	//ippiAdd_32f_C1R(tmp1_32f,psb_32f,tmp2_32f,psb_32f,or_,psb_32f,srcsize);
	//**** 
		
	ippiAdd_32f_C1R(covy2,psb_32f,covx2,psb_32f,tmp1_32f,psb_32f,srcsize);
	ippiAdd_32f_C1R(denom,psb_32f,tmp1_32f,psb_32f,M,psb_32f,srcsize);
	ippiDivC_32f_C1IR(2.0,M,psb_32f,srcsize);//   EDGES
	
	ippiSub_32f_C1R(denom,psb_32f,tmp1_32f,psb_32f,m,psb_32f,srcsize);
	ippiDivC_32f_C1IR(2.0,m,psb_32f,srcsize);//   CORNERS
	//****PHASECONG	
	



	//%*****SYMMETRY
	ippiAddC_32f_C1R(totalSumAn,psb_32f,EPSILON,tmp1_32f,psb_32f,srcsize);
	ippiDiv_32f_C1R(tmp1_32f,psb_32f,totalEnergy,psb_32f,phaseSym,psb_32f,srcsize);

	//****SYMMETRY ORIENTATION NOT REQUIRED/USED AT THE MOMENT:
	//ippiMulC_32f_C1IR(180.0/norient,orientation,psb_32f,srcsize);
	//****

	//%*****SYMMETRY
	

	






	////PREPARE OUTPUTS:
	
	//SPATIAL ORIENTATION UNIQUENESS:
	//Wait for CS processing to finish:
	//done_cs = 0;
	//while (done_cs<norient){
	//  done_cs = 0;
	//  usleep(1000);
	//  for (int n=0;n<norient;n++){ 
	//    done_cs += (int) centsurcs[n]->is_done();
	//  }
	//}
	//combine all into a single map:
	ippiSet_32f_C1R(0.0,ocs_tot_32f,psb_32f,srcsize);
	for(int n=0;n<norient;n++){
	  ippiAdd_32f_C1IR(centsurcs[n]->get_cs_tot_32f(),centsurcs[n]->get_psb_32f(),ocs_tot_32f,psb_32f,srcsize);
	}


	conv_32f_to_8u(ocs_tot_32f,psb_32f,ocs_tot_8u,psb,srcsize);
	Bottle& tmpBot_ocs = outPort_ocs.prepare();
	tmpBot_ocs.clear();
	tmpBot_ocs.add(Value::makeBlob(ocs_tot_8u,psb*srcsize.height));
	tmpBot_ocs.addString("ocs");
	outPort_ocs.write();  // Send it on its way

	//PHASECONG
	conv_32f_to_8u(m,psb_32f,m_8u,psb,srcsize);
	Bottle& tmpBot_m = outPort_m.prepare();
	tmpBot_m.clear();
	tmpBot_m.add(Value::makeBlob(m_8u,psb*srcsize.height));
	tmpBot_m.addString("cor");
	outPort_m.write();  // Send it on its way
	
	conv_32f_to_8u(M,psb_32f,M_8u,psb,srcsize);
	Bottle& tmpBot_M = outPort_M.prepare();
	tmpBot_M.clear();
	tmpBot_M.add(Value::makeBlob(M_8u,psb*srcsize.height));
	tmpBot_M.addString("edg");
	outPort_M.write();  // Send it on its way
	
	//PHASE SYMMETRY:
	conv_32f_to_8u(phaseSym,psb_32f,phaseSym_8u,psb,srcsize);
	Bottle& tmpBot_phaseSym = outPort_phaseSym.prepare();
	tmpBot_phaseSym.clear();
	tmpBot_phaseSym.add(Value::makeBlob(phaseSym_8u,psb*srcsize.height));
	tmpBot_phaseSym.addString("sym");
	outPort_phaseSym.write();  // Send it on its way
 

	//printf("postproc stop!\n");		


      }
      else{
	printf("No Input\n");
	usleep(5000); //dont blow out port
      }
      
      TSTOP	
   }

      
  //never here..
  TEND

}


