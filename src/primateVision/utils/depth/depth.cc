/*
 * Copyright (C) 2003-2009 Andrew Dankers. All rights reserved.
 * 
 */

#include "depth.h"

iCub::contrib::primateVision::Depth::Depth(IppiSize fsize_,
	     int filterSize_,
	     int filterCap_,
	     int windowSize_,
	     int minDisparity_,
	     int numDisparities_,
	     int threshold_,
	     int uniqueness_
	     )
{

  fsize.width = fsize_.width;
  fsize.height = fsize_.height;
  numDisparities = numDisparities_;

  state=cvCreateStereoBMState( CV_STEREO_BM_BASIC, 15 );
  assert(state !=0);
  state->preFilterSize       = filterSize_;
  state->preFilterCap        = filterCap_;
  state->SADWindowSize       = windowSize_;
  state->minDisparity        = minDisparity_;
  state->numberOfDisparities = numDisparities_;
  state->textureThreshold    = threshold_;
  state->uniquenessRatio     = uniqueness_;



  imgLeft  = cvCreateImage(cvSize(fsize.width,fsize.height),IPL_DEPTH_8U,1);
  imgRight = cvCreateImage(cvSize(fsize.width,fsize.height),IPL_DEPTH_8U,1);

  disp     = cvCreateMat(imgLeft->height,imgLeft->width,CV_16S);
  vdisp    = cvCreateMat(imgLeft->height,imgLeft->width,CV_8U);
  disp_ret = ippiMalloc_8u_C1(fsize.width,fsize.height,&psb_o);

}

iCub::contrib::primateVision::Depth::~Depth()
{

}


void iCub::contrib::primateVision::Depth::calc_disp()
{
  printf("calculating disparity0\n");
  cvFindStereoCorrespondenceBM(imgLeft,imgRight,disp,state );
  printf("calculating disparity1\n");
  cvNormalize(disp,vdisp,0,numDisparities,CV_MINMAX);

}


void iCub::contrib::primateVision::Depth::proc(Ipp8u*im_l_,Ipp8u*im_r_, int psb_i_){
   //copy data into opencv:
  ippiCopy_8u_C1R(im_l_,psb_i_,(Ipp8u*)imgLeft->imageData,imgLeft->width,fsize);
  ippiCopy_8u_C1R(im_r_,psb_i_,(Ipp8u*)imgRight->imageData,imgRight->width,fsize);
printf("calculating disparity\n");
  //calculate disparity:
  calc_disp();
printf("finished calculating disparity\n");
  //copy result back to ippi:
  uchar* pData;
  cvGetRawData(vdisp,(uchar**)&pData);
  ippiCopy_8u_C1R((Ipp8u*)pData,imgLeft->width,disp_ret,psb_o,fsize);


}

