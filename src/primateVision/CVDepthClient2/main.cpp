/**
 * @ingroup icub_primatevision_recserver
 * \defgroup icub_primatevision_recserver_cvdepthclient2 CVDepthClient2
 *
 *This endclient module CVDepthClient2 performs disparity mapping of epipolar rectified images obtained from the RecServer using the >libCV1.1pre BMDepth method. Primarily for example/testing.
 *
 * This module should be called in the following way:\n \b CVDepthClient2 \b\n
 *\n
 * Ports accessed:\n
 * <ul>
 * <li> /recserver/output/left_ye
 * <li> /recserver/output/right_ye
 
 * Input ports:\n
 * <ul>
 * <li> /cvdc2/input/left_ye
 * <li> /cvdc2/input/right_ye
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> NONE! It's an endClient!!
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/CVDepthClient2/main.cc 
 * 
 */


/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */




#include <stdio.h>
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <ace/config.h>
#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <fstream>
#include <iostream>
#include <recio.h>
#include <ipp.h>
#include <math.h>
#include <time.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;
using namespace yarp::os;

using namespace iCub::contrib::primateVision;


IplImage* imgLeft = 0;
IplImage* imgRight = 0;
CvMat* disp;
CvMat* vdisp,*vdisp2,*lowtoHigh,*cvTh;
CvMat* disp2,*vdisp3,*vdisp4;
IplImage* imgRight2;
IplImage* imgLeft2;
CvStereoBMState* state;
int filterSize,filterCap,windowSize, minDisparity,numDisparities, threshold, uniqueness, numD;






void upPyramidInformation(CvMat *lowLayer,CvMat *layer, int minD){
  cvPyrUp( lowLayer, lowtoHigh );
  cvMinS( layer, (minD+1)*16, cvTh);
  cvMaxS( cvTh, minD*16, cvTh);
  cvNormalize( cvTh,cvTh, 0,1 , CV_MINMAX );
  cvMul(layer, cvTh,layer,1);
  cvNormalize( layer,vdisp3, 0,255 , CV_MINMAX );
  cvSubRS( cvTh, cvScalar(1), cvTh);
  cvNormalize( cvTh,vdisp4, 0,255 , CV_MINMAX );
  cvNormalize( cvTh,cvTh, 0,1 , CV_MINMAX );
  cvMul( lowtoHigh, cvTh, lowtoHigh,2);
  cvAdd(lowtoHigh,layer,layer);
}


void on_trackbar(int h)
{
  //5..255, odd
  filterSize = (filterSize/2) * 2 + 1;
  if (filterSize<5){filterSize=5;}

  //1..63
  if(filterCap<1){filterCap=1;}
  if(filterCap>63){filterCap=63;}
  state->preFilterCap=filterCap;

  //5..240, odd
  windowSize = (windowSize/2) * 2 + 1;
  if (windowSize<5){windowSize=5;}
  if (windowSize>imgLeft->height){windowSize=imgLeft->height;}
  if (windowSize>255){windowSize=255;}


  //%able by 16!
  numDisparities = (numDisparities/16) * 16;
  if (numDisparities<16){numDisparities = 16;}


  state->preFilterSize=filterSize;
  state->SADWindowSize=windowSize;
  state->minDisparity=-minDisparity*2;
  state->numberOfDisparities=numDisparities*2;
  state->textureThreshold=threshold;
  state->uniquenessRatio=uniqueness;



  printf("1:  fs:%d fc:%d ws:%d md:%d nd:%d th:%d un:%d\n", 
	 state->preFilterSize,
	 state->preFilterCap,
	 state->SADWindowSize,
	 state->minDisparity,
	 state->numberOfDisparities,
	 state->textureThreshold,
	 state->uniquenessRatio);

  cvFindStereoCorrespondenceBM( imgLeft, imgRight, disp, state );
  
  
  cvPyrDown(imgLeft, imgLeft2);
  cvPyrDown(imgRight, imgRight2);

  state->minDisparity=-minDisparity;
  state->numberOfDisparities=numDisparities;
  

  printf("2:  fs:%d fc:%d ws:%d md:%d nd:%d th:%d un:%d\n", 
	 state->preFilterSize,
	 state->preFilterCap,
	 state->SADWindowSize,
	 state->minDisparity,
	 state->numberOfDisparities,
	 state->textureThreshold,
	 state->uniquenessRatio);

  cvFindStereoCorrespondenceBM(imgLeft2,imgRight2,disp2,state);
    
  cvNormalize( disp,vdisp, 0,255 , CV_MINMAX );
  cvNormalize( disp2,vdisp2, 0,255 , CV_MINMAX );
  upPyramidInformation(disp2,disp, -minDisparity);
  cvNormalize( disp,vdisp, 0,255 , CV_MINMAX );

  cvShowImage("Mix",vdisp);
  
}





int main(int argc, char *argv[]){

  
  Network::init();





 Port inPort_s;
  inPort_s.open("/cvdepth/input/serv_params");     // Give it a name on the network.
  Network::connect("/cvdepth/input/serv_params", "/recserver/output/serv_params");
  Network::connect("/recserver/output/serv_params", "/cvdepth/input/serv_params");
  BinPortable<RecServerParams> server_response;
  Bottle empty;
  inPort_s.write(empty,server_response);
  RecServerParams rsp = server_response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl;
  
  RecResultParams* rec_res_l;
  RecResultParams* rec_res_r;
  
  
  BufferedPort<Bottle> inPort_ly;    
  inPort_ly.open("/cvdepth/input/rec_ly"); 
  Network::connect("/recserver/output/left_ye" , "/cvdepth/input/rec_ly");
  Bottle *inBot_ly;
  Ipp8u* rec_im_ly;

  BufferedPort<Bottle> inPort_ry;    
  inPort_ry.open("/cvdepth/input/rec_ry"); 
  Network::connect("/recserver/output/right_ye" , "/cvdepth/input/rec_ry");
  Bottle *inBot_ry;
  Ipp8u* rec_im_ry;

  int width = rsp.width;
  int height = rsp.height;
  int psb = rsp.psb;

  IppiSize srcsize, tmpsize;
  srcsize.width = width;
  srcsize.height = height;
  int psb_tmp;
  Ipp8u* rec_im_ly_tmp = ippiMalloc_8u_C1(width, height, &psb_tmp);
  Ipp8u* rec_im_ry_tmp = ippiMalloc_8u_C1(width, height, &psb_tmp);
  
  int ixl,iyl,ixr,iyr,hd,px,py;
  
  




  filterSize=11;
  filterCap=10;
  windowSize=15;
  minDisparity=16;
  numDisparities=32;
  threshold=10;
  uniqueness=20;


  state=cvCreateStereoBMState( CV_STEREO_BM_BASIC, 15 );
  state->preFilterSize=filterSize;
  state->preFilterCap=filterCap;
  state->SADWindowSize=windowSize;
  state->minDisparity=-minDisparity;
  state->numberOfDisparities=numDisparities;
  state->textureThreshold=threshold;
  state->uniquenessRatio=uniqueness;


  imgLeft   = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 1);
  imgRight  = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 1);
  disp      = cvCreateMat( imgLeft->height,imgLeft->width, CV_16S );
  vdisp     = cvCreateMat( imgLeft->height,imgLeft->width, CV_8U );
  disp2     = cvCreateMat( imgLeft->height/2,imgLeft->width/2, CV_16S );
  vdisp2    = cvCreateMat( imgLeft->height/2,imgLeft->width/2, CV_8U );
  imgRight2 = cvCreateImage(cvSize(imgLeft->width/2, imgLeft->height/2), IPL_DEPTH_8U, 1);
  imgLeft2  = cvCreateImage(cvSize(imgLeft->width/2, imgLeft->height/2), IPL_DEPTH_8U, 1);
  

  lowtoHigh = cvCreateMat( disp->height,disp->width, CV_16S );
  cvTh      = cvCreateMat( disp->height,disp->width, CV_16S );
  vdisp3    = cvCreateMat( disp->height,disp->width, CV_8U );
  vdisp4    = cvCreateMat( disp->height,disp->width, CV_8U );





  
  
  cvNamedWindow( "Right", 0);
  cvMoveWindow("Right", 900, 50);
  cvNamedWindow( "Left", 0);
  cvMoveWindow("Left", 500, 50);
  cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("mainWin", 100, 100);
  cvNamedWindow("Mix", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("Mix", 100, 100);


  cvCreateTrackbar( "FilterSize", "mainWin",  &filterSize, 255,0);
  cvSetTrackbarPos( "FilterSize", "mainWin", 11 );
  cvCreateTrackbar( "FilterCap", "mainWin",  &filterCap, 63,0);
  cvSetTrackbarPos( "FilterCap", "mainWin", 10 );
  cvCreateTrackbar( "SadWindowSize", "mainWin",  &windowSize, 240,0);
  cvSetTrackbarPos( "SadWindowSize", "mainWin", 15);
  cvCreateTrackbar( "minDisparity", "mainWin",  &minDisparity, 240, 0);    
  cvSetTrackbarPos( "minDisparity", "mainWin", 16);
  cvCreateTrackbar( "numDisparities", "mainWin",  &numDisparities, 256,0);
  cvSetTrackbarPos( "numDisparities", "mainWin", 32);
  cvCreateTrackbar( "Threshold", "mainWin",  &threshold, 100,0);
  cvSetTrackbarPos( "Threshold", "mainWin", 10);
  cvCreateTrackbar( "Uniqueness", "mainWin",  &uniqueness, 100,0);
  cvSetTrackbarPos( "Uniqueness", "mainWin", 20);

  
 




 
  while(1) {
    

    //get left image:
    inBot_ly = inPort_ly.read();//false
    if (inBot_ly!=NULL){
      rec_im_ly = (Ipp8u*) inBot_ly->get(0).asBlob();
      rec_res_l = (RecResultParams*) inBot_ly->get(1).asBlob();
    }    
    
    //get right image:
    inBot_ry = inPort_ry.read(false);
    if (inBot_ry!=NULL){
      rec_im_ry = (Ipp8u*) inBot_ry->get(0).asBlob();
      rec_res_r = (RecResultParams*) inBot_ry->get(1).asBlob();
    }
    


    if (inBot_ly!=NULL && inBot_ry!=NULL){
      
      //printf("GOT IMAGES\n");

      ixl = rec_res_l->lx;
      iyl = rec_res_l->ly;
      ixr = rec_res_r->rx;
      iyr = rec_res_r->ry;
      
      //figure out stereo params:
      hd = ixl-ixr + 1;  //horiz displacement between L & R images
      px = (ixl+ixr)/2 - width/2; //centre of l-r overlap x
      py = (iyr+iyl)/2 - height/2; //centre of l-r overlap y
      
      
      //SHIFT IMAGES TO ALIGN VERTICALLY:
      if (iyl>iyr){//if left higher
	tmpsize.width = srcsize.width;
	tmpsize.height = srcsize.height-(iyl-iyr);
	//copy im_l_dog down by (iyr-iyl):
	ippiCopy_8u_C1R(rec_im_ly,psb,&rec_im_ly_tmp[(iyl-iyr)*psb],psb,tmpsize);
	ippiCopy_8u_C1R(rec_im_ry,psb,rec_im_ry_tmp,psb,srcsize);
      }
      else{//if right lower
	tmpsize.width = srcsize.width;
	tmpsize.height = srcsize.height-(iyr-iyl);
	//copy im_r_dog down by (iyl-iyr):
	ippiCopy_8u_C1R(rec_im_ry,psb,&rec_im_ry_tmp[(iyr-iyl)*psb],psb,tmpsize);
	ippiCopy_8u_C1R(rec_im_ly,psb,rec_im_ly_tmp,psb,srcsize);
      }
      
      
      //copy data into opencv
      ippiCopy_8u_C1R(rec_im_ly_tmp, psb, (Ipp8u*)imgLeft->imageData ,psb,srcsize);
      ippiCopy_8u_C1R(rec_im_ry_tmp, psb, (Ipp8u*)imgRight->imageData ,psb,srcsize);
      
      

      cvShowImage("Left", imgLeft);
      cvShowImage("Right", imgRight);

         
  



      on_trackbar(0);
        
            

      cvWaitKey(10);
      
    }
    else {usleep(50000);}
    
  }
  
  
  


  return 0;
}
