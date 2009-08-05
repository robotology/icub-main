using namespace std;

#include <iostream>
#include <stdlib.h>
#include "opencv/cv.h"
#include "opencv/cvaux.h"
#include "public.h"
#include "distortionCorrector.h"
#include "Camera.h"

DistortionCorrector::DistortionCorrector()
{
 // //  distortionMap = NULL;
//   chessboard[0]=5; //wdith
//   chessboard[1]=7; // height
//   chessboard[2]=2; // size of square
//   undistortion_ready= 0;
//   grabber = NULL;
//   //  intrinsicParam = NULL;
//   strcpy(paramfile,"params1");
//   tmp_image= NULL;
//   calib = NULL;
//   distMap=NULL;
  Init();
}

DistortionCorrector::DistortionCorrector(char *filename)
{
//   chessboard[0]=5; //wdith
//   chessboard[1]=7; // height
//   chessboard[2]=2; // size of square
//   // intrinsicParam = NULL;
//   tmp_image= NULL;
  Init(); 
 strcpy(paramfile,filename);
//   calib=NULL;
//   mapx=NULL;
//   mapy=NULL;
//   distMap=NULL;

}

void DistortionCorrector::Init()
{
  chessboard[0]=7; //wdith
  chessboard[1]=9; // height
  chessboard[2]=2.5; // size of square
  undistortion_ready= 0;
  grabber = NULL;
  strcpy(paramfile,"default_params");
  tmp_image= NULL;
  calib = NULL;
  distMap=NULL; 
}

DistortionCorrector::~DistortionCorrector()
{
 //  if(distortionMap){
//     cvReleaseImage(distortionMap);
//   }
  if(calib){
    delete calib;
  }
  if(tmp_image){
    cvReleaseImage(&tmp_image);
  }
}

void DistortionCorrector::SetGrabber(FrameGrabber *grab){
  grabber = grab;  
//   IplImage *tmpframe;
//   cvNamedWindow("grabber",CV_WINDOW_AUTOSIZE);
//   grab->GrabFrame(&tmpframe,0);  
//   cvShowImage("grabber",tmpframe);
}

void DistortionCorrector:: SetChessboardParams(int width_small,int height_big, 
					      float square_size){
    chessboard[0] = (double) width_small;
    chessboard[1] = (double)height_big;
    chessboard[2] = (double)square_size;
}

int DistortionCorrector::CalibrateCamera(int nb_frames)
{
  IplImage *tmpframe;
  int frame_cnt = 0;

#ifdef NEW_VERSION 
  CvSize s = grabber->GetSize();
#else
  CvSize *s = grabber->GetSize(); 
#endif
  cvNamedWindow("calibration window",0);
#ifdef NEW_VERSION
  cvResizeWindow("calibration window",s.width,s.height);
#else
  cvResizeWindow("calibration window",s->width,s->height);
#endif
		//CV_WINDOW_AUTOSIZE);
 //  while(1){
//     if(grabber){
//       grabber->GrabFrame(&tmpframe,0);
//       cvShowImage("calibration window",tmpframe);
//       cvWaitKey(10);
//     }
//   }

  if(!calib){
    calib = new CvCalibFilter();
  }
  //calib->Stop();
  calib->SetEtalon(CV_CALIB_ETALON_CHESSBOARD, chessboard);
  calib->SetFrames(nb_frames);
  cout<<chessboard[0]<<" "<<chessboard[1]<<" "<<chessboard[2]<<endl;
  while(1){
    if('q'== cvWaitKey(5)){
      cout<<" quitting calibration"<<endl;
      cvDestroyWindow("calibration window");
      return 0;
    }
    grabber->GrabFrame(&tmpframe,0);
    //cvShowImage("calibration window",tmpframe);
    if(calib->FindEtalon(&tmpframe)){
      frame_cnt++;
      calib->Push();
      //if(calib->GetFrameCount()<nb_frames){
      if(frame_cnt<nb_frames){
	cout <<"frame "<<calib->GetFrameCount()<<" of "<<nb_frames << " press 'n' key for next one" <<endl;
	while('n'!=cvWaitKey(10)){
	  grabber->GrabFrame(&tmpframe,0);
	  cvShowImage("calibration window",tmpframe);
	}
      }
      else{
	break;
      }
    }
    else{
      calib->DrawPoints(&tmpframe);
    }
    cvShowImage("calibration window",tmpframe);
  }
  
 //  CvCamera* params = calib->getCameraParams();
//   cvUnDistortInit(tmpframe, DistMap,
//                   params->matrix,
//                   params->distortion,
//                   1);
  cout<<"Calibration done"<<endl;
  cvDestroyWindow("calibration window");
  return InitUndistortion(tmpframe);
}

int  DistortionCorrector::IsCalibrated()
{
  if(!calib){
    return 0;
  }
  else{
    return calib->IsCalibrated();
  }
}

void DistortionCorrector::Apply(IplImage *img)
{
  if(!calib){
   if(! LoadCameraParams(paramfile)){
     if(!CalibrateCamera(10)){
       cout<<"calibration failed"<<endl;
       return;
     }
     else{
       if(SaveCameraParams(paramfile)){
	 cout<<"calibration saved in "<<paramfile<<endl;
       }
       else{
	 cout<<"cannot calibration parameters in "<<paramfile<<endl;
       }
     }
   }
  }
  if(!undistortion_ready){
    InitUndistortion(img);
  }
  if(!tmp_image){
    tmp_image = cvCloneImage(img);
  }

//  IplImage *tmp=cvCloneImage(img);
//   calib->Undistort(&img,&tmp_image);
//   cvCopy(tmp_image,img);
  cvUnDistort(img,tmp_image ,distMap, 1 );
//  cvRemap(img,tmp_image,mapx,mapy,1);
//  tmp_image = ((CameraImage *) grabber)->GetUnDistortImage();
  cvCopy(tmp_image,img);
//  img = grabber->Undistort()
//   cvSub(img,tmp,tmp_image);
//   cvShowImage("diff",tmp_image);

}

int DistortionCorrector::SaveCameraParams( const char* filename )
{
  if(calib){
    if(filename){
      return (int) calib->SaveCameraParams(filename);
    }
    else{
      return  (int) calib->SaveCameraParams(paramfile);
    }
  }
  else{
    return 0;
  }
}

int DistortionCorrector::LoadCameraParams( const char* filename )
{
  int ret;
  if(!calib){
    calib = new CvCalibFilter();
  }
  calib->LoadCameraParams(filename);
  undistortion_ready =0;
  return IsCalibrated();
}
 
IplImage * DistortionCorrector::Process(IplImage *image){
  return NULL;
}
 
void DistortionCorrector::Config(IplImage *image, CvRect selection){return;}


int  DistortionCorrector::InitUndistortion(IplImage *img){
  if(IsCalibrated()){
    const CvCamera* params;
      if((params = calib->GetCameraParams(0))){
// #ifdef NEXT_VERSION  
//   if(mapx==NULL){
//     mapx   = cvCreateImage( cvSize(1*tmpframe->width,1*tmpframe->height), IPL_DEPTH_32S, 1 );
//     }
//     if(mapy==NULL){
//     mapy   = cvCreateImage( cvSize(1*tmpframe->width,1*tmpframe->height), IPL_DEPTH_32S, 1 );
//     }
//     cvInitUndistortMap(params->matrix,params->distortion,mapx.mapy);
// #else
//      SetCameraParams(params);
//     else{
//       cout<<"no grabber"<<endl;
//     }
// #endif
	if(distMap==NULL){
	  distMap = cvCreateImage( cvSize(3*img->width,3*img->height), IPL_DEPTH_32S, 1 );
	}
	cvUnDistortInit(img, distMap,params->matrix,params->distortion,1);
	undistortion_ready= 1;
	return 1;
      }
      else{
	cout<<"no param"<<endl;
      }
  }
  else{
    cout<<"not calibrated"<<endl;
  }
  return 0;
}
           
const CvCamera* DistortionCorrector::GetCameraParams() const
{
  if(calib){
    return calib->GetCameraParams();
  }
  else{
    return NULL;
  }
}
