#include "public.h"
#include "ipfilters.h"
#include "visualRotation.h"



bool VisualRotation::InitGrabber(){
    //grabber = new ImgGrabber("./test/cedric_noeye/left00000.png");
  return false;
}

void VisualRotation::Init(int nb_points){
  InitGrabber();
  InitInterface();
  qFlow = new QuaternionFlow(Vec2(DEG2RAD(30),DEG2RAD(30)),nb_points);
  t0=0;
}

	//FrameGrabber *grabber = new CvcamGrabber("cfg", cvSize(320,240));
	//FrameGrabber *grabber = new CvCaptureGrabber();
	

	//FrameGrabber *grabber = new AviGrabber("360degreecamerapanat.avi");
// 	FrameGrabber *grabber_left = new ImgGrabber("./test/calib1/left_calib00000.png"); 
// 	FrameGrabber *grabber_right = new ImgGrabber("./test/calib1/right_calib00000.png");

	


void VisualRotation::InitInterface(){	
  CvSize res = grabber->GetSize();	
  frame = cvCreateImage(res, IPL_DEPTH_8U, 3);
  cvNamedWindow("graph",0);
  cvNamedWindow("image",0);
}

int VisualRotation::Loop(){
  grabber->GrabFrame(&frame);
  qFlow->Apply(frame);
  qFlow->Draw(frame);
  cvShowImage("image", frame);
  cvResizeWindow("image",frame->width, frame->height);
  int key = cvWaitKey(10);
  return ExecuteCommand(key);
}

int VisualRotation::ExecuteCommand(int key){ 
  if(key==27) return 0;;
  switch(key){
  case ' ':
    break;
  case 'p':
    cvWaitKey();
    break;
  case 'q':
      printf("quiting\n");
      return 0;
  }
  return 1;
}


void VisualRotation::Free(){
  grabber->Kill();
  delete qFlow;
  IMKILL(frame);
  cvDestroyAllWindows();
}


void VisualRotation::Run(){
  Init();
  while(Loop()){};
  Free();
}
