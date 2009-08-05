#include "public.h"
#include "ipfilters.h"
#include "visionServer.h"
#include "yarpFrameGrabber.h"
#include "visualRotationModule.h"



bool VisualRotationModule::open(Searchable& config){
  Value& nbpoints = config.find("nb_points");
  if(!nbpoints.isNull()){
    Init(nbpoints.asInt());
  }
  else{
    Init();
  } 
  char tmp[30];
  sprintf(tmp,"visualRotation:o");
  port.SetPortName(getName(tmp).c_str());
  port.Start();
  return true;
}

void VisualRotationModule::Init(int nb_points){
  InitGrabber();
  InitInterface();
  //roughly measured on icub
  qFlow = new QuaternionFlow(Vec2(DEG2RAD(32),DEG2RAD(24)),nb_points);
  t0=0;
}


bool  VisualRotationModule::InitGrabber(){
  char name [80];
  grabber = new YarpFrameGrabber();
  sprintf(name,"%s:i",getName("image").c_str());
  grabber->Init(name);
  return Network::connect("/icub/cam/right",name);
}

void VisualRotationModule::sendOutput(double t1){
  Quat rot =  qFlow->GetRotation();
  //  if(rot.x*rot.x+rot.y*rot.y+rot.z*rot.z<0.00000001)return; 
  Bottle& b=port.prepare();
  b.clear();
  //inverting because we are looking for the perceived motion
  b.addDouble(-rot.x);
  b.addDouble(-rot.y);
  b.addDouble(-rot.z);
  b.addDouble(t0);
  b.addDouble(t1);
  port.write();
}

int VisualRotationModule::Loop(){
  grabber->GrabFrame(&frame);
  double t1 = grabber->GetTime();
  qFlow->Apply(frame);
  qFlow->Draw(frame);
  cvShowImage("image", frame);
  cvResizeWindow("image",frame->width, frame->height);
  //  printf("%f %f\n",t0,t1);
  sendOutput(t1);
  t0 = t1; 
  int key = cvWaitKey(100);
  return ExecuteCommand(key)>0;
}

int VisualRotationModule::ExecuteCommand(int key){ 
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

int main(int argc,char *argv[]){
  VisualRotationModule mod;
  mod.openFromCommand(argc,argv,true);
  mod.runModule();
}


#ifdef STEREO_QUATERNION_FLOW


void VisualRotation::InitGrabbers(){
  grabber_left = new ImgGrabber("./test/cedric_noeye/left00000.png");
  grabber_right = new ImgGrabber("./test/cedric_noeye/right00000.png");
}

void VisualRotation::InitVision(const char *paramdir, double chessboard[3]){
  InitGrabbers();
  char prefix[80];
  char paramfile[100];
   if(paramdir){
      sprintf(prefix,"params/%s",paramdir);
    }
   else{
      sprintf(prefix,"params");
   }
   sprintf(paramfile,"%s/params0",prefix);
   right_distortion = new DistortionCorrector(paramfile);
   sprintf(paramfile,"%s/params1",prefix);
   left_distortion = new DistortionCorrector(paramfile);

   right_distortion->SetGrabber(grabber_right);
   left_distortion->SetGrabber(grabber_left);
   
   right_distortion->SetChessboardParams(chessboard);
   left_distortion->SetChessboardParams(chessboard);

   StereoQuaternionFlow *flow = new StereoQuaternionFlow(2000);
}

	//FrameGrabber *grabber = new CvcamGrabber("cfg", cvSize(320,240));
	//FrameGrabber *grabber = new CvCaptureGrabber();
	

	//FrameGrabber *grabber = new AviGrabber("360degreecamerapanat.avi");
// 	FrameGrabber *grabber_left = new ImgGrabber("./test/calib1/left_calib00000.png"); 
// 	FrameGrabber *grabber_right = new ImgGrabber("./test/calib1/right_calib00000.png");

	


void VisualRotation::InitInterface(){	
  CvSize res = grabber_left->GetSize();	
  IplImage *frame_left = cvCreateImage(res, IPL_DEPTH_8U, 3);
  IplImage *frame_right = cvCreateImage(res, IPL_DEPTH_8U, 3);	
  cvNamedWindow("graph",0);
  cvNamedWindow("image right",0);
  cvNamedWindow("image left",0);
}

bool VisualTotation::Loop(){
  grabber_left->GrabFrame(&frame_left);
  grabber_right->GrabFrame(&frame_right);
  distortion_left->Apply(&frame_left);
  distortion_right->Apply(&frame_right);
 
  flow->SetFrames(frame_right,frame_left);
  flow->Apply();
  //	  flow->Draw(NULL);
  
  cvShowImage("image left", frame_left);
  cvShowImage("image right", frame_right);
  
  cvResizeWindow("image left",frame_left->width, frame_left->height);
  cvResizeWindow("image right",frame_right->width, frame_right->height);

  int key = cvWaitKey(10);
  return (bool) ExecuteCommand(key);
}

int VisualRotation::ExecuteCommand(int key){ 
  if(key==27) return 0;;
  switch(key){
  case ' ':
    break;
  case 'p':
    cvWaitKey();
    break;
  }
  return 1;
}


void VisualRotation::Free(){
  grabber_left->Kill();
  grabber_right->Kill();
  
  delete flow;
  delete left_distortion;
  delete right_distortion;
  
  
  IMKILL(frame_left);
  IMKILL(frame_right);
  IMKILL(image);
  cvDestroyAllWindows();
}


void VisualRotation::Run(){
  InitGrabbers();
  InitVision();
  InitInterface();
  while(Loop()){};
  Free();
}


#endif
