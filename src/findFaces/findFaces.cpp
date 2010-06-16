/**
*
@ingroup icub_module
\defgroup icub_findFaces findFaces

This module find faces in both cameras and sends out the face positions


\section lib_sec Libraries
YARP, OPEN_CV
 
\section portsa_sec Ports Accessed
- /findfaces/faces/left/in
- /findfaces/faces/right/in

\section portsc_sec Ports Created
- /findfaces/faces/left/out
- /findfaces/faces/right/out

\section in_files_sec Input Data Files
/fullpathname/haarCascadeFileFace.xml


\section tested_os_sec Tested OS
Linux 

\section example_sec Example Instantiation of the Module
findFaces /usr/local/src/robot/iCub/src/findFaces/haarcascade_frontalface_default.xml

\author Zenon Mathews

Copyright (C) 2010 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at findFaces/findFaces.cpp.
**/





#include <stdio.h>

// Get all OS and signal processing YARP classes

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

#include "torsodet.h"

#include <vector>
#include <math.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

using namespace std;


int main(int argc, char **argv){

  Network yarp; // set up yarp
  // for IQR
  BufferedPort<Vector> faces_left_outPort;
  faces_left_outPort.open("/findfaces/faces/left/out");
  BufferedPort<Vector> faces_right_outPort;
  faces_right_outPort.open("/findfaces/faces/right/out");
 

  TorsoDetector faceDetector;
  if (argc < 2){
    printf("-------->haar cascade file full path not supplied\n");
    exit(0);
  }
  
  bool okf = faceDetector.init(argv[1]);
  
  
  if (okf){
    printf("face detector initialized");
  }
  else{
    printf("\n\n\n not face detector not correctly initialized!!\n\n\n");
    exit(0);
  }

  bool foundleftface = false;
  bool foundrightface = false;

 

  BufferedPort<ImageOf<PixelRgb> > imageLeftPort; // port for reading in images
  BufferedPort<ImageOf<PixelRgb> > imageRightPort; // port for reading in image
  imageLeftPort.open("/findfaces/image/left/in"); // give a name for port
  imageRightPort.open("/findfaces/image/right/in"); // give a name for port
  
  ImageOf<PixelRgb> *imageLeft = imageLeftPort.read(); // read an image
  ImageOf<PixelRgb> *imageRight = imageRightPort.read(); // read an image
  IplImage *cvLeftImage = cvCreateImage(cvSize(imageLeft->width(), imageLeft->height()),IPL_DEPTH_8U, 3);
  IplImage *cvRightImage = cvCreateImage(cvSize(imageRight->width(), imageRight->height()),IPL_DEPTH_8U, 3);
    
  //cvNamedWindow("left-faces", 1);
  //cvNamedWindow("right-faces", 1);

  while(1){ // repeat forever
    
    
    imageLeft = imageLeftPort.read(); // read an image
    imageRight = imageRightPort.read(); // read an image
    if (imageLeft  != NULL){
      
      cvCvtColor((IplImage*)imageLeft->getIplImage(), cvLeftImage, CV_RGB2BGR);

      // do whatever in opencv with the image
      CvSeq* facesleft = faceDetector.detect(cvLeftImage);
      
      // for IQR
      Vector& leftFaces = faces_left_outPort.prepare();
      int countIQR = 0;
      if (facesleft->total > 0){
	leftFaces.resize((facesleft->total) * 2);
      
      }
      
      
      // draw circle on found left faces
      float scale = 1.0;
      for(int i = 0; i < facesleft->total; i++){
	CvRect* r = (CvRect*)cvGetSeqElem(facesleft, i);
	CvPoint center;
	int radius;
	
	center.x = cvRound((r->x + r->width * 0.5) * scale);
	center.y = cvRound((r->y + r->height * 0.5) * scale);
	radius = cvRound((r->width + r->height)*0.25*scale);
	cvCircle(cvLeftImage, center, radius, cvScalar(0.5));

	// to send to IQR
	leftFaces[countIQR] = center.x;
	countIQR++;
	leftFaces[countIQR] = center.y;
	countIQR++;

	foundleftface = true;

      }

     

      // send to IQR
       if (foundleftface){			
	
	 faces_left_outPort.write();
	 
      }
      
      // show the image
      //cvShowImage("left-faces", cvLeftImage);
      //cvWaitKey(10);
    }

    //
    // ----------------------- right image ---------------------------//
    //
    //
    
    if (imageRight  != NULL){
       cvCvtColor((IplImage*)imageRight->getIplImage(), cvRightImage, CV_RGB2BGR);

      CvSeq* facesright = faceDetector.detect(cvRightImage);

      // simulaneously send right faces to IQR
      Vector& rightFaces = faces_right_outPort.prepare();
      int countIQRr = 0;
      if (facesright->total > 0){
	rightFaces.resize(facesright->total * 2);
      
      }
     
      // draw circle on faces
      float scale = 1.0;
      for(int i = 0; i < facesright->total; i++){
	CvRect* r = (CvRect*)cvGetSeqElem(facesright, i);
	CvPoint center;
	int radius;
	center.x = cvRound((r->x + r->width * 0.5) * scale);
	center.y = cvRound((r->y + r->height * 0.5) * scale);
	radius = cvRound((r->width + r->height)*0.25*scale);
	cvCircle(cvRightImage, center, radius, cvScalar(0.5));

	// to send to IQR
	rightFaces[countIQRr] = center.x;
	countIQRr++;
	rightFaces[countIQRr] = center.y;
	countIQRr++;

	
	foundrightface = true;
      }

     
      // show the image
      //cvShowImage("right-faces", cvRightImage);
      //cvWaitKey(10);
    }


   
    // send to IQR
   if (foundrightface){		  
     faces_right_outPort.write();
    
    }
 
      
    foundleftface = false;
    foundrightface = false;

 
    
  }
  cvDestroyWindow("left");
  cvDestroyWindow("right");
  return 0;
  
}
