/**
*
@ingroup icub_module
\defgroup icub_findTorsos findTorsos

This module finds torsos in both cameras and sends out the torso positions


\section lib_sec Libraries
YARP, OPEN_CV
 
\section portsa_sec Ports Accessed
- /findTorsos/torsos/left/in
- /findTorsos/torsos/right/in

\section portsc_sec Ports Created
- /findTorsos/torsos/left/out
- /findTorsos/torsos/right/out

\section in_files_sec Input Data Files
/fullpathname/haarCascadeFileTorso.xml


\section tested_os_sec Tested OS
Linux 

\section example_sec Example Instantiation of the Module
findTorsos /usr/local/src/robot/iCub/src/findFaces/haarcascade_UpperBodyNew.xml

\author Zenon Mathews

Copyright (C) 2010 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at findTorsos/findTorsos.cpp.
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
  if (argc < 2){
    printf("haar file not provided \n");
    exit(0);
  }
 
  // for IQR
  BufferedPort<Vector> torsos_left_outPort;
  torsos_left_outPort.open("/findtorsos/torsos/left/out");
  BufferedPort<Vector> torsos_right_outPort;
  torsos_right_outPort.open("/findtorsos/torsos/right/out");

 

  TorsoDetector torsoDetector;
  
  bool okt = torsoDetector.init(argv[1]);

  if (okt){
    printf("torso detector initialized");
  }
  else{
    printf("\n\n\n not torso detector not correctly initialized!!\n\n\n");
  }

  bool foundlefttorso = false;
  bool foundrighttorso = false;

  

  BufferedPort<ImageOf<PixelRgb> > imageLeftPort; // port for reading in images
  BufferedPort<ImageOf<PixelRgb> > imageRightPort; // port for reading in image
  imageLeftPort.open("/findtorsos/image/left/in"); // give a name for port
  imageRightPort.open("/findtorsos/image/right/in"); // give a name for port
  

  ImageOf<PixelRgb> *imageLeft = imageLeftPort.read(); // read an image
  ImageOf<PixelRgb> *imageRight = imageRightPort.read(); // read an image
  IplImage *cvLeftImage = cvCreateImage(cvSize(imageLeft->width(), imageLeft->height()),IPL_DEPTH_8U, 3);
  IplImage *cvRightImage = cvCreateImage(cvSize(imageRight->width(), imageRight->height()),IPL_DEPTH_8U, 3);
    
  //cvNamedWindow("left-torsos", 1);
  //cvNamedWindow("right-torsos", 1);

  while(1){ // repeat forever
    
    
    imageLeft = imageLeftPort.read(); // read an image
    imageRight = imageRightPort.read(); // read an image
    if (imageLeft  != NULL){
      
      cvCvtColor((IplImage*)imageLeft->getIplImage(), cvLeftImage, CV_RGB2BGR);

      // do whatever in opencv with the image
      CvSeq* torsosleft = torsoDetector.detect(cvLeftImage);
           
      // for IQR
     
      Vector& leftTorsos = torsos_left_outPort.prepare();
      int countIQRt = 0;
      if (torsosleft->total > 0){
	leftTorsos.resize((torsosleft->total) * 2);
      
      }
      
     
      // draw circle on found left torsos
      float scaleT = 1.0;
      for(int i = 0; i < torsosleft->total; i++){
	CvRect* r = (CvRect*)cvGetSeqElem(torsosleft, i);
	CvPoint center;
	int radius;
	
	center.x = cvRound((r->x + r->width * 0.5) * scaleT);
	center.y = cvRound((r->y + r->height * 0.5) * scaleT);
	//radius = cvRound((r->width + r->height)*0.25*scaleT);
	//cvCircle(cvLeftImage, center, radius, cvScalar(0.5));


	// to send to IQR
	leftTorsos[countIQRt] = center.x;
	countIQRt++;
	leftTorsos[countIQRt] = center.y;
	countIQRt++;

	foundlefttorso = true;

      }



       // send to IQR
       if (foundlefttorso){			
	
	 torsos_left_outPort.write();
	 
      }

      // show the image
      //cvShowImage("left-torsos", cvLeftImage);
      //cvWaitKey(10);
    }

    //
    // ----------------------- right image ---------------------------//
    //
    //
    
    if (imageRight  != NULL){
       cvCvtColor((IplImage*)imageRight->getIplImage(), cvRightImage, CV_RGB2BGR);

      
      CvSeq* torsosright = torsoDetector.detect(cvRightImage);

      // simulaneously send right faces to IQR
     
       Vector& rightTorsos = torsos_right_outPort.prepare();
      int countIQRrt = 0;
      if (torsosright->total > 0){
	rightTorsos.resize(torsosright->total * 2);
      
      }

     
      // draw circle on torsos
      float scalef = 1.0;
      for(int i = 0; i < torsosright->total; i++){
	CvRect* r = (CvRect*)cvGetSeqElem(torsosright, i);
	CvPoint center;
	int radius;
	center.x = cvRound((r->x + r->width * 0.5) * scalef);
	center.y = cvRound((r->y + r->height * 0.5) * scalef);
	//radius = cvRound((r->width + r->height)*0.25*scalef);
	//cvCircle(cvRightImage, center, radius, cvScalar(0.5));

	// to send to IQR
	rightTorsos[countIQRrt] = center.x;
	countIQRrt++;
	rightTorsos[countIQRrt] = center.y;
	countIQRrt++;

	foundrighttorso = true;
      }
      

      // show the image
      //cvShowImage("right-torsos", cvRightImage);
      //cvWaitKey(10);
    }


   
 
  // send to IQR
   if (foundrighttorso){		  
     torsos_right_outPort.write();
    
    }
   
      
    foundlefttorso = false;
    foundrighttorso = false;

 
    
  }
  //cvDestroyWindow("left");
  //cvDestroyWindow("right");
  return 0;
  
}
