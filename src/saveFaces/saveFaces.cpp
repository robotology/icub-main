/*
 * Copyright (C) 2010 Zenon Mathews
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * 
 * saves faces into a directory
 */


#include <stdio.h>

// Get all OS and signal processing YARP classes

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

#include "torsodet.h"

#include <vector>
#include <math.h>
#include <string>

#include <iostream>
#include <sstream>
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

struct point{
  int x, y;
};

float prevxl, prevxr, prevyl, prevyr;
float xl,xr,yl,yr;
float xx,yy,zz;
int count;
const char* saveImageDir;

void writeImage(int index, IplImage* img, CvPoint center ){

  int xoffset = 70;
  int yoffset = 70;

  //saveFileName = saveImageDir +  QString("logImg%1.jpg").arg(index);
  char tmpFileName[256];
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  
  sprintf(tmpFileName, "logImage%d.jpg", index );
  ss << saveImageDir << "/" <<  tmpFileName;
  string str(ss.str());
  const char* tmpFileNameImg = str.c_str();

  IplImage *tmpImage = cvCreateImage(cvSize(100, 100),IPL_DEPTH_8U, 3);
  cvSetImageROI(img, cvRect(center.x - xoffset, center.y - yoffset, xoffset*2, yoffset * 2));
  cvResize(img, tmpImage);
  cvResetImageROI(img);
  cvSaveImage(tmpFileNameImg, tmpImage);
}

int main(int argc, const char* argv[]){

  saveImageDir = argv[1];
  printf("saveImageDir: %s \n ", saveImageDir);
  vector<point> leftpoints;
  vector<point> rightpoints;

  int  count = 0;

  // sending x1,x2,y1,y2 points to iKinHead
  BufferedPort<Vector> two_d_outPort;
  two_d_outPort.open("/savefaces/two_d_points/out");
  // receiving x,y,z 3d points from iKinHead
 

  // just one face at the moment per camera
  int leftcamfacex = 320/2;
  int leftcamfacey = 240/2; 
  int rightcamfacex = 320/2;
  int rightcamfacey = 240/2; 

  prevxl = leftcamfacex;
  prevyl = leftcamfacey;
  prevxr = rightcamfacex;
  prevyr = rightcamfacey;

  xx = 0.0;
  yy = 0.0;
  zz = 0.0;

  xl = prevxl;
  xr = prevxr;
  yl = prevyl;
  yr = prevyr;

  TorsoDetector torsoDetector;
  //bool ok = torsoDetector.init("/usr/local/src/robot/iCub/src/iCubFindHumans/haarcascade_UpperBodyNew.xml");
   bool ok = torsoDetector.init("/usr/local/src/robot/iCub/src/iCubFindHumans/haarcascade_frontalface_default.xml");
  //bool ok = torsoDetector.init("/usr/local/src/robot/iCub/src/iCubFindHumans/haarcascade_UpperBodyNew.xml");

  if (ok){
    printf("torso detector initialized");
  }
  else{
    printf("\n\n\n not torso detector not correctly initialized!!\n\n\n");
  }

  bool foundleftface = false;
  bool foundrightface = false;

  Network yarp; // set up yarp

  BufferedPort<ImageOf<PixelRgb> > imageLeftPort; // port for reading in images
  BufferedPort<ImageOf<PixelRgb> > imageRightPort; // port for reading in image
  imageLeftPort.open("/savefaces/image/left/in"); // give a name for port
  imageRightPort.open("/savefaces/image/right/in"); // give a name for port
  


  ImageOf<PixelRgb> *imageLeft = imageLeftPort.read(); // read an image
  ImageOf<PixelRgb> *imageRight = imageRightPort.read(); // read an image
  IplImage *cvLeftImage = cvCreateImage(cvSize(imageLeft->width(), imageLeft->height()),IPL_DEPTH_8U, 3);
  IplImage *cvRightImage = cvCreateImage(cvSize(imageRight->width(), imageRight->height()),IPL_DEPTH_8U, 3);
    
  cvNamedWindow("left", 1);
  cvNamedWindow("right", 1);

  while(1){ // repeat forever
    
    count++;

    imageLeft = imageLeftPort.read(); // read an image
    imageRight = imageRightPort.read(); // read an image
    if (imageLeft  != NULL){
      //printf("We got image of size %dx%d\n ", imageLeft->width(), imageLeft->height());
      cvCvtColor((IplImage*)imageLeft->getIplImage(), cvLeftImage, CV_RGB2BGR);

      // do whatever in opencv with the image
      CvSeq* torsosleft = torsoDetector.detect(cvLeftImage);

      //printf("found torsos = %d\n", torsosleft->total);

      // draw circle on torso
      float scale = 1.0;
      for(int i = 0; i < torsosleft->total; i++){
	CvRect* r = (CvRect*)cvGetSeqElem(torsosleft, i);
	CvPoint center;
	int radius;


	
	center.x = cvRound((r->x + r->width * 0.5) * scale);
	center.y = cvRound((r->y + r->height * 0.5) * scale);

	// save this image onto a file before drawing the circle
	writeImage(count,cvLeftImage, center);

	radius = cvRound((r->width + r->height)*0.25*scale);
	//cvCircle(cvLeftImage, center, radius, cvScalar(0.5));

	// rectangle around face
	CvPoint pp1;
	pp1.x = center.x - 50;
	pp1.y = center.y - 50;
	CvPoint pp2;
	pp2.x = center.x + 50;
	pp2.y = center.y + 50;
	cvRectangle(cvLeftImage, pp1, pp2, CV_RGB(255,0,0),1,8,0);
	


	leftcamfacex = center.x;
	leftcamfacey = center.y;

	point p;
	p.x = center.x;
	p.y = center.y;
	leftpoints.push_back(p);

	foundleftface = true;

      }

      // show the image
      cvShowImage("left", cvLeftImage);
      cvWaitKey(10);
    }
    if (imageRight  != NULL){
      //printf("We got image of size %dx%d\n ", imageLeft->width(), imageLeft->height());
      cvCvtColor((IplImage*)imageRight->getIplImage(), cvRightImage, CV_RGB2BGR);

      // do whatever in opencv with the image
       // do whatever in opencv with the image
      CvSeq* torsosright = torsoDetector.detect(cvRightImage);

      //printf("found torsos = %d\n", torsosleft->total);

      // draw circle on torso
      float scale = 1.0;
      for(int i = 0; i < torsosright->total; i++){
	CvRect* r = (CvRect*)cvGetSeqElem(torsosright, i);
	CvPoint center;
	int radius;
	center.x = cvRound((r->x + r->width * 0.5) * scale);
	center.y = cvRound((r->y + r->height * 0.5) * scale);
	radius = cvRound((r->width + r->height)*0.25*scale);
	//cvCircle(cvRightImage, center, radius, cvScalar(0.5));

	
	CvPoint pp1;
	pp1.x = center.x - 50;
	pp1.y = center.y - 50;
	CvPoint pp2;
	pp2.x = center.x + 50;
	pp2.y = center.y + 50;
	cvRectangle(cvRightImage, pp1, pp2, CV_RGB(255,0,0),1,8,0);

	rightcamfacex = center.x;
	rightcamfacey = center.y;

	point p;
	p.x = center.x;
	p.y = center.y;
	rightpoints.push_back(p);

	foundrightface = true;
      }

      // show the image
      cvShowImage("right", cvRightImage);
      cvWaitKey(10);
    }

   
    xl = prevxl;
    xr = prevxr;
    yl = prevyl;
    yr = prevyr;
    bool done = false;
   float bdist = 100.0;
   
    if (foundleftface && foundrightface){ 
      
      
      float curxl, curyl, curxr, curyr, curd;
      for (int i = 0; i < leftpoints.size();i++){
	for (int j = 0; j < rightpoints.size();j++){
	  curxl = leftpoints.at(i).x;
	  curyl = leftpoints.at(i).y;
	  curxr = rightpoints.at(j).x;
	  curyr = rightpoints.at(j).y;
	  

	  curd = sqrt((curxl - curxr)*(curxl - curxr)+(curyl - curyr)*(curyl - curyr) );

	  //printf("left: %f, %f, right: %f, %f, dist: %f \n ", curxl, curyl , curxr, curyr, curd);
	  //if ((curd < bdist) && (curxl > 20) && (curxl < 300) && (curyl > 20) && (curyl < 220)){
	 if ((curd < bdist)){
	    bdist = curd;
	    xl = curxl;
	    yl = curyl;
	    xr = curxr;
	    yr = curyr;
	    
	    done = true;
	 }
	}
      }

     }
    
  
    if (!done){
      xl = prevxl;
      yl = prevyl;
      xr = prevxr;
      yr = prevyr;
    }
  
    
    // now send these 2D points

    if (done){
      Vector& twoDv = two_d_outPort.prepare();
      twoDv.resize(4);
      twoDv[0] = xl;
      twoDv[1] = yl;
      twoDv[2] = xr;
      twoDv[3] = yr;
      two_d_outPort.write();

      //printf("sending: %f,%f,%f,%f, dist = %f \n ", xl,xr,yl,yr, bdist);
   
    }
    else{
      
      //printf("too much bdist = %f \n ", bdist);
      // Vector& twoDv = two_d_outPort.prepare();
      //twoDv.resize(4);
      //twoDv[0] = 160;
      //twoDv[1] = 120;
      //twoDv[2] = 160;
      //twoDv[3] = 120;
      //two_d_outPort.write();
    }
   
    /*
    // receive 3D points
    bool gotThreeD = false;
    
    Vector* threeD = three_d_inPort.read();
    threeD = three_d_inPort.read();
    if (threeD != NULL){
      gotThreeD = true;
      xx = (*threeD)[0];
      yy = (*threeD)[1];
      zz = (*threeD)[2];
    }
    
    

  
    // send a single found face
    float found = 0.0;
    if (foundleftface && foundrightface){found = 1.0;}
    

    
      
      
      
    if (found){

	Vector& target = targetPort.prepare();
      target.resize(7);
      target[0] = xx;
      target[1] = yy;
      target[2] = zz;
      target[3] = 0.01;
      target[4] = xl;
      target[5] = yl;
      target[6] = found;
      targetPort.write();

	printf("----------------------------------\n");
	printf("2D: %f, %f, %f,%f\n",xl,xr,yl,yr);
	printf("3D: %f,%f,%f: \n", xx,yy,zz);
	printf("sent %g,%g,%g,%g,%g,%g,%g:\n", target[0],target[1],target[2],target[3], target[4],target[5],target[6]); 
	
	


     
      }      
    
    */
   
    foundleftface = false;
    foundrightface = false;

    leftpoints.clear();
    rightpoints.clear();


    prevxl = xl;
    prevxr = xr;
    prevyl = yl;
    prevyr = yr;
    
    //if (count == 10 )break;
    
  }
  cvDestroyWindow("left");
  cvDestroyWindow("right");
  return 0;
  
}
