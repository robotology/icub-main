// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2007 Micha Hersch, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   <firstname.secondname>@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */
/**
 * @author Micha Hersch
 * @date January 2007
 */ 

#include <iostream>
#include "iCub/public.h"
//#include "iCub/frameGrabber.h"
#include "iCub/colordetect.h"
#include "colorTracker.h"
#include "iCub/distortionCorrector.h"
#include "iCub/blobTracker.h"
#include "iCub/gmmTracker.h"
#include "iCub/basicMath.h"
#include "iCub/locator3D.h"
#include "iCub/visionServer.h"
#include "iCub/pd_kalman.h"
#include "iCub/Camera.h"

#ifndef ICUB
#ifdef LINUX
#include "CameraV4L.h"
#else
#include "cvcam.h"
#include "cvcamGrabber.h"
#endif
#endif


using namespace std;

#define MAX_CAM 2

#ifndef MAX_OBJECTS
#define MAX_OBJECTS 3
#endif


/**
 * @brief a stereo-vision application tracking color blobs and sending their 3d position through yarp
 */
class Stereovision{

 protected:

  FrameGrabber *grabber[2];
  ColorTracker *colorfinder[2][MAX_OBJECTS];
  BlobTracker *blob[2];
  GmmTracker *gmm[2];
  Locator3D *loc;
  CKalman *kal[MAX_OBJECTS];
  DistortionCorrector *dc[2];

    //  VisionServer port;
    VisionServer port[MAX_OBJECTS];



  Vec2 point2d[2];
  Vec3 point3d[MAX_OBJECTS];
  Vec3 point3df[MAX_OBJECTS];
  int found3d[MAX_OBJECTS];
  double position[MAX_OBJECTS*3];
  int found;
  int save3d;
  Camera **cams;
  IplImage *img[2], *tmp1_img[2], *tmp2_img[2] ;
  int nbcams, i,j;
  char wname[2][80];
  char tbname[2][80];
  char diff_name[2][80];
  char paramfile[2][80];
  char stereoparamfile[80];
  int diff;
  int finish;
  int track;
  int locate;
  int comm;
  int kalman ;

  int nb_obj;
  int skip;
  int trackbar[2];



  mouseParam_t mparam[2];


 protected:

    virtual void InitCameras();
    virtual bool  InitGrabber(int i);
    virtual void SendOutput(VisionServer *port,double *position);

 public:
 
 /** 
   * @brief  constructor and initiaizer
   * @param if argc==2 then argv[1] contains the directory 
   * where to save the parameters
   */
  Stereovision(int argc=0, char **argv = NULL);
 
  /** 
   * @brief  distructor
   */
  
  virtual ~Stereovision();


 /**
   * @brief executes a command
   * command include: 
   * 't' enable/disable 2d tracking
   * 'l' enable/disable 3d localization
   * 'a' add an object to be tracked
   * 'u' start/stop sending the 3d location through yarp
   * 'k' enable/disable kalman filtering for all objects
   * 'n' switch to next object
   * 'M' raise measurement noise of the kalman filter for the current object
   * 'm' lower measurement noise of the kalman filter for the current object
   * 'P' raise process noise of the kalman filter for the current object
   * 'p' lower process noise of the kalman filter for the current object
   * 'r' stop/resume tracking the current object (only one of them can be stopped)
   * 'c' calibrate the camera intrinsics parameters (distortion coeffs)
   * '3' calibrate the camera extrinsics parameters (for position and orientation)
   * 'w' switch color mode (YCbCr vs )
   * 'd' show effect of undistortion
   * 'q' exit application
   */
  void ExecuteCommand(int key);


  /**
   * vision initialization. Instantiates all trackers,grabber,filters etc
   * @param paramdir directory where to put calib parameters
   */
  void InitVision(const char *paramdir=NULL);

  /**
   * @brief main loop of the application
   */
  void Run();

    /**
     * @brief grabs the two frames and process them 
     */
  void ProcessNextFrame();

};

#ifdef OPENCV_9_5
void on_mouse0( int event, int x, int y, int flags);//, void* param );
void on_mouse1( int event, int x, int y, int flags);
#else
void on_mouse0( int event, int x, int y, int flags, void *param );
void on_mouse1( int event, int x, int y, int flags,void *param);
#endif

void on_mouse( int event, int x, int y, int flags, int index);


void on_mouse( int event, int x, int y, int flags, int index);


// trackbar callback functions obsolete I think
void update_sigma0(int slider);
void update_sigma1(int slider);
void update_sigma(int slider,int window);



