// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 Eric Sauser, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   eric.sauser@a3.epfl.ch
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
 *
@ingroup icub_lasaImitation_module
\defgroup icub__lasaImitation_MotionSensorsModule Motion Sensors Capture Module

This module or application allows to capture joints angles from X-Sens motion sensors and 5DT Datagloves to be mapped directly to iCub for direct telepoeration. 

\section intro_sec Description

When the software is running, an 3d scene appears with two stick figures. One correspond to the direct reading from the sensors, while the scond correspond to the joint-based reconstruction (iCub-standard). Joint position are always steamed to the output ports. So it's the user responsibility to not accept data when the system is not properly calibrated. Following are the key command to be typed on the user interface.

    - 'c' : calibrate xsens sensors (You should have both arm straight down and wait for 10 sec)
    - 'b' : sequential calibration for left hand dataglove (should be pressed 7 times, each time with a different hand posture)
          - 1. Hand fully opened
          - 2. All fingers straight but sticked together
          - 3. Same but with the thumb at 90 degrees forward (sot on the side)
          - 4. Same but index and thumb should touch and form a circle
          - 5. Victory sign
          - 6. Same but only with the index finger up
          - 7. Fist position (thumb out)
    - 'n' : sequential calibration for right hand dataglove (should be pressed 7 times, each time with a different hand posture)
    - 's' : start/stop saving data. Saved data are numbered and put in : ./data/dataXXX.txt.

\section dependencies_sec Dependencies

- YARP
- OpenGL, GLUT
- libgloved.so (from 5DT technologies)
- libmtobject.so (from X-Sens technologies)


\section parameters_sec Parameters

\verbatim
-x /dev/ttyXXX: Enable the use of X-Sens sensors which can be read 
                from the given unix device port (e.g. /dev/ttyUSB0)
-g /dev/ttyXXX: Enable the use of Datagloves which can be read 
                from the given unix device port (e.g. /dev/ttyUSB1)
-noyarp:        If you don't want to use yarp this time (no need for 
                yarpserver, e.g. just for data recording)\endverbatim
\endverbatim

\section portsa_sec Ports Accessed

\section portsc_sec Ports Created

Input ports:

None

Output ports:

Input ports:

None

Output ports:

- /MotionSensors/left_glove: calibrated joint values for left hand
- /MotionSensors/right_glove: calibrated joint values for right hand
- /MotionSensors/left_arm: calibrated joint values for left arm
- /MotionSensors/right_arm: calibrated joint values for right arm



\section in_files_sec Input Data Files

None

\section out_data_sec Output Data Files

None

\section conf_file_sec Configuration Files

None

\section tested_os_sec Tested OS

LWorks on Linux only


\section example_sec Example Instantiation of the Module

\verbatim
YarpedMotionSensors -x /dev/ttyUSB0 -g /dev/ttyUSB1
\endverbatim

\author Eric Sauser

Copyright (C) 2008 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/lasaImitation/YarpedMotionSensors/include/XSens2ICub.h. 
**/

#ifndef XSENS2HOAP_H_
#define XSENS2HOAP_H_

#include <GL/glut.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
using namespace std;

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

using namespace yarp;
using namespace yarp::os;
//using namespace yarp::sig;

#include "StdTools/Timer.h"
#include "GLTools/GLTools.h"
#include "GLTools/GLCamera.h"
#include "GLTools/GLFont.h"
#include "UDPConnection/UDPNetwork.h"
#include "XSens.h"
#include "DataGlove.h"


#define WINDOW_WIDTH    640
#define WINDOW_HEIGHT   480

Network mYarp;

bool bUseYarp;

BufferedPort<yarp::sig::Vector>      mGlovePorts[2];
BufferedPort<yarp::sig::Vector>      mArmPorts[2];


bool      bReadFile = false;
ifstream  jfile;

Vector3   pts[4];
typedef struct{
  double  mPalmAperture;
  double  mThumbOpposition;
  double  mThumb0;    
  double  mThumb1;
  double  mIndex0;    
  double  mIndex1;
  double  mMiddle0;    
  double  mMiddle1;
  double  mRingLittle;
} iCubHand;

typedef struct{
  double  mSFE;
  double  mSAA;
  double  mSHR;
  double  mEB;
  double  mWR;
  double  mWP;
  double  mWY;  
} iCubArm;

typedef struct{
  double  mTorso0;
  double  mTorso1;
  double  mTorso2;  
} iCubTorso;

typedef struct{
  float torso;
  float torso0,torso1,torso2;  
  float rsfe,rsaa,rshr,reb;
  float lsfe,lsaa,lshr,leb;
  float rwr,lwr;
  float rwp,rwy;
  float lwp,lwy;
  float rfng,lfng;
  float hflx,hpan,htlt;
}BasicUpperBodyJointAngles;

Matrix4 leftHandHMatrix;
Matrix4 rightHandHMatrix;
bool   bHandsOn;

enum BodyMatricesId {
  BID_TORSO = 0,
  BID_HEAD,
  BID_LEFTSHOULDER,
  BID_LEFTELBOW,
  BID_LEFTWRIST,
  BID_RIGHTSHOULDER,
  BID_RIGHTELBOW,
  BID_RIGHTWRIST,
  BID_SIZE
};

typedef struct{
  // XSens matrices
  Matrix3   mBodyMatrices[BID_SIZE];
  Matrix3   mBodyMatricesTransform[BID_SIZE];
  // Zero matrices      
  Matrix3   mBodyMatricesZero[BID_SIZE];

  Matrix3   mBodyMatricesReconstructed[BID_SIZE];

  int       mBodyMatricesIndex[BID_SIZE];  

  DataGloveHand  mHandLeft;
  DataGloveHand  mHandRight;

  DataGloveHand  mHandOrigin;
  DataGloveHand  mHandScale;
  
  float baseTruncHeight;        
  float truncHeight,truncWidth;
  float headLength;  
  float neckLength;
  float upperArmLength;
  float forearmLength;
  float handLength;
  float finger0Length;
  float finger1Length;

} BasicUpperBody;

BasicUpperBody              mSensorBody;

BasicUpperBodyJointAngles   hoap3angles;
BasicUpperBodyJointAngles   hoap3min,hoap3max;
BasicUpperBodyJointAngles   hoap3anglesPrev;

iCubHand    mICubHandLeft;
iCubHand    mICubHandRight;
iCubArm     mICubArmLeft;
iCubArm     mICubArmRight;


DataGloves  gloves;
bool        bUseGloves;
char        glovesPort[256];

XSens       xsens;
bool        bUseXSens;
char        xsensPort[256];

Timer       calibTimer;
int         calibTimeout;
bool        bCalibRequest;

int         refreshPeriod;
Timer       refreshTimer;

UDPNetwork hoap3Net;
bool  bUseNetwork;
int   port;
char  host[256];


bool bShowPose = false;
int poseCount = 0;
Matrix3 posesMatrices[10][BID_SIZE];

Chrono mTimeChrono;


bool        bUseGUI;

GLCamera    camera;
int         currX;
int         currY;
int         currBtn;

bool        bSavingData;
int         saveFileCount;
ofstream    dataFile;
float       bkgColor;

char        dataString[1024];

GLFont      mFont;

int   Init(int argc, char* argv[]);
void  InitGL();
void  Display(); 
void  OnReshape(int width, int height);
void  OnKeyPress(unsigned char key, int x, int y);
void  OnSpecialKeyPress(int key, int x, int y);
void  OnMouseButton(int button, int state, int x, int y);
void  OnMouseMotion(int x, int y);
void  OnIdle();

void  UpdateSkel();

void  DrawSkel(Matrix3 *bodyMatrices);

void  CalculateHoap3Angles();
void  ComputeAngles();
void  PrintHoap3Angles();

void  UpdateDataString();

void  StartSavingData(string filename);
void  UpdateSavingData();
void  StopSavingData();

void  GlovesToICub(DataGloveHandSensors *ghand, iCubHand *hand);
void  SendDataToYarp();

#endif /*XSENS2HOAP_H_*/
