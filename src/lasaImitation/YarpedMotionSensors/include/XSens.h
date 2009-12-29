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

#ifndef __XSENS_H__
#define __XSENS_H__

#include "MathLib/MathLib.h"
#include "StdTools/Timer.h"
#include "MotionTracker.h"

using namespace MathLib;

#define MAX_MPCCOUNT 16 

/**
 * \class XSens
 * 
 * \brief A class for managing X-Sens in a "simple" way
 * 
 * This class, when started, simply looks for each present sensor
 * on the XSens bus. Data can be retreived by using the id of the
 * desired sensor.
 */

class XSens
{
public:
  void              *module;
  MotionTracker     *pMT;
  destroy_t         *destroy_mtobject;

  char               mCOMPortName[256];

public:
  float       mRawData[1024];
  char        cDeviceIDs[512];
  
  bool        bInit;
  int         nDevices;
  int        *mDeviceIDs;

  Matrix3     mIdentity;
  Matrix3    *mRawMatrices;
  Matrix3    *mPreMatrices;
  Matrix3    *mCalMatrices;
  Matrix3    *mOutMatrices;  
  Matrix3    *mFltMatrices;  


  bool        bCalibrate;
  bool        bTransposeOutput;
  
  int         mMultiPoseCalibrationCount;
  Matrix3    *mMPCTransformMatrices;
  Matrix3    *mMPCDesiredMatrices;
  Matrix3    *mMPCObservedMatrices;
  bool        bMPCalibrate;

  Matrix      mSequencerData;
  Vector      mSequencerTimeData;
  int         mSequencerState;
  int         mSequencerPos;
  Chrono      mSequencerChrono;
  int        *mSequencerDeviceIds;
  
public:
  /// Constructror
  XSens();
  /// Destructor
  ~XSens();

  /// Initialize the XSens device. Return true if success.
  int     Init(const char* portName = NULL);    
  /// Free all
  void    Free();
  
  /// Start the xsens device  
  int     Start();
  /// Reset it
  void    Reset();
  /// Stop it
  void    Stop();

  /// Set the identity transformation to the current sensors' state ( at the next call to GetData() ) 
  void    Calibrate();

  void    MultiPoseCalibrationReset();
  void    MultiPoseCalibrationSetPose(int id, Matrix3 &desiredPose);
  void    MultiPoseCalibrationFetch();
  void    MultiPoseCalibrate();
    
  /// Capture function which should be placed in the main processing loop
  int     GetData();
  
  /// Print some stuff
  void    PrintData();
  
  /// Return the orientation matrix of the corresponding sensor (See Calibrate() function)
  Matrix3 &GetOutput(int id);
  
  /// Tool function for an automatic transpose of the orientation matrices
  void    TransposeOutput(bool yes=true);
  
  void    SetPreMatrix(int id, Matrix3 & matrix);
  
  void    StartSequencer(bool record = false);
  void    StopSequencer();
  void    LoadSequencerData(const char * filename);
  void    SaveSequencerData(const char * filename);
  
  static  XSens*  mActiveXSens;
  static  void    SegFaultHandler(int val);
};

#endif

