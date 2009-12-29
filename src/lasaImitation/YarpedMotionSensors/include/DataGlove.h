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

#ifndef __DATAGLOVE_H__
#define __DATAGLOVE_H__

#include <stdio.h>
#include <string.h>

#include "MathLib/MathLib.h"
#include "fglove.h"

using namespace MathLib;
 
/**
 * \class DataGloves
* 
 * \brief A class for managing up to a pair of 5DT Data-Gloves in a "simple" way
 * 
 */

typedef struct{
  REALTYPE  mThumb0;    
  REALTYPE  mThumb1;

  REALTYPE  mThumbIndex;
      
  REALTYPE  mIndex0;    
  REALTYPE  mIndex1;

  REALTYPE  mIndexMiddle;

  REALTYPE  mMiddle0;    
  REALTYPE  mMiddle1;

  REALTYPE  mMiddleRing;

  REALTYPE  mRing0;    
  REALTYPE  mRing1;

  REALTYPE  mRingLittle;

  REALTYPE  mLittle0;    
  REALTYPE  mLittle1;
} DataGloveHandSensors;

  

union DataGloveHand 
{
  DataGloveHandSensors  mSensors;
  REALTYPE              mSensorsArray[14];
};  
  

class DataGloves
{
public:
  fdGlove          *mGloveLeft;
  fdGlove          *mGloveRight;

  unsigned short    mGloveLeftRawData[18];
  unsigned short    mGloveRightRawData[18];

  DataGloveHand     mHandLeft;
  DataGloveHand     mHandRight; 
  
  DataGloveHand     mHandRangeLeft[2];
  DataGloveHand     mHandRangeRight[2];

  bool              bInit;
  bool              bFirst;
  int               nDevices;
  bool              bCalibrate;

  char              mCOMPortName[256];

  int               mNbCalibrationPoses;
  int               mNbTargetDofs;
  Matrix            mCalibrationPoses;
  Matrix            mTargetCalibrationPoses;
  Matrix            mCalibrationTransform[2];

public:
  
public:
  /// Constructror
  DataGloves();
  /// Destructor
  ~DataGloves();

  /// Initialize the DataGloves device. Return true if success.
  int     Init(const char* portName = NULL);    
  /// Free all
  void    Free();
  
    
  int     Update();

  /// Set the identity transformation to the current sensors' state ( at the next call to GetData() ) 
  void    StartCalibration();
  void    StopCalibration();
  
  int     GetNbCalibrationPose();
  void    GetCalibrationPose(EfdGloveHand hand, int id);
  void    GetTargetCalibrationPose(int id, Vector *pose);
  void    PosesCalibration(EfdGloveHand hand);
  void    GetCalibratedPose(EfdGloveHand hand, Vector *pose);
  
  REALTYPE        GetSensorValue(EfdGloveHand hand, EfdSensors id);

  DataGloveHand&  GetSensorValue(EfdGloveHand hand);
  
  /// Print some stuff
  void    Print();
  
};

#endif

