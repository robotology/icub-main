// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
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

#include <iostream>
#include <vector>
#include <yarp/os/all.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include "ICubShoulderDecouplingBox.h"

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

class VelocityController
{
public:
    enum VCMode{VC_IDLE=0, VC_ACTIVE};

private:
    char                        mBaseName[256];
    char                        mName[256];
    PolyDriver                 *mDriver;

    Semaphore                   mMutex;

    IEncoders                   *mEncoders;
    IControlLimits              *mLimitsController;
    IVelocityControl            *mVelocityController; 
    
    double                      mTime;
    double                      mPrevTime;
    double                      mLastPosCommandTime;
    double                      mLastVelCommandTime;
    double                      mCommandTimeout;
    double                      mMinimumLoopTime;
    double                      mCummulativeDt;
    
    bool                        bIsReady;

    bool                        bPosTimeoutPause;
    bool                        bVelTimeoutPause;
    bool                        bPosTargetSet;
    bool                        bVelTargetSet;
    
    vector<VCMode>              mMode;
          
    int                         mJointsSize;
    Vector                      mJointsPos;
    Vector                      mJointsVel;
    Vector                      mJointsAcc;
    
    Vector                      mJointsTargetPos;
    Vector                      mJointsTargetVel;
    Vector                      mJointsOutputVel;
    Vector                      mJointsPrevOutputVel;
    
    Vector                      mJointsPosLimits[2];
    Vector                      mJointsVelLimits[2];
    Vector                      mJointsRange;   
    
    Vector                      mJointsRest;   
    
    Vector                      mJointsError;
    
    Vector                      mJointsMask;

    Vector                      mJointsKp;
    Vector                      mJointsKd;
    
    ICubShoulderDecouplingBox   mDecouplingBox;
    bool                        bUseShoulderDecoupling;
    
    bool                        bFirst;
    
    bool                        bMoveToRest;

    double                      mSpeedFactor;
public:

    VelocityController();
    ~VelocityController();
    
    bool    Init(PolyDriver *driver, const char* name, const char* basename = NULL);
    void    Free();

    char*   GetBaseName();

    void    Update();
    void    Update(double dt);       

    void    Stop();

    void    SetPositionTarget(Vector &target);
    void    SetVelocityTarget(Vector &target);
    void    GetPosition(Vector &pos);
    void    GetVelocity(Vector &vel);
    
    void    SetMask(Vector &mask);
    void    SetMaskAll();
    void    SetMaskNone();
         
    void    SetControlMode(VCMode mode);
    void    SetKp(double kps);
    void    SetKd(double kds);
    
    void    SetCommandTimeout(double timeout);
    void    SetMinimumLoopTime(double time);

    int     GetJointsSize();
    
    void    SetShoulderDecoupling(bool set);
    
    void    MoveToRest();
    
    void    SetSpeedFactor(double fact);
};









