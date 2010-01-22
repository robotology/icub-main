// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author:  Eric Sauser
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

#ifndef RobotControllerTHREAD_H_
#define RobotControllerTHREAD_H_

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::sig;

#include <iCub/iKinFwd.h>
#include <iCub/iKinIpOpt.h>

#include "MathLib/IKGroupSolver.h"

#include "iCubAddKinChain.h"

#define INPUTS_TIMEOUT 0.5

class RobotControllerThread: public RateThread
{
public:
    enum State {RCS_IDLE = 0,
                RCS_RUN};
                
private:    
    Semaphore               mMutex;
    int                     mPeriod;
    char                    mBaseName[256];

    State                   mState;

    double                  mTime;
    double                  mPrevTime;
    
    bool                    bOpenLoopControl;
    Vector                  mOpenLoopJointPos;
    bool                    bPauseOpenLoopControl;
    
    int                     mJointSize;
    int                     mIKJointSize;
    Vector                  mTargetJointPos;
    Vector                  mTargetJointVel;
    Vector                  mCurrentJointPos;
    Vector                  mCurrentJointVel;

    
    Vector                  mJointsLimits[2];

    Vector                  mHandPoses[4];
    bool                    hHandState[2];
    double                  mHandGain;
    
    Vector                  mIKJointsRest;
    Vector                  mIKJointsTarget;
    Vector                  mIKJointsPos;
    MathLib::Vector         mIKDofWeights;
    MathLib::Vector         mIKInvDofWeights;
    
    iKin::iCubWrist         *mFwdKinWrist[2];
    iKin::iCubArm           *mFwdKinArm[2];
    iKin::iCubThirdEye      *mFwdKinEye;

    Vector                   mFwdKinWristJoints[2];
    Vector                   mFwdKinArmJoints[2];
    Vector                   mFwdKinEyeJoints;

    Vector                   mFwdKinWristPose[2];
    Vector                   mFwdKinArmPose[2];
    Vector                   mFwdKinEyePose;

    Matrix                   mFwdKinWristJacobian[2];
    Matrix                   mFwdKinArmJacobian[2];
    Matrix                   mFwdKinEyeJacobian;

    Matrix                   mFwdKinWristRef[2];
    Matrix                   mFwdKinArmRef[2];
    Matrix                   mFwdKinEyeRef;

    bool                     bIKUseNullSpace;
    bool                     bIKUseRestNullSpace;
    double                   mNullSpaceGain;
    
    double                   mDesiredCartGain;
    double                   mDesiredCartOriGain;
    int                      bUseDesiredCartPos[2];
    
    Vector                   mDesiredCartPos[2];
    Vector                   mDesiredCartVel[2];
    Vector                   mDesiredCartWristVel[2];
    Vector                   mDesiredWristOpt[2];

    Vector                   mDesiredCartEyeInEyePos;
    Vector                   mDesiredCartEyePos;
    Vector                   mDesiredCartEyeVel;

    MathLib::IKGroupSolver   mIKSolver;
    
    enum SolverID{
        IKArmPosR = 0,
        IKArmPosL,
        IKArmOriR,
        IKArmOriL,
        IKWristR,
        IKWristL,
        IKEye,
        IKSize};
        
    
    vector<unsigned int>    mSrcToArmIndices[2];
    vector<unsigned int>    mSrcToWristIndices[2];
    vector<unsigned int>    mSrcToEyeIndices;
    vector<unsigned int>    mArmToIKSIndices[2];
    vector<unsigned int>    mWristToIKSIndices[2];
    vector<unsigned int>    mEyeToIKSIndices;
    vector<unsigned int>    mSrcToIKSIndices;

    
    BufferedPort<Vector>    mTargetJointPosPort;
    BufferedPort<Vector>    mTargetJointVelPort;

    BufferedPort<Vector>    mCurrentJointPosPort;
    BufferedPort<Vector>    mCurrentJointVelPort;


    BufferedPort<Vector>    mDesiredCartPosRPort;
    BufferedPort<Vector>    mDesiredCartPosLPort;

    BufferedPort<Vector>    mDesiredCartVelRPort;
    BufferedPort<Vector>    mDesiredCartVelLPort;

    BufferedPort<Vector>    mDesiredCartWristVelRPort;
    BufferedPort<Vector>    mDesiredCartWristVelLPort;

    BufferedPort<Vector>    mDesiredHandPosRPort;
    BufferedPort<Vector>    mDesiredHandPosLPort;

    BufferedPort<Vector>    mDesiredArmJointsRPort;
    BufferedPort<Vector>    mDesiredArmJointsLPort;

    BufferedPort<Vector>    mDesiredCartEyeInEyePort;
    BufferedPort<Vector>    mDesiredCartEyePort;
    
    
    
    double                  mCurrentJointPosLastTime;
    
    double                  mDesiredCartPosRLastTime;
    double                  mDesiredCartPosLLastTime;
    double                  mDesiredCartVelRLastTime;
    double                  mDesiredCartVelLLastTime;
    double                  mDesiredCartWristVelRLastTime;
    double                  mDesiredCartWristVelLLastTime;
    double                  mDesiredArmJointsRLastTime;
    double                  mDesiredArmJointsLLastTime;
    double                  mDesiredHandPosRLastTime;
    double                  mDesiredHandPosLLastTime;
    double                  mDesiredCartEyeInEyeLastTime;
    double                  mDesiredCartEyeLastTime;
    
    //Output port
    BufferedPort<Matrix>    mCurrentWristRefRPort;
    BufferedPort<Matrix>    mCurrentWristRefLPort;

    BufferedPort<Vector>    mCurrentCartPosRPort;
    BufferedPort<Vector>    mCurrentCartPosLPort;
    BufferedPort<Vector>    mCurrentCarEyeTargetPort;

public:
            RobotControllerThread(int period, const char* baseName);
    virtual ~RobotControllerThread();

            enum IKSetID{
                IKS_None = 0,
                IKS_RightArm,
                IKS_RightArmPos,
                IKS_RightWrist,
                IKS_LeftArm,
                IKS_LeftArmPos,
                IKS_LeftWrist,
                IKS_Eye,
                IKS_Joints,
                IKS_Rest
            };
            
            IKSetID IKStringToIKSet(string str);
            void    SetIKSolverSet(IKSetID setId, bool enable);
            
            void    SetJointControlMode(int mode);
            void    SetState(State state);
            void    SetHandPose(bool rightHand, bool open);
    
    virtual void    run();
    virtual bool    threadInit();
    virtual void    threadRelease();

protected:
            void    Init();
            void    Free();

            void    ReadFromPorts();
            void    CheckInputsTimeout();
            void    WriteToPorts();
            void    UpdateKinChains();
            void    PrepareIKSolver();
            void    ApplyIKSolver();
    
            void    ConvertEyeInEyeTarget();
            void    ComputeVelocities();
};

#endif

