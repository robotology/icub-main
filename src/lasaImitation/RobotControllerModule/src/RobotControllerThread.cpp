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

 
#include "RobotControllerThread.h"

#include <iostream>
#include <string.h>

using namespace std;


RobotControllerThread::RobotControllerThread(int period, const char* baseName)
:RateThread(period)
{
    mPeriod = period;
    strncpy(mBaseName,baseName,256);
    mFwdKinArm[0]       = NULL;
    mFwdKinArm[1]       = NULL;
    mFwdKinWrist[0]     = NULL;
    mFwdKinWrist[1]     = NULL;
}

RobotControllerThread::~RobotControllerThread()
{}

bool RobotControllerThread::threadInit()
{
    Init();
    
    char portName[256];
    snprintf(portName,256,"/%s/input",mBaseName);
    mInputPort.open(portName);

    snprintf(portName,256,"/%s/output",mBaseName);
    mOutputPort.open(portName);

    snprintf(portName,256,"/%s/targetJointPosition",mBaseName);
    mTargetJointPosPort.open(portName);
    
    snprintf(portName,256,"/%s/targetJointVelocity",mBaseName);
    mTargetJointVelPort.open(portName);

    snprintf(portName,256,"/%s/currentJointPosition",mBaseName);
    mCurrentJointPosPort.open(portName);
    
    snprintf(portName,256,"/%s/currentJointVelocity",mBaseName);
    mCurrentJointVelPort.open(portName);

    return true;
}

void RobotControllerThread::threadRelease()
{
    mInputPort.close();
    mOutputPort.close();
}

void    RobotControllerThread::Init(){
    mState = RCS_IDLE;
    
    mJointSize = 16+16+3;
    mTargetJointPos.resize(mJointSize);
    mTargetJointVel.resize(mJointSize);
    mCurrentJointPos.resize(mJointSize);
    mCurrentJointVel.resize(mJointSize);
    
    mFwdKinArm[0]       = new iKin::iCubArm("right");
    mFwdKinWrist[0]     = new iKin::iCubWrist("right");
    mFwdKinArm[1]       = new iKin::iCubArm("left");
    mFwdKinWrist[1]     = new iKin::iCubWrist("left");
    for(int i=0;i<3;i++){
        mFwdKinArm[0]->releaseLink(i);
        mFwdKinArm[1]->releaseLink(i);
        mFwdKinWrist[0]->releaseLink(i);
        mFwdKinWrist[1]->releaseLink(i);
    }        

    //mFwdKinArmChain[0]      = mFwdKinArm[0]->asChain();
    //mFwdKinArmChain[1]      = mFwdKinArm[1]->asChain();
    //mFwdKinWristChain[0]    = mFwdKinWrist[0]->asChain();
    //mFwdKinWristChain[1]    = mFwdKinWrist[1]->asChain();
}

void    RobotControllerThread::Free(){
    if(mFwdKinArm[0])   delete mFwdKinArm[0];   mFwdKinArm[0] = NULL;
    if(mFwdKinArm[1])   delete mFwdKinArm[1];   mFwdKinArm[1] = NULL;
    if(mFwdKinWrist[0]) delete mFwdKinWrist[0]; mFwdKinWrist[0] = NULL;
    if(mFwdKinWrist[1]) delete mFwdKinWrist[1]; mFwdKinWrist[1] = NULL;
}

void RobotControllerThread::run()
{
    mMutex.wait();
    
    // Read data from input port
    Vector *inputVec;
    inputVec = mCurrentJointPosPort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size()==mJointSize) mCurrentJointPos = *inputVec;
        else cerr << "Bad vector size on port <currentJointPosition>: " << inputVec->size() << "!="<< mJointSize << endl;
    }
    inputVec = mCurrentJointVelPort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size()==mJointSize) mCurrentJointVel = *inputVec;
        else cerr << "Bad vector size on port <currentJointVelocity>: " << inputVec->size() << "!="<< mJointSize << endl;
    }

    
    switch(mState){
    case RCS_IDLE:
        mTargetJointPos = mCurrentJointPos;
        for(int i=0;i<mJointSize;i++){
            mTargetJointPos(i) = mCurrentJointPos(i)-5;
        }
        mTargetJointVel = 0;
        break;
    case RCS_RUN:
        break;
    }
    
    
    
    
    
    // Write data to output port
    {
        Vector &outputVec = mTargetJointPosPort.prepare();
        outputVec = mTargetJointPos;
        mTargetJointPosPort.write();
    }
    {
        Vector &outputVec = mTargetJointVelPort.prepare();
        outputVec = mTargetJointVel;
        mTargetJointVelPort.write();
    }

    mMutex.post();
}

