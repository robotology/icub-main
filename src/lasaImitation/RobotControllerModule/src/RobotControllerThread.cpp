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

#include "YarpTools/YarpMathLibInterface.h"

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

    for(int i=0;i<2;i++){
        mFwdKinWristJoints[i].resize(mFwdKinWrist[i]->getDOF());
        mFwdKinArmJoints[i].resize(mFwdKinArm[i]->getDOF());
    }
    
    for(unsigned int i=0;i<2;i++){
        mSrcToArmIndices[i].clear();
        mSrcToWristIndices[i].clear();
        mArmToIKSIndices[i].clear();
        mWristToIKSIndices[i].clear();
    }
    mSrcToIKSIndices.clear();
    for(unsigned int j=0;j<2;j++){
        for(unsigned int i=0;i<3;i++){
            mSrcToArmIndices[j].push_back(2*16+i);
            mSrcToWristIndices[j].push_back(2*16+i);
        }
        for(unsigned int i=0;i<5;i++){
            mSrcToArmIndices[j].push_back(j*16+i);
            mSrcToWristIndices[j].push_back(j*16+i);
        }
        for(unsigned int i=5;i<7;i++)
            mSrcToArmIndices[j].push_back(j*16+i);
    }
    for(unsigned int i=0;i<3;i++)
        mSrcToIKSIndices.push_back(2*16+i);
    for(unsigned int i=0;i<7;i++)
        mSrcToIKSIndices.push_back(i);
    for(unsigned int i=0;i<7;i++)
        mSrcToIKSIndices.push_back(16+i);

    /*vector<unsigned int> &array = mSrcToArmIndices[1];
    for(size_t j=0;j<array.size();j++)
        cout << array[j]<<" ";
    cout << endl;
    */
    mIKSolver.SetSizes(2*7+3);
    mIKSolver.AddSolverItem(5+3);
    mIKSolver.AddSolverItem(5+3);
    mIKSolver.AddSolverItem(7+3);
    mIKSolver.AddSolverItem(7+3);
    mIKSolver.SetVerbose(false);
    mIKSolver.SetThresholds(0.00005,0.00001);        

    mIKSolver.SetPriority(0,0);
    mIKSolver.SetPriority(1,1);
    mIKSolver.SetPriority(2,2);
    mIKSolver.SetPriority(3,3);
    mIKSolver.Enable(false,0);
    mIKSolver.Enable(false,1);
    
    mJointsLimits[0].resize(2*16+3);
    mJointsLimits[1].resize(2*16+3);
    mIKJointsRest.resize(2*16+3);
    mIKJointsPos.resize(2*16+3);
    double limHigh[] = { 10,160, 80,106, 90,  0, 40,60,100,80,90,80,90,80,90,115, 10,160, 80,106, 90,  0, 40,60,100,80,90,80,90,80,90,115, 50, 30, 70};
    double limLow[]  = {-90,  0,-37,  6,-90,-90,-20, 0,-15, 0, 0, 0, 0, 0, 0,  0,-90,  0,-37,  6,-90,-90,-20, 0,-15, 0, 0, 0, 0, 0, 0,  0,-50,-30,-10};
    
    for(int i=0;i<(2*16+3);i++){
        mJointsLimits[0][i] = limLow[i];
        mJointsLimits[1][i] = limHigh[i];
        mIKJointsRest[i] = (mJointsLimits[0][i]+(mJointsLimits[1][i]-mJointsLimits[0][i])*0.5)*(PI/180.0);
    }
    //cout << mJointsLimits[0].toString()<<endl;
    //cout << mJointsLimits[1].toString()<<endl;
    
    mTime               = 0.0;
    mPrevTime           =-1.0;    
}

void    RobotControllerThread::Free(){
    if(mFwdKinArm[0])   delete mFwdKinArm[0];   mFwdKinArm[0] = NULL;
    if(mFwdKinArm[1])   delete mFwdKinArm[1];   mFwdKinArm[1] = NULL;
    if(mFwdKinWrist[0]) delete mFwdKinWrist[0]; mFwdKinWrist[0] = NULL;
    if(mFwdKinWrist[1]) delete mFwdKinWrist[1]; mFwdKinWrist[1] = NULL;
}

void RobotControllerThread::run()
{
    if(mPrevTime<0.0){
        mPrevTime = Time::now();
        return;
    }else{
        mPrevTime = mTime;    
    }    
    mTime       = Time::now();
    double dt   = mTime - mPrevTime;    
    
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

    mTargetJointPos = mCurrentJointPos;
    mTargetJointVel = 0;

    // Update each kin chain
    for(int j=0;j<2;j++){
        // Get angles
        mSrcToArmIndices[j][1]=0;
        for(size_t i=0;i<mSrcToArmIndices[j].size();i++)    
            mFwdKinArmJoints[j][i]   = mCurrentJointPos[mSrcToArmIndices[j][i]]*(M_PI/180.0);
        for(size_t i=0;i<mSrcToWristIndices[j].size();i++)    
            mFwdKinWristJoints[j][i]   = mCurrentJointPos[mSrcToWristIndices[j][i]]*(M_PI/180.0);        

        // Get pose and jacobian
        mFwdKinWristPose[j]     = mFwdKinWrist[j]->EndEffPose(mFwdKinWristJoints[j]);
        mFwdKinWristJacobian[j] = mFwdKinWrist[j]->GeoJacobian();
        mFwdKinWristRef[j]      = mFwdKinWrist[j]->getH();
        mFwdKinArmPose[j]       = mFwdKinArm[j]->EndEffPose(mFwdKinArmJoints[j]);
        mFwdKinArmJacobian[j]   = mFwdKinArm[j]->GeoJacobian();
        mFwdKinArmRef[j]        = mFwdKinArm[j]->getH();        
    }
    
    //Update IK    
    MathLib::Matrix tmpM;
    for(int i=0;i<2;i++){
        mIKSolver.SetJacobian(YarpMatrixToMatrix(mFwdKinWristJacobian[i],tmpM), 0+i);
        mIKSolver.SetJacobian(YarpMatrixToMatrix(mFwdKinArmJacobian[i],tmpM),   2+i);
    }
    
    // Limiting output: Max 60 deg per seconds
    Vector lim1; lim1.resize(2*7+3); lim1=-(60.0 *(M_PI/180.0));
    Vector lim2; lim2.resize(2*7+3); lim2= (60.0 *(M_PI/180.0));
    for(int i=0;i<(2*7+3);i++){
        // Limiting speed when approaching 10 deg from joint range
        unsigned int j = mSrcToIKSIndices[i];
        if(mCurrentJointPos[j]-mJointsLimits[0][j]<10.0){
            if(mCurrentJointPos[j]-mJointsLimits[0][j]<0.0) lim1[i] = 0.0;
            else lim1[i] *= (mCurrentJointPos[j]-mJointsLimits[0][j])/10.0;    
        }else if(mJointsLimits[1][j]-mCurrentJointPos[j]<10.0){
            if(mJointsLimits[1][j]-mCurrentJointPos[j]<0.0) lim2[i] = 0.0;
            else lim2[i] *= (mJointsLimits[1][j]-mCurrentJointPos[j])/10.0;                    
        }
    }
    mIKSolver.SetLimits(YarpVectorToVector(lim1),YarpVectorToVector(lim2));
    
    //Vector asd = mJointsRest - 
    for(size_t i=0;i<mSrcToIKSIndices.size();i++){
        mIKJointsPos(i) = mCurrentJointPos(mSrcToIKSIndices[i])*(PI/180.0);
    }    
    mIKSolver.SetNullTarget((YarpVectorToVector(mIKJointsPos)-YarpVectorToVector(mIKJointsRest))*0.1);

    mIKSolver.Enable(false,2);
    mIKSolver.Enable(false,3);
    mIKSolver.Solve();
    
    Vector ikOutput;
    VectorToYarpVector(mIKSolver.GetOutput(),ikOutput);
    cout << ikOutput.toString()<<endl;
    for(size_t i=0;i<mSrcToIKSIndices.size();i++){
        mTargetJointPos(mSrcToIKSIndices[i]) = mCurrentJointPos(mSrcToIKSIndices[i]) + ikOutput[i]*(180.0/PI)*dt;
    }
    
    switch(mState){
    case RCS_IDLE:
        mTargetJointPos = mCurrentJointPos;
        mTargetJointVel = 0;
        /*for(int i=0;i<mJointSize;i++){
            mTargetJointPos(i) = mCurrentJointPos(i)-5;
        }*/
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

