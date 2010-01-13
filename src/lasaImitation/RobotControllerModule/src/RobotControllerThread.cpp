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
    snprintf(portName,256,"/%s/targetJointPosition",mBaseName);
    mTargetJointPosPort.open(portName);
    
    snprintf(portName,256,"/%s/targetJointVelocity",mBaseName);
    mTargetJointVelPort.open(portName);

    snprintf(portName,256,"/%s/currentJointPosition",mBaseName);
    mCurrentJointPosPort.open(portName);
    
    snprintf(portName,256,"/%s/currentJointVelocity",mBaseName);
    mCurrentJointVelPort.open(portName);
    
    snprintf(portName,256,"/%s/desiredCartVelocityR",mBaseName);
    mDesiredCartVelRPort.open(portName);

    snprintf(portName,256,"/%s/desiredCartVelocityL",mBaseName);
    mDesiredCartVelLPort.open(portName);

    snprintf(portName,256,"/%s/desiredCartPositionR",mBaseName);
    mDesiredCartPosRPort.open(portName);

    snprintf(portName,256,"/%s/desiredCartPositionL",mBaseName);
    mDesiredCartPosLPort.open(portName);

    snprintf(portName,256,"/%s/desiredCartWristVelocityR",mBaseName);
    mDesiredCartWristVelRPort.open(portName);

    snprintf(portName,256,"/%s/desiredCartWristVelocityL",mBaseName);
    mDesiredCartWristVelLPort.open(portName);

    snprintf(portName,256,"/%s/currentWristRefR",mBaseName);
    mCurrentWristRefRPort.open(portName);

    snprintf(portName,256,"/%s/currentWristRefL",mBaseName);
    mCurrentWristRefLPort.open(portName);

    snprintf(portName,256,"/%s/desiredCartEyeInEyePosition",mBaseName);
    mDesiredCartEyeInEyePort.open(portName);

    snprintf(portName,256,"/%s/desiredCartEyePosition",mBaseName);
    mDesiredCartEyePort.open(portName);
    
    snprintf(portName,256,"/%s/currentCartPositionR",mBaseName);
    mCurrentCartPosRPort.open(portName);

    snprintf(portName,256,"/%s/currentCartPositionL",mBaseName);
    mCurrentCartPosLPort.open(portName);

    snprintf(portName,256,"/%s/currentCartEyeTargetPosition",mBaseName);
    mCurrentCarEyeTargetPort.open(portName);
    return true;
}

void RobotControllerThread::threadRelease()
{
    mTargetJointPosPort.close();
    mTargetJointVelPort.close();
    mCurrentJointPosPort.close();
    mCurrentJointVelPort.close();
    mDesiredCartVelRPort.close();
    mDesiredCartVelLPort.close();
    mDesiredCartPosRPort.close();
    mDesiredCartPosLPort.close();
    mDesiredCartWristVelRPort.close();
    mDesiredCartWristVelLPort.close();
    mCurrentWristRefRPort.close();
    mCurrentWristRefLPort.close();
    mDesiredCartEyeInEyePort.close();
    mDesiredCartEyePort.close();
    mCurrentCartPosRPort.close();
    mCurrentCartPosLPort.close();
    mCurrentCarEyeTargetPort.close();
}

void    RobotControllerThread::Init(){
    mState = RCS_IDLE;
    
    mJointSize      = 16+16+3+6;
    mIKJointSize    = 3+7+7+3;
    mTargetJointPos.resize(mJointSize);
    mTargetJointVel.resize(mJointSize);
    mCurrentJointPos.resize(mJointSize);
    mCurrentJointVel.resize(mJointSize);
    mDesiredCartPos[0].resize(6); mDesiredCartPos[0] = 0;
    mDesiredCartPos[1].resize(6); mDesiredCartPos[1] = 0;
    mDesiredCartVel[0].resize(6); mDesiredCartVel[0] = 0;
    mDesiredCartVel[1].resize(6); mDesiredCartVel[1] = 0;
    mDesiredCartWristVel[0].resize(6); mDesiredCartWristVel[0] = 0;
    mDesiredCartWristVel[1].resize(6); mDesiredCartWristVel[1] = 0;
    mDesiredWristOpt[0].resize(5); mDesiredWristOpt[0] = 0;
    mDesiredWristOpt[1].resize(5); mDesiredWristOpt[1] = 0;
    mDesiredCartEyePos.resize(3); mDesiredCartEyePos = 0;
    mDesiredCartEyeVel.resize(3); mDesiredCartEyeVel = 0;
    mDesiredCartEyeInEyePos.resize(3); mDesiredCartEyeInEyePos = 0; mDesiredCartEyeInEyePos[2] = 0.2;
    mFwdKinArm[0]       = new iKin::iCubArm("right");
    mFwdKinWrist[0]     = new iKin::iCubWrist("right");
    mFwdKinArm[1]       = new iKin::iCubArm("left");
    mFwdKinWrist[1]     = new iKin::iCubWrist("left");
    mFwdKinEye          = new iKin::iCubThirdEye();
    for(int j=0;j<2;j++){
        for(int i=0;i<3;i++){
            mFwdKinArm[j]->releaseLink(i);
            mFwdKinWrist[j]->releaseLink(i);
            if(j==0) mFwdKinEye->releaseLink(i);
        }
        for(int i=6;i<8;i++){
            if(j==0) mFwdKinEye->blockLink(i,0.0);
        }
    }

    
    for(int i=0;i<2;i++){
        mFwdKinWristJoints[i].resize(mFwdKinWrist[i]->getDOF());
        mFwdKinArmJoints[i].resize(mFwdKinArm[i]->getDOF());
    }
    mFwdKinEyeJoints.resize(mFwdKinEye->getDOF());
    
    for(unsigned int i=0;i<2;i++){
        mSrcToArmIndices[i].clear();
        mSrcToWristIndices[i].clear();
        mArmToIKSIndices[i].clear();
        mWristToIKSIndices[i].clear();
    }
    mSrcToEyeIndices.clear();
    mEyeToIKSIndices.clear();
    
    mSrcToIKSIndices.clear();
    for(unsigned int j=0;j<2;j++){
        for(unsigned int i=0;i<3;i++){
            mSrcToArmIndices[j].push_back(2*16+(2-i));
            mSrcToWristIndices[j].push_back(2*16+(2-i));
            mArmToIKSIndices[j].push_back(i);
            mWristToIKSIndices[j].push_back(i);
            if(j==0){
                mSrcToEyeIndices.push_back(2*16+(2-i));
                mEyeToIKSIndices.push_back(i);
            } 
        }
        for(unsigned int i=0;i<5;i++){
            mSrcToArmIndices[j].push_back(j*16+i);
            mSrcToWristIndices[j].push_back(j*16+i);
            mArmToIKSIndices[j].push_back(7*j+3+i);
            mWristToIKSIndices[j].push_back(7*j+3+i);
        }
        for(unsigned int i=5;i<7;i++){
            mSrcToArmIndices[j].push_back(j*16+i);
            mArmToIKSIndices[j].push_back(7*j+3+i);
        }
        if(j==0){
            for(unsigned int i=0;i<3;i++){
                mSrcToEyeIndices.push_back(2*16+3+i);
                mEyeToIKSIndices.push_back(2*7+3+i);
            }
        } 
    }
    for(unsigned int i=0;i<3;i++)
        mSrcToIKSIndices.push_back(2*16+(2-i));
    for(unsigned int i=0;i<7;i++)
        mSrcToIKSIndices.push_back(i);
    for(unsigned int i=0;i<7;i++)
        mSrcToIKSIndices.push_back(16+i);
    for(unsigned int i=0;i<3;i++)
        mSrcToIKSIndices.push_back(2*16+3+i);

    {
    vector<unsigned int> &array = mEyeToIKSIndices;
    for(size_t j=0;j<array.size();j++)
        cout << array[j]<<" ";
    cout << endl;
    }
    {
    vector<unsigned int> &array = mSrcToEyeIndices;
    for(size_t j=0;j<array.size();j++)
        cout << array[j]<<" ";
    cout << endl;
    }
    mIKSolver.SetSizes(mIKJointSize);
    mIKSolver.AddSolverItem(6);
    mIKSolver.AddSolverItem(6);
    mIKSolver.AddSolverItem(3);
    mIKSolver.AddSolverItem(3);
    mIKSolver.AddSolverItem(3);
    mIKSolver.AddSolverItem(3);
    mIKSolver.AddSolverItem(3);

    mIKSolver.SetVerbose(false);
    mIKSolver.SetThresholds(0.001,0.00001);

    for(int i=0;i<IKSize;i++){
        mIKSolver.SetPriority(i,i);
        mIKSolver.Enable(false,i);
    }

    mIKSolver.SetDofsIndices(mWristToIKSIndices[0],IKWristR);
    mIKSolver.SetDofsIndices(mWristToIKSIndices[1],IKWristL);
    mIKSolver.SetDofsIndices(mArmToIKSIndices[0],IKArmPosR);
    mIKSolver.SetDofsIndices(mArmToIKSIndices[1],IKArmPosL);
    mIKSolver.SetDofsIndices(mArmToIKSIndices[0],IKArmOriR);
    mIKSolver.SetDofsIndices(mArmToIKSIndices[1],IKArmOriL);
    mIKSolver.SetDofsIndices(mEyeToIKSIndices,IKEye);
    
    mJointsLimits[0].resize(mJointSize);
    mJointsLimits[1].resize(mJointSize);
    double limHigh[] = { 10,160, 80,106, 90,  0, 40,60,100,80,90,80,90,80,90,115, 10,160, 80,106, 90,  0, 40,60,100,80,90,80,90,80,90,115, 50, 30, 70, 30, 45, 55, 15, 50, 45};
    double limLow[]  = {-90,  0,-37,  6,-90,-90,-20, 0,-15, 0, 0, 0, 0, 0, 0,  0,-90,  0,-37,  6,-90,-90,-20, 0,-15, 0, 0, 0, 0, 0, 0,  0,-50,-30,-10,-40,-45,-55,-35,-50,  0};
    
    for(int i=0;i<mJointSize;i++){
        mJointsLimits[0][i] = limLow[i];
        mJointsLimits[1][i] = limHigh[i];
    }
    
    mIKJointsRest.resize(mIKJointSize);
    mIKJointsTarget.resize(mIKJointSize);
    mIKJointsPos.resize(mIKJointSize);
    
    double rest[] = { 0.0, 0.0, 0.0, -10.0, 20.0, 40.0, 30.0, -40.0, 0.0, 0.0,-10.0, 20.0, 40.0, 30.0, -40.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for(size_t i=0;i<mSrcToIKSIndices.size();i++)
        mIKJointsRest[i] = rest [i] * (PI/180.0);

    mIKJointsTarget = mIKJointsRest;
    
    mIKDofWeights.Resize(mIKJointSize);
    mIKInvDofWeights.Resize(mIKJointSize);
    mIKDofWeights.One();
    mIKInvDofWeights.One();
    
    mHandPoses[0].resize(9);
    mHandPoses[0][0] = 40;
    mHandPoses[0][1] = 0;
    mHandPoses[0][2] = 0;
    mHandPoses[0][3] = 0;
    mHandPoses[0][4] = 0;
    mHandPoses[0][5] = 0;
    mHandPoses[0][6] = 0;
    mHandPoses[0][7] = 0;
    mHandPoses[0][8] = 0;
    mHandPoses[1].resize(9);
    mHandPoses[1][0] = 40;
    mHandPoses[1][1] = 40;
    mHandPoses[1][2] = 30;
    mHandPoses[1][3] = 45;
    mHandPoses[1][4] = 30;
    mHandPoses[1][5] = 40;
    mHandPoses[1][6] = 40;
    mHandPoses[1][7] = 40;
    mHandPoses[1][8] = 70;

    hHandState[0] = false;
    hHandState[1] = false;
    
    
    bIKUseNullSpace         = true;
    bIKUseRestNullSpace     = true;
    
    mNullSpaceGain          = 0.3;
    mDesiredCartGain        = 1.0;
    mDesiredCartOriGain     = 1.0;
    bUseDesiredCartPos[0]   = 0;
    bUseDesiredCartPos[1]   = 0;

    mHandGain               = 1.0;
    
    mTime               = 0.0;
    mPrevTime           =-1.0;

    mDesiredCartPosRLastTime        = -1.0;
    mDesiredCartPosLLastTime        = -1.0;
    mDesiredCartVelRLastTime        = -1.0;
    mDesiredCartVelLLastTime        = -1.0;
    mDesiredCartWristVelRLastTime   = -1.0;
    mDesiredCartWristVelLLastTime   = -1.0;
    mDesiredCartEyeInEyeLastTime    = -1.0;
    mDesiredCartEyeLastTime         = -1.0;
}

void    RobotControllerThread::Free(){
    if(mFwdKinArm[0])   delete mFwdKinArm[0];   mFwdKinArm[0] = NULL;
    if(mFwdKinArm[1])   delete mFwdKinArm[1];   mFwdKinArm[1] = NULL;
    if(mFwdKinWrist[0]) delete mFwdKinWrist[0]; mFwdKinWrist[0] = NULL;
    if(mFwdKinWrist[1]) delete mFwdKinWrist[1]; mFwdKinWrist[1] = NULL;
    if(mFwdKinEye)      delete mFwdKinEye;      mFwdKinEye = NULL;
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

    ReadFromPorts();
    
    mTargetJointPos = mCurrentJointPos;
    mTargetJointVel = 0;

    UpdateKinChains();
    
    CheckInputsTimeout();
    
    if(mState != RCS_IDLE){
    
        


        /*
        mDesiredCartPos[0][0] = mDesiredCartPos[1][0] = inRootEyeTarget[0];
        mDesiredCartPos[0][1] = mDesiredCartPos[1][1] = inRootEyeTarget[1];
        mDesiredCartPos[0][2] = mDesiredCartPos[1][2] = inRootEyeTarget[2];
        */
            
        

        // EyeTarget
        /*
        MathLib::Matrix4 eyeRef;
        YarpMatrix4ToMatrix4(mFwdKinEyeRef,eyeRef);
        MathLib::Vector3 inEyeTarget,inRootEyeTarget;
        inEyeTarget(0) = -mDesiredCartEyeInEyePos[0];inEyeTarget(1) = -mDesiredCartEyeInEyePos[1];inEyeTarget(2) = -mDesiredCartEyeInEyePos[2];
        eyeRef.Transform(inEyeTarget,inRootEyeTarget);
        */
        /*
        mDesiredCartEyePos[0] = mFwdKinEyePose[0]-1;
        mDesiredCartEyePos[1] = mFwdKinEyePose[1];
        mDesiredCartEyePos[2] = mFwdKinEyePose[2]+1;
        mDesiredCartEyePos[0] = mFwdKinArmPose[1][0];
        mDesiredCartEyePos[1] = mFwdKinArmPose[1][1];
        mDesiredCartEyePos[2] = mFwdKinArmPose[1][2];
        */
        //mDesiredCartEyePos[0] = inRootEyeTarget[0];
        //mDesiredCartEyePos[1] = inRootEyeTarget[1];
        //mDesiredCartEyePos[2] = inRootEyeTarget[2];
        
        

        //mIKSolver.Enable(true,IKArmPosR);
        //mIKSolver.Enable(true,IKArmOriR);
        //mIKSolver.Enable(true,IKArmPosL);
        //mIKSolver.Enable(true,IKArmOriL);
        //mIKSolver.Enable(true,IKArmPosR);
        //mIKSolver.Enable(true,IKArmOriR);
        //mIKSolver.Enable(true,IKWristR);
        //mIKSolver.Enable(true,IKEye);

        //YarpPose7ToPose6(mFwdKinArmPose[0],pose);
        //pose.Print();
        //cout << "------------------"<<endl;
        //cout << mFwdKinArmPose[0].toString()<<endl;;
        //cout << mFwdKinWristPose[0].toString()<<endl;;

        ComputeVelocities();
        
        PrepareIKSolver();
        
        ApplyIKSolver();
        
        //cout <<mDesiredCartEyeVel.toString()<<endl;
        //mIKSolver.GetTargetOutput(IKEye).Print();


        mTargetJointVel[mSrcToIKSIndices[ 8]] += mDesiredWristOpt[0][0]*(180.0/PI);
        mTargetJointVel[mSrcToIKSIndices[ 9]] += mDesiredWristOpt[0][1]*(180.0/PI);
        mTargetJointVel[mSrcToIKSIndices[15]] += mDesiredWristOpt[1][0]*(180.0/PI);
        mTargetJointVel[mSrcToIKSIndices[16]] += mDesiredWristOpt[1][1]*(180.0/PI);

        // Hand and fingers
        for(int i=0;i<9;i++){
            mTargetJointVel[   7+i] = mHandGain * (mHandPoses[(hHandState[0]?0:1)][i] - mCurrentJointPos[   7+i]);
            mTargetJointVel[16+7+i] = mHandGain * (mHandPoses[(hHandState[1]?0:1)][i] - mCurrentJointPos[16+7+i]);
        }
        cout << mTargetJointVel.toString()<<endl;
        
        mTargetJointPos = mCurrentJointPos;
        
        
        switch(mState){
        case RCS_IDLE:
            /*for(int i=0;i<mJointSize;i++){
                mTargetJointPos(i) = mCurrentJointPos(i)-5;
            }*/
            break;
        case RCS_RUN:
            //mTargetJointPos = mCurrentJointPos;
            //mTargetJointVel = 0;
            break;
        }
    
    }else{
        mTargetJointPos = mCurrentJointPos;
        mTargetJointVel = 0;
    }
    
    
    
    WriteToPorts();

    mMutex.post();
}

RobotControllerThread::IKSetID RobotControllerThread::IKStringToIKSet(string str){
         if(str == "RightArm")      return IKS_RightArm;
    else if(str == "RightArmPos")   return IKS_RightArmPos;
    else if(str == "RightWrist")    return IKS_RightWrist;
    else if(str == "LeftArm")       return IKS_LeftArm;
    else if(str == "leftArmPos")    return IKS_LeftArmPos;
    else if(str == "LeftWrist")     return IKS_LeftWrist;
    else if(str == "Eye")           return IKS_Eye;
    else if(str == "Joints")        return IKS_Joints;
    else if(str == "Rest")          return IKS_Rest;
    else                            return IKS_None;

}

void    RobotControllerThread::SetIKSolverSet(IKSetID setId, bool enable){
    mMutex.wait();

    switch(setId){
    case IKS_None:
        for(int i=0;i<IKSize;i++)
            mIKSolver.Enable(false,i);
        bIKUseNullSpace = false;
        break;
    case IKS_RightArm:
        mIKSolver.Enable(enable,IKArmPosR);
        mIKSolver.Enable(enable,IKArmOriR);
        break;
    case IKS_RightArmPos:
        mIKSolver.Enable(enable,IKArmPosR);
        break;
    case IKS_RightWrist:
        mIKSolver.Enable(enable,IKWristR);
        break;
    case IKS_LeftArm:
        mIKSolver.Enable(enable,IKArmPosL);
        mIKSolver.Enable(enable,IKArmOriL);
        break;
    case IKS_LeftArmPos:
        mIKSolver.Enable(enable,IKArmPosL);
        break;
    case IKS_LeftWrist:
        mIKSolver.Enable(enable,IKWristL);
        break;
    case IKS_Eye:
        mIKSolver.Enable(enable,IKEye);
        break;
    case IKS_Joints:
        bIKUseNullSpace     = enable;
        bIKUseRestNullSpace = false;
        break;
    case IKS_Rest:
        bIKUseNullSpace     = enable;
        bIKUseRestNullSpace = enable;
        break;
    }
    
    mMutex.post();
}

void    RobotControllerThread::UpdateKinChains(){
    // Update each kin chain
    for(int j=0;j<2;j++){
        // Get angles
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
    for(size_t i=0;i<mSrcToEyeIndices.size();i++)    
        mFwdKinEyeJoints[i]   = mCurrentJointPos[mSrcToEyeIndices[i]]*(M_PI/180.0);
    mFwdKinEyePose      = mFwdKinEye->EndEffPose(mFwdKinEyeJoints);
    mFwdKinEyeJacobian  = mFwdKinEye->GeoJacobian();
    mFwdKinEyeRef       = mFwdKinEye->getH();
}

void    RobotControllerThread::PrepareIKSolver(){
    mIKSolver.SetJacobian(YarpMatrixToMatrix(mFwdKinArmJacobian[0]),                  IKArmPosR);
    mIKSolver.SetJacobian(YarpMatrixToMatrix(mFwdKinArmJacobian[1]),                  IKArmPosL);
    mIKSolver.SetJacobian(YarpMatrixToMatrix(mFwdKinArmJacobian[0]).GetRowSpace(3,3), IKArmOriR);
    mIKSolver.SetJacobian(YarpMatrixToMatrix(mFwdKinArmJacobian[1]).GetRowSpace(3,3), IKArmOriL);
    mIKSolver.SetJacobian(YarpMatrixToMatrix(mFwdKinWristJacobian[0]),                IKWristR);
    mIKSolver.SetJacobian(YarpMatrixToMatrix(mFwdKinWristJacobian[1]),                IKWristL);
    mIKSolver.SetJacobian(YarpMatrixToMatrix(mFwdKinEyeJacobian).GetRowSpace(3,3),    IKEye);

    
    // Limiting output: Max 45 deg per seconds
    Vector lim1; lim1.resize(mIKJointSize); lim1=-(45.0 *(M_PI/180.0));
    Vector lim2; lim2.resize(mIKJointSize); lim2= (45.0 *(M_PI/180.0));
    for(int i=0;i<mIKJointSize;i++){
        // Limiting speed when approaching 10 deg from joint range
        unsigned int j = mSrcToIKSIndices[i];
        if(mCurrentJointPos[j]-mJointsLimits[0][j]<10.0){
            if(mCurrentJointPos[j]-mJointsLimits[0][j]<5.0) lim1[i] = 0.0;
            else lim1[i] *= (mCurrentJointPos[j]-mJointsLimits[0][j]-5.0)/5.0;    
        }else if(mJointsLimits[1][j]-mCurrentJointPos[j]<10.0){
            if(mJointsLimits[1][j]-mCurrentJointPos[j]<5.0) lim2[i] = 0.0;
            else lim2[i] *= (mJointsLimits[1][j]-mCurrentJointPos[j]-5.0)/5.0;                    
        }
    }
    mIKSolver.SetLimits(YarpVectorToVector(lim1),YarpVectorToVector(lim2));


    mIKSolver.SetTarget(YarpVectorToVector(mDesiredCartVel[0]),                        IKArmPosR);
    mIKSolver.SetTarget(YarpVectorToVector(mDesiredCartVel[1]),                        IKArmPosL);
    mIKSolver.SetTarget(YarpVectorToVector(mDesiredCartVel[0]).GetSubVector(3,3),      IKArmOriR);
    mIKSolver.SetTarget(YarpVectorToVector(mDesiredCartVel[1]).GetSubVector(3,3),      IKArmOriL);
    mIKSolver.SetTarget(YarpVectorToVector(mDesiredCartWristVel[0]),                   IKWristR);
    mIKSolver.SetTarget(YarpVectorToVector(mDesiredCartWristVel[1]),                   IKWristL);
    mIKSolver.SetTarget(YarpVectorToVector(mDesiredCartEyeVel),                        IKEye);

    
    mIKDofWeights.One();
    mIKDofWeights(0)=mIKDofWeights(1)=mIKDofWeights(2)=0.;
    mIKDofWeights(18) = 0.0;
    mIKSolver.SetDofsWeights(mIKDofWeights);

    if(bIKUseNullSpace){
        for(int i=0;i<mIKJointSize;i++){
            if(mIKDofWeights(i)>0.05) mIKInvDofWeights(i) = mNullSpaceGain/(mIKDofWeights(i)*mIKDofWeights(i));
            else mIKInvDofWeights(i) = 0.0;
        }
    }else{
        mIKInvDofWeights.Zero();
    }
    for(size_t i=0;i<mSrcToIKSIndices.size();i++)
        mIKJointsPos(i) = mCurrentJointPos(mSrcToIKSIndices[i])*(PI/180.0);
    
    if(bIKUseRestNullSpace){
        mIKSolver.SetNullTarget((YarpVectorToVector(mIKJointsRest)-YarpVectorToVector(mIKJointsPos))^mIKInvDofWeights);
    }else{
        mIKSolver.SetNullTarget((YarpVectorToVector(mIKJointsTarget)-YarpVectorToVector(mIKJointsPos))^mIKInvDofWeights);
    }
}


void    RobotControllerThread::ApplyIKSolver(){
    mIKSolver.Solve();
    
    Vector ikOutput;
    VectorToYarpVector(mIKSolver.GetOutput(),ikOutput);
    
    for(size_t i=0;i<mSrcToIKSIndices.size();i++){
        mTargetJointVel(mSrcToIKSIndices[i]) += ikOutput[i]*(180.0/PI);
    }
}

void    RobotControllerThread::ReadFromPorts(){
    // Read data from input port
    Vector *inputVec;
    inputVec = mCurrentJointPosPort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size()==mJointSize) mCurrentJointPos = *inputVec;
        else cerr << "Bad vector size on port <currentJointPosition>: " << inputVec->size() << "!="<< mJointSize << endl;
    }
    for(int i=0;i<mCurrentJointPos.size();i++){
        mCurrentJointPos[i] = TRUNC(mCurrentJointPos[i],mJointsLimits[0][i],mJointsLimits[1][i]);
    }
    inputVec = mCurrentJointVelPort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size()==mJointSize) mCurrentJointVel = *inputVec;
        else cerr << "Bad vector size on port <currentJointVelocity>: " << inputVec->size() << "!="<< mJointSize << endl;
    }

    inputVec = mDesiredCartPosRPort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size()==6){
            mDesiredCartPos[0] = *inputVec;
            mDesiredCartPosRLastTime = mTime;
        }else if(inputVec->size()==3){
            mDesiredCartPos[0] = 0; mDesiredCartPos[0][0] = (*inputVec)[0]; mDesiredCartPos[0][1] = (*inputVec)[1]; mDesiredCartPos[0][2] = (*inputVec)[2];
            mDesiredCartPosRLastTime = mTime;
        }else cerr << "Bad vector size on port <desiredCartPosR>: " << inputVec->size() << "!= 3 or 6"<< endl;
    }
    inputVec = mDesiredCartPosLPort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size()==6){
            mDesiredCartPos[1] = *inputVec;
            mDesiredCartPosLLastTime = mTime;
        }else if(inputVec->size()==3){
            mDesiredCartPos[1] = 0; mDesiredCartPos[1][0] = (*inputVec)[0]; mDesiredCartPos[1][1] = (*inputVec)[1]; mDesiredCartPos[1][2] = (*inputVec)[2];
            mDesiredCartPosLLastTime = mTime;
        }else cerr << "Bad vector size on port <desiredCartPosL>: " << inputVec->size() << "!= 3 or 6"<< endl;
    }

    inputVec = mDesiredCartVelRPort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size()==6){
            mDesiredCartVel[0] = *inputVec;
            mDesiredCartVelRLastTime = mTime;
        }else if(inputVec->size()==3){
            mDesiredCartVel[0] = 0; mDesiredCartVel[0][0] = (*inputVec)[0]; mDesiredCartVel[0][1] = (*inputVec)[1]; mDesiredCartVel[0][2] = (*inputVec)[2];
            mDesiredCartVelRLastTime = mTime;
        }else cerr << "Bad vector size on port <desiredCartVelR>: " << inputVec->size() << "!= 3 or 6"<< endl;
    }
    inputVec = mDesiredCartVelLPort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size()==6){
            mDesiredCartVel[1] = *inputVec;
            mDesiredCartVelLLastTime = mTime;
        }else if(inputVec->size()==3){
            mDesiredCartVel[1] = 0; mDesiredCartVel[1][0] = (*inputVec)[0]; mDesiredCartVel[1][1] = (*inputVec)[1]; mDesiredCartVel[1][2] = (*inputVec)[2];
            mDesiredCartVelLLastTime = mTime;
        }else cerr << "Bad vector size on port <desiredCartVelL>: " << inputVec->size() << "!= 3 or 6"<< endl;
    }
    inputVec = mDesiredCartWristVelRPort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size()==6){
            mDesiredCartWristVel[0] = *inputVec;
            mDesiredCartWristVelRLastTime = mTime;
        }else if(inputVec->size()==3){
            mDesiredCartWristVel[0] = 0; mDesiredCartWristVel[0][0] = (*inputVec)[0]; mDesiredCartWristVel[0][1] = (*inputVec)[1]; mDesiredCartWristVel[0][2] = (*inputVec)[2];
            mDesiredCartWristVelRLastTime = mTime;
        }else if(inputVec->size()==11){
            for(int i=0;i<6;i++) mDesiredCartWristVel[0][i] = (*inputVec)[i];
            for(int i=0;i<5;i++) mDesiredWristOpt[0][i]     = (*inputVec)[i+6];
            mDesiredCartWristVelRLastTime = mTime;
        }else cerr << "Bad vector size on port <desiredCartWristVelR>: " << inputVec->size() << "!= 3 or 6 or 11"<< endl;
    }
    inputVec = mDesiredCartWristVelLPort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size()==6){
            mDesiredCartWristVel[1] = *inputVec;
            mDesiredCartWristVelLLastTime = mTime;
        }else if(inputVec->size()==3){
            mDesiredCartWristVel[1] = 0; mDesiredCartWristVel[1][0] = (*inputVec)[0]; mDesiredCartWristVel[1][1] = (*inputVec)[1]; mDesiredCartWristVel[1][2] = (*inputVec)[2];
            mDesiredCartWristVelLLastTime = mTime;
        }else if(inputVec->size()==11){
            for(int i=0;i<6;i++) mDesiredCartWristVel[1][i] = (*inputVec)[i];
            for(int i=0;i<5;i++) mDesiredWristOpt[1][i]     = (*inputVec)[i+6];
            mDesiredCartWristVelLLastTime = mTime;
        }else cerr << "Bad vector size on port <desiredCartWristVelL>: " << inputVec->size() << "!= 3 or 6 or 11"<< endl;
    }

    inputVec = mDesiredCartEyeInEyePort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size()==3){
            mDesiredCartEyeInEyePos = *inputVec;
            mDesiredCartEyeInEyeLastTime = mTime;
        }else if(inputVec->size()==6){
            mDesiredCartEyeInEyePos[0] = (*inputVec)[0]; mDesiredCartEyeInEyePos[1] = (*inputVec)[1]; mDesiredCartEyeInEyePos[2] = (*inputVec)[2]; 
            mDesiredCartEyeInEyeLastTime = mTime;
        }else cerr << "Bad vector size on port <desiredCartEyeInEye>: " << inputVec->size() << "!= 3 or 6"<< endl;
    }

    inputVec = mDesiredCartEyePort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size()==3){
            mDesiredCartEyePos = *inputVec;
            mDesiredCartEyeLastTime = mTime;
        }else if(inputVec->size()==6){
            mDesiredCartEyePos[0] = (*inputVec)[0]; mDesiredCartEyePos[1] = (*inputVec)[1]; mDesiredCartEyePos[2] = (*inputVec)[2]; 
            mDesiredCartEyeLastTime = mTime;
        }else cerr << "Bad vector size on port <desiredCartEye>: " << inputVec->size() << "!= 3 or 6"<< endl;
    }
}

void    RobotControllerThread::CheckInputsTimeout(){
    if(mTime-mDesiredCartPosRLastTime        >= INPUTS_TIMEOUT)  YarpPose7ToYarpPose6(mFwdKinArmPose[0],mDesiredCartPos[0]);
    if(mTime-mDesiredCartPosLLastTime        >= INPUTS_TIMEOUT)  YarpPose7ToYarpPose6(mFwdKinArmPose[1],mDesiredCartPos[1]);
    if(mTime-mDesiredCartVelRLastTime        >= INPUTS_TIMEOUT)  mDesiredCartVel[0]      = 0.0;
    if(mTime-mDesiredCartVelLLastTime        >= INPUTS_TIMEOUT)  mDesiredCartVel[1]      = 0.0;
    if(mTime-mDesiredCartWristVelRLastTime   >= INPUTS_TIMEOUT)  mDesiredCartWristVel[0] = 0.0;
    if(mTime-mDesiredCartWristVelLLastTime   >= INPUTS_TIMEOUT)  mDesiredCartWristVel[1] = 0.0;

    if(mTime-mDesiredCartEyeLastTime            > INPUTS_TIMEOUT){
        if(mTime-mDesiredCartEyeInEyeLastTime       > INPUTS_TIMEOUT){
            mDesiredCartEyeInEyePos = 0; mDesiredCartEyeInEyePos[2] = 0.2;
        }  
        ConvertEyeInEyeTarget();
    }
}

void    RobotControllerThread::WriteToPorts(){
    // Write data to output port
    {
        //Vector &outputVec = mTargetJointPosPort.prepare();
        //outputVec = mTargetJointPos;
        //mTargetJointPosPort.write();
    }
    {
        Vector &outputVec = mTargetJointVelPort.prepare();
        outputVec = mTargetJointVel;
        mTargetJointVelPort.write();
    }
    {
        Matrix &outputMat = mCurrentWristRefRPort.prepare();
        outputMat = mFwdKinWristRef[0];
        mCurrentWristRefRPort.write();
    }
    {
        Matrix &outputMat = mCurrentWristRefLPort.prepare();
        outputMat = mFwdKinWristRef[1];
        mCurrentWristRefLPort.write();
    }
    {
        Vector &outputVec = mCurrentCartPosRPort.prepare();
        YarpPose7ToYarpPose6(mFwdKinArmPose[0],outputVec);
        mCurrentCartPosRPort.write();
    }
    {
        Vector &outputVec = mCurrentCartPosLPort.prepare();
        YarpPose7ToYarpPose6(mFwdKinArmPose[1],outputVec);
        mCurrentCartPosLPort.write();
    }
    {
        Vector &outputVec = mCurrentCarEyeTargetPort.prepare();
        outputVec = mDesiredCartEyePos;
        mCurrentCarEyeTargetPort.write();
    }
    
}
void    RobotControllerThread::SetState(State state){
    mMutex.wait();
    mState = state; 
    mMutex.post();
}

void    RobotControllerThread::ConvertEyeInEyeTarget(){
    MathLib::Matrix4 eyeRef;
    YarpMatrix4ToMatrix4(mFwdKinEyeRef,eyeRef);
    MathLib::Vector3 inEyeTarget,inRootEyeTarget;
    YarpVector3ToVector3(mDesiredCartEyeInEyePos,inEyeTarget);
    eyeRef.Transform(inEyeTarget,inRootEyeTarget);
    Vector3ToYarpVector3(inRootEyeTarget,mDesiredCartEyePos);
}

void    RobotControllerThread::ComputeVelocities(){
    // Eye-Head velocity
    MathLib::Vector3 up(0,0,-1);
    MathLib::Vector3 wTarget;
    MathLib::Vector3 vTarget(mDesiredCartEyePos[0]-mFwdKinEyePose[0],mDesiredCartEyePos[1]-mFwdKinEyePose[1],mDesiredCartEyePos[2]-mFwdKinEyePose[2]);
    
    MathLib::Matrix3 hrot,res1,res2;
    hrot.SetColumn(vTarget,2);
    hrot.SetColumn(up,1);
    hrot.SetColumn(vTarget.Cross(up),0);
    hrot.Normalize(2);
    YarpMatrix4ToMatrix3(mFwdKinEyeRef,res1);
    res2 = (hrot*res1.Transpose());
    res2.GetExactRotationAxis(wTarget);
    Vector3ToYarpVector3(wTarget,mDesiredCartEyeVel);

    
    // Hand velocities
    bool bUseDesiredCartPos[2];
    bUseDesiredCartPos[0] = (mTime-mDesiredCartPosRLastTime < INPUTS_TIMEOUT);
    bUseDesiredCartPos[1] = (mTime-mDesiredCartPosLLastTime < INPUTS_TIMEOUT);

    for(int i=0;i<2;i++){
        if(bUseDesiredCartPos[i]){
            
            MathLib::Vector3 vel,avel,pos,ori,cpos,cori;
            YarpPose7ToPose6(mFwdKinArmPose[i], cpos,cori);
            YarpPose6ToPose6(mDesiredCartPos[i],pos, ori);
            
            pos.Sub(cpos,vel);
            vel *= mDesiredCartGain;
            
            MathLib::Matrix3 src,dst,res;
            src.RotationV(cori);
            dst.RotationV(ori);
            dst.Mult(src.Transpose(),res);
            res.GetExactRotationAxis(avel);
            
            avel *= mDesiredCartGain; 
            //if(bUseDesiredCartPos[i]==1)
            //Pose6ToYarpPose6(vel,avel,mDesiredCartVel[i]);
            //else
            AddPose6ToYarpPose6(vel,avel,mDesiredCartVel[i]);
        }
    }
}
void    RobotControllerThread::SetHandPose(bool rightHand, bool open){
    hHandState[(rightHand?0:1)] = open;
}

