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

#include "VelocityController.h"
#include <cstring>
#include <iostream>
using namespace std;

VelocityController::VelocityController(){
    bIsReady = false;
}

VelocityController::~VelocityController(){
    Free();
}


bool VelocityController::Init(PolyDriver *driver, const char* name, const char* basename){
    if(driver==NULL){
        fprintf(stderr,"Warning: Null driver\n");
    }
    mDriver = driver;
    
    if(basename!=NULL){
        strcpy(mBaseName,basename);
    }else{
        mBaseName[0] = 0;
    }
    if(name!=NULL){
        strcpy(mName,name);
    }else{
        strcpy(mName,"VC");   
    }
    

  
    bool bOk = true;
    if(mDriver){
        bOk &= mDriver->view(mEncoders);  
        bOk &= mDriver->view(mVelocityController);
        bOk &= mDriver->view(mLimitsController);
    }else{
        mEncoders           = NULL;
        mLimitsController   = NULL;
        mVelocityController = NULL;         
    }
    if(!bOk){
        fprintf(stderr,"Error: Problem getting drivers interfaces\n");
        return false;    
    }
    
    fprintf(stderr,"********************************************\n");
    fprintf(stderr,"Starting Velocity Controller: <%s/%s>\n",mBaseName,mName);
    if(mVelocityController){
        mVelocityController->getAxes(&mJointsSize);
    }else{
        fprintf(stderr,"  Fake one...\n");
        mJointsSize = 1;
    }
    fprintf(stderr,"  # Joints: %d\n",mJointsSize);

    Vector accs; accs.resize(mJointsSize);
    accs = 1000000;
    if(mVelocityController){
        mVelocityController->setRefAccelerations(accs.data());
    }

    mJointsPos.resize(mJointsSize);
    mJointsVel.resize(mJointsSize);
    mJointsAcc.resize(mJointsSize);
    mJointsTargetPos.resize(mJointsSize);
    mJointsTargetVel.resize(mJointsSize);
    mJointsOutputVel.resize(mJointsSize);
    mJointsPrevOutputVel.resize(mJointsSize);
    mJointsMask.resize(mJointsSize);
    mJointsError.resize(mJointsSize);
    mJointsKp.resize(mJointsSize);
    mJointsKd.resize(mJointsSize);
    mJointsPosLimits[0].resize(mJointsSize);
    mJointsPosLimits[1].resize(mJointsSize);
    mJointsVelLimits[0].resize(mJointsSize);
    mJointsVelLimits[1].resize(mJointsSize);
    mJointsRange.resize(mJointsSize);    
    for(int i=0;i<mJointsSize;i++){
        if(mLimitsController){
            mLimitsController->getLimits(i,mJointsPosLimits[0].data()+i,mJointsPosLimits[1].data()+i);
        }
        mJointsRange[i] = mJointsPosLimits[1][i] - mJointsPosLimits[0][i];
    }
    
    mJointsRest.resize(mJointsSize);
    
    mJointsKp = 2.0;
    mJointsKd = 0;    
    
    mJointsMask = 1.0;
    
    mJointsTargetPos = 0;    
    mJointsTargetVel = 0;    
    mJointsError     = 0;
    
    mMode.resize(mJointsSize);
    for(int i=0;i<mJointsSize;i++){
        mMode[i]    = VC_IDLE;
    }

    mTime               = 0.0;
    mPrevTime           =-1.0;

    mCommandTimeout     = 0.1;
    mMinimumLoopTime    = 0.005;
    mCummulativeDt      = 0.0;    
    mLastPosCommandTime = mCommandTimeout + 0.1;
    mLastVelCommandTime = mCommandTimeout + 0.1;
    
    mJointsVelLimits[0] = -60.0;
    mJointsVelLimits[1] = +60.0;
    
    bPosTargetSet           = false;
    bVelTargetSet           = false;
    bPosTimeoutPause        = false;
    bVelTimeoutPause        = false;
    bUseShoulderDecoupling  = false;
    
    mSpeedFactor = 1.0;
    
    bMoveToRest = false;
    bFirst      = true;
    bIsReady    = true;
    
    fprintf(stderr,"********************************************\n");
    return true;
}


void VelocityController::Free(){
    Stop();
    bIsReady = false;
}

void VelocityController::SetShoulderDecoupling(bool set){
    bUseShoulderDecoupling = true;
}

void VelocityController::Update(){
    if(!bIsReady) return;
    if(mPrevTime<0.0){
        mPrevTime = Time::now();
        return;
    }else{
        mPrevTime = mTime;    
    }
    
    mTime       = Time::now();
    double dt   = mTime - mPrevTime;
    
    Update(dt);    
    
}

void VelocityController::Update(double dt){
    if(!bIsReady) return;


    if(dt < mMinimumLoopTime){
        mCummulativeDt += dt;
        cout << "Too fast: "<< dt << "<" <<mMinimumLoopTime<<endl;
        if(mCummulativeDt < mMinimumLoopTime){
            mMutex.post();
            return;    
        }else{
            dt = mCummulativeDt;
        }        
    }
    mCummulativeDt = 0.0;
    double invDt = 1.0/dt;

    mMutex.wait();

    if(mEncoders){
        mEncoders->getEncoders              (mJointsPos.data());
        mEncoders->getEncoderSpeeds         (mJointsVel.data());
        mEncoders->getEncoderAccelerations  (mJointsAcc.data());
    }
    
    if(bFirst){
        mDecouplingBox.ResetSetPoint(mJointsPos);
        mJointsRest = mJointsPos;
        mJointsPrevOutputVel = 0.0;
        bFirst = false;
    }


    // Get commands
    if(bPosTargetSet){
        mLastPosCommandTime = 0.0;
        bPosTargetSet       = false;
    }else{
        mLastPosCommandTime += dt;
    }
    if(bVelTargetSet){
        mLastVelCommandTime = 0.0;
        bVelTargetSet       = false;
    }else{
        mLastVelCommandTime += dt;        
    }
    // Check for last command time
    if(mLastPosCommandTime>mCommandTimeout){
        if(!bPosTimeoutPause){
            cout << mBaseName<<"/"<<mName<<": Timeout: pausing position control..."<<endl;
            bPosTimeoutPause = true;            
        }
        mJointsTargetPos = mJointsPos;
    }else{
        if(bPosTimeoutPause){
            cout << mBaseName<<"/"<<mName<<": Timeout: resuming position control..."<<endl;
            bPosTimeoutPause = false;
        }                    
    }    
    // Check for last command time
    if(mLastVelCommandTime>mCommandTimeout){
        if(!bVelTimeoutPause){
            cout << mBaseName<<"/"<<mName<<": Timeout: pausing velocity control..."<<endl;
            bVelTimeoutPause = true;            
        }
        mJointsTargetVel = 0.0;
    }else{
        if(bVelTimeoutPause){
            cout << mBaseName<<"/"<<mName<<": Timeout: resuming velocity control..."<<endl;
            bVelTimeoutPause = false;
        }                    
    }    

    if(bMoveToRest){
        mJointsTargetPos = mJointsRest;
        mJointsTargetVel = 0.0; 
    }

    for(int i=0;i<mJointsSize;i++){
        double currError = 0.0;
        double kdPart    = 0.0;
        double kpPart    = 0.0;
        
        double mVelTau   = 0.1;
        double dtOnTau   = dt/mVelTau;
        
        switch(mMode[i]){
        case VC_IDLE:
            mDecouplingBox.ResetSetPoint(mJointsPos);
            mJointsTargetPos[i] = mJointsPos[i];
            mJointsTargetVel[i] = 0.0;
            mJointsOutputVel[i] = 0.0;
            break;
        case  VC_ACTIVE:
            // Velocity control
            double outputVel =  mJointsTargetVel[i];
            //mJointsOutputVel[i] =
            
            // Position control
            if(mJointsTargetPos[i]<mJointsPosLimits[0][i]){
                mJointsTargetPos[i] = mJointsPosLimits[0][i];
            }else if(mJointsTargetPos[i]>mJointsPosLimits[1][i]){
                mJointsTargetPos[i] = mJointsPosLimits[1][i];
            }
            
            currError    = mJointsTargetPos[i]-mJointsPos[i];
            if(fabs(currError)<0.2) currError = 0.0;
            kdPart = mJointsKd[i]*(currError - mJointsError[i])*invDt;
                
            mJointsError[i]     = currError;
            kpPart = mJointsKp[i]*(mJointsError[i]);
            //cout << kdPart<<" ";
            outputVel += kpPart + kdPart;

            mJointsOutputVel[i] = mJointsPrevOutputVel[i] + (-mJointsPrevOutputVel[i] + outputVel)*dtOnTau;
            //mJointsPrevOutputVel[i]
            
            double minVelLimit = mJointsVelLimits[0][i];
            double maxVelLimit = mJointsVelLimits[1][i];
            
            if(mJointsPos[i]-mJointsPosLimits[0][i]<5.0){
                if(mJointsPos[i]-mJointsPosLimits[0][i]<0.0) minVelLimit = 0.0;
                else minVelLimit *= (mJointsPos[i]-mJointsPosLimits[0][i])/5.0;
            }else if(mJointsPosLimits[1][i]-mJointsPos[i]<5.0){
                if(mJointsPosLimits[1][i]-mJointsPos[i]<0.0) maxVelLimit = 0.0;
                else maxVelLimit *= (mJointsPosLimits[1][i]-mJointsPos[i])/5.0; 
            }

            if(mJointsOutputVel[i]<minVelLimit){
                mJointsOutputVel[i] = minVelLimit;
            }else if(mJointsOutputVel[i]>maxVelLimit){
                mJointsOutputVel[i] = maxVelLimit;
            }
            break;
        }
    }
    //cout << endl;
    
    mJointsPrevOutputVel = mJointsOutputVel;
    //cout << " ------------"<<endl;
    //cout << mJointsOutputVel.toString()<<endl;

    if(bUseShoulderDecoupling){
        mDecouplingBox.mDecFactors[0] = -0.002335;
        mDecouplingBox.mDecFactors[1] = -0.001662;
        mDecouplingBox.mDecFactors[2] =  0.001196;
        //Vector ov = mJointsOutputVel;
        mDecouplingBox.Decouple(mJointsOutputVel, mJointsOutputVel, dt);        
        //cout <<ov[0]-mJointsOutputVel[0]<<" "<<ov[1]-mJointsOutputVel[1]<<" "<<ov[2]-mJointsOutputVel[2]<<" " <<endl;
    }
    //cout << mJointsOutputVel.toString()<<endl;

    
    for(int i=0;i<mJointsSize;i++)
        mJointsOutputVel[i] *= mSpeedFactor;
    
    if(mVelocityController)
        mVelocityController->velocityMove(mJointsOutputVel.data());

    
    mMutex.post();
}

void VelocityController::Stop(){
    if(!bIsReady) return;
    
    mMutex.wait();
    
    if(mEncoders)
        mEncoders->getEncoders(mJointsPos.data());
    for(int i=0;i<mJointsSize;i++){
        mMode[i] = VC_IDLE;
        mJointsTargetPos[i] = mJointsPos[i];
        mJointsTargetVel[i] = 0.0;
    }   
    cout << "Stopping motors"<<endl;
    if(mVelocityController)
        mVelocityController->velocityMove(mJointsTargetVel.data());
        
    mMutex.post();
}

char* VelocityController::GetBaseName(){
    return mBaseName;
}

void    VelocityController::GetPosition(Vector &pos){
    pos = mJointsPos;
}
void    VelocityController::GetVelocity(Vector &vel){
    vel = mJointsVel;
}

void    VelocityController::SetPositionTarget(Vector &target){
    mMutex.wait();

    int mx = (target.size()>=mJointsSize?mJointsSize:target.size());
    for(int i=0;i<mx;i++){
        mJointsTargetPos[i] = target[i];   
    }
    for(int i=mx;i<mJointsSize;i++){
        mJointsTargetPos[i] = mJointsPos[i];   
    }
    bPosTargetSet = true;
    
    mMutex.post();
}
void    VelocityController::SetVelocityTarget(Vector &target){
    mMutex.wait();
    
    int mx = (target.size()>=mJointsSize?mJointsSize:target.size());
    for(int i=0;i<mx;i++){
        mJointsTargetVel[i] = target[i];   
    }
    for(int i=mx;i<mJointsSize;i++){
        mJointsTargetVel[i] = 0.0;   
    }
    bVelTargetSet = true;
    
    mMutex.post();
}

void    VelocityController::SetMask(Vector &mask){
    int mx = (mask.size()>=mJointsSize?mJointsSize:mask.size());
    mJointsMask = 0.0;
    for(int i=0;i<mx;i++){
        mJointsMask[i] = (mask[i]>0.0?1.0:0.0);   
    }
}
void    VelocityController::SetMaskAll(){
    for(int i=0;i<mJointsSize;i++){
        mJointsMask[i]=1.0;
    }
}
void    VelocityController::SetMaskNone(){
    for(int i=0;i<mJointsSize;i++){
        mJointsMask[i]=0.0;
    }
}
     
void    VelocityController::SetControlMode(VCMode mode){
    mMutex.wait();
    bMoveToRest = false;
    for(int i=0;i<mJointsSize;i++){
        if(mJointsMask[i]!=0.0){
            if(mMode[i] != mode){
                mMode[i] = mode;
                if(mode != VC_IDLE)
                    mJointsError[i] = 0;
            }
        }   
    }
    mMutex.post();
}
void    VelocityController::SetKp(double kps){
    mMutex.wait();
    kps = (kps>0.0?kps:0.0);
    for(int i=0;i<mJointsSize;i++){
        if(mJointsMask[i]!=0.0){
            mJointsKp[i] = kps;
        }   
    }
    mMutex.post();
}
void    VelocityController::SetKd(double kds){
    mMutex.wait();
    kds = (kds>0.0?kds:0.0);
    for(int i=0;i<mJointsSize;i++){
        if(mJointsMask[i]!=0.0){
            mJointsKd[i] = kds;
        }   
    }
    mMutex.post();
}

void    VelocityController::SetCommandTimeout(double timeout){
    mMutex.wait();
    mCommandTimeout = (timeout>0.0?timeout:0.0);
    mMutex.post();
}
void    VelocityController::SetMinimumLoopTime(double time){
    mMutex.wait();
    mMinimumLoopTime = (time>0.0?time:0.0);    
    mMutex.post();
}
int     VelocityController::GetJointsSize(){
    return mJointsSize;    
}

void    VelocityController::MoveToRest(){
    bMoveToRest = true;
}
void    VelocityController::SetSpeedFactor(double fact){
    mSpeedFactor = fact;
}