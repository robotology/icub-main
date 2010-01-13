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

 
#include "GaussianMixtureModelThread.h"
#include "YarpTools/YarpMathLibInterface.h"

#include <string.h>


GaussianMixtureModelThread::GaussianMixtureModelThread(int period, const char* baseName)
:RateThread(period)
{
    mPeriod = period;
    strncpy(mBaseName,baseName,256);
}

GaussianMixtureModelThread::~GaussianMixtureModelThread()
{}

void    GaussianMixtureModelThread::Init(){
    bGMMIsReady = false;

    mGMMInternalTimeSpan    =  1.0;
    mGMMReproTime           = 10.0;
    
    mIOSize                 = 6;
    mGMMIOSize              = 7;
    
    mGMMInComp.Resize(1); 
    mGMMInComp(0) = 0;
    mGMMOutComp.Resize(mGMMIOSize);
    for(int i=0;i<mGMMIOSize;i++)
        mGMMOutComp(i) = i+1;
    mGMMSigmas = new MathLib::Matrix();
    
    mGMMInputV.Resize(1);
    mGMMInputV(0)  = 0;
    mGMMInput.Resize(1,1);
    mGMMInput(0,0) = 0;

    mGMMLambda.Resize(mGMMIOSize);
    mGMMTarget.Resize(1,mGMMIOSize);
    mGMMTargetV.Resize(mGMMIOSize);
    
    mGMMCurrState.Resize(mGMMIOSize);

    mGMMLambdaTreshold  = 2.0;
    mGMMLambdaTau       = 0.5;
}
void    GaussianMixtureModelThread::Free(){
    
}

bool GaussianMixtureModelThread::threadInit()
{
    Init();
    
    char portName[256];
    snprintf(portName,256,"/%s/input",mBaseName);
    mInputPort.open(portName);

    snprintf(portName,256,"/%s/output",mBaseName);
    mOutputPort.open(portName);

    mTime               = 0.0;
    mPrevTime           =-1.0;

    mInputLastTime      =-1.0;
    
    return true;
}

void GaussianMixtureModelThread::threadRelease()
{
    Free();
    
    mInputPort.close();
    mOutputPort.close();
}

void GaussianMixtureModelThread::run()
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
    Vector *inputVec = mInputPort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size()==mIOSize){
            YarpVectorToVector(*inputVec,mInputVector);
            mInputLastTime = mTime;
        }else cerr << "Error: Bad input size: "<<inputVec->size()<<" != "<<mIOSize<<endl;
    }
    
    if(mTime-mInputLastTime > 0.5)
        mState = GMM_IDLE;
        
    bool bGMMIsRunning = false;
    
    bool bStateChanged = (mPrevState != mState);
    if(bStateChanged){
        cout << "State change: <"<<mPrevState<<"> -> <"<<mState<<">"<<endl;
    }
    
    mNextState = mState;
    switch(mState){
    case GMM_IDLE:
        break;
    case GMM_REPRO_INIT:
        mGMMTime        = 0.0;
        mGMMInputV(0)   = mGMMInternalTimeSpan*mGMMTime/mGMMReproTime;
        mGMMInput(0,0)  = mGMMInputV(0); 
        {
            MathLib::Vector lambda1,lambda2;
            
            // Choosing best regression quaternion
            Pose6ToQPose(mInputVector, mGMMCurrState,false);
            lambda1 = mGMM.getRegressionOffset(mGMMInputV, mGMMCurrState, mGMMInComp, mGMMOutComp);

            Pose6ToQPose(mInputVector, mGMMCurrState,true);
            lambda2 = mGMM.getRegressionOffset(mGMMInputV, mGMMCurrState, mGMMInComp, mGMMOutComp);
        
            if(lambda1.Norm2()<lambda2.Norm2()) mGMMLambda = lambda1;
            else                                mGMMLambda = lambda2;
        }
        bGMMIsRunning = true;

        break;

    case GMM_REPRO_RUN:
        mGMMTime       += dt;
        mGMMInputV(0)   = mGMMInternalTimeSpan*mGMMTime/mGMMReproTime;
        mGMMInput(0,0)  = mGMMInputV(0); 
        {
            double norm = mGMMLambda.Norm();
            if(norm > mGMMLambdaTreshold){
                mGMMLambda += (mGMMLambda*(mGMMLambdaTreshold/norm-1.0))*dt/mGMMLambdaTau;
                //mGMMLambda += (-mGMMLambda + mGMMLambda*(mGMMLambdaTreshold/norm))*dt/mGMMLambdaTau;
                //for(int i=0;i<7;i++)
                //    mGMMLambda(i) += (-mGMMLambda(i) + mGMMLambda(i)/(norm/2.0))*dt/1.0;
            }
        }
        bGMMIsRunning = true;
        
        if(mGMMTime>=mGMMReproTime)
            mNextState = GMM_REPRO_END;
        
        break;

    case GMM_REPRO_END:
        mGMMTime        = mGMMReproTime;
        mGMMInputV(0)   = mGMMInternalTimeSpan;
        mGMMInput(0,0)  = mGMMInputV(0); 
        
        bGMMIsRunning = true;
        break;
    }
    
    mPrevState  = mState;
    mState      = mNextState;
    
    if(bGMMIsRunning){
        mGMMTarget = mGMM.doOffsetRegression(mGMMInput, mGMMLambda, mGMMSigmas, mGMMInComp, mGMMOutComp);
        QPoseToPose6(mGMMTarget.GetRow(0),mGMMTargetV);
    }
    
    // Write data to output port
    if((bGMMIsReady)&&(bGMMIsRunning)){
        Vector &outputVec = mOutputPort.prepare();
        VectorToYarpVector(mGMMTargetV,outputVec);
        mOutputPort.write();
    }
    mMutex.post();
}

void    GaussianMixtureModelThread::Load(const char* modelFilename){
    mMutex.wait();
    
    bGMMIsReady = mGMM.loadParams(modelFilename);

    mMutex.post();
}

void    GaussianMixtureModelThread::Save(const char* modelFilename){
    mMutex.wait();
    if(bGMMIsReady)
        mGMM.saveParams(modelFilename);
    
    mMutex.post();
}












MathLib::Vector& AxisToQuat(const MathLib::Vector& axis, MathLib::Vector& quat, bool inv){
    quat.Resize(4);
    double angle = axis.Norm();
    double s     = sin(angle/2.0) * (inv?-1.0:1.0);
    double c     = cos(angle/2.0) * (inv?-1.0:1.0);
    if(angle<1e-6){
        quat(0) = axis.At(0) * s;
        quat(1) = axis.At(1) * s;
        quat(2) = axis.At(2) * s;
        quat(3) = (inv?-1.0:1.0);
    }else{
        s /= angle;
        quat(0) = axis.At(0) * s;
        quat(1) = axis.At(1) * s;
        quat(2) = axis.At(2) * s;
        quat(3) = c;
    }
    return quat;
} 

MathLib::Vector& QuatToAxis(const MathLib::Vector& quat, MathLib::Vector& axis){
    axis.Resize(3);
    double angle = 2.0 * acos(TRUNC(quat.At(3),-1.0,1.0));
    double n = sqrt(1.0-quat.At(3)*quat.At(3));
    if((n)<1e-6){
        axis(0) = 0.0;    
        axis(1) = 0.0;    
        axis(2) = 0.0;    
    }else{    
        n = angle/n;
        axis(0) = quat.At(0) *n;
        axis(1) = quat.At(1) *n;
        axis(2) = quat.At(2) *n;
    }
    return axis;   
} 

MathLib::Vector& Pose6ToQPose(const MathLib::Vector& pose, MathLib::Vector& qpose,bool inv){
    qpose.Resize(7);
    MathLib::Vector a(3),qa(4);
    for(int i=0;i<3;i++) a(i)       = pose.At(i+3);
    AxisToQuat(a,qa,inv);
    for(int i=0;i<3;i++) qpose(i)   = pose.At(i);
    for(int i=0;i<4;i++) qpose(i+3) = qa(i);
    return qpose;        
}

MathLib::Vector& QPoseToPose6(const MathLib::Vector& qpose, MathLib::Vector& pose){
    pose.Resize(6);
    MathLib::Vector a(3),qa(4);
    for(int i=0;i<4;i++) qa(i)      = qpose.At(i+3);
    QuatToAxis(qa,a);
    for(int i=0;i<3;i++) pose(i)    = qpose.At(i);
    for(int i=0;i<3;i++) pose(i+3)  = a(i);
    return pose;        
}
