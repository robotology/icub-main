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

 
#include "RefTransformThread.h"

#include <string.h>

#include "YarpTools/YarpMathLibInterface.h"

#define INPUT_TIMEOUT   0.5

RefTransformThread::RefTransformThread(int period, const char* baseName)
:RateThread(period)
{
    mPeriod = period;
    strncpy(mBaseName,baseName,256);
}

RefTransformThread::~RefTransformThread()
{}

bool RefTransformThread::threadInit()
{
    char portName[256];
    snprintf(portName,256,"/%s/inputPose",mBaseName);
    mInputPosePort.open(portName);
    snprintf(portName,256,"/%s/outputPose",mBaseName);
    mOutputPosePort.open(portName);
    snprintf(portName,256,"/%s/inputPoseInRef",mBaseName);
    mInputPoseInRefPort.open(portName);
    snprintf(portName,256,"/%s/outputPoseInRef",mBaseName);
    mOutputPoseInRefPort.open(portName);
    snprintf(portName,256,"/%s/inputRef",mBaseName);
    mInputRefPort.open(portName);

    /*
    MathLib::Vector3        mInputPosePos;
    MathLib::Vector3        mInputPoseOri;
    MathLib::Vector3        mOutputPosePos;
    MathLib::Vector3        mOutputPoseOri;
    MathLib::Vector3        mInputPoseInRefPos;
    MathLib::Vector3        mInputPoseInRefOri;
    MathLib::Vector3        mOutputPoseInRefPos;
    MathLib::Vector3        mOutputPoseInRefOri;
    MathLib::Vector3        mInputRefPos;
    MathLib::Vector3        mInputRefOri;
    */
    
    mRef.Identity();
    mRefNoOri.Identity();
    mInvRef.Identity();
    mInvRefNoOri.Identity();
    
    bBypass         = true;
    bLocked         = false;
    bEnableOrient   = false;
    bRun            = true;

    mTime                       = 0.0;
    mPrevTime                   =-1.0;
    
    mInputPoseLastTime          =-1.0;
    mInputPoseInRefLastTime     =-1.0;
    mInputRefPortLastTime       =-1.0;
    
    return true;
}

void RefTransformThread::threadRelease()
{
    mInputPosePort.close();
    mOutputPosePort.close();
    mInputPoseInRefPort.close();
    mOutputPoseInRefPort.close();
    mInputRefPort.close();
}

void RefTransformThread::run()
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
    
    if(!bRun){
        mMutex.post();
        return;
    }

    // Read data from input port
    Vector *inputVec;
    inputVec = mInputPosePort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size() == 6){
            YarpPose6ToPose6(*inputVec,mInputPosePos,mInputPoseOri);
            mInputPoseLastTime = mTime;
        }else if(inputVec->size() == 3){
            mInputPoseOri.Zero(); for(int i=0;i<3;i++) mInputPosePos[i] = (*inputVec)[i];
            mInputPoseLastTime = mTime;
        }else cerr << "Bad input pose size"<<endl;
    }
    inputVec = mInputPoseInRefPort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size() == 6){
            YarpPose6ToPose6(*inputVec,mInputPoseInRefPos,mInputPoseInRefOri);
            mInputPoseInRefLastTime = mTime;
        }else if(inputVec->size() == 3){
            mInputPoseInRefOri.Zero(); for(int i=0;i<3;i++) mInputPoseInRefPos[i] = (*inputVec)[i];
            mInputPoseInRefLastTime = mTime;
        }else cerr << "Bad input pose in ref size"<<endl;
    }
    inputVec = mInputRefPort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size() == 6){
            YarpPose6ToPose6(*inputVec,mInputRefPos,mInputRefOri);
            mInputRefPortLastTime = mTime;
        }else if(inputVec->size() == 3){
            mInputRefOri.Zero(); for(int i=0;i<3;i++) mInputRefPos[i] = (*inputVec)[i];
            mInputRefPortLastTime = mTime;
        }else cerr << "Bad input pose in ref size"<<endl;
    }

    // Calculations
    if(!bLocked){
        if((mTime - mInputRefPortLastTime) < INPUT_TIMEOUT){
            MathLib::Matrix3 ori;
            ori.RotationV(mInputRefOri);
            mRef.Transformation(ori, mInputRefPos);
            mInvRef.InverseTransformation(ori, mInputRefPos);
            ori.Identity();
            mRefNoOri.Transformation(ori, mInputRefPos);
            mInvRefNoOri.InverseTransformation(ori, mInputRefPos);
            cout << "Got Ref :            "<< mInputRefPos[0] <<" "<< mInputRefPos[1] <<" "<< mInputRefPos[2] <<" "<<mInputRefOri[0] <<" "<< mInputRefOri[1] <<" "<< mInputRefOri[2] << endl;
        }
    }
    
    if(bBypass){
        mOutputPoseInRefPos = mInputPosePos;
        mOutputPoseInRefOri = mInputPoseOri;
        mOutputPosePos      = mInputPoseInRefPos;
        mOutputPoseOri      = mInputPoseInRefOri;
        //cout << "bypass"<<endl;

    }else{
        MathLib::Matrix4 src,res;
        MathLib::Matrix3 ori;

        ori.RotationV(mInputPoseOri);
        src.Transformation(ori, mInputPosePos);
        //src.Print();
        
        if(bEnableOrient){
            mInvRef.Transform(src,res);
            //res.Print();
            //res = mInvRef*src;
            //res.Print();
        }   
        else{
            mInvRefNoOri.Transform(src,res);
            //mInvRefNoOri.Print();
        }                
        //res.Print();
        
        res.GetTranslation(mOutputPoseInRefPos);
        res.GetOrientation(ori);
        ori.GetExactRotationAxis(mOutputPoseInRefOri);

        ori.RotationV(mInputPoseInRefOri);
        src.Transformation(ori, mInputPoseInRefPos);

        if(bEnableOrient)   mRef.Transform(src,res);
        else                mRefNoOri.Transform(src,res);
        
        res.GetTranslation(mOutputPosePos);
        res.GetOrientation(ori);
        ori.GetExactRotationAxis(mOutputPoseOri);

        //cout << "no bypass"<<endl;
    }

    cout << "Status: enabled: "<< !bBypass<<" locked: "<<bLocked<< "ori: "<<bEnableOrient<<endl;
    if(bEnableOrient)   mRef.Print();
    else                mRefNoOri.Print();

    // Write data to output port
    // Check ref
    if(bBypass||bLocked||((mTime - mInputRefPortLastTime) < INPUT_TIMEOUT)){
        
        if((mTime - mInputPoseLastTime) < INPUT_TIMEOUT){
            Vector &outputVec = mOutputPoseInRefPort.prepare();
            outputVec.resize(6);
            Pose6ToYarpPose6(mOutputPoseInRefPos,mOutputPoseInRefOri,outputVec);
            mOutputPoseInRefPort.write();
            
            MathLib::Matrix4 src,dst;
            MathLib::Matrix3 srcR,dstR;
            src.Transformation(srcR.RotationV(mInputPoseOri),mInputPosePos);
            dst.Transformation(dstR.RotationV(mOutputPoseInRefOri),mOutputPoseInRefPos);
            //cout << "Sending outInRef: ";
            //dst.Print();
            //cout << "From:";
            //src.Print();
            //cout << "Sending outInRef: "<< outputVec.toString() << endl;
            //cout << "from :            "<< mInputPosePos[0] <<" "<< mInputPosePos[1] <<" "<< mInputPosePos[2] <<" "<<mInputPoseOri[0] <<" "<< mInputPoseOri[1] <<" "<< mInputPoseOri[2] << endl;
        }

        if((mTime - mInputPoseInRefLastTime) < INPUT_TIMEOUT){
            Vector &outputVec = mOutputPosePort.prepare();
            outputVec.resize(6);
            Pose6ToYarpPose6(mOutputPosePos,mOutputPoseOri,outputVec);
            mOutputPosePort.write();
            //cout << "Sending out     : "<< outputVec.toString() << endl;
            //cout << "from :            "<< mInputPoseInRefPos[0] <<" "<< mInputPoseInRefPos[1] <<" "<< mInputPoseInRefPos[2] <<" "<<mInputPoseInRefOri[0] <<" "<< mInputPoseInRefOri[1] <<" "<< mInputPoseInRefOri[2] << endl;
        }
    }
    mMutex.post();
}

void RefTransformThread::Enable(bool enable){
    mMutex.wait();
    bBypass = !enable;
    mMutex.post();
}
void RefTransformThread::Activate(bool enable){
    mMutex.wait();
    bRun = enable;
    mMutex.post();
}
void RefTransformThread::Lock(bool lock){
    mMutex.wait();
    bLocked = lock;
    mMutex.post();
}
void RefTransformThread::EnableOrient(bool enable){
    mMutex.wait();
    bEnableOrient = enable;
    mMutex.post();
}
