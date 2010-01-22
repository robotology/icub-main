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

 
#include "TouchControllerThread.h"

#include <string.h>
#include <iostream>
using namespace std;

#include "YarpTools/YarpMathLibInterface.h"

TouchControllerThread::TouchControllerThread(int period, const char* baseName)
:RateThread(period)
{
    mPeriod = period;
    strncpy(mBaseName,baseName,256);
    mFrameOfRef.resize(3,3);
    mFrameOfRef.zero();
    mFrameOfRef(0,0) = mFrameOfRef(1,1) = mFrameOfRef(2,2) = 1.0;
    mTransGain  = 0.0;
    mRotGain    = 0.0;
    mTransLimit = 0.1;
    mRotLimit   = 40.0*(M_PI/180.0);

    mCoefs.resize(6);
    mCoefs[0] = mCoefs[1] = mCoefs[2] = mTransGain;
    mCoefs[4] = mCoefs[5] = mCoefs[6] = mRotGain;
    bDisplay = false;
    bRunning = true;
}

TouchControllerThread::~TouchControllerThread()
{}

void TouchControllerThread::SetDevice(const char* deviceName,int type){
    strncpy(mDeviceName,deviceName,256);
    mType = type;
}

void TouchControllerThread::LoadMap(){
    int l = strlen(mBaseName)+1;
    for(int i=0;i<l;i++) if(mBaseName[i]=='/') l = i;
    char filename[256];
    snprintf(filename,256,"mice_%s.map",mBaseName+l+1);
    mMiceDriver.LoadMap(filename);    
}

bool TouchControllerThread::threadInit()
{
    cerr << "Starting TouchController..."<<endl<< "  Device <"<< mDeviceName<< "> with type <"<<mType<<">"<<endl;
    
    if (!mMiceDriver.open()) {
        cerr << "Error: Unable to open Mice Driver..."<<endl;
        return false;
    }
    if(strncmp(mDeviceName,"fake",256)!=0){
        bFake = false;
        mMiceDriver.SetBaseMiceName(mDeviceName);
        if(mMiceDriver.GetNumValidDevices()<=0){
            cerr << "Error: No device named <"<<mDeviceName<<"> found..."<<endl;
            return false;
        }
    }else{
        bFake = true;
        mMiceDriver.SetBaseMiceName(mDeviceName);
    }
    switch(mType){
    case 0:
        mMiceDriver.SetMode(MMiceDeviceDriver::MMM_RELTOABS);
        mTransGain = 0.0003;
        mRotGain   = 0.001;
        break;
    case 1:
        {
        int ids[4]={0,1,2,3};
        mMiceDriver.LinkMiceButtons(ids,4);
        mMiceDriver.SetMode(MMiceDeviceDriver::MMM_STANDARD);
        mTouchController.SetMMiceDriver(&mMiceDriver);
        mTransGain = 0.06;
        mRotGain   = 0.40;
        mTouchController.SetDecayFactor(0.1);
        }
        break;
    default:
        cerr << "Error: Bad device type <"<<mType<<">"<<endl;
        return false;
        break;
    }


    char portName[256];
    snprintf(portName,256,"/%s/frameOfRef",mBaseName);
    mFrameOfRefPort.open(portName);

    snprintf(portName,256,"/%s/velocity",mBaseName);
    mOutputPort.open(portName);

    snprintf(portName,256,"/%s/signals",mBaseName);
    mSignalPort.open(portName);
    

    return true;
}

void TouchControllerThread::threadRelease()
{
    int l = strlen(mBaseName)+1;
    for(int i=0;i<l;i++) if(mBaseName[i]=='/') l = i;
    char filename[256];
    snprintf(filename,256,"mice_%s.map",mBaseName+l+1);
    mMiceDriver.SaveMap(filename);

    mFrameOfRefPort.close();
    mOutputPort.close();
    mSignalPort.close();

}

void TouchControllerThread::run()
{
    mMutex.wait();

    // Read data from input port
    Matrix *inputMat = mFrameOfRefPort.read(false);
    if(inputMat!=NULL){
        if(((inputMat->rows()==3)&&(inputMat->cols()==3))||((inputMat->rows()==4)&&(inputMat->cols()==4)))
            mFrameOfRef = *inputMat;
    }
    
    // Write data to output port
    Vector &outputVec = mOutputPort.prepare();
    Vector &signalVec = mSignalPort.prepare();
    
    signalVec.resize(3);
    signalVec = 0;
    
    if(!bFake)
        mMiceDriver.Update();

    if(mType==1){
        mCoefs[0] = mCoefs[1] = mCoefs[2] = mTransGain;
        mCoefs[3] = mCoefs[4] = mCoefs[5] = mRotGain;
        if(!bFake){
            mTouchController.Set6DOFSensingCoefs(mCoefs);
            mTouchController.SetOutputLimits(mTransLimit,mRotLimit);
            mTouchController.SetOrientationFrame(mFrameOfRef);
            
            mTouchController.Update();
        }
        outputVec.resize(9);
        outputVec = 0;
        if(!bFake){
            Vector tmpOutput;
            mTouchController.GetOrientationFrameOutput(tmpOutput);
            for(int i=0;i<6;i++)
                outputVec[i] = tmpOutput[i];
            
            Vector optOut; optOut.size(2); 
            mTouchController.Get2DOFSensingOutput(optOut);
            outputVec[6] = optOut[0];
            outputVec[7] = optOut[1];
            if(mTouchController.GetLastMouseTouch()){
                outputVec[8] = 1.0;
                signalVec[2] = 1.0;
            }
            if(mTouchController.TouchDetected())
                signalVec[0] = 1.0;
            if(mTouchController.ResetDetected())
                signalVec[1] = 1.0;
        }
    }else if(mType == 0){
        outputVec.resize(6);
        if(!bFake){
            MMiceDeviceDriver::MouseEventSummary *mouse3d = mMiceDriver.GetMouseData(0);
            outputVec[0] = -mouse3d->mAbs.Y     * mTransGain;
            outputVec[1] = -mouse3d->mAbs.X     * mTransGain;
            outputVec[2] = -mouse3d->mAbs.Z     * mTransGain;
            outputVec[3] = -mouse3d->mAbs.RY    * mRotGain;
            outputVec[4] = -mouse3d->mAbs.RX    * mRotGain;
            outputVec[5] = -mouse3d->mAbs.RZ    * mRotGain;
            
            MathLib::Matrix mat3(3,3);
            YarpMatrixToMatrix(mFrameOfRef,mat3);
            mat3.Resize(3,3,true);
            MathLib::Vector vec(3),vec2(3);
            vec[0] = outputVec[0]; vec[1] = outputVec[1]; vec[2] = outputVec[2];
            mat3.Mult(vec,vec2);
            outputVec[0] = TRUNC(vec[0],-mTransLimit,mTransLimit); outputVec[1] = TRUNC(vec[1],-mTransLimit,mTransLimit); outputVec[2] = TRUNC(vec[2],-mTransLimit,mTransLimit); 
            vec[0] = outputVec[3]; vec[1] = outputVec[4]; vec[2] = outputVec[5];
            mat3.Mult(vec,vec2);
            outputVec[3] = TRUNC(vec[0],-mRotLimit,mRotLimit); outputVec[4] = TRUNC(vec[1],-mRotLimit,mRotLimit); outputVec[5] = TRUNC(vec[2],-mRotLimit,mRotLimit); 
            
            if(mouse3d->btnState>0)
                signalVec[1] = signalVec[2] = 1.0;
            double norm = 0.0;
            for(int i=0;i<6;i++) norm += outputVec[i]*outputVec[i];
            if(norm>0.0001)
                signalVec[0] = 1.0;
        }
    }

    
    if((!bRunning)||(bFake))
        outputVec.zero();
    if(bDisplay)
        cout << outputVec.toString().c_str()<< "***" << signalVec.toString() <<endl;        
    
    // Write data to output port
    mOutputPort.write();
    mSignalPort.write();

    mMutex.post();
}

void TouchControllerThread::SetSensorIdByTouch(int id){
    mMiceDriver.TouchAndSetMouseId(id);
}

void    TouchControllerThread::Activate(bool act){
    bRunning = act;
}
void    TouchControllerThread::SetTransGain(double gain){
    mTransGain = gain;
}
void    TouchControllerThread::SetTransLimit(double gain){
    mTransLimit = MAX(0.0,gain);
}
void    TouchControllerThread::SetRotGain(double gain){
    mRotGain = gain;
}
void    TouchControllerThread::SetRotLimit(double gain){
    mRotLimit = MAX(0.0,gain);
}
void    TouchControllerThread::SetDisplay(bool disp){
    bDisplay = disp;
}