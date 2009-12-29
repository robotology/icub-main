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

 
#include "DataStreamerThread.h"

DataStreamerThread::DataStreamerThread(int period, const char* baseName)
:RateThread(period)
{
    mPeriod = period;
    strncpy(mBaseName,baseName,256);
}

DataStreamerThread::~DataStreamerThread()
{}

bool DataStreamerThread::threadInit()
{
    mState              = DS_IDLE;
    mNextState          = DS_IDLE;
    bRecord             = false;
    bUseTime            = false;
    bLoop               = false;
    mCurrTime           = 0.0;
    mStartTime          = 0.0;
    mPauseTime          = 0.0;
    mStreamSize         = 0;
    mStreamMaxSize      = 8;
    mStreamLineSize     = 16;


    char portName[256];
    snprintf(portName,256,"/%s/input",mBaseName);
    mInputPort.open(portName);
    snprintf(portName,256,"/%s/output",mBaseName);
    mOutputPort.open(portName);

    mStartTime = Time::now();
    
    return true;
}

void DataStreamerThread::threadRelease()
{
    mInputPort.close();
    mOutputPort.close();
}

void DataStreamerThread::run()
{
    mMutex.wait();
    
    mCurrTime = Time::now();
    mCurrTime = mCurrTime - mStartTime;
    
    // Read data from input port
    Vector *inputVec = mInputPort.read(false);
    if(inputVec!=NULL){
        if(bUseTime){
            mInputVector.Resize(inputVec->size()+1);
            mInputVector(0) = mCurrTime;
            for(int i=0;i<inputVec->size();i++){
                mInputVector(i+1) = (*inputVec)[i];
            }
        }
    }

    bool bWriteData = false;

    switch(mState){
    case DS_IDLE:
        break;
    case DS_RUN:
        //cout << mCurrTime<<" "<< mCurrentLine <<" "<<mStreamSize<<" "<<mStreamMaxSize<<endl;

        if(bRecord){
            if(inputVec!=NULL){
                mData.SetRow(mInputVector,mCurrentLine);
            }
        }
    
        bWriteData = true;

        mData.GetRow(mCurrentLine,mOutputVector);
        if(!bRecord){
            if(bUseTime){
                while(mOutputVector(0) < mCurrTime){
                    mCurrentLine++;
                    if(mCurrentLine>=mStreamSize){
                        if(!bLoop){
                            mCurrentLine = mStreamSize-1;
                        }else{
                            mCurrentLine = 0;
                            mStartTime      = Time::now();
                            mCurrTime       = 0;
                        }
                        break;
                    }
                    mData.GetRow(mCurrentLine,mOutputVector);
                }
            }else{
                mCurrentLine++;
                if(mCurrentLine>=mStreamSize){
                    if(!bLoop){
                        mCurrentLine = mStreamSize-1;
                    }else{
                        mCurrentLine = 0;
                    }
                }
            }
        }else{
            mCurrentLine++;
            if(mCurrentLine>=mStreamMaxSize){
                if(!bLoop){
                    mCurrentLine = mStreamMaxSize-1;
                }else{
                    mCurrentLine = 0;
                }
                
            }
        }
        break;
    case DS_PAUSE:
        break;    
    }



    if(bWriteData){
        // Write data to output port
        Vector &outputVec = mOutputPort.prepare();
        if(bUseTime){
            outputVec.resize(mStreamLineSize-1);
            for(int i=0;i<outputVec.size();i++){
                outputVec(i) = mOutputVector[i+1];
            }
        }else{
            outputVec.resize(mStreamLineSize);
            for(int i=0;i<outputVec.size();i++){
                outputVec(i) = mOutputVector[i];
            }
        }
        // fecth data
        mOutputPort.write();
    }

    mMutex.post();
}

void    DataStreamerThread::Start(){
    mMutex.wait();
    if(mState!=DS_RUN){
        mState          = DS_RUN;
        mStartTime      = Time::now();
        mCurrentLine    = 0;
        // Add start stuff here
        
    }
    mMutex.post();
}
void    DataStreamerThread::Stop(){
    mMutex.wait();
    if(mState!=DS_IDLE){
        mState = DS_IDLE;
        // add stop stuff here
    }
    mMutex.post();
}
void    DataStreamerThread::Pause(){
    mMutex.wait();
    if(mState==DS_RUN){
        mState      = DS_PAUSE;
        mPauseTime  = Time::now();
        // add pause stuff here
    }
    mMutex.post();
}
void    DataStreamerThread::Resume(){
    mMutex.wait();
    if(mState==DS_PAUSE){
        mState      = DS_RUN;
        mStartTime  = mStartTime + (Time::now()-mPauseTime);
        // add pause stuff here
    }
    mMutex.post();
}
void    DataStreamerThread::SetRecordMode(bool rec){
    mMutex.wait();
    if(rec && !bRecord){
        mCurrentLine = 0;
        // add rec start stuff here
    }
    bRecord = rec;
    mMutex.post();
}
void    DataStreamerThread::Save(const char* filename){
    Stop();
    mMutex.wait();
    MathLib::Matrix data;
    mData.GetRowSpace(0, mCurrentLine,data);
    data.Save(filename);
    mMutex.post();
}
void    DataStreamerThread::Load(const char* filename){
    Stop();
    mMutex.wait();
    mData.Load(filename);
    if(mData.RowSize()==0){
        cerr << "File "<<filename<<" not found or bad data..."<<endl;
    }else{
        mStreamSize = mData.RowSize();
        mStreamLineSize = mData.ColumnSize();
        mData.Resize(mStreamMaxSize,mStreamLineSize,true);
        mData.Print();
    }
    mMutex.post();
}
void    DataStreamerThread::Clear(){
    Stop();
    mMutex.wait();
    mCurrentLine = 0;
    mStreamSize  = 0;
    mMutex.post();
}
void    DataStreamerThread::SetStreamLineSize(int size){
    Stop();
    mMutex.wait();
    if(size<0) size = 0;
    mStreamLineSize = size;
    mData.Resize(mStreamMaxSize,mStreamLineSize,true);
    mMutex.post();
}
void    DataStreamerThread::SetStreamMaxSize(int size){
    Stop();
    mMutex.wait();
    if(size<0) size = 0;
    mStreamMaxSize = size;
    mData.Resize(mStreamMaxSize,mStreamLineSize,true);
    mMutex.post();
}
void    DataStreamerThread::SetUseTime(bool useTime){
    Stop();
    mMutex.wait();
    bUseTime = useTime;
    // Fill in stuff...
    mMutex.post();
}
void    DataStreamerThread::SetLoop(bool loop){
    mMutex.wait();
    bLoop = loop;
    mMutex.post();
}

