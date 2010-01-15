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
#include "StdTools/Various.h"

#include <string.h>
#include <fstream>

#define INPUT_TIMEOUT 0.4
#define SIGNAL_TIMEOUT 1.0

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
    
    mEMProcessMode  = PM_NONE;
    mEMInitMode     = IM_NONE;
    mEMNbComponents = 6;
    mEMNbDemos      = 0;
    mEMNbCorrDemos  = 0;
    mEMNbDimensions = 0;
    strncpy(mEMDemosPath,    ".",256);
    strncpy(mEMCorrDemosPath,".",256);
    strncpy(mEMCurrModelPath,".",256);
    mEMTimeSpan     = 1.0;
    
    mCurrDemoId     = 0;
    
    mCurrCorrWeight = 0;
    mCurrOffset.Resize(6);
    mCurrOffset.Zero();
    
    mRunMode        = RM_NONE;

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

    snprintf(portName,256,"/%s/signals",mBaseName);
    mSignalPort.open(portName);
    
    snprintf(portName,256,"/%s/dataStreamOutput",mBaseName);
    mDSOutputPort.open(portName);
    
    snprintf(portName,256,"/%s/dataStreamRpc",mBaseName);
    mDSRpcPort.open(portName);

    mSignalVector.resize(3);
    mSignalVector = 0;
    
    mTime               = 0.0;
    mPrevTime           =-1.0;

    mInputLastTime      =-1.0;
    mSignalLastTime     =-1.0;
    return true;
}

void GaussianMixtureModelThread::threadRelease()
{
    Free();
    
    mInputPort.close();
    mOutputPort.close();
    mSignalPort.close();
    mDSOutputPort.close();
    mDSRpcPort.close();
    
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
    Vector *inputVec;
    inputVec = mInputPort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size()==mIOSize){
            YarpVectorToVector(*inputVec,mInputVector);
            Pose6ToQPose(mInputVector, mDSOutputVector,false);
            mInputLastTime = mTime;
        }else cerr << "Error: Bad input size: "<<inputVec->size()<<" != "<<mIOSize<<endl;
    }
    inputVec = mSignalPort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size()==3){
            mSignalVector = *inputVec;
            mSignalLastTime = mTime;
        }else cerr << "Error: Bad signal size: "<<inputVec->size()<<" != 3"<<endl;
    }
    if(mTime-mSignalLastTime < SIGNAL_TIMEOUT)
        mSignalVector = 0;
    
    
    //if(mTime-mInputLastTime >= INPUT_TIMEOUT)
    //    mNextState = GMMS_IDLE;
        
    bool bGMMIsRunning = false;
    
    if(mState != mNextState){
        mState = mNextState;
    }
    
    bool bStateChanged = (mPrevState != mState);
    if(bStateChanged){
        cout << "State change: <"<<mPrevState<<"> -> <"<<mState<<">"<<endl;
    }
    
    mNextState = mState;
    switch(mState){
    case GMMS_IDLE:
        break;
    case GMMS_INIT:
    case GMMS_INIT_PAUSE:
        if((mRunMode == RM_REPRO)||(mRunMode == RM_CORR)){
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
            mCurrCorrWeight = 0.0;
            mCurrOffset.Zero();
            bGMMIsRunning = true;
        }
        if(mState == GMMS_INIT)
            mNextState = GMMS_RUN;
        break;

    case GMMS_RUN:
        if((mRunMode == RM_REPRO)||(mRunMode == RM_CORR)){
            mGMMTime       += dt;
            mGMMInputV(0)   = mGMMInternalTimeSpan*mGMMTime/mGMMReproTime;
            mGMMInput(0,0)  = mGMMInputV(0); 
            double norm = mGMMLambda.Norm();
            if(norm > mGMMLambdaTreshold){
                mGMMLambda += (mGMMLambda*(mGMMLambdaTreshold/norm-1.0))*dt/mGMMLambdaTau;
                //mGMMLambda += (-mGMMLambda + mGMMLambda*(mGMMLambdaTreshold/norm))*dt/mGMMLambdaTau;
                //for(int i=0;i<7;i++)
                //    mGMMLambda(i) += (-mGMMLambda(i) + mGMMLambda(i)/(norm/2.0))*dt/1.0;
            }
            if(mGMMTime>=mGMMReproTime)
                mNextState = GMMS_END;
            
            if(mRunMode == RM_CORR){
                if(mSignalVector[0]>0.0){
                    mCurrCorrWeight += 1.0;
                    PauseRun(false);
                }
            }
            bGMMIsRunning = true;
        }
        break;
    case GMMS_PAUSE:
        if((mRunMode == RM_REPRO)||(mRunMode == RM_CORR)){
            if(mRunMode == RM_CORR){
                GetDeltaPose6FromPose6(mGMMTargetV, mInputVector, mCurrOffset);
                if(mSignalVector[1]>0.0){
                    ResumeRun(false);
                }
            }
        }
        break;

    case GMMS_END:
        if((mRunMode == RM_REPRO)||(mRunMode == RM_CORR)){
            mGMMTime        = mGMMReproTime;
            mGMMInputV(0)   = mGMMInternalTimeSpan;
            mGMMInput(0,0)  = mGMMInputV(0); 
            
            bGMMIsRunning = true;
        }
        break;
    }
    
    mPrevState  = mState;
    mState      = mNextState;
    
    if(bGMMIsRunning){
        mGMMTarget = mGMM.doOffsetRegression(mGMMInput, mGMMLambda, mGMMSigmas, mGMMInComp, mGMMOutComp);
        QPoseToPose6(mGMMTarget.GetRow(0),mGMMTargetV);
        AddPose6ToPose6(mGMMTargetV,mCurrOffset);
        mGMMTargetV.Print();
    }
    
    // Write data to output port
    if((bGMMIsReady)&&(bGMMIsRunning)){
        Vector &outputVec = mOutputPort.prepare();
        VectorToYarpVector(mGMMTargetV,outputVec);
        mOutputPort.write();
    }
    
    if(mTime-mInputLastTime < INPUT_TIMEOUT){
        Vector &outputVec = mDSOutputPort.prepare();
        if(mRunMode != RM_CORR){
            VectorToYarpVector(mDSOutputVector,outputVec);
        }else{
            MathLib::Vector tmpV(mDSOutputVector.Size()+1);
            tmpV.SetSubVector(0,mDSOutputVector);
            tmpV(mDSOutputVector.Size()) = mCurrCorrWeight;
            
            VectorToYarpVector(mDSOutputVector,outputVec);
        }
        mDSOutputPort.write();
    }

    mMutex.post();
}

void    GaussianMixtureModelThread::Load(const char* modelPath){
    mMutex.wait();
    char filename[256];
    snprintf(filename,256,"%s/gmmParams.txt",modelPath);
    bGMMIsReady = mGMM.loadParams(filename);
    if(!bGMMIsReady) cerr << "Error while loading file "<<filename<<endl;
    else cerr << "File "<<filename<<" sucessfully loaded"<<endl;
    strncpy(mEMCurrModelPath,modelPath,256);
    mMutex.post();
}

void    GaussianMixtureModelThread::Save(const char* modelPath){
    mMutex.wait();
    char filename[256];
    snprintf(filename,256,"%s/gmmParams.txt",modelPath);
    if(bGMMIsReady){
        mGMM.saveParams(filename);
        cerr << "File "<<filename<<" sucessfully saved"<<endl;
    }else{
        cerr << "Error while saving file "<<filename<<endl;
    }
    strncpy(mEMCurrModelPath,modelPath,256);
    mMutex.post();
}

void    GaussianMixtureModelThread::SetRunMode(GaussianMixtureModelThread::RunMode mode){
    mRunMode = mode;
}

void    GaussianMixtureModelThread::InitRun(){
    mMutex.wait();
    mNextState = GMMS_INIT_PAUSE;

    char txt[256];
    if((mRunMode == RM_REC)||(mRunMode == RM_CORR)){
        SendCommandToDataStreamer("run stop");
        SendCommandToDataStreamer("rec set");
        snprintf(txt,256,"data lineSize %d",mGMMIOSize+1+(mRunMode==RM_REC?0:1));
        SendCommandToDataStreamer(txt);
        snprintf(txt,256,"data maxSize %d",4096);
        SendCommandToDataStreamer(txt);
        SendCommandToDataStreamer("data timeOn");
        SendCommandToDataStreamer("data clear");
    }
    mMutex.post();
}
void    GaussianMixtureModelThread::StartRun(){
    mMutex.wait();
    mNextState = GMMS_INIT;
    char txt[256];
    if((mRunMode == RM_REC)||(mRunMode == RM_CORR)){
        SendCommandToDataStreamer("run stop");
        SendCommandToDataStreamer("rec set");
        snprintf(txt,256,"data lineSize %d",mGMMIOSize+1+(mRunMode==RM_REC?0:1));
        SendCommandToDataStreamer(txt);
        snprintf(txt,256,"data maxSize %d",4096);
        SendCommandToDataStreamer(txt);
        SendCommandToDataStreamer("data timeOn");
        SendCommandToDataStreamer("data clear");
        SendCommandToDataStreamer("run start");
    }
    mMutex.post();
}
void    GaussianMixtureModelThread::PauseRun(bool useMutex){
    if(useMutex) mMutex.wait();
    if(mState == GMMS_RUN){
        mNextState = GMMS_PAUSE;
        if((mRunMode == RM_REC)||(mRunMode == RM_CORR)){
            SendCommandToDataStreamer("run pause");
        }
    }
    if(useMutex) mMutex.post();
}
void    GaussianMixtureModelThread::ResumeRun(bool useMutex){
    if(useMutex) mMutex.wait();
    if(mState == GMMS_PAUSE){
        mNextState = GMMS_RUN;
        if((mRunMode == RM_REC)||(mRunMode == RM_CORR)){
            SendCommandToDataStreamer("run resume");
        }
    }
    if(useMutex) mMutex.post();
}
void    GaussianMixtureModelThread::StopRun(){
    mMutex.wait();
    mNextState = GMMS_IDLE;
    if((mRunMode == RM_REC)||(mRunMode == RM_CORR)){
        SendCommandToDataStreamer("run stop");
        char txt[256];
        snprintf(txt,256,"data save %s/datalog%03d.txt",(mRunMode==RM_REC?mEMDemosPath:mEMCorrDemosPath),mCurrDemoId);
        SendCommandToDataStreamer(txt);
    }

    mMutex.post();
}

void    GaussianMixtureModelThread::SetEMDemosPath(const char* path){
    mMutex.wait();
    strncpy(mEMDemosPath,path,256);
    mMutex.post();
}
void    GaussianMixtureModelThread::SetEMCorrDemosPath(const char* path){
    mMutex.wait();
    strncpy(mEMCorrDemosPath,path,256);    
    mMutex.post();
}

void    GaussianMixtureModelThread::SetEMProcessingMode(GaussianMixtureModelThread::ProcessMode mode){
    mMutex.wait();
    mEMProcessMode  = mode;
    mMutex.post();
}
void    GaussianMixtureModelThread::SetEMInitialisationMode(GaussianMixtureModelThread::InitMode mode){
    mMutex.wait();
    mEMInitMode = mode;
    mMutex.post();
}
void    GaussianMixtureModelThread::SetEMNbComponents(int nb){
    mMutex.wait();
    mEMNbComponents = nb;
    mMutex.post();
}
void    GaussianMixtureModelThread::SetEMDemoLength(int len){
    mMutex.wait();
    mEMDemoLength = len;
    mMutex.post();
}
void    GaussianMixtureModelThread::SetEMTimeSpan(double timeSpan){
    mMutex.wait();
    mEMTimeSpan = timeSpan;
    mMutex.post();
}

void    GaussianMixtureModelThread::SetCurrDemoId(int id){
    mMutex.wait();
    mCurrDemoId = id;
    mMutex.post();
}
bool    GaussianMixtureModelThread::Learn(){
    mMutex.wait();

    char filename[256];

    bool bSuccess = true;

    MathLib::Matrix data;
    MathLib::Vector timeBase;
    MathLib::Matrix fullData;
    MathLib::Vector weights;
    MathLib::Vector fullWeights;

    bSuccess = (mEMProcessMode!=PM_NONE);
    if(!bSuccess){cout << "Error: No processing mode selected..."<<endl; mMutex.post(); return false;}
    
    // Original demos
    cout << "Loading demo files..."<<endl;
    mEMNbDemos = GetConsecutiveFileCount(mEMDemosPath,"data%03d.txt");
    cout << "Demos found: "<<mEMNbDemos<<endl;
    bSuccess = (mEMNbDemos>0);
    if(!bSuccess){cout << "Error..."<<endl; mMutex.post(); return false;}


    for(int i=0;i<mEMNbDemos;i++){
        snprintf(filename,256,"%s/data%03d.txt",mEMDemosPath,i);
        cout << "Loading file: "<<filename<<endl;
        bSuccess = data.Load(filename);
        if(!bSuccess){cout << "Error..."<<endl; mMutex.post(); return false;}
        
        if(i==0){
            mEMDemoLength   = data.RowSize();
            mEMNbDimensions = data.ColumnSize();
            
            timeBase.Resize(mEMDemoLength);
            for(int j=0;j<mEMDemoLength;j++) 
                timeBase(j) = mEMTimeSpan*double(j)/double(mEMDemoLength-1);
            
            fullData.Resize(mEMDemoLength*mEMNbDemos,mEMNbDimensions,false);
        }else{
            bSuccess = ((data.RowSize()==mEMDemoLength)&&(data.ColumnSize()==mEMNbDimensions));
            if(!bSuccess){cout << "Error: Bad demo length..."<<endl; mMutex.post(); return false;}
        }
        data.SetColumn(timeBase,0);
        fullData.SetRowSpace(data,i*mEMDemoLength);
    }
    

    if(mEMProcessMode == PM_WEIGHTED){
        // Correction demos
        cout << "Loading correction demo files..."<<endl;
        mEMNbCorrDemos = GetConsecutiveFileCount(mEMCorrDemosPath,"data%03d.txt");
        cout << "Correction demos found: "<<mEMNbCorrDemos<<endl;
        bSuccess = (mEMNbDemos>0);
        if(!bSuccess){cout << "Error..."<<endl; mMutex.post(); return false;}

        fullData.Resize(mEMDemoLength*(mEMNbDemos+mEMNbCorrDemos),mEMNbDimensions,true);

        for(int i=0;i<mEMNbCorrDemos;i++){
            MathLib::Matrix data;
            snprintf(filename,256,"%s/data%03d.txt",mEMCorrDemosPath,i);
            cout << "Loading file: "<<filename<<endl;
            bSuccess = data.Load(filename);
            if(!bSuccess){cout << "Error..."<<endl; mMutex.post(); return false;}
            
            bSuccess = ((data.RowSize()==mEMDemoLength)&&(data.ColumnSize()==mEMNbDimensions));
            if(!bSuccess){cout << "Error: Bad demo length..."<<endl; mMutex.post(); return false;}

            data.SetColumn(timeBase,0);
            fullData.SetRowSpace(data,(mEMNbDemos+i)*mEMDemoLength);
        }

        // Weights
        MathLib::Matrix dataWeights;
        cout << "Loading weights"<<endl;
        snprintf(filename,256,"%s/weights.txt",mEMCorrDemosPath);
        bSuccess = dataWeights.Load(filename);
        if(!bSuccess){cout << "Error..."<<endl; mMutex.post(); return false;}
        bSuccess = ((dataWeights.RowSize()==mEMDemoLength)&&(dataWeights.ColumnSize()==2));
        if(!bSuccess){cout << "Error: Bad weights length..."<<endl; mMutex.post(); return false;}
        
        dataWeights.GetColumn(1,weights);

        fullWeights.Resize(mEMDemoLength*(mEMNbDemos+mEMNbCorrDemos),false);

        MathLib::Vector ones(mEMDemoLength);
        ones.Zero(); ones+=1.0;
        ones.Sub(weights,weights);
        weights *= (1.0-double(mEMNbDemos)/double(mEMNbDemos+mEMNbCorrDemos)); 
        for(int i=0;i<mEMNbDemos;i++)
            fullWeights.SetSubVector(i*mEMDemoLength, weights);
        
        dataWeights.GetColumn(1,weights);
        weights *= (1.0-double(mEMNbCorrDemos)/double(mEMNbDemos+mEMNbCorrDemos));             
        for(int i=0;i<mEMNbCorrDemos;i++)
            fullWeights.SetSubVector((mEMNbDemos+i)*mEMDemoLength, weights);

        cout << "Removing zero-weights datapoints"<<endl;
        MathLib::IndicesVector nonNull;
        int fullLength = fullData.RowSize();
        for(int i=0;i<fullLength;i++){
            if(fullWeights(i)>0)
                nonNull.push_back(i);
        }    
        MathLib::Matrix tmpM = fullData;
        tmpM.GetRowSpace(nonNull,fullData);
        MathLib::Vector tmpV = fullWeights;
        tmpV.GetSubVector(nonNull,fullWeights);
    }
    
    cout << "Learning Initialisation..."<<endl;
    if(mEMInitMode == IM_NONE){
        cout << "Using original model"<<endl;
        snprintf(filename,256,"%s/gmmParams.txt",mEMDemosPath);
        bSuccess = mGMM.loadParams(filename);
        if(!bSuccess){cout << "Error, file not found..."<<endl; mMutex.post(); return false;}
        mEMNbComponents = mGMM.nState;
    }
    
    cout << "Using "<<mEMNbComponents<<" components..."<<endl;
    bSuccess = (mEMNbComponents>0);
    if(!bSuccess){cout << "Error..."<<endl; return false;}
    
    switch(mEMInitMode){
    case IM_NONE:
        break;
    case IM_TIMESPLIT:
        cout << "Using time split..."<<endl;
        mGMM.initEM_TimeSplit(mEMNbComponents, fullData); break;
    case IM_KMEANS:
        cout << "Using kmeans..."<<endl;
        mGMM.initEM_kmeans(mEMNbComponents,    fullData); break;
    case IM_RAND:
        cout << "Using random..."<<endl;
        mGMM.initEM_random(mEMNbComponents,    fullData); break;
    }
    bGMMIsReady = false;

    cout << "Learning..."<<endl;
    fullData.Print();
    
    if(mEMProcessMode==PM_SIMPLE){
        mGMM.doEM(fullData);
        snprintf(filename,256,"%s/gmmParams.txt",mEMDemosPath);
        strncpy(mEMCurrModelPath,mEMDemosPath,256);
    }else{
        mGMM.doEM(fullData,fullWeights);
        snprintf(filename,256,"%s/gmmParams.txt",mEMCorrDemosPath);        
        strncpy(mEMCurrModelPath,mEMCorrDemosPath,256);
    }
    
    cout << "Saving model: "<<filename<<endl;
    mGMM.saveParams(filename);

    bGMMIsReady = true;
    mMutex.post();
    return true;
}

void GaussianMixtureModelThread::GenerateDefaultRegression(){
    if(!bGMMIsReady) return;

    mMutex.wait();
    
    MathLib::Matrix in(mEMDemoLength,1);
    for(int j=0;j<mEMDemoLength;j++)
        in(j,0) = mEMTimeSpan*double(j)/double(mEMDemoLength-1);

    
    MathLib::Matrix *sigmas; 
    sigmas = new MathLib::Matrix[mEMDemoLength];
    MathLib::Matrix res = mGMM.doRegression(in,sigmas,mGMMInComp,mGMMOutComp);

    int D = mGMMOutComp.Size();
    MathLib::Matrix sig(mEMDemoLength,D);
    for(int i=0;i<mEMDemoLength;i++){
        for(int j=0;j<D;j++){
            sig(i,j) = sigmas[i](j,j);    
        }
    }
    delete [] sigmas;

    char filename[256];
    snprintf(filename,256,"%s/regMean.txt",mEMCurrModelPath);
    res.Save(filename);
    
    snprintf(filename,256,"%s/regSigma.txt",mEMCurrModelPath);
    sig.Save(filename);

    mMutex.post();
}

bool    GaussianMixtureModelThread::ProcessRawDemos(){
    mMutex.wait();

    char filename[256];

    bool bSuccess = true;

    MathLib::Matrix rawData;
    MathLib::Matrix data;
    MathLib::Matrix axisData;
    MathLib::Matrix quatData;
    MathLib::Matrix tmpData;
    MathLib::Vector quatRef(4);
    MathLib::Vector weights;
    MathLib::Matrix weightsMat;
    
    // Original demos
    cout << "Loading raw demo files..."<<endl;
    mEMNbDemos = GetConsecutiveFileCount(mEMDemosPath,"datalog%03d.txt");
    cout << "Raw demos found: "<<mEMNbDemos<<endl;
    bSuccess = (mEMNbDemos>0);
    if(!bSuccess){cout << "Error..."<<endl; mMutex.post(); return false;}

    cout << "Using "<< mEMDemoLength <<" as a default length"<<endl;
    bSuccess = (mEMDemoLength>0);
    if(!bSuccess){cout << "Error..."<<endl; mMutex.post(); return false;}
    
    for(int i=0;i<mEMNbDemos;i++){
        snprintf(filename,256,"%s/datalog%03d.txt",mEMDemosPath,i);
        cout << "Loading file: "<<filename<<endl;
        bSuccess = rawData.Load(filename);
        if(!bSuccess){cout << "Error..."<<endl; mMutex.post(); return false;}
        
        Resample(rawData, data, mEMDemoLength);

        data.GetColumnSpace(4,4,quatData);
        SmoothQuatStream(quatData);
        if(i==0){
            quatData.GetRow(mEMDemoLength-1,quatRef);
        }else{
            AlignQuatStreamEnd(quatData,quatRef);
        }
        
        data.SetColumnSpace(quatData,4);
        snprintf(filename,256,"%s/data%03d.txt",mEMDemosPath,i);
        cout << "Saving file: "<<filename<<endl;
        data.Save(filename);
    }
    
    if(mEMProcessMode==PM_WEIGHTED){
        // Correction demos
        cout << "Loading raw corr demo files..."<<endl;
        mEMNbCorrDemos = GetConsecutiveFileCount(mEMCorrDemosPath,"datalog%03d.txt");
        cout << "Raw demos found: "<<mEMNbCorrDemos <<endl;
        bSuccess = (mEMNbCorrDemos>0);
        if(!bSuccess){cout << "Error..."<<endl; mMutex.post(); return false;}

        weightsMat.Resize(mEMDemoLength,mEMNbCorrDemos,false);
        weightsMat.Zero();

        for(int i=0;i<mEMNbCorrDemos;i++){
            snprintf(filename,256,"%s/datalog%03d.txt",mEMCorrDemosPath,i);
            cout << "Loading file: "<<filename<<endl;
            bSuccess = rawData.Load(filename);
            if(!bSuccess){cout << "Error..."<<endl; mMutex.post(); return false;}

            Resample(rawData, data, mEMDemoLength);
            data.GetColumnSpace(4,4,quatData);
            SmoothQuatStream(quatData);
            AlignQuatStreamEnd(quatData,quatRef);
            
            data.SetColumnSpace(quatData,4);
            data.GetColumn(8,weights);
            
            double wMax = weights.Max();
            if(wMax>1e-6)
                weights *= (1.0/wMax);
            
            weightsMat.SetColumn(weights,i);
            
            data.Resize(mEMDemoLength,7,true);
            
            snprintf(filename,256,"%s/data%03d.txt",mEMDemosPath,i);
            cout << "Saving file: "<<filename<<endl;
            data.Save(filename);
        }
        //Processing weights;
        for(int i=0;i<mEMDemoLength;i++){
            weights(i) = weightsMat.GetColumn(i).Sum();
        }
        double wMax = weights.Max();
        if(wMax>1e-6)
            weights *= (1.0/wMax);
        
    }
    
    mMutex.post();
    return true;
}



void GaussianMixtureModelThread::GenerateGnuplotScript(){
    char filename[256];
    ofstream file;
    snprintf(filename,256,"%s/plot.gp",mEMDemosPath);
    cout << "Generating gnuplot script: "<<filename<<endl;
    file.open(filename);
    if(file.is_open()){
        file << "set style line 1 lt 1 lw 3"<<endl;
        file << "set style line 2 lt 3 lw 2"<<endl;
        file << "set style line 3 lt 2 lw 1"<<endl;
        file << "set noxtic"<<endl;
        file << "set ytics 0.1"<<endl;
        file << "set mytics 0.01"<<endl;
        file << "set rmargin 0"<<endl;
        file << "set lmargin 0"<<endl;
        file << "set tmargin 0"<<endl;
        file << "set bmargin 0"<<endl;
        file << "set multiplot"<<endl;
        file << "boxspace = "<< double(1)/double(7) << endl;
        file << "space    = "<< double(1)/double(7) * 0.95 << endl;
        file << "offspace = (boxspace-space)*0.5"<<endl;
        file << "set size 0.9,space"<<endl;
        for(int i=0;i<7;i++){
            file << "set origin 0.05,"<<i<<"*boxspace+offspace"<<endl;
            file << "plot \"< paste regMean.txt regSigma.txt\" using ($"<<i+1<<")           with lines linestyle 1 title \"\", \\"<<endl;
            file << "     \"< paste regMean.txt regSigma.txt\" using ($"<<i+1<<"+sqrt($"<<i+8<<"))           with lines linestyle 2 title \"\", \\"<<endl;
            file << "     \"< paste regMean.txt regSigma.txt\" using ($"<<i+1<<"-sqrt($"<<i+8<<"))           with lines linestyle 2 title \"\" ";
            if(mEMNbDemos==0)   file <<endl;
            else                file <<", \\"<<endl;                
            for(int j=0;j<mEMNbDemos;j++){
                char txt[64]; snprintf(txt,16,"data%03d.txt",j);
                file << "     \""<<txt<<"\"                      using ($"<<i+2<<")           with lines linestyle 3 title \"\"";
                if(j<mEMNbDemos-1) file <<", \\"<<endl;
                else file << endl;
            }
            file << endl;
        }

        file << "set nomultiplot"<<endl;
        file.close();
        snprintf(filename,256,"gnuplot -persist %s/plot.gp",mEMDemosPath);
        int res = system(filename);
    }else{
        cout << "Failed..."<<endl;
    }
}



void GaussianMixtureModelThread::SendCommandToDataStreamer(const char *cmd){
    Bottle &cmdBottle = mDSRpcPort.prepare();
    /*cmdBottle.clear();
    vector<string> currCmds = Tokenize(RemoveSpaces(cmd));
    for(size_t j=0;j<currCmds.size();j++)
        cmdBottle.addString(currCmds[j].c_str());
    */
    cmdBottle.fromString(cmd);
    mDSRpcPort.writeStrict();
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

MathLib::Matrix& AxisToQuat(const MathLib::Matrix& axis,  MathLib::Matrix& quat, bool inv){
    int N = axis.RowSize();
    quat.Resize(N,4,false);
    MathLib::Vector axisV(3),quatV(4);
    for(int i=0;i<N;i++){
        quat.SetRow(AxisToQuat(axis.GetRow(i,axisV),quatV,inv),i);
    }
    return quat;
}

MathLib::Vector& QuatToAxis(const MathLib::Vector& quat, MathLib::Vector& axis){
    axis.Resize(3);
    double norm  = quat.Norm();
    if(norm<1e-6){
        axis.Zero();
    }else{
        norm = 1.0/norm;
        double angle = 2.0 * acos(TRUNC(quat.At(3)*norm,-1.0,1.0));
        double n = sqrt(1.0-quat.At(3)*quat.At(3)*norm*norm);
        if((n)<1e-6){
            axis(0) = 0.0;    
            axis(1) = 0.0;    
            axis(2) = 0.0;    
        }else{    
            n = angle*norm/n;
            axis(0) = quat.At(0) *n;
            axis(1) = quat.At(1) *n;
            axis(2) = quat.At(2) *n;
        }
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

MathLib::Matrix& SmoothQuatStream(MathLib::Matrix& qstream){
    int N = qstream.RowSize();
    for(int i=1;i<N;i++){
        double qdot = qstream.At(i,0) * qstream.At(i-1,0) +
                      qstream.At(i,1) * qstream.At(i-1,1) + 
                      qstream.At(i,2) * qstream.At(i-1,2) +
                      qstream.At(i,3) * qstream.At(i-1,3);
        if(qdot<-qdot){
            qstream(i,0) = -qstream.At(i,0);
            qstream(i,1) = -qstream.At(i,1);
            qstream(i,2) = -qstream.At(i,2);
            qstream(i,3) = -qstream.At(i,3);
        }
    }
    return qstream;
}
MathLib::Matrix& AlignQuatStreamEnd(MathLib::Matrix& qstream, const MathLib::Vector& end){
    int N = int(qstream.RowSize())-1;
    double qdot = qstream.At(N,0) * end.At(0) +
                  qstream.At(N,1) * end.At(1) +
                  qstream.At(N,2) * end.At(2) +
                  qstream.At(N,3) * end.At(3);
    if(qdot<-qdot){
        qstream.SMinus();
    }
    return qstream;
}

MathLib::Matrix& Resample(const MathLib::Matrix& src, MathLib::Matrix& result, int length){
    int D = src.ColumnSize();
    int N = src.RowSize();
    
    MathLib::Vector tmp(D);
    
    result.Resize(length,D,false);
    for(int i=0;i<length;i++){
        int i2 = (i*N)/length;
        //cout << i << "/"<<length<<" : "<<i2<<"/"<<N<<endl;
        i2 = MIN(N-1,i2);
        result.SetRow(src.GetRow(i2,tmp),i);
    }
    return result;
}
