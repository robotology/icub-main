// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 Eric Sauser, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
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

#include <stdlib.h>
#include <stdio.h>
#include <dlfcn.h> // Need dlfcn.h for the routines to dynamically load libraries
#include <unistd.h> // Needed for sleep function

#include "XSens.h"
#include "signal.h"

// Return values for MT_GetOrientationData function
#define MT_NEWDATA		   1
#define MT_NODATA		     2
#define MT_NOSENSORID		 3
#define MT_INCOMPLETE		 4
#define MT_CHECKSUMERROR 5
#define MT_NOPORT		     6
#define MT_NOCALIBVALUES 7

// Output possibilites for MotionTracker library
#define MT_LOGQUATERNION	0
#define MT_LOGEULER		    1
#define MT_LOGROTMATRIX		2

#define RESET_HEADING		0
#define RESET_GLOBAL		1
#define RESET_OBJECT		2
#define RESET_ALIGN		  3

#define MAXSENSORS		  8


XSens* XSens::mActiveXSens = NULL;

void XSens::SegFaultHandler(int val){
  
  cout << "SEGFAULT"<<endl;
  exit(0);
  if(mActiveXSens){  
    mActiveXSens->Stop();    
    mActiveXSens->Reset();
    mActiveXSens->Start();
  }
}


XSens::XSens(){
  bInit       = false;
  nDevices    = 0;
  bCalibrate  = false;  
  bMPCalibrate = false;
  bTransposeOutput = true;
  mIdentity.Identity();
  strcpy(mCOMPortName,"/dev/ttyUSB0");
  mActiveXSens = this;
  mSequencerState = 0;
  mSequencerPos   = 0;
}
XSens::~XSens(){
  
}

int XSens::Init(const char* portName)
{
  if(bInit) return 1;
    
  if(portName!=NULL)
    strcpy(mCOMPortName, portName);  
  
  
  //signal(SIGSEGV,&XSens::SegFaultHandler);
    
  // Set MT library options
  short bLogCalibratedData = 0;

  // Set MT library variables
  //short nPortNumber = 4; //1 for serial port, 4 for usb;
  float fGain = 1.0f;
  short nCorInterval = 1;
  float fRho = 0.0f;

  ////////////////////////////////////////////////////////
  // Sample frequency and .XMU file location not needed with MT9-B
  // Sample frequency and .XMU file location needed with MT9-A
  //short nSampleFrequency = 100;
  char cXmuLocation[26] = "00002087_08052003_003.xmu";
  ////////////////////////////////////////////////////////

  // Dynamically load MT library
  printf("Loading the MotionTracker library...\n");
  module = dlopen("./lib/libmtobject.so",RTLD_NOW);
  if (!module) {
    module = dlopen("libmtobject.so",RTLD_NOW);
    if (!module) {
      printf("Couldn't open MotionTracker library: %s\n", dlerror());
      return 0;
    }
  }


  // Load the symbols
  create_t* create_mtobject = (create_t*) dlsym(module,"create");
  destroy_mtobject = (destroy_t*) dlsym(module,"destroy");
  
  if (!create_mtobject || !destroy_mtobject) {
    printf("Cannot load symbols\n");
    return 0;
  }
  
  // create an instance of the class
  pMT = create_mtobject();
  
  printf("Locating devices on port <%s>...\n",mCOMPortName);
  //pMT->MT_SetCOMPort_DeviceName(mCOMPortName);
  pMT->XM_SetCOMPort_DeviceName(mCOMPortName);
  short nDevs=0;  
  //pMT->XM_QueryXbusMasterB(&nDevs,(char*)cDeviceIDs);
  pMT->XM_QueryXbusMaster(&nDevs,(char*)(cDeviceIDs+0*9),(char*)(cDeviceIDs+1*9),(char*)(cDeviceIDs+2*9),(char*)(cDeviceIDs+3*9),(char*)(cDeviceIDs+4*9),(char*)(cDeviceIDs+5*9));
  if(nDevs<=0){ 
    printf("XSens error: No devices found\n");
    return FALSE;
  }
  nDevices = int(nDevs);
  pMT->XM_SetCalibratedOutput(bLogCalibratedData);

  printf("************************************************\n");
  printf("number of devices found: <%d>\n",nDevices);
  printf("************************************************\n");
  for (int i = 0; i < nDevices; i++){
    pMT->XM_SetFilterSettings((cDeviceIDs+i*9), fGain, nCorInterval, fRho);
    pMT->XM_SetxmuLocation((cDeviceIDs+i*9),cXmuLocation); 
   // pMT->XM_SetDoAMD((cDeviceIDs+i*9), 1);
    printf("<%02d>: %s\n",i,(cDeviceIDs+i*9));
  }    
  printf("************************************************\n");
  
  pMT->XM_SetTimeout(2);  
  pMT->XM_SetOutputMode(MT_LOGROTMATRIX);

  int fr = 0;
  pMT->MT_GetMotionTrackerSampleFrequency(&fr);
  printf("Sample frequency:           %d \n",fr);
  pMT->MT_GetMotionTrackerBaudrate(&fr);
  printf("Baud rate:                  %d \n",fr);
  pMT->XM_GetXbusMasterSampleFrequency(&fr);
  printf("BusMaster sample frequency: %d \n",fr);
  pMT->XM_GetXbusMasterBaudrate(&fr);
  printf("BusMaster baud rate         %d \n",fr);
  printf("************************************************\n");


  for (int i = 0; i < nDevices; i++){
    pMT->XM_GetFilterSettings((cDeviceIDs+i*9), &fGain, &nCorInterval, &fRho);
    printf("%s: %f %d %f\n",(cDeviceIDs+i*9), fGain, nCorInterval, fRho);
  }

  mRawMatrices = new Matrix3[nDevices];
  mPreMatrices = new Matrix3[nDevices];
  mCalMatrices = new Matrix3[nDevices];
  mOutMatrices = new Matrix3[nDevices];
  mFltMatrices = new Matrix3[nDevices];
  
  mMPCTransformMatrices = new Matrix3[nDevices*MAX_MPCCOUNT];
  mMPCDesiredMatrices   = new Matrix3[nDevices*MAX_MPCCOUNT];
  mMPCObservedMatrices  = new Matrix3[nDevices*MAX_MPCCOUNT];
  
  mDeviceIDs   = new int[nDevices];
  for(int i=0;i<nDevices;i++){
    mRawMatrices[i].Identity();
    mPreMatrices[i].Identity();
    mCalMatrices[i].Identity();
    mOutMatrices[i].Identity();
    mFltMatrices[i].Identity();
    mDeviceIDs[i] = atoi((cDeviceIDs+i*9));    

    for(int j=0;j<MAX_MPCCOUNT;j++){
      mMPCTransformMatrices[i*MAX_MPCCOUNT+j].Zero();
      mMPCDesiredMatrices  [i*MAX_MPCCOUNT+j].Zero();
      mMPCObservedMatrices [i*MAX_MPCCOUNT+j].Zero();     
    }

  }
  
  bInit = true;
  return 1;
}

void XSens::SetPreMatrix(int id, Matrix3 & matrix){
  for(int i=0;i<nDevices;i++){
    if(mDeviceIDs[i]==id){
      mPreMatrices[i] = matrix;
    }  
  }  
}

int XSens::GetData(){
  
  if(!bInit) return FALSE;
  
  short res     = 0;
  int   retVal  = FALSE;
  //usleep(1);
  //cout << "Get0"<<endl;
  pMT->MT_GetOrientationData(&res,mRawData,1);
  //cout << "Get1"<<endl;

  Matrix3 tmpMatrix;
  Vector3 tmpVector;

  switch(res) {
  case MT_NEWDATA:  
    for(int i=0;i<nDevices;i++){
       mRawMatrices[i].SetForceFloat(mRawData+i*9);
    }
    
    if(bCalibrate){      
      for(int i=0;i<nDevices;i++){
        mPreMatrices[i].Mult(mRawMatrices[i],mCalMatrices[i]);
        mCalMatrices[i].STranspose();
      }
      bCalibrate = false;
    }
    
    if(bMPCalibrate){      
      for(int i=0;i<nDevices;i++){
        mMPCObservedMatrices[i*MAX_MPCCOUNT+mMultiPoseCalibrationCount] = mRawMatrices[i]; 
      }
      mMultiPoseCalibrationCount++;
      bMPCalibrate = false;
    }
    
    

    for(int i=0;i<nDevices;i++){
      mPreMatrices[i].Mult(mRawMatrices[i],tmpMatrix);
      tmpMatrix.Mult(mCalMatrices[i],mOutMatrices[i]);      
    }
    
    for(int i=0;i<nDevices;i++){
      mFltMatrices[i].Transpose().Mult(mOutMatrices[i],tmpMatrix);
      tmpMatrix.GetExactRotationAxis(tmpVector);
      tmpVector*=0.5;
      tmpMatrix.RotationV(tmpVector);
      mFltMatrices[i].Mult(tmpMatrix,mOutMatrices[i]);      
      mFltMatrices[i] = mOutMatrices[i];
    }
    /*
      Matrix3 currMat,nextMat,resMat,tmpMat;
  Vector3 currVec,nextVec,resVec,tmpVec;
  ref.GetMatrix().GetOrientation(currMat); 
  ref.GetMatrix().GetTranslation(currVec);
  grab.GetOrientation(nextMat);
  grab.GetTranslation(nextVec);
  
  resVec = currVec + (-currVec+nextVec)*tau;
  
  tmpMat = currMat.Transpose()*nextMat;  
  tmpVec = tmpMat.GetExactRotationAxis();
  tmpVec*=tau;
  float angle = tmpVec.Norm();
  if(angle>EPSILON){
    tmpVec.Normalize();
    tmpMat.RotationV(angle,tmpVec);
  }else{
    tmpMat.Identity();
  }
  resMat = currMat*tmpMat;
  
  Matrix4 result;
  result.SetOrientation(resMat);
  result.SetTranslation(resVec);
  SetPos(result);
    */

    if(bTransposeOutput){
      for(int i=0;i<nDevices;i++){
        mOutMatrices[i].STranspose();
      }
    }
    
    
    if(mSequencerState==2){
      long  ltime = mSequencerChrono.ElapsedTimeMs();
      int   size  = mSequencerTimeData.Size();
      if(mSequencerPos>=size){
        mSequencerData.Resize(3*(size+1),nDevices*3,true);
        mSequencerTimeData.Resize(size+1,true);
      }
      int roffset = 3*mSequencerPos;
      for(int j=0;j<3;j++){
        int coffset = 0;
        for(int i=0;i<nDevices;i++){
          for(int k=0;k<3;k++){
            mSequencerData(roffset,coffset) = mOutMatrices[i](j,k);
            cout << mOutMatrices[i](j,k)<< " ";
            coffset++; 
          }
        }
        cout << endl;
        roffset++;
      }
      
      mSequencerTimeData(mSequencerPos) = REALTYPE(ltime)*0.001;
      //mSequencerPos++;
      
      //mSequencerData.Print();
    }
    
    
    
    
    
    
    mIdentity.Identity();
    retVal = TRUE;  
    break;
  case MT_NODATA:
    printf("XSens error: No Data On COM Port\n");
    break;
  case MT_NOSENSORID:
    printf("XSens error: No Sensor ID Received From Sensor\n");
    break;
  case MT_INCOMPLETE:
    printf("XSens error: Incomplete Data Received (Connection Lost)\n");
    break;
  case MT_CHECKSUMERROR:
    printf("XSens error: Checksum Error\n");
    break;
  case MT_NOPORT:
    printf("XSens error: COM port could not be opened\n");
    break;
  case MT_NOCALIBVALUES:
    printf("XSens error: XMU File With Calibration Data Could Not Be Read or \nMTS Data With Calibration Data Not Set\n");
    break;
  default:
    printf("XSens error: Unknown\n");
    break;
  }

  return retVal;
}

int XSens::Start(){
  
  if(!bInit) 
    if(!Init())
      return FALSE;

  printf("Stating XSens..."); fflush(stdout);
  pMT->MT_StartProcess();
  printf("done\n");

  return 1;
}

void XSens::Reset()
{
  if(bInit){
    printf("Reseting XSens..."); fflush(stdout);
    pMT->XM_ResetOrientation(RESET_GLOBAL);
    printf("done\n");
  }
}

void XSens::Stop()
{
  if(bInit){   
    printf("Stoping XSens..."); fflush(stdout);
    pMT->XM_StopProcess();
    printf("done\n");
  }
}

void XSens::PrintData(){
  if(bInit){
    for(int i=0;i<nDevices;i++)
      mOutMatrices[i].Print();    
  }
}

void    XSens::Calibrate(){
  bCalibrate = true;  
}

void    XSens::MultiPoseCalibrationReset(){
  mMultiPoseCalibrationCount = 0;
  for(int i=0;i<nDevices;i++){
    for(int j=0;j<MAX_MPCCOUNT;j++){
      mMPCTransformMatrices[i*MAX_MPCCOUNT+j].Identity();
      mMPCDesiredMatrices  [i*MAX_MPCCOUNT+j].Identity();
      mMPCObservedMatrices [i*MAX_MPCCOUNT+j].Identity();     
    }
  }  
}
void    XSens::MultiPoseCalibrationSetPose(int id, Matrix3 &desiredPose){
  for(int i=0;i<nDevices;i++){
    if(mDeviceIDs[i]==id){
      mMPCDesiredMatrices[i*MAX_MPCCOUNT+mMultiPoseCalibrationCount] = desiredPose;
    }  
  } 
}
  void    MultiPoseCalibrationSetPose(int id, Matrix3 &desiredPose);

void    XSens::MultiPoseCalibrationFetch(){
  bMPCalibrate = true;  
}

void    XSens::MultiPoseCalibrate(){
  for(int j=0;j<mMultiPoseCalibrationCount;j++){
    for(int i=0;i<nDevices;i++){
      mMPCObservedMatrices[i*MAX_MPCCOUNT+j].Transpose().Mult(mMPCDesiredMatrices[i*MAX_MPCCOUNT+j],mMPCTransformMatrices[i*MAX_MPCCOUNT+j]);
    }
  }
  for(int i=0;i<nDevices;i++){
    mCalMatrices[i] = mMPCTransformMatrices[i*MAX_MPCCOUNT];
    mPreMatrices[i].Identity();
  }
  
  
  
  
  Matrix      A(3*mMultiPoseCalibrationCount,mMultiPoseCalibrationCount-1);
  Matrix  baseA(3,mMultiPoseCalibrationCount-1);
  
  Vector      B(3*mMultiPoseCalibrationCount);
  
  for(int i=0;i<nDevices;i++){
    Matrix3 tmpM;
    Vector3 tmpV;

    for(int j=0;j<mMultiPoseCalibrationCount-1;j++){
      mMPCTransformMatrices[i*MAX_MPCCOUNT+0].Transpose().Mult(mMPCTransformMatrices[i*MAX_MPCCOUNT+j+1],tmpM);
      tmpM.GetExactRotationAxis(tmpV);
      Vector v(tmpV);
      baseA.SetColumn(v,j);
    }

    for(int j=0;j<mMultiPoseCalibrationCount;j++){
      A.SetRowSpace(baseA,j*3);      
      mMPCTransformMatrices[i*MAX_MPCCOUNT+0].Mult(mMPCTransformMatrices[i*MAX_MPCCOUNT+j],tmpM);
      tmpM.GetExactRotationAxis(tmpV);
      Vector v(tmpV);
      B.SetSubVector(j*3,v);
      
    }
    Matrix InvA;
    A.Inverse(InvA);
    Vector c;
    InvA.Mult(B,c);
    InvA.Print();
    A.Print();
    B.Print();
    c.Print();

    Vector deltaV;
    baseA.Mult(c,deltaV);
    tmpV.x() = deltaV(0);    
    tmpV.y() = deltaV(1);    
    tmpV.z() = deltaV(2);    
    tmpM.RotationV(tmpV);
    mMPCTransformMatrices[i*MAX_MPCCOUNT+0].Mult(tmpM,mCalMatrices[i]);

  }
  
  
  
}


Matrix3 &XSens::GetOutput(int id){
  for(int i=0;i<nDevices;i++){
    if(mDeviceIDs[i]==id){
      return mOutMatrices[i];
    }  
  }
  return mIdentity;
}

void    XSens::TransposeOutput(bool yes){
  bTransposeOutput = yes;
}

void    XSens::StartSequencer(bool record){
  if(record){
    mSequencerState = 2;
    mSequencerPos   = 0;
    mSequencerData.Resize(0,0,false);
    mSequencerTimeData.Resize(0);
    mSequencerChrono.Start();
  }else{
    mSequencerState = 1;
    mSequencerPos   = 0;
  }
}

void    XSens::StopSequencer(){
  mSequencerState = 0;
  mSequencerPos   = 0;
}

void    XSens::LoadSequencerData(const char * filename){
  mSequencerState = 0;
}
void    XSens::SaveSequencerData(const char * filename){
  mSequencerState = 0;
}
