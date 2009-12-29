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

#include "DataGlove.h"



DataGloves::DataGloves(){
  mGloveLeft  = NULL;
  mGloveRight = NULL;
  bInit       = false;
  nDevices    = 0;
  bCalibrate  = false;  
  bFirst      = false;
}

DataGloves::~DataGloves(){
}

/// Initialize the DataGloves device. Return the number of initialized gloves.
int DataGloves::Init(const char* portName){
  bool ret = true;
  if(bInit) return TRUE;

  if(portName!=NULL)
    strcpy(mCOMPortName, portName);  

  fdGlove *tmpGlove = NULL;

  nDevices    = 0;
  mGloveLeft  = NULL;
  mGloveRight = NULL;
  
  printf( "Opening the first glove on %s...\n", mCOMPortName);
  tmpGlove = fdOpen(mCOMPortName);
  if(tmpGlove==NULL){
    printf( "DataGlove error: Couldn't find any glove\n");
    ret = false;
  }
  
  if(ret){
    if(fdGetGloveHand(tmpGlove)==FD_HAND_RIGHT){
      mGloveRight = tmpGlove;
    }else{
      mGloveLeft  = tmpGlove;    
    }
  
    printf( "Opening the second glove on %s...\n", mCOMPortName);
    tmpGlove = fdOpen(mCOMPortName);
    if(tmpGlove==NULL){
      printf( "Couldn't find the second glove...\n");
    }else{
      if(fdGetGloveHand(tmpGlove)==FD_HAND_RIGHT){
        mGloveRight = tmpGlove;
      }else{
        mGloveLeft  = tmpGlove;    
      }
    }
  }
  for(int i=0;i<14;i++){
    mHandRangeLeft[0].mSensorsArray[i]  = R_ZERO;
    mHandRangeLeft[1].mSensorsArray[i]  = R_ZERO;
    mHandRangeRight[0].mSensorsArray[i] = R_ZERO;
    mHandRangeRight[1].mSensorsArray[i] = R_ZERO;
  }
    
  bInit  = true;
  bFirst = true;
  
  nDevices = (mGloveLeft!=NULL?1:0) + (mGloveRight!=NULL?1:0); 
  



  // Calibration pose
  mNbCalibrationPoses = 7;
  //mCalibrationPoses;
  REALTYPE tcp[9*7] =
         {  0,-10, 0, 0, 0, 0, 0, 0,  0,
           60,-10,80, 0, 0, 0, 0, 0,  0,
           60, 30, 0, 0, 0, 0, 0, 0,  0,
           60, 20,50,30,30,50, 0, 0,  0,
//           60, 30,40,30, 0, 0,45,50,  0,
//           60, 60,60,30, 0, 0, 0, 0, 70,
            0, 75,50,30, 0, 0, 0, 0,110,
           50, 40,60,50, 0, 0,80,90,110,
           50, 10,40,50,80,90,70,90,110};
//           45,  0,80,90,70,50,70,75,110};
  mTargetCalibrationPoses.Set(tcp,mNbCalibrationPoses,9);
  //mCalibrationPoses.Resize(mNbCalibrationPoses,14);
  //mCalibrationPoses.Zero();

  REALTYPE cp[14*7] =
  {         389, 1616, 2554,  764,  485, 2857,  451, 1356, 3506, 1058, 1277, 3148, 461,  940,
            393, 1778, 2709,  825,  467, 2906,  447, 1264, 3481, 1012, 1259, 3056, 442,  881,
            387, 2122, 2469, 1095, 1433, 2923,  514, 1239, 3525, 1110, 1248, 3087, 451,  873,
            387, 2122, 2469, 1095, 1433, 2923,  514, 1239, 3525, 1110, 1248, 3087, 451,  873,
//            506, 2112, 2450, 1018,  506, 2586,  616, 2581, 3514, 1145, 1259, 3137, 446,  895,
//            483, 2124, 2386,  882,  494, 3021,  449, 1293, 3223, 1201, 1982, 3168, 581, 1560,
            472, 2096, 2341,  826,  485, 2269,  488, 1348, 3064, 1406, 2118, 3279, 658, 1591,
            449, 2115, 2315, 1172,  498, 2069, 1153, 2644, 3339, 1348, 2281, 3143, 668, 1493,
            452, 2098, 2351, 1317, 1737, 2948, 1074, 2631, 3430, 1399, 2423, 3129, 754, 1600};
//            686, 2841, 2765, 1446, 1514, 3017, 1000, 2506, 3412, 1239, 2047, 3142, 707, 1683};
  mCalibrationPoses.Set(cp,mNbCalibrationPoses,14);
  mNbTargetDofs = 9;
  PosesCalibration(FD_HAND_LEFT);
  PosesCalibration(FD_HAND_RIGHT);
  return nDevices;
}    

void DataGloves::Free(){
  if(mGloveLeft)  fdClose(mGloveLeft);
  if(mGloveRight) fdClose(mGloveRight);
  mGloveLeft  = NULL;
  mGloveRight = NULL;
  bInit       = false;
  bFirst      = false;
  bCalibrate  = false;  
  nDevices    = 0;
}

int DataGloves::Update(){
  if(!bInit) 
    return FALSE;

  
  if(mGloveLeft)    fdGetSensorRawAll(mGloveLeft,  mGloveLeftRawData);
  else              memset(mGloveLeftRawData,0,18*sizeof(unsigned short));

  if(mGloveRight)  fdGetSensorRawAll(mGloveRight, mGloveRightRawData);
  else              memset(mGloveRightRawData,0,18*sizeof(unsigned short));


  if(bFirst){
    for(int i=0;i<14;i++){
      mHandRangeLeft[0].mSensorsArray[i]  = ((REALTYPE)mGloveLeftRawData[i])-0.5;      
      mHandRangeLeft[1].mSensorsArray[i]  = ((REALTYPE)mGloveLeftRawData[i])+0.5;      
      mHandRangeRight[0].mSensorsArray[i] = ((REALTYPE)mGloveRightRawData[i])-0.5;      
      mHandRangeRight[1].mSensorsArray[i] = ((REALTYPE)mGloveRightRawData[i])+0.5;      
    }     
    bFirst = false; 
  }
  if(bCalibrate){
    for(int i=0;i<14;i++){
      mHandRangeLeft[0].mSensorsArray[i]  = MIN(mHandRangeLeft[0].mSensorsArray[i],
                                               ((REALTYPE)mGloveLeftRawData[i]));      
      mHandRangeLeft[1].mSensorsArray[i]  = MAX(mHandRangeLeft[1].mSensorsArray[i],
                                               ((REALTYPE)mGloveLeftRawData[i]));      
      mHandRangeRight[0].mSensorsArray[i] = MIN(mHandRangeRight[0].mSensorsArray[i],
                                               ((REALTYPE)mGloveRightRawData[i]));      
      mHandRangeRight[1].mSensorsArray[i] = MAX(mHandRangeRight[1].mSensorsArray[i],
                                               ((REALTYPE)mGloveRightRawData[i]));      
    }  
  }

  for(int i=0;i<14;i++){
    mHandLeft.mSensorsArray[i]  = TRUNC(((REALTYPE)mGloveLeftRawData[i]), 
                                        mHandRangeLeft[0].mSensorsArray[i],
                                        mHandRangeLeft[1].mSensorsArray[i]);
    mHandLeft.mSensorsArray[i] -= mHandRangeLeft[0].mSensorsArray[i];
    mHandLeft.mSensorsArray[i] /= (mHandRangeLeft[1].mSensorsArray[i] - mHandRangeLeft[0].mSensorsArray[i]); 
                                            
    mHandRight.mSensorsArray[i] = TRUNC(((REALTYPE)mGloveRightRawData[i]), 
                                        mHandRangeRight[0].mSensorsArray[i],
                                        mHandRangeRight[1].mSensorsArray[i]);
    mHandRight.mSensorsArray[i] -= mHandRangeRight[0].mSensorsArray[i];
    mHandRight.mSensorsArray[i] /= (mHandRangeRight[1].mSensorsArray[i] - mHandRangeRight[0].mSensorsArray[i]); 

    //mHandRight.mSensorsArray[i] = ((REALTYPE)mGloveRightRawData[i]); 
    if((i==2)||(i==5)||(i==8)||(i==11)){
      mHandRight.mSensorsArray[i] = 1.0 - mHandRight.mSensorsArray[i];
      mHandLeft.mSensorsArray[i] = 1.0 - mHandLeft.mSensorsArray[i];      
    }
    

  }

  return TRUE;
}

void  DataGloves::StartCalibration(){
  bFirst = true;
  
  bCalibrate  = true;  
}
void  DataGloves::StopCalibration(){
  bCalibrate  = false;  
}



REALTYPE        DataGloves::GetSensorValue(EfdGloveHand hand, EfdSensors id){
  if(hand == FD_HAND_LEFT){
    return (id>FD_LITTLEFAR ? R_ZERO : mHandLeft.mSensorsArray[id]);
  }else{
    return (id>FD_LITTLEFAR ? R_ZERO : mHandRight.mSensorsArray[id]);    
  }    
}

DataGloveHand&  DataGloves::GetSensorValue(EfdGloveHand hand){
  if(hand == FD_HAND_LEFT){
    return mHandLeft;
  }else{    
    return mHandRight;
  }
}

void    DataGloves::Print(){
  /*printf("Left hand:  ");
  for(int i=0;i<14;i++){
    printf("%.2f ",mHandLeft.mSensorsArray[i]);  
  }
  printf("\n");  
  */
  printf("Right hand: ");
  for(int i=0;i<14;i++){
    printf("%.2f ",mHandRight.mSensorsArray[i]);  
  }
  printf("\n");  
  /*
  printf("----------- ");
  for(int i=0;i<14;i++){
    printf("%.2f ",mHandRangeRight[0].mSensorsArray[i]);  
  }
  printf("\n");  
  printf("----------- ");
  for(int i=0;i<14;i++){
    printf("%.2f ",mHandRangeRight[1].mSensorsArray[i]);  
  }
  printf("\n");*/  
}

int     DataGloves::GetNbCalibrationPose(){
  return mNbCalibrationPoses;
}
void    DataGloves::GetCalibrationPose(EfdGloveHand hand, int id){
  Vector rawData(14);
  if(hand==FD_HAND_LEFT){
    for(int i=0;i<14;i++) rawData(i)=REALTYPE(mGloveLeftRawData[i]);    
  }else{
    for(int i=0;i<14;i++) rawData(i)=REALTYPE(mGloveRightRawData[i]);
  }
  if((id>=0)&&(id<mNbCalibrationPoses))
    mCalibrationPoses.SetRow(rawData,id);
}
void    DataGloves::GetTargetCalibrationPose(int id, Vector *pose){
}

#define STD_VECTOR_SET(vec,data,size) {vec.resize(size); for(int i=0;i<size;i++){vec[i] = data[i];}}

void    DataGloves::PosesCalibration(EfdGloveHand hand){
  int id = (hand==FD_HAND_LEFT?0:1);
    


  vector<IndicesVector> dofSubSpace;
  dofSubSpace.resize(mNbTargetDofs);
  {REALTYPE f[] = {5,6,8,9,11};         STD_VECTOR_SET(dofSubSpace[0],f,6);}
  {REALTYPE f[] = {0,2,3};              STD_VECTOR_SET(dofSubSpace[1],f,3);}
  {REALTYPE f[] = {0,1,2};              STD_VECTOR_SET(dofSubSpace[2],f,3);}
  {REALTYPE f[] = {1};                  STD_VECTOR_SET(dofSubSpace[3],f,1);}
  {REALTYPE f[] = {3,4,5};              STD_VECTOR_SET(dofSubSpace[4],f,3);}
  {REALTYPE f[] = {4};                  STD_VECTOR_SET(dofSubSpace[5],f,1);}
  {REALTYPE f[] = {5,6,7,8};            STD_VECTOR_SET(dofSubSpace[6],f,4);}
  {REALTYPE f[] = {7};                  STD_VECTOR_SET(dofSubSpace[7],f,1);}
  {REALTYPE f[] = {9,10,12,13};         STD_VECTOR_SET(dofSubSpace[8],f,4);}

  mCalibrationTransform[id].Resize(mNbTargetDofs,15);
  for(int i=0;i<mNbTargetDofs;i++){
    Matrix DI;
    mCalibrationPoses.GetColumnSpace(dofSubSpace[i],DI);
    DI.Resize(DI.RowSize(),DI.ColumnSize()+1);
    for(int j=0;j<int(DI.RowSize());j++)
      DI(j,DI.ColumnSize()-1) = 1.0;     
    Vector DO;
    mTargetCalibrationPoses.GetColumn(i,DO);
    Matrix DIt;
    DI.Transpose(DIt);
    Matrix DItDI;
    DIt.Mult(DI,DItDI);
    Matrix InvDItDI;
    DItDI.Inverse(InvDItDI);
    InvDItDI.Mult(DIt,DItDI);
    Vector alphas;
    DItDI.Mult(DO,alphas);
    for(int j=0;j<int(dofSubSpace[i].size());j++){
      mCalibrationTransform[id](i,dofSubSpace[i][j]) = alphas(j);
    }  
    mCalibrationTransform[id](i,14) = alphas(int(dofSubSpace[i].size()));
  }
  mCalibrationTransform[id].Print();
  Matrix At;
  mCalibrationTransform[id].Transpose(At);
  Matrix R;
  Matrix DI=mCalibrationPoses;
  DI.Resize(DI.RowSize(),DI.ColumnSize()+1);
  for(int j=0;j<int(DI.RowSize());j++)
    DI(j,DI.ColumnSize()-1) = 1.0;     
  DI.Mult(At,R);
  (R-mTargetCalibrationPoses).Print();
}

void    DataGloves::GetCalibratedPose(EfdGloveHand hand, Vector *pose){
  Vector rawData(15);
  if(hand==FD_HAND_LEFT){
    for(int i=0;i<14;i++) rawData(i)=REALTYPE(mGloveLeftRawData[i]);    
  }else{
    for(int i=0;i<14;i++) rawData(i)=REALTYPE(mGloveRightRawData[i]);
  }
  rawData(14) = 1.0;

  if(hand==FD_HAND_LEFT)
    mCalibrationTransform[0].Mult(rawData,*pose);
  else
    mCalibrationTransform[1].Mult(rawData,*pose);

}
