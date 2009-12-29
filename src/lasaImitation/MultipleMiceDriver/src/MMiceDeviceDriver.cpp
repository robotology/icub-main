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

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include "MMiceDeviceDriver.h"

using namespace std;
using namespace yarp::os;
//using namespace yarp::dev;


bool                                  MMiceDeviceDriver::sIsReady           = false;
int                                   MMiceDeviceDriver::sNumDevices        = 0;
int                                   MMiceDeviceDriver::sNumDrivers        = 0;
MMiceDeviceDriver::MouseEventSummary *MMiceDeviceDriver::sMiceEvents        = NULL;
vector<int>                           MMiceDeviceDriver::sDriverUpdateState;
MMiceDeviceDriver::Mode              *MMiceDeviceDriver::sMode              = NULL;
int                                   MMiceDeviceDriver::sLastDeviceTouched = -1;

MMiceDeviceDriver::MMiceDeviceDriver() {
  mValidCount           = 0;
  mValidDevices         = NULL;
  mMice                 = NULL;
  mNumMice              = 0;
  mMiceMap              = NULL;
  mMouseToId            = -1;
  bIsReady              = false;
  mButtonLinks          = NULL;
  mButtonLinksSize      = 0;
  mCurrentTime          = Time::now();
  mIntegratorStopTime   = 0.5;
}

MMiceDeviceDriver::~MMiceDeviceDriver() {
  close();
}

bool MMiceDeviceDriver::open() {
  if(bIsReady) return true;

  fprintf(stderr,"MMice: Opening MMice Driver...\n");

  sDriverUpdateState.push_back(0);
  mId = int(sDriverUpdateState.size())-1;
  
  sNumDrivers++;
  
  if(!sIsReady){
    fprintf(stderr,"MMice: Initializing ManyMouse Core...\n");
    sNumDevices = ManyMouse_Init();
    if(sNumDevices>0){
        sMiceEvents = new MouseEventSummary[sNumDevices];
        sMode = new Mode[sNumDevices];
        for(int i=0;i<sNumDevices;i++){
          memset(&sMiceEvents[i],0,sizeof(MouseEventSummary));
          sMiceEvents[i].device = i;
          sMode[i] = MMM_STANDARD;
        }
    }
    sIsReady = true;
  }

  if (sNumDevices < 0) {
    fprintf(stderr,"MMice: Error: ManyMouse problems...\n");
    return false;
  } else if(sNumDevices == 0) {
    fprintf(stderr,"MMice: Error: No mice detected...\n");
    return true;
  } else {
  
    mValidDevices = new int[sNumDevices];
    mMiceMap      = new int[sNumDevices];
    for (int i=0; i<sNumDevices; i++){
      mValidDevices[i] = 1;
      mMiceMap[i]      = i;
    }
    mValidCount = sNumDevices;

    bIsReady = true;
    
    fprintf(stderr,"MMice: Found %i devices.\n", sNumDevices);
      
    SetBaseMiceName(NULL);
  }

  return true;
}

bool MMiceDeviceDriver::close() {  

  if(mValidDevices) 
    delete [] mValidDevices;
  if(mMiceMap)      
    delete [] mMiceMap;
  if(mMice)      
    delete [] mMice;
  

  if(mButtonLinks){
    for(int i=0;i<mButtonLinksSize;i++){
      UnlinkMiceButtons(i);
    }  
    delete [] mButtonLinks;
    mButtonLinks     = NULL;
    mButtonLinksSize = 0;
  }

  if(bIsReady){
    fprintf(stderr,"MMice: Closing MMice Driver...\n");   
    sNumDrivers--;
    if(sNumDrivers<=0){
      fprintf(stderr,"MMice: Closing ManyMouse Core...\n");
      ManyMouse_Quit();  
      sNumDevices  = 0;
      if(sMiceEvents)
        delete [] sMiceEvents;
      sIsReady     = false;
    }
  }

  mValidCount       = 0;
  mValidDevices     = NULL;
  mMice             = NULL;
  mMiceMap          = NULL;
  bIsReady          = false;
  
  return true;

}

bool  MMiceDeviceDriver::ClearMiceData(int id){
  if((id<0)||(id>=mNumMice)){
    return false;  
  }
  memset(&mMice[id],0,sizeof(MouseEventSummary));
  return true;
}


bool MMiceDeviceDriver::SetBaseMiceName(const char * baseMiceName) {
  if(!bIsReady) return false;
  
  mValidCount = 0;
  for (int i=0; i<sNumDevices; i++){
    if (baseMiceName) {
      if (strncmp(baseMiceName,ManyMouse_DeviceName(i),strlen(baseMiceName))==0) {
        mValidDevices[i] = 1;
      }else{
        mValidDevices[i] = 0;
      }
    } else {
      mValidDevices[i] = 1;
    }            

    if(mValidDevices[i])
      mValidCount++;    
  }    
    
  if(baseMiceName){
    fprintf(stderr,"MMice: - Mice matching <%s> found: %d\n",baseMiceName,mValidCount);
  }else{
    fprintf(stderr,"MMice: - Mice found: %d\n",mValidCount);
  }
  for(int i=0; i<sNumDevices; i++){
    if(mValidDevices[i])
      fprintf(stderr,"MMice:    %s\n", ManyMouse_DeviceName(i));
  }

  InitMap(mValidCount);

  return true;
}

int           MMiceDeviceDriver::GetNumDevices(){
  return sNumDevices;
}
const char*   MMiceDeviceDriver::GetDeviceName(int i){
  if ((i<0) || (i>=sNumDevices))
    return NULL;
  return ManyMouse_DeviceName(i);
}
int           MMiceDeviceDriver::GetNumValidDevices(){
  return mValidCount;
}
bool          MMiceDeviceDriver::IsDeviceValid(int i){
  if ((i<0) || (i>=sNumDevices))
    return false; 
  return mValidDevices[i]; 
}
MMiceDeviceDriver::MouseEventSummary *MMiceDeviceDriver::GetMouseData(int id){
  if((id<0) || (id>=mNumMice))
    return NULL;  
  return &mMice[id];
}
bool               MMiceDeviceDriver::HasMouseChanged(int id){
  if((id<0) || (id>=mNumMice))
    return false;  
  return mMice[id].hasChanged;    
}
void  MMiceDeviceDriver::SetMode(MMiceDeviceDriver::Mode newMode){
  for(int i=0;i<sNumDevices;i++){
    if(mMiceMap[i]>=0){
      sMode[i] = newMode;
      //memcpy(&mMice[mMiceMap[i]],&sMiceEvents[i],sizeof(MouseEventSummary));
    }
  }
}


void MMiceDeviceDriver::Update(){
  if(!bIsReady) return;

  bool bDoUpdate = false;
  
  //for(int i=0;i<int(sDriverUpdateState.size());i++) printf("%d",sDriverUpdateState[i]); printf("\n");
  if(sDriverUpdateState[mId]==0){
    bool bOne = false;
    for(int i=0;i<int(sDriverUpdateState.size());i++){
      if(mId!=i){
        if(sDriverUpdateState[i]==1){
          bOne=true;
          break;
        }
      }
    }
    sDriverUpdateState[mId] = 1; 
    if(bOne){
      bDoUpdate = false;    
    }else{
      bDoUpdate = true;  
    }    
  }else{
    for(int i=0;i<int(sDriverUpdateState.size());i++){
      if(mId!=i){
        sDriverUpdateState[i]=0;
      }
    }    
    bDoUpdate = true;
  }

  if(bDoUpdate){
    //printf(" Update %d\n",mId);

  double timeNow = Time::now();
    
  ManyMouseEvent event;

  for(int i=0;i<sNumDevices;i++){
    int        btnState     = sMiceEvents[i].btnState;
    int        btnLinkState = sMiceEvents[i].btnLinkState;
    double     lastTime     = sMiceEvents[i].lastTime;
    double     lastRelTime  = sMiceEvents[i].lastRelTime;
    Point6DInt abs          = sMiceEvents[i].mAbs;
    Point6DInt rel          = sMiceEvents[i].mRel;
    
    memset(&sMiceEvents[i],0,sizeof(MouseEventSummary));
    sMiceEvents[i].device           = i;
    sMiceEvents[i].btnState         = btnState;
    sMiceEvents[i].btnLinkState     = btnLinkState;
    sMiceEvents[i].lastRelTime      = lastRelTime;
    if((btnState>0)||(btnLinkState)||(sMode[i] == MMM_INTEGRATOR)){
      sMiceEvents[i].mAbs  = abs;
    }else{
      for(int j=0;j<6;j++) sMiceEvents[i].mAbsArray[j] = 0;
    }
    if(sMode[i] == MMM_RELTOABS){
            if((timeNow - sMiceEvents[i].lastRelTime)>0.05){
          bool bReset=false;
              for(int j=0;j<6;j++){
            if(((int*)&rel)[j] != 0){
              bReset = true;
              break;
            }  
          }
          if(bReset){
            sMiceEvents[i].lastEvent   = RELMOTION; 
            sMiceEvents[i].hasChanged  = 1;          
          }
            }else{
          sMiceEvents[i].mRel  = rel;
            }
    }
  }

  sLastDeviceTouched = -1;

  while (ManyMouse_PollEvent(&event)){
    if(int(event.device)>=sNumDevices)    continue;
    //if(mValidDevices[event.device]==0)    continue;
    
    sLastDeviceTouched = event.device;


    switch(event.type) {
    case MANYMOUSE_EVENT_RELMOTION:
      if((event.item>=0)&&(event.item<6)){
        sMiceEvents[event.device].mRelArray[event.item] += event.value;
        if(sMode[event.device] == MMM_RELTOABS){
          sMiceEvents[event.device].mRelArray[event.item] = event.value;
        }
        if((sMiceEvents[event.device].btnState>0)||(sMiceEvents[event.device].btnLinkState>0)||(sMode[event.device] == MMM_INTEGRATOR)){
          sMiceEvents[event.device].mAbsArray[event.item] += event.value;
        }
      }
      sMiceEvents[event.device].lastEvent   = RELMOTION; 
      sMiceEvents[event.device].hasChanged  = 1;
      sMiceEvents[event.device].lastTime    = timeNow; 
      sMiceEvents[event.device].lastRelTime = timeNow; 
      break;
    case MANYMOUSE_EVENT_BUTTON:
      if(event.item==0){
        if(event.value==0){
          sMiceEvents[event.device].btnUp ++;
          sMiceEvents[event.device].btnState = 0;
        }else{
          sMiceEvents[event.device].btnDown   ++;
          sMiceEvents[event.device].btnState = 1;
          for(int i=0;i<6;i++) sMiceEvents[event.device].mAbsArray[i] = 0;
        }
      }
      sMiceEvents[event.device].lastEvent  = BUTTON; 
      sMiceEvents[event.device].hasChanged = 1;
      sMiceEvents[event.device].lastTime   = timeNow; 
      break;
    case MANYMOUSE_EVENT_DISCONNECT:
      sMiceEvents[event.device].lastEvent  = DISCONNECT; 
      sMiceEvents[event.device].hasChanged = 1;
      sMiceEvents[event.device].lastTime   = timeNow; 
      break;
    default:
      break;
    }
  }  
  
  for(int i=0;i<sNumDevices;i++){
    if(sMode[i] == MMM_RELTOABS){
      sMiceEvents[i].mAbs = sMiceEvents[i].mRel;
    }
  }

  }
    

  // setting the mouse-->ID mapping      
  if(mMouseToId>=0) {
    if(sLastDeviceTouched>=0){
      mMiceMap[sLastDeviceTouched] = mMouseToId;
      // unset the old mapping
      for (int i=0;i<sNumDevices;i++) {
        if(i!=int(sLastDeviceTouched)) {
          if(mMiceMap[i] == mMouseToId) {
            mMiceMap[i] = -1;
          }
        }
      }
      // unset/set flag
      mMouseToId = -1;
    }
  }
  
  for(int i=0;i<mNumMice;i++){
      ClearMiceData(i);
  }
  for(int i=0;i<sNumDevices;i++){
    if(mMiceMap[i]>=0){
      memcpy(&mMice[mMiceMap[i]],&sMiceEvents[i],sizeof(MouseEventSummary));
    }
  }
}

/*
void MMiceDeviceDriver::PrintSummary(){
  for(int i=0;i<sNumDevices;i++){
    if(mValidDevices[i]){
        cout << "Device " << sMiceEvents[i].device <<" Mapped to "<<mMiceMap[i]<<endl;
        cout << "  Delta:    (" << sMiceEvents[i].mRedelX << "," << sMiceEvents[i].delY<<"," << sMiceEvents[i].delZ<< ")" << endl;
        cout << "            (" << sMiceEvents[i].delRX << "," << sMiceEvents[i].delRY<<"," << sMiceEvents[i].delRZ<< ")" << endl;
        cout << "  Absolute: (" << sMiceEvents[i].absX << "," << sMiceEvents[i].absY<< ")" << endl;
        cout << "  Scroll:   (" << sMiceEvents[i].scrX << "," << sMiceEvents[i].scrY<< ")" << endl;
        cout << "  ButtonEv: (" << sMiceEvents[i].btnDown << "," << sMiceEvents[i].btnUp<< ")=> " << sMiceEvents[i].btnState<<"("<<sMiceEvents[i].btnLinkState<<")"<<endl;
        cout << "  lastEvent: " << sMiceEvents[i].lastEvent << endl;
        cout << "  Changed:   " << sMiceEvents[i].hasChanged << endl;
    }
  }

}

void MMiceDeviceDriver::PrintSummary2(){
  for(int i=0;i<sNumDevices;i++){
    if(mValidDevices[i]){
        cout << sMiceEvents[i].hasChanged<<",";
        cout << sMiceEvents[i].mRel.X << ","<< sMiceEvents[i].mRel.Y << ","<< sMiceEvents[i].mRel.Z << ",";
        cout << sMiceEvents[i].mRel.RX << ","<< sMiceEvents[i].mRel.RY << ","<< sMiceEvents[i].mRel.RZ << endl;
        cout << sMiceEvents[i].hasChanged<<",";
        cout << sMiceEvents[i].mAbs.X << ","<< sMiceEvents[i].mAbs.Y << ","<< sMiceEvents[i].mAbs.Z << ",";
        cout << sMiceEvents[i].mAbs.RX << ","<< sMiceEvents[i].mAbs.RY << ","<< sMiceEvents[i].mAbs.RZ << endl;
    }
  }        
}
*/

bool MMiceDeviceDriver::TouchAndSetMouseId(int id){

  if (id>=mNumMice) { 
    fprintf(stderr,"MMice: Error: ID exceeds the number mices (%d)\n",mNumMice);
    return false;
  }

  if (id>=0) {
    mMouseToId = id;
    return true;
  }    
  
  return false;

}

bool MMiceDeviceDriver::ResetMap(){
  if(!bIsReady) return false;
    
  int count=0;
  for (int i=0; i<sNumDevices; i++){
    if(mValidDevices[i]){
      if(count < mNumMice){
        mMiceMap[i] = count;    
      }else{
        mMiceMap[i] = -1;        
      }
      count++;
    }else{
      mMiceMap[i] = -1;
    }
  }
  return true;
}    

bool MMiceDeviceDriver::InitMap(int nmice) {
  if(nmice<0){
    fprintf(stderr,"MMice: Error: Number of number of mice (%i) must be positive...\n" ,nmice);
    return false;
  }

  // set mMice and mNumMice
  if(!mMice) {    
    mMice    = new MouseEventSummary[nmice];
    mNumMice = nmice;
  }else{
    // copy existing data
    MouseEventSummary * tmp = new MouseEventSummary[nmice];
    for (int i=0; i<nmice; i++) { 
      tmp[i] = mMice[i];
    }
    delete [] mMice;    
    mMice  = tmp;

    mNumMice = nmice;
  } 

  ResetMap();

  return true;
}









bool  MMiceDeviceDriver::LoadMap(const char * filename){

  fprintf(stderr,"MMice: Loading mice map: %s\n",filename);

  ifstream file;
  file.open(filename);  
  
  if(file.is_open()){
    int  n;
    char dummy[256];
    file >> n;
    file.getline(dummy,256);
    char **names;
    int *used;
    names = new char* [n];
    used  = new int   [n];
    for(int i=0;i<n;i++){
      names[i] = new char[256];  
    }
    for(int i=0;i<n;i++){      
      file.getline(names[i],256);      
      used[i] = 0;
      fprintf(stderr,"MMice:   %d <- %s\n",i,names[i]);
    }
    
    InitMap(n);
    
    for(int i=0;i<sNumDevices;i++){
      bool bFound = false;
      
      for (int j=0; j<n; j++) {
        if (strcmp(GetDeviceName(i),names[j])==0) {
          mMiceMap[i] = j;
          used[j] = 1;
          bFound = true;
          break;            
        }
      }
      
      if(!bFound){
        mMiceMap[i] = -1;  
      }
    }
    
    for(int i=0;i<n;i++){
      if(used[i]==0){
        if(strcmp("None",names[i])!=0)
          fprintf(stderr,"MMice: Warning: Entry %s was not found in the device list...\n",names[i]);
      }  
    }

    fprintf(stderr,"MMice: Loading mice map ok...\n");
    
    for(int i=0;i<n;i++) delete [] names[i];  
    delete [] names;
    delete [] used;     
    
    return true;
  }

  fprintf(stderr,"MMice: Error: Loading mice map failed...\n");  
  return false;

}

bool  MMiceDeviceDriver::SaveMap(const char * filename){  

  ofstream file;
  file.open(filename);

  if(file.is_open()){
    file << mNumMice << endl;    
    for(int i=0;i<mNumMice;i++){
      bool bFound = false;
      for(int j=0;j<sNumDevices;j++){
        if(mMiceMap[j]==i){
          file << GetDeviceName(j) << endl;
          bFound = true;
          break;
        }
      }
      if(!bFound)
        file << "None"<<endl;
    }  
    file.close();
    fprintf(stderr,"MMice: Saving mice map: %s ok...\n",filename);
    return true;
  }
  fprintf(stderr,"MMice: Error: Saving mice map: %s failed...\n",filename);  
  return false;  
}




int    MMiceDeviceDriver::LinkMiceButtons(int *ids, int len){
  if((ids==NULL)||(len==0))
    return -1;
  int resId = -1;
  if(mButtonLinks==NULL){
    mButtonLinks     = new int*[1];
    mButtonLinksSize = 1;
    resId = 0;  
  }else{
    for(int i=0;i<mButtonLinksSize;i++){
      if(mButtonLinks[i]==NULL){
        resId = i;
        break;  
      }
    }
    if(resId<0){
      int **tmp = new int*[mButtonLinksSize+1];
      for(int i=0;i<mButtonLinksSize;i++){
        tmp[i] = mButtonLinks[i]; 
      }
      delete [] mButtonLinks;
      mButtonLinks = tmp;
      resId = mButtonLinksSize;
      mButtonLinksSize++;
    }
  }
  mButtonLinks[resId] = new int[len+1];
  for(int i=0;i<len;i++){
    mButtonLinks[resId][i] = ids[i];  
  }
  mButtonLinks[resId][len] = -1;
  return resId;
}

void   MMiceDeviceDriver::UnlinkMiceButtons(int linkId){
  if((linkId<0)||(linkId>=mButtonLinksSize))
    return;
  if(mButtonLinks[linkId]!=NULL)
    delete [] mButtonLinks[linkId];
  mButtonLinks[linkId] = NULL;  
}

