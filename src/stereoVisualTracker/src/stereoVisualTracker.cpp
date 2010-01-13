// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2007 Micha Hersch, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   <firstname.secondname>@robotcub.org
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
#include "iCub/stereoVisualTracker.h"

//#define NO_WAIT_KEY

bool stereoVisualTracker::updateModule(){
#ifndef NO_WAIT_KEY 
  int key =cvWaitKey(10);
  ExecuteCommand(key);
#endif
  ProcessNextFrame();
  return finish==0;

#ifdef RECORDING
  writer[0]=NULL;
  writer[1]=NULL;
#endif
}

bool stereoVisualTracker::open(Searchable& config){
  Value& dir = config.find("directory");
  if(dir.isNull()){
      InitVision();
  }
  else{
      InitVision(dir.asString().c_str());
  }
  cout<<"vision initialized"<<endl;
  // port.SetPortName(getName("position:o"));
      double chess_p[3];

      
      Value& w = config.find("width");
      if(!w.isNull()){
          chess_p[0] = w.asDouble();
      }
      else{
          cout<<"cannot find chessboard width"<<endl;
          return false;
      }
      Value& h = config.find("height");
      if(!h.isNull()){
          chess_p[1] = h.asDouble();
      }
      else{
          cout<<"cannot find chessboard height"<<endl;
          return false;
      }
      Value& s = config.find("size");
      if(!s.isNull()){
          chess_p[2] = s.asDouble();
      }
      else{
          cout<<"cannot find chessboard size"<<endl;
          return false;
      }
      
      for(int i=0;i<MAX_CAM;i++){
          dc[i]->SetChessboardParams(chess_p);
      }
      char tmp[30];
      for(int j=0;j<MAX_OBJECTS;j++){
          sprintf(tmp,"vision%d:o",j);
          port[j].SetPortName(getName(tmp).c_str());
      }
#ifdef RECORDING
      Value& rec = config.find("record");
      if(!rec.isNull()){
          writer[0] = new AviWriter("mov0.avi");
          writer[1] = new AviWriter("mov1.avi");
      }
#endif

    mPeriod = 0;
    Value& p = config.find("period");
    if(!p.isNull()){
        mPeriod = p.asDouble();
    }
  return true;
}


bool stereoVisualTracker::close(){
  for(int i=0;i<MAX_CAM;i++){
    grabber[i]->Kill();
  }
  return true;
}

bool  stereoVisualTracker::InitGrabber(int i){
  char name [80];
  grabber[i] = new YarpFrameGrabber();
  sprintf(name,"%s%d:i",getName("image").c_str(),i);
  return grabber[i]->Init(name);
}


void stereoVisualTracker::SendOutput(VisionServer *port, double *position){//to check
    port->setEnvelope(((YarpFrameGrabber *)grabber[0])->stamp);
    port->SendPosition(position[0],position[1],position[2]);
}
double 	stereoVisualTracker::getPeriod (){
    return mPeriod;
}
int main(int argc, char *argv[]){
  stereoVisualTracker svt;
  svt.openFromCommand(argc,argv,true);
  svt.runModule();
}
