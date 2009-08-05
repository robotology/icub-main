// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Micha Hersch - LASA - EPFL
 * email:   micha.hersch@robotcub.org
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
#include "SpecializedPort.h"

void CartesianDataPort::SendPosition(const float *position, bool strict){
  Bottle& b = prepare();
  b.clear();
  for(int i=0;i<3;i++){
    b.addDouble(position[i]);
  }
  //  cout <<"sending "<<b.toString().c_str()<<endl;
  write(strict);
}


void CartesianDataPort::SendPositionAndOrientation(const float *position, const float *orientation, bool strict){
  Bottle& b = prepare();
  b.clear();
  for(int i=0;i<3;i++){
    b.addDouble(position[i]);
  }
  for(int i=0;i<3;i++){
    b.addDouble(orientation[i]);
  }
  //  cout <<"sending "<<b.toString().c_str()<<endl;
  write(strict);
}

bool CartesianDataPort::ReadPosition(float *position){
  Bottle *b = read(false);
  if(b){
    for(int i=0;i<3;i++){
      position[i] =b->get(i).asDouble();
    }
    return true;
  }
  return false;
}

int CartesianDataPort::ReadPositionAndOrientation(float *position, float *orientation){
  Bottle *b = read(false);
  if(b){
    if(b->size()>=3){
      for(int i=0;i<3;i++){
	position[i] =b->get(i).asDouble();
      }
    }
    if(b->size()>=6){
      for(int i=0;i<3;i++){
	orientation[i] =b->get(i+3).asDouble();
      }
    }
    return b->size();
  }
  return 0;
}

JointAngleDataPort::JointAngleDataPort(){
  mapping_size =0;
  mapping =NULL;
  data_size = 0;
}


bool JointAngleDataPort::SetMapping(const int *map, int m_size){
  if(m_size!=mapping_size){
    if(mapping) delete [] mapping;
    mapping = new int[m_size];
  }
  mapping_size = m_size;
  for(int i=0;i<mapping_size;i++){
    mapping[i] = map[i];
  }
  return true;
}


void JointAngleDataPort::SendPosition(const float *position, int size,bool strict){
  Bottle& b = prepare();
  b.clear();
  for(int i=0;i<size;i++){
    b.addDouble(position[i]);
  }
  //  cout <<"sending "<<b.toString().c_str()<<endl;
  write(strict);
}

bool JointAngleDataPort::ReadPosition(float *position, int max_size,bool shouldWait){
  Bottle *b = read(shouldWait);
  if(b){
    int si=data_size=b->size();
    if(si>max_size){
  //     cout<<"warning: cannot read all data in JointAngleDataPort::ReadPosition() ("
//           <<max_size<<" instead of "<<si<<" )"<<endl;
      si =max_size;
    }
    for(int i=0;i<si;i++){
      position[i] =b->get(i).asDouble();
    }
    return true;
  }
  return false;
}


void JointAngleDataPort::SendWithMapping(const float *position, const int size,bool strict){
  Bottle& b = prepare();
  b.clear();
  for(int i=0;i<size;i++){
    b.addInt(mapping[i]);
    b.addDouble(position[i]);
  }
  //  cout <<"sending "<<b.toString().c_str()<<endl;
  write(strict);
}

bool JointAngleDataPort::ReadWithMapping(float *position, int max_size){
  Bottle *b = read(false);
  if(b){
    int si = b->size();
    if(si>max_size*2){
        cout<<"not enough space allocated to JointAngleDataPort::ReadWithMapping  ("<<si<<" > 2*"<<max_size<<")"<<endl; 
      si=max_size*2;
    }
    for(int i=0;i<si;i=i+2){
      position[b->get(i).asInt()]=b->get(i+1).asDouble();
    }
    return true;
  }
  return false;
} 

BodySchemaDataPort::BodySchemaDataPort(){
    data_size=0;
}

void BodySchemaDataPort::SendBodySchema(const float *bodyschema, int size,bool strict){
  Bottle& b = prepare();
  b.clear();
  for(int i=0;i<size;i++){
      b.addDouble(bodyschema[i]);
  }
  //  cout <<"sending "<<b.toString().c_str()<<endl;
  write(strict);
}

bool BodySchemaDataPort::ReadBodySchema(float *bodyschema, int max_size){
  Bottle *b = read(false);
  if(b){
    int si=data_size = b->size();
    if(si>max_size){
      cout<<"warning: cannot read all data in BodySchemaDataPort::ReadBodySchema()"<<endl;
      si =max_size;
    }
    for(int i=0;i<si;i++){
      bodyschema[i] =b->get(i).asDouble();
    }
    return true;
  }
  return false;
}


void CommandPort::SendQuit(){
    Bottle& b = prepare();
    b.clear();
    b.fromString("quit");
    write();
}

bool CommandPort::ShouldQuit(){
    Bottle *b = read(false);
    if(b){
        if(b->toString()==ConstString("quit")){
            return true;
        }
    }
    //    cout<<"no msg"<<endl;
    return false;
}

void VisualRotationPort::SendRotation(const float *rot, double t0, double t1){
    Bottle& b = prepare();
    b.clear();
    for(int i=0;i<3;i++){
        b.addDouble(rot[i]);
    }
    b.addDouble(t0);
    b.addDouble(t1);
    write();
}

bool VisualRotationPort::ReadRotation(float *rot, double *t0, double *t1){
  Bottle *b = read(false);
  if(b){
    int si=b->size();
    if(si!= 5){
        cout<<"warning: wrong format in VisualRotation::ReadPosition expecting 5 doubles"<<endl;
        return false;
    }
    for(int i=0;i<3;i++){
      rot[i] =b->get(i).asDouble();
    }
    *t0 = b->get(3).asDouble();
    *t1 = b->get(4).asDouble();
    return true;
  }
  return false;
}
