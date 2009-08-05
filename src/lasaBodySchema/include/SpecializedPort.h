// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008 Micha Hersch, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
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
#ifndef __SPECIALIZED_PORT_H__
#define __SPECIALIZED_PORT_H__

#include "yarp/os/all.h"
#include <iostream>

using namespace yarp::os;
using namespace std;

class CartesianDataPort : public BufferedPort<Bottle>{

 public:
  //  SendPosition(float x,float y,float z);
  void SendPosition(const float *position,bool strict=true);
  void SendPositionAndOrientation(const float *position,const float *orientation,bool strict=true);
  bool ReadPosition(float *position);
  int ReadPositionAndOrientation(float *position, float *orientation);
  
};

class JointAngleDataPort : public BufferedPort<Bottle>{

 protected:
  
  int *mapping;
  bool checked;
  int mapping_size;
    int data_size;
    
public:
    JointAngleDataPort();
    bool SetMapping(const int *map, int m_size);
    void SendPosition(const float *position, const int size,bool strict=true);
    bool ReadPosition(float *position, int max_size,bool shoudWait=false);
    void SendWithMapping(const float *position, const int size,bool strict=true);
    bool ReadWithMapping(float *position, int max_size);
    int GetSize(){return data_size;}
};


class BodySchemaDataPort : public BufferedPort<Bottle>{
protected:
   int data_size;
 public:
    BodySchemaDataPort();
    void SendBodySchema(const float *data, int nb_dofs,bool strict=true);
    bool ReadBodySchema(float *data, int max_nb_dofs);
    int  GetSize(){return data_size;}
};


class CommandPort : public BufferedPort<Bottle>{
public:
    void SendQuit();
    //   int ReadCmd();
    bool ShouldQuit();
};

class VisualRotationPort : public BufferedPort<Bottle>{
public:
    void SendRotation(const float *rot, double t0, double t1) ;
    bool ReadRotation(float *rot, double *t0, double *t1) ;
};

#endif

