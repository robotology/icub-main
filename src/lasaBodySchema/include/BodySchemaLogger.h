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

#ifndef __BODY_SCHEMA_LOGGER_H__
#define __BODY_SCHEMA_LOGGER_H__

#include "yarp/os/all.h"
#include "yarp/os/Stamp.h"
#include "SpecializedPort.h"
//#include "KinematicTree.h"
#include <iostream>
#include <fstream>
class BodySchemaLogger :public Module{

 protected:
    BodySchemaDataPort body_schema_port;
    JointAngleDataPort head_port;
    JointAngleDataPort arm_port;
    CartesianDataPort vision_port;
    VisualRotationPort visual_rotation_port;
    
    CommandPort cmd;
  
    int body_data_size;
    int head_size;
    int arm_size;

    float *body_data;
    float *head_position;
    float *arm_position;
    
    float vision[4];
    float visual_rotation[5];

    bool binary;

    ofstream body_schema_file;
    ofstream head_file;
    ofstream arm_file;
    ofstream vision_file;
    ofstream visual_rotation_file;


// protected:
//   int loadChain(KinematicChain *chain,Searchable &s,const char *from, 
// 		const char *to, const char *from_def, const char *to_def);

protected:
  void  putInFile(ofstream& out, const float *data, int size);

 public:
  virtual bool open(Searchable& config);
  virtual bool close();
  virtual bool updateModule();
};


#endif
