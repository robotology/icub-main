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
#ifndef __PERCEPTION_SIMULATION_MODULE_H__
#define __PERCEPTION_SIMULATION_MODULE_H__

#include "KinematicTree.h"
#include "BodySchema.h"
#include "yarp/os/all.h"
#include "SpecializedPort.h"
#include "yarp/os/Stamp.h"

class PerceptionSimulationModule: public Module{

 protected:

    KinematicTree *body;
    KinematicChain *eyes_arm;
    KinematicChain *eye_world;
    KinematicChain *head;
    KinematicChain *arm;
    
    float delay;
    
    joint_vec_t arm_angles;
    joint_vec_t head_angles;
    cart_vec_t position;
    cart_vec_t vis_rot;

    Rotation rot;

    JointAngleDataPort efferent_head;
    JointAngleDataPort efferent_arm;
    JointAngleDataPort proprioception_head;
    JointAngleDataPort proprioception_arm;
   
    CartesianDataPort vision;
    VisualRotationPort visualRotation;    

    CommandPort cmdPort;

    Stamp stamp;
    double last_time;

protected:
    int loadChain(KinematicChain *chain,Searchable &s,
                  const char *from, const char *to, 
                  const char *from_def, const char *to_def);

public:
    PerceptionSimulationModule();
    ~PerceptionSimulationModule();
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool updateModule();
    //  virtual bool respond(const Bottle& command, Bottle& reply);
};


#endif
