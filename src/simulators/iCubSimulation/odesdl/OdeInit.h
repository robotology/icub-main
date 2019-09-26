// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Vadim Tikhanoff
* email:   vadim.tikhanoff@iit.it
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
/**
 * \file OdeInit.h
 * \brief This file is responsible for the initialisation of the world parameters that are controlled by ODE. 
 * Some extra parameters are/can be added here for simulation stability
 * \author Vadim Tikhanoff
 * \date 2007
 * \note Released under GNU GPL v2.0
 **/

#ifndef ICUBSIMULATION_ODEINIT_INC
#define ICUBSIMULATION_ODEINIT_INC

#include "iCub.h"
#include "world.h"
#include <time.h>
#include <yarp/os/Semaphore.h>
#include "iCubLogicalJoints.h"
#include "iCubSimulationControl.h"
#include "iCubSimulationIMU.h"
//#include <vector>

#include "RobotConfig.h"

#include <list>

using namespace std;
using namespace yarp::dev;

class ICubSim;

/**
 *
 * ODE state information.
 *
 */
class OdeInit {
public:
    double SimTime;
    dWorldID world;
    dSpaceID space;
    dJointGroupID contactgroup;
    dGeomID ground;
    //dJointFeedback *feedback;
    //dJointFeedback *feedback1;
    //dJointFeedback *feedback_mat;
    yarp::os::Semaphore mutex;
    yarp::os::Semaphore mutexTexture;
    ICubSim *_iCub;
    worldSim *_wrld;
    bool stop;
    bool sync;
    //bool verbose;
    int verbosity;
    string name;
    iCubSimulationControl **_controls;
    iCubSimulationIMU * _imu{nullptr};
    double contactFrictionCoefficient; //unlike the other ODE params fron .ini file that are used to intiialize the properties of the simulation (dWorldSet...),
    //This parameter is employed on the run as contact joints are created (in OdeSdlSimulation::nearCallback() )
    //for whole_body_skin_emul
    struct contactOnSkin_t {
        dGeomID body_geom_id;
        dSpaceID body_geom_space_id;
        int body_index; //if there are two iCub bodies colliding, they share the same contact geom and joint - but we need to know that they are two different 
        dContactGeom contact_geom;
        dJointID contact_joint;
    };
    list<contactOnSkin_t> listOfSkinContactInfos;
  

    void setName( string module ){
        name = module;
    }
    string getName(){
        return name;
    }

    ~OdeInit();

    void setSimulationControl(iCubSimulationControl *control, int part);
    void removeSimulationControl(int part);
    void setSimulationIMU(iCubSimulationIMU *imu);
    void removeSimulationIMU();
    static OdeInit& init(RobotConfig *config);
    void sendHomePos();

    static OdeInit& get();

    static void destroy();
    
    static void printGeomClassAndNr(int geom_class, int geom_nr);
    static void printInfoOnSpace(dSpaceID my_space,const std::string & my_space_name);
    

private:
    OdeInit(RobotConfig *config);
    static OdeInit *_odeinit;

    RobotConfig *robot_config;
};

#endif
