// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/**
*
@ingroup icub_module
\defgroup icub_controlBasisResource controlBasisResource

A generic ControlBasis resource module for typed sensors & effectors

\section intro_sec Description
This module allows a user to start up a control basis module such as a 
configuration resource, position sensor (such as the end-effector
location of a link), etc.  These resources can be used by control basis
controllers to create co-artciculated control programs.

\section lib_sec Libraries
YARP

\section parameters_sec Parameters

--type: the specific type of resoruce (e.g., yarpConfiguration, 
endEffector, triangulatedPosition, etc.).  This type shouldn't be 
confused by the formal type of the sensor (configuration, cartesianposition,
etc.).

--robot: specifies the name of the robot. It will be used to form 
the names of the ports created and accessed by module.

--part: part to control (e.g. head,arm_right, arm_left, lef_right...), it 
will be used to form the names of the ports created and accessed by the 
module.

--cbName: if you want to create a special name for the resource, do it here. The
default will be "/robot/part"

--configFile: the name of the config file to load for the resource

--velocityMode: [0/1] if the resource is a yarpConfiguration, you can control 
it in either position or velocity mode.

--simulationMode: [0/1] if the resource comes from a simulated source.

--updateDelay: the update delay (in ms) that resource will wait between upates.


\section portsa_sec Ports Accessed
For the iCub, it assumes \ref icub_iCubInterface and \ref icub_velocityControl run for the necessary 
parts. 

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None.

\section conf_file_sec Configuration Files
None.

\section tested_os_sec Tested OS
Linux.

\section example_sec Example Instantiation of the Module

controlBasisResource --type yarpConfiguration --robot icub --part right_arm --configFile right_arm.dh 

Starts the module using the robot icub to control the right arm in velocity 
mode (default). it reads DH parameters and joint/link numbers from right_arm.dh

\author Stephen Hart

Copyright (C) 2010 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/controlBasis/modules/main.cpp.
**/

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include "ControlBasisResource.h"
#include "YARPConfigurationVariables.h"
#include "EndEffectorCartesianPosition.h"
#include "YARPAttentionMechanismHeading.h"
#include "YARPAttentionMechanismStereoHeading.h"
#include "Force.h"
#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>

#include <iostream>
#include <string.h>

using namespace yarp::os;
using namespace yarp::dev;

using namespace std;
using namespace CB;

class ControlBasisResourceModule : public RFModule {

  Port handlerPort; // to handle messagees

  ControlBasisResource *resource;
  bool resourceRunning;

public:

  ControlBasisResourceModule() 
  {
    resourceRunning = false;
  }

  ~ControlBasisResourceModule() 
  {
    if (resourceRunning) {
      resource->stopResource();
      delete resource;
    }
  }
  

  bool configure(ResourceFinder &rf) 
  {

    string resourceType = "";
    string configFile = "";
    string yarpName = "";
    string cbName = "";
    string velPortName = "";
    string part = "";
    string robot = "";
    double updateDelay = -1;
    bool simulationMode = true;
    bool velocityMode = false;

    string iCubDir(getenv("ICUB_ROOT"));
    if(rf.check("type")) {
      resourceType=rf.find("type").asString().c_str();
    } 
    if(rf.check("part")) {
      part=rf.find("part").asString().c_str();
    } 
    if(rf.check("robot")) {
      robot=rf.find("robot").asString().c_str();
    }
    if(rf.check("configFile")) {
      configFile= rf.find("configFile").asString().c_str();
      configFile = iCubDir+"/app/controlBasis/conf/"+configFile;
    }
    if(rf.check("cbName")) {
      cbName=rf.find("cbName").asString().c_str();
    }
    if(rf.check("updateDelay")) {
      updateDelay=rf.find("updateDelay").asDouble();
    }
    if(rf.check("velocityMode")) {
      velocityMode=(bool)(rf.find("velocityMode").asInt());
    }

    cout << resourceType << endl;
    cout << robot << endl;
    cout << part << endl;
    cout << endl<< endl<< endl;
    cout << configFile << endl;
    //    cout << endl<< endl<< endl;
    if( (part == "") || (robot == "") ) {
      return false;
    }

    yarpName = robot + "/" + part;
    velPortName = robot + "/vc/" + part;
    if(cbName == "") cbName = yarpName;
    
    if(resourceType=="yarpConfiguration") {
        resource = new YARPConfigurationVariables(cbName,yarpName);
        if(configFile != "") { 
            ((YARPConfigurationVariables *)resource)->loadConfig(configFile);
        }
        if(updateDelay != -1) ((YARPConfigurationVariables *)resource)->setUpdateDelay(updateDelay);
        if(velocityMode) ((YARPConfigurationVariables *)resource)->setVelocityControlMode(velocityMode,velPortName);
    } else if(resourceType=="endEffector") {
        if(yarpName == "") {
            return false;
        }
        resource = new EndEffectorCartesianPosition(yarpName);
        if(!((EndEffectorCartesianPosition *)resource)->connectToConfiguration()) return false;
    } else {
        cout<<"unknown resoruce type..."<<endl;
        return false;
    }
    
    cout<<"starting resource..."<<endl;
    resource->startResource();
    resourceRunning = true;

    return true;
  }

  virtual bool respond(const Bottle &command, Bottle &reply) 
  {
      if(command.get(0).asString()=="quit")
          return false;
      else 
          reply=command;
      return true;
  }
    
    virtual bool close()
    {
        cout << "Closing CB Resource Module..." << endl; 
        if (resourceRunning) {
            resource->stopResource();
            delete resource;
        }
        resourceRunning = false;
        return true;
    }
    
    double getPeriod() {
        return 1;
    }
    
    bool updateModule() {
        return true;
    }
    
};

int main(int argc, char *argv[])
{
    Network yarp;
    ControlBasisResourceModule mod;
    
    ResourceFinder rf;
    rf.setDefault("type","yarpConfiguration");
    rf.setDefault("part","right_arm");
    rf.setDefault("robot","/icubSim");
    rf.setDefault("configFile","right_arm.dh");
    rf.setDefault("velocityMode","1");
    rf.setDefault("updateDelay","0.1");
    rf.setDefault("simulationMode","1");

    rf.configure("ICUB_ROOT", argc, argv);
    rf.setVerbose(true);

    cout<<"Starting CB Resource Module..."<<endl;
    return mod.runModule(rf);

}
