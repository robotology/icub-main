

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include "ControlBasisResource.h"
#include "YARPConfigurationVariables.h"
#include "EndEffectorCartesianPosition.h"
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
  //YARPConfigurationVariables *resource;
  bool resourceRunning;

public:

  ControlBasisResourceModule() 
  {
    resourceRunning = false;
  }

  /*  ~ControlBasisResourceModule() 
  {
  }
  */

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
      configFile = "/home/hart/iCub/src/controlBasis/config/"+configFile;
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
    cout << configFile << endl;

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
