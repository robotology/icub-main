// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/**
*
@ingroup icub_module
\defgroup icub_iCubControlBasisResources iCubControlBasisResources

A module to start the ControlBasis resources for the iCub (or simulator)

\section intro_sec Description
This module allows a user to start up the iCub resources for use in
Control Basis applications.  The options include Configuration resources 
for all iCub parts (including the torso+arm "full arm"), and the Cartesian
Positions of the arm and leg "end-effectors."

\section lib_sec Libraries
YARP

ControlBasis

\section parameters_sec Parameters

--simulation [true | false]: specifies whether using the simulated iCub. Default=true.

--velocityControlMode [true | false]: specifies whether the configuration resources should send position deltas to velocityControl modules or to PolyDrivers. Default=false; 

--from [<filename>.ini]: the name of the config file to load. Can specify either real-robot or simulated-robot resources. OPTIONAL

\section portsa_sec Ports Accessed
For the iCub, it assumes \ref icub_iCubInterface and \ref icub_velocityControl run for the necessary 
parts. For the iCub simulator, only \ref icub_iCubInterface needs to run.

\section conf_file_sec Configuration Files

<tt>iCubControlBasisResources.ini</tt>
<tt>iCubSimControlBasisResources.ini</tt>

Here's how the configuration file will look like: 
 
\code 
simulation false
velocityControlMode true               

[configuration_resources]                 
right_arm      1
left_arm       1
right_leg      0
left_leg       0
right_hand     1
left_hand      1
right_full_arm 0
left_full_arm  0
torso          0
head           1
eyes           0

[cartesianposition_resources]
right_arm      1
left_arm       1
right_leg      0
left_leg       0
right_full_arm 0
left_full_arm  0

//DO NOT REMOVE THIS LINE                 
\endcode 
 
\section tested_os_sec Tested OS
Linux.

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

#include <YARPConfigurationVariables.h>
#include <iCubFullArmConfigurationVariables.h>
#include <EndEffectorCartesianPosition.h>
#include <ManipulatorPositionJacobian.h>
#include <Controller.h>
#include <RunnableControlLaw.h>
#include <YARPAttentionMechanismHeading.h>

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>

#include <iostream>
#include <string.h>

using namespace yarp::os;
using namespace yarp::dev;

using namespace std;
using namespace CB;

class iCubControlBasisResourceStarter : public RFModule {

    Port handlerPort; // to handle messagees

    iCubFullArmConfigurationVariables *iCubFullArm[2];
    YARPConfigurationVariables *iCubArm[2];
    YARPConfigurationVariables *iCubLeg[2];
    YARPConfigurationVariables *iCubHand[2];
    YARPConfigurationVariables *iCubEyes;
    YARPConfigurationVariables *iCubHead;
    YARPConfigurationVariables *iCubTorso;

    EndEffectorCartesianPosition *armEndEffector[2]; 
    EndEffectorCartesianPosition *legEndEffector[2]; 
    EndEffectorCartesianPosition *fullArmEndEffector[2]; 

    YARPAttentionMechanismHeading *salientHeadings[2];

    bool runArmConfig[2];
    bool runLegConfig[2];
    bool runHandConfig[2];
    bool runFullArmConfig[2];
    bool runEyesConfig;
    bool runTorsoConfig;
    bool runHeadConfig;

    bool runArmPosition[2];
    bool runFullArmPosition[2];
    bool runLegPosition[2];

    bool runHeadings[2];

    string fullArmName[2];
    string fullArmConfigFile[2];
    string fullArmVelPort[2];

    string armName[2];
    string armConfigFile[2];
    string armVelPort[2];

    string legName[2];
    string legConfigFile[2];
    string legVelPort[2];

    string handName[2];
    string handConfigFile[2];
    string handVelPort[2];

    string eyesName;
    string eyesConfigFile;
    string eyesVelPort;

    string headName;
    string headConfigFile;
    string headVelPort;

    string torsoName;
    string torsoConfigFile;
    string torsoVelPort;

    string headingName[2];
    string headingVelPort[2];

    int armNumJoints;
    int armNumLinks;

    int legNumJoints;
    int legNumLinks;

    int handNumJoints;
    int handNumLinks;

    int headNumJoints;
    int headNumLinks;

    int torsoNumJoints;
    int torsoNumLinks;

    int eyesNumJoints;
    int eyesNumLinks;
    
    string robot_prefix;
    bool simulationMode;
    bool velocityControlMode;
    bool resourcesRunning;

public:
    
    iCubControlBasisResourceStarter() :
        resourcesRunning(false) 
    {
    }

    ~iCubControlBasisResourceStarter() 
    {
        if (resourcesRunning) {
            stopResources();
        }

        if(resourcesRunning) {
            for(int i=0; i<2; i++) {
                if(runFullArmConfig[i]) delete iCubFullArm[i];
                if(runArmConfig[i]) delete iCubArm[i];
                if(runLegConfig[i]) delete iCubLeg[i];
                if(runHandConfig[i]) delete iCubHand[i];
                if(runArmPosition[i]) delete armEndEffector[i]; 
                if(runLegPosition[i]) delete legEndEffector[i]; 
                if(runFullArmPosition[i]) delete fullArmEndEffector[i];                 
                if(runHeadings[i]) delete salientHeadings[i];
            }     
            if(runHeadConfig) delete iCubHead;
            if(runTorsoConfig) delete iCubTorso;
            if(runEyesConfig) delete iCubEyes;                
        }

    }
    

    bool configure(ResourceFinder &rf) 
    {

        // get configuration flags
        if (rf.check("simulation")) 
            simulationMode=(rf.find("simulation").asString() == "true");        

        if (rf.check("velocityControlMode")) 
            velocityControlMode=(rf.find("velocityControlMode").asString()=="true");

        cout << "Configuring resources with simulation="<<simulationMode<<",velMode="<<velocityControlMode<<endl;

        // parse configuration resources to run
        Bottle &config_group=rf.findGroup("configuration_resources");

        if(config_group.check("right_arm")) {
            runArmConfig[0] = (bool)(config_group.find("right_arm").asInt());
        } else {
            runArmConfig[0] = false;
        }
        //cout << "run right arm config: " << runArmConfig[0] << endl;

        if(config_group.check("left_arm")) {
            runArmConfig[1] = (bool)(config_group.find("left_arm").asInt());;
        } else {
            runArmConfig[1] = false;
        }
        //cout << "run left arm config: " << runArmConfig[1] << endl;

        if(config_group.check("right_full_arm")) {
            runFullArmConfig[0] = (bool)(config_group.find("right_full_arm").asInt());
        } else {
            runFullArmConfig[0] = false;
        }
        //cout << "run right full arm config: " << runFullArmConfig[0] << endl;

        if(config_group.check("left_full_arm")) {
            runFullArmConfig[1] = (bool)(config_group.find("left_full_arm").asInt());;
        } else {
            runFullArmConfig[1] = false;
        }
        //cout << "run left full arm config: " << runFullArmConfig[1] << endl;

        if(config_group.check("right_leg")) {
            runLegConfig[0] = (bool)(config_group.find("right_leg").asInt());
        } else {
            runLegConfig[0] = false;
        }
        //cout << "run right leg config: " << runLegConfig[0] << endl;

        if(config_group.check("left_leg_arm")) {
            runLegConfig[1] = (bool)(config_group.find("left_leg").asInt());;
        } else {
            runLegConfig[1] = false;
        }
        //cout << "run left leg config: " << runLegConfig[1] << endl;

        if(config_group.check("right_hand")) {
            runHandConfig[0] = (bool)(config_group.find("right_hand").asInt());
        } else {
            runHandConfig[0] = false;
        }
        //cout << "run right hand config: " << runHandConfig[0] << endl;

        if(config_group.check("left_hand")) {
            runHandConfig[1] = (bool)(config_group.find("left_hand").asInt());;
        } else {
            runHandConfig[1] = false;
        }
        //cout << "run left hand config: " << runHandConfig[1] << endl;

        if(config_group.check("head")) {
            runHeadConfig = (bool)(config_group.find("head").asInt());
        } else {
            runHeadConfig = false;
        }
        //cout << "run head config: " << runHeadConfig << endl;

        if(config_group.check("torso")) {
            runTorsoConfig = (bool)(config_group.find("torso").asInt());;
        } else {
            runTorsoConfig = false;
        }
        //cout << "run torso config: " << runTorsoConfig << endl;

        if(config_group.check("eyes")) {
            runEyesConfig = (bool)(config_group.find("eyes").asInt());;
        } else {
            runEyesConfig = false;
        }
        //cout << "run eyes config: " << runEyesConfig << endl;


        // parse cartesian position resources
        Bottle &cartpos_group=rf.findGroup("cartesianposition_resources");
        
        if(cartpos_group.check("right_arm")) {
            runArmPosition[0] = (bool)(cartpos_group.find("right_arm").asInt());
        } else {
            runArmPosition[0] = false;
        }
        //cout << "run right arm position: " << runArmPosition[0] << endl;

        if(cartpos_group.check("left_arm")) {
            runArmPosition[1] = (bool)(cartpos_group.find("left_arm").asInt());;
        } else {
            runArmPosition[1] = false;
        }
        //cout << "run left arm position: " << runArmPosition[1] << endl;

        if(cartpos_group.check("right_full_arm")) {
            runFullArmPosition[0] = (bool)(cartpos_group.find("right_full_arm").asInt());
        } else {
            runFullArmPosition[0] = false;
        }
        //cout << "run right full arm position: " << runFullArmPosition[0] << endl;

        if(cartpos_group.check("left_full_arm")) {
            runFullArmPosition[1] = (bool)(cartpos_group.find("left_full_arm").asInt());;
        } else {
            runFullArmPosition[1] = false;
        }
        //cout << "run left full arm position: " << runFullArmPosition[1] << endl;

        if(cartpos_group.check("right_leg")) {
            runLegPosition[0] = (bool)(cartpos_group.find("right_leg").asInt());
        } else {
            runLegPosition[0] = false;
        }
        //cout << "run right leg position: " << runLegPosition[0] << endl;

        if(cartpos_group.check("left_leg_arm")) {
            runLegPosition[1] = (bool)(cartpos_group.find("left_leg").asInt());;
        } else {
            runLegPosition[1] = false;
        }
        //cout << "run left leg position: " << runLegPosition[1] << endl;

        // parse heading resources
        Bottle &heading_group=rf.findGroup("heading_resources");
        
        if(heading_group.check("right_eye")) {
            runHeadings[0] = (bool)(heading_group.find("right_eye").asInt());
        } else {
            runHeadings[0] = false;
        }
        //cout << "run right heading: " << runHeadings[0] << endl;

        if(heading_group.check("left_eye")) {
            runHeadings[1] = (bool)(heading_group.find("left_eye").asInt());
        } else {
            runHeadings[1] = false;
        }
        //cout << "run left heading: " << runHeadings[1] << endl;

        // now that we have the config vals, set the params used by resources
        setParameters();

        // run the resources
        startResources();

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
        cout << "Closing iCub CB Resource Starter Module..." << endl; 
        if (resourcesRunning) {
            stopResources();
        }
        resourcesRunning = false;
        return true;
    }
    
    double getPeriod() {
        return 1;
    }
    
    bool updateModule() {        
        return true;
    }


    void setParameters() {

        if(simulationMode) {
            robot_prefix = "/icubSim";
        } else {
            robot_prefix = "/icub";
        }

        string iCubDir(getenv("ICUB_ROOT"));
        string configFilePath = iCubDir+"/app/controlBasis/conf/";

        cout << "config file path: " << configFilePath.c_str() << endl;

        fullArmConfigFile[0] = configFilePath+"full_right_arm.dh";
        fullArmName[0] = "right";
        fullArmConfigFile[1] = configFilePath+"full_left_arm.dh";
        fullArmName[1] = "left";
        
        armConfigFile[0] = configFilePath+"right_arm.dh";
        armName[0] = robot_prefix + "/right_arm";
        armVelPort[0] = robot_prefix + "/vc/right_arm";
        armConfigFile[1] = configFilePath+"left_arm.dh";
        armName[1] = robot_prefix + "/left_arm";
        armVelPort[1] = robot_prefix + "/vc/left_arm";
        
        legConfigFile[0] = configFilePath+"right_leg.dh";
        legName[0] = robot_prefix + "/right_leg";
        legVelPort[0] = robot_prefix + "/vc/right_leg";
        legConfigFile[1] = configFilePath+"left_leg.dh";
        legName[1] = robot_prefix + "/left_leg";
        legVelPort[1] = robot_prefix + "/vc/left_leg";

        handConfigFile[0] = configFilePath+"right_hand.dh";
        handName[0] = robot_prefix + "/right_hand";
        handVelPort[0] = robot_prefix + "/vc/right_hand";
        handConfigFile[1] = configFilePath+"left_hand.dh";
        handName[1] = robot_prefix + "/left_hand";
        handVelPort[1] = robot_prefix + "/vc/left_hand";
        
        headConfigFile = configFilePath+"head.dh";
        headName = robot_prefix + "/head";
        headVelPort = robot_prefix + "/vc/head";

        eyesConfigFile = configFilePath+"eyes.dh";
        eyesName = robot_prefix + "/eyes";
        eyesVelPort = robot_prefix + "/vc/head";
        
        torsoName = robot_prefix + "/torso";
        torsoVelPort = robot_prefix + "/vc/torso";

        headingName[0] = robot_prefix + "/right_eye";
        headingName[1] = robot_prefix + "/left_eye";
    
        armNumJoints = 7;
        armNumLinks = 12;
        
        legNumJoints = 6;
        legNumLinks = 8;
        
        headNumJoints = 3;
        headNumLinks = 8;
        
        handNumJoints = 9;
        handNumLinks = 9;
        
        torsoNumJoints = 3;
        torsoNumLinks = 3;

        eyesNumJoints = 3;
        eyesNumLinks = 3;

    }
    

    void startResources() {

        stopResources();

        cout << "starting resources..."<<endl;

        // start up configuartion variables
        for(int i=0; i<2; i++) {

            if(runFullArmConfig[i]) {
                cout << "starting full arm config for arm: " << i << endl;
                iCubFullArm[i] = new iCubFullArmConfigurationVariables(fullArmName[i], simulationMode, fullArmConfigFile[i]);
                iCubFullArm[i]->startResource();
                Time::delay(.75);
            }

            if(runArmConfig[i]) {
                cout << "starting arm config for arm: " << i << ", armName=" << armName[i] << endl;
                iCubArm[i] = new YARPConfigurationVariables(armName[i], armName[i], armNumJoints, armNumLinks);
                cout << "config file: " << armConfigFile[i] << endl;
                iCubArm[i]->loadConfig(armConfigFile[i]);
                iCubArm[i]->setUpdateDelay(0.05);
                iCubArm[i]->startResource();                        
                if(velocityControlMode) {
                    iCubArm[i]->setVelocityControlMode(true, armVelPort[i]);
                } else {
                    iCubArm[i]->setVelocityControlMode(false, "");
                }
                Time::delay(.75);
            }
            
            if(runLegConfig[i]) {
                cout << "starting leg config for leg: " << i << endl;
                iCubLeg[i] = new YARPConfigurationVariables(legName[i], legName[i], legNumJoints, legNumLinks);
                iCubLeg[i]->loadConfig(legConfigFile[i]);
                iCubLeg[i]->startResource();
                if(velocityControlMode) {
                    iCubLeg[i]->setVelocityControlMode(true, legVelPort[i]);
                } else {
                    iCubLeg[i]->setVelocityControlMode(false, "");
                }
                Time::delay(.75);
            }

            if(runHandConfig[i]) {
                cout << "starting hand config for hand: " << i << endl;
                iCubHand[i] = new YARPConfigurationVariables(handName[i], armName[i], handNumJoints, handNumLinks);
                iCubHand[i]->loadConfig(handConfigFile[i]);
                iCubHand[i]->startResource();
                if(velocityControlMode) {
                    iCubHand[i]->setVelocityControlMode(true, handVelPort[i]);
                } else {
                    iCubHand[i]->setVelocityControlMode(false, "");
                }
                Time::delay(.75);
            }


            // start up end effectors
            if(runArmPosition[i]) {                
                cout << "starting arm position for arm: " << i << endl;
                if(runArmConfig[i]) {
                    armEndEffector[i] = new EndEffectorCartesianPosition(armName[i]);
                    armEndEffector[i]->startResource();
                    armEndEffector[i]->setUpdateDelay(0.05);
                } else {
                    cout<<"Can't start CartesianPosition resource for arm["<<i<<"] because configuration resource is not running!!"<<endl;
                }
            }

            if(runLegPosition[i]) {    
                cout << "starting leg position for leg: " << i << endl;            
                if(runLegConfig[i]) {
                    legEndEffector[i] = new EndEffectorCartesianPosition(legName[i]);
                    legEndEffector[i]->startResource();
                    //legEndEffector[i]->setUpdateDelay(0.05);
                } else {
                    cout<<"Can't start CartesianPosition resource for leg["<<i<<"] because configuration resource is not running!!"<<endl;
                }                
            }
                
            if(runFullArmPosition[i]) {
                cout << "starting full arm position for arm: " << i << endl;
                if(runFullArmConfig[i]) {
                    string ee_name = robot_prefix + "/full_" + fullArmName[i] + "_arm";
                    fullArmEndEffector[i] = new EndEffectorCartesianPosition(ee_name);
                    fullArmEndEffector[i]->startResource();
                } else {
                    cout<<"Can't start CartesianPosition resource for fullArm["<<i<<"] because configuration resource is not running!!"<<endl;
                }     
            }           
    
            if(runHeadings[i]) {
                cout << "starting heading for eye: " << i << endl;
                salientHeadings[i] = new YARPAttentionMechanismHeading(headingName[i]);
                salientHeadings[i]->startResource();
            }           
        }

        if(runHeadConfig) {
            cout << "starting head config" << endl;
            iCubHead = new YARPConfigurationVariables(headName, "", headNumJoints, headNumLinks);
            iCubHead->loadConfig(headConfigFile);
            iCubHead->startResource();        
            if(velocityControlMode) {
                iCubHead->setVelocityControlMode(true, headVelPort);
            } else {
                iCubHead->setVelocityControlMode(false, "");
            }
        }

        if(runTorsoConfig) {
            cout << "starting torso config" << endl;
            iCubTorso = new YARPConfigurationVariables(torsoName, "", torsoNumJoints, torsoNumLinks);
            iCubTorso->startResource();
            if(velocityControlMode) {
                iCubTorso->setVelocityControlMode(true, torsoVelPort);
            } else {
                iCubTorso->setVelocityControlMode(false, "");
            }
        }

        if(runEyesConfig) {
            cout << "starting eyes config" << endl;
            iCubEyes = new YARPConfigurationVariables(eyesName, headName, eyesNumJoints, eyesNumLinks);
            iCubEyes->loadConfig(eyesConfigFile);
            iCubEyes->startResource();
            if(velocityControlMode) {
                iCubEyes->setVelocityControlMode(true, eyesVelPort);
            } else {
                iCubEyes->setVelocityControlMode(false, "");
            }
        }

        resourcesRunning = true;

    }    


    void stopResources() {
        
        if(resourcesRunning) {
            for(int i=0; i<2; i++) {
                if(runFullArmConfig[i]) iCubFullArm[i]->stopResource();
                if(runArmConfig[i]) iCubArm[i]->stopResource();
                if(runLegConfig[i]) iCubLeg[i]->stopResource();
                if(runHandConfig[i]) iCubHand[i]->stopResource();
                if(runArmPosition[i]) armEndEffector[i]->stopResource(); 
                if(runLegPosition[i]) legEndEffector[i]->stopResource(); 
                if(runFullArmPosition[i]) fullArmEndEffector[i]->stopResource(); 
                if(runHeadings[i]) salientHeadings[i]->stopResource();
            }     
            if(runHeadConfig) iCubHead->stopResource();
            if(runTorsoConfig) iCubTorso->stopResource();
            if(runEyesConfig) iCubEyes->stopResource();
        }
        resourcesRunning=false;
    }

};

int main(int argc, char *argv[])
{
    Network yarp;
    iCubControlBasisResourceStarter mod;
    
    ResourceFinder rf;
    rf.setDefault("simulation","true");
    rf.setDefault("velocityControlMode","false");

    rf.setDefaultContext("controlBasis/conf");
    rf.setDefaultConfigFile("iCubSimControlBasisResources.ini");

    rf.configure("ICUB_ROOT", argc, argv);
    rf.setVerbose(true);

    cout<<"Starting iCub CB Resources..."<<endl;
    return mod.runModule(rf);

}
