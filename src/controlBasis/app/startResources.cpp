// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include <YARPConfigurationVariables.h>
#include <iCubFullArmConfigurationVariables.h>
#include <EndEffectorCartesianPosition.h>
//#include <CartesianPositionHarmonicReference.h>
#include <ManipulatorPositionJacobian.h>
#include <Controller.h>
#include <RunnableControlLaw.h>

#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>
#include <iostream>
#include <ace/ACE.h>

using namespace CB;
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

void startResources();
void stopResources();
void printMenu();
void doUI();
static void close_handler (int);

iCubFullArmConfigurationVariables *iCubFullArm[2];
YARPConfigurationVariables *iCubArm[2];
YARPConfigurationVariables *iCubLeg[2];
YARPConfigurationVariables *iCubHand[2];
YARPConfigurationVariables *iCubHead;
YARPConfigurationVariables *iCubTorso;
EndEffectorCartesianPosition *endEffector[2]; 
EndEffectorCartesianPosition *legEndEffector[2]; 
EndEffectorCartesianPosition *fullArmEndEffector[2]; 
//CartesianPositionHarmonicReference *harmonicPositionRef;

string fullArmName[2];
string fullArmConfigFile[2];
string armName[2];
string armConfigFile[2];
string armVelPort[2];
string legName[2];
string legConfigFile[2];
string handName[2];
string handConfigFile[2];
string headName;
string headConfigFile;
string torsoName;
string torsoConfigFile;
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
Vector posRef;
bool resourcesStarted;
bool simulationMode;
string robot_prefix;

int main(int argc, char *argv[]) {

    Network yarp;
    Network::init();
    Time::turboBoost();

    ACE_OS::signal(SIGINT, (ACE_SignalHandler) close_handler);
    ACE_OS::signal(SIGTERM, (ACE_SignalHandler) close_handler);

    simulationMode = true;
    if(argc > 1) {
        if(!strcmp(argv[1],"-r")) {
            simulationMode = false;
        }
    }

    printf("simulation mode: %d\n", (int)simulationMode);

    fullArmConfigFile[0] = "../config/full_right_arm.dh";
    fullArmName[0] = "right";
    fullArmConfigFile[1] = "../config/full_left_arm.dh";
    fullArmName[1] = "left";

    if(simulationMode) {
        robot_prefix = "/icubSim";
    } else {
        robot_prefix = "/icub";
    }
    armConfigFile[0] = "../config/right_arm.dh";
    armName[0] = robot_prefix + "/right_arm";
    armVelPort[0] = robot_prefix + "/vc/right_arm";
    armConfigFile[1] = "../config/left_arm.dh";
    armName[1] = robot_prefix + "/left_arm";
    armVelPort[1] = robot_prefix + "/vc/left_arm";

    legConfigFile[0] = "../config/right_leg.dh";
    legName[0] = robot_prefix + "/right_leg";
    legConfigFile[1] = "../config/left_leg.dh";
    legName[1] = robot_prefix + "/left_leg";

    handConfigFile[0] = "../config/right_hand.dh";
    handName[0] = robot_prefix + "/right_hand";
    handConfigFile[1] = "../config/left_hand.dh";
    handName[1] = robot_prefix + "/left_hand";

    headConfigFile = "../config/head.dh";
    headName = robot_prefix + "/head";

    torsoName = robot_prefix + "/torso";

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

    posRef.resize(3); 
    posRef.zero();

    resourcesStarted = false;

    doUI();

    printf("Success!!\n");
    return 1;    
}

void stopResources() {

    cout << "start resources shutting down resources..." << endl;
    if(resourcesStarted) {
        for(int i=0; i<1; i++) {
            //iCubFullArm[i]->stopResource();
            iCubArm[i]->stopResource();
            //iCubLeg[i]->stopResource();
            //iCubHand[i]->stopResource();
            endEffector[i]->stopResource(); 
            //legEndEffector[i]->stopResource(); 
            //fullArmEndEffector[i]->stopResource(); 
     
            delete iCubFullArm[i];
            delete iCubArm[i];
            delete iCubLeg[i];
            delete iCubHand[i];
            delete endEffector[i];
            delete legEndEffector[i];
            delete fullArmEndEffector[i];
        }
        //iCubHead->stopResource();
        //iCubTorso->stopResource();
        //harmonicPositionRef->stopResource();
        delete iCubHead;
        delete iCubTorso;
        //        delete harmonicPositionRef;
    }
}


void startResources() {

    stopResources();

    // start up configuartion variables
    for(int i=0; i<1; i++) {
        /*
        iCubFullArm[i] = new iCubFullArmConfigurationVariables(fullArmName[i], simulationMode, fullArmConfigFile[i]);
        iCubFullArm[i]->startResource();
        Time::delay(.75);
        */
        iCubArm[i] = new YARPConfigurationVariables(armName[i], armName[i], armNumJoints, armNumLinks);
        iCubArm[i]->loadConfig(armConfigFile[i]);
        iCubArm[i]->setUpdateDelay(0.05);
        iCubArm[i]->startResource();

        if(!simulationMode) {
            iCubArm[i]->setVelocityControlMode(true, armVelPort[i]);
        } else {
            iCubArm[i]->setVelocityControlMode(false, "");
        }
        Time::delay(.75);
        
        /*
        iCubLeg[i] = new YARPConfigurationVariables(legName[i], legName[i], legNumJoints, legNumLinks);
        iCubLeg[i]->loadConfig(legConfigFile[i]);
        iCubLeg[i]->startResource();
        Time::delay(.75);

        iCubHand[i] = new YARPConfigurationVariables(handName[i], armName[i], handNumJoints, handNumLinks);
        iCubHand[i]->loadConfig(handConfigFile[i]);
        iCubHand[i]->startResource();
        Time::delay(.75);
        */
        // start up end effectors

        endEffector[i] = new EndEffectorCartesianPosition(armName[i]);
        endEffector[i]->startResource();
        endEffector[i]->setUpdateDelay(0.05);
        /*
        if(!endEffector[i]->connectToConfiguration()) {
            printf("Couldn't connect end effector[%d]...\n",i);
        } else {
            endEffector[i]->startResource();
        }
        */
        /*
        legEndEffector[i] = new EndEffectorCartesianPosition(legName[i]);
        if(!legEndEffector[i]->connectToConfiguration()) {
            printf("Couldn't connect leg end effector[%d]...\n",i);
        } else {
            legEndEffector[i]->startResource();
        }
                
        string ee_name = robot_prefix + "/full_" + fullArmName[i] + "_arm";
        fullArmEndEffector[i] = new EndEffectorCartesianPosition(ee_name);
        if(!fullArmEndEffector[i]->connectToConfiguration()) {
            printf("Couldn't connect full arm end effector[%d]...\n",i);
        } else {
            fullArmEndEffector[i]->startResource();
        }
        */
    }
    
    /*
    iCubHead = new YARPConfigurationVariables(headName, "", headNumJoints, headNumLinks);
    iCubHead->loadConfig(headConfigFile);
    iCubHead->startResource();

    iCubTorso = new YARPConfigurationVariables(torsoName, "", torsoNumJoints, torsoNumLinks);
    //    iCubTorso->loadConfig(torsoConfigFile);
    iCubTorso->startResource();

    harmonicPositionRef = new CartesianPositionHarmonicReference("/icub/harmonic_goal");
    harmonicPositionRef->setGoal(posRef);
    harmonicPositionRef->setUpdateDelay(0.2);
    harmonicPositionRef->startResource();
    */
    resourcesStarted = true;
}


void doUI() {

    std::string command;
    printMenu();

    float x,y,z,j0,j1,j2,j3,j4,j5,j6,j7,j8;
    
    ACE_OS::printf("main> ");
    cin >> command;
    printf("command: %s\n", command.c_str());

    while(command != "q") {
        if(command == "start") {
            ACE_OS::printf("starting resources for devices\n");
            startResources();
        } 
        else if(command == "xpos") {            
            ACE_OS::printf("enter pos [x y z]: ");
            cin >> x >> y >> z;
            ACE_OS::printf("Setting Cartesian ref to (%.3f %.3f %.3f)\n", x,y,z);
            posRef[0] = (double)x;
            posRef[1] = (double)y;
            posRef[2] = (double)z;
            //            harmonicPositionRef->setGoal(posRef);            
        } 
        else if(command=="quit") {
            break;
        }
        else {            
            printMenu();
        }

        Time::delay(0.5);
        ACE_OS::printf("main> ");        
        cin >> command;
    }

    stopResources();
}

void printMenu() {
    ACE_OS::printf("\nCONTROL BASIS MENU\n---------------------\n\n");
    ACE_OS::printf("start \t --> \t starts the default YARP devices\n");
    ACE_OS::printf("xpos \t --> \t sets the 3D xyz positon for Cartesian controller\n");
    ACE_OS::printf("");
    ACE_OS::printf("? \t --> \t print this menu\n");
    ACE_OS::printf("[q]uit \t --> \t quit\n");
}


static void close_handler (int) {
    stopResources();
    ACE_OS::printf("Shutting down...\n\n");
    exit(0);
}
