// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include <YARPConfigurationVariablesModule.h>
//#include <EndEffectorCartesianPosition.h>
//#include <ConfigurationSquaredError.h>
//#include <iCubConfigurationReference.h>
//#include <CartesianPositionReference.h>
//#include <ManipulatorPositionJacobian.h>
//#include <Controller.h>

#include <yarp/os/Network.h>
#include <stdio.h>

using namespace CB;

//void sendControlSignal(const Vector &Vdes);
void doUI();
void printMenu();
//void runController(int t);
void startResources();

string robotName;
BufferedPort<Bottle> configPort;
BufferedPort<Bottle> lockPort;

YARPConfigurationVariablesModule *yarpRobot;
//iCubConfigurationReference *iCubConfigRef;
//CartesianPositionReference *iCubPositionRef;
//EndEffectorCartesianPosition *endEffector; 
//Controller *controller;   

string configFile;
int numJoints;
int numLinks;
Vector armRef;
Vector posRef;
bool resourcesStarted;

int main(int argc, char *argv[]) {

    Network yarp;
    Network::init();
    Time::turboBoost();

    configFile = "right_arm.dh";
    numJoints = 7;
    numLinks = 0;
    armRef.resize(7);
    armRef.zero();
    posRef.resize(3);
    posRef.zero();

    resourcesStarted = false;

    if(argc > 1) {
        for(int i=1; i<argc; i++) {
            if(!strcmp(argv[i],"--robot"))
                robotName = argv[i+1];
            if(!strcmp(argv[i],"--dof")) 
                sscanf(argv[i+1], "%d", &numJoints);
            if(!strcmp(argv[i],"--links")) 
                sscanf(argv[i+1], "%d", &numLinks);
            if(!strcmp(argv[i],"--config")) 
                configFile = argv[i+1];
            if(!strcmp(argv[i],"--cpos")) {
                for(int k=0; k<numJoints; k++) {
                    armRef[k] = atof(argv[i+k+1])*TORAD;
                }
            }        
            if(!strcmp(argv[i],"--xpos")) {
                for(int k=0; k<3; k++) {
                    posRef[k] = atof(argv[i+k+1]);                        
                }
            }
        }
    } 

    doUI();

    printf("Success!!\n");
    return 1;    
}

void startResources() {

    if(resourcesStarted) {
        yarpRobot->stopResource();
        //iCubConfigRef->stopResource();
        //iCubPositionRef->stopResource();
        //endEffector->stopResource(); 
        //resourcesStarted = false;       
    }

    // start up configuartion variables
    yarpRobot = new YARPConfigurationVariablesModule(robotName, numJoints, numLinks);
    yarpRobot->loadConfig(configFile);
    printf("moduleTest -- starting robot module...\n");
    yarpRobot->startResource();
    printf("moduleTest -- started module.\n");
    Time::delay(2);

    /*
    // start up end effector
    endEffector = new EndEffectorCartesianPosition(robotName);
    if(!endEffector->connectToConfiguration()) {
        printf("Couldn't connect end effector...\n");
    } else {
        endEffector->startResource();
    }

    // start up the iCub reference
    iCubConfigRef = new iCubConfigurationReference(robotName, 7);  
    iCubConfigRef->setVals(armRef);
    iCubConfigRef->setUpdateDelay(0.25);
    iCubConfigRef->startResource();
    Time::delay(0.25);
    
    iCubPositionRef = new CartesianPositionReference(robotName);
    iCubPositionRef->setVals(posRef);
    iCubPositionRef->setUpdateDelay(0.2);
    iCubPositionRef->startResource();
    Time::delay(0.25); 
    */
    resourcesStarted = true;
}
/*
void sendControlSignal(const Vector &Vdes) {

    Bottle& b = configPort.prepare();
    b.clear();
    for(int i=0; i<Vdes.size(); i++) {
        b.addDouble(Vdes[i]);
    }
    printf("configTest writing data...\n");
    configPort.write();

}

void runController(int t) {

    printf("controlTest -- starting controller\n");
    controller->startAction();
    printf("started\n");
    Time::delay(1);
    
    string lockOutputPortName ="/cb/test/lock:o";    
    string lockPortName = "/cb/configuration" + robotName + "/lock:i";
        
    ACE_OS::printf("configTest opening lock input port: %s\n", lockPortName.c_str());
    lockPort.open(lockOutputPortName.c_str());
    Network::connect(lockOutputPortName.c_str(),lockPortName.c_str(), "tcp");
    
    // turn off log for setting
    Bottle &unlock_bottle = lockPort.prepare();
    unlock_bottle.clear();
    unlock_bottle.addInt((int)false);
    printf("configTest opening lock...\n");
    lockPort.write();
    Time::delay(0.25);

    Vector Vout(7);    
    for(int i=0; i<t; i++) {
        ACE_OS::printf("\n\nCONTROLLER ITERATION[%d]", i);
        Vout = controller->getControlOutput();
        yarpRobot->setDesiredIncrement(Vout);
        Time::delay(0.1);
    }
        
    Bottle &lock_bottle = lockPort.prepare();
    lock_bottle.clear();
    lock_bottle.addInt((int)true);
    printf("configTest closing lock...\n");
    lockPort.write();
    //    Time::delay(0.1);
    lockPort.close();

    controller->stopAction();
    //printf("deleting controller...\n");
    //    delete controller;
    //printf("controller deleted\n");

    return;
}
*/
void doUI() {

    printMenu();

    char *command;
    char *question;

    float x,y,z,j0,j1,j2,j3,j4,j5,j6;
    
    command = (char *)malloc(128);
    question = (char *)malloc(128);

    ACE_OS::printf("> ");
    int r = scanf("%s", command);
    while(command[0] != 'q') {

        if(!strncmp(command,"set",3)) {
            ACE_OS::printf("enter device name: ");
            r = scanf("%s", question);
            robotName = question;
            ACE_OS::printf("setting device: \'%s\'\n", robotName.c_str());
            startResources();
        }
        else if(!strncmp(command,"start",5)) {
            ACE_OS::printf("starting resources for device: \'%s\'\n", robotName.c_str());
            startResources();
        /*} else if(!strncmp(command,"Cj",2)) {
            controller = new Controller("/cb/configuration" + robotName,
                                        "/cb/configuration" + robotName + "/ref",
                                        "/cb/configuration/squared_error_pf",
                                        "/cb/configuration" + robotName);
            runController(50);
            ACE_OS::printf("done running Cj\n");        
        } else if(!strncmp(command,"Cx",2)) {
            controller = new Controller("/cb/cartesianposition" + robotName,
                                        "/cb/cartesianposition" + robotName + "/ref", 
                                        "/cb/cartesianposition/squared_error_pf",
                                        "/cb/configuration" + robotName);
            runController(50);
            ACE_OS::printf("done running Cx\n");        
        } else if(!strncmp(command,"Crom",4)) {            
            controller = new Controller("/cb/configuration" + robotName,
                                        "/cb/configuration/cosfield_pf",
                                        "/cb/configuration" + robotName);
            controller->setGain(10);
            runController(50);
            ACE_OS::printf("done running Crom\n");        
        } else if(!strncmp(command,"Cm",2)) {            
            controller = new Controller("/cb/configuration" + robotName,
                                        "/cb/configuration/manipulability_pf",
                                        "/cb/configuration" + robotName);
            //controller->setGain(10);
            runController(100);
            ACE_OS::printf("done running Cm\n");        
        } else if(!strncmp(command,"xpos",4)) {            
            ACE_OS::printf("enter pos [x y z]: ");
            r = scanf("%f %f %f", &x, &y, &z);
            posRef[0] = (double)x;
            posRef[1] = (double)y;
            posRef[2] = (double)z;
            iCubPositionRef->setVals(posRef);
        } else if(!strncmp(command,"jpos",7)) {            
            ACE_OS::printf("enter pos [j0 j1 j2 j3 j4 j5 j6]: ");
            r = scanf("%f %f %f %f %f %f %f", &j0, &j1, &j2, &j3, &j4, &j5, &j6);
            armRef[0] = (double)j0;
            armRef[1] = (double)j1;
            armRef[2] = (double)j2;
            armRef[3] = (double)j3;
            armRef[4] = (double)j4;
            armRef[5] = (double)j5;
            armRef[6] = (double)j6;
            iCubConfigRef->setVals(armRef);
        */
        } else if(!strcmp(command,"?")) {            
            printMenu();
        }
        Time::delay(0.5);
        ACE_OS::printf("> ");        
        r = scanf("%s", command);
    }

    yarpRobot->stopResource();
//iCubConfigRef->stopResource();
//    iCubPositionRef->stopResource();
//  endEffector->stopResource();

    free(question);
    free(command);

}

void printMenu() {

    ACE_OS::printf("\nCONTROL BASIS MENU\n---------------------\n\n");
    ACE_OS::printf("set \t --> \t sets the YARP device for controllers\n");
    ACE_OS::printf("start \t --> \t starts the default YARP devices\n");
    //  ACE_OS::printf("xpos \t --> \t sets the 3D xyz positon for Cartesian controller\n");
    //ACE_OS::printf("jpos \t --> \t sets the 7D positon for arm controller\n");
    //ACE_OS::printf("Crom \t --> \t runs the range-of-motion controller\n");
    //ACE_OS::printf("Cm \t --> \t runs the manipulability controller\n");
    //ACE_OS::printf("Cx \t --> \t runs the Cartesian controller\n");
    //ACE_OS::printf("Cj \t --> \t runs the Joint controller\n");
    ACE_OS::printf("");
    ACE_OS::printf("? \t --> \t print this menu\n");
    ACE_OS::printf("q \t --> \t quit\n");

}

