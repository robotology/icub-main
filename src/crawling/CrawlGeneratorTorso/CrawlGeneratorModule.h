// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef CrawlGeneratorModule__H
#define CrawlGeneratorModule__H

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/RateThread.h>
#include <yarp/String.h>

#include <stdio.h>
#include <iostream>
#include "cpgs.h"
#include "CrawlInvKin.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

#define LIMIT_TOL 2.0 //conservative limit tolerance in degrees
#define MAX_FREQUENCY 1.5 //in Hz
#define MAX_TURN_ANGLE 0.52 //in radians 

#define LEFT 1
#define RIGHT 2

#define ARM 1
#define LEG 2

class generatorThread : public yarp::os::RateThread
{
 public:
    generatorThread(int period); //constructor
    ~generatorThread(); //destructor
  
    void run(); //the main loop
    bool threadInit();
    void threadRelease();

    bool init(Searchable &s);
    void getParameters();
    
 private:
 
    double period; //in second

    yarp::String partName;

    int nbDOFs;
    int nbLIMBs;
    int *jointMapping;
    double *y_cpgs;
    double *states;
    double *previous_states;
    double *encoders;
    double *joint_limit_up;
    double *joint_limit_down;
    double *initPos;
    double original_time;
    double theoretical_time;
    double lastBeat_time;
    double amplit;
    int previous_quadrant;
    int beat;
    
    int side, limb;

    cpgs *myCpg;
    PolyDriver *ddPart;
    IEncoders *PartEncoders;
    IKManager *myIK;

    BufferedPort<Bottle> vcControl_port, vcFastCommand_port;
    BufferedPort<Bottle> parameters_port, check_status_port;
    BufferedPort<Bottle> other_part_port[3], current_state_port;
    BufferedPort<Bottle> contact_port;
    
    
    bool other_part_connected[3];
    ConstString other_part_name[3];
    
    ConstString robot;
   
    bool current_action;

    FILE *target_file, *parameters_file, *encoder_file, *feedback_file;

    bool sendJointCommand();
    bool sendFastJointCommand();
    bool checkJointLimits();
    bool getEncoders();
    //bool sendEncoders();
    bool getOtherLimbStatus();
    void getContactInformation();
    void connectToOtherLimbs();
    void disconnectPorts();
    void sendStatusForManager();
    bool getQuadrant();
};

class CrawlGeneratorModule : public Module
{
 private:        
    generatorThread *theThread;
    yarp::String partName;
    
 public:
    virtual bool close();
    virtual bool open(yarp::os::Searchable &s);
    virtual double getPeriod();
    virtual bool updateModule();
};

#endif

