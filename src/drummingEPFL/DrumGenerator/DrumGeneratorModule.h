// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef DrumGeneratorModule__H
#define DrumGeneratorModule__H

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/RateThread.h>
#include <yarp/String.h>
#include <iostream>
#include <stdio.h>
#include "cpgs.h"
#include <list>

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

#define LIMIT_TOL 2.0 //conservative limit tolerance in degrees
#define MAX_FREQUENCY 1.5 //in Hz
#define VELOCITY_INDEX_OFFSET 1000

#define LEFT_THRESHOLD -1.0
#define RIGHT_THRESHOLD 1.5
#define MOV_AV 1


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
    void getHit();
 private:
    double period; //in second

    yarp::String partName;

    int nbDOFs;
    int *jointMapping;
    double *y_cpgs;
    double *states;
    double *dstates;
    double *previous_states;
    double *encoders;
    double *joint_limit_up;
    double *joint_limit_down;
    double *initPos;
    double original_time;
    double theoretical_time;
    double lastBeat_time;
    bool previous_quadrant[2]; //used to calculate the new beat
    int beat;
    int notes[3];
    cpg_manager *myManager;
    PolyDriver *ddPart;
    IEncoders *PartEncoders;
    
    list<double> moving_average;

    BufferedPort<Bottle> vcControl_port, vcFastCommand_port;
    BufferedPort<Bottle> parameters_port, check_motion_port;
    BufferedPort<Bottle> clock_port, sound_port, ForceSensor_port;

    bool external_clock;

    FILE *target_file, *parameters_file, *encoder_file, *feedback_file;

    bool sendJointCommand();
    bool sendFastJointCommand();
    void checkJointLimits();
    bool getEncoders();
    bool getExternalClock();
    bool getNewBeat();
};

class DrumGeneratorModule : public Module
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

