// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// Copyright (C) 2008 RobotCub Consortium
// Author: Lorenzo Natale
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef __VELCONTROLTHREAD__
#define __VELCONTROLTHREAD__

#include <mutex>
#include <string>

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>

//class yarp::dev::PolyDriver;

class velControlThread: public yarp::os::PeriodicThread
{
private:
    char robotName[255];
    yarp::dev::IVelocityControl *ivel;
    yarp::dev::IPositionControl *ipos;
    yarp::dev::IEncoders        *ienc;
    yarp::dev::IControlMode     *imod;
    yarp::dev::IPidControl      *ipid;
    int nJoints;
    yarp::dev::PolyDriver *driver;

    yarp::sig::Vector encoders;
    yarp::sig::Vector encoders_speed;
    yarp::sig::Vector Kp;
    yarp::sig::Vector Kd; //derivative term

    yarp::sig::Vector targets;
    yarp::sig::Vector ffVelocities;
    yarp::sig::Vector command;

    yarp::sig::Vector error;
    yarp::sig::Vector error_d;

    yarp::sig::Vector maxVel; //added ludo

    bool suspended;

    int nb_void_loops;

    std::mutex _mutex;

    int control_rate; //in ms

    yarp::os::BufferedPort<yarp::os::Bottle> command_port; //deprecated
    yarp::os::BufferedPort<yarp::os::Bottle> command_port2; //new

    double time_watch;
    double time_loop;
    int count;

    FILE *currentSpeedFile;
    FILE *targetSpeedFile;

public:
    velControlThread(int rate);
    ~velControlThread();

    bool init(yarp::dev::PolyDriver *d, std::string partName,
              std::string robotName);

    void halt();
    void go();
    void setRef(int i, double pos);

    void setVel(int i, double vel); //added ludovic to set max velocity
    void setGain(int i, double gain); //to set the Kp gains

    void run();
    bool threadInit();
    void threadRelease();

    void limitSpeed(yarp::sig::Vector &command);
};

#endif

