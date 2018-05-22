// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// Copyright (C) 2015 Istituto Italiano di Tecnologia - iCub Facility
// Author: Marco Randazzo  <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef __POSDIRCONTROLTHREAD__
#define __POSDIRCONTROLTHREAD__

#include <string>

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>

class positionDirectControlThread: public yarp::os::RateThread
{
private:
    char robotName[255];
    yarp::dev::IPositionDirect  *idir;
    yarp::dev::IControlLimits   *ilim;
    yarp::dev::IPositionControl *ipos;
    yarp::dev::IEncoders        *ienc;
    yarp::dev::IControlMode     *imod;
    
    unsigned int part_joints;
    unsigned int control_joints;
    int* control_joints_list;

    yarp::dev::PolyDriver *driver;

    yarp::sig::Vector min_limits;
    yarp::sig::Vector max_limits;
    yarp::sig::Vector encoders;
    
    yarp::sig::Vector targets;
    yarp::sig::Vector prev_targets;

    yarp::sig::Vector error;
    yarp::os::Semaphore _mutex;

    bool suspended;

    int control_period;

    yarp::os::BufferedPort<yarp::os::Bottle> command_port;

public:
    positionDirectControlThread(int rate);
    ~positionDirectControlThread();

    bool init(yarp::dev::PolyDriver *d, std::string moduleName, std::string partName, std::string robotName, yarp::os::Bottle* jointsList);

    void halt();
    void go();

    void setVel(int i, double vel);
    void setGain(int i, double gain); 

    void run();
    bool threadInit();
    void threadRelease();

    void limitSpeed(yarp::sig::Vector &command);
};

#endif

