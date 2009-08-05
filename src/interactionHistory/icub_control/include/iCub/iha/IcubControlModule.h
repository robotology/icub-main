// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// vim:expandtab:tabstop=4:shiftwidth=4:softtabstop=4:

/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Assif Mirza
 * email:   assif.mirza@robotcub.org
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

#ifndef __ICUBCONTROL_MODULE_H__
#define __ICUBCONTROL_MODULE_H__

#include <stdio.h>
#include <string>
#include <iostream>

#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <iCub/iha/debug.h>
#include <iCub/iha/Actions.h>
#include <iCub/iha/mem_util.h>

#define JOINTREF_UNDEF 99999

#define numParts 6

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

namespace iCub {
    namespace contrib {
        class IcubControlModule;
    }
}
namespace iCub {
    namespace iha {
        class EncodersOutputThread;
    }
}

using namespace iCub::contrib;
using namespace iCub::iha;

class iCub::contrib::IcubControlModule;

/**
 * \brief Rate controlled tread for writing encoder values and current action number
 *
 */
class iCub::iha::EncodersOutputThread : public yarp::os::RateThread 
{
public:
    EncodersOutputThread()
      : yarp::os::RateThread(10)
    {
        progstart = Time::now();
        IhaDebug::pmesg(DBGL_DEBUG1, "EncodersOutputThread default constructor.\n");
    }
    EncodersOutputThread(int datarate, IcubControlModule* _controller, yarp::os::Port &_outPort) 
      : yarp::os::RateThread(datarate), controller(_controller), outPort(&_outPort)
    {
        progstart = Time::now();
        IhaDebug::pmesg(DBGL_DEBUG1, "EncodersOutputThread created.\n");
    }
    ~EncodersOutputThread() { }

    void init(int datarate, IcubControlModule* _controller, yarp::os::Port &_outPort) {
        IhaDebug::pmesg(DBGL_DEBUG1, "EncodersOutputThread initializing\n");
        controller=_controller;
        outPort=&_outPort;
        setRate(datarate);
    }

    virtual void run();
    virtual void onStop() {}
    virtual void beforeStart() {}
    virtual void afterStart() {}
    virtual bool threadInit() { return true; }
    virtual void threadRelease() {}
private:
    double progstart;
    iCub::contrib::IcubControlModule* controller;
    yarp::os::Port* outPort;
};

/**
 *
 * Icub Control Module class
 *
 * \brief See \ref icub_iha_IcubControl
 */
class iCub::contrib::IcubControlModule : public yarp::os::Module {

private:

    // ports
    yarp::os::BufferedPort<yarp::os::Bottle> quitPort;
    yarp::os::Port commandPort;
    yarp::os::Port encodersPort;
    yarp::os::Port expressionRawInPort;

    // parameters read from ini file/command line
    
    //-------------------------------------------------------------------
    // Robot setup
    bool active[numParts];
    int numJointsTotal;
    int numJoints[numParts];

    IPositionControl *pos[numParts];
    IEncoders *enc[numParts];

    double *refs[numParts];
    double *speeds[numParts];
    double *encs[numParts];

    bool *motorMoving[numParts];
    bool *motorFault[numParts];
    bool partMoving[numParts];

    int motor_motion_timeout;
    int sensor_output_rate;
    double speed_multiplier;
    bool always_set_speed;

    int currentAction;
    //-------------------------------------------------------------------

	Bottle response;

    Actions iCubActions;
    int NUM_ACTIONS;
    vector<std::string> action_commands;

    bool doPositionMove(IPositionControl *pos, int joint, int ref, int speed);
    bool doPositionMove(IPositionControl *pos, int joint, int ref);
    bool doPositionMove(int joint, int ref, int speed);
    bool doSetSpeed(IPositionControl *pos, int joint, int speed);
    bool doSetSpeed(int joint, int speed);
    bool doMultiPositionMove(IPositionControl *posc, const double *refs);
    bool waitForCompletion();
    bool multiMove();
    bool runSequenceParallel(string seq);
    bool sendAction(int act);
    bool sendExpression(int act);

    EncodersOutputThread* encodersOutputThread;

    /**
     * Get multiplied speed
     */
    int getMultipledSpeed(int speed) { return (int) ((double)speed * speed_multiplier); }

public:

    IcubControlModule();
    virtual ~IcubControlModule();
    
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
    virtual bool respond(const Bottle &command,Bottle &reply);

    int getCurrentAction() { return currentAction; }
    double** getEncoderReadings();
    int getNumJoints(int part) { if (active[part]) return numJoints[part]; else return 0; }

    bool isActive(int part) { return active[part]; }

    void debugPrintArrays();
};


#endif
