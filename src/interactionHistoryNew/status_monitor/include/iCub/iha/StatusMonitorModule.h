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

#ifndef __STATUS_MONITOR_MODULE_H__
#define __STATUS_MONITOR_MODULE_H__

#include <stdio.h>
#include <string>
#include <iostream>
#include <map>

#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <iCub/iha/debug.h>

#define MAX_ITEM_LEN 20

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

namespace iCub {
    namespace contrib {
        class StatusMonitorModule;
    }
}
namespace iCub {
    namespace iha {
        class StatusOutputThread;
        class SubsystemStatus;
    }
}

using namespace iCub::contrib;
using namespace iCub::iha;

class iCub::contrib::StatusMonitorModule;
class iCub::iha::SubsystemStatus;
class iCub::iha::StatusOutputThread;

/**
 * \brief Subsystem Status holds a list of statuses for a subsystem consisting of system->value pairs in a map (see \ref icub_iha_StatusMonitor )
 */
class iCub::iha::SubsystemStatus {
public:
    string subsystem;
    map<string, int> int_values;

    SubsystemStatus() {}
    ~SubsystemStatus() {}

    void setName(const char* name) {
        subsystem.clear();
        subsystem.append(name);
    }

    void debugPrint(int dbgl) {
        for (map<string, int>::iterator it=int_values.begin();it!=int_values.end();it++) {
            IhaDebug::pmesg(dbgl,"Item : %s = %d\n", it->first.c_str(), it->second);
        }
    }
    void debugPrintValue(int dbgl) {
        for (map<string, int>::iterator it=int_values.begin();it!=int_values.end();it++) {
            IhaDebug::pmesg(dbgl,"%10d\n", it->second);
        }
    }
};

/**
 * \brief Rate controlled thread for writing status (see \ref icub_iha_StatusMonitor )
 *
 */
class iCub::iha::StatusOutputThread : public yarp::os::RateThread 
{
public:
    StatusOutputThread()
      : yarp::os::RateThread(10)
    {
        progstart = Time::now();
        IhaDebug::pmesg(DBGL_DEBUG1, "StatusOutputThread default constructor.\n");
    }
    StatusOutputThread(int datarate, map<string, SubsystemStatus*>* _systemStatus, yarp::os::Port &_outPort) 
      : yarp::os::RateThread(datarate), systemStatus(_systemStatus), outPort(&_outPort)
    {
        progstart = Time::now();
        IhaDebug::pmesg(DBGL_DEBUG1, "StatusOutputThread created.\n");
    }
    ~StatusOutputThread() { }

    void init(int datarate, map<string, SubsystemStatus*>* _systemStatus, yarp::os::Port &_outPort) {
        IhaDebug::pmesg(DBGL_DEBUG1, "StatusOutputThread initializing\n");
        systemStatus=_systemStatus;
        outPort=&_outPort;
        setRate(datarate);
    }

    virtual void run() ;
    virtual void onStop() {}
    virtual void beforeStart() {}
    virtual void afterStart() {}
    virtual bool threadInit() { return true; }
    virtual void threadRelease() {}
private:
    double progstart;
    map<string, SubsystemStatus*>* systemStatus;
    yarp::os::Port* outPort;
};

/**
 *
 * Status Monitor Module class
 *
 * \brief See \ref icub_iha_StatusMonitor
 */
class iCub::contrib::StatusMonitorModule : public yarp::os::Module {

private:

    // ports
    yarp::os::BufferedPort<yarp::os::Bottle> quitPort;
    yarp::os::Port monitorInPort;
    yarp::os::Port monitorOutPort;

    // parameters read from ini file/command line
    int status_output_rate;

    map<string, SubsystemStatus*> systemStatus;
    StatusOutputThread* statusOutputThread;

public:

    StatusMonitorModule();
    virtual ~StatusMonitorModule();
    
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
    virtual bool respond(const Bottle &command,Bottle &reply);

    void addMissingItems();

    void debugPrintSystemStatus(int dbgl) {
        IhaDebug::pmesg(dbgl,"--------------------------------------------\n");
        for (map<string,SubsystemStatus*>::iterator ssit = systemStatus.begin(); ssit != systemStatus.end() ; ssit++) {
            IhaDebug::pmesg(dbgl,"SubSystem : %s\n",ssit->first.c_str());
            ssit->second->debugPrint(dbgl);
        }
        IhaDebug::pmesg(dbgl,"--------------------------------------------\n");
    }
};


#endif
