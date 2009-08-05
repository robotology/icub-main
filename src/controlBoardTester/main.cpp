// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_module
 *
 * \defgroup icub_controlBoardTester controlBoardTester
 *
 * A basic module for understanding if supplied data  
 * are effected by unusual errors. The main module application
 * will be checking if data supplied by a controlBaord (e.g.
 * positions) are effected by expected
 * levels of noise. The level of expectation is defined in terms 
 * of a max and min values, and standard deviation.
 *
 * \section intro_sec Description
 * 
 * Current implementation (for each joint) checks if the 
 * applied voltage is constant. If this is the case, then
 * the process checks if the corresponding position measurament
 * is constant as well. This is assumed to be the normal situation
 * when the robot is not perfomring movements. Therefore, no message
 * is reported when both applied voltages and position measuraments
 * are constant. Otherwise, errors are reported.
 *
 * \section libraries_sec Libraries
 * Yarp libraries.
 * 
 * 
 * \section parameters_sec Parameters
 * The module uses the ResourceFinder class as a way to retrieve its
 * own configuration. In particular, the default config file is
 * controlBoardTester.ini and it is assumed to be in the app/controlBoardTester
 * directory or in the current directory. The file structure is the following:
 *
 * \code
 *
 * rate         r                //rate of the main thread
 *
 * robot        R                //name of the used robot
 *
 * part         p                //name of the used part
 *
 * \endcode
 * 
 * If no such file can be found, default values are used
 * (r = 500, p = head, R=icub).

 * \section portsa_sec Ports Accessed
 * The module accesses the controlBoard ports.
 *
 * \section portsc_sec Ports Created
 * The module creates the remoteControlBoard ports.
 *
 * \author Francesco Nori
 *
 * Copyright (C) 2008 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/controlBoardDumper/main.cpp.
 */

//A local class for statistics on a port supplied data
#include "statCollector.h"
//ACE
#include <ace/OS.h>
#include <ace/Log_Msg.h>
//YARP
#include <yarp/os/Bottle.h>
#include <yarp/os/impl/String.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Module.h>

//

//using namespace yarp::dev;
using namespace yarp::os::impl;

class DumpModule: public Module
{
private:
    statCollector *st;
    PolyDriver dd;
public:
    DumpModule() 
    { 

    }

    virtual bool open(Searchable &s)
    {
        Time::turboBoost();
        // get command line options
        Property options;
        options.fromString(s.toString());
        if (!options.check("rate")) 
            {
                ACE_OS::printf("Missing --rate option. Quitting!\n");
                return false;
            }


        //get the remoteControlBoard
        Property ddOptions;
        ddOptions.put("device", "remote_controlboard");
    
        ConstString localPortName = "/";
        localPortName = localPortName + "controlBoardTester/";
        localPortName = localPortName + options.find("robot").toString();
        localPortName = localPortName + options.find("part").toString();
        ddOptions.put("local", localPortName.c_str());

        ConstString remotePortName = "/";
        remotePortName = remotePortName + options.find("robot").toString();
        remotePortName = remotePortName + "/";
        remotePortName = remotePortName + options.find("part").toString();
        ddOptions.put("remote", remotePortName.c_str());
    
        fprintf(stderr, "%s", ddOptions.toString().c_str());    
        // create a device 
        dd.open(ddOptions);
        if (!dd.isValid()) {
            ACE_OS::printf("Device not available.  Here are the known devices:\n");
            ACE_OS::printf("%s", Drivers::factory().toString().c_str());
            Network::fini();
            return false;
        }
        
        //create the statistic collector
        int rate = options.find("rate").asInt();
        st = new statCollector(rate);
        st->setDeviceDriver(&dd);
        st->start();
        return true;
    }

    virtual bool close()
    {
        fprintf(stderr, "Stopping the threads\n");
        st->stop();
        fprintf(stderr, "Deleting the threads\n");
        delete st;
        dd.close();
        
        fprintf(stderr, "Deleting the input port\n");
        return true;
    }
};


int main(int argc, char *argv[]) 
{

    Network::init();
    ResourceFinder rf;

    rf.setVerbose(true);
    rf.setDefaultContext("controlBoardTester");
    rf.setDefaultConfigFile("controlBoardTester.ini");
    rf.setDefault("part", "head");
    rf.setDefault("robot", "icub");
    rf.configure("ICUB_ROOT", argc, argv);

    Property p;
    DumpModule mod;
    p.fromString(rf.toString());

    if (!p.check("rate"))
        p.put("rate", 500);

    fprintf(stderr, "Current configuration is: %s\n", p.toString().c_str());
    if (mod.open(p))
        return mod.runModule();
    else 
        return 0;

    Network::fini();
}
