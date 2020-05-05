// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// Copyright (C) 2015 Istituto Italiano di Tecnologia - iCub Facility
// Author: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

// *******************************
// THIS MODULE IS EXPERIMENTAL!!!!
// *******************************

#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include "positionDirectThread.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <string.h>

//default rate for the control loop
const int CONTROL_PERIOD=10;

using namespace yarp::os;
using namespace yarp::dev;

class VelControlModule: public RFModule {
private:
    PolyDriver driver;
    positionDirectControlThread *pThread;
    char partName[255];
  
    Port rpc_port;

public:
    virtual bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
    {
        reply.clear(); 
        if (command.get(0).isString())
        {
            if (command.get(0).asString()=="help")
            {
                reply.addVocab(Vocab::encode("many"));
                reply.addString("Available commands:");
                reply.addString("currently nothing");
                return true;
            }
            else if (command.get(0).asString()=="***")
            {
                return true;
            }
        }
        reply.addString("Unknown command");
        return true;
    }
  
    virtual bool configure(yarp::os::ResourceFinder &rf)
    {
        Property options;
        options.fromString(rf.toString());
        char robotName[255];
        Bottle *jointsList=0;
        std::string moduleName = "directPositionControl";

        if (options.check("help"))
        {
            yInfo() << "Available options:";
            yInfo() << "--robot <robot name>";
            yInfo() << "--name <local_module_name>";
            yInfo() << "--part <part_name>";
            yInfo() << "--joints ""(x x x)"" ";
            yInfo() << "--joints_limiter <deg>";
            yInfo() << "--target_limiter <deg>";
            yInfo() << "--period <s>";
            return false;
        }

        options.put("device", "remote_controlboard");
        if(options.check("robot"))
            strncpy(robotName, options.find("robot").asString().c_str(),sizeof(robotName));
        else
            strncpy(robotName, "icub", sizeof(robotName));

        if (options.check("name"))
        {
            moduleName = options.find("name").asString();
        }

        if(options.check("part"))
        {
            snprintf(partName, sizeof(partName), "%s", options.find("part").asString().c_str());

            char tmp[800];
            snprintf(tmp, sizeof(tmp), "/%s/%s/%s/client", moduleName.c_str(), robotName, partName);
            options.put("local",tmp);
        
            snprintf(tmp, sizeof(tmp), "/%s/%s", robotName, partName);
            options.put("remote", tmp);
        
            snprintf(tmp, sizeof(tmp), "/%s/%s/rpc", moduleName.c_str(), partName);
            rpc_port.open(tmp);
            
            options.put("carrier", "tcp");
            
            attach(rpc_port);
        }
        else
        {
            yError("Please specify part (e.g. --part head)\n");
            return false;
        }

        if(options.check("joints"))
        {
            jointsList = options.find("joints").asList();
            if (jointsList==0) yError("Unable to parts 'joints' parameter\n");
        }
        else
        {
            yError("Please specify the joints to control (e.g. --joints ""(0 1 2)"" ");
            return false;
        }


        double joints_limiter = 2.0; //deg
        double target_limiter = 1.0; //deg
        if (options.check("joints_limiter"))
        {
            joints_limiter = options.find("joints_limiter").asDouble();
        }
        if (options.check("target_limiter"))
        {
            target_limiter = options.find("target_limiter").asDouble();
        }

        yDebug() << "joints_limiter:" << joints_limiter;
        yDebug() << "target_limiter" << target_limiter;

        //opening the device driver
        if (!driver.open(options))
        {
            yError("Error opening device, check parameters\n");
            return false;
        }

        ///starting the thread
        int period = CONTROL_PERIOD;
        if(options.check("period"))
            period = options.find("period").asInt();
        
        yInfo("control rate is %d ms",period);

        pThread=new positionDirectControlThread(period);
        pThread->init(&driver, moduleName, partName, robotName, jointsList);
        pThread->joints_limiter = joints_limiter;
        pThread->target_limiter = target_limiter;
        pThread->start();

        return true;
    }

    virtual bool close()
    {
        yInfo("Closing module [%s]\n", partName);
        pThread->stop();
        pThread->halt();
        delete pThread;
        yInfo("Thread [%s] stopped\n", partName);

        driver.close();
        rpc_port.close();

        yInfo("Module [%s] closed\n", partName);
        return true;
    }

    bool updateModule()
    {
        return true;
    }

};

int main(int argc, char *argv[])
{
    Network yarp;
    VelControlModule mod;
    ResourceFinder rf;
    
    rf.configure(argc, argv);
    if (mod.configure(rf)==false) return 1;
    mod.runModule();
    yInfo("Main returning\n");
    return 0;

}
