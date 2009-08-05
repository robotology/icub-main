// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <ace/OS.h>
#include <ace/Log_Msg.h>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>

#include <yarp/String.h> 

#include <yarp/dev/PolyDriver.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp;

int get_device(PolyDriver& dd, int argc, char *argv[]) 
{
    // just list the devices if no argument given
    if (argc <= 2) {
        ACE_OS::printf("You can call %s like this:\n", argv[0]);
        ACE_OS::printf("   %s --robot ROBOTNAME --OPTION VALUE ...\n", argv[0]);
        ACE_OS::printf("For example:\n");
        ACE_OS::printf("   %s --robot icub --local /talkto/james --remote /controlboard/rpc\n", argv[0]);
        ACE_OS::printf("Here are devices listed for your system:\n");
        ACE_OS::printf("%s", Drivers::factory().toString().c_str());
        return 1;
    }

    // get command line options
    Property options;
    options.fromCommand(argc, argv);
    if (!options.check("robot") || !options.check("part")) {
        ACE_OS::printf("Missing either --robot or --part options\n");
        return 1;
    }

    Network::init();
    Time::turboBoost();
    
    yarp::String name((size_t)1024);
    Value& v = options.find("robot");
    Value& part = options.find("part");

    Value *val;
    if (!options.check("device", val)) {
        options.put("device", "remote_controlboard");
    }
    if (!options.check("local", val)) {
        ACE_OS::sprintf(&name[0], "/%s/%s/client", v.asString().c_str(), part.asString().c_str());
        options.put("local", name.c_str());
    }
    if (!options.check("remote", val)) {
        ACE_OS::sprintf(&name[0], "/%s/%s", v.asString().c_str(), part.asString().c_str());
        options.put("remote", name.c_str());
    }
    
    // create a device
    dd.open(options);
    if (!dd.isValid()) {
        ACE_OS::printf("Device not available.  Here are the known devices:\n");
        ACE_OS::printf("%s", Drivers::factory().toString().c_str());
        Network::fini();
        return 1;
    }

    return 0;
}
