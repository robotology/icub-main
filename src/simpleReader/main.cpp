// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_module
 *
 * \defgroup icub_simple_client simple_client
 *
 * A basic remote interface to a controller.
 * Call with no arguments for usage information.
 *
 * \author Maybe Giorgio?
 *
 */


#include <ace/OS.h>
#include <ace/Log_Msg.h>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <yarp/String.h> 

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp;

//
int main(int argc, char *argv[]) 
{
    // just list the devices if no argument given
    if (argc <= 2) {
        ACE_OS::printf("You can call %s like this:\n", argv[0]);
        ACE_OS::printf("   %s --robot ROBOTNAME --OPTION VALUE ...\n", argv[0]);
        ACE_OS::printf("For example:\n");
        ACE_OS::printf("   %s --robot icub --local /talkto/james --remote /controlboard/rpc\n", argv[0]);
        ACE_OS::printf("Here are devices listed for your system:\n");
        ACE_OS::printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    // get command line options
    Property options;
    options.fromCommand(argc, argv);
    if (!options.check("robot") || !options.check("part")) {
        ACE_OS::printf("Missing either --robot or --part options\n");
        return 0;
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
        ACE_OS::sprintf(&name[0], "/%s/%s/reader", v.asString().c_str(), part.asString().c_str());
        options.put("local", name.c_str());
    }
    if (!options.check("remote", val)) {
        ACE_OS::sprintf(&name[0], "/%s/%s", v.asString().c_str(), part.asString().c_str());
        options.put("remote", name.c_str());
    }

	fprintf(stderr, "%s", options.toString().c_str());

    
    // create a device 
    PolyDriver dd(options);
    if (!dd.isValid()) {
        ACE_OS::printf("Device not available.  Here are the known devices:\n");
        ACE_OS::printf("%s", Drivers::factory().toString().c_str());
        Network::fini();
        return 1;
    }

    IPositionControl *pos;
    IVelocityControl *vel;
    IEncoders *enc;
    IPidControl *pid;
    IAmplifierControl *amp;
    IControlLimits *lim;

    bool ok;
    ok = dd.view(pos);
    ok &= dd.view(vel);
    ok &= dd.view(enc);
    ok &= dd.view(pid);
    ok &= dd.view(amp);
    ok &= dd.view(lim);

    if (!ok) {
        ACE_OS::printf("Problems acquiring interfaces\n");
        return 1;
    }

    int jnts = 0;
    pos->getAxes(&jnts);
    ACE_OS::printf("Working with %d axes\n", jnts);
    double *tmp = new double[jnts];
    ACE_ASSERT (tmp != NULL);

    ACE_OS::printf("Device active...\n");
    while (dd.isValid()) {
		char c='s';
		while (c!='q')
		{	
			Time::delay(0.1);
			enc->getEncoders(tmp);
            for(int i = 0; i < jnts; i++)
				ACE_OS::printf ("%.2f ", tmp[i]);
            ACE_OS::printf ("\r");
		
		}
} /* while () */

ApplicationCleanQuit:
    dd.close();
    delete[] tmp;

    Network::fini();
    return 0;
}
