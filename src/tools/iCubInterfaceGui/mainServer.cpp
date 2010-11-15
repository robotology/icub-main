// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \defgroup icub_framegrabbergui2 frameGrabberGui2
 * @ingroup icub_guis
 * @ingroup icub_tools
 *
 * A graphical interface to set gain, shutter, brightness and color balance on a remote framegrabber.
 * Run without options to see help on usage.
 *
 * The frameGrabberGui module opens one port connected to a RemoteFrameGrabber control port.
 * The name of the local and remote ports are provided as command arguments.
 *
 *
 * Usage:\n
 * frameGrabberGui2 --local <localportname> --remote <grabberport> [--width <gui_width>] [--height <gui_height>] [--x <gui_x_pos>] [--y <gui_y_pos>]
 * 
 * \see yarp::dev::RemoteFrameGrabber, FrameGrabberGUIControl
 *
 * \author Alessandro Scalzo
 *
 * Copyright (C) 2009 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/gui/iCubInterfaceGui/main.cpp.
 *
 */

#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include "iCubInterfaceGuiServer.h"

class FakeDriver : public yarp::os::RateThread
{
public:
    FakeDriver(iCubInterfaceGuiServer *server) 
        : yarp::os::RateThread(1500) // period is greater than client period:
                                     // some packets will be empty
    {
        pServer=server;
    }

    virtual ~FakeDriver()
    {
    }

    void run()
    {
        static int counter=0;

        pServer->findAndWrite("net_headtorso,2,1,5",yarp::os::Value(counter=!counter));
    }

protected:
    iCubInterfaceGuiServer *pServer;
};

int main(int argc, char *argv[])
{
	yarp::os::Network yarp; //initialize network, this goes before everything

	yarp::os::ResourceFinder rf;
	rf.setVerbose();
	rf.setDefaultConfigFile("iCubInterface.ini");
	rf.configure("ICUB_ROOT",argc,argv);
	yarp::os::ConstString configFile=rf.findFile("config");

	//yarp::os::ConstString cartRightArmFile=rf.findFile("cartRightArm");
	//yarp::os::ConstString cartLeftArmFile=rf.findFile("cartLeftArm");

	if (configFile=="")
	{
		printf("\n");
		printf("Error: iCubInterfaceGui was not able to find a valid configuration file\n");

		return -1;
	}

	yarp::os::Property robot;
	if (!robot.fromConfigFile(configFile.c_str()))
	{
		fprintf(stderr,"Sorry could not open %s\n",configFile.c_str());
		return -1;
	}
	printf("Config file loaded successfully\n");

    //printf("%s\n%s\n",configFile.c_str(),robot.toString().c_str());

	iCubInterfaceGuiServer server;

    server.config(robot);

    server.start();

    FakeDriver fakeDriver(&server);

    fakeDriver.start();

    getchar();

    fakeDriver.stop();
    server.stop();

	return 0;
}
