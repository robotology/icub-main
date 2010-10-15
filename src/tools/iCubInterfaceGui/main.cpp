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

#include "iCubNetwork.h"
#include "iCubInterfaceGuiServer.h"

int main(int argc, char *argv[])
{
    return 0;
}

#ifdef notdef

#include <gtkmm.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include "iCubInterfaceGui.h"

int main(int argc, char *argv[])
{
	yarp::os::Network yarp; //initialize network, this goes before everything
	Gtk::Main kit(argc,argv);

	yarp::os::ResourceFinder rf;
	rf.setVerbose();
	rf.setDefaultConfigFile("iCubInterface.ini");
	rf.configure("ICUB_ROOT",argc,argv);
	yarp::os::ConstString configFile=rf.findFile("config");
	yarp::os::ConstString cartRightArmFile=rf.findFile("cartRightArm");
	yarp::os::ConstString cartLeftArmFile=rf.findFile("cartLeftArm");

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

	iCubInterfaceGui window(robot);
	
	Gtk::Main::run(window);

	return 0;
}
#endif

#ifdef notdef

#include <gtkmm.h>

#include <yarp/os/Property.h> 
#include <yarp/os/Network.h>

#include "iCubInterfaceGui.h"

int main(int argc, char *argv[])
{
	yarp::os::Network yarp;

    Gtk::Main kit(argc, argv);

    /*
	yarp::os::Property options;
    options.fromCommand(argc,argv);

	// switch to subsections if available
	yarp::os::Searchable *Network = &options.findGroup("NETWORK");
	yarp::os::Searchable *Window = &options.findGroup("WINDOW");
	//yarp::os::Searchable *Program = &options.findGroup("PROGRAM");

	if (Network->isNull()) { Network = &options; }
	if (Window->isNull()) { Window = &options; }
	//if (Program->isNull()) { Program = &options; }

	yarp::os::Value *val;
    
	char portName[256]="";
    char outPortName[256]="";

	if (Network->check("PortName",val)||Network->check("local",val)) 
	{
		strcpy(portName, val->asString().c_str());
	}

	if (Network->check("OutPortName",val)||Network->check("remote",val)) 
	{
		strcpy(outPortName, val->asString().c_str());
	}

    printf("using local=%s remote=%s x=%d y=%d\n",portName,outPortName,posX,posY);
	fflush(stdout);
    */

        

	iCubInterfaceGui window("","");

    /*
	int posX=0,posY=0;
	int width,height;
	window.get_size(width,height);

	if (Window->check("PosX",val)||Window->check("x",val)) posX = val->asInt();
	if (Window->check("PosY",val)||Window->check("y",val)) posY = val->asInt();
	if (Window->check("width",val)) width = val->asInt();
	if (Window->check("height",val)) height = val->asInt();
    */

	window.set_size_request(320,480);
	window.move(64,64);
	Gtk::Main::run(window);

    

    return 0;
}

#endif