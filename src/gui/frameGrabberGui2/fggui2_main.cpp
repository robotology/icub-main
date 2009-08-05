// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alessandro Scalzo
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
 * @ingroup icub_module
 *
 * \defgroup icub_framegrabbergui frameGrabberGui
 *
 * A graphical interface to set gain, shutter, brightness and color balance on a remote framegrabber.
 * Run without options to see help on usage.
 *
 * The frameGrabberGui module opens one port connected to a RemoteFrameGrabber control port.
 * The name of the local and remote ports are provided as command arguments.
 *
 * \dot
 * digraph module_framegrabbergui_example {
 *     graph [ rankdir = "LR" ];
 *     edge [arrowhead="open", style="solid"];
 *     node [shape=ellipse];
 *     subgraph cluster_framegrabbergui {
 *      color = "black"; style = "solid";
 *      label = "frameGrabberGui module";
 *       "/fggui/right";
 *     }
 *     "/fggui/right" -> "/icub/cam/right"
 * \enddot
 *
 * Usage:\n
 * frameGrabberGui --local <localportname> --remote <grabberport> [--width <gui_width>] [--height <gui_height>] [--x <gui_x_pos>] [--y <gui_y_pos>]
 * 
 * \see yarp::dev::RemoteFrameGrabber, FrameGrabberGUIControl
 *
 * \author Alessandro Scalzo
 *
 */

#include <gtkmm.h>
#include "FrameGrabberGUIControl2.h"

//#include <ace/config.h>
//#include <ace/OS.h>
#include <yarp/os/Property.h> 
#include <yarp/os/Network.h>
#include <string.h>

int main(int argc, char *argv[])
{
	yarp::os::Network yarp;

    Gtk::Main kit(argc, argv);

	if (argc == 3)
	{
		FrameGrabberGUIControl2 window(argv[1],argv[2]);
		Gtk::Main::run(window);
	}
	else if (argc<3)
	{
		//FrameGrabberGUIControl window("","");
		//Gtk::Main::run(window);
		printf("usage:\n");
		printf("%s --local <localportname> --remote <grabberport> [--width <gui width>] [--height <gui height>] [--x <gui x pos>] [--y <gui y pos>]\n",argv[0]);
	}
	else if (argc>3)
	{
		yarp::os::Property options;
        options.fromCommand(argc,argv);
    
		printf("%s\n",options.toString().c_str());

		// switch to subsections if available
		yarp::os::Searchable *Network = &options.findGroup("NETWORK");
		yarp::os::Searchable *Window = &options.findGroup("WINDOW");
		//yarp::os::Searchable *Program = &options.findGroup("PROGRAM");

		if (Network->isNull()) { Network = &options; }
		if (Window->isNull()) { Window = &options; }
		//if (Program->isNull()) { Program = &options; }

		yarp::os::Value *val;
    
		char portName[256]="",outPortName[256]="";

		if (Network->check("PortName",val)||Network->check("local",val)) 
		{
			strcpy(portName, val->asString().c_str());
		}

		if (Network->check("OutPortName",val)||Network->check("remote",val)) 
		{
			strcpy(outPortName, val->asString().c_str());
		}

		//printf("using local=%s remote=%s x=%d y=%d\n",portName,outPortName,posX,posY);
		//fflush(stdout);

		FrameGrabberGUIControl2 window(portName,outPortName);
		int posX=0,posY=0;
		int width,height;
		window.get_size(width,height);

		if (Window->check("PosX",val)||Window->check("x",val)) posX = val->asInt();
		if (Window->check("PosY",val)||Window->check("y",val)) posY = val->asInt();
		if (Window->check("width",val)) width = val->asInt();
		if (Window->check("height",val)) height = val->asInt();

		window.set_size_request(width,height);
		window.move(posX,posY);
		Gtk::Main::run(window);
	}

    return 0;
}
