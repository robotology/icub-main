// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

 /**
* \file fggui2_main.cpp A graphical interface to set gain, shutter, brightness and color balance on a remote framegrabber.
*/


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
 * Copyright (C) 2007 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at /src/tools/frameGrabberGui2/srcfggui2_main.cpp
 *
 */

#include <gtkmm.h>
#include <FrameGrabberGUIControl2.h>

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
