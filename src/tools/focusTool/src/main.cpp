/*
 * Copyright (C) 2011 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
*
* \defgroup focustool_gui focusTool
* @ingroup icub_guis
*
*
* A simple camera focus calibration tool.
*
*
* \section intro_sec Description
* 
* When this tool is connected to a running camera, it provides a visual representation of
* the energy of the edges in the images. A level indicator bar shows the rate between
* the actual energy in the image and the maximum energy level detected during the calibration
* process. The calibration procedure is the following: the focus adjust must be slowly changed 
* from its minimum to its maximum; the maximum of the energy is recorder by the tool during this
* calibration phase. In the second phase, the focus adjust must be brought back until the maximum
* is shown by the energy level bar. 
* 
*
* \section lib_sec Libraries
*  - YARP
*  - GtkMM.
*
* \section parameters_sec Parameters
* --local /localport
* --remote /cameraport
*
* \section portsa_sec Ports Accessed
*
* \code
* /localport
* /cameraport
* \endcode
*
* \section tested_os_sec Tested OS
*
* Linux and Windows.
*
* \author Alessandro Scalzo
*
* Copyright (C) 2011 RobotCub Consortium
*
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
* This file can be edited at main/src/tools/focusTool/main.cpp.
*/




#include <gtkmm.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/Network.h>

#include "iCubFocusGui.h"

int main(int argc, char *argv[])
{
    if (argc!=3)
    {
        printf("USAGE: %s /localport /cameraport\n",argv[0]);
        return 1;
    }
    
    Glib::thread_init();
    Gtk::Main kit(argc,argv);
	yarp::os::Network yarp;

    yarp::os::Property config;
    config.put("local",argv[1]);
    config.put("remote",argv[2]);
    //config.fromCommand(argc,argv);
    iCubFocusGuiWindow client(config);

    kit.run(client);

    return 0;
}
