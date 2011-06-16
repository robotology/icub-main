/*
 * Copyright (C) 2010 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
*
* \defgroup icubinterfacegui_gui iCubInterfaceGuiClient
* @ingroup icub_guis
*
*
* A gui that shows the full state of the robot control board networks.
*
*
* \section intro_sec Description
* 
* Displays the status of the robot motor and sensor control board networks
* in a hyerarchical tree view. It connects to its server part running inside
* the \ref icub_iCubInterface iCubInterface. 
* 
* \image html icubinterfacegui.jpg "Screenshots: iCubInterfaceGuiClient running on Windows"
*
*
* \section lib_sec Libraries
*  - YARP
*  - GtkMM.
*
* \section parameters_sec Parameters
* It doesn't need any parameter or configuration file because
* it receives the network configuration from the iCubInterface
* server side in the connection phase.
*
* \section portsa_sec Ports Accessed
*
* \code
* icubinterfacegui/server
* \endcode
*
* \section tested_os_sec Tested OS
*
* Linux and Windows.
*
* \author Alessandro Scalzo
*
* Copyright (C) 2010 RobotCub Consortium
*
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
* This file can be edited at main/src/tools/iCubInterfaceGui/iCubInterfaceGuiClient/mainClient.cpp.
*/



#include <gtkmm.h>
#include <yarp/os/Network.h>

#include "iCubInterfaceGuiClient.h"

int main(int argc, char *argv[])
{
    Glib::thread_init();
    Gtk::Main kit(argc,argv);

	yarp::os::Network yarp;

    iCubInterfaceGuiWindow client;

    kit.run(client);

    return 0;
}
