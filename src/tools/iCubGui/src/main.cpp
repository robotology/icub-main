/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Based on:
 *
 *   Qavimator
 *   Copyright (C) 2006 by Zi Ree   *
 *   Zi Ree @ SecondLife   *
 *   Released under the terms of the GNU GPL v2.0.
 */

/**
*
* \defgroup icub_gui iCubGui
* @ingroup icub_guis
*
*
* A gui that shows the full state of the robot. At the moment
* encoders and inertial sensors.
*
*
* \section intro_sec Description
*
* Display a kinematic model of the robot from the encoders. It also
* shows a representation of the output of the inertial sensor.
*
* \image html icubgui-merged.jpg "Screenshots: iCubGui running on Linux"
* \image latex icubgui1.eps "Screenshots: iCubGui running on Linux" width=10cm
*
* \section visobj_sec Vision objects
*
* iCubGui can also display data blobs from vision modules, representing them as
* 3D ellipsoids characterized by name, dimensions, color, transparency, position
* and orientation. Objects in iCubGui are managed by a port specified in the
* iCubGui.ini configuration file with the tag \e objport, as follows:
*
* \code
* robot /icubSim
* geometry skeleton.ini
* objport /iCubGui/objects
* \endcode
*
* To add an object to the display object list, send to iCubGui a bottle
* composed as following on the port specified in iCubGui.ini:
*
* \code
* yarp::os::Bottle obj;
*
* obj.addString("object"); // command to add/update an object
* obj.addString("my_object_name");
*
* // object dimensions in millimiters
* // (it will be displayed as an ellipsoid with the tag "my_object_name")
* obj.addDouble(dimX);
* obj.addDouble(dimY);
* obj.addDouble(dimZ);
*
* // object position in millimiters
* // reference frame: X=fwd, Y=left, Z=up
* obj.addDouble(posX);
* obj.addDouble(posY);
* obj.addDouble(posZ);
*
* // object orientation (roll, pitch, yaw) in degrees
* obj.addDouble(rotX);
* obj.addDouble(rotY);
* obj.addDouble(rotZ);
*
* // object color (0-255)
* obj.addInt(R);
* obj.addInt(G);
* obj.addInt(B);
* // transparency (0.0=invisible 1.0=solid)
* obj.addDouble(alpha);
* \endcode
*
* To delete an object:
*
* \code
* yarp::os::Bottle obj;
*
* obj.addString("delete");
* obj.addString("my_object_name");
* \endcode
*
* To reset the object list:
*
* \code
* yarp::os::Bottle obj;
*
* obj.addString("reset");
* \endcode
*
* \section lib_sec Libraries
*  - YARP
*  - Qt5, glut, opengl.
*
* \section parameters_sec Parameters
* None.
*
* \section portsa_sec Ports Accessed
*
* Needs access to all the robot interfaces and ports (from iCubInterface).
*
* \section tested_os_sec Tested OS
*
* Linux and Windows.
*
* \author Alessandro Scalzo
*
* Copyright (C) 2009 RobotCub Consortium
*
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
* This file can be edited at src/gui/iCubGui/src/main.cpp.
*/

#include <signal.h>

#include "qavimator.h"
#include <QApplication>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#ifdef USE_ICUB_MOD
#include "drivers.h"
#endif

qavimator* mw=NULL;

std::string GUI_NAME("/iCubGui");

void sig_handler(int sig)
{
    if (mw) mw->fileExit();
}

int main(int argc, char *argv[])
{
    yarp::os::Network yarp; //initialize network, this goes before everything

#ifdef USE_ICUB_MOD
    yarp::dev::DriverCollection dev;
#endif

#ifndef WIN32
    signal(SIGINT,sig_handler);
    signal(SIGTERM,sig_handler);
    signal(SIGHUP,sig_handler);
#endif

    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("iCubGui");
    rf.setDefaultConfigFile("iCubGui.ini");
    rf.configure(argc,argv);

    QApplication a(argc, argv);

    mw = new qavimator(rf);
    mw->show();
    a.connect(&a,SIGNAL(lastWindowClosed()),&a,SLOT(quit()));
    int ret =  a.exec();

    if(mw){
        delete mw;
    }
    return (ret!=0?1:0);
}
