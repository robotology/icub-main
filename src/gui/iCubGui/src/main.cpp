/*
*
* \defgroup icub_iCubGui iCubGui
* @ingroup icub_guis
*
* A gui that shows the full state of the robot. At the moment
* encoders and inertial sensors.
*
*
* \section intro_sec Description
* 
* Display a kinematic model of the robot from the encoders. It also
* shows a representation of the output of the inertial sensor.

You might find it convenient to include an image:
\image html EXAMPLE.jpg
\image latex EXAMPLE.eps "MODULENAME running on Linux" width=10cm

\section lib_sec Libraries
 - YARP
 - Qt3, glut, opengl.

\section parameters_sec Parameters
None.

\section portsa_sec Ports Accessed

iCubGui needs access to all the robot interfaces and ports (from iCubInterface).

\section tested_os_sec Tested OS

Linux and Windows.

\section example_sec Example Instantiation of the Module

iCubGui

\author Alessandro Scalzo

Copyright (C) 2009 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/gui/iCubGui/src/main.cpp.
**/

#include "qavimator.h"
#include <qapplication.h>

int main( int argc, char ** argv )
{
    QApplication a(argc,argv);
    qavimator* mw=new qavimator();
    mw->show();
    a.connect(&a,SIGNAL(lastWindowClosed()),&a,SLOT(quit()));
    return a.exec();
}
