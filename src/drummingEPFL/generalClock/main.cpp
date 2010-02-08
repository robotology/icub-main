/* 
 * Copyright (C) 2008, Sarah Degallier, Ludovic Righetti, BIRG - EPFL Lausanne
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   sarah.degallier@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 * @ingroup icub_module
 *
 * \defgroup icub_GeneralClock GeneralClock
 *
 * This module is part of the application \ref icub_drummingEPFL "drummingEPFL"
 *
 *\section intro_sec Description 
 *
 * This module is a general clock that serves as an absolute time referential for the network of oscillators (CPGs). If you want to  use the whole drumming application, please refer to \ref icub_drummingEPFL "drummingEPFL".
 *
 *\section lib_sec Libraries
 *
 *No external libraries.
 *
 *\section parameters_sec Parameters
 *
 * --period 20
 * 
 * It gives the period of the module - it should be the same that the one of the \ref icub_velocityControl "velocityControl" modules.  
 *
 *\section portssa_sec Ports Accessed 
 *
 *Input ports\n
 *<ul>
 *<li> /clock/check_motion/in (created by the \ref icub_DrumManager "DrumManager" module)
 *</ul>
 *
 * Output ports\n
 * <ul>
 * <li> /clock/parameters/out (created by the \ref icub_DrumManager "DrumManager" module)
 *</ul>
 *  
 *
 *\section portsc_sec Ports Created
 *
 * Input ports\n
 * <ul>
 * <li> the port /clock/parameters/in receives the frequency of the movement. 
 *
 * Output ports\n
 * <ul>
 *<li> The part /clock/check_motion/out sends to the \ref icub_DrumManager "DrumManager" the current beat. 
 *</ul>
 *
 *\section conf_file_sec Configuration Files
 *
 * This module does not require any config file.
 *
 *\section tested_os_sec Tested OS
 *
 * This module has been tested on Linux and Windows. 
 *
 *\section example_sec Example Instantiation of the Module
 *
 * ./GeneralClock --period 20 
 *
 * This file can be edited at \in src/drummingEPFL/DrumGenerator/main.cpp 
 *
 *\authors Sarah Degallier Ludovic Righetti 
 *
 **/




#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Module.h>

#include "clockModule.h"

using namespace yarp::os;
using namespace yarp::dev;



int main(int argc, char *argv[])
{
  Network yarp;

  //create and run the Drum generator module
  clockModule mod;
  
  return mod.runModule(argc,argv);

}
