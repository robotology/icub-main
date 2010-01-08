
/* 
 * Copyright (C) 2008 Sebastien GAY
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   sebastien.gay@epfl.ch
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
 * \defgroup icub_ReachManager ReachManager
 *
 * This module is part of the application \ref icub_Crawling "Crawling"
 *
 *\section intro_sec Description 
 *
 * This module gets a list (Bottle) of objects defined as an ID and its 3D position in the root reference frame of the.// 
 * It computes the joints angles in order to reach the nearest position of a certain ID. 
 *
 *\section lib_sec Libraries
 *
 * This module requires two iKinCartesianSolver (see ref iKinCartesianSolver) modules started. One for each arm.
 *
 *
 *\section parameters_sec Parameters
 *
 * The config file (default is  $ICUB_ROOT/app/Crawling/config/reach/config.ini). call with :
 *   --file  [path to config file].ini
 *
 *
 *
 *\section portssa_sec Ports Accessed 
 *
 * 
 * <ul>
 * <li> all ports created by the iKinCartesianSolver module for both arms should be available.
 *</ul>
 *
 *\section portsc_sec Ports Created
 *
 * </ul>
 * Input ports\n
 * <ul>
 * <li> /ReachManager/in to receive the IDs of the objects and their position in the root reference frame of the robot
 *</ul>
 * Output ports\n
 * <ul>
 * <li> /ReachManager/out to output the joint configurations.
 * </ul>
 *
 *\section conf_file_sec Configuration Files
 *
 *This module requires a config file :
 *<ul>
 *<li> config.ini
 *</ul>
 *
 * It can be found at \in $ICUB_ROOT/app/Crawling/config/reach/config.ini 
 *
 *
 *\section tested_os_sec Tested OS
 *
 * This module has been tested on Linux and Windows. 
 *
 *\section example_sec Example Instantiation of the Module
 *
 * ReachManager --file "C:\iCub\app\Crawling\config\reach\config.ini"

 * This file can be edited at \in src/crawling/ReachManager/main.cpp 
 *
 *\authors Sebastien GAY
 *
 **/

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Vector.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;


#include "ReachManager.h"

int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

	ReachManager reachManagerModule;
	
	return reachManagerModule.runModule(argc, argv);;
}


      
