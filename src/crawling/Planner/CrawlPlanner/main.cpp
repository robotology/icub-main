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
 * \defgroup icub_MultiMarkerTracker MultiMarkerTracker
 *
 *This module is part of the application \ref icub_Crawling "Crawling"
 *
 *\section intro_sec Description 
 *
 * This module is a path planner for the iCub when crawling.
 * It receives IDs and 3D position of objects in the root reference frame of the robot.
 * Some IDs correspond to obstacles, some to goals.
 * The module computes the rotation angle of the robot (torso rool angle) to avoid obstacles and reach goals using an artificial potential fields approach.
 * The module outputs the torso yaw angle of the robot.
 *
 *\section lib_sec Libraries
 *
 * This module requires no additional library
 *
 *
 *\section parameters_sec Parameters
 *
 * The config file (default is  $ICUB_ROOT/app/Crawling/config/tracker/config.ini). call with :
 *   --from  [path to config file].ini
 *
 *
 *
 *\section portssa_sec Ports Accessed 
 *
 * 
 * <ul>
 * <li> all ports of the iCub Interface for the head should be running 
 * for the module to have access to the encoders of the head.
 * </ul>
 *
 *\section portsc_sec Ports Created
 *
 * </ul>
 * Input ports\n
 * <ul>
 * <li> /CrawlPlanner/in to receive the positions of the obtacles and goals.
 *</ul>
 * Output ports\n
 * <ul>
 * <li> /CrawlPlanner/out to output the torso angle.
 * </ul>
  * Other ports\n
 * <ul>
 * <li> /CrawlPlanner/neckAngle to get the encoders of the head.
 * <li> /CrawlPlanner/supervisor/out to communicate with the optional gui to display the potential field.
 * </ul>
 *
 *\section conf_file_sec Configuration Files
 *
 *This module requires a config file :
 *<ul>
 *<li> config.ini
 *</ul>
 *
 * It can be found at \in $ICUB_ROOT/app/Crawling/config/planner/config.ini 
 *
 *
 *\section tested_os_sec Tested OS
 *
 * This module has been tested on Windows. 
 *
 *\section example_sec Example Instantiation of the Module
 *
 * CrawlPlanner --file "C:\iCub\app\Crawling\config\planner\config.ini"

 * This file can be edited at \in src/crawling/Planner/CrawlPlanner/main.cpp 
 *
 *\authors Sebastien GAY
 *
 **/

#include <stdio.h>
#include <yarp/os/all.h>
using namespace yarp::os;

#include "CrawlPlanner.h"

int main(int argc, char* argv[])
{
    Network yarp;

    //create and run tracker module
    CrawlPlanner CrawlPlanner;
  
    return CrawlPlanner.runModule(argc, argv);
}