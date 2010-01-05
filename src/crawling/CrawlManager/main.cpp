
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2008 Sarah Degallier Ludovic Righetti BIRG EPFL Lausanne
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   sarah.degallier@robotcub.org ludovic.righetti@a3.epfl.ch
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
 * \defgroup icub_CrawlManager CrawlManager
 *
 *This module is part of the application \ref icub_Crawling "Crawling"
 *
 *\section intro_sec Description 
 *
 * This module transforms a score into the appropriate parameters for the dynamical systems generating the trajectories (\in \ref icub_Crawling "Crawling"). It sends the parameters at timing corresponding to the beat of each of the part.If you want to  use the whole drumming application, please refer to \ref icub_Crawling "Crawling". 
 *
 *\section lib_sec Libraries
 *
 *No external libraries
 *
 *
 *\section parameters_sec Parameters
 *
 * No parameters. 
*
* They are read by default from the file app/Crawling/config/managerConfig.ini, another file \a newFile.ini can be used through the command :
*   --config-path  newFile.ini
 *
 *\section portssa_sec Ports Accessed 
 *
 * Input ports\n
 * <ul>
 * <li> for each active \a part (i.e left_arm, right_arm, left_leg, right_leg, torso, head) of the robot: /part/parameters/in (created by the \ref icub_CrawlGenerator "CrawlGenerator" module)
 *</ul>
 *
 *\section portsc_sec Ports Created
 *
 * </ul>
 * Output ports\n
 * <ul>
 * <li> For each \a part of the robot, a corresponding port /part/parameters/out sends 2*ndofs+1 doubles (where ndofs is the number of dofs of \a part)
 * <ul> 
 * <li> the amplitude of the movement (1 if crawling; -5 otherwise) and the target position for each dof
 * <li> the frequency
 * </ul>
 *</ul>
 *
 *\section conf_file_sec Configuration Files
 *
 *This module requires a config file :
 *<ul>
 *<li> managerConfig.ini
 *</ul>
 *
 * It can be found at \in app/Crawling/config/managerConfig.ini  
 *
 *
 *\section tested_os_sec Tested OS
 *
 * This module has been tested on Linux and Windows. 
 *
 *\section example_sec Example Instantiation of the Module
 *
 * ./CrawlManager

 * This file can be edited at \in src/crawling/CrawlManager/main.cpp 
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

#include "CrawlManagerModule.h"

using namespace yarp::os;
using namespace yarp::dev;


int main(int argc, char *argv[])
{
    Network yarp;

    //create and run the Crawl generator module
    CrawlManagerModule mod;
  
    return mod.runModule(argc,argv);

}

