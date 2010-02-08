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
 * \defgroup icub_CrawlGenerator CrawlGenerator
 *
 * This module is part of the application \ref icub_Crawling "Crawling"
 *
 *\section intro_sec Description 
 *
 * This module transforms a set of parameters for the dynamical systems into target trajectories for a specific interface of the robot (head, torso, left_arm, right_arm, left_leg, right_leg). The command are then sent to the corresponding \ref icub_velocityControl "velocityControl" module. If you want to  use the whole crawling application, please refer to \ref icub_Crawling "Crawling".
 *
 *\section lib_sec Libraries
 *
 *No external libraries. 
 *
 *
 *\section parameters_sec Parameters
 *
 * For instance for the left arm:  \n
 * --part left_arm \n
 * 
 * The same type of command can be used for right_arm, left_leg, right_leg, torso and head. 
 *
 *\section portssa_sec Ports Accessed 
 *
 * Output ports\n
 * <ul>
 * <li> for each active \a part (i.e left_arm, right_arm, left_leg, right_leg, torso, head) of the robot: /part/parameters/out (created by the \ref icub_CrawlManager "CrawlManager" module)
 *</ul>
 *  
 *
 *\section portsc_sec Ports Created
 *
 *  Input ports\n
 * <ul>
 * <li> For each \a part of the robot, a corresponding port /part/parameters/in receives from the \ref icub_CrawlManager "CrawlManager" 2*ndofs+1 doubles (where ndofs is the number of dofs of \a part):
 * <ul> 
 * <li> the amplitude of the movement (1 if crawling; -5 otherwise) and the target position for each dof
 * <li> the frequency
 * </ul>
 *</ul>
 *
 *
 *\section conf_file_sec Configuration Files
 *
 * For each active  \a part, this module requires:
 *<ul>
 *<li> partConfig.ini
 *</ul>
 *
 * Examples of such files can be found at \in app/Crawling/config/left_armConfig.ini
 *
 *
 *\section tested_os_sec Tested OS
 *
 * This module has been tested on Linux and Windows. 
 *
 *\section example_sec Example Instantiation of the Module
 *
 * ./CrawlGenerator  --part left_arm 
 *
 * This file can be edited at \in src/crawling/CrawlGenerator/main.cpp 
 *
 *\authors Sarah Degallier 
 *
 **/




#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Module.h>
#include "CrawlGeneratorModule.h"

using namespace yarp::os;
using namespace yarp::dev;

int main(int argc, char *argv[])
{
    Network yarp;

    //create and run the Drum generator module
    CrawlGeneratorModule mod;
  
    return mod.runModule(argc,argv);

}
