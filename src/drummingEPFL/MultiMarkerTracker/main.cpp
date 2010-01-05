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
 * This module tracks and outputs the 3D position of one or multiple ARToolkit markers in an image.
 * It outputs the Artoolkit ID of the marker the 3D position in the reference frame of the eye, 
 * and the 3D position in the root reference frame of the robot.
 *
 *\section lib_sec Libraries
 *
 * This module requires :
 * the Artoolkit Plus library to be installed.
 * the iKinFwd library
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
 * <li> all ports of the iCub Interface for the head and the torso should be running for to compute the 
 * position of the markers in the root refrence frame with forward kinematics.
 * </ul>
 *
 *\section portsc_sec Ports Created
 *
 * </ul>
 * Input ports\n
 * <ul>
 * <li> /MultiMarkerTracker/image to receive images
 *</ul>
 * Output ports\n
 * <ul>
 * <li> /MultiMarkerTracker/EyePos to output the position in the eye reference frame
 * <li> /MultiMarkerTracker/RootPos to output the position in the root reference frame
 * <li> /MultiMarkerTracker/view to display the camera image and a cross on the detected patches in a yarpview
 * </ul>
 *
 *\section conf_file_sec Configuration Files
 *
 *This module requires a config file :
 *<ul>
 *<li> config.ini
 *</ul>
 *
 * It can be found at \in $ICUB_ROOT/app/Crawling/config/tracker/config.ini 
 *
 *
 *\section tested_os_sec Tested OS
 *
 * This module has been tested on Linux and Windows. 
 *
 *\section example_sec Example Instantiation of the Module
 *
 * ReachManager --file "C:\iCub\app\Crawling\config\tracker\config.ini"

 * This file can be edited at \in src/crawling/MultiMarkerTracker/main.cpp 
 *
 *\authors Sebastien GAY (inspired by the code of the artkpTrackSingleMarker module of Alexandre Bernardino)
 *
 **/
#include <yarp/os/Network.h>
using namespace yarp::os;

#include "MultiMarkerTracker.h"

int main(int argc, char *argv[]) 
{

    Network yarp;
    MultiMarkerTracker module;
    module.setName("/MultiMarkerTracker"); // set default name of module
    return module.runModule(argc,argv);
}