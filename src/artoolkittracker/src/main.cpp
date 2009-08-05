// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/* 
 * Copyright (C) 2008 Manuel Lopes, Alexandre Bernardino - vislab.isr.ist.utl.pt
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   macl@isr.ist.utl.pt
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
 * \defgroup icub_artoolkittracker artoolkittracker
 *
 *This module is part of the application \ref demoaff "demoaff"
 *
 *\section intro_sec Description 
 *
 * This module is a wrapper of the ARTOOLKIT tracker (http://www.hitl.washington.edu/artoolkit/). It detects and tracks specially designed square markers.
 *
 *\section lib_sec Libraries
 *
 * artoolkit
 * http://www.hitl.washington.edu/artoolkit/
 *
 *\section parameters_sec Parameters
 *
 *
 *\section portssa_sec Ports Accessed 
 *
 * Input ports\n
 * <ul>
 * <li> image from a camera (calibrated or not) 
 *</ul>
 *
 * Output ports\n
 * <ul>
 * <li> a port /interactive/out that sends information about the detected markers with the following format
               <name> "T" <tranformation matrix>[12] "imgcoord" x y "size" size "imgcoordnorm" nx ny as many repetitions as detected markers
 * </ul>
 *  
 *
 *\section portsc_sec Ports Created
 *
 * Input ports\n
 * <ul>
 * <li> A port /artoolkittracker/image receives input images
 * </ul>
 * Output ports\n
 * <ul>
 * <li> /artoolkittracker/out
 *</ul>
 *
 *\section conf_file_sec Configuration Files
 *
 * This module requires:
 *<ul>
 *<li> options: $ICUB_ROOT/conf/iCubARMarkerDetectorModule.ini including number of markers and name of the files
 *<li> $ICUB_ROOT/src/artoolkittracker/data/camera_para.dat necessary to be present. if calibrated artoolkit returns 3D estimation of pose
 *<li>$ICUB_ROOT/src/artoolkittracker/data/object_data.txt information about the markers, their file locations
 *</ul>
 *
 * Examples of such files can be found at \in /src/artoolkittracker/data and /conf/
 *
 *
 *\section tested_os_sec Tested OS
 *
 * This module has been tested on Linux and Windows. 
 *
 *\section example_sec Example Instantiation of the Module
 *
 * ./artoolkittracker --file $ICUB_ROOT/conf/iCubARMarkerDetectorModule.ini
 *
 * This file can be edited at \in src/artoolkittracker/src
 *
 *\authors Manuel Lopes and Alexandre Bernardino
 *
 **/


// yarp

#include <yarp/os/Network.h>

#include <yarp/os/Module.h>



#include "arMarkerDetectorModule.h"



using namespace std;

using namespace yarp::os;





int main(int argc, char *argv[]) {



    Network yarp;

    ARMarkerDetectorModule module;

    module.setName("/artracker"); // set default name of module

    return module.runModule(argc,argv);

}

