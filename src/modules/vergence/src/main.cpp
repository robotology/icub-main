// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/** 
 * @ingroup icub_module
 *
 * \defgroup icub_vergence vergence
 *
 * Receives two logPolar images (left and right) and determines the appropriate veregence angle to apply. The module can control directly the robots by using the ikinGazeCtrl interface 
 * or by using the arbitrer created for the logpolar attention system
 * 
 * \section lib_sec Libraries
 *
 * YARP
 *
 * \section parameters_sec Parameters
 * 
 * Command-line Parameters
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c veregence.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c vergence/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/main/app to the configuration file
 *
 * - \c name \c vergence \n   
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - \c robot \c icub or icubSim\n   
 *   specifies the type of the module to work on (yuv or hsv)  
 * 
 * - \c ctrl \c gazeCtrl or arbitrer\n   
 *   specifies the period of the module's thread  
 *
 * Configuration File Parameters
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - \c imageInLeft \c /left:i \n    
 *   specifies the input port name (this string will be prefixed by \c /vergence 
 *   or whatever else is specifed by the name parameter
 *
 * - \c imageInRight \c /right:i \n  
*   specifies the input port name (this string will be prefixed by \c /vergence 
 *   or whatever else is specifed by the name parameter
 *
 * - \c histoOutPort \c /histo:o \n  
 *   specifies the output port name when histo is selected(this string will be prefixed by \c /vergence 
 *   or whatever else is specifed by the name parameter
 *
 * - \c cmdOutput \c /comd:o \n  
 *   specifies the output port name when command is selected(this string will be prefixed by \c /vergence 
 *   or whatever else is specifed by the name parameter
 *
 * - \c shiftOutput \c /shift:o \n  
 *   specifies the output port name when shift is selected(this string will be prefixed by \c /vergence 
 *   or whatever else is specifed by the name parameter
 *
 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                      
 * \section portsc_sec Ports Created
 *
 * Input ports
 *
 *  - \c /vergence \n 
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *    \c help \n
 *    \c quit \n
 *    \c ctrl \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /vergence
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /vergence/left:i \n
 *  - \c /vergence/right:i \n
 *
 * Output ports
 *
 *  - \c /vergence \n
 *    see above
 *
 *  - \c /vergence/histo:o when histo is selected\n
 *  - \c /vergence/cmd:o when command is selected\n
 *  - \c /yuvProc/shift:o when shift is selected\n
 *
 * Port types
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c BufferedPort<ImageOf<PixelRgb> >   \c  imageInLeft; \n 
 * \c BufferedPort<ImageOf<PixelRgb> >   \c imageInRight;   
 * \c BufferedPort<ImageOf<PixelMono> >   \c histoOutPort;   
 * \c Port   \c cmdOutput; 
 * \c Port   \c shiftOutput;
 *
 * \section in_files_sec Input Data Files
 *
 * None
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * \c vergence.ini  in \c $ICUB_ROOT/main/app/vergence/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux: Ubuntu 9.10, Debian Stable and windows 
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>vergence --name vergence --robot icub (or icubSim) --ctrl gazeCtlr (or arbitrer) --context vergence/conf --from vergence.ini </tt>
 *
 * \author 
 * 
 * Vadim Tikhanoff, Francesco Rea
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at \c $ICUB_ROOT/main/src/modules/vergence/src/main.cpp
 * 
 */

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea, Vadim Tikhanoff
 * email:   francesco.rea@iit.it
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

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <iCub/disparityModule.h>
#include <string.h>

using namespace yarp::os;
using namespace std;

YARP_DECLARE_DEVICES(icubmod)

int main(int argc, char *argv[]) 
{
    /* initialize yarp network */ 
    Network yarp;

    YARP_REGISTER_DEVICES(icubmod)
    /* create the module */
    disparityModule module; 

    /* prepare and configure the resource finder */
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("attentionMechanism.ini"); //overridden by --from parameter
    rf.setDefaultContext("attentionMechanism/conf");   //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);
     
    module.runModule(rf);
}
