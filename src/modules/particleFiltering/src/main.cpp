/** 
 * @ingroup icub_module
 *
 * \defgroup icub_particleFiltering particleModule
 *
This module expects a template from the following port /particleMod/template/image:i in order to commence tracking. This is a simple single-object tracker that uses a color histogram-based observation model. 
Particle filtering is a Monte Carlo sampling approach to Bayesian filtering.The particle filtering algorithm maintains a probability distribution over the state of the system it is monitoring, in this case, the state -- location, scale, etc. -- of the object being tracked. Particle filtering represents the distribution as a set of weighted samples, or particles. Each particle describes one possible location of the object being tracked. The set of particles contains more weight at locations where the object being tracked is more likely to be. The most probable state of the object is determined by finding the location in the particle filtering distribution with the highest weight.
 *
 * 
 * \section lib_sec Libraries
 *
 * YARP
 * OPENCV
 *
 * \section parameters_sec Parameters
 * 
 * Command-line Parameters
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c particleMod.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c particleFiltering/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c particleMod \n   
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * <b>Configuration File Parameters </b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - \c inputPortNameTemp \c /particleMod/template/image:i \n    
 *   specifies the input port name (this string will be prefixed by \c /particleMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c inputPortNameLeft \c /particleMod/left/image:i \n    
 *   specifies the input port name (this string will be prefixed by \c /particleMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c inputPortNameRight \c /particleMod/right/image:i \n    
 *   specifies the input port name (this string will be prefixed by \c /particleMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c outputPortNameLeft \c /particleMod/left/image:o \n  
 *   specifies the output port name (this string will be prefixed by \c /particleMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c outputPortNameRight \c /particleMod/right/image:o \n  
 *   specifies the output port name (this string will be prefixed by \c /particleMod 
 *   or whatever else is specifed by the name parameter
 *
 *   \c outputPortNameLeftBlob \c /particleMod/leftblob/image:o \n  
 *   specifies the output port name (this string will be prefixed by \c /particleMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c outputPortNameLeftBlob \c /particleMod/rightblob/image:o \n  
 *   specifies the output port name (this string will be prefixed by \c /particleMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c outputPortNameTarget \c /particleMod/target:o \n  
 *   specifies the output port name (this string will be prefixed by \c /particleMod 
 *   or whatever else is specifed by the name parameter
 *
 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                      
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *  - \c /particleMod \n
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *    \c help \n
 *    \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /yuvProc
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /particleMod/template/image:i \n
 *  - \c /particleMod/left/image:i \n
 *  - \c /particleMod/right/image:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /particleMod \n
 *    see above
 *
 *  - \c /particleMod/left/image:o \n
 *  - \c /particleMod/right/image:o \n
 *  - \c /particleMod/leftblob/image:o \n
 *  - \c /particleMod/rightblob/image:o \n
 *  - \c /particleMod/target:o \n
 *
 * <b>Port types </b>
 *
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
 * \c particleMod.ini  in \c $ICUB_ROOT/app/particleFiltering/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux: Ubuntu 9.10 and Debian Stable 
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>particleMod --name particleFiltering --context particleFiltering/conf --from particleMod.ini </tt>
 *
 * \author 
 * 
 * Vadim Tikhanoff
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at \c $ICUB_ROOT/main/src/modules/particleMod/src/main.cpp
 * 
 */


/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff
 * email:   vadim.tikhanoff@iit.it
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
  

#include "iCub/particleMod.h" 

using namespace yarp::os;

int main(int argc, char * argv[])
{
    /* initialize yarp network */ 
    //Network yarp;
    Network::init();
    
    /* create the module */
    particleMod module; 

    /* prepare and configure the resource finder */
    ResourceFinder rf;
    rf.setVerbose( true );
    rf.setDefaultConfigFile( "particleMod.ini" ); //overridden by --from parameter
    rf.setDefaultContext( "particleMod/conf" );   //overridden by --context parameter
    rf.configure( "ICUB_ROOT", argc, argv );
 
    /* run the module: runModule() calls configure first and, if successful, it then runs */
    module.runModule(rf);
    Network::fini();

    return 0;
}
