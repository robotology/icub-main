// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _VISUAL_FILTER_MODULE_H_
#define _VISUAL_FILTER_MODULE_H_

/** 
 * @ingroup icub_module
 *
 * \defgroup icub_visualFilter visualFilter
 *
 * This is a module that correctly applies filters to the input logpolar image:
 *
 * - extract colour oppenency maps based on RGB colours
 * - extract edges based on sobel operator
 * - 
 * - 
 * 
 *
 * A complete tutorial for this example is available on the iCub wiki at 
 * http://eris.liralab.it/wiki/Summary_of_iCub_Software_Development_Guidelines
 *
 * 
 * \section lib_sec Libraries
 *
 * YARP.
 *
 * \section parameters_sec Parameters
 * 
 * <b>Command-line Parameters <\b> 
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c visualFilter.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c visualFilter/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c visualFilter \n 
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - \c robot \c icub \n 
 *   specifies the name of the robot (used to form the root of robot port names)
 *
 *
 * <b>Configuration File Parameters </b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 *
 * 
 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                      
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *  - \c /visualFilter \n
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *    \c help \n
 *    \c quit \n
 *    \c set \c thr \c <n>   ... set the threshold for binary segmentation of the input RGB image 
 *    (where \c <n> is an integer number)
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /visualFilter
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /visualFilter/image:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /visualFilter \n
 *    see above
 *
 *  - \c /visualFilter/image:o \n
 *
 * <b>Port types </b>
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c BufferedPort<ImageOf<PixelRgb> >   \c myInputPort; \n 
 * \c BufferedPort<ImageOf<PixelRgb> >   \c myOutputPort;       
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
 * \c visualFilter.ini  in \c $ICUB_ROOT/app/visualFilter/conf \n
 * \c icubEyes.ini  in \c $ICUB_ROOT/app/visualFilter/conf
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>visualFilter --name visualFilter --context visualFilter/conf --from visualFilter.ini --robot icub</tt>
 *
 * \author 
 * 
 * Rea Francesco
 * 
 * Copyright (C) 2010 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at \c $ICUB_ROOT/src/visualFilter/include/iCub/visualFilter.h
 * 
 */


/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
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


#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
 
//within project includes  
#include <iCub/visualFilterThread.h>

class visualFilterModule:public yarp::os::RFModule
{
   /* module parameters */

   std::string moduleName;
   std::string robotName; 
   std::string robotPortName;  
   std::string inputPortName;
   std::string outputPortName;  
   std::string handlerPortName;
   std::string cameraConfigFilename;
   float  fxLeft,  fyLeft;          // focal length
   float  fxRight, fyRight;         // focal length
   float  cxLeft,  cyLeft;          // coordinates of the principal point
   float  cxRight, cyRight;         // coordinates of the principal point
   int thresholdValue;

   /* class variables */

  
   yarp::os::Port handlerPort;      //a port to handle messages 

   /* pointer to a new thread to be created and started in configure() and stopped in close() */

   visualFilterThread *vfThread;


public:
   
   bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
   bool interruptModule();                       // interrupt, e.g., the ports 
   bool close();                                 // close and shut down the module
   bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
   double getPeriod(); 
   bool updateModule();
};


#endif // __VISUAL_FILTER_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

