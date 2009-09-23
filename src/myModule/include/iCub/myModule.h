/** 
 * @ingroup icub_module
 *
 * \defgroup icub_myModule myModule
 *
 * This is a simple example to illustrate a module that is compliant with iCub Software Standards, addressing:
 *
 * - configuration
 * - graceful shut-down
 * - thread-based execution
 * - run-time user interaction
 * - documentation and coding standards
 *
 * Functionally, the module just converts an input image to a binary image based on the supplied threshold
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
 * Command-line Parameters
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing -- to the key 
 * (e.g. --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - from myModule.ini       
 *   specifies the configuration file
 *
 * - context myModule/conf   
 *   specifies the sub-path from $ICUB_ROOT/icub/app to the configuration file
 *
 * - name myModule          
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - robot icub             
 *   specifies the name of the robot (used to form the root of robot port names)
 *
 *
 * Configuration File Parameters
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - myInputPort /image:i     
 *   specifies the input port name (this string will be prefixed by /myModule/ 
 *   or whatever else is specifed by the name parameter
 *
 * - myOutputPort /image:o     
 *   specifies the output port name (this string will be prefixed by /myModule/ 
 *   or whatever else is specifed by the name parameter
 *
 * - cameraConfig icubEyes.ini
 *   specifies the camera configuration file containing the intrinsic parameters of
 *   the left and right cameras
 *
 * - threshold 7             
 *   specifies the threshold value
 *
 * 
 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                      
 * \section portsc_sec Ports Created
 *
 *  Input ports
 *
 *  - /myModule
 *    This port is used to change the parameters of the module at run time or stop the module
 *    The following commands are available
 * 
 *    help
 *    quit
 *    set thr <n>   ... set the threshold for binary segmentation of the input RGB image
 *    (where <n> is an integer number)
 *
 *    Note that the name of this port mirrors whatever is provided by the --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: yarp rpc /myModule
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - /myModule/image:i
 *
 * Output ports
 *
 *  - /myModule
 *    see above
 *
 *  - /myModule/image:o
 *
 * Port types 
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * - BufferedPort<ImageOf<PixelRgb> >   myInputPort;     
 * - BufferedPort<ImageOf<PixelRgb> >   myOutputPort;       
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
 * myModule.ini  in $ICUB_ROOT/icub/app/myModule/conf
 * icubEyes.ini  in $ICUB_ROOT/icub/app/myModule/conf
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * myModule --name myModule --context myModule/conf --from myModule.ini --robot icub
 *
 * \author David Vernon
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at src/myModule/src/myModule.h
 * 
 */


/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: David Vernon
 * email:   david@vernon.eu
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


/*
 * Audit Trail
 * -----------
 * 26/08/09  First version validated   DV
 * 12/09/09  Added functionality to read additional configuration file DV
 * 21/09/09  Removed code to replace a double / ("//") in a path as this is now done in getName() DV
 */ 


#ifndef __ICUB_MYMODULE_MODULE_H__
#define __ICUB_MYMODULE_MODULE_H__

#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
 
using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
  
class MyThread : public Thread
{
private:

   /* class variables */

   int      x, y;
   PixelRgb rgbPixel;
   ImageOf<PixelRgb> *image;
  	    
   /* thread parameters: they are pointers so that they refer to the original variables in myModule */

   BufferedPort<ImageOf<PixelRgb>> *imagePortIn;
   BufferedPort<ImageOf<PixelRgb>> *imagePortOut;   
   int *thresholdValue;     

public:

   /* class methods */

   MyThread(BufferedPort<ImageOf<PixelRgb>> *imageIn,  BufferedPort<ImageOf<PixelRgb>> *imageOut, int *threshold );
   bool threadInit();     
   void threadRelease();
   void run(); 
};


class MyModule:public RFModule
{
   /* module parameters */

   string moduleName;
   string robotName; 
   string robotPortName;  
   string inputPortName;
   string outputPortName;  
   string handlerPortName;
   string cameraConfigFilename;
   float  fxLeft,  fyLeft;          // focal length
   float  fxRight, fyRight;         // focal length
   float  cxLeft,  cyLeft;          // coordinates of the principal point
   float  cxRight, cyRight;         // coordinates of the principal point
   int thresholdValue;

   /* class variables */

   BufferedPort<ImageOf<PixelRgb> > imageIn;      //example input port
   BufferedPort<ImageOf<PixelRgb> > imageOut;     //example output port
   Port handlerPort;      //a port to handle messages 

   /* pointer to a new thread to be created and started in configure() and stopped in close() */

   MyThread *myThread;


public:
   
   bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
   bool interruptModule();                       // interrupt, e.g., the ports 
   bool close();                                 // close and shut down the module
   bool respond(const Bottle& command, Bottle& reply);
   double getPeriod(); 
   bool updateModule();
};


#endif // __ICUB_MYMODULE_MODULE_H__
//empty line to make gcc happy

