/** 
 * @ingroup icub_module
 *
 * \defgroup icub_logPolarTransform logPolarTransform
 *
 * Perform a log-polar transform on an input image and generate a transformed output image.
 * The direction of the transform, from Cartesian to log-polar or vice versa, is specified by a flag in the configuration file.
 * The default direction is Cartesian to log-polar.
 * The parameters of the transform, i.e. the number of angles, the number of rings, and the overlap of the receptive fields are specified in the configuration file.
 * The default number of angles and rings is 252 and 152, respectively.  The default overlap is 0.5.
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
 * - from logPolarTransform.ini       
 *   specifies the configuration file
 *
 * - context LogPolarTransform/conf   
 *   specifies the sub-path from $ICUB_ROOT/icub/app to the configuration file
 *
 * - name LogPolarTransform          
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * Configuration File Parameters
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - imageInputPort /image:i     
 *   specifies the input port name (this string will be prefixed by /LogPolarTransform/ 
 *   or whatever else is specifed by the name parameter
 *
 * - imageOutputPort /image:o     
 *   specifies the input port name (this string will be prefixed by /LogPolarTransform/ 
 *   or whatever else is specifed by the name parameter
 *
 * - direction CARTESIAN2LOGPOLAR
 *   specifies the direction of the tranform; the alternative direction is LOGPOLAR2CARTESIAN
 *   
 * - angles 252             
 *   specifies the number of receptive fields per ring (i.e. the number of samples in the theta/angular dimension); 
 *   required for CARTESIAN2LOGPOLAR transform direction
 *
 * - rings 152             
 *   specifies the number of rings (i.e. the number of samples in the r dimension);
 *   required for CARTESIAN2LOGPOLAR transform direction
 *
 * - xsize 320             
 *   specifies the number of samples in the X dimension; 
 *   required for LOGPOLAR2CARTESIAN transform direction
 *
 * - ysize 240             
 *   specifies the number samples in the Y dimension; 
 *   required for LOGPOLAR2CARTESIAN transform direction
 *
 * - overlap 0.5             
 *   specifies the relative overlap of each receptive field
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
 *  - /logPolarTransform
 *    This port is used to change the parameters of the module at run time or stop the module
 *    The following commands are available
 * 
 *    help
 *    quit
 *    set overlap <n>   ... set the overlap of the receptive fields
 *    (where <n> is an real number)
 *
 *    Note that the name of this port mirrors whatever is provided by the --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: yarp rpc /LogPolarTransform
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - /logPolarTransform/image:i
 *
 * Output ports
 *
 *  - /logPolarTransform
 *    see above
 *
 *  - /logPolarTransform/image:o
 *
 * Port types 
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * - BufferedPort<ImageOf<PixelRgb> >   imageInputPort;     
 * - BufferedPort<ImageOf<PixelRgb> >   imageOutputPort;       
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
 * logPolarTransform.ini  in $ICUB_ROOT/icub/app/logPolarTransform/conf
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * logPolarTransform --name logPolarTransform --context logPolarTransform/conf --from lp2cart.ini  
 * logPolarTransform --name logPolarTransform --context logPolarTransform/conf --from cart2lp.ini  
 *
 * \author David Vernon
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at src/logPolarTransform/src/logPolarTransform.h
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
 * 20/09/09  Began development  DV
 */ 


#ifndef __ICUB_LOGPOLARTRANSFORM_MODULE_H__
#define __ICUB_LOGPOLARTRANSFORM_MODULE_H__

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

/* fourierVision includes */

#include "iCub/fourierVision.h"


/* Log-Polar includes */

#include "iCub/RC_DIST_FB_logpolar_mapper.h"




class LogPolarTransformThread : public Thread
{
private:

   /* class variables */

   int debug;
   unsigned char pixel_value;
   int width, height, depth;
   int x, y;
   PixelRgb rgbPixel;
   ImageOf<PixelRgb> *image;
   DVimage *cartesian;
   DVimage *logpolar;
  	    
   /* thread parameters: they are pointers so that they refer to the original variables in LogPolarTransform */

   BufferedPort<ImageOf<PixelRgb>> *imagePortIn;
   BufferedPort<ImageOf<PixelRgb>> *imagePortOut;   
   int *directionValue;     
   int *anglesValue;     
   int *ringsValue;   
   int *xSizeValue;
   int *ySizeValue;
   double *overlapValue;     

public:

   /* class methods */

   LogPolarTransformThread(BufferedPort<ImageOf<PixelRgb>> *imageIn,  BufferedPort<ImageOf<PixelRgb>> *imageOut, 
                           int *direction, int *x, int *y, int *angles, int  *rings, double *overlap);
   bool threadInit();     
   void threadRelease();
   void run(); 
};


class LogPolarTransform:public RFModule
{

   int debug;

   /* module parameters */

   string moduleName;
   string robotPortName;  
   string inputPortName;
   string outputPortName;  
   string handlerPortName;
   string transformDirection;
   int    direction;                // direction of transform
   int    numberOfAngles;           // theta samples
   int    numberOfRings;            // r samples
   int    xSize;                    // x samples
   int    ySize;                    // y samples
   double  overlap;                 // overlap of receptive fields

   /* class variables */

   BufferedPort<ImageOf<PixelRgb> > imageIn;      // input port
   BufferedPort<ImageOf<PixelRgb> > imageOut;     // output port
   Port handlerPort;                              //a port to handle messages 

   /* pointer to a new thread to be created and started in configure() and stopped in close() */

   LogPolarTransformThread *logPolarTransformThread;


public:
   
   bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
   bool interruptModule();                       // interrupt, e.g., the ports 
   bool close();                                 // close and shut down the module
   bool respond(const Bottle& command, Bottle& reply);
   double getPeriod(); 
   bool updateModule();
};


#endif // __ICUB_LOGPOLARTRANSFORM_MODULE_H__
//empty line to make gcc happy

