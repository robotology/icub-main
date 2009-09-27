/** 
 * @ingroup icub_module
 *
 * \defgroup icub_imageSource imageSource 
 *
 * Read an image from a file and stream it to a specified port.
 * Add random noise to differentiate between each image streamed.
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
 * - from imageSource.ini       
 *   specifies the configuration file
 *
 * - context imageSource/conf   
 *   specifies the sub-path from $ICUB_ROOT/icub/app to the configuration file
 *
 * - name imageSource          
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 *
 * Configuration File Parameters
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - outputPort /image:o     
 *   specifies the complete output port name to which the image should be streamed
 *
 * - imageFile image.ppm
 *   specifies the image filename
 *
 * - frequency 10             
 *   specifies the number of images to be streamed per second.  
 *   A low frequency avoids this module hogging the CPU; this is important if you are streaming several images
 *
 * - width 320             
 *   specifies the width of the image to be streamed.  
 *   The image read from the file is rescaled if necessary.
 *
 * - height 240             
 *   specifies the height of the image to be streamed.  
 *   The image read from the file is rescaled if necessary.
 *
 * - noise 20
 *   specifies the random noise level
 * 
 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                      
 * \section portsc_sec Ports Created
 *
 *  Input ports
 *
 *  - /imageSource
 *    This port is used to change the parameters of the module at run time or stop the module
 *    The following commands are available
 * 
 *    help
 *    quit
 *    set noise <n>   ... set the random noise level
 *    (where <n> is an integer number in the range 0-255)
 *
 *    Note that the name of this port mirrors whatever is provided by the --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: yarp rpc /imageSource
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *
 * Output ports
 *
 *  - /imageSource
 *    see above
 *
 *  - /image:o
 *
 * Port types 
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * - BufferedPort<ImageOf<PixelRgb> >   outputPort;       
 *
 * \section in_files_sec Input Data Files
 *
 * image.ppm, or whatever is specified as a argument for the --imageFile key-value. 
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * imageSource.ini  in $ICUB_ROOT/icub/app/imageSource/conf
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * imageSource --name imageSource --context imageSource/conf --from imageSource.ini 
 *
 * \author David Vernon
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at src/imageSource/src/imageSource.h
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
 * 22/09/09  First version validated   DV
 */ 


#ifndef __ICUB_IMAGESOURCE_MODULE_H__
#define __ICUB_IMAGESOURCE_MODULE_H__

#include <iostream>
#include <string>
#include <cstdlib>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
 
using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
using namespace yarp::sig::file;
  


class ImageSourceThread : public RateThread
{
private:

   /* class variables */

   int      x, y;
   PixelRgb rgbPixel;
   ImageOf<PixelRgb> inputImage;
   int input_width;
   int input_height;
   int input_depth;
   bool debug;

   /* thread parameters: they are pointers so that they refer to the original variables in imageSource */

   BufferedPort<ImageOf<PixelRgb> > *imagePortOut;   
   int *widthValue;     
   int *heightValue;     
   int *noiseValue;  
   string *imageFilenameValue;  

public:

   /* class methods */

   ImageSourceThread(BufferedPort<ImageOf<PixelRgb> > *imageOut, string *imageFilename,  int period, int *width, int *height, int *noise);
   bool threadInit();     
   void threadRelease();
   void run(); 
};


class ImageSource:public RFModule
{

   /* class variables */

   bool debug; 

   /* module parameters */

   string moduleName;
   string outputPortName;  
   string handlerPortName;
   string imageFilename;
   int    frequency;
   int    width;
   int    height;
   int    noiseLevel;

   /* class variables */

   BufferedPort<ImageOf<PixelRgb> > imageOut;     // output port
   Port handlerPort;                              // a port to handle messages 

   /* pointer to a new thread to be created and started in configure() and stopped in close() */

   ImageSourceThread *imageSourceThread;

public:
   
   bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
   bool interruptModule();                       // interrupt, e.g., the ports 
   bool close();                                 // close and shut down the module
   bool respond(const Bottle& command, Bottle& reply);
   double getPeriod(); 
   bool updateModule();
};


#endif // __ICUB_IMAGESOURCE_MODULE_H__
//empty line to make gcc happy

