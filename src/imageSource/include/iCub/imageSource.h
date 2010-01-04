/** 
 * @ingroup icub_module
 *
 * \defgroup icub_imageSource imageSource 
 *
 * Read an image from a file and stream it with specified dimensions to a specified port.
 * Add random noise to differentiate between each image streamed.
 *
 * If the window key-value pair is set to 1, and provided the dimensions of the image read from file is 
 * larger than the dimensions of the image to be streamed (as specified in the width and height key-value pairs), 
 * then we extract a sub-image of the required dimensions.  
 * The scan pattern is random if the random key-value pair is set to 1; otherwise it is a regular scan pattern in
 * row major order, with x and y increment equal to the window dimensions so that the window scans the complete image 
 * (except for borders at the right-hand side and bottom).
 * In either case (random or regular scan), the module also writes the gaze angles to an output port, 
 * simulating the functionality of the attentionSelection module.
 * It also writes encoder values for the head, simulating the functionality of the /icub/head/state:o port on the iCub.
 *
 * 
 * \section lib_sec Libraries
 *
 * YARP.
 *
 * \section parameters_sec Parameters
 * 
 * <b>Command-line Parameters</b>
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing -- to the key 
 * (e.g. \c --from file.ini . The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c imageSource.ini \n     
 *   The configuration file
 *
 * - \c context \c imageSource/conf \n
 *   The sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c imageSource \n
 *   The name of the module (used to form the stem of module port names)  
 *
 *
 * <b>Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - \c outputPort \c /image:o  \n  
 *   The complete output port name to which the image should be streamed
 *
 * - \c gazePort \c /gaze:o  \n  
 *   The complete output port name to which the gaze angles should be streamed
 *
 * - \c encoderPort \c /icub/head/state:o   \n  
 *   The complete output port name to which the gaze angles should be streamed
 *
 * - \c imageFile \c image.ppm \n
 *   specifies the image filename
 *
 * - \c frequency \c 10 \n         
 *   specifies the number of images to be streamed per second.  
 *   A low frequency avoids this module hogging the CPU; this is important if you are streaming several images
 *
 * - \c width \c 320 \n           
 *   specifies the width of the image to be streamed.  
 *   The image read from the file is rescaled if necessary.
 *
 * - \c height \c 240 \n           
 *   specifies the height of the image to be streamed.  
 *   The image read from the file is rescaled if necessary.
 *
 * - \c noise \c 20 \n
 *   specifies the random noise level
 *
 * - \c window \c 0 \n
 *   specifies whether or not to extract a sub-image (default 0 is not to do so, in which case the image is scaled)
 * 
 * - \c random \c 0 \n
 *   specifies the scan pattern when extracting a sub-image (default 0 is regular scan, row major order)
 * 
 * - \c horizontalViewAngle \c 120.0 \n
 *   specifies the horizontal field of view in degrees
 * 
 * - \c verticalViewAngle \c 90.0 \n
 *   specifies the vertical field of view in degrees
 * 
 * Note: neither of the port names above (/image:o or /gaze:o) will be prefixed with module name (i.e. /imageSource)
 * since the purpose of this module is to simulate the output ports of other modules (e.g. /icub/cam/left or /attentionSelection/o:position)
 * 
 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                      
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *  - \c /imageSource \n
 *    This port is used to change the parameters of the module at run time or stop the module.
 *    The following commands are available
 * 
 *    help \n
 *    quit \n
 *    set noise <n>   ... set the random noise level
 *    (where <n> is an integer number in the range 0-255).
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /imageSource
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *
 * <b>Output ports</b>
 *
 *  - \c /imageSource \n
 *    see above
 *
 *  - \c /image:o
 *
 *  - \c /gaze:o \n
 * 
 *  - \c  /icub/head/state:o  \n
 *
 * <b>Port types </b>
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * - \c BufferedPort<ImageOf<PixelRgb> >   \c outputPort;      
 * - \c BufferedPort<VectorOf<double> >    \c gazeOutPort;     <tt>//double azimuth, elevation, 'a', 0, 0 </tt>
 * - \c BufferedPort<VectorOf<double> >    \c encoderOutPort;  <tt>//double 0, 1, 2, 3, 4, 5 </tt>
 * 
 * Note that the protocol used for the gazeOutPort is the same as that used by the attentionSelection module
 * when controlling the controlGaze2 module using the /pos port.
 *
 * \section in_files_sec Input Data Files
 *
 * image.ppm, or whatever is specified as a argument for the \c --imageFile \c key-value. 
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * \c imageSource.ini  in \c $ICUB_ROOT/app/imageSource/conf
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>imageSource --name imageSource --context imageSource/conf --from imageSource.ini </tt>
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
 * 20/11/09  Added windowing and gaze functionality   DV
 * 04/01/10  Added encoder functionality   DV
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
   int xOffset;
   int yOffset;
   bool debug;
   bool windowFlag;

   /* thread parameters: they are pointers so that they refer to the original variables in imageSource */

   BufferedPort<ImageOf<PixelRgb> > *imagePortOut;   
   BufferedPort<VectorOf<double> >  *gazePortOut;   
   BufferedPort<VectorOf<double> >  *encoderPortOut;   
   int *widthValue;     
   int *heightValue;     
   int *noiseValue; 
   int *windowValue; 
   int *randomValue; 
   double *horizontalViewAngleValue;
   double *verticalViewAngleValue;

   string *imageFilenameValue;  

public:

   /* class methods */

   ImageSourceThread(BufferedPort<ImageOf<PixelRgb> > *imageOut,  
                     BufferedPort<VectorOf<double> >  *gazePortOut,
                     BufferedPort<VectorOf<double> >  *encoderPortOut, 
                     string *imageFilename,  int period, int *width, int *height, int *noise, int *window, int *random,
                     double *horizontalViewAngle, double *verticalViewAngle);
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
   string gazePortName;
   string encoderPortName;
   string handlerPortName;
   string imageFilename;
   int    frequency;
   int    width;
   int    height;
   int    noiseLevel;
   int    window;
   int    random;
   double horizontalViewAngle;
   double verticalViewAngle;

   /* class variables */

   BufferedPort<ImageOf<PixelRgb> > imageOut;     // output port
   BufferedPort<VectorOf<double> >  gazeOut;      // gaze port
   BufferedPort<VectorOf<double> >  encoderOut;   // encoder port
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

