/**
 *
 * @ingroup icub_module
 * \defgroup icub_crossPowerSpectrumVergence crossPowerSpectrumVergence
 *
 * Compute histogram of disparity values and select the local maximum value which corresponds 
 * to the regions closest to the cameras to control vergence
 *
 *
 * \section intro_sec Description
 * Determine the relative shift required to register one or more regions 
 * in two input images using the cross-power spectrum.
 *
 * The cross-power spectrum of two images is defined as
 *
 *
 * F(w_x, w_y) G*(w_x, w_y) / (|F(w_x, w_y) G(w_x, w_y)|)
 *
 *
 * where F(w_x, w_y) and G(w_x, w_y) are the Fourier tranforms of images f(x, y) and g(x, y), 
 * and G*(w_x, w_y) is the complex conjugate of G(w_x, w_y)
 * 
 *
 * The positions of local maxima in the cross-power spectrum, 
 * specifically the offset of a detected maximum from the centre of the CPS image, 
 * indicates the relative image shift required to register regions in the image.
 * This can be used to control the vergence of the two eyes.
 *
 * Typically, there will be several regions in the image with different disparities 
 * and hence different vergence angles, each with its own corresponding maximum,
 * we need a strategy to choose one of the maxima to control the fixation/vergence
 *
 * The possibilities are: 
 *
 * 1 choose the largest maximum (number 0); 
 *   this is probably going to correspond to the object that occupies 
 *   the largest amount of the field of view (or largest energy in the image)
 *
 * 2 choose the maximum that is closest to the centre;
 *   the corresponds to the object that is closest to the current fixation distance
 *
 * 3 choose the maximum that is furthest to the LEFT of the cross-power spectrum; 
 *   this corresponds to the object that is closest to the cameras
 *
 *
 * Option 3 is the only option currently implemented.
 *
 *
 * \section lib_sec Libraries
 * 
 * YARP.
 *
 *
 * \section parameters_sec Parameters
 * 
 * Command Line Parameters 
 *
 * The following key-value pairs can be specified as command-line parameters by prefixing -- to the key 
 * (e.g. --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 * 
 * - from crossPowerSpectrumVergence.ini       specifies the configuration file
 * - context crossPowerSpectrumVergence/conf   specifies the sub-path from $ICUB_ROOT/icub/app to the configuration file
 * - name crossPowerSpectrumVergence           specifies the name of the module (used to form the stem of module port names)
 * - robot icub                                specifies the name of the robot (used to form the root of robot port names)*
 *
 *
 * Configuration File Parameters 
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 * 
 * 
 * - std_dev                         20
 *   Standard deviation of the Gaussian mask used to apodize the input images; 
 *   the apodized images are s output to /left_image:o and /right_image:o
 * - number_of_maxima                 2   
 *   Number of local maxima (i.e. image regions a given disparity or depth) to consider in the final selection (typical value 2)
 * - threshold                       20   
 *   Threshold for detection of maxima: integer % of global maximum (typical value 20)
 * - filter_radius                    2   
 *   Radius in pixels of filter used to amplify local maxima (typical value 2)
 * - non_maxima_suppression_radius    5   
 *   Radius in pixels of the non-maxima suppression filter (typical value 2)
 *
 *
 * Port names
 *
 * - left_camera                      /left_camera:i
 *   Input from the left camera
 * - right_camera                     /right_camera:i
 *   Input from the right camera
 * - left_output                      /left_image:o
 *   Output of the Gaussian-apodized image from the left camera
 * - right_output                     /right_image:o
 *   Output of the Gaussian-apodized image from the right camera
 * - cross-power_spectrum             /cross-power_spectrum:o
 *   Output of the raw cross-power spectrum image
 * - filtered_cross-power_spectrum    /filtered_cross-power_spectrum:o
 *   Output of the filtered cross-power spectrum with maxima enhancement, non-maxima suppression, and cross-hairs showing selected maxima 
 * - vergence_disparity               /vergence_disparity:o 
 *   The disparity, in normalized coordinates (-1,+1), of the object closest to the head
 *
 *
 * Port types 
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * - BufferedPort<ImageOf<PixelRgb> >   left_camera        
 * - BufferedPort<ImageOf<PixelRgb> >   right_camera
 * - BufferedPort<ImageOf<PixelRgb> >   left_output        
 * - BufferedPort<ImageOf<PixelRgb> >   right_output         
 * - BufferedPort<ImageOf<PixelRgb> >   cross-power_spectrum        
 * - BufferedPort<ImageOf<PixelRgb> >   filtered_cross-power_spectrum        
 * - BufferedPort<Vector>               vergence_disparity;         
 *         
 * 
 * \section portsa_sec Ports Accessed
 * 
 * None.
 *                      
 * \section portsc_sec Ports Created
 *
 *  Input ports
 *
 *  - /crossPowerSpectrumVergence
 *    This port is used to change the parameters of the module at run time or stop the module
 *    The following commands are available
 * 
 *    help
 *    quit
 *    set std <n>   ... set the standard deviation 
 *    set max <n>   ... set the number of maxima to detect
 *    set thr <n>   ... set the threshold for detection
 *    (where <n> is an integer number)
 *
 *    Note that the name of this port mirrors whatever is provided by the --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    You can also connect to it using yarp rpc /crossPowerSpectrumVergence 
 *
 *  - /crossPowerSpectrumVergence/left_camera:i
 *  - /crossPowerSpectrumVergence/right_camera:i
 *
 * Output ports
 *
 * - /crossPowerSpectrumVergence/crossPowerSpectrumVergence
 * - /crossPowerSpectrumVergence/left_image:o
 * - /crossPowerSpectrumVergence/right_image:o
 * - /crossPowerSpectrumVergence/cross-power_spectrum:o
 * - /crossPowerSpectrumVergence/filtered_cross-power_spectrum:o
 * - /crossPowerSpectrumVergence/vergence_disparity:o
 *
 * \section in_files_sec Input Data Files
 * None
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * crossPowerSpectrumVergence.ini
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux and Windows
 *
 * \section example_sec Example Instantiation of the Module
 *
 * crossPowerSpectrumVergence  --context crossPowerSpectrumVergence/conf  --from crossPowerSpectrumVergence.ini
 *
 * \author 
 *
 * David Vernon
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at src/crossPowerSpectrumVergence/include/iCub/crossPowerSpectrumVergence.h
 *
**/

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
 * 18/07/07  Started work on the development of a YARP version of this module   DV
 * 30/07/09  Migrated to the RFModule class to allow use of the resource finder DV
 * 17/08/09  Amended to comply with iCub Software Development Guidelines        DV
 * 24/08/09  Implemented Thread-based execution                                 DV
 * 25/08/09  Implemented run-time modification of parameters using portHandler  DV
 */ 

#ifndef __ICUB_CROSSPOWERSPECTRUMVERGENCE_MODULE_H__
#define __ICUB_CROSSPOWERSPECTRUMVERGENCE_MODULE_H__


/* System includes */

#include <assert.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/timeb.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string>
#include <iostream>

/* YARP includes */

#include <ace/config.h>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;


/* fourierVision includes */

#include "iCub/fourierVision.h"
  

class WorkThread : public Thread
{
private:

    /* class variables */
  
    int debug;

    eyecub_image   *image1;
    eyecub_image   *image2;
    eyecub_image   *image_a;
    eyecub_image   *image_b;
    eyecub_image   *image_c;
    eyecub_image   *image_d;

    int width;
    int height;
    int depth;
    int image_size;
    unsigned char pixel_value;
    float float_pixel_value;
    int temp;
    int i, j, p, q;
    int x, y;
    float sigma;
    float controlGaze_x;
    float controlGaze_y;

    PixelRgb rgb_pixel;

    ImageOf<PixelRgb> *imgIn1;
    ImageOf<PixelRgb> *imgIn2;
  
    maxima_data_type maxima[10];
 	    
    /* thread parameters: they are all pointers so that they refer to the original variables in the crossPowerSpectrumVergence */

    BufferedPort<ImageOf<PixelRgb> > *portIn1;  // ports for acquiring and sending images
    BufferedPort<ImageOf<PixelRgb> > *portIn2;  //  
    BufferedPort<ImageOf<PixelRgb> > *portOut1; //
    BufferedPort<ImageOf<PixelRgb> > *portOut2; //
    BufferedPort<ImageOf<PixelRgb> > *portOut3; //
    BufferedPort<ImageOf<PixelRgb> > *portOut4; //
    BufferedPort<Vector>             *portOut5; // port for sending servo data to controlGaze

    int *threshold;                            // % of maximum value
    int *filter_radius;                        // pixels
    int *number_of_maxima;                     // cardinal number
    int *non_maxima_suppression_radius;        // pixels
    int *std_dev;                              // % of image width

public:

    /* class methods  */

   WorkThread(BufferedPort<ImageOf<PixelRgb> > *imageIn1,  
              BufferedPort<ImageOf<PixelRgb> > *imageIn2,   
              BufferedPort<ImageOf<PixelRgb> > *imageOut1,  
              BufferedPort<ImageOf<PixelRgb> > *imageOut2, 
              BufferedPort<ImageOf<PixelRgb> > *imageOut3,  
              BufferedPort<ImageOf<PixelRgb> > *imageOut4,  
              BufferedPort<Vector>             *vergenceOut5, 
              int *thresh,                             
              int *filterRadius,                 
              int *numMaxima,
              int *suppressionRadius, 
              int *stdDev);

   bool threadInit();     
   void threadRelease();
   void run(); 
};


class CrossPowerSpectrumVergence: public RFModule

{
private:
    
    /* class variables  */
 
    int debug;

    /* ports */

    BufferedPort<ImageOf<PixelRgb> > portIn1;  // ports for acquiring and sending images
    BufferedPort<ImageOf<PixelRgb> > portIn2;  //  
    BufferedPort<ImageOf<PixelRgb> > portOut1; //
    BufferedPort<ImageOf<PixelRgb> > portOut2; //
    BufferedPort<ImageOf<PixelRgb> > portOut3; //
    BufferedPort<ImageOf<PixelRgb> > portOut4; //
    BufferedPort<Vector>             portOut5; // port for sending servo data to controlGaze
    Port handlerPort;                          // port to handle messages

    /* module parameters */

    int threshold;                            // % of maximum value
    int filter_radius;                        // pixels
    int number_of_maxima;                     // cardinal number
    int non_maxima_suppression_radius;        // pixels
    int std_dev;                              // % of image width

    /* module name */

    string moduleName;

    /* port names */

    string leftCameraPortName;
    string rightCameraPortName;
    string leftImagePortName; 
    string rightImagePortName;
    string crossPowerSpectrumPortName; 
    string filteredCrossPowerSpectrumPortName;
    string vergenceDisparityPortName; 
    string handlerPortName; 
          

    /* pointer to a new thread to be created and started in configure() and stopped in close() */

    WorkThread *crossPowerSpectrumVergenceThread;
 
 public:

    /* class methods */

    CrossPowerSpectrumVergence();
    ~CrossPowerSpectrumVergence();
    double getPeriod();
    bool respond(const Bottle& command, Bottle& reply);
    bool configure(yarp::os::ResourceFinder &rf);
    bool close();
    bool interruptModule();
    bool updateModule();
};


#endif // __ICUB_CROSSPOWERSPECTRUMVERGENCE_MODULE_H__
/* empty line to make gcc happy */


