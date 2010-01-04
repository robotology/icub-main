// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup icub_module
 * \defgroup icub_sampleVideo sampleVideo
 *
 * Sample a stream of images at a given frequency.  This may be useful in circumstances when a device, 
 * or another module streams more images per second than a down-stream module can process in time. 
 *
 * The frequency at which images are read is provided as a module parameter and is set in the sampleVideo configuration file. 
 * 
 * The sampleVideo module has the following inputs: 
 * 
 * - an input image 
 *
 * The sampleVideo has the following outputs: 
 * 
 * - an output image, copied directly from the sampled input image.
 *
 *
 * \section lib_sec Libraries
 *
 * YARP.
 *
 * \section parameters_sec Parameters
 * <b>Command Line Parameters </b> 
 *
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from \c file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 * 
 * - \c from \c sampleVideo.ini       \n     
 *   specifies the configuration file
 * 
 * - \c context \c sampleVideo/conf   \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 * 
 * - \c name \c sampleVideo           \n                              
 *   specifies the name of the module (used to form the stem of module port names)
 * 
 *
 * <b>Configuration File Parameters </b> 
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 * 
 * - \c imageInPort         \c /image:i  \n
 *   specifies the port for input of an image
 *
 * - \c imageOutPort        \c /image:o  \n
 *   specifies the image output port
 *
 * - \c frequency           \c 2     \n         
 *   specifies the sampling frequency and therefore the number of output images to be streamed per second. 
 * 
 *
 * \section portsa_sec Ports Accessed
 * 
 * None
 *                      
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b> 
 *
 * - \c /sampleVideo/image:i
 *
 * <b>Output ports</b> 
 *
 *  - \c /sampleVideo/image:o
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 *
 * <b>I/O Port Types & Naming</b> 
 *
 * - \c BufferedPort<ImageOf<PixelRgb> > \c imageInPort;
 * - \c BufferedPort<ImageOf<PixelRgb> > \c imageOutPort;
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
 * \c sampleVideo.ini (see above)
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux and Windows
 *
 * \section example_sec Example Instantiation of the Module
 *
 * \c sampleVideo \c --context \c sampleVideo/conf  \c --from sampleVideo.ini
 *
 * \author David Vernon 
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at src/sampleVideo/include/iCub/sampleVideo.h
**/
  
/*
 * Audit Trail
 * -----------
 *
 * 02/01/10  First version of the sampleVideo module
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


#ifndef __ICUB_SAMPLEVIDEO_MODULE_H__
#define __ICUB_SAMPLEVIDEO_MODULE_H__

//yarp
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/all.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>

//system
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

 
class SampleVideoThread : public RateThread
{
private:

   /* class variables */

   bool debug;

   ImageOf<PixelRgb> *imgIn;

   /* thread parameters: they are pointers so that they refer to the original variables in imageSource */

   BufferedPort<ImageOf<PixelRgb> > *imageInPort;
   BufferedPort<ImageOf<PixelRgb> > *imageOutPort;

public:

   /* class methods */

   SampleVideoThread (BufferedPort<ImageOf<PixelRgb> > *imageIn,
                      BufferedPort<ImageOf<PixelRgb> > *imageOut, 
                      int                              period);
   bool threadInit();     
   void threadRelease();
   void run(); 

};


class SampleVideo : public RFModule
{
private:
   /* class variables */

   bool debug; 

   /* port names */

   string imageInPortName;
   string imageOutPortName;
   string moduleName;

   /* parameters */

   int    frequency;


   // ports

   BufferedPort<ImageOf<PixelRgb> > imageIn;
   BufferedPort<ImageOf<PixelRgb> > imageOut;

   /* pointer to a new thread to be created and started in configure() and stopped in close() */

   SampleVideoThread *sampleVideoThread;

public:
   virtual bool configure(yarp::os::ResourceFinder &rf);
   virtual bool updateModule();
   virtual bool interruptModule();
   virtual bool close();
   virtual double getPeriod();
};

#endif // __ICUB_SAMPLEVIDEO_MODULE_H__
//empty line to make gcc happy

