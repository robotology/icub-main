// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _LOGPOLARMODULE_H_
#define _LOGPOLARMODULE_H_


//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>


//#include <iCub/RC_DIST_FB_logpolar_mapper.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;


/**
  * Module that extracts the log polar image of an input image.
  * The module is able to produce a fake output image composed of coloured circles
  * in order to help the processing downward in the module chain.
  * @author Francesco Rea

 */

/**
*
@ingroup icub_module
\defgroup icub_openCVLogPolar openCVLogPolar

This module is a wrapper of the opencv logpolar transform; it extracts the log polar image 
from an input image.
It can produce a fake output image composed of coloured circles

\section intro_sec Description

The module does:
-	creates a logpolar image starting from the input image
-	regenerate the original image from the logPolar image
-	creates a fake image composed of coloured circles

\section lib_sec Libraries
YARP 
OPENCV

\section parameters_sec Parameters
Here is a  comprehensive list of the parameters you can pass to the module.

--name (string) name of the module. The name of all the ports will be istantiated starting from this name 
--mode (string) defines the modality with which the module will produce the output: 
			[SIMULATION: produce a fake image, FORWARD: from image to logPolar, REVERSE: from logPolar to image]
 
\section portsa_sec Ports Accessed
none

\section portsc_sec Ports Created
<name>/in
<name>/out
<name>/outSimulation
<name>/outInverse

Output ports:
- <name>/out: streams out a yarp::sig::ImageOf<PixelRgb> which is the result of the processing (either FORWARD or REVERSE)
- <name>/outInverse:  streams out a yarp::sig::Image<PixelRgb> which is the reverse logPolar of the image in out (only FORWARD mode)
- <name>/outSimulation:  streams out a yarp::sig::Image<PixelRgb> which is composed of coloured circles

Input ports:
- <name>/in: input ports which takes as input a yarp::sig::ImageOf<PixelRgb>

\section in_files_sec Input Data Files
none

\section out_data_sec Output Data Files
none
 
\section conf_file_sec Configuration Files
none

\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
openCVLogPolar --name /LogPolar/ --mode FORWARD

\author Francesco Rea

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

**/



class LogPolarModule : public Module {
private:
	/**
	* a port for reading and writing images
	*/
    BufferedPort<ImageOf<PixelRgb> > port; // 
	/**
	* port for writing the log polar mapping
	*/
	BufferedPort<ImageOf<PixelRgb> > port2; //
	/**
	* port for the inverse log polar mapping
	*/
	BufferedPort<ImageOf<PixelRgb> > port3; //
	/**
	* port for the simulated output
	*/
	BufferedPort<ImageOf<PixelRgb> > port4; //
	/**
	* port for various command
	*/
    Port cmdPort;
	/**
	* exexution step counter
	*/
    int ct;
	/**
	*yarp returned image
	*/
	ImageOf<PixelRgb> yarpReturnImage;  
	/**
	* YARP pointer to the output image
	*/
	ImageOf<PixelRgb> *yarpReturnImagePointer;
	/**
	* destination color image of the outPort
	*/
	IplImage* dstColor;   //
	/**
	* destination color image of the invese outPort
	*/
	IplImage* dstColor2;
	CvRect rec;
	/**
	* input image (always 320,240)
	*/
	ImageOf<PixelRgb> *img; 
	/**
	* ------------
	*/
	ImageOf<PixelRgb> *image2;
	/**
	* OpenCV image necessary during the second step processing
	*/
	IplImage *cvImage;
	/**
	* OpenCV image necessary for copy the rectangular input image in the inverse mode
	*/
	IplImage *cvImage1;
	/**
	* OpenCV image for the first step of processing (240,240 in INVERSE mode)
	*/
	IplImage *cvImage2;
	/**
	* options of the module deteched from command line
	*/
	Property options;
	/**
	* mode of work of the module
	*/
	 int mode;
public:
	/**
	*opens the port and intialise the module
	* @param config configuration of the module
	*/
	bool open(Searchable& config); 
	/**
	* tries to interrupt any communications or resource usage
	*/
    bool interruptModule(); // 
	/**
	* function that set the options detected from the command line
	* @param opt options passed to the module
	*/
	void setOptions(yarp::os::Property opt);
	/**
	* catches all the commands that have to be executed when the module is closed
	*/
    bool close(); 
	/**
	* updates the module
	*/
	bool updateModule();
    /**
    * loads a bitmap image
    */
	unsigned char* Load_Bitmap (int *X_Size, int *Y_Size, int *planes, char *filename){};
	/**
    * loads a bitmap image
    */
	unsigned char* Load_Bitmap (int *X_Size, int *Y_Size, int *planes, ImageOf<PixelRgb> *src){};
	/**
    * loads a bitmap image
    */
	unsigned char* Load_Bitmap (int *X_Size, int *Y_Size, int *planes, ImageOf<PixelMono> *src){};
	/**
	* save the image as bitmap
	*/
	void Save_Bitmap (unsigned char *image, int X_Size, int Y_Size, int planes,char *filename){};
};


#endif
