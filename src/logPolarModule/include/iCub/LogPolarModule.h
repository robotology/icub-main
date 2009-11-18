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
\defgroup icub_yourModule yourModule

Place here a short description of the module. This will
appear in the list of the modules.

\section intro_sec Description
This module is not implemented, it is only a template for writing
the documentation of modules.

Place here a description of your module. You might want to use a list as in:

The module does:
-	this
-	that
-	...

You might find it convenient to include an image:
\image html EXAMPLE.jpg
\image latex EXAMPLE.eps "MODULENAME running on Linux" width=10cm

\section lib_sec Libraries
List here dependencies. Often these are just YARP libraries.

\section parameters_sec Parameters
Provide a comprehensive list of the parameters you can pass to the module. For example:

--file mymodule.ini: configuration file to use
 
\section portsa_sec Ports Accessed
This is important. List here ports accessed by the module. This is useful to build a list of dependencies between modules.

\section portsc_sec Ports Created
Provide the list of ports created by the module. Separate them in input and output ports, specify expected data format.

Example:

Output ports:
- /mymodule/head/out: streams out a yarp::sig::vector which contains the commanded velocity of the head, the size of the vector matches the number of joints of the head
- /mymodule/right_arm/out: ...

Input ports:
- /mymodule/rpc:i: input ports to control the module, accept a yarp::os::Bottle which contains commands to start/stop/quit the module.
    -	[start]: start the module
    -	[stop]: stop the module (resume with start)
    -	[quit]: quit the module (exit)

\section in_files_sec Input Data Files
If your module expect data from a file, say so.

\section out_data_sec Output Data Files
If your module writes data to a file, say so.
 
\section conf_file_sec Configuration Files
If parameters to your module can be passed through a txt file, describe it here. 

For example:
The module requires a description of the robot through the parameter 
--file.

The file consists in a few sections:
\code
name        myModule
rate        20
\endcode

\e name determines the name of the module

\e rate specifies the rate (ms) of the thread

...

\section tested_os_sec Tested OS
Specify the operating systems on which the module was tested
Example:

Linux and Windows.

\section example_sec Example Instantiation of the Module
Provide here a typical example of use of your module.
Example:

myModule --from module.ini

\author your name

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/myModule/main.cpp.
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
