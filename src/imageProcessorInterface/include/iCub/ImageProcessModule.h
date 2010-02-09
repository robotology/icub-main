// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _IMAGEPROCESSMODULE_H_
#define _IMAGEPROCESSMODULE_H_

#include <ace/config.h>

//within project includes
#include <iCub/ImageProcessor.h>
#include <iCub/graphicThread.h>

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

/**
*
@ingroup icub_module
\defgroup icub_imageProcessorInterface imageProcessorInterface

This module provides a graphical interface for imageProcessor module.

\section intro_sec Description
This module sends commands as bottle to the imageProcessor module. The command respect a communication stardard based
on bottle composed of vocabols


The module does:
- send commands to the imageProcessor module
- plot on the window drawing area image of an input image

\image html imageProcessor.png

\section lib_sec Libraries
YARP
OPENCV
GTK

\section parameters_sec Parameters
none
 
\section portsa_sec Ports Accessed
- /imageProcessor/cmd 


\section portsc_sec Ports Created
Input ports:
- /imageProcessorInterface/img:i
Outports
- /imageProcessorInterface/cmd



\section in_files_sec Input Data Files
none

\section out_data_sec Output Data Files
none
 
\section conf_file_sec Configuration Files
none


\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
imageProcessorInterface


\author Francesco Rea

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

**/

class ImageProcessModule : public Module {
private:
	/**
	* a port for reading and writing images
	*/
    BufferedPort<ImageOf<PixelRgb> > port; // 
	/**
	* a port for reading and writing images
	*/
	BufferedPort<ImageOf<PixelRgb> > port2; //
	/**
	* port where the red plane of the input image is buffered
	*/
	BufferedPort<ImageOf<PixelMono> > port_plane; // 
	/**
	* command port of the module
	*/
    BufferedPort<Bottle > cmdPort;
    /**
    * reference to the graphic unit interface managed by a thread
    */
    graphicThread* gui;
	/**
	* counter of the module
	*/
    int ct;
	/**
	* options of the connection
	*/
	Property options;	//
	
public:
	/**
	*open the ports of the module
	*/
	bool open(Searchable& config); //
	/**
	* tryes to interrupt any communications or resource usage
	*/
    bool interruptModule(); // 
	/**
	* closes the modules and all its components
	*/
	bool close(); //
	/**
	* active control of the Module
	*/
	bool updateModule(); //
	/**
	* set the attribute options of class Property
	*/
	void setOptions(Property options); //
	
	
	//gint timeout_CB (gpointer data);
	//bool getImage();
	/**
	* opens all the ports necessary for the module
	*/
	bool openPorts();
	/**
	* closes all the ports opened when the module started
	*/
	bool closePorts();
	/**
	* streams out data on ports
	*/
	bool outPorts();
	
	//---attributes
	// Output Point Ports
	/**
	* output port for the first Processor result
	*/
	yarp::os::BufferedPort<ImageOf<PixelRgb> > *_pOutPort; //
	/**
	* output port for the first Processor result
	*/
	yarp::os::BufferedPort<ImageOf<PixelRgb> > *_pOutPort2; //
	/**
	* output port for the first Processor result
	*/
	yarp::os::BufferedPort<ImageOf<PixelRgb> > *_pOutPort3; //
	/**
	* output port for R+G- Color Opponency Image
	*/
	yarp::os::BufferedPort<ImageOf<PixelMono> > *portRg; //
	/**
	* output port for G+R- Color Opponency Image
	*/
	yarp::os::BufferedPort<ImageOf<PixelMono> > *portGr; //
	/**
	* output port for B+Y- Color Opponency Image
	*/
	yarp::os::BufferedPort<ImageOf<PixelMono> > *portBy; //
	/**
	* output port for R+G- Color Opponency Image
	*/
	yarp::os::BufferedPort<ImageOf<PixelMono> > *portRedPlane; //
	/**
	* output port for G+R- Color Opponency Image
	*/
	yarp::os::BufferedPort<ImageOf<PixelMono> > *portGreenPlane; //
	/**
	* output port for B+Y- Color Opponency Image
	*/
	yarp::os::BufferedPort<ImageOf<PixelMono> > *portBluePlane; //
	/**
	* Output Bottle Container
	*/
	yarp::os::Bottle _outBottle;
	/**
	* first processor
	*/
	ImageProcessor *processor1;
	/**
	* second processor
	*/
	ImageProcessor *processor2;
	/**
	* third processor
	*/
	ImageProcessor *processor3;
	/**
	* processor actually active
	*/
	ImageProcessor *currentProcessor;
	/**
	* flag that control if the inputImage has been ever read
	*/
	bool inputImage_flag;
    /** 
    * reference to the string refering to the last command to send
    */
    std::string* command;
};

#endif //_IMAGEPROCESSMODULE_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
