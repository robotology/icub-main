// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _IMAGEPROCESSMODULE_H_
#define _IMAGEPROCESSMODULE_H_

#include <ace/config.h>

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

//=============================================================================
// GTK Includes 
//=============================================================================
#include <gtk/gtk.h>

#include <iCub/YarpImage2Pixbuf.h>
#include <iCub/YARPImgRecv.h>
#include <iCub/YARPIntegralImage.h>

#include <iCub/MachineBoltzmann.h>

//#include <ipp.h>




using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;



/**
*
@ingroup icub_guis
\defgroup biControlGazeInterface biControlGazeInterface

the module provides the interface in order to use the Biological inspired control of the gaze

\section intro_sec Description
This module sends commands as bottle to the biologicalInspired gaze control module. The command respect a communication stardard based
on bottle composed of vocabols


The module does:
- send commands to the imageProcessor module
- plot on the window drawing area image of an input image

\image html biologicalInspiredControlGaze.png

\section lib_sec Libraries
YARP
GTK

\section parameters_sec Parameters
--name:defines the name of the module and the rootname of every port
 
\section portsa_sec Ports Accessed
- /BIControlGazeEngine/cmd 


\section portsc_sec Ports Created
Input ports:
- /e <name>/img:i
Outports
- /e <name>/cmd



\section in_files_sec Input Data Files
none

\section out_data_sec Output Data Files
none
 
\section conf_file_sec Configuration Files
none


\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
BIControlGazeInterface --name biControlGazeInterface


\author Francesco Rea

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

**/

class BIControlGazeInterface : public Module {
private:
	/**
	* a port for reading an input image
	*/
    BufferedPort<ImageOf<PixelRgb> > port_in; // 
	
	/**
	*port where the processed image is buffered out
	*/
	BufferedPort<ImageOf<PixelRgb> > port_out; 
	
	/**
	* port dedicated to the command communication to the actual computation module
	*/
    Port cmdPort;
	/**
	* counter
	*/
    int ct;
	/**
	* options of the connection
	*/
	Property options;	//
public:
	/**
	* constructor of the BiologicalInspired control of the gaze (Interface)
	*/
	BIControlGazeInterface();
	/**
	* open the port
	*/
	bool open(Searchable& config); //
	/**
	* try to interrupt any communications or resource usage
	*/
    bool interruptModule(); // 
	/**
	* closes all the ports
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
	/**
	* set the images to the output ports
	*/
	bool outPorts(); 
	/**
	* close all the ports activated when the module starts
	*/
	bool closePorts(); 
	
	

	//--- atributes ---
	/**
    * Output Point Port as bottle
    */
	yarp::os::BufferedPort<yarp::os::Bottle> *_pOutPort;
	/**
    * Output Point Port2 as ImageOf PixelRgb
    */
    yarp::os::BufferedPort<ImageOf<PixelRgb> > *_pOutPort2;
	/** 
	* string where the command that has to be sent is stored
	*/
	string *command;
	/**
	* Output Bottle Container
	*/
	yarp::os::Bottle _outBottle;

	
	

	
	/**
	* image which is plotted in the drawing area
	*/
	ImageOf<PixelRgb> *image_out; //
	

	/**
	*bottle containing the option of the command
	*/
	Bottle bOptions;
	
	
};

#endif //_IMAGEPROCESSMODULE_H_