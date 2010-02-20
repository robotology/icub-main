// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _BIControlGazeEngine_H_
#define _BIControlGazeEngine_H_

#include <ace/config.h>

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

//IPP include
//#include <ipp.h>

//within Project Include
//#include <iCub/BMLEngine.h>
#include <iCub/MachineBoltzmann.h>
#include <iCub/YARPImgRecv.h>
#include <iCub/TrackerThread.h>
#include <string>


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;


/**
*
@ingroup icub_module
\defgroup icub_biologicalControlGazeEngine biologicalControlGazeEngine

This class implements a process able of choose between different biological plausible control techniques for the gaze of the robot after training.
The motion is driven by a velocity controller. The module is able to get command from the associated interface or from an gui.

\section intro_sec Description
The module provides a biological plausible technique in order to select the best behaviour in order to perfom the most rewarded gaze action.
The module can choose between a series of behaviours ( vergence, saccade, smooth pursuit, etc ..) based on a complex combination of sensorial inputs
Those sensorial inputs are processed and the dimensionality of the data is reduced after training. The training extracts what the most salient input
of the input data are using a custom boltzmann machine. The user can shape the boltzmann machine as he prefers

Finally This module receives commands as bottle from the biologicalControlGazeInterface GUI. The command respect a communication stardard based
on bottle composed of vocabols


The module does:
- reads commands from the biological ControlGaze GUI
- able to control the robot with every single control algorithm
- start a training process based on the input channel
- after the training is termined the system can choose the best behaviour considering the very same data input


\image html 

\section lib_sec Libraries
YARP
BML

\section parameters_sec Parameters
--name:defines the name of the module and the rootname of every port
 
\section portsa_sec Ports Accessed
none

\section portsc_sec Ports Created
Input ports:
- <name>/cmd:i
- <name>/image:i

Outports
- <name>/cmd:o


\section in_files_sec Input Data Files
none

\section out_data_sec Output Data Files
none
 
\section conf_file_sec Configuration Files
none


\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
biologicalControlGazeEngine --name biologicalControlGazeEngine


\author Francesco Rea

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

**/


class BIControlGazeEngine : public Module {
private:
	 
	
	/** 
	 * option for the tracker thread
	 */
	Property options; 

    /** 
	 * input port for possible coming images
	 */
	BufferedPort<ImageOf<PixelRgb> > port; // input port for possible coming images
	/** 
	* output port n°0 for writing image out
	*/
	BufferedPort<ImageOf<PixelRgb> > port0; //port for writing the image out
	/** 
	* output port n°1 for writing image out
	*/
	BufferedPort<ImageOf<PixelRgb> > port1; //port for writing the image out
	/** 
	* output port n°2 for writing image out
	*/
	BufferedPort<ImageOf<PixelRgb> > port2; //port for writing the image out
	/** 
	* output port n°3 for writing image out
	*/
	BufferedPort<ImageOf<PixelRgb> > port3; //port for writing the image out
	/** 
	* port where the commands are vehiculated from controller to engine
	*/
	BufferedPort<yarp::os::Bottle> portCmd;

	BufferedPort<ImageOf<PixelMono> > port_plane; 
	/** 
	* counter for the update step
	*/
    int ct;
	/** 
	* Object that recalls the Boltzmann Machine Library
	*/
	MachineBoltzmann *mb;
	/** 
	* scale factor for the output image representing a layer (X axis)
	*/
	int scaleFactorX;
	/** 
	* scale factor for the output image representing a layer (Y axis)
	*/
	int scaleFactorY;
	/** 
	* sinchronized with the number of layer active
	*/
	int currentLayer;
	/**
	*counter incremented inside the updateModule
	*/
	int count;  
	map<std::string,Layer>::iterator iterE;
	map<std::string,Unit>::iterator iterU;
	map<std::string,Connection>::iterator iterC;
	ImageOf<PixelRgb> img_tmp; //=new ImageOf<PixelRgb>;
	ImageOf<PixelRgb> img; //=new ImageOf<PixelRgb>;
	ImageOf<PixelRgb> *img0; //=new ImageOf<PixelRgb>;
	ImageOf<PixelRgb> *img2; //=new ImageOf<PixelRgb>;
	/** 
	* flag that enable the drawing of the layer present in the simulation
	*/
	bool enableDraw;
	/** 
	* flag that regulates the execution of the freely mode
	*/
	bool runFreely;
	/** 
	* flag that regulates the execution of the clamped mode
	*/
	bool runClamped;
	
	/**
	* temporary red plane of the output image
	*/
	//Ipp8u* red_tmp;
	/**
	* temporary green plane of the output image
	*/
	//Ipp8u* green_tmp;
	/**
	* temporary blue plane of the output image
	*/
	//Ipp8u* blue_tmp;

	//Ipp8u *im_out;
	//Ipp8u* im_tmp[3];
	//Ipp8u* im_tmp_tmp;
	int psb;
	/** 
	* value that indicates whether an area can be visually clamped 
	*/
	int clampingThreshold;
public:
	/** 
	* open the ports
	* @param config configuration of the opening
	*/
	bool open(Searchable& config); //open the port
	/** 
	* try to interrupt any communication or resource usage
	*/
    bool interruptModule(); // try to interrupt any communications or resource usage
	/** 
	* close all the ports
	*/
	bool close(); //closes all the ports
	/** 
	* active control of the module
	*/
 	bool updateModule(); //active control of the Module
	/** 
	* set the attribute options of class Property
	* @param options option to be set
	*/
	void setOptions(Property options); //set the attribute options of class Property
	/** 
	* function that istantiate the TrackerThread
	* @param property of the thread
	*/
	void istantiateThread(Property options); //set the attribute options of class Property
	/** 
	* function that sets the scaleFactorX
	* @param value new value of the scaleFactorX
	*/
	void setScaleFactorX(int value);
	/** 
	* function that sets the scaleFactorY
	* @param value new value of the scaleFactorY
	*/
	void setScaleFactorY(int value);
	/** 
	* function that set the number of the layer active 
	* @param value number of the layer actually active
	*/
	void setCurrentLayer(int value);
	/** 
	* function that set and reset the boolean for the control of just the eyes
	* @param value of the flag to be set
	*/
	void setJustEyes(bool value);
	/** 
	* function that set the  vector of the tracker thread
	* @param a first element of the vector
	* @param b second element of the vector
	* @param c third element of the vector
	*/
	void setTrackerVector(double a, double b, double c);
	/** 
	* function that set the left vector of the tracker thread
	*/
	void setTrackerLeftVector();
	/** 
	* function that set the right vector of the tracker thread
	*/
	void setTrackerRightVector();
	/** 
	* function that stop the tracker where it is
	*/
	void stopTracker();
	/** 
	 * tracker thread for the head motion
	 */
	TrackerThread *thread;
	/** 
	* input Image which is mapped onto the selected layer
	*/
	ImageOf<PixelRgb> *ptr_inputImage;
	/** 
	* input Image which is mapped onto the selected layer
	*/
	ImageOf<PixelRgb> *ptr_inputImage2;
	/** 
	* number of layers already istantiated
	*/
	int countLayer;

};

#endif //_BIControlGazeEngine_H_