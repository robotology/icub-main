// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _BIControlGazeEngine_H_
#define _BIControlGazeEngine_H_

#include <ace/config.h>

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>



//IPP include
//#include <ipp.h>

//within Project Include
#include <iCub/TrackerThread.h>
#include <string>


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;


/**
*
@ingroup icub_module
\defgroup icub_vergenceController vergenceController



\section intro_sec Description


The module does:



\image html 

\section lib_sec Libraries
YARP


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


// general command vocab's
#define COMMAND_VOCAB_HELP VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_RSET VOCAB4('r','s','e','t')
#define COMMAND_VOCAB_FLT VOCAB3('f','l','t')
#define COMMAND_VOCAB_SET VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET VOCAB3('g','e','t')
#define COMMAND_VOCAB_RUN VOCAB3('r','u','n')
#define COMMAND_VOCAB_IS VOCAB2('i','s')
#define COMMAND_VOCAB_FAILED VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_OK VOCAB2('o','k')
#define COMMAND_VOCAB_CHILD_COUNT VOCAB2('c','c')

#define COMMAND_VOCAB_NAME VOCAB2('s','1')
#define COMMAND_VOCAB_CHILD_NAME VOCAB2('c','n')
#define COMMAND_VOCAB_SALIENCE_THRESHOLD VOCAB2('t','h')
#define COMMAND_VOCAB_NUM_BLUR_PASSES VOCAB2('s','2')
#define COMMAND_VOCAB_RGB_PROCESSOR VOCAB3('r','g','b')
#define COMMAND_VOCAB_YUV_PROCESSOR VOCAB3('y','u','v')
// command vocab for the output produced
#define COMMAND_VOCAB_JNT0 VOCAB4('j','n','t','0')
#define COMMAND_VOCAB_JNT1 VOCAB4('j','n','t','1')
#define COMMAND_VOCAB_JNT2 VOCAB4('j','n','t','2')
#define COMMAND_VOCAB_JNT3 VOCAB4('j','n','t','3')
#define COMMAND_VOCAB_JNT4 VOCAB4('j','n','t','4')
#define COMMAND_VOCAB_JNT5 VOCAB4('j','n','t','5')
#define COMMAND_VOCAB_JNT6 VOCAB4('j','n','t','6')

// other commands
#define COMMAND_VOCAB_TCON VOCAB4('t','c','o','n') //time contant for the controlGaze2
#define COMMAND_VOCAB_TCEN VOCAB4('t','c','e','n') //time constant for the iKinGazeCtrl


#define COMMAND_VOCAB_KBU VOCAB3('k','b','u') //weight of the bottom-up algorithm
#define COMMAND_VOCAB_KTD VOCAB3('k','t','d') //weight of top-down algorithm
#define COMMAND_VOCAB_RIN VOCAB3('r','i','n') //red intensity value
#define COMMAND_VOCAB_GIN VOCAB3('g','i','n') //green intensity value
#define COMMAND_VOCAB_BIN VOCAB3('b','i','n') //blue intensity value
#define COMMAND_VOCAB_MAXDB VOCAB3('M','d','b') //Maximum dimension of the blob drawn
#define COMMAND_VOCAB_MINDB VOCAB3('m','d','b') //minimum dimension of the blob drawn


// directional saliency filter vocab's
#define COMMAND_VOCAB_DIRECTIONAL_NUM_DIRECTIONS VOCAB3('d','n','d')
#define COMMAND_VOCAB_DIRECTIONAL_NUM_SCALES VOCAB3('d','n','s')
#define COMMAND_VOCAB_DIRECTIONAL_DBG_SCALE_INDEX VOCAB3('d','s','i')
#define COMMAND_VOCAB_DIRECTIONAL_DBG_DIRECTION_INDEX VOCAB3('d','d','i')
#define COMMAND_VOCAB_DIRECTIONAL_DBG_IMAGE_ARRAY_NAMES VOCAB4('d','a','n','s')
#define COMMAND_VOCAB_DIRECTIONAL_DBG_IMAGE_ARRAY_NAME VOCAB3('d','a','n')



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
    /**
     * semaphore for the respond function
     */
     Semaphore mutex;
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
    * functions to respond to external commands
    * @param command received command
    * @param reply reply message
    */
    bool respond(const Bottle &command,Bottle &reply);
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

#endif 