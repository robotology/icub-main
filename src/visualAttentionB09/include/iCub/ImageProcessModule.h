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

//within Project Include
#include <iCub/ImageProcessor.h>
#include <iCub/YARPImgRecv.h>
#include <iCub/YarpImage2Pixbuf.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

/**
*
@ingroup icub_module
\defgroup icub_watershedModule visualAttentionB09

Module that performs image processing on the input image
The module has been developed to be first stage of the protoObjectVisualAttention

\section intro_sec Description
This module reads from a buffered port the input image. It straight away produces an output image based on the standard
parameters configuration. However these parameters can be modified any time in order to process the inputimage differently.



The module does:
-   process the input image based on the actual configuration
-   allow the user to change the undertaken processing
-   stream the outimage out
-	stream the colour planes of the image
-	stream the colour opponency maps (R+G-, G+R-, B+Y-)
-	allow the user to visually check the process

\image html EXAMPLE.jpg

\section lib_sec Libraries
YARP
OPENCV
GTK

\section parameters_sec Parameters
--name : name of the module and relative port name

 
\section portsa_sec Ports Accessed



\section portsc_sec Ports Created
Input ports:
- /imageProcessor/inputImage:i
Outports
- /ImageProcessor/outImage:o
- /ImageProcessor/outRed:o 
- /ImageProcessor/outGreen:o
- /ImageProcessor/outBlue:o
- /ImageProcessor/outRG:o
- /ImageProcessor/outGR:o
- /ImageProcessor/outBY:o

\section in_files_sec Input Data Files
none

\section out_data_sec Output Data Files
none
 
\section conf_file_sec Configuration Files
none
--file.

\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
visualAttentionB09 


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
    Port cmdPort;
	/**
	* counter of the module
	*/
    int ct;
	/**
	* options of the connection
	*/
	Property options;	//
	/**
	* maximum value for the gtk sliding control
	*/
	double maxAdj;
	/**
	* maximum value for the gtk sliding control
	*/
	double minAdj;
	/**
	* step adjustment for the gtk sliding control
	*/
	double stepAdj;
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
	/**
	* creates the main Window
	*/
	GtkWidget* createMainWindow(); //
	/**
	* menuFileSingle callback
	*/
	static gint menuFileSingle_CB(GtkWidget *widget, GdkEventExpose *event, gpointer data);
	/**
	* menufileset callback
	*/
	gint menuFileSet_CB(GtkWidget *widget, GdkEventExpose *event, gpointer data);
	/**
	* creates the menubar
	*/
	GtkWidget* createMenubar();
	/**
	* updates status bar
	*/
	void updateStatusbar (GtkStatusbar  *statusbar);
	/**
	* creates some objects necessary for the window
	*/
	void createObjects();
	/**
	* sets the adjustments in the window
	*/
	void setAdjs();
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
	/**
	* sets the module up
	*/
	void setUp();
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
};

#endif //_IMAGEPROCESSMODULE_H_