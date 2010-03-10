// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _BMLINTERFACE_H_
#define _BMLINTERFACE_H_

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

//======================
// within project includes
//======================
#include <iCub/YarpImage2Pixbuf.h>
#include <iCub/YARPImgRecv.h>
#include <iCub/YARPIntegralImage.h>
#include <iCub/MachineBoltzmann.h>
#include <iCub/graphicThread.h>

//#include <ipp.h>




using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;


/**
*
@ingroup icub_guis
\defgroup bmlInterface bmlInterface

 Graphical interface used in order to send preformatted commands to the BMLEngine through port
 and to visualize the input data, the state of the layer and the feature extracted by the boltzmann Machine

\section intro_sec Description
This module sends commands as bottle to the BMLEngine module. The command respect the communication stardard of the BMLEngine.
In addition this module is able to draw the active layers in the associated instance of the BMLEngine. This allows the user to 
visually control the learning process of the Boltzmann Machine.

The user is able to
- add new layers
- set the dimension of new layer
- interconnect 2 layers
- save the state of the boltzmann machine
- load the state of the boltzmann machine


The module does:
- send commands to the BMLEngine module
- plot  images of the active layers on the drawing area of the window

\image html boltzmannMachineLibrary.png

\section lib_sec Libraries
YARP
BML

\section parameters_sec Parameters
--name:defines the name of the module and the rootname of every port
 
\section portsa_sec Ports Accessed
- BMLEngine/cmd 


\section portsc_sec Ports Created
Input ports:
- <name>/image:i
- <name>/layer1:i
- <name>/layer2:i
- <name>/layer3:i
- <name>/layer4:i
- <name>/layer5:i
- <name>/layer6:i
- <name>/layer7:i
Outports
- <name>/command:o

\section in_files_sec Input Data Files
none

\section out_data_sec Output Data Files
none
 
\section conf_file_sec Configuration Files
none


\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
bmlInterface


\author Francesco Rea

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

**/

class BMLInterface : public Module {
private:
	/**
	* a port for reading the edge image
	*/
    BufferedPort<ImageOf<PixelRgb> > port_in; //  
	/**
	* a port for reading the input Image of the Red Plane
	*/
	BufferedPort<ImageOf<PixelRgb> > portRedPlane; // 
	/**
	* a port for reading the input Image of the Green Plane
	*/
	BufferedPort<ImageOf<PixelRgb> > portGreenPlane; //
	/**
	* a port for reading the input Image of the Blue Plane 
	*/
	BufferedPort<ImageOf<PixelRgb> > portBluePlane; // 
	/**
	*a port for reading the R+G- colour opponency Image
	*/
	BufferedPort<ImageOf<PixelRgb> > portRG; // 
	/**
	* a port for reading the G+R- colour opponency Image
	*/
	BufferedPort<ImageOf<PixelRgb> > portGR; 
	/**
	*a port for reading the B+Y- colour opponency Image
	*/
	BufferedPort<ImageOf<PixelRgb> > portBY; //  
	/**
	*port where the processed image is buffered out
	*/
	BufferedPort<ImageOf<PixelRgb> > port_out; 
	/**
	*port where the image of the found blob is put
	*/
	BufferedPort<ImageOf<PixelBgr> > port_Blobs; 
	/**
	* port used for all the commands
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
	* constructor of the class
	*/
	BMLInterface();
    /**
	* destructor of the class
	*/
	~BMLInterface();
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
	* set all the components up
	*/
	void setUp();
	/**
	* create all the necessary objects
	*/
	void createObjects();
	/**
	* open all the ports and all the YARP image receivers 
	* on the dedicated ports
	*/
	bool openPorts();
	/**
	* close all the ports and all the YARP image receivers 
	*/
	bool closePorts();

	//void drawAllBlobs(bool stable);
	//--- atributes ---
	/**
	* Output Port for commands 
	*/
	yarp::os::BufferedPort<yarp::os::Bottle> *_pOutPort;
	/**
	* Output Port for commands 
	*/
	yarp::os::BufferedPort<ImageOf<PixelRgb> > *_pOutPort2;
	/**
	* command that is send throgh the command port
	*/
	string *command;
	/**
	* Output Bottle Container
	*/
	yarp::os::Bottle _outBottle;

	
	/**
	* flag that indicates when an input Image is regurarly 
	* acquired on the dedicated port
	*/
	bool inputImageReady_flag;
	/**
	* function that 
	*/
	void setRowDim(int number);
	/**
	* pointer to the input image of layer8
	*/
	void setColDim(int number);

	ImageOf<PixelMonoSigned> *_inputImgRGS;
	ImageOf<PixelMonoSigned> *_inputImgGRS;
	ImageOf<PixelMonoSigned> *_inputImgBYS;
	ImageOf<PixelMono> *blobFov;
	/**
	* image which is plotted in the drawing area
	*/
	ImageOf<PixelRgb> *image_out; //
	//---------- FLAGS -------------------------------
	/**
	* flag that indicates the control box stopEvolution is active
	*/
	bool stopEvolution_flag;
	/**
	* flag that indicates the control box EvolveClamped is active
	*/
	bool evolveClamped_flag;
	
	/**
	* flag that indicates when the freely run mode is active
	*/
	bool runFreely_flag;
	/**
	* flag that indicates when the clamped run mode is active
	*/
	bool runClamped_flag;
	/**
	* flag that indicates when the ClampLayer Button has been pushed
	*/
	bool clampLayer_flag;

	/**
	*  checkButton Green 
	*/
	GtkWidget *buttonCheckGreen;
	/**
	*  checkButton Red 
	*/
	GtkWidget *buttonCheckRed;
	/**
	*  checkButton Blue 
	*/
	GtkWidget *buttonCheckBlue;
	/**
	*bottle containing the option of the command
	*/
	Bottle bOptions;
	/**
	*Row Dimension of the layer currently set
	*/
	int rowDim;
	/**
	*Column Dimension of the layer currently set
	*/
	int colDim;
    /**
    * graphic unit interface composed by a window with drawing area and buttons
    */
    graphicThread* gui;
	
};

#endif //_BMLINTERFACE_H_


//----- end-of-file --- ( next line intentionally left blank ) ------------------
