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

#include <ipp.h>




using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

/**
  * the module provides the interface in order to use the Biological inspired control of the gaze
  *
  * \author Francesco Rea
 */

class BIControlGazeInterface : public Module {
private:
	/**
	* a port for reading an input image
	*/
    BufferedPort<ImageOf<PixelRgb>> port_in; // 
	
	/**
	*port where the processed image is buffered out
	*/
	BufferedPort<ImageOf<PixelRgb>> port_out; 
	
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
	* create the main Window of the module
	*/
	GtkWidget* createMainWindow(); //
	/**
	* creates the menu bar
	*/
	GtkWidget* createMenubar(void); //
	/**
	* set the images to the output ports
	*/
	bool outPorts(); //
	void setUp();
	void createObjects();
	/** 
	* istantiate the port and open them
	*/
	bool openPorts();

	//--- atributes ---
	// Output Point Port
	yarp::os::BufferedPort<yarp::os::Bottle> *_pOutPort;
	yarp::os::BufferedPort<ImageOf<PixelRgb>> *_pOutPort2;
	/** 
	* string where the command that has to be sent is stored
	*/
	string *command;
	/**
	* Output Bottle Container
	*/
	yarp::os::Bottle _outBottle;

	/**
	* pointer to the input image o
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImage; //
	/**
	* pointer to the input image of layer0
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputLayer0; //
	/**
	* pointer to the input image of layer1
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputLayer1; //
	/**
	* pointer to the input image of layer2
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputLayer2; //
	/**
	* pointer to the input image of layer3
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputLayer3;  //
	/**
	* pointer to the input image of layer4
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputLayer4;  //
	/**
	* pointer to the input image of layer5
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputLayer5; //
	/**
	* pointer to the input image of layer6
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputLayer6; //
	/**
	* pointer to the input image of layer7
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputLayer7; //
	/**
	* pointer to the input image of layer8
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputLayer8; //
	/**
	* function that set the dimension in row
	*/
	void setRowDim(int number);
	/**
	* pointer to the input image of layer8
	*/
	void setColDim(int number);

	
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
	* flag that indicates the control box inputImage is active
	*/
	bool inputImage_flag;
	/**
	* flag that indicates the control box Layer0 is active
	*/
	bool inLayer0_flag;
	/**
	* flag that indicates the control box Layer1 is active
	*/
	bool inLayer1_flag;
	/**
	* flag that indicates the control box Layer2 is active
	*/
	bool inLayer2_flag;
	/**
	* flag that indicates the control box Layer3 is active
	*/
	bool inLayer3_flag;
	/**
	* flag that indicates the control box Layer4 is active
	*/
	bool inLayer4_flag;
	/**
	* flag that indicates the control box Layer5 is active
	*/
	bool inLayer5_flag;
	/**
	* flag that indicates the control box Layer6 is active
	*/
	bool inLayer6_flag;
	/**
	* flag that indicates the control box Layer7 is active
	*/
	bool inLayer7_flag;
	/**
	* flag that indicates the control box Layer8 is active
	*/
	bool inLayer8_flag;
	/**
	* flag that indicates the control box SelectLayer0 is active
	*/
	bool SelectLayer0_flag;
	/**
	* flag that indicates the control box SelectLayer0 is active
	*/
	bool SelectLayer1_flag;
	/**
	* flag that indicates the control box SelectLayer0 is active
	*/
	bool SelectLayer2_flag;
	/**
	* flag that indicates the control box SelectLayer0 is active
	*/
	bool SelectLayer3_flag;
	/**
	* flag that indicates the control box SelectLayer0 is active
	*/
	bool SelectLayer4_flag;
	/**
	* flag that indicates the control box SelectLayer0 is active
	*/
	bool SelectLayer5_flag;
	/**
	* flag that indicates the control box SelectLayer0 is active
	*/
	bool SelectLayer6_flag;
	/**
	* flag that indicates the control box SelectLayer0 is active
	*/
	bool SelectLayer7_flag;
	/**
	* flag that indicates the control box SelectLayer0 is active
	*/
	bool SelectLayer8_flag;
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

	//----------- checkButtons ---------------------

	/**
	* button present on the graphical interface
	*/
	GtkWidget *buttonCheckGreen,*buttonCheckRed,*buttonCheckBlue;

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
	
};

#endif //_IMAGEPROCESSMODULE_H_