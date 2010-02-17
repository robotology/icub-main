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
* Graphical interface used in order to send preformatted commands to the BMLEngine through port
* and to visualize the input data, the state of the layer and the feature extracted by the boltzmann Machine
* @author Francesco Rea
*/

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
	
};

#endif //_IMAGEPROCESSMODULE_H_