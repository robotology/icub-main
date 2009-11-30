// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>

//=============================================================================
// GTK Includes 
//=============================================================================
#include <gtk/gtk.h>

//within Project Include

#include <iCub/YARPImgRecv.h>
#include <iCub/YarpImage2Pixbuf.h>
#include <iCub/fingerImageThread.h>
#include <iCub/fingerReaderThread.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

/**
* Module that creates an interface for the representation of the skin sensor of the finger
* \author Francesco Rea
*/


class fingerInterfaceModule : public Module {
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
	* port where the red plane of the input image is buffered
	*/
	BufferedPort<ImageOf<PixelMono> > port_plane; // 
	/** 
	* port for commands
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
	/**
	* max value of the adjustments
	*/
	float maxAdj;
	/**
	* min value of the adjustments
	*/
	float minAdj;
	/**
	* step of the adjustments
	*/
	float stepAdj;
public:
	/**
	* flag for stopping the Module
	*/
	bool stop_flag;
	/**
	* opens the ports
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
	* activates control process of the Module
	*/
	bool updateModule(); //
	/**
	* create the main Window
	*/
	GtkWidget* createMainWindow(); //
	/**
	* creates objects in the main window
	*/
	void createObjects();
	/**
	* sets the adjustments of the window
	*/
	bool openPorts();
	/**
	* set up elements of the window
	*/
	void setUp();
	//---attributes
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
	*Output Bottle Container
	*/
	yarp::os::Bottle _outBottle;

	fingerImageThread *imageThread;
};

