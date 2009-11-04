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

class ImageProcessModule : public Module {
private:
    BufferedPort<ImageOf<PixelRgb> > port; // a port for reading and writing images
	BufferedPort<ImageOf<PixelRgb> > port2; //port for writing the log polar mapping
	BufferedPort<ImageOf<PixelMono> > port_plane; // port where the red plane of the input image is buffered
    Port cmdPort;
    int ct;
	Property options;	//options of the connection
	double maxAdj;
	double minAdj;
	double stepAdj;
public:
	bool open(Searchable& config); //open the port
    bool interruptModule(); // try to interrupt any communications or resource usage
	bool close(); //closes the modules and all its components
	bool updateModule(); //active control of the Module
	void setOptions(Property options); //set the attribute options of class Property
	GtkWidget* createMainWindow(); //create the main Window
	static gint menuFileSingle_CB(GtkWidget *widget, GdkEventExpose *event, gpointer data);
	gint menuFileSet_CB(GtkWidget *widget, GdkEventExpose *event, gpointer data);
	GtkWidget* createMenubar();
	void updateStatusbar (GtkStatusbar  *statusbar);
	void createObjects();
	void setAdjs();
	//gint timeout_CB (gpointer data);
	//bool getImage();
	bool openPorts();
	bool closePorts();
	bool outPorts();
	void setUp();
	//---attributes
	// Output Point Ports
	yarp::os::BufferedPort<ImageOf<PixelRgb> > *_pOutPort; //output port for the first Processor result
	yarp::os::BufferedPort<ImageOf<PixelRgb> > *_pOutPort2; //output port for the first Processor result
	yarp::os::BufferedPort<ImageOf<PixelRgb> > *_pOutPort3; //output port for the first Processor result
	yarp::os::BufferedPort<ImageOf<PixelMono> > *portRg; //output port for R+G- Color Opponency Image
	yarp::os::BufferedPort<ImageOf<PixelMono> > *portGr; //output port for G+R- Color Opponency Image
	yarp::os::BufferedPort<ImageOf<PixelMono> > *portBy; //output port for B+Y- Color Opponency Image
	yarp::os::BufferedPort<ImageOf<PixelMono> > *portRedPlane; //output port for R+G- Color Opponency Image
	yarp::os::BufferedPort<ImageOf<PixelMono> > *portGreenPlane; //output port for G+R- Color Opponency Image
	yarp::os::BufferedPort<ImageOf<PixelMono> > *portBluePlane; //output port for B+Y- Color Opponency Image
	// Output Bottle Container
	yarp::os::Bottle _outBottle;
	ImageProcessor *processor1;
	ImageProcessor *processor2;
	ImageProcessor *processor3;
	ImageProcessor *currentProcessor;
	
};

#endif //_IMAGEPROCESSMODULE_H_