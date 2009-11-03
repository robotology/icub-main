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
//=============================================================================
// YARPImgRecv Includes 
//=============================================================================
#include <iCub/YarpImage2Pixbuf.h>
#include <iCub/YARPImgRecv.h>

//within Project Include
#include <iCub/WatershedOperator.h>
#include <iCub/SalienceOperator.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

/**
 * Module that applies Watershed techinique (rain falling) on the image buffered on the input port
 * @author Francesco Rea
 */


class WatershedModule : public Module {
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
	* a port for reading the R+G- colour opponency Image
	*/
	BufferedPort<ImageOf<PixelRgb> > portRG; // 
	/**
	* a port for reading the G+R- colour opponency Image
	*/
	BufferedPort<ImageOf<PixelRgb> > portGR; // 
	/**
	* a port for reading the B+Y- colour opponency Image 
	*/
	BufferedPort<ImageOf<PixelRgb> > portBY; // 
	/**
	* port where the processed image is buffered out
	*/
	BufferedPort<ImageOf<PixelRgb> > port_out; //
	/**
	* port where the image of the found blob is put
	*/
	BufferedPort<ImageOf<PixelBgr> > port_Blobs; //
	/**
	* port where the commands are sent
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
	//------------------ PUBLIC METHODS -----------------------------------

	/**
	* default constructor
	*/
	WatershedModule();
	/**
	* open and initialise the module
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
	* create the main Window
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
	/**
	* set different components of the module up
	*/
	void setUp();
	/**
	* creats some components of the module
	*/
	void createObjects();
	/**
	* open some additional ports
	*/
	bool openPorts();
	/**
	* close additional ports
	*/
	bool closePorts();
	/**
	* draws all the blobs in the module
	*/
	void drawAllBlobs(bool stable);
	
	//----------------- PUBLIC ATTRIBUTES -----------------------------
	/**
	* saliency parameter
	*/
	double salienceTD;
	/**
	* saliency parameter
	*/
	double salienceBU;
	/**
	* Output Bottle Port
	*/
	yarp::os::BufferedPort<yarp::os::Bottle> *_pOutPort;
	/**
	* Output Bottle Port #2
	*/
	yarp::os::BufferedPort<ImageOf<PixelRgb> > *_pOutPort2;
	/**
	*Output Bottle Container
	*/
	yarp::os::Bottle _outBottle;
	/**
	* reference to the watershed operator
	*/
	WatershedOperator *wOperator;
	/**
	* reference to the salience operator
	*/
	SalienceOperator *salience;
	int max_tag;
	/**
	* vector of boolean which tells whether there is a blob or not
	*/
	char* blobList; //
	/**
	*vector of tags to the sequence of blobs
	*/
	ImageOf<PixelInt>* tagged;  //
	/**
	* image result of the function outContrastLP
	*/
	ImageOf<PixelMono> *outContrastLP;
	/**
	* image result of the function meanColourLP;
	*/
	ImageOf<PixelBgr> *outMeanColourLP;
	/**
	* image of the most salient blob
	*/
	ImageOf<PixelMono>* maxSalienceBlob_img;
	/**
	* pointer to the input image of the red plane
	*/
	yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputRed; //
	/**
	* pointer to the input image of the green plane
	*/
	yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputGreen; //
	/**
	* pointer to the input image of the blue plane
	*/
	yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputBlue; //
	/**
	* pointer to the input image of the R+G- colour opponency
	*/
	yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputRG;  //
	/**
	* pointer to the input image of the G+R- colour opponency
	*/
	yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputGR;  //
	/**
	* pointer to the input image of the B+Y- colour opponency
	*/
	yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputBY; //
	/**
	* input image of the opponency R+G-
	*/
	ImageOf<PixelMonoSigned> *_inputImgRGS;
	/**
	* input image of the opponency G+R-
	*/
	ImageOf<PixelMonoSigned> *_inputImgGRS;
	/**
	* input image of the opponency B+Y-
	*/
	ImageOf<PixelMonoSigned> *_inputImgBYS;
	/**
	* image of the fovea blob
	*/
	ImageOf<PixelMono> *blobFov;
	/**
	* image which is plotted in the drawing area
	*/
	ImageOf<PixelRgb> *image_out; //
	//---------- flags --------------------------
	/**
	* flag for drawing contrastLP
	*/
	bool contrastLP_flag;
	/**
	* flag for drawing meanColourImage
	*/
	bool meanColour_flag;
	/**
	* flag for drawing blobCatalog
	*/
	bool blobCataloged_flag;
	/**
	* flag for drawing foveaBlob
	*/
	bool foveaBlob_flag;
	/**
	* flag for drawing colorVQ
	*/
	bool colorVQ_flag;
	/**
	* flag for drawing maxSaliencyBlob
	*/
	bool maxSaliencyBlob_flag;
	/**
	* flag for drawing blobList
	*/
	bool blobList_flag;
	/**
	* flag for the drawings
	*/
	bool tagged_flag;
	/**
	* flag for drawing watershed image
	*/
	bool watershed_flag;
	/**
	* flag for drawing redPlane image
	*/
	bool redPlane_flag;
	/**
	* flag for drawing greenPlane image
	*/
	bool greenPlane_flag;
	/**
	* flag for drawing bluePlane image
	*/
	bool bluePlane_flag;
	/**
	* flag for drawing RedGreen Opponency Map
	*/
	bool RG_flag;
	/**
	* flag for drawing GreenRed Opponency Map
	*/
	bool GR_flag;
	/**
	* flag for drawing BlueYellow Opponency Map
	*/
	bool BY_flag;


	//----checkButtons
	GtkWidget *buttonCheckGreen,*buttonCheckRed,*buttonCheckBlue;
	YARPBox* max_boxes;
	
};

#endif //_IMAGEPROCESSMODULE_H_