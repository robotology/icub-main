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
*
@ingroup icub_module
\defgroup icub_watershedModule watershedModule

Module that applies Watershed techinique (rain falling) on the image buffered on the input port

\section intro_sec Description
This module receives an edge image and the respective red plane, green plane and yellow plane.
It needs the opponency maps composed as R+G-, G+R-, B+Y-.
The module applies the watershed technique in order to extract all the blob and calculates a saliency map based on the mean color of the blob,
its dimension and the the color distance to the target object


The module does:
-   stream the mean color image of all the blobs
-   stream the image of the fovea blob
-   strean the saliency map as a gray scale image
-	stream the most salient blob


\image html EXAMPLE.jpg

\section lib_sec Libraries
YARP
OPENCV
IPP
GTK

\section parameters_sec Parameters
This provides a comprehensive list of the parameters. For example:
none

 
\section portsa_sec Ports Accessed
ImageProcessor/outImage:o
ImageProcessor/outRed:o
ImageProcessor/outGreen:o
ImageProcessor/outBlue:o
ImageProcessor/outRG:o
ImageProcessor/outGR:o
ImageProcessor/outBY:o


\section portsc_sec Ports Created
Input ports:
- /watershed/inputImage:i
- /watershed/inRed:i
- /watershed/inGreen:i
- /watershed/inBlue:i
- /watershed/inRG:i
- /watershed/inGR:i
- /watershed/inBY:i
Outports
- /watershed/outBlobs:o
- /watershed/outView:o
- /watershed/outImage:o

\section in_files_sec Input Data Files
none

\section out_data_sec Output Data Files
none
 
\section conf_file_sec Configuration Files
none

For example:
The module requires a description of the robot through the parameter 
--file.

\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
Provide here a typical example of use of your module.
Example:

myModule --from module.ini

\author Francesco Rea

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

**/


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
	* saliencyTOT linear combination Ktd coefficient (TOP DOWN saliency weight)
	*/
	double salienceTD;
	/**
	* saliencyTOT linear combination Kbu coefficient (BOTTOM-UP saliency weight)
	*/
	double salienceBU;
	/**
	* red intensity of the target that has been found 
	*/
	double targetRED;
	/**
	* green intensity of the target that has been found 
	*/
	double targetGREEN;
	/**
	* blue intensity of the target that has been found 
	*/
	double targetBLUE;

	/**
	* Output Bottle Port for any possible command
	*/
	yarp::os::BufferedPort<yarp::os::Bottle> *_pOutPort;
	/**
	* Output port that send out a PixelRgb image
	*/
	yarp::os::BufferedPort<ImageOf<PixelRgb> > *_pOutPort2;
	/**
	* Output port that send out a PixelRgb image
	*/
	yarp::os::BufferedPort<ImageOf<PixelRgb> > *_pOutPort3;
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
	/**
	* flag that indicates that Opponency images are not ready
	*/
	bool noOpponencies_flag;
	/**
	* flag that indicates that Plane images are not ready
	*/
	bool noPlanes_flag;
	/**
	* maxBLOB dimension
	*/
	int maxBLOB;
	/**
	* minBLOB dimension
	*/
	int minBLOB;

	//----checkButtons
	GtkWidget *buttonCheckGreen,*buttonCheckRed,*buttonCheckBlue;
	YARPBox* max_boxes;
	
};

#endif //_IMAGEPROCESSMODULE_H_