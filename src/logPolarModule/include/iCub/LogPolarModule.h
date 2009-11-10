// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _LOGPOLARMODULE_H_
#define _LOGPOLARMODULE_H_

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>


#include <iCub/RC_DIST_FB_logpolar_mapper.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;


/**
  *Module that extracts the log polar image of an input image.
  * The module is able to produce a fake output image composed of coloured circles
  * in order to help the processing downward in the module chain.
  * @author Francesco Rea
 */


class LogPolarModule : public Module {
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
	* port for the inverse log polar mapping
	*/
	BufferedPort<ImageOf<PixelRgb> > port3; //
	/**
	* port for the simulated output
	*/
	BufferedPort<ImageOf<PixelRgb> > port4; //
	/**
	* port for various command
	*/
    Port cmdPort;
	/**
	* exexution step counter
	*/
    int ct;
	/**
	*yarp returned image
	*/
	ImageOf<PixelRgb> yarpReturnImage;  
	/**
	* YARP pointer to the output image
	*/
	ImageOf<PixelRgb> *yarpReturnImagePointer;
	/**
	* destination color image of the outPort
	*/
	IplImage* dstColor;   //
	/**
	* destination color image of the invese outPort
	*/
	IplImage* dstColor2;
	CvRect rec;
	/**
	* input image (always 320,240)
	*/
	ImageOf<PixelRgb> *img; 
	/**
	* ------------
	*/
	ImageOf<PixelRgb> *image2;
	/**
	* OpenCV image necessary during the second step processing
	*/
	IplImage *cvImage;
	/**
	* OpenCV image necessary for copy the rectangular input image in the inverse mode
	*/
	IplImage *cvImage1;
	/**
	* OpenCV image for the first step of processing (240,240 in INVERSE mode)
	*/
	IplImage *cvImage2;
	/**
	* options of the module deteched from command line
	*/
	Property options;
	/**
	* mode of work of the module
	*/
	 int mode;
public:
	/**
	*opens the port and intialise the module
	* @param config configuration of the module
	*/
	bool open(Searchable& config); 
	/**
	* tries to interrupt any communications or resource usage
	*/
    bool interruptModule(); // 
	/**
	* function that set the options detected from the command line
	* @param opt options passed to the module
	*/
	void setOptions(yarp::os::Property opt);
	/**
	* catches all the commands that have to be executed when the module is closed
	*/
    bool close(); 
	/**
	* updates the module
	*/
	bool updateModule();
    /**
    * loads a bitmap image
    */
	unsigned char* Load_Bitmap (int *X_Size, int *Y_Size, int *planes, char *filename){};
	/**
    * loads a bitmap image
    */
	unsigned char* Load_Bitmap (int *X_Size, int *Y_Size, int *planes, ImageOf<PixelRgb> *src){};
	/**
    * loads a bitmap image
    */
	unsigned char* Load_Bitmap (int *X_Size, int *Y_Size, int *planes, ImageOf<PixelMono> *src){};
	/**
	* save the image as bitmap
	*/
	void Save_Bitmap (unsigned char *image, int X_Size, int Y_Size, int planes,char *filename){};
};


#endif
