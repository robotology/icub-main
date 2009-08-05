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
  *Module that extract the log polar image of an input image
  *the module is able to produce a fake output image composed of coloured circle
  * in order to help the processing downward in the module chain
  * \author Francesco Rea
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
	* destination color image
	*/
	IplImage* dstColor;   //
	/**
	* destination color image
	*/
	IplImage* dstColor2;
	ImageOf<PixelRgb> *img; 
	ImageOf<PixelRgb> *image2;
	/**
	* OpenCV image necessary during the processing
	*/
	IplImage *cvImage;
	/**
	* OpenCV image necessary during the processing
	*/
	IplImage *cvImage2;
public:
	/**
	*open the port and intialise the module
	*/
	bool open(Searchable& config); 
	/**
	* try to interrupt any communications or resource usage
	*/
    bool interruptModule(); // 

	/**
	* catches all the commands that have to be executed when the module is closed
	*/
    bool close(); // 
	/**
	* update the module
	*/
	bool updateModule();
    /**
    * load a bitmap image
    */
	unsigned char* Load_Bitmap (int *X_Size, int *Y_Size, int *planes, char *filename){};
	unsigned char* Load_Bitmap (int *X_Size, int *Y_Size, int *planes, ImageOf<PixelRgb> *src){};
	unsigned char* Load_Bitmap (int *X_Size, int *Y_Size, int *planes, ImageOf<PixelMono> *src){};
	/**
	* save the image as bitmap
	*/
	void Save_Bitmap (unsigned char *image, int X_Size, int Y_Size, int planes,char *filename){};
};


#endif
