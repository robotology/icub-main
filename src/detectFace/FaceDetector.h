#pragma once

#include "Sensor.h"

//=============================================================================
// YARP Includes - Class Specific
//=============================================================================
#include <yarp/sig/Image.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
//#include <conio.h>
#include <simio.h> // more portable
//#include "YARPImgRecv.h"
#include "cv.h"
#include "highgui.h"

#include <yarp/os/Network.h> 


#include "CFaceDetect.h"


#define _WINSOCKAPI_
#define YARP_CVTYPES_H_


using namespace yarp::os;
using namespace yarp::sig;







class FaceDetector:public Sensor
{
	private:
		CFaceDetect* facedetector;


	public:
		
		FaceDetector(int,int);
		~FaceDetector();
		int get_nr_of_features();
		int try_detect_features(IplImage *img);
		
		
		void setOptionsToDefault();
		bool getImage();



		yarp::os::Semaphore _semaphore;
		yarp::sig::ImageOf<yarp::sig::PixelRgb> _inputImg;

		CvSize imgSize;
		IplImage *img2;
		
};

