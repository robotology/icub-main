//#include <ace/config.h>
//#include <ace/OS.h>
//#include <ace/Log_Msg.h>


#include <yarp/os/all.h>
/*
#include <yarp/os/PortablePair.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Terminator.h>
#include <yarp/os/RateThread.h>*/

#include <yarp/sig/all.h>
//#include <yarp/String.h>

//primatevision include
#include "multiclass.h"
#include <iCub/convert_rgb.h>
using namespace iCub::contrib::primateVision;

//using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
//using namespace yarp;

const int THREAD_RATE=30;


class classifierThread: public RateThread
{
	/**
	* reference to the multiclassifier
	*/
	MultiClass* m;
	/**
	* converter to RGB
	*/
	Convert_RGB* ci;
	/**
	* output image (always 320,240)
	*/
	ImageOf<PixelMono> *imgOut; 
	/**
	* the image that represent everything not of class A
	*/
	ImageOf<PixelMono> *imageOther;
	/**
	* the image that represent the class A
	*/
	ImageOf<PixelMono> *imageClassA;
	/**
	* size of the output
	*/
	IppiSize isize;
	/**
	* edge map reference
	*/
	Ipp8u *edge_map;
	/**
	* input image
	*/
	Ipp8u * in;
	/**
	* input image (always 320,240)
	*/
	ImageOf<PixelRgb> *imgInput;
	
	/**
	* probability map image Other (always 320,240)
	*/
	ImageOf<PixelMono> *imgProbOther;
	/**
	* series of parameters for the multiclass object
	*/
	MultiClass::Parameters properties;
public:
	/**
	* constructor of the thread
	*/
	classifierThread(Property &op);
	/**
	* destructor of the thread
	*/
	~classifierThread();
	/**
	*	initialization of the thread 
	*/
	bool threadInit();
	/**
	* active loop of the thread
	*/
	void run();
	/**
	*	releases the thread
    */
	void threadRelease();
	//----------- public ATTRIBUTES -------------
	/**
	* probability map image (always 320,240)
	*/
	ImageOf<PixelMono> *imgProbClassA; 
};