// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>

#include <iCub/YarpImgRecv.h>
#include <iCub/MachineBoltzmann.h>

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>


#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Thread.h>

//IPP include//#include <ipp.h>


#include <string>


//using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace std;

/**
* Thread that reads the port and traslate the value into a colour
* The corrispective element in the output image is highlighted proportionally to the value read
* /author Francesco Rea
*/


class imageThread : public RateThread {
public:
	/**
	* default constructor of the imageThread class 
	*/
	imageThread():RateThread(50){};
	/**
	* constructor of the imageThread class
	* @param r rate of thread update
	*/
	imageThread(int r):RateThread(r){};
	/**
	* constructor of the imageThread class
	* @param r rate of thread update
	*/
	imageThread(int r,string name):RateThread(r){setName(name);};
    /**
	* initialise the thread
	*/
	virtual bool threadInit();
	/**
	* code that is executed after the thread starts
	* @param s is true if the thread started
	*/
	virtual void afterStart(bool s);
	
	/**
	* running code of the thread
	*/
    virtual void run();
	/**
	* code executed when the thread is released
	*/
    virtual void threadRelease();
	/**
	* returns the name of the thread
	*/
	string getName();
	/**
	* sets the name of the thread
	* @param name string contains the name of the thread
	*/
	void setName(string name);
	/** 
	* set the attribute options of class Property
	* @param options option to be set
	*/
	void setOptions(Property options); //set the attribute options of class Property
	/** 
	* function that istantiate the TrackerThread
	* @param property of the thread
	*/
	void istantiateThread(Property options); //set the attribute options of class Property
	
	/** 
	* function that sets the scaleFactorX
	* @param value new value of the scaleFactorX
	*/
	void setScaleFactorX(int value);
	
	/** 
	* function that sets the scaleFactorY
	* @param value new value of the scaleFactorY
	*/
	void setScaleFactorY(int value);
	
	/** 
	* function that set the number of the layer active 
	* @param value number of the layer actually active
	*/
	void setCurrentLayer(int value);
	
	/** 
	* function that set and reset the boolean for the control of just the eyes
	* @param value of the flag to be set
	*/
	void setJustEyes(bool value);
	
	/** 
	* function that set the  vector of the tracker thread
	* @param a first element of the vector
	* @param b second element of the vector
	* @param c third element of the vector
	*/
	void setTrackerVector(double a, double b, double c);
	/** 
	* function that set the left vector of the tracker thread
	*/
	void setTrackerLeftVector();
	/** 
	* function that set the right vector of the tracker thread
	*/
	void setTrackerRightVector();
	/** 
	* function that stop the tracker where it is
	*/
	void stopTracker();
	/**
	* function that sets the pointer to the related layer
	*/
	void setLayer(Layer* layer);
	/**
	* returns the yarp image
	*/
	ImageOf<PixelRgb>* getYarpImage();

	//----------ATTRIBUTES-----------------
	/** 
	* input Image which is mapped onto the selected layer
	*/
	ImageOf<PixelRgb> *ptr_inputImage;
	/** 
	* input Image which is mapped onto the selected layer
	*/
	ImageOf<PixelRgb> *ptr_inputImage2;
	/** 
	* number of layers already istantiated
	*/
	int countLayer;
	/**
	* layer which the image is represented
	*/
	Layer* plottedLayer;
	/** 
	 * option for the tracker thread
	 */
	Property options; 
    /** 
	 * input port for possible coming images
	 */
	BufferedPort<ImageOf<PixelRgb> > port; // input port for possible coming images
	/** 
	* port where the commands are vehiculated from controller to engine
	*/
	BufferedPort<yarp::os::Bottle> portCmd;
	/**
	* port plane
	*/
	BufferedPort<ImageOf<PixelMono>> port_plane; 
	/** 
	* counter for the update step
	*/
    int ct;
	/** 
	* scale factor for the output image representing a layer (X axis)
	*/
	int scaleFactorX;
	/** 
	* scale factor for the output image representing a layer (Y axis)
	*/
	int scaleFactorY;
	/** 
	* sinchronized with the number of layer active
	*/
	int currentLayer;
	/**
	*counter incremented inside the updateModule
	*/
	int count;  
	/**
	* temporary image
	*/
	ImageOf<PixelRgb> img_tmp; //=new ImageOf<PixelRgb>;
	
	ImageOf<PixelRgb> img; //=new ImageOf<PixelRgb>;
	ImageOf<PixelRgb> *img0; //=new ImageOf<PixelRgb>;
	/**
	* YARP image where all the objects are drawn sequentially
	*/
	ImageOf<PixelRgb> *image2; 
	/**
	* openCv image where all the objects are drawn sequentially
	*/
	IplImage *cvImage;
	/** 
	* flag that enable the drawing of the layer present in the simulation
	*/
	bool enableDraw;
	/**
	* name of the thread and name of the port
	*/
	string name;
};
