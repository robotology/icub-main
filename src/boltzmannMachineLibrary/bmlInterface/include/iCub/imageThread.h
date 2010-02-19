// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef _IMAGETHREAD_H_
#define _IMAGETHREAD_H_

#include <stdio.h>

//openCV include
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>


//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>


#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Thread.h>

#include <iCub/YARPImgRecv.h>



//IPP include
//#include <ipp.h>


#include <string>


//using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace std;


/**
* Thread that reads the port and traslate the value into a colour
* The corrispective element in the output image is highlighted proportionally to the value read
* @author Francesco Rea
*/
class imageThread : public RateThread {
public:
    /**
    * default constructor of the imageThread class
    * 
    */
    imageThread():RateThread(50){};

    /**
    * constructor of the imageThread class
    * @param r rate of thread update
    */
    imageThread(int r):RateThread(r){};

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
     * option for the tracker thread
     */
    Property options; 

    /** 
     * input port for possible coming images
     */
    BufferedPort<ImageOf<PixelRgb> > port; // input port for possible coming images
    /** 
    * output port n°0 for writing image out
    */
    BufferedPort<ImageOf<PixelRgb> > port0; //port for writing the image out
    /** 
    * output port n°1 for writing image out
    */
    BufferedPort<ImageOf<PixelRgb> > port1; //port for writing the image out
    /** 
    * output port n°2 for writing image out
    */
    BufferedPort<ImageOf<PixelRgb> > port2; //port for writing the image out
    /** 
    * output port n°3 for writing image out
    */
    BufferedPort<ImageOf<PixelRgb> > port3; //port for writing the image out
    /** 
    * port where the commands are vehiculated from controller to engine
    */
    BufferedPort<yarp::os::Bottle> portCmd;
    
    BufferedPort<ImageOf<PixelMono> > port_plane; 
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
    * temporary RGB image
    */
    ImageOf<PixelRgb> img_tmp; //=new ImageOf<PixelRgb>;
    /**
    * temporary RGB image
    */
    ImageOf<PixelRgb> img; //=new ImageOf<PixelRgb>;
    /**
    * temporary RGB image
    */
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

};

#endif //IMAGETHREAD

//----- end-of-file --- ( next line intentionally left blank ) ------------------
