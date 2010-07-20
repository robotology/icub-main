// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef _RGBPROCESSORTHREAD_H_
#define _RGBPROCESSORTHREAD_H_

//YARP include
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/Thread.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>
#include <yarp/sig/Image.h>

//IPP include
#include <ipp.h>
#include <string>


const double TIMEOUT=0.1;
const double STOP_TIME=3;
const int THREAD_RATE=30;

class rgbProcessorThread: public yarp::os::Thread
{
private:
    /**
    * width of images
    */
    int width;
    /**
    * height of images
    */
    int height;
    /**
    * IppiSize reference to the dimension of the input image
    */
    IppiSize srcsize;
    /**
    * reference to the temporary 3 planes for extraction
    */
    Ipp8u* shift[3];
    /***
    * temporary image mono for processing
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono>* tmp;
    /**
    * temporary psb for processing
    */
    int psb;
    /**
    * port where the input image is read from
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputPort;
    /**
    * port where the red plane of the image is streamed
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > redPort;
    /**
    * port where the green plane of the image is streamed
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > greenPort;
    /**
    * port where the blue plane of the image is streamed
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > bluePort;
    /**
    * port where the difference of gaussian R+G- is streamed
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > rgPort;
    /**
    * port where the difference of gaussian G+R- is streamed
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > grPort;
    /**
    * port where the difference of gaussian B+Y- of the image is streamed
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > byPort;
    /**
    * port where the yellow plane of the image is streamed
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > yellowPort;
     
    /**
    * temp variable for the plane extraction
    */
    Ipp32f* redPlane_ippi32_f; //
    /**
    * temp variable for the plane extraction
    */
    Ipp32f* bluePlane_ippi32_f; //
    /**
    * temp variable for the plane extraction
    */
    Ipp32f* yellowPlane_ippi32_f; //
    /**
    *temp variable for the plane extraction
    */
    Ipp32f* greenPlane_ippi32_f; //
    /**
    * rootname of the module
    */
    std::string name;
    
    //_______________ private method  __________________________

    /**
    * function that extracts the blue plane
    */
    void getBluePlane(yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage,yarp::sig::ImageOf<yarp::sig::PixelMono>* tmp);
    /**
    * function that extracts the red plane
    */
    void getRedPlane(yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage,yarp::sig::ImageOf<yarp::sig::PixelMono>* tmp);
    /**
    * function that extracts the green plane
    */
    void getGreenPlane(yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage,yarp::sig::ImageOf<yarp::sig::PixelMono>* tmp);
    /**
    * function that calculates the colour opponency maps as differences of Gaussians
    */
    void colourOpponency();
    
public:
    /**
    * constructor of the thread
    */
    rgbProcessorThread();//:RateThread(THREAD_RATE){};
    /**
    * constructor of the thread
    */
    //processorThread(Property &op);
    /**
    * destructor of the thread
    */
    ~rgbProcessorThread();
    /**
    *	initialization of the thread 
    */
    bool threadInit();
    /**
    * active loop of the thread
    */
    void run();
    /**
    * function called when the module is interrupted
    */
    void interrupted();
    /**
    *	releases the thread
    */
    void threadRelease();
    /**
    * function that reinitiases some attributes of the class
    *@param width width of the image
    *@param height height of the image
    */
    void reinitialise(int width, int height);
    /**
    * function that extracts planes from the input image
    */
    void extractPlanes(yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage);
    /**
    * set the reference to the input image
    */
    void setInputImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage);
    /**
    *  function that sends images on the output ports
    */
    void outPorts();
    /**
    * function that gives reference to the name of the module
    * @param name of the module
    */
    void setName(std::string name);
    /**
    * function that returns the name of the module
    * @param str string to be added
    * @return name of the module
    */
    std::string getName(const char* str);
    //______ public attributes________
    /**
    * flag that indicates when the reinitiazation has already be done
    */
    bool reinit_flag;
    /**
    * flag that is set when interrupted (blocking port)
    */
    bool interrupted_flag;
    /**
    *yarp mono image of the red channel
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono>* redPlane; 
    /**
    *yarp mono image of the green channel
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono>* greenPlane; 
    /**
    *yarp mono image of the blue channel
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono>* bluePlane; 
    /**
    * yarp image for Opponency Map R+G-
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono>* redGreen_yarp;
    /**
    * yarp image for Opponency Map G+R-
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono>* greenRed_yarp;
    /**
    * yarp image for Opponency Map B+Y-
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono>* blueYellow_yarp;
    /**
    * input image
    */
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *img;
    /**
    * input image
    */
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputImg;
};

#endif //_RGBPROCESSORTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
