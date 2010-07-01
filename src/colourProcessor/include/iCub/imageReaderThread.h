// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef _IMAGEREADERTHREAD_H_
#define _IMAGEREADERTHREAD_H_

//YARP include
//os section
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Thread.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/BufferedPort.h>
//sig section
#include <yarp/sig/Image.h>

#include <string>

//IPP include
#include <ipp.h>

//const double TIMEOUT=0.1;
//const double STOP_TIME=3;
const int THREAD_RATE_IMAGE=30;

//class imageReaderThread: public yarp::os::RateThread
class imageReaderThread: public yarp::os::Thread
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
    * port where the ychannel of the image is streamed
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > yPort;
    /**
    * port where the uchannel of the image is streamed
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > uPort;
    /**
    * port where the vchannel plane of the image is streamed
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > vPort;
     /**
    * port where the uvchannel of the input image is streamed
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > uvPort;
    /***
    * temporary image mono for processing
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono>* tmp;
    /**
    * temporary psb for processing
    */
    int psb;
    /**
    * flag that indicates when the reinitiazation has already be done
    */
    bool reinit_flag;
    /**
    * input image
    */
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *img; 
    
    
    //_______________ private method  __________________________
    /**
    * function that streams the images out on the ports
    */
    void outPorts();
    /**
    * name of the module and rootname of the connection
    */
    std::string name;
    
public:
    /**
    * constructor of the thread
    */
    imageReaderThread();//:RateThread(THREAD_RATE){};
    /**
    * constructor of the thread
    */
    //processorThread(Property &op);
    /**
    * destructor of the thread
    */
    ~imageReaderThread();
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

    /**
    * function called when the module is poked with an interrupt command
    */
    void interrupt();
    /**
    * function that reinitiases some attributes of the class
    */
    void reinitialise(int width, int height);
    /**
    * function that gives reference to the name of the module
    * @param name of the module
    */
    void setName(const char* name);
    /**
    * function that returns the name of the module
    * @param str string to be added
    * @return name of the module
    */
    std::string getName(const char* str);

    //______ public attributes________
    /**
    * input image
    */
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputImg;
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
    *yarp mono image of the y channel (intensity information)
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono>* yPlane; 
    /**
    *yarp mono image of the u channel (chrominance information)
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono>* uPlane; 
    /**
    *yarp mono image of the v channel (chrominance information)
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono>* vPlane;
    /**
    *yarp mono image of the uv channel (chrominance information)
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono>* uvPlane;
};

#endif //_IMAGEREADERTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
