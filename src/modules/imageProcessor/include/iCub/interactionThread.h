// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef _INTERACTIONTHREAD_H_
#define _INTERACTIONTHREAD_H_

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

//class interactionThread: public yarp::os::RateThread
class interactionThread: public yarp::os::Thread
{
private:
    
    /**
    * IppiSize reference to the dimension of the input image
    */
    IppiSize srcsize;
    /**
    * a port for the inputImage
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inImagePort; // 
    /**
    * a port for the redPlane
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > redPlanePort; //
    /**
    * a port for the greenPlane
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > greenPlanePort; //
    /**
    * a port for the bluePlane
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > bluePlanePort; //	 
    /**
    * input port for the R+G- colour Opponency Map
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > rgPort; 
    /**
    * input port for the G+R- colour Opponency Map
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > grPort; 
    /**
    * input port for the B+Y- colour Opponency Map
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > byPort; 	
    /**
    *  output port for edges in R+G- colour Opponency Map
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > rgEdgesPort; 
    /**
    * output port for edges in G+R- colour Opponency Map
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > grEdgesPort; 
    /**
    * output port for edges in B+Y- colour Opponency Map
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > byEdgesPort;
    /**
    * overall edges port combination of maximum values of all the colorOpponency edges
    */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > edgesPort;
    
    /**
    * name of the module and rootname of the connection
    */
    std::string name;
    /**
    * flag that is set after the dimension of the images is defined
    */
    bool reinit_flag;
    /**
    * flag set when the interrputed function has already be called
    */
    bool interrupted;
    //_______________ private method  __________________________
    
public:
    /**
    * constructor of the thread
    */
    interactionThread();//:RateThread(THREAD_RATE){};
    /**
    * constructor of the thread
    */
    //processorThread(Property &op);
    /**
    * destructor of the thread
    */
    ~interactionThread();
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
    void setName(std::string name);
    /**
    * function that returns the name of the module
    * @param str string to be added
    * @return name of the module
    */
    std::string getName(const char* str);
    /**
    * opens all the ports necessary for the module
    * @return return whether the operation was successful
    */
    bool openPorts();
    /**
    * closes all the ports opened when the module started
    * @return return whether the operation was successful
    */
    bool closePorts();
    /**
    * streams out data on ports
    * @return return whether the operation was successful
    */
    bool outPorts();
    

    //______ public attributes________
    /**
    * width of images
    */
    int width;
    /**
    * height of images
    */
    int height;
    /**
    * input image reference
    */
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputImg;
    /**
    * input image reference
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono> *tmp;
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
    * temp images necessary for planes extraction
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono> *bluePlane; //
    /**
    * temp images necessary for planes extraction
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono> *redPlane; //
    /**
    * temp images necessary for planes extraction
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenPlane; //
    /**
    * temp images necessary for planes extraction
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono> *yellowPlane; //
    /**
    * yarp image of the composition of all the edges
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono>* edges_yarp;
    /**
    * flag on when the image is successfully acquired
    */
    int* redGreen_flag;
    /**
    * flag on when the image is successfully acquired
    */
    int* greenRed_flag;
    /**
    * flag on when the image is successfully acquired
    */
    int* blueYellow_flag;
};

#endif //_INTERACTIONTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
