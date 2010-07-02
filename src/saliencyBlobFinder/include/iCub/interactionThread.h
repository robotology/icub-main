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

#include <time.h>
#include <string>

//IPP include
#include <ipp.h>

//within project include
#include <iCub/blobFinderThread.h>

//const double TIMEOUT=0.1;
//const double STOP_TIME=3;
const int INT_THREAD_RATE=30;

class interactionThread: public yarp::os::Thread
{
private:
    /**
    * execution step counter
    */
    int ct;
    /**
    * IppiSize reference to the dimension of the input image
    */
    IppiSize srcsize;
    /**
    * port used for centroid position to controlGaze2
    */
    BufferedPort<Bottle> centroidPort;
    /**
    * port used for centroid position to iKinHead
    */
    Port triangulationPort;
    /**
    * port used for sending responses from triangulationPort back into i
    */
    BufferedPort<Bottle> gazeControlPort;
    /**
    * port where the input image is read from
    */
    BufferedPort<ImageOf<PixelRgb> > inputPort;
    /**
    * port where the red plane of the image is streamed
    */
    BufferedPort<ImageOf<PixelMono> > redPort;
    /**
    * port where the green plane of the image is streamed
    */
    BufferedPort<ImageOf<PixelMono> > greenPort;
    /**
    * port where the blue plane of the image is streamed
    */
    BufferedPort<ImageOf<PixelMono> > bluePort;
    /**
    * port where the difference of gaussian R+G- is streamed
    */
    BufferedPort<ImageOf<PixelMono> > rgPort;
    /**
    * port where the difference of gaussian G+R- is streamed
    */
    BufferedPort<ImageOf<PixelMono> > grPort;
    /**
    * port where the difference of gaussian B+Y- of the image is streamed
    */
    BufferedPort<ImageOf<PixelMono> > byPort;
    /**
    * port where the yellow plane of the image is streamed
    */
    BufferedPort<ImageOf<PixelMono> > yellowPort;
    /**
    * port that returns the image output
    */
    BufferedPort<ImageOf<PixelRgb> > outputPort;
    /**
    * buffer image for received image
    */
    ImageOf<PixelMono> *tmpImage;
    /**
    * main thread responsable to process the input images and produce an output
    */
    blobFinderThread* blobFinder;
    /**
    * name of the module and rootname of the connection
    */
    std::string name;
    /**
    * flag that is set after the dimension of the images is defined
    */
    bool reinit_flag;
    /*
    * flag that indicates when the thread has been interrupted
    */
    bool interrupted_flag;
    /**
    * time variable
    */
    time_t startTimer;
    /**
    * time variable
    */
    time_t endTimer;
    /**
    * position of the centroid X in the previous time instant
    */
    int previous_target_x;
    /**
    * position of the centroid Y in the previous time instant
    */
    int previous_target_y;
    /**
    * position of the target in the body-centered frame (x coordinate)
    */
    double target_x;
    /**
    * position of the target in the body-centered frame (y coordinate)
    */
    double target_y;
    /**
    * position of the target in the body-centered frame (z coordinate)
    */
    double target_z;
    
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
    /**
    * function that reads the ports for colour RGB opponency maps
    */
    bool getOpponencies();
    /**
    * function that reads the ports for the RGB planes
    */
    bool getPlanes();
    

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
    * reference to the input image
    */
    ImageOf<PixelRgb> *img;
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
    /**
    * it indicates if the control of the time is on
    */
    bool timeControl_flag;
    /**
    *   number spikes that have to be counted before the maxSaliency blob can be choosen
    */
    int countSpikes;
    /**
    * dispacement on the x axis for the target
    */
    int xdisp;
    /**
    * dispacement on the y axis for the target
    */
    int ydisp;
};

#endif //_INTERACTIONTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
