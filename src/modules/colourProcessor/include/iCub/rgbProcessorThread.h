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

const double yr=0.299;const double yg=0.587;const double yb=0.114;
const double ur=-0.147;const double ug=-0.289;const double ub=0.436;
const double vr=0.615;const double vg=-0.515;const double vb=-0.1;


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
    
    int psb; //temporary psb for processing
    int psb2; //temporary psb for processing

    
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputPort; //port where the input image is read from
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
    
    /**
    * rootname of the module
    */
    std::string name;
    /*
    * pointers to channels for processing
    */
    unsigned char *pr,*pg,*pb,*py,*pu,*pv,*puv;


    Ipp8u *redPlane_ippi,*greenPlane_ippi,*bluePlane_ippi,*yellowPlane_ippi;
    Ipp8u *redGreen_ippi,*greenRed_ippi,*blueYellow_ippi;
    Ipp8u *bluePlane_ippi_f,*redPlane_ippi_f,*yellowPlane_ippi_f,*greenPlane_ippi_f;

    Ipp32f *redPlane_ippi32,*bluePlane_ippi32,*greenPlane_ippi32;
	Ipp32f *redGreen_ippi32,*blueYellow_ippi32,*greenRed_ippi32;

	Ipp32f *redPlane_ippi32_f,*bluePlane_ippi32_f,*greenPlane_ippi32_f,*yellowPlane_ippi32_f;
    Ipp8u *redPlane_ippi8u_f,*bluePlane_ippi8u_f,*yellowPlane_ippi8u_f,*greenPlane_ippi8u_f;

    Ipp8u *pBufferBlue;
    Ipp8u *pBufferRed;
    Ipp8u *pBufferGreen;
        

        
       
    
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
    * function that extracts the y channel
    */
    void getYPlane(yarp::sig::ImageOf<yarp::sig::PixelMono>* tmp);
    /**
    * function that extracts the u channel
    */
    void getUPlane(yarp::sig::ImageOf<yarp::sig::PixelMono>* tmp);
    /**
    * function that extracts the v channel
    */
    void getVPlane(yarp::sig::ImageOf<yarp::sig::PixelMono>* tmp);
    /**
    * function that adds U and V channels
    */
    void addUVPlanes();
    /**
    * function that calculates the colour opponency maps as differences of Gaussians
    */
    void colourOpponency();
    /**
    * function that extracts the YUV images from the RGB images
    */
    void extractYUV();
    
    
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
