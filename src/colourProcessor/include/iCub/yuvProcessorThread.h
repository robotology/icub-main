// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef _YUVPROCESSORTHREAD_H_
#define _YUVPROCESSORTHREAD_H_

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//IPP include
#include <ippi.h>

using namespace yarp::os;
using namespace yarp::sig;



const int THREAD_RATE_YUV=30;

const double yr=0.299;const double yg=0.587;const double yb=0.114;
const double ur=-0.147;const double ug=-0.289;const double ub=0.436;
const double vr=0.615;const double vg=-0.515;const double vb=-0.1;


class yuvProcessorThread: public RateThread
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
    /*
    * pointers to channel for processing
    */
    unsigned char *pr,*pg,*pb,*py,*pu,*pv,*puv;
    /**
    * reference to the temporary 3 planes for extraction
    */
    Ipp8u* shift[3];
    /***
    * temporary image mono for processing
    */
    ImageOf<PixelMono>* tmp;
    /**
    * temporary psb for processing
    */
    int psb;
    /**
    * flag that indicates when the reinitiazation has already be done
    */
    bool reinit_flag;
    /**
    * ipp reference to the dimension of input images
    */
    IppiSize srcsize;
    
    //_______________ private method  __________________________

    /**
    * function that extracts the y channel
    */
    void getYPlane(ImageOf<PixelMono>* tmp);
    /**
    * function that extracts the u channel
    */
    void getUPlane(ImageOf<PixelMono>* tmp);
    /**
    * function that extracts the v channel
    */
    void getVPlane(ImageOf<PixelMono>* tmp);
    /**
    * function that adds U and V channels
    */
    void addUVPlanes();
public:
    /**
    * constructor of the thread
    */
    yuvProcessorThread(); //:RateThread(THREAD_RATE){};
    /**
    * constructor of the thread
    */
    //processorThread(Property &op);
    /**
    * destructor of the thread
    */
    ~yuvProcessorThread();
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
    * function that reinitiases some attributes of the class
    */
    void reinitialise();
    /**
    * function that extracts planes from the input image
    */
    void extractYUV();
    
    /**
    * set the reference to the input image
    */
    void setInputImage(ImageOf<PixelMono>* redImage,ImageOf<PixelMono>* greenImage,ImageOf<PixelMono>* blueImage);

    //______ public attributes________

    /**
    *yarp mono image of the red channel
    */
    ImageOf<PixelMono>* redPlane; 
    /**
    *yarp mono image of the green channel
    */
    ImageOf<PixelMono>* greenPlane; 
    /**
    *yarp mono image of the blue channel
    */
    ImageOf<PixelMono>* bluePlane; 
     /**
    *yarp mono image of the y channel (intensity information)
    */
    ImageOf<PixelMono>* yPlane; 
    /**
    *yarp mono image of the u channel (chrominance information)
    */
    ImageOf<PixelMono>* uPlane; 
    /**
    *yarp mono image of the v channel (chrominance information)
    */
    ImageOf<PixelMono>* vPlane;
    /**
    *yarp mono image of the uv channel (chrominance information)
    */
    ImageOf<PixelMono>* uvPlane;
};

#endif //_YUVPROCESSORTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
