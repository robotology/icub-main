// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef _RGBPROCESSORTHREAD_H_
#define _RGBPROCESSORTHREAD_H_

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//IPP include
#include <ipp.h>

using namespace yarp::os;
using namespace yarp::sig;


const double TIMEOUT=0.1;
const double STOP_TIME=3;
const int THREAD_RATE=30;

class rgbProcessorThread: public RateThread
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
    * input image
    */
    ImageOf<PixelRgb> *img; 
    
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
    
    //_______________ private method  __________________________

    /**
    * function that extracts the blue plane
    */
    void getBluePlane(ImageOf<PixelRgb>* inputImage,ImageOf<PixelMono>* tmp);
    /**
    * function that extracts the red plane
    */
    void getRedPlane(ImageOf<PixelRgb>* inputImage,ImageOf<PixelMono>* tmp);
    /**
    * function that extracts the green plane
    */
    void getGreenPlane(ImageOf<PixelRgb>* inputImage,ImageOf<PixelMono>* tmp);
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
    void extractPlanes(ImageOf<PixelRgb>* inputImage);
    /**
    * set the reference to the input image
    */
    void setInputImage(ImageOf<PixelRgb>* inputImage);

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
    * yarp image for Opponency Map R+G-
    */
    ImageOf<PixelMono>* redGreen_yarp;
    /**
    * yarp image for Opponency Map G+R-
    */
    ImageOf<PixelMono>* greenRed_yarp;
    /**
    * yarp image for Opponency Map B+Y-
    */
    ImageOf<PixelMono>* blueYellow_yarp;
};

#endif //_RGBPROCESSORTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
