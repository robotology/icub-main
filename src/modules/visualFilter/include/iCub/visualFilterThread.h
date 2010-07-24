// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _VISUAL_FILTER_THREAD_H_
#define _VISUAL_FILTER_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <iostream>

// outside project includes
#include <ippi.h>


class visualFilterThread : public yarp::os::Thread
{
private:

    /* class variables */

    IppiSize srcsize; //ROI for the images in the processing
    
    int psb;

    int width_orig, height_orig; //dimension of the input image (original)

    int width, height; //dimension of the extended input image (extending)
    
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputImage;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputExtImage;
        
    yarp::sig::ImageOf<yarp::sig::PixelMono> *redPlane; //image of the red channel

    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenPlane; //image of the green channel

    yarp::sig::ImageOf<yarp::sig::PixelMono> *bluePlane; //image of the blue channel

    yarp::sig::ImageOf<yarp::sig::PixelMono> *yellowPlane; //image of the blue channel

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortIn;     // input port

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imagePortOut;     // output port   

    std::string name; //rootname of all the ports opened by this thread

   
public:

    /**
    * constructor
    */
    visualFilterThread();

    bool threadInit();     
    void threadRelease();
    void run(); 
    void onStop();

    /**
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
    * function that resizes the necessary and already allocated images
    * @param width width of the input image
    * @param height height of the input image
    */
    void resize(int width, int height);

    /**
    * extending logpolar input image in fovea and theta axis
    */
    void extending();

    /**
    * function that extendes the original image of the desired value for future convolutions (in-place operation)
    * @param origImage originalImage
    * @param extDimension dimension of the extention on each of the sides of the image
    */
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* extender(yarp::sig::ImageOf<yarp::sig::PixelRgb>* origImage,int extDimension); 

    /**
    * extracting RGB and Y planes
    */
    void extractPlanes();

    /**
    * gaussing filtering of the of RGBY
    */
    void filtering();

    /**
    * function which constructs colourOpponency maps
    */
    void colourOpponency();

    /**
    * applying sobel operators on the colourOpponency maps and combining via maximisation of the 3 edges
    */
    void edgesExtract();

};

#endif  //_VISUAL_FILTER_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
