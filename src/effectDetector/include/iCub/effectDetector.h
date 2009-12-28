// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino (VisLab/ISR/IST)
 */

#ifndef _EFFECTDETECTOR_
#define _EFFECTDETECTOR_

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/all.h>

//OpenCV
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <math.h>

#include <iCub/yarpTimer.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;


// ISNAN, from inkscape isnan.h, public domain code
// deal with some messes in the possible names of isnan
#if defined(__isnan)
# define isNaN(_a) (__isnan(_a))
#elif defined(__APPLE__) && __GNUC__ == 3
# define isNaN(_a) (__isnan(_a))    /* MacOSX/Darwin definition < 10.4 */
#elif defined(WIN32) || defined(_isnan)
# define isNaN(_a) (_isnan(_a))     /* Win32 definition */
#elif defined(isnan) || defined(__FreeBSD__) || defined(__osf__)
# define isNaN(_a) (isnan(_a))      /* GNU definition */
#elif defined (SOLARIS_2_8) && __GNUC__ == 3 && __GNUC_MINOR__ == 2
# define isNaN(_a) (isnan(_a))      /* GNU definition */
#else
# define isNaN(_a) (std::isnan(_a))
#endif




enum EffectDetectorState { PROCESSING, NOTPROCESSING }; //I thought I would need more states.



void on_mouse( int event, int x, int y, int flags, void* param );



class EffectDetector : public RFModule
{


  private:
    IplImage *buffer, *image, *hsv, *hue, *mask, *backproject, *histimg, *maskTEST;
    int backproject_mode;
    int show_hist;
    CvRect track_window;
    CvBox2D track_box;
    CvConnectedComp track_comp;
    int hdims;
    float hranges_arr[2];
    float* hranges;
	int _default_vmin;
    int _vmin, _vmax, _smin;    
    int bin_w,i,c;
    double refreshDelay;
    double timeToRefresh;

    bool newimage;
    bool newselection;
    bool onselection;
    bool ontracking;
    bool newtrackbar;		//trackbar changed position
    bool overlaydisplay; 

    ImageOf<PixelRgb> *in;
    ImageOf<PixelMono> out;
    Bottle *inRoi;
    


    //***********************************
    //*************MATTEO****************
    //***********************************
    Port initPort;                                    // Input/Output: initialization data, return value
    BufferedPort< ImageOf<PixelRgb> > rawCurrImgPort; // Input
    BufferedPort< ImageOf<PixelRgb> > rawSegmImgPort; // Input
    BufferedPort<Bottle> effectPort;                  // Output: (u,v) position
    BufferedPort<Bottle> errorPort;                   // Output: 0 means the module is working well, not(0) means there's some kind of problem.
    
    Bottle initMessage; //initialization message, received on /effectDetector/init
    ImageOf<PixelRgb> *yarpImg; //image as received by YARP
    IplImage *rawCurrImg, *rawSegmImg; //current image and image used for segmentation in OpenCV format
    IplImage *tempImg1, *tempImg2; //temporary images, used for conversions.
    CvHistogram *hist; //stores the color histogram of the tracked region
    EffectDetectorState state; //flag that tells the update method whether it should process the incoming stream of data or not
    int _w; //image width
    int _h; //image height

    float computeSimilarity(IplImage *rawSegmImg, IplImage *rawCurrImg, int u, int v, int width, int height);
    float simThreshold;
    bool firstImageEver;
    
  public:

    EffectDetector();
    ~EffectDetector();
    
    //virtual bool open(Searchable& config);
	virtual bool configure(ResourceFinder &rf); /* configure module parameters, return true if successful */

    virtual bool close();
    virtual bool interruptModule();
    CvScalar hsv2rgb( float hue );
    virtual bool updateModule();
    bool respond(const Bottle & command, Bottle & reply);
	virtual double getPeriod();

};

#endif /* _EFFECTDETECTOR_ */
