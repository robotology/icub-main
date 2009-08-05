// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_IOR_INC
#define ICUB_IOR_INC

// std
#include <stdio.h>
#include <iostream>
#include <string>
#include <math.h>

// cv
#include <cv.h>

// yarp
#include <yarp/sig/all.h>
#include <yarp/os/Value.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/IConfig.h>

namespace iCub {
    namespace contrib {
        class IOR;
    }
}

using namespace std;

/**
 * Inhibition of return for visual scenes.\n
 * 
 * IOR is used to modify saliency maps according to previously attended locations
 * 
 * IOR updates two internal floating point maps with range 0...1.0.\n
 * When calling updateIOR(gazeX, gazeY) the attention map increases values near the specified
 * attended location by adding values from a gaussian kernel to this area. 
 * The attention decay rate specifies the decrease of accumulated values.
 * The exact update rule is:\n
 * attMap(x,y) = attMap(x,y) + attDecay*(attIncrease - attMap(x,y))\n
 * where attIncrease is the value of the kernel at (x,y).\n
 * The kernel diameter and the decay rate can be configured; see terminal output
 * for exact syntax.\n
 * The second map, the IOR map, is initialized to 1.0 and acts like an operator on the passed (saliency) map 
 * when calling applyIOR(). The IOR map starts to contain values < 1.0 if a location in 
 * the attention map reaches a specified threshold. In this case the attention map 
 * 'fires' and an inverse gaussian kernel is subtracted from the IOR map
 * at that location (the kernel is the same as the one used for the attention map update described above).
 * The saliency map passed to applyIOR is updated by multiplying each pixel with the
 * corresponding value of the IOR map: S(x,y) = S(x,y)*IOR(x,y).
 * In each step the IOR map 'grows' back to 1.0 according to the IOR decay parameter.
 */
class iCub::contrib::IOR : public yarp::os::IConfig {
private:
    bool debug;
    int width, height; // size of maps
    float attentionDiameterFraction;
    int attDiam;   // diameter of attention
    int attRad;    // diameter of attention / 2  --> e.g. 5/2 = 2 (!)
    float attDecay;
    float iorDecay;
    float threshold;
    float **iorKernel;
    yarp::sig::ImageOf<yarp::sig::PixelFloat> attMap; // map of attended locations
    yarp::sig::ImageOf<yarp::sig::PixelFloat> iorMap; // operator for applyIOR, everywhere 1.0 except for the 'holes' for 'attentional-saturated' regions
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > prtAttMap;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > prtIorMap;
    /* add a 'hole' to the passed map (upended iorKernel) */
    virtual void addIORKernel(yarp::sig::ImageOf<yarp::sig::PixelFloat> &map, int iorX, int iorY);
    virtual void init(int w, int h);
    virtual void release();
    virtual void createGaussianFilter2D(float **filter, int sizex, int sizey, float ex, float ey, bool normalize);
    virtual void float2Rgb(IplImage* imgFloat, IplImage* imgRgb, float scaleFactor);
public:
    IOR();
    virtual ~IOR();

	/**
	 * add an inhibition region manually at location (x,y)
	 */
	virtual bool addIORRegion(int x, int y);

    /** 
     * Apply the internal IOR map to a saliency map.\
     * (The first time applyIOR is called the internal maps are initialized according
     * to the passed map's width and height.)
     */
    virtual bool applyIOR(yarp::sig::ImageOf<yarp::sig::PixelFloat> &map);
    /** 
     * Updates attention and IOR map.
     * Calculates decay and adds optionally a new gaze point location.\n
     * Before calling updateIOR you have to call applyIOR at least once to 
     * initialize internal maps, otherwise updateIOR won't do anything.
     */
    virtual bool updateIOR(int gazeX=-1, int gazeY=-1);

    // IConfig
    virtual bool open(yarp::os::Searchable &config);
    virtual bool close();

	// reset maps
	virtual bool reset();
};
		
#endif

