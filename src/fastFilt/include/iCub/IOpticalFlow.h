// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#ifndef __UZH_IOPTICALFLOW__
#define __UZH_IOPTICALFLOW__

// cv
#include <cv.h>        
        
namespace iCub {
    namespace contrib {
        class IOpticalFlow;
    }
}

/**
* Interface for optical flow implementations.
*/

class iCub::contrib::IOpticalFlow {
    
    public: 

	/**
	 * Calculate optical flow.
	 * @param imageT Grayscale image at timestep t
	 * @param imageTMinus 1 Grayscale image at timestep t-1
	 * @param velx Floating point image containing x components of calculated flow
	 * @param vely Floating point image containing y components of calculated flow
	 * @param absQuad Floating point image containing squared absolute flow 
	 */
    virtual void calculate_flow(IplImage* imageT, IplImage* imageTMinus1, IplImage* velx, IplImage* vely, IplImage* abs=0) = 0;
    
    /**
     * Draw flow to image.
     * @param image Image to draw on (bgr format)
     * @param rgbX X flow component converted to rgb (scaled r=b=g)
     * @param rgbY Y flow component converted to rgb (scaled r=b=g)
     */
    virtual void draw_flow(IplImage *image, IplImage *rgbX, IplImage *rgbY) = 0;

};

#endif

