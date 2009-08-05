// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __BLOBFUNCTIONS__
#define __BLOBFUNCTIONS__

// std
#include <iostream>
#include <float.h>

// opencv
#include <cv.h>

// cvBlobs
#include <Blob.h>
#include <BlobResult.h>

namespace iCub {
    namespace contrib {
        class BlobFunctions;
    }
}

using namespace iCub::contrib;

class iCub::contrib::BlobFunctions {

public:
    /** Filters for the blob with largest area */
  	static void filterBiggest(CBlobResult *blobs);
  	/** Filters for the blob closest to target point */
  	static void filterClosest(CBlobResult *blobs, CvPoint target);
  	/** Filters for one random blob */
  	static void filterRandom(CBlobResult *blobs);

    /** Convert to grayscale image from floating point image */
    static void    grayFromFloat(IplImage* imgFloat, IplImage* imgGray, float scale);
    /** Draw a black border around given image 
     * @param imgBgr Image to draw on. Can be of IPL_DEPTH_8U 
     * with any number of channels or of IPL_DEPTH_32F
     * @param borderSize Border width in pixels
     */
	static void	   makeBlackBorder(IplImage* imgBgr, int borderSize);
};

#endif
