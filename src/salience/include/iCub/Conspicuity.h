// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_CONSPICUITY_INC
#define ICUB_CONSPICUITY_INC

// std
#include <stdio.h>
#include <iostream>
#include <string>
#include <math.h>

// opencv
#include <cv.h>
//#include <highgui.h> //not needed?, and people having problems -paulfitz

// iCub

// yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

namespace iCub {
    namespace contrib {
        class Conspicuity;
    }
}

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;

/**
 * Conspicuity calculation. Uses a gaussian pyramid to build a pyramid 
 * of center-surround difference maps (CSD-Maps). The number of center-surround 
 * levels can be specified as a configuration parameter. CSD level 'i' is built
 * from gaussian pyramid level i and i + 2 (D(i) = G(i) - G(i+2)).
 * This 'standalone' implementation of conspicuity might be considered
 * for cases where the input map describes the 'presence' of some feature/value
 * (e.g. intensity).
 */
class iCub::contrib::Conspicuity : yarp::os::IConfig {

private:

    //BufferedPort<ImageOf<PixelRgb> >  _prtRgb;

    float               MAX_SALIENCE;

    // Images
    int                 _sizePyr;
    int                 _sizeConsp;
    IplImage            *_imgDstTmp;
    IplImage			**_imgPyr;	
    IplImage            **_imgConsp;
    IplImage            **_imgPyrTmp;
    float               *_fltConspFactors;
    float               _globalMax;
    
    CvSize              _sizeOld;
    bool                _needInit;

    bool initImages(int w, int h);
    void releaseImages();
    void calcConspicuityFactor(IplImage *img, float &factor);
    inline void calcGlobalMaximum(IplImage *img, float &max){
        float* data = (float*)img->imageData;
        int posRow, pos;
        max = 0.0f;
        for (int y = 0; y < img->height; y++){
            posRow = img->width * y;
            for (int x = 0; x < img->width; x++){
                pos = posRow + x;
                if (data[pos] > max){
                    max = data[pos];
                }
            }
        }
    }
    inline void float2Rgb(IplImage *imgFloat, IplImage *imgRgb){
        int arraypos = 0;
        for (int y = 0; y < imgRgb->height; y++){
            arraypos = imgRgb->widthStep*y;
            for (int x = 0; x < imgRgb->width; x++){
                ((uchar*)(imgRgb->imageData + imgRgb->widthStep*y))[x*3 + 0] = (uchar)((float*)(imgFloat->imageData + imgFloat->widthStep * y))[x];
				((uchar*)(imgRgb->imageData + imgRgb->widthStep*y))[x*3 + 1] = (uchar)((float*)(imgFloat->imageData + imgFloat->widthStep * y))[x];
				((uchar*)(imgRgb->imageData + imgRgb->widthStep*y))[x*3 + 2] = (uchar)((float*)(imgFloat->imageData + imgFloat->widthStep * y))[x];
            }
        }
    }
    inline void normalizeArray(float *arr, int size);

public:
    Conspicuity();
    virtual ~Conspicuity();

    virtual void apply(IplImage *src, IplImage *dst);

    // IConfig
    virtual bool open(yarp::os::Searchable& config);
    //virtual bool configure(yarp::os::Searchable& config);
    virtual bool close();

};

#endif
