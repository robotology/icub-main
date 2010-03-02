// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _COLORVQ_H_
#define _COLORVQ_H_

#include <ace/config.h>

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

//within Project Include
//#include <iCub/ImageProcessor.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;



// bounding box type.
class ColorVQ {
    ImageOf<PixelInt> tmp1;
    ImageOf<PixelInt> tmp2;
    
    int height, width;

public:
    
    ColorVQ() {}
    ColorVQ(int x, int y, int fovea) { Resize(x, y, fovea); }
    ~ColorVQ() {}

    void Resize(int x, int y, int fovea);
    void Variance(ImageOf<PixelMono> &src, ImageOf<PixelInt> &dst, int size);
    void Compactness(ImageOf<PixelMono> &src, int fovea, int val, int eps);
    void DominantQuantization(ImageOf<PixelBgr> &src, ImageOf<PixelBgr> &dst, unsigned char t);
    void DominantQuantization(PixelBgr src, PixelBgr &dst, unsigned char t);
};



#endif //_COLORVQ_H_


//----- end-of-file --- ( next line intentionally left blank ) ------------------
