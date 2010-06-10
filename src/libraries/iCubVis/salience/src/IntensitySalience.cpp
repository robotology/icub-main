// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <yarp/sig/all.h>
#include <iCub/vis/IntensitySalience.h>

using namespace yarp::sig;
using namespace yarp::os;
using namespace iCub::vis;

IntensitySalience::IntensitySalience(){
    _imgGray = NULL;
}

bool IntensitySalience::open(yarp::os::Searchable& config){
    bool ok = Salience::open(config);
    if (!ok)
        return false;
    return true;
}


void IntensitySalience::applyImpl(ImageOf<PixelRgb>& src, 
                          ImageOf<PixelRgb>& dest,
                          ImageOf<PixelFloat>& sal) {
    dest.resize(src);
    sal.resize(src);
    if (_sizeOld.width != src.width() || 
        _sizeOld.height != src.height())
        initImages(cvSize(src.width(), src.height()));
    
    cvCvtColor(src.getIplImage(), _imgGray, CV_RGB2GRAY);
    cvCvtColor(_imgGray, dest.getIplImage(), CV_GRAY2RGB);
    gray2Float(_imgGray, (IplImage*)sal.getIplImage());

    _sizeOld.width = src.width();
    _sizeOld.height = src.height();
}

void IntensitySalience::initImages(CvSize size){
    releaseImages();
    _imgGray = cvCreateImage(size, IPL_DEPTH_8U, 1);
}

void IntensitySalience::releaseImages(){
    if(_imgGray != NULL)
        cvReleaseImage(&_imgGray);
    _imgGray = NULL;
}

void IntensitySalience::gray2Float(IplImage *imgGray, IplImage *imgFloat){
    uchar* src = (uchar*)imgGray->imageData;
    float* dst = (float*)imgFloat->imageData;
    int pos;
    for (int y = 0; y < imgFloat->height; y++){
        pos = imgFloat->width * y;
        for (int x = 0; x < imgFloat->width; x++){
            dst[pos+x] = (float)src[pos+x];   
        }
    }
}
