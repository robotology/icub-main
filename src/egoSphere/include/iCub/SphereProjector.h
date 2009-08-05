// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __SPHEREPROJECTOR__
#define __SPHEREPROJECTOR__

#ifndef M_PI_2
#define M_PI_2	((float)(asin(1.0)))
#endif
#ifndef M_PI
#define M_PI	((float)(2*M_PI_2))
#endif

#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// yarp
#include <yarp/sig/Image.h>
#include <yarp/os/IConfig.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>

// opencv
#include <cv.h>

// iCub
#include <iCub/spherical_projection.h>

namespace iCub {
    namespace contrib{
        class SphereProjector;
    }
}

using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::contrib;
using namespace std;

/**
 * Project spherically calibrated camera images onto collective spherical image.
 */

class iCub::contrib::SphereProjector : public IConfig
{
 private:
    
    //ImageOf<PixelRgb>   _yrpImgBase;
    IplImage            *_mapx;
    IplImage            *_mapy;

    double          _fx, _fx_scaled;
    double          _fy, _fy_scaled;
    double          _cx, _cx_scaled;
    double          _cy, _cy_scaled;
    double          _k1, _k2, _p1, _p2;
    double          _fa, _fe, _ca, _ce;
    double          _azSpan;    // total image angle horizontal
    double          _elSpan;    // total image angle vertical 

    /*double          _pixPerDegXSphere;
    double          _pixPerDegYSphere;
    double          _pixPerDegXIn;
    double          _pixPerDegYIn;*/
    //CvMat           *_map;

    bool _needInit;

    CvSize          _calibImgSize;
    CvSize          _oldInSize;
    CvSize          _oldSphereSize;

    bool init(CvSize currImgSize, CvSize calibImgSize);
    bool initSphere(CvSize sphereSize);
    
public:

    SphereProjector();
    virtual ~SphereProjector();
    
    virtual bool open (Searchable &config);
    virtual bool close ();
    virtual bool configure (Searchable &config);

    //void project(const ImageOf<PixelRgb> &in, 
    //             ImageOf<PixelRgb> &sphere, 
    //             double azimuth,  // laengengrad
    //             double elevation,   // breitengrad
    //             double rotation);   
    void project(const Image &in, 
                 Image &sphere, 
                 double *R);

    /** Get horizontal angle of view (degrees)*/
    double getAzimuthSpan(){ return (_azSpan*180.0/M_PI);}
    /** Get vertical angle of view (degrees) */
    double getElevationSpan(){ return (_elSpan*180.0/M_PI);}
    double getImageCenterX() { return (_cx_scaled);}
    double getImageCenterY() { return (_cy_scaled);}
    double getImageFocalX() { return (_fx_scaled);}
    double getImageFocalY() { return (_fy_scaled);}

};


#endif

 
 
 
