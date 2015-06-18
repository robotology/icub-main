// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __SPHERICALCALIBTOOL__
#define __SPHERICALCALIBTOOL__

// yarp
//#include <yarp/sig/Image.h>
#include <yarp/sig/all.h>
#include <yarp/os/IConfig.h>
#include <yarp/os/Value.h>

// opencv
#include <cv.h>

// iCub
#include <iCub/ICalibTool.h>
#include <iCub/spherical_projection.h>


/**
 * Interface to calibrate and spherically project input image\n
 */
class SphericalCalibTool : public ICalibTool
{
private:

    IplImage        *_mapX;
    IplImage        *_mapY;

    double          _fx, _fx_scaled;
    double          _fy, _fy_scaled;
    double          _cx, _cx_scaled;
    double          _cy, _cy_scaled;
    double          _p1;
    double          _p2;
    double          _k1;
    double          _k2;

    bool _needInit;

    CvSize          _calibImgSize;
    CvSize          _oldImgSize;

    bool _drawCenterCross;

    bool init(CvSize currImgSize, CvSize calibImgSize);

public:

    SphericalCalibTool();
    virtual ~SphericalCalibTool();

    // IConfig
    virtual bool open (yarp::os::Searchable &config);
    virtual bool close();
    virtual bool configure (yarp::os::Searchable &config);
    /** Stop module if there is an unread value */
    void stopConfig( std::string val );

    // ICalibTool
    void apply(const yarp::sig::ImageOf<yarp::sig::PixelRgb> & in,
               yarp::sig::ImageOf<yarp::sig::PixelRgb> & out);    
};


#endif

 
 
 
