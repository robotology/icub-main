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
#include <yarp/os/Value.h>

// opencv
#include <opencv2/core.hpp>

// iCub
#include <iCub/ICalibTool.h>
#include <iCub/spherical_projection.h>


/**
 * Interface to calibrate and spherically project input image\n
 */
class SphericalCalibTool : public ICalibTool
{
private:

    cv::Mat         _mapX;
    cv::Mat         _mapY;

    double          _fx, _fx_scaled;
    double          _fy, _fy_scaled;
    double          _cx, _cx_scaled;
    double          _cy, _cy_scaled;
    double          _p1;
    double          _p2;
    double          _k1;
    double          _k2;

    bool _needInit;

    cv::Size        _calibImgSize;
    cv::Size        _oldImgSize;

    bool _drawCenterCross;

    bool init(cv::Size currImgSize, cv::Size calibImgSize);

public:

    SphericalCalibTool();
    virtual ~SphericalCalibTool();

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

 
 
 
