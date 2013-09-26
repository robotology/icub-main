// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Lijin Aryananda, Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __PINHOLECALIBTOOL__
#define __PINHOLECALIBTOOL__

#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// opencv
#include <cv.h>

// yarp
//#include <yarp/sig/Image.h>
#include <yarp/sig/all.h>
#include <yarp/os/IConfig.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>

// iCub
#include <iCub/ICalibTool.h>


/**
 * Class to calibrate input image based on camera's internal parameters\n
 * Configuration: See PinholeCalibTool::configure
 */

class PinholeCalibTool : public ICalibTool
{
 private:
    
    CvMat           *_intrinsic_matrix;
    CvMat           *_intrinsic_matrix_scaled;
    CvMat           *_distortion_coeffs;;

    IplImage        *_mapUndistortX;
    IplImage        *_mapUndistortY;

    bool _needInit;

    CvSize          _calibImgSize;
    CvSize          _oldImgSize;

    bool _drawCenterCross;

    bool init(CvSize currImgSize, CvSize calibImgSize);

public:

    PinholeCalibTool();
    virtual ~PinholeCalibTool();
    
    /** open() passes on to configure()*/
    virtual bool open (yarp::os::Searchable &config);
    virtual bool close ();
    /**
      The PinholeCalibTool expects a configuration file as show below.
      The calibration parameters for a specific camera setup
      can be extracted easily by using the camcalibconf module.\n
      
      fx, fy = focal length (Matlab toolbox: fc) (OpenCV fx, fy)\n
      cx, cy = principal point (Matlab toolbox: cc) (OpenCV cx, cy)\n
      k1, k2, p1, p2 = distortion (Matlab toolbox: First 4 components of vector kc (4th order only))\n 
     (OpenCV:  k1, k2, p1, p2 where k's are radial and p's are tangential distortion coefficients. )\n
      
      Example configuration for a specific camera setup with resolution 320 x 240:\n

      [CAMERA_CALIBRATION]\n
      projection pinhole\n 
      w  320\n
      h  240\n
      fx 325.42\n
      fy 332.31\n
      cx 177.29\n
      cy 127.3\n
      k1 -0.4153\n
      k2 0.2467\n
      p1 -0.00195\n
      p2 0.00185\n
    */ 
    virtual bool configure (yarp::os::Searchable &config);

    /** Stop module if there is a wrong value */
    void stopConfig( std::string val );

  /** Apply calibration, in = rgb image, out = calibrated rgb image.
    * If necessary the output image is resized to match the size of the 
    * input image.
    */
    void apply(const yarp::sig::ImageOf<yarp::sig::PixelRgb> & in,
               yarp::sig::ImageOf<yarp::sig::PixelRgb> & out);    
    
};


#endif

 
 
 
