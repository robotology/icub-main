// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __UZH_CAMCALIBMODULE__
#define __UZH_CAMCALIBMODULE__

 // std
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// opencv
#include <cv.h>

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Semaphore.h>

namespace iCub {
    namespace contrib {
        class CamCalibConfModule;
    }
}

using namespace iCub::contrib;

/**
 *
 * Camera calibration configuration module class
 *
 * \see icub_camcalibconf
 *
 */
class iCub::contrib::CamCalibConfModule : public yarp::os::RFModule {

private:

	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    _prtImg;
    yarp::os::Port _configPort;

    IplImage                            *_ocvImgIn;
    IplImage                            *_ocvImgTmp1;
    IplImage                            *_ocvImgTmp2;
    IplImage                            *_ocvImgOut;

    CvSize                              _oldImgSize;

    int                                 _numPatternImagesRequired;
    int                                 _numPatternInnerCornersX; // number of inner corners of the chessboard pattern (black touches black in corner) 
    int                                 _numPatternInnerCornersY; // "
    double                              _patternSquareSideLength;   
    CvPoint2D32f                        *_corners;                // detected pattern points
    CvMat                               *_imagePoints;            // The joint matrix of corresponding image points, 2xN or Nx2, where N is the total number of points in all views.
    CvMat                               *_objectPoints;           // The joint matrix of object points, 3xN or Nx3, where N is the total number of points in all views.
    CvMat                               *_pointCounts;            // Vector containing numbers of points in each particular view, 1xM or Mx1, where M is the number of a scene views.
    CvMat                               *_intrinsicMatrix;        // The output camera matrix (A) [fx 0 cx; 0 fy cy; 0 0 1].
    CvMat                               *_distortionCoeffs;       // The output 4x1 or 1x4 vector of distortion coefficients [k1, k2, p1, p2].
    //CvMat                               *_rotationVectors;        // The output 3xM or Mx3 array of rotation vectors (compact representation of rotation matrices, see cvRodrigues2).
    //CvMat                               *_translationVectors;     // The output 3xM or Mx3 array of translation vectors.
    int                                 _patternImageCounter;  
	std::string                              _outputFilename;  
    std::string                              _outputGroupname;

    bool                                _grabFlag;                // if true grab image if pattern recognized

    yarp::os::Semaphore                           _semaphore;

    virtual void initImages(int width, int height);
    virtual bool writeCalibrationToFile(int width, int height,
                                        float fx, float fy,
                                        float cx, float cy,
                                        float k1, float k2,
                                        float p1, float p2,
                                        std::string filename,
                                        std::string groupname);

public:

    CamCalibConfModule();
    ~CamCalibConfModule();
    
    /** Passes config on to iCub::contrib::CalibTool */
	virtual bool configure(yarp::os::ResourceFinder& rf);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();

    virtual bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

};

#endif
