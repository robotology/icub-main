// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __ATTENTIONLOGGERMODULE__
#define __ATTENTIONLOGGERMODULE__

 // std
#include <stdio.h>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <time.h>

// yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/ResourceFinder.h>

// cv
#include <cv.h>

// iCub
#include <iCub/head/iCubHeadKinematics.h>

namespace iCub {
    namespace contrib {
        class AttentionLoggerModule;
    }
}



/**
 *
 * AttentionLogger Module class
 *
 * \see icub_attentionLogger
 *
 */
class iCub::contrib::AttentionLoggerModule : public yarp::os::Module {
protected:
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> > _prtEgoSalience;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > _prtEgoRgb;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > _prtEgoIOR;
	yarp::os::BufferedPort<yarp::sig::VectorOf<double> > _prtGazeTarget;
    yarp::os::BufferedPort<yarp::os::Bottle> _configPort;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> _imgEgoSalienceRgb;
    // controlboard
    yarp::dev::PolyDriver _dd;
    yarp::dev::IEncoders *_iEnc;
    double *_encoders;
    int _numAxes;
    // kinematics
    iCubHeadKinematics _headKin;
    RobMatrix _eyeMatrix;
    double *_gaze;
    double *_rotW2C; // world to camera rotation
    double *_rotC2W; // camera to world rotation
    double _azimuth;
    double _elevation;
    int _gazeX;
    int _gazeY;
    int _egoImageWidth;
    int _egoImageHeight;
    bool _flagNewTarget;
    // log file
    std::ofstream _logFile;
    std::stringstream _sstream;
    int _targetIndex;
    double _targetPosDegX;
    double _targetPosDegY;
    double _targetPosDegXOld;
    double _targetPosDegYOld;
    int _targetPosPixX;
    int _targetPosPixY;
    int _targetPosPixXOld;
    int _targetPosPixYOld;
    double _timeStart;
    // properties
    std::string _logFileName;
    std::string _logDirName;
    void convertRobMatrix(RobMatrix &robMatrix, double *matrix);
    // camera2world 3x3, gaze 1x3
    void calcGazeVector(double *rotCamera2World, double *gaze);
    void calcGazePixel(double &azimuth, double &elevation, int &gazeX, int &gazeY);
    // in degrees
    void calcAngles(double *gaze, double &azimuth, double &elevation);
    inline void transpose(double *src, double *dst){
        dst[0] = src[0]; dst[1] = src[3]; dst[2] = src[6];
        dst[3] = src[1]; dst[4] = src[4]; dst[5] = src[7];
        dst[6] = src[2]; dst[7] = src[5]; dst[8] = src[8];
    }
    void colorRgbFromFloat(IplImage* imgFloat, IplImage* imgRgb, float scaleFactor);
public:

    AttentionLoggerModule();
    virtual ~AttentionLoggerModule();
    
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();

};

#endif
