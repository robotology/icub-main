// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 Alex Bernardino, Vislab, IST/ISR
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __ARMARKERDETECTORMODULE__
#define __ARMARKERDETECTORMODULE__

 // std
#include <stdio.h>


// ARToolkit
#include <AR/ar.h>

// OpenCV
#include  <cv.h>

//object database
#include "object.h"
#include "coord_frames.h"

// yarp
#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/os/Module.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/**
 *
 * ARToolkit Marker Detector Module
 *
 *
 */

class ARMarkerDetectorModule : public Module {

private:

    bool _reference_found;
    int _xsize, _ysize;
    int _thresh;
    int _object_num;
    double _object_center[2];
    ARParam cparam;
    ObjectData_T *_object;
    IplImage *_frame;
    double _reference[3][4];
	double _timestart;

    BufferedPort<ImageOf<PixelRgb> >       _imgPort;
	BufferedPort<ImageOf<PixelRgb> >       _viewPort;
	BufferedPort<yarp::sig::Vector>        _targetrelativePort;
    BufferedPort<Bottle>                   _configPort;
	BufferedPort<Bottle>                   _coordsPort;

    void copytrans(double src[3][4], double dst[3][4]);
    void calibtrans(double src[3][4], double dst[3][4]);

public:
    cCoordFrames myFrames;

    ARMarkerDetectorModule();
    ~ARMarkerDetectorModule();
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
};


#endif
