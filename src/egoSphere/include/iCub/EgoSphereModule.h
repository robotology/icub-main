// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch, Alexandre Bernardino, Dario Figueira
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef __EGOSPHEREMODULE__
#define __EGOSPHEREMODULE__

#ifndef M_PI_2
#define M_PI_2	((float)(asin(1.0)))
#endif
#ifndef M_PI
#define M_PI	((float)(2*M_PI_2))
#endif

 // std
#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>

// opencv
#include <cv.h>

// yarp
#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/ResourceFinder.h>

// iCub
#include <iCub/SphereProjector.h>
#include <iCub/head/iCubHeadKinematics.h>
#include <iCub/EgoSphereInterfaces.h>
#include <iCub/IOR.h>
#include <iCub/AcousticMap.h>
#include <iCub/VisualMap.h>
#include <iCub/ObjectMap.h>
#include <iCub/Framerate.h>
#include <iCub/IModalityMap.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

namespace iCub {
    namespace contrib {
        class EgoSphereModule;
    }
}

using namespace iCub::contrib;

/**
 *
 * Ego-sphere Module class
 *
 * \see icub_egosphere
 *
 */
class iCub::contrib::EgoSphereModule : public Module,
                                       public IEgoSphereControls {

private:

    // image ports
    BufferedPort<ImageOf<PixelFloat> >	_prtImgEgoFloat;

    // config port
    BufferedPort<Bottle>                _configPort;

    // Object locations port
   
    // modality maps
	std::vector<IModalityMap*>			_vctMap;

    // controlboard
    PolyDriver			                _dd;
    IEncoders			                *_ienc;
    IPositionControl                    *_ipos;
    double                              *_encoders;
    int                                 _numAxes;

	// kinematics
    iCubHeadKinematics                  _headKin;
	RobMatrix							_rEyeMatrix;
	RobMatrix							_lEyeMatrix;
	RobMatrix							_headMatrix;
	double								*_rotREye_W2C; // 1x9
	double								*_rotLEye_W2C; // 1x9
	double								*_rotHead_W2C; // 1x9
	double								*_rotREye_C2W; // 1x9
	double								*_rotLEye_C2W; // 1x9
	double								*_rotHead_C2W; // 1x9
	double								*_gazeREye; // 1x3
	double								*_gazeLEye; // 1x3
	double								*_gazeHead; // 1x3
	double								_azREye; // 1x1
	double								_elREye; // 1x1
	double								_azLEye; // 1x1
	double								_elLEye; // 1x1
	double								_azHead; // 1x1
	double								_elHead; // 1x1
    int                                 _gazeREyeX;
    int                                 _gazeREyeY;
	int									_gazeLEyeX;
	int									_gazeLEyeY;
	int									_gazeHeadX;
	int									_gazeHeadY;

    IOR                                 _ior;

    ImageOf<PixelFloat>                 _yrpImgFloatEgo;    // final aggregated egosphere
    
    CvSize                              _egoImgSize;
    bool                                _blnSaccadicSuppression;
    double                              _refreshTime;
    float                               _thresholdSalience;

    bool                                _activateIOR;
    bool                                _activateModVision;
    bool                                _activateModAuditory;
    bool                                _activateModObjects; 

	bool								_blnControlboard;
	std::string							_strControlboard;

    Semaphore                           _semaphore;

	Framerate							_framerate;

    void processClick();
    // camera2world 3x3, gaze 1x3
    void calcGazeVector(double *rotCamera2World, double *gaze);
    void calcGazePixel(double azimuth, double elevation, int &gazeX, int &gazeY);
    // in degrees
    void calcAngles(double *gaze, double &azimuth, double &elevation);
    void getPeak(ImageOf<PixelFloat> &img, int &i, int &j, float &v);

    // reset internal state of the module (zero images) (remoted)
    bool reset();

public:

    EgoSphereModule();
    virtual ~EgoSphereModule();
    
    /** Passes config on to iCub::contrib::CalibTool */
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
    virtual bool respond(const Bottle &command,Bottle &reply);

    // egosphere controls
    virtual bool setSaccadicSuppression(bool on);
    virtual bool getSaccadicSuppression();
    virtual bool setSalienceDecay(double rate);
    virtual double getSalienceDecay();
    virtual bool setThresholdSalience(float thr);
    virtual float getThresholdSalience();
	virtual bool addIORRegion(double azimuth, double elevation);

	// convert from RobMatrix to 1 dimensional 3x3 -> 1x9 array
    static void convertRobMatrix(RobMatrix &robMatrix, double *matrix);

	static inline void transpose(double *src, double *dst){
        dst[0] = src[0]; dst[1] = src[3]; dst[2] = src[6];
        dst[3] = src[1]; dst[4] = src[4]; dst[5] = src[7];
        dst[6] = src[2]; dst[7] = src[5]; dst[8] = src[8];
    }
};


#endif
