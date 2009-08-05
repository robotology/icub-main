// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Manuel Lopes
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __CONTROL_GAZE__
#define __CONTROL_GAZE__

 // std
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
//#include <conio.h> // does not exist on linux
#include <string>

// yarp
#include <yarp/String.h>
#include <yarp/sig/Vector.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Module.h>

/* Added by Alex 20/7/2007 */
#include <iCub/camera.h>
/* End Addition ************/
// iCub
#include <iCub/head/iCubHeadKinematics.h>
#include <iCub/kinematics/gsl_aux.h>


// to turn attention off
#include <iCub/RemoteEgoSphere.h>
#include <iCub/ControlGazeInterfaces.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

namespace iCub {
    namespace contrib {
        class Control_GazeModule;
    }
}

/**
 *
 * Control Gaze Module class
 *
 *	@author Manuel Lopes  macl@isr.ist.utl.pt
 *
 */
class iCub::contrib::Control_GazeModule : public yarp::os::Module,
                                          public iCub::contrib::IControlGazeControls {

private:

	/** input ports
		_positionInput_port - position reference
	*/

   	yarp::os::BufferedPort< yarp::sig::Vector >			_smoothInput_port;
	yarp::os::BufferedPort< yarp::sig::Vector >			_saccadeInput_port;

	yarp::os::BufferedPort< yarp::sig::Vector >			_disparityInput_port;

	yarp::os::BufferedPort< yarp::sig::Vector >			_inertialInput_port;

    // config port
    BufferedPort<Bottle> _configPort;

	/**	output ports
	*/
	yarp::os::BufferedPort< yarp::sig::Vector >			_posdirOutput_port;
	yarp::os::BufferedPort< yarp::os::Bottle >			_trackersignalOutput_port;
	

	yarp::os::Semaphore                 _semaphore;

	/** communication with the motors
	*/
	yarp::dev::PolyDriver				dd;
	yarp::dev::IPositionControl			*ipos;
	yarp::dev::IVelocityControl			*ivel;
	yarp::dev::IEncoders				*ienc;
	yarp::dev::IAmplifierControl		*iamp;
	yarp::dev::IPidControl				*ipid;
	yarp::dev::IControlLimits			*ilim;

    int _numAxes;

	// type of input is 'p' for normalized pixels and 'a' for angles
	int processposinput(Vector *posinput, double *headpos, char type);
	int posmove(double *pos);
	int velmove(double *vels);
	int mycheckmotion( int axis, double target);

	iCub::contrib::iCubHeadKinematics	head;
    double _eye_azy;    // neck to eye
    double _eye_elev;   // neck to eye
    double _neck_azy;   // neck to head
    double _neck_elev;  // neck to head


	double	desazy;			/** current desired gaze for saccades */
	double	deselev;		/** current desired gaze for saccades */
	double currenterror;	/** current error */

	/* other eye */
	double	desazy_oe;
	double	deselev_oe;

	/** used to deactivate the egosphere motion detection and other filters */
	RemoteEgoSphere			egosphere;
	bool					egosphcom;

	/** used with the inertial sensor to know the expected motion of it */
	gsl_vector				*oldW;
	gsl_vector				*inertialW;

	/** targtype is 'a' for absolute and 'r' for relative */
	char		targtype;
	/** behavior is 's' saccade and 'p' for pursuit (smooth) and 'r' for rest*/
	char		behavior;

	/** saccade id to identify the request number and so reset the tracker */
	int saccadeid;
	double timesaccadeid;
    double _updateTime;

	/** control settings */
	int controlType;
	double egosphereVisualUpdateThreshold;

	//to log
	int dolog;
	FILE *fp;
	int ncycles;
	double start;

	double framerate;
    /* Added by Alex 20/7/2007 */
    Camera _cam;
    /* End Addition ************/

    /* Added by Jonas 070723 */
    double _limitResetTime;

    /* Added by paulfitz Wed Aug 15 16:55:46 CEST 2007 */
    double headpos[8];


public:

    Control_GazeModule();
    ~Control_GazeModule();
    
	yarp::os::Semaphore _mutex;

	virtual bool open( yarp::os::Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
	virtual bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);

    // IControlGazeControls
    virtual bool reset();
    virtual bool saccadeAbsolute(double azimuth, double elevation);
    virtual bool saccadeRelative(double azimuth, double elevation);
    virtual bool saccadeImageRef(double pnx, double pny);
    virtual bool getControllerStatus(string &status);
    virtual bool getSaccadeTime(double &time);
    virtual bool getReference(double &azimuth, double &elevation);
    virtual bool getDirectionEyeRight(double &azimuth, double &elevation);
    virtual bool getDirectionEyeLeft(double &azimuth, double &elevation);
    virtual bool getDirectionHead(double &azimuth, double &elevation);
};


#endif
