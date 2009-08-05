// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/*/* 
 * Copyright (C) 2007 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Manuel Lopes, Jonas Ruesch, Alex Bernardino
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __CONTROL_GAZE__
#define __CONTROL_GAZE__

 // std
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

// yarp
#include <yarp/String.h>
#include <yarp/sig/Vector.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Module.h>
#include <yarp/os/ResourceFinder.h>

#include <iCub/camera.h>
#include <iCub/head/iCubHeadKinematics.h>
#include <iCub/kinematics/gsl_aux.h>
#include <iCub/predictors.h>


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
 *
 */
class iCub::contrib::Control_GazeModule : public yarp::os::Module,
                                          public iCub::contrib::IControlGazeControls {

private:

	/** input ports
		_positionInput_port - position reference
	*/
   	BufferedPort< yarp::sig::Vector >			_smoothInput_port;
	BufferedPort< yarp::sig::Vector >			_saccadeInput_port;
	BufferedPort< yarp::sig::Vector >			_disparityInput_port;
	BufferedPort< yarp::sig::Vector >			_inertialInput_port;
	BufferedPort<Bottle> _imageCoordPort;				// For simple connection with a viewer click

    // config port
    BufferedPort<Bottle> _configPort;

	
	

	/**	output ports
	*/
	BufferedPort< yarp::sig::Vector >			_posdirOutput_port;
	BufferedPort< yarp::os::Bottle >			_trackersignalOutput_port;
    BufferedPort< yarp::os::Bottle >			_status_port;
	

	Semaphore                 _semaphore;

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

	double vels[6];

	// type of input is 'p' for normalized pixels and 'a' for angles
	//int processposinput(double coord1, double coord2, double *headpos, char type, char behav);
	int posmove(double *pos);
	int velmove(double *vels);
	int mycheckmotion( int axis, double target);


	iCub::contrib::iCubHeadKinematics	head;
    double _eye_azy;    // neck to eye
    double _eye_elev;   // neck to eye
    double _neck_azy;   // neck to head
    double _neck_elev;  // neck to head
	double _targ_azy;
	double _targ_elev;
	double	desazy;			/** current desired gaze for saccades */
	double	deselev;		/** current desired gaze for saccades */
	double currenterror;	/** current error */

    double vergenceGain;

	/* other eye */
	double	desazy_oe;
	double	deselev_oe;

	double *_prediction;

	/** used to deactivate the egosphere motion detection and other filters */
	RemoteEgoSphere			egosphere;
	bool					egosphcom;

	/** used with the inertial sensor to know the expected motion*/
	gsl_vector				*oldW;
	gsl_vector				*inertialW;

    enum Behaviour { SACCADE_1 = 1 , SACCADE_2 = 2, CONTINUE_PURSUIT = 3, LIMIT = 4, REST = 5, START_PURSUIT = 6 };
	enum Command {NONE = 0, SACCADE = 1, PURSUIT = 2, INTERRUPT = 3};
    enum Coordinate {ABSOLUTE_ANGLE = 1, RELATIVE_ANGLE = 2, NORMALIZED_PIXEL = 3, IMAGE_PIXEL = 4};

	
	bool updateAbsoluteGazeReference(double coord1, double coord2, double *headpos, 
		double curr_azy, double cur_elev, double &new_azy, double &new_elev, 
		Command cmd, Coordinate coord);

	//saccades from the integrated spatial memory (egosphere) are absolute angle
	//saccades from the sound locatization are in relative angles w.r.t. the head.
	//saccades from the images are either normalized pixels or image pixels
	//pursuit commands are always relative 

	/** targtype is 'a' for absolute and 'r' for relative */
	//char		targtype;
	/** behavior is 's' saccade and 'p' for pursuit (smooth) and 'r' for rest*/
	//char		behavior;

    Behaviour _behav;
    Coordinate _coord, _lastcoord;
	Command _cmd, _lastcmd;
	double _command_x, _command_y, _lastcommand_x, _lastcommand_y;


	/** saccade id to identify the request number and so reset the tracker */
	int saccadeid;
	double timesaccadeid;
    double _updateTime;
	double _headSaccadeDelay;

	/** control settings */
	int controlType;
	double egosphereVisualUpdateThreshold;

	bool _bVOR;
	//to log
	bool _bLog;
	FILE *fp;
	int ncycles;
	double start;

	double framerate;
	double controlrate;
    /* Added by Alex 20/7/2007 */
    Camera _cam;
	predictors _pred;
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

	virtual double getPeriod();

    // IControlGazeControls
    virtual bool reset();
	virtual bool help();
    virtual bool saccadeAbsolute(double azimuth, double elevation);
    virtual bool saccadeRelative(double azimuth, double elevation);
    virtual bool saccadeImageRef(double pnx, double pny);
	virtual bool saccadePixelRef(double px, double py);
	virtual bool pursuitAbsolute(double azimuth, double elevation);
    virtual bool pursuitRelative(double azimuth, double elevation);
    virtual bool pursuitImageRef(double pnx, double pny);
	virtual bool pursuitPixelRef(double px, double py);
	virtual bool pursuitInterrupt();
    virtual bool getControllerStatus(string &status);
    virtual bool getSaccadeTime(double &time);
    virtual bool getReference(double &azimuth, double &elevation);
    virtual bool getDirectionEyeRight(double &azimuth, double &elevation);
    virtual bool getDirectionEyeLeft(double &azimuth, double &elevation);
    virtual bool getDirectionHead(double &azimuth, double &elevation);
    virtual bool getLimitResetTime(double &time);
    virtual bool getMinSaccadeTime(double &time);
    virtual bool setLimitResetTime(double time);
    virtual bool setMinSaccadeTime(double time);
};


#endif

