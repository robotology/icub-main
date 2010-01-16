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


#define USE_PREDICTIVE_CONTROL 0

 // std
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

// yarp
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Module.h>
#include <yarp/os/ResourceFinder.h>

#include <iCub/camera.h>
#include <iCub/head/iCubHeadKinematics.h>
#include <iCub/kinematics/gsl_aux.h>

#include <iCub/iKinFwd.h>
using namespace iKin;
using namespace yarp::math;

#if USE_PREDICTIVE_CONTROL
	#include <iCub/predictors.h>
#endif


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
class iCub::contrib::Control_GazeModule : public yarp::os::RFModule,
                                          public iCub::contrib::IControlGazeControls {

private:

	/** input ports
		_positionInput_port - position reference
	*/
   	yarp::os::BufferedPort< yarp::sig::Vector >			_smoothInput_port;
	yarp::os::BufferedPort< yarp::sig::Vector >			_saccadeInput_port;
	yarp::os::BufferedPort< yarp::sig::Vector >			_disparityInput_port;
	yarp::os::BufferedPort< yarp::sig::Vector >			_inertialInput_port;
	yarp::os::BufferedPort< yarp::sig::Vector >			_torsoInput_port;
	yarp::os::BufferedPort<Bottle> _imageCoordPort;				// For simple connection with a viewer click

    // config port
	yarp::os::Port _configPort;

	
	

	/**	output ports
	*/
	yarp::os::BufferedPort< yarp::sig::Vector >			_posdirOutput_port;
	yarp::os::BufferedPort< yarp::os::Bottle >			_trackersignalOutput_port;
    yarp::os::BufferedPort< yarp::os::Bottle >			_status_port;
	

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

	double vels[6];

	// type of input is 'p' for normalized pixels and 'a' for angles
	//int processposinput(double coord1, double coord2, double *headpos, char type, char behav);
	int posmove(double *pos);
	int velmove(double *vels);
	int relmove(double *deltas);
	int mycheckmotion( int axis, double target);


	iCub::contrib::iCubHeadKinematics	head;
    double _eye_azy;		// eye azimuth w.r.t. the neck base reference frame
    double _eye_elev;		// eye elevation w.r.t. the neck base reference frame
    double _head_azy;		// head azimuth w.r.t. the neck base reference frame
    double _head_elev;		// head elevation w.r.t. the neck base reference frame
	double _targ_azy;		// target azimuth w.r.t. the neck base reference frame
	double _targ_elev;		// target elevation w.r.t. the neck base reference frame
	double	desazy;			// current desired gaze azimuth w.r.t. the neck base reference frame
	double	deselev;		// current desired gaze elevation w.r.t. the neck base reference frame
	
	double _abs_eye_az;
	double _abs_eye_el;
	double _abs_head_az;
	double _abs_head_el;
	double _abs_ref_az;		// reference azimuth w.r.t. the waist reference frame (absolute)
	double _abs_ref_el;		// reference elevation w.r.t. the waist reference frame (absolute)
	
	double currenterror;	/** current error */

    double vergenceGain;

	/* other eye */
	//double	desazy_oe;      // is this really needed ?
	//double	deselev_oe;     // is this really needed ?

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

	bool updateAbsoluteGazeReference2(double coord1, double coord2, double *headpos, double *torsopos,
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
	#if USE_PREDICTIVE_CONTROL
	predictors _pred;
	#endif
    /* End Addition ************/

    /* Added by Jonas 070723 */
    double _limitResetTime;

    /* Added by paulfitz Wed Aug 15 16:55:46 CEST 2007 */
    double headpos[8];

	/* Added by alex Mon Jan 11 2010 */
	double torsopos[8];
	bool _fake_velocity_control;
	double _fake_velocity_gain;
	double _velocity_gain;

	//for testing Ugo's classes
protected: 
	iCubEye eyeKinematics; 
	iCubInertialSensor inertialKinematics; 
   
	void initInertialKinematics(Vector &maxL, Vector &minL)
	{
		for (int j = 0; j < 3; j++) 
			inertialKinematics.releaseLink(j);
			
		iKinChain* inertialChain = inertialKinematics.asChain();

		int n = inertialKinematics.getDOF();

		for(int i = 0; i < n; i++)
		{
			(*inertialChain)[i].setMin((M_PI/180.0)*minL(i));
			(*inertialChain)[i].setMax((M_PI/180.0)*maxL(i));
		}
	}
	void initEyeKinematics(Vector &maxL, Vector &minL)
	{
		for (int j = 0; j < 3; j++) {
			eyeKinematics.releaseLink(j);
		}

		iKinChain* eyeChain = eyeKinematics.asChain();
		
		for(int i = 0; i < 8; i++)
		{
			(*eyeChain)[i].setMin((M_PI/180.0)*minL(i));
			(*eyeChain)[i].setMax((M_PI/180.0)*maxL(i));
		}
	};
	Matrix getNeck2WaistTransf(double *torsoEncoders )
	{
		Matrix tNeck2Waist;
		Vector torsoData(6);  //will use the inertial sensor kinematic chain
		// torsoEncoders in degrees
		// torsoData units shall be in radians
		// remind that the torso is in reverse order:
		// their joints are sent assuming the neck as kinematic origin
		// and not the waist, hence we've got to invert them!
		torsoData[0]=(M_PI/180.0)*torsoEncoders[2];	
		torsoData[1]=(M_PI/180.0)*torsoEncoders[1];
		torsoData[2]=(M_PI/180.0)*torsoEncoders[0];
		//the remaining ones no not matter- can be set to zero
		torsoData[3] = 0.0;
		torsoData[4] = 0.0;
		torsoData[5] = 0.0;
		inertialKinematics.setAng(torsoData);
		tNeck2Waist = inertialKinematics.getH(2);
		return tNeck2Waist;
	}

	Matrix getCyclopPose(Vector &cyclopData)
	{
		eyeKinematics.setAng(cyclopData);
		return eyeKinematics.getH();
	}

	//Using Craig's formulae
	Matrix getRotationMatrixFromRollPitchYawAngles(double roll, double pitch, double yaw)
	{
		double gamma = roll*(M_PI/180);
		double beta = pitch*(M_PI/180);
		double alpha = yaw*(M_PI/180);
		double ca = cos(alpha);
		double cb = cos(beta);
		double cg = cos(gamma);
		double sa = sin(alpha);
		double sb = sin(beta);
		double sg = sin(gamma);
		Matrix R(3,3);
		R(0,0) = ca*cb; 
		R(0,1) = ca*sb*sg-sa*cg;
		R(0,2) = ca*sb*cg-sa*sg;
		R(1,0) = sa*cb;
		R(1,1) = sa*sb*sg+ca*cg;
		R(1,2) = sa*sb*cg-ca*sg;
		R(2,0) = -sb;
		R(2,1) = cb*sg;
		R(2,2) = cb*cg;
		return R;
	}
	void getRollPitchYawAnglesFromRotationMatrix( Matrix &R, double &roll, double &pitch, double &yaw )
	{
		double gamma = atan2(R(2,1),R(2,2));
		double beta = atan2(-R(2,0), sqrt(R(0,0)*R(0,0)+R(1,0)*R(1,0)));
		double alpha = atan2(R(1,0),R(0,0));
		roll = gamma*(180/M_PI);
		pitch = beta*(180/M_PI);
		yaw = alpha*(180/M_PI);
	}
	void getEyeRollPitchYawAnglesWRTWaist(double *head_encoders, double *torso_encoders, double &roll, double &pitch, double &yaw)
	{
		Vector headData(8);
		// units shall be in radians
		// remind that the torso is in reverse order:
		// their joints are sent assuming the neck as kinematic origin
		// and not the waist, hence we've got to invert them!
		headData[0]=(M_PI/180.0)*torso_encoders[2];	
		headData[1]=(M_PI/180.0)*torso_encoders[1];
		headData[2]=(M_PI/180.0)*torso_encoders[0];
		// neck part
		headData[3]=(M_PI/180.0)*head_encoders[0];
		headData[4]=(M_PI/180.0)*head_encoders[1];
		headData[5]=(M_PI/180.0)*head_encoders[2];
		// eye part
		headData[6] = (M_PI/180.0)*head_encoders[3];
		headData[7] = (M_PI/180.0)*head_encoders[4];
		// set the joints
		eyeKinematics.setAng(headData);
		//Get Trasformation matriz
		yarp::sig::Matrix eyeH = eyeKinematics.getH();
		//Get rotation submatrix
		Matrix headR = eyeH.submatrix(0,2,0,2);
		//Trasform to suitable (Base) coordinates for Roll Pitch Yaw computation
		Matrix RT(3,3);
		RT(0,0)=0; RT(0,1)=1; RT(0,2)=0;
		RT(1,0)=0; RT(1,1)=0; RT(1,2)=-1;
		RT(2,0)=-1; RT(2,1)=0; RT(2,2)=0;
		Matrix headRT = headR*RT;
		//Get roll, pitch, yaw angles
		double temp;
		getRollPitchYawAnglesFromRotationMatrix(headRT, temp, pitch, yaw);		
	}
	void getHeadRollPitchYawAnglesWRTWaist(double *head_encoders, double *torso_encoders, double &roll, double &pitch, double &yaw)
	{
		Vector headData(8);
		// units shall be in radians
		// remind that the torso is in reverse order:
		// their joints are sent assuming the neck as kinematic origin
		// and not the waist, hence we've got to invert them!
		headData[0]=(M_PI/180.0)*torso_encoders[2];	
		headData[1]=(M_PI/180.0)*torso_encoders[1];
		headData[2]=(M_PI/180.0)*torso_encoders[0];
		// neck part
		headData[3]=(M_PI/180.0)*head_encoders[0];
		headData[4]=(M_PI/180.0)*head_encoders[1];
		headData[5]=(M_PI/180.0)*head_encoders[2];
		// set the joints
		inertialKinematics.setAng(headData);
		//Get Trasformation matriz
		yarp::sig::Matrix headH = inertialKinematics.getH();
		//Get rotation submatrix
		Matrix headR = headH.submatrix(0,2,0,2);
		//Get roll, pitch, yaw angles
		double temp;
		getRollPitchYawAnglesFromRotationMatrix(headR, temp, pitch, yaw);		
	}

public:

    Control_GazeModule();
    ~Control_GazeModule();
    
	yarp::os::Semaphore _mutex;

	virtual bool configure(yarp::os::ResourceFinder &rf);
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
	//virtual bool getReferenceToWaist(double &azimuth, double &elevation);
    virtual bool getDirectionEyeRight(double &azimuth, double &elevation);
	//virtual bool getDirectionEyeRightToWaist(double &azimuth, double &elevation);
    virtual bool getDirectionEyeLeft(double &azimuth, double &elevation);
	//virtual bool getDirectionEyeLeftToWaist(double &azimuth, double &elevation);
    virtual bool getDirectionHead(double &azimuth, double &elevation);
	//virtual bool getDirectionHeadToWaist(double &azimuth, double &elevation);
    virtual bool getLimitResetTime(double &time);
    virtual bool getMinSaccadeTime(double &time);
    virtual bool setLimitResetTime(double time);
    virtual bool setMinSaccadeTime(double time);
};


#endif

