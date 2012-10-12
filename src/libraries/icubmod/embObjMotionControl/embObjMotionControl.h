// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/* Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Alberto Cardellino
 * email: alberto.cardellino@iit.it
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

// update comment hereafter

/**
 * @ingroup icub_hardware_modules 
 * \defgroup eth2ems eth2ems
 *
 * Implements <a href="http://eris.liralab.it/yarpdoc/d3/d5b/classyarp_1_1dev_1_1ICanBus.html" ICanBus interface <\a> for a ems to can bus device.
 * This is the eth2ems module device.
 *
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 *
 * Author: Alberto Cardellino
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/modules/....h
 *
 */

//
// $Id: embObjMotionControl.h,v 1.5 2008/06/25 22:33:53 nat Exp $
//

#ifndef __embObjMotionControlh__
#define __embObjMotionControlh__

#undef __cplucplus

using namespace std;

/////  Yarp stuff
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include "yarp/dev/ControlBoardInterfacesImpl.inl" //ControlBoardHelper
///////////////////

#include <iCub/FactoryInterface.h>
#include <linux/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


// ACE udp socket
#include <ace/ACE.h>
#include <ace/SOCK_Dgram_Bcast.h>

#include "FeatureInterface.h"

#include "EoMotionControl.h"
#include <ethManager.h>
//#include "../ethManager/ethManager.h"
#include "../embObjLib/hostTransceiver.hpp"
#include "IRobotInterface.h"

#include "eoRequestsQueue.hpp"

// Move to a different file?

//
//   Help structure
//

struct ImpedanceLimits
{
	double min_stiff;
	double max_stiff;
	double min_damp;
	double max_damp;
	double param_a;
	double param_b;
	double param_c;

	public:
	ImpedanceLimits()
	{
		min_stiff=0; max_stiff=0;
		min_damp=0;  max_damp=0;
		param_a=0; param_b=0; param_c=0;
	}

	double get_min_stiff() {return min_stiff;}
	double get_max_stiff() {return max_stiff;}
	double get_min_damp()  {return min_damp;}
	double get_max_damp()  {return max_damp;}
};

struct ImpedanceParameters
{
		double stiffness;
		double damping;
		bool   enabled;
		ImpedanceLimits limits;
		ImpedanceParameters() {stiffness=0; damping=0; enabled=false;}
};

struct SpeedEstimationParameters
{
	double jnt_Vel_estimator_shift;
	double jnt_Acc_estimator_shift;
	double mot_Vel_estimator_shift;
	double mot_Acc_estimator_shift;

	SpeedEstimationParameters()
	{
		jnt_Vel_estimator_shift=0;
		jnt_Acc_estimator_shift=0;
		mot_Vel_estimator_shift=0;
		mot_Acc_estimator_shift=0;
	}
};


////////////////////////////////////////////
// indirizzi ip
#define DEFAULT_LAPTOP_IP	"10.255.37.155" // da usare col pc104
#define DEFAULT_EMS_IP 		"127.0.0.1"		// "10.255.39.152"  ip della workstation qui dietro.
#define DEFAULT_EMS_PORT	33333


#define EMS_MAX_CARDS		1				// TO BE REMOVED

namespace yarp{
    namespace dev{
        class embObjMotionControl;
    }
}

void copyPid_iCub2eo(const Pid *in, eOmc_PID_t *out);
void copyPid_eo2iCub(eOmc_PID_t *in, Pid *out);


class yarp::dev::embObjMotionControl: 	public DeviceDriver,
							public IPidControlRaw,
							public IControlCalibration2Raw,
							public IAmplifierControlRaw,
				            public IEncodersTimedRaw,
				            public ImplementEncodersTimed,
							public IPositionControlRaw,
				            public IVelocityControlRaw,
							public IControlModeRaw,
							public ImplementControlMode,
							public ImplementAmplifierControl<embObjMotionControl, IAmplifierControl>,
							public ImplementPositionControl<embObjMotionControl, IPositionControl>,
							public ImplementControlCalibration2<embObjMotionControl, IControlCalibration2>,
				            public ImplementPidControl<embObjMotionControl, IPidControl>,
				            public ImplementVelocityControl<embObjMotionControl, IVelocityControl>
{
private:
    int 					tot_packet_recv, errors;
    tm						*hr_time1, *hr_time2;
    char 					send_time_string[40];
    char 					recv_time_string[40];
    timeval					th_time;

    uint8_t 				*udppkt_data;
	uint16_t 				udppkt_size;

	// _AC_
	//hostTransceiver 		*transceiver;
    yarp::os::Semaphore 	_mutex;
    FEAT_ID					_fId;

	// Joint/Mechanical data

//    int _networkN;								/** network number */
//    unsigned char *_destinations;       		/** destination addresses */
//    unsigned char _my_address;					/** my address */
//    int _polling_interval;						/** thread polling interval [ms] */
//    int _timeout;								/** number of cycles before timing out */

    int *_axisMap;                              /** axis remapping lookup-table */
    double *_angleToEncoder;                    /** angle to encoder conversion factors */
    double *_rotToEncoder;                      /** angle to rotor conversion factors */
    double *_zeros;                             /** encoder zeros */
    Pid *_pids;                                 /** initial gains */
	Pid *_tpids;								/** initial torque gains */
	bool _tpidsEnabled;							/** abilitation for torque gains */
	SpeedEstimationParameters *_estim_params;   /** parameters for speed/acceleration estimation */
//	DebugParameters *_debug_params;             /** debug parameters */
	ImpedanceParameters *_impedance_params;		/** impedance parameters */
	ImpedanceLimits     *_impedance_limits;     /** impedancel imits */
    double *_limitsMin;                         /** joint limits, max*/
    double *_limitsMax;                         /** joint limits, min*/
    double *_currentLimits;                     /** current limits */
	int *_velocityShifts;                       /** velocity shifts */
	int *_velocityTimeout;                      /** velocity shifts */
	int *_torqueSensorId;						/** Id of associated Joint Torque Sensor */
	int *_torqueSensorChan;						/** Channel of associated Joint Torque Sensor */
	double *_maxTorque;						    /** Max torque of a joint */
	double *_newtonsToSensor;                   /** Newtons to force sensor units conversion factors */

// basic knowledge of my joints
	int		_njoints;							// Number of joints handled by this EMS; this values will be extracted by the config file
    int 	_firstJoint;						// in case the EMS controls joints from x to y where x is not 0, functions like setpidS need to know how to run the for loop


    // internal stuff
    bool    *_enabledAmp;		// Middle step toward a full enabled motor controller. Amp (pwm) plus Pid enable command must be sent in order to get the joint into an active state.
    bool    *_enabledPid;		// Depends on enabledAmp. When both are set, the joint exits the idle mode and goes into position mode. If one of them is disabled, it falls to idle.
    bool    *_calibrated;		// Flag to know if the calibrate function has been called for the joint
    double 	*_ref_positions;	// used for position control.
    double 	*_ref_speeds;		// used for position control.
    double 	*_command_speeds;	// used for velocity control.
    double 	*_ref_accs;			// for velocity control, in position min jerk eq is used.
	double 	*_ref_torques;		// for torque control.


	uint16_t 		NVnumber;		// keep if useful to store, otherwise can be removed. It is used to pass the total number of this EP to the requestqueue

public:
    embObjMotionControl();
    ~embObjMotionControl();

    char					info[SIZE_INFO];
    Semaphore				semaphore;
	eoRequestsQueue			*requestQueue;	// tabella che contiene la lista delle attese
    ethResources 			*res;

    //debug
    yarp::os::ConstString 	ethDevName;

    /*Device Driver*/
    virtual bool open(yarp::os::Searchable &par);
    virtual bool close();
    bool fromConfig(yarp::os::Searchable &config);

    // _AC_
    eoThreadEntry * appendWaitRequest(int j, uint16_t nvid);
    void getMotorController(DeviceDriver *iMC);
//    void waitSem();
//    void postSem();
    bool alloc(int nj);
    bool init(void);

    ///////// 	PID INTERFACE		/////////
    virtual bool setPidRaw(int j, const Pid &pid);
    virtual bool setPidsRaw(const Pid *pids);
    virtual bool setReferenceRaw(int j, double ref);
    virtual bool setReferencesRaw(const double *refs);
    virtual bool setErrorLimitRaw(int j, double limit);
    virtual bool setErrorLimitsRaw(const double *limits);
    virtual bool getErrorRaw(int j, double *err);
    virtual bool getErrorsRaw(double *errs);
    virtual bool getOutputRaw(int j, double *out);
    virtual bool getOutputsRaw(double *outs);
    virtual bool getPidRaw(int j, Pid *pid);
    virtual bool getPidsRaw(Pid *pids);
    virtual bool getReferenceRaw(int j, double *ref);
    virtual bool getReferencesRaw(double *refs);
    virtual bool getErrorLimitRaw(int j, double *limit);
    virtual bool getErrorLimitsRaw(double *limits);
    virtual bool resetPidRaw(int j);
    virtual bool disablePidRaw(int j);
    virtual bool enablePidRaw(int j);
    virtual bool setOffsetRaw(int j, double v);

	/// POSITION CONTROL INTERFACE RAW
	virtual bool getAxes(int *ax);
	virtual bool setPositionModeRaw();
	virtual bool positionMoveRaw(int j, double ref);
	virtual bool positionMoveRaw(const double *refs);
	virtual bool relativeMoveRaw(int j, double delta);
	virtual bool relativeMoveRaw(const double *deltas);
	virtual bool checkMotionDoneRaw(bool *flag);
	virtual bool checkMotionDoneRaw(int j, bool *flag);
	virtual bool setRefSpeedRaw(int j, double sp);
	virtual bool setRefSpeedsRaw(const double *spds);
	virtual bool setRefAccelerationRaw(int j, double acc);
	virtual bool setRefAccelerationsRaw(const double *accs);
	virtual bool getRefSpeedRaw(int j, double *ref);
	virtual bool getRefSpeedsRaw(double *spds);
	virtual bool getRefAccelerationRaw(int j, double *acc);
	virtual bool getRefAccelerationsRaw(double *accs);
	virtual bool stopRaw(int j);
	virtual bool stopRaw();

    ///////// 	Velocity control interface raw	/////////
     virtual bool setVelocityModeRaw();
     virtual bool velocityMoveRaw(int j, double sp);
     virtual bool velocityMoveRaw(const double *sp);


    // calibration
//    virtual bool calibrate(int axis, unsigned int type, double p1, double p2, double p3);
//    virtual bool done(int j);

    // calibrationraw
//    virtual bool calibrateRaw(int j, double p);
//    virtual bool doneRaw(int j);

//    // calibration2
//    virtual bool calibrate2(int axis, unsigned int type, double p1, double p2, double p3);
//    virtual bool done(int j);

    // calibration2raw
    virtual bool calibrate2Raw(int axis, unsigned int type, double p1, double p2, double p3);
    virtual bool doneRaw(int j);


	//
	/////////////////////////////// END Position Control INTERFACE

      // ControlMode
	virtual bool setPositionModeRaw(int j);
	virtual bool setVelocityModeRaw(int j);
	virtual bool setTorqueModeRaw(int j);
	virtual bool setImpedancePositionModeRaw(int j);
	virtual bool setImpedanceVelocityModeRaw(int j);
	virtual bool setOpenLoopModeRaw(int j);
	virtual bool getControlModeRaw(int j, int *v);
	virtual bool getControlModesRaw(int* v);

	//////////////////////// BEGIN EncoderInterface
	//
	virtual bool resetEncoderRaw(int j);
	virtual bool resetEncodersRaw();
	virtual bool setEncoderRaw(int j, double val);
	virtual bool setEncodersRaw(const double *vals);
	virtual bool getEncoderRaw(int j, double *v);
	virtual bool getEncodersRaw(double *encs);
	virtual bool getEncoderSpeedRaw(int j, double *sp);
	virtual bool getEncoderSpeedsRaw(double *spds);
	virtual bool getEncoderAccelerationRaw(int j, double *spds);
	virtual bool getEncoderAccelerationsRaw(double *accs);
	///////////////////////// END Encoder Interface

    virtual bool getEncodersTimedRaw(double *encs, double *stamps);
    virtual bool getEncoderTimedRaw(int j, double *encs, double *stamp);

	////// Amplifier interface
	virtual bool enableAmpRaw(int j);
	virtual bool disableAmpRaw(int j);
	virtual bool getCurrentsRaw(double *vals);
	virtual bool getCurrentRaw(int j, double *val);
	virtual bool setMaxCurrentRaw(int j, double val);
	virtual bool getAmpStatusRaw(int *st);
	virtual bool getAmpStatusRaw(int j, int *st);
	/////////////// END AMPLIFIER INTERFACE

};

#endif // include guard
