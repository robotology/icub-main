// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

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

// Boards configurations
#include "eOcfg_EPs_rem_board.h"

#include "../embObjLib/hostTransceiver.hpp"

// indirizzi ip
#define DEFAULT_LAPTOP_IP	"10.255.37.155" // da usare col pc104
#define DEFAULT_EMS_IP 		"127.0.0.1"		// "10.255.39.152"  ip della workstation qui dietro.
#define DEFAULT_EMS_PORT	33333

namespace yarp{
    namespace dev{
        class embObjMotionControl;
    }
}

void copyPid2eo(Pid in, eOmc_PID_t *out);

class yarp::dev::embObjMotionControl: 	public DeviceDriver,
							public os::RateThread,
							public IPidControlRaw,
							public IControlCalibration2Raw,
							public IAmplifierControlRaw,
							public IEncodersRaw,
							public IPositionControlRaw,
							public IControlModeRaw,
							public ImplementControlMode,
							public ImplementAmplifierControl<embObjMotionControl, IAmplifierControl>,
							public ImplementPositionControl<embObjMotionControl, IPositionControl>,
							public ImplementEncoders<embObjMotionControl, IEncoders>,
							public ImplementControlCalibration2<embObjMotionControl, IControlCalibration2>,
				            public ImplementPidControl<embObjMotionControl, IPidControl>,
				            public ImplementVelocityControl<embObjMotionControl, IVelocityControl>,
				            public IFactoryInterface
{
private:
     int 					ret, tot_packet_recv, errors;
    tm						*hr_time1, *hr_time2;
    char 					send_time_string[40];
    char 					recv_time_string[40];
    timeval					th_time;

    uint8_t 				*udppkt_data;
	uint16_t 				udppkt_size;

	hostTransceiver 		*transceiver;
    yarp::os::Semaphore 	_mutex;

	// Joint/Mechanical data
	int						_njoints;	// Number of joints handled by this EMS; this values will be extracted by the config file

public:
    embObjMotionControl();
    ~embObjMotionControl();

    char					info[126];
//    ethResources *resource;

    yarp::os::ConstString ethDevName;
    PolyDriver polyDriver;


    /*Device Driver*/
    virtual bool open(yarp::os::Searchable &par);
    virtual bool close();

    // _AC_
    bool configureTransceiver(ITransceiver *trans);

    bool __init(void);
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

    ///////// 	Velocity control interface raw	/////////
     virtual bool setVelocityModeRaw();
     virtual bool velocityMoveRaw(int j, double sp);
     virtual bool velocityMoveRaw(const double *sp);


    // calibration
    virtual bool calibrate(int axis, unsigned int type, double p1, double p2, double p3);
//    virtual bool done(int j);

    // calibrationraw
    virtual bool calibrateRaw(int j, double p);
//    virtual bool doneRaw(int j);

    // calibration2
    virtual bool calibrate2(int axis, unsigned int type, double p1, double p2, double p3);
    virtual bool done(int j);

    // calibration2raw
    virtual bool calibrate2Raw(int axis, unsigned int type, double p1, double p2, double p3);
    virtual bool doneRaw(int j);

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

	////// Amplifier interface
	virtual bool enableAmpRaw(int j);
	virtual bool disableAmpRaw(int j);
	virtual bool getCurrentsRaw(double *vals);
	virtual bool getCurrentRaw(int j, double *val);
	virtual bool setMaxCurrentRaw(int j, double val);
	virtual bool getAmpStatusRaw(int *st);
	virtual bool getAmpStatusRaw(int j, int *st);
	/////////////// END AMPLIFIER INTERFACE


    ////////////// IFactoryInterface
    yarp::dev::DeviceDriver *createDevice(yarp::os::Searchable& config);

protected:
   // Tread
   virtual void run(void);
   virtual bool threadInit();
   virtual void threadRelease();
};

#endif // include guard
