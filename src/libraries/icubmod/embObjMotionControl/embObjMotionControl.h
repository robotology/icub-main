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
#include "udp.h"
#include "DSocket.h"
#include <ace/ACE.h>
#include <ace/SOCK_Dgram_Bcast.h>

// Boards configurations
#include "rem-embobj-cfg/eOcfg_EPs_rem_board.h"


#include "../ethManager/ethManager.h"


#define SKELETON_MODE

#define _local_pc104_
// #undef 	_local_pc104_

// indirizzi ip
#ifdef _local_pc104_
#define DEFAULT_LAPTOP_IP	"10.0.0.1" // da usare col pc104
#define DEFAULT_EMS_IP 		"10.0.0.2"

#else
#define DEFAULT_LAPTOP_IP	"10.255.37.155" // <- dhcp;   "10.0.0.1" da usare col pc104
#define DEFAULT_EMS_IP 		"10.255.39.152" // ip della workstation qui dietro.
#endif


#define DEFAULT_EMS_PORT	33333
#define SIZE 				512

/* ETH dummy payload
 *
 * Now it's a dummy structure. It'll be modified in the near future
 * to adapt this data to the AceMor EmbObj since EMS and 2FOC boards will use that
 * protocol. The interface toward the "upper side", i.e. the iCubInterface, has to
 * be the same as others drivers.
 * */

typedef struct
{
	int            			id;
	int            			len;
	timeval    				send_time;
	timeval            		recv_time;
	unsigned char			data[50];
} __attribute__((__packed__)) ETHCAN_MSG;

typedef struct
{
	uint32_t            	a;
	uint32_t            	b;
	timeval    				c;								// send time either from pc104 or from EMS
	timeval            		d;
	char					e[32];
} __attribute__((__packed__)) ETHCAN_STAT;


/* ETH device handle */
typedef struct __ETHCAN_HANDLE
{
	struct sockaddr_in udp_addr;
    int port;
} ETHCAN_HANDLE;

namespace yarp{
    namespace dev{
        class embObjMotionControl;
        class embObjMessage;
    }
}


class yarp::dev::embObjMotionControl: 	public DeviceDriver,
							public os::RateThread,
							public IPidControlRaw,
				            public IControlCalibrationRaw,
							public IControlCalibration2Raw,
							public IAmplifierControlRaw,
							public IEncodersRaw,
							public IPositionControlRaw,
							public IControlModeRaw,
							public ImplementControlMode,
							public ImplementAmplifierControl<embObjMotionControl, IAmplifierControl>,
							public ImplementPositionControl<embObjMotionControl, IPositionControl>,
							public ImplementEncoders<embObjMotionControl, IEncoders>,
							public ImplementControlCalibration<embObjMotionControl, IControlCalibration>,
							public ImplementControlCalibration2<embObjMotionControl, IControlCalibration2>,
				            public ImplementPidControl<embObjMotionControl, IPidControl>,
				            public ImplementVelocityControl<embObjMotionControl, IVelocityControl>,
				            public IFactoryInterface
{
private:
     int 					ret, tot_packet_recv, errors;
    ETHCAN_MSG 				Recv_tmp;
    ETHCAN_MSG 				msg;
    tm						*hr_time1, *hr_time2;
    char 					send_time_string[40];
    char 					recv_time_string[40];
    timeval					th_time;

    uint8_t 				*udppkt_data;
	uint16_t 				udppkt_size;

	// Joint/Mechanical data
	int					_njoints;	// Number of joints handled by this EMS; this values will be extracted by the config file

public:
    embObjMotionControl();
    ~embObjMotionControl();

    ethResources *resource;

    yarp::os::ConstString ethDevName;
    PolyDriver polyDriver;

    /*Device Driver*/
    virtual bool open(yarp::os::Searchable &par);
    virtual bool close();

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

class yarp::dev::embObjMessage // : public yarp::dev::CanMessage
{
 public:
    ETHCAN_MSG *msg;

 public:
    embObjMessage()
    {
        msg=0;
    }
    
    virtual ~embObjMessage()
    {
    }

    //virtual CanMessage &operator=(const CanMessage &l);

    virtual unsigned int getId() const
        { return msg->id;}

    virtual unsigned char getLen() const
        { return msg->len;}

    virtual void setLen(unsigned char len)
        { msg->len=len;}

    virtual void setId(unsigned int id)
        { msg->id=id;}

    virtual const unsigned char *getData() const
        { return msg->data; }

    virtual unsigned char *getData()
        { return msg->data; }

    virtual unsigned char *getPointer()
        { return (unsigned char *) msg; }

    virtual const unsigned char *getPointer() const
        { return (const unsigned char *) msg; }

    virtual void setBuffer(unsigned char *b)
    { 
        if (b!=0)
            msg=(ETHCAN_MSG *)(b);
    }
};

#endif // include guard
