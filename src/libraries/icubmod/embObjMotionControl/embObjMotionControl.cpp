// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2010 Robotcub Consortium
* Author: Lorenzo Natale
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/
///
/// $Id: embObjMotionControl.cpp,v 1.9 2008/03/08 10:07:01 babybot Exp $
///

//#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Bottle.h>
#include <string.h>
#include <iostream>

#include "embObjMotionControl.h"

#include <yarp/os/Log.h>
#include <yarp/os/impl/Logger.h>


#include "hostTransceiver.hpp"


using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;


embObjMotionControl::embObjMotionControl() : 	RateThread(10),
						ImplementControlCalibration<embObjMotionControl, IControlCalibration>(this),
						ImplementControlCalibration2<embObjMotionControl, IControlCalibration2>(this),
						ImplementAmplifierControl<embObjMotionControl, IAmplifierControl>(this),
						ImplementPidControl<embObjMotionControl, IPidControl>(this),
						ImplementEncoders<embObjMotionControl, IEncoders>(this),
						ImplementPositionControl<embObjMotionControl, IPositionControl>(this),
				        ImplementVelocityControl<embObjMotionControl, IVelocityControl>(this),
				        ImplementControlMode(this)
{
	YARP_INFO(Logger::get(),"embObjMotionControl::embObjMotionControl()", Logger::get().log_files.f3);

}

embObjMotionControl::~embObjMotionControl()
{
	char tmp[126];
	YARP_INFO(Logger::get(),"embObjMotionControl::~embObjMotionControl()", Logger::get().log_files.f3);
    /*if (handle!=0)
    {
    	sprintf(tmp, "embObjMotionControl::~embObjMotionControl() 2 handle= 0x%06X", handle);
    	YARP_DEBUG(Logger::get(),tmp); //, Logger::get().log_files.f3);
        delete handle;
    }*/
}

bool embObjMotionControl::open(yarp::os::Searchable &config)
{
    int port;
	char tmp[80], address[16];
	using namespace std;
	Bottle xtmp, xtmp2;
	string str=config.toString().c_str();

	YARP_INFO(Logger::get(), "embObjMotionControl::open", Logger::get().log_files.f3);

	// Clean device and subdevice fileds
    Property prop;
    prop.fromString(str.c_str());
    prop.unput("device");
    prop.unput("subdevice");
    // look for ethernet device driver to use and put it into the "device" field.
    Value &ethprotocol=config.find("ethprotocol");
    prop.put("device", ethprotocol.asString().c_str());

    // get the pc104 ip address from config
    Value PC104IpAddress=config.find("PC104IpAddress");
    prop.put("PC104IpAddress",PC104IpAddress);

    // get robot parameters
	Bottle& general = config.findGroup("GENERAL");
	_njoints = config.findGroup("GENERAL").check("Joints",Value(1),   "Number of degrees of freedom").asInt();

	int i;
	int _axisMap[100];
	double _angleToEncoder[100];
	double _zeros[100];

	// leggere i valori da file
	//Bottle xtmp;
//	if (!validate(general, xtmp, "AxisMap", "a list of reordered indices for the axes", _njoints+1))
//		return false;

	for (i = 1; i < xtmp.size(); i++)
		_axisMap[i-1] = xtmp.get(i).asInt();

//	if (!validate(general, xtmp, "Encoder", "a list of scales for the encoders", _njoints+1))
//		return false;

	int test = xtmp.size();
	for (i = 1; i < xtmp.size(); i++)
		_angleToEncoder[i-1] = xtmp.get(i).asDouble();

//	if (!validate(general, xtmp, "Zeros","a list of offsets for the zero point", _njoints+1))
//		return false;

	for (i = 1; i < xtmp.size(); i++)
		_zeros[i-1] = xtmp.get(i).asDouble();

    ImplementControlCalibration<embObjMotionControl, IControlCalibration>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementControlCalibration2<embObjMotionControl, IControlCalibration2>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementAmplifierControl<embObjMotionControl, IAmplifierControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementEncoders<embObjMotionControl, IEncoders>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementPositionControl<embObjMotionControl, IPositionControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementPidControl<embObjMotionControl, IPidControl>:: initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementControlMode::initialize(_njoints, _axisMap);
    ImplementVelocityControl<embObjMotionControl, IVelocityControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);


	s_eom_hostprotoc_extra_protocoltransceiver_load_occasional_rop(eo_ropcode_ask, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID__applicationinfo);
	hostTransceiver_GetTransmit(&udppkt_data, &udppkt_size);

	// write into skt: udppkt_data, udppkt_size
//	UDP_socket.SendTo(udppkt_data, udppkt_size, remote01.port, remote01.address_ACE);
	printf("Sent EmbObj packet, size = %d\n", udppkt_size);


    polyDriver.open(prop);
    RateThread::start();
	return true;
}

bool embObjMotionControl::close()
{
	YARP_INFO(Logger::get(),"embObjMotionControl::close", Logger::get().log_files.f3);
    RateThread::stop();

    ImplementPositionControl<embObjMotionControl, IPositionControl>::uninitialize();
    ImplementVelocityControl<embObjMotionControl, IVelocityControl>::uninitialize();
    ImplementPidControl<embObjMotionControl, IPidControl>::uninitialize();
}


// Thread
void embObjMotionControl::run(void)
{

#if 0
	//	cmdSocket.send(msg);

	  if(cmdSocket.getState() == UDP_STATE_CONNECTED)
	  {
	    ret = cmdSocket.recv((char*)&Recv_tmp);

	    if (ret > 0)
	    {
	    	gettimeofday(&th_time, NULL);

		  Recv_tmp.recv_time.tv_sec=th_time.tv_sec;
		  Recv_tmp.recv_time.tv_usec=th_time.tv_usec;

	      if(  Recv_tmp.id - msg.id != 1)
	      {
	    	  printf("Warning: packet number NON in progressive order. Missing packet %d!\n ", msg.id +1);
	    	  errors++;
	      }

	      tot_packet_recv++;

	      if(tot_packet_recv % 100 == 0)
	      {
	          hr_time1=localtime(&Recv_tmp.send_time.tv_sec);

	    	  // Format the date and time, down to a single second.
	    	  strftime(send_time_string, sizeof(send_time_string), "%Y-%m-%d %H:%M:%S", hr_time1);

	          hr_time2=localtime(&Recv_tmp.recv_time.tv_sec);
	    	  strftime(recv_time_string, sizeof(recv_time_string), "%Y-%m-%d %H:%M:%S", hr_time2);

	    	  // Print the formatted time, in seconds, followed by a decimal point and the microseconds.
	    	  fprintf(stdout, "%5d send_time %s.%06ld %05ld.%06ld ",
	    	      			  Recv_tmp.id, send_time_string, Recv_tmp.send_time.tv_usec, Recv_tmp.send_time.tv_sec, Recv_tmp.send_time.tv_usec);

	    	  fprintf(stdout, "recv_time %s.%06ld %05ld.%06ld\n",
	    			  	  	  recv_time_string, Recv_tmp.recv_time.tv_usec, Recv_tmp.recv_time.tv_sec, Recv_tmp.recv_time.tv_usec);

	      }
	      memcpy( &msg, &Recv_tmp, sizeof(ETHCAN_MSG));
	    }
	    else
	    {
	    	fprintf(stdout, "Error receiving packet\n");
	    }
	  }

	  printf("Missed %d over %d packet received\n", errors, tot_packet_recv);
	  fprintf(stdout, "tot_packet_recv : %d", tot_packet_recv);

	  pthread_exit(NULL);
#endif
	  return;
}

bool embObjMotionControl::threadInit()
{
	YARP_INFO(Logger::get(),"embObjMotionControl::threadInit", Logger::get().log_files.f3);
	return true;
}

void embObjMotionControl::threadRelease()
{
	YARP_INFO(Logger::get(),"embObjMotionControl::threadRelease", Logger::get().log_files.f3);
	return;
}

///////////// PID INTERFACE
//
bool embObjMotionControl::setPidRaw(int j, const Pid &pid)
{
	YARP_INFO(Logger::get(),"embObjMotionControl::setPidRaw", Logger::get().log_files.f3);
	// Check if j is valid for this specific instance of embObjMotionControl, i.e. if this joint is actually controlled by
	// the EMS I'm referring to.

	msg.id=55;
	msg.len=sizeof(Pid);
	gettimeofday(&msg.send_time, NULL);
	memcpy(msg.data, &pid, msg.len);

	//cmdSocket.send( (char*) &msg);
	// Simil-Final code:

	// if(j_is_ok)
		//former.createMsg(NV_joint_j, value_PID);


}

bool embObjMotionControl::setPidsRaw(const Pid *pids)
{
	YARP_INFO(Logger::get(),"embObjMotionControl::setPidsRaw", Logger::get().log_files.f3);
}

bool embObjMotionControl::setReferenceRaw(int j, double ref)
{
	YARP_INFO(Logger::get(),"embObjMotionControl::setReferenceRaw", Logger::get().log_files.f3);
}

bool embObjMotionControl::setReferencesRaw(const double *refs)
{
	YARP_INFO(Logger::get(),"embObjMotionControl::setReferencesRaw", Logger::get().log_files.f3);
}

bool embObjMotionControl::setErrorLimitRaw(int j, double limit)
{
	YARP_INFO(Logger::get(),"embObjMotionControl::setErrorLimitRaw", Logger::get().log_files.f3);
}

bool embObjMotionControl::setErrorLimitsRaw(const double *limits)
{
	YARP_INFO(Logger::get(),"embObjMotionControl::setErrorLimitsRaw", Logger::get().log_files.f3);
}

bool embObjMotionControl::getErrorRaw(int j, double *err)
{
	YARP_INFO(Logger::get(),"embObjMotionControl::getErrorRaw", Logger::get().log_files.f3);
}

bool embObjMotionControl::getErrorsRaw(double *errs)
{
	YARP_INFO(Logger::get(),"embObjMotionControl::getErrorsRaw", Logger::get().log_files.f3);
}

bool embObjMotionControl::getOutputRaw(int j, double *out)
{
	YARP_INFO(Logger::get(),"embObjMotionControl::getOutputRaw", Logger::get().log_files.f3);
}

bool embObjMotionControl::getOutputsRaw(double *outs)
{
	YARP_INFO(Logger::get(),"embObjMotionControl::getOutputsRaw", Logger::get().log_files.f3);
}

bool embObjMotionControl::getPidRaw(int j, Pid *pid)
{
	YARP_INFO(Logger::get(),"embObjMotionControl::getPidRaw", Logger::get().log_files.f3);
}

bool embObjMotionControl::getPidsRaw(Pid *pids)
{
	YARP_INFO(Logger::get(),"embObjMotionControl::getPidsRaw", Logger::get().log_files.f3);
}

bool embObjMotionControl::getReferenceRaw(int j, double *ref)
{

}

bool embObjMotionControl::getReferencesRaw(double *refs)
{

}

bool embObjMotionControl::getErrorLimitRaw(int j, double *limit)
{

}

bool embObjMotionControl::getErrorLimitsRaw(double *limits)
{

}

bool embObjMotionControl::resetPidRaw(int j)
{

}

bool embObjMotionControl::disablePidRaw(int j)
{

}

bool embObjMotionControl::enablePidRaw(int j)
{

}

bool embObjMotionControl::setOffsetRaw(int j, double v)
{

}

///////////// Velocity control interface raw

bool embObjMotionControl::setVelocityModeRaw()
{
	YARP_INFO(Logger::get(),"embObjMotionControl::setVelocityModeRaw", Logger::get().log_files.f3);
}

bool embObjMotionControl::velocityMoveRaw(int j, double sp)
{
	YARP_INFO(Logger::get(),"embObjMotionControl::velocityMoveRaw(int j, double sp)", Logger::get().log_files.f3);
}

bool embObjMotionControl::velocityMoveRaw(const double *sp)
{
	YARP_INFO(Logger::get(),"embObjMotionControl::velocityMoveRaw(const double *sp)", Logger::get().log_files.f3);
}



///////////// Calibration interface

bool embObjMotionControl::calibrate(int axis, unsigned int type, double p1, double p2, double p3)
{
	printf("embObjMotionControl::calibrate2");
    return true;
}

// CalibrationRaw
bool embObjMotionControl::calibrateRaw(int j, double p)
{
	printf("embObjMotionControl::calibrateRaw");
    return true;
}

// Calibration2
bool embObjMotionControl::calibrate2(int axis, unsigned int type, double p1, double p2, double p3)
{
	printf("embObjMotionControl::calibrate2");
    return true;
}

// Calibration2Raw
bool embObjMotionControl::calibrate2Raw(int axis, unsigned int type, double p1, double p2, double p3)
{
	printf("embObjMotionControl::calibrateRaw");
    return true;
}

bool embObjMotionControl::doneRaw(int axis)
{
	printf("embObjMotionControl::doneRaw");
    return true;
}

bool embObjMotionControl::done(int axis)
{
	printf("embObjMotionControl::done");
    return true;
}

/// POSITION CONTROL INTERFACE RAW
bool embObjMotionControl::getAxes(int *ax) { *ax=_njoints; return true;}
bool embObjMotionControl::setPositionModeRaw(){ }
bool embObjMotionControl::positionMoveRaw(int j, double ref){ }
bool embObjMotionControl::positionMoveRaw(const double *refs){ }
bool embObjMotionControl::relativeMoveRaw(int j, double delta){ }
bool embObjMotionControl::relativeMoveRaw(const double *deltas){ }
bool embObjMotionControl::checkMotionDoneRaw(bool *flag){ }
bool embObjMotionControl::checkMotionDoneRaw(int j, bool *flag){ }
bool embObjMotionControl::setRefSpeedRaw(int j, double sp){ }
bool embObjMotionControl::setRefSpeedsRaw(const double *spds){ }
bool embObjMotionControl::setRefAccelerationRaw(int j, double acc){ }
bool embObjMotionControl::setRefAccelerationsRaw(const double *accs){ }
bool embObjMotionControl::getRefSpeedRaw(int j, double *ref){ }
bool embObjMotionControl::getRefSpeedsRaw(double *spds){ }
bool embObjMotionControl::getRefAccelerationRaw(int j, double *acc){ }
bool embObjMotionControl::getRefAccelerationsRaw(double *accs){ }
bool embObjMotionControl::stopRaw(int j){ }
bool embObjMotionControl::stopRaw(){ }
//
/////////////////////////////// END Position Control INTERFACE

  // ControlMode
bool embObjMotionControl::setPositionModeRaw(int j){ }
bool embObjMotionControl::setVelocityModeRaw(int j){ }
bool embObjMotionControl::setTorqueModeRaw(int j){ }
bool embObjMotionControl::setImpedancePositionModeRaw(int j){ }
bool embObjMotionControl::setImpedanceVelocityModeRaw(int j){ }
bool embObjMotionControl::setOpenLoopModeRaw(int j){ }
bool embObjMotionControl::getControlModeRaw(int j, int *v){ }
bool embObjMotionControl::getControlModesRaw(int* v){ }

//////////////////////// BEGIN EncoderInterface
//
bool embObjMotionControl::resetEncoderRaw(int j){ }
bool embObjMotionControl::resetEncodersRaw(){ }
bool embObjMotionControl::setEncoderRaw(int j, double val){ }
bool embObjMotionControl::setEncodersRaw(const double *vals){ }
bool embObjMotionControl::getEncoderRaw(int j, double *v){ }
bool embObjMotionControl::getEncodersRaw(double *encs){ }
bool embObjMotionControl::getEncoderSpeedRaw(int j, double *sp){ }
bool embObjMotionControl::getEncoderSpeedsRaw(double *spds){ }
bool embObjMotionControl::getEncoderAccelerationRaw(int j, double *spds){ }
bool embObjMotionControl::getEncoderAccelerationsRaw(double *accs){ }
//
///////////////////////// END Encoder Interface

////// Amplifier interface
//
bool embObjMotionControl::enableAmpRaw(int j){ }
bool embObjMotionControl::disableAmpRaw(int j){ }
bool embObjMotionControl::getCurrentsRaw(double *vals){ }
bool embObjMotionControl::getCurrentRaw(int j, double *val){ }
bool embObjMotionControl::setMaxCurrentRaw(int j, double val){ }
bool embObjMotionControl::getAmpStatusRaw(int *st){ }
bool embObjMotionControl::getAmpStatusRaw(int j, int *st){ }


yarp::dev::DeviceDriver *embObjMotionControl::createDevice(yarp::os::Searchable& config)
{

}


