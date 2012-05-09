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


#include <yarp/os/Log.h>
#include <yarp/os/impl/Logger.h>

// embObj includes

// Boards configurations
extern "C" {
#include "EOnv_hid.h"
#include "eOcfg_EPs_eb7.h"
#include "EoMotionControl.h"
}

#include "embObjMotionControl.h"


using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;

///////////////////////////////////////////////////
//  	Helper functions
///////////////////////////////////////////////////
/*
 * simple helper template to alloc memory.

template <class T>
inline T* allocAndCheck(int size)
{
    T* t = new T[size];
    _YARP_ASSERT (t != 0);
    memset(t, 0, sizeof(T) * size);
    return t;
}
*/

bool embObjMotionControl::alloc(int nj)
{
    _networkN = 0;
    _destinations = allocAndCheck<unsigned char> (EMS_MAX_CARDS);
    _axisMap = allocAndCheck<int>(nj);
    _angleToEncoder = allocAndCheck<double>(nj);
    _zeros = allocAndCheck<double>(nj);
    _torqueSensorId= allocAndCheck<int>(nj);
	_torqueSensorChan= allocAndCheck<int>(nj);
	_maxTorque=allocAndCheck<double>(nj);
	_newtonsToSensor=allocAndCheck<double>(nj);

    _pids=allocAndCheck<Pid>(nj);
	_tpids=allocAndCheck<Pid>(nj);
/*
	_debug_params=allocAndCheck<DebugParameters>(nj);
	_impedance_params=allocAndCheck<ImpedanceParameters>(nj);
	_impedance_limits=allocAndCheck<ImpedanceLimits>(nj);
	_estim_params=allocAndCheck<SpeedEstimationParameters>(nj);
*/
    _limitsMax=allocAndCheck<double>(nj);
    _limitsMin=allocAndCheck<double>(nj);
    _currentLimits=allocAndCheck<double>(nj);
    _velocityShifts=allocAndCheck<int>(EMS_MAX_CARDS);
    _velocityTimeout=allocAndCheck<int>(EMS_MAX_CARDS);
    memset(_limitsMin, 0, sizeof(double)*nj);
    memset(_limitsMax, 0, sizeof(double)*nj);
    memset(_currentLimits, 0, sizeof(double)*nj);
    memset(_velocityShifts, 0, sizeof(int)*nj);
    memset(_velocityTimeout, 0, sizeof(int)*nj);


    _polling_interval = 10;
    _timeout = 20;
    _njoints = nj;

    // followings should not be useful now

    /*_my_address = 0;
    _txQueueSize = -1; // 2047;  // max len of the buffer for the esd driver
    _rxQueueSize = -1; //2047;
    _txTimeout = 20;  // 20ms timeout
    _rxTimeout = 20;

    _broadcast_mask=allocAndCheck<int>(nj);
    for(int j=0;j<nj;j++)
    {
        _broadcast_mask[j]=CAN_BCAST_NONE;
    }*/

    return true;
}

//generic function that check is key1 is present in input bottle and that the result has size elements
// return true/false
bool readAndCheckFromConfigData(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        fprintf(stderr, "%s not found\n", key1.c_str());
        return false;
    }

    if(tmp.size()!=size)
    {
        fprintf(stderr, "%s incorrect number of entries\n", key1.c_str());
        return false;
    }

    out=tmp;

    return true;
}


embObjMotionControl::embObjMotionControl() : 	RateThread(10),
						ImplementControlCalibration2<embObjMotionControl, IControlCalibration2>(this),
						ImplementAmplifierControl<embObjMotionControl, IAmplifierControl>(this),
						ImplementPidControl<embObjMotionControl, IPidControl>(this),
						ImplementEncoders<embObjMotionControl, IEncoders>(this),
						ImplementPositionControl<embObjMotionControl, IPositionControl>(this),
				        ImplementVelocityControl<embObjMotionControl, IVelocityControl>(this),
				        ImplementControlMode(this),
				        _mutex(1)
{
	YARP_INFO(Logger::get(), "embObjMotionControl::embObjMotionControl()", Logger::get().log_files.f3);
	transceiver = 0x00;
	udppkt_data = 0x00;
	udppkt_size = 0x00;

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
	YARP_INFO(Logger::get(), "embObjMotionControl::open", Logger::get().log_files.f3);
	// Debug info
	memset(info, 0x00, SIZE_INFO);
	Bottle xtmp, xtmp2;
	ACE_TCHAR address[64];
	xtmp = Bottle(config.findGroup("ETH"));
	xtmp2 = xtmp.findGroup("IpAddress");
	strcpy(address, xtmp2.get(1).asString().c_str());
	sprintf(info, "embObjMotionControl - referred to EMS: %s", address);


	//
	//	CONFIGURATION
	//

	int i;
//	int _axisMap[100];
//	double _angleToEncoder[100];
//	double _zeros[100];


	ACE_TCHAR tmp[126];
	string str=config.toString().c_str();

    // get the pc104 ip address from config
    Value PC104IpAddress=config.find("PC104IpAddress");
//    prop.put("PC104IpAddress",PC104IpAddress);

    // get robot parameters
	Bottle& general = config.findGroup("GENERAL");
	_njoints = config.findGroup("GENERAL").check("Joints",Value(1),   "Number of degrees of freedom").asInt();

	alloc(_njoints);

	// leggere i valori da file

	if (!readAndCheckFromConfigData(general, xtmp, "AxisMap", "a list of reordered indices for the axes", _njoints+1))
		return false;

	for (i = 1; i < xtmp.size(); i++)
		_axisMap[i-1] = xtmp.get(i).asInt();

	if (!readAndCheckFromConfigData(general, xtmp, "Encoder", "a list of scales for the encoders", _njoints+1))
		return false;


	for (i = 1; i < xtmp.size(); i++)
		_angleToEncoder[i-1] = xtmp.get(i).asDouble();

	if (!readAndCheckFromConfigData(general, xtmp, "Zeros","a list of offsets for the zero point", _njoints+1))
		return false;

	for (i = 1; i < xtmp.size(); i++)
		_zeros[i-1] = xtmp.get(i).asDouble();


	//
	//  INIT ALL INTERFACES
	//
    ImplementControlCalibration2<embObjMotionControl, IControlCalibration2>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementAmplifierControl<embObjMotionControl, IAmplifierControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementEncoders<embObjMotionControl, IEncoders>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementPositionControl<embObjMotionControl, IPositionControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementPidControl<embObjMotionControl, IPidControl>:: initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementControlMode::initialize(_njoints, _axisMap);
    ImplementVelocityControl<embObjMotionControl, IVelocityControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);


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

bool embObjMotionControl::configureTransceiver(ITransceiver *trans)
{
	transceiver = (hostTransceiver *) trans;
	return 1;
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

	EOnv tmp;
//	EOnv = eo_nvscfg_GetNV;
// 	using new file
//	eo_nv_remoteSet();

	// old using my test joint e.p.
//	eOmc_joint_config_t *cfg =  &eo_cfg_nvsEP_joint_usr_rem_board_mem_local->cfg;
//	copyPid2eo(pid, &cfg->pidposition);

  //  transceiver->load_occasional_rop(eo_ropcode_set, EOK_cfg_nvsEP_joint_endpoint, EOK_cfg_nvsEP_joint_NVID__cfg);

//	transceiver->hostTransceiver_AddSetROP(EOK_cfg_nvsEP_joint_endpoint, EOK_cfg_nvsEP_joint_NVID__cfg, (uint8_t*) &cfg, sizeof(cfg));

	//transceiver->hostTransceiver_AddSetROP(EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID__localise, &tmp, 1);
//	printf("Sent EmbObj packet, size = %d\n", udppkt_size);

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

	eOmc_joint_config_t				a;
	uint16_t						sizze;

	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get(endpoint_mc_leftlowerleg, (eo_cfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jconfig__pidposition);

	// la chiamata seguente va sostituita dalle linee successive
//	transceiver->askNV(endpoint_mc_leftlowerleg, nvid, (uint8_t *)&a, &sizze);

	EOnv					*nv = NULL;
	load_occasional_rop(eo_ropcode_ask, endpoint, id);
	//_mutex.wait();		// meccanismo di wait
	transceiver->getNVvalue(nv, (uint8_t *)&a, &sizze);
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
	printf("embObjMotionControl::calibrate");
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

// Utilities

void copyPid2eo(Pid in, eOmc_PID_t *out)
{
	out->kp = in.kp;
	out->ki = in.ki;
	out->kd = in.kd;
	out->limitonintegral = in.max_int;
	out->limitonoutput = in.max_output;
	out->offset = in.offset;
	out->scale = in.scale;
}
