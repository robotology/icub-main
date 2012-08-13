// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

//#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Bottle.h>
#include <string.h>
#include <iostream>


#include <yarp/os/Log.h>
#include <yarp/os/impl/Logger.h>

// embObj includes

// Boards configurations
#include "EOnv_hid.h"
#include "EoMotionControl.h"


#include "embObjMotionControl.h"
#include "debugging.h"

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

void embObjMotionControl::waitSem()
{
	semaphore.wait();
}

void embObjMotionControl::postSem()
{
	semaphore.post();
}

embObjMotionControl::embObjMotionControl() : 	RateThread(10),
						ImplementControlCalibration2<embObjMotionControl, IControlCalibration2>(this),
						ImplementAmplifierControl<embObjMotionControl, IAmplifierControl>(this),
						ImplementPidControl<embObjMotionControl, IPidControl>(this),
						ImplementEncodersTimed(this),
						ImplementPositionControl<embObjMotionControl, IPositionControl>(this),
				        ImplementVelocityControl<embObjMotionControl, IVelocityControl>(this),
				        ImplementControlMode(this),
				        _mutex(1)
{
	print_debug(AC_trace_file, "embObjMotionControl::embObjMotionControl()");
	udppkt_data = 0x00;
	udppkt_size = 0x00;
}

embObjMotionControl::~embObjMotionControl()
{
	char tmp[126];
	//YARP_INFO(Logger::get(),"embObjMotionControl::~embObjMotionControl()", Logger::get().log_files.f3);
    /*if (handle!=0)
    {
    	sprintf(tmp, "embObjMotionControl::~embObjMotionControl() 2 handle= 0x%06X", handle);
    	//YARP_DEBUG(Logger::get(),tmp); //, Logger::get().log_files.f3);
        delete handle;
    }*/
}

bool embObjMotionControl::open(yarp::os::Searchable &config)
{
	print_debug(AC_trace_file, " embObjMotionControl::open\n");
	// Debug info
	memset(info, 0x00, SIZE_INFO);
	Bottle xtmp, xtmp2;
	ACE_TCHAR address[64];
	xtmp = Bottle(config.findGroup("ETH"));
	xtmp2 = xtmp.findGroup("IpAddress");
	strcpy(address, xtmp2.get(1).asString().c_str());
	sprintf(info, "embObjMotionControl - referred to EMS: %s", address);

	//
	// open ethResource, if needed
	//

	Property prop;
	ACE_TCHAR tmp[126];
	string str=config.toString().c_str();
	xtmp = Bottle(config.findGroup("FEATURES"));
	prop.fromString(str.c_str());
	prop.unput("device");
	prop.unput("subdevice");
	// look for Ethernet device driver to use and put it into the "device" field.
	Value &device=xtmp.find("device");
	prop.put("device", device.asString().c_str());

	ethResCreator *resList = ethResCreator::instance();
	res = resList->getResource(prop);

	// Defining Unique Id
	_fId.type = MotionControl;
	std::string featId = config.find("FeatId").asString().c_str();
	cout << "FeatId = " << featId << endl;
	strcpy(_fId.name, featId.c_str());

	_fId.ep = 255;

	if( 0 == strcmp("EMS_2BLL_H2", _fId.name) )
		_fId.ep = endpoint_sk_emsboard_leftlowerarm;

	if( 0 == strcmp("EMS_2BLL_H4", _fId.name) )
		_fId.ep = endpoint_sk_emsboard_rightlowerarm;


	if(255 == _fId.ep)
	{
		printf("\n ERROR: MotionControl Endpoint not found!!!\n");
		return false;
	}

	//
	//	CONFIGURATION
	//

	int i;
//	int _axisMap[100];
//	double _angleToEncoder[100];
//	double _zeros[100];




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
    ImplementEncodersTimed::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementPositionControl<embObjMotionControl, IPositionControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementPidControl<embObjMotionControl, IPidControl>:: initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementControlMode::initialize(_njoints, _axisMap);
    ImplementVelocityControl<embObjMotionControl, IVelocityControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);


	return true;
}

bool embObjMotionControl::close()
{
	//YARP_INFO(Logger::get(),"embObjMotionControl::close", Logger::get().log_files.f3);
    RateThread::stop();

    ImplementEncodersTimed::uninitialize();
    ImplementPositionControl<embObjMotionControl, IPositionControl>::uninitialize();
    ImplementVelocityControl<embObjMotionControl, IVelocityControl>::uninitialize();
    ImplementPidControl<embObjMotionControl, IPidControl>::uninitialize();
}


void embObjMotionControl::getMotorController(DeviceDriver *iMC)
{

}

// Thread
void embObjMotionControl::run(void)  // probaly useless now.... think about it
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
	print_debug(AC_trace_file, "embObjMotionControl::threadInit()");
	return true;
}

void embObjMotionControl::threadRelease()
{
	print_debug(AC_trace_file, "embObjMotionControl::threadRelease()");
	return;
}

///////////// PID INTERFACE
//
bool embObjMotionControl::setPidRaw(int j, const Pid &pid)
{
	print_debug(AC_trace_file, "embObjMotionControl::setPidRaw()");
	// Check if j is valid for this specific instance of embObjMotionControl, i.e. if this joint is actually controlled by
	// the EMS I'm referring to.
	eOnvID_t 					nvid = -1;
	EOnv 						*nvRoot = NULL;

	nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, 0x00, jointNVindex_jconfig__pidposition);
	nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	if(NULL == nvRoot)
	{
		printf("\n>>> ERROR \ntransceiver->getNVhandler returned NULL!!\n");
		return false;
	}
	//eOmc_joint_config_t *cfg =  &eo_cfg_nvsEP_joint_usr_rem_board_mem_local->cfg;
	eOmc_PID_t	outPid;
	copyPid2eo(pid, &outPid);
	if( eores_OK != eo_nv_Set(nvRoot, &outPid, eobool_true, eo_nv_upd_dontdo))
	{
		printf("\n>>> ERROR \neo_nv_Set !!\n");
		return false;
	}
	res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, jointNVindex_jconfig__pidposition);
	printf("Sent EmbObj packet, size = %d\n", udppkt_size);
	return true;
}

bool embObjMotionControl::setPidsRaw(const Pid *pids)
{
	print_debug(AC_trace_file, "embObjMotionControl::setPidRaw()");
}

bool embObjMotionControl::setReferenceRaw(int j, double ref)
{
	print_debug(AC_trace_file, "embObjMotionControl::setReferenceRaw()");
}

bool embObjMotionControl::setReferencesRaw(const double *refs)
{
	print_debug(AC_trace_file, "embObjMotionControl::setReferencesRaw()");
}

bool embObjMotionControl::setErrorLimitRaw(int j, double limit)
{
	print_debug(AC_trace_file, "embObjMotionControl::setErrorLimitRaw()");
}

bool embObjMotionControl::setErrorLimitsRaw(const double *limits)
{
	print_debug(AC_trace_file, "embObjMotionControl::setErrorLimitsRaw()");
}

bool embObjMotionControl::getErrorRaw(int j, double *err)
{
	print_debug(AC_trace_file, "embObjMotionControl::getErrorRaw()");
}

bool embObjMotionControl::getErrorsRaw(double *errs)
{
	print_debug(AC_trace_file, "embObjMotionControl::getErrorRaw()");
}

bool embObjMotionControl::getOutputRaw(int j, double *out)
{
	print_debug(AC_trace_file, "embObjMotionControl::getOutputRaw()");
}

bool embObjMotionControl::getOutputsRaw(double *outs)
{
	print_debug(AC_trace_file, "embObjMotionControl::getOutputsRaw()");
}

bool embObjMotionControl::getPidRaw(int j, Pid *pid)
{
	print_debug(AC_trace_file, "embObjMotionControl::getPidRaw()");

	eOmc_joint_config_t				a;
	uint16_t						sizze;

	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get(endpoint_mc_leftlowerleg, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jconfig__pidposition);

	// la chiamata seguente va sostituita dalle linee successive
//	transceiver->askNV(endpoint_mc_leftlowerleg, nvid, (uint8_t *)&a, &sizze);

	res->transceiver->load_occasional_rop(eo_ropcode_ask, endpoint_mc_leftlowerleg, nvid);
	//_mutex.wait();		// meccanismo di wait

	EOnv	*nv = res->transceiver->getNVhandler( endpoint_mc_leftlowerleg,  nvid);  //??

	res->transceiver->getNVvalue(nv, (uint8_t *)&a, &sizze);
	waitSem();
}

bool embObjMotionControl::getPidsRaw(Pid *pids)
{
	print_debug(AC_trace_file, "embObjMotionControl::getPidsRaw()");
}

bool embObjMotionControl::getReferenceRaw(int j, double *ref)
{
	print_debug(AC_trace_file, "embObjMotionControl::getReferenceRaw()");
}

bool embObjMotionControl::getReferencesRaw(double *refs)
{
	print_debug(AC_trace_file, "embObjMotionControl::getReferencesRaw()");
}

bool embObjMotionControl::getErrorLimitRaw(int j, double *limit)
{
	print_debug(AC_trace_file, "embObjMotionControl::getErrorLimitRaw()");
}

bool embObjMotionControl::getErrorLimitsRaw(double *limits)
{
	print_debug(AC_trace_file, "embObjMotionControl::getErrorLimitsRaw()");
}

bool embObjMotionControl::resetPidRaw(int j)
{
	print_debug(AC_trace_file, "embObjMotionControl::resetPidRaw()");
}

bool embObjMotionControl::disablePidRaw(int j)
{
	print_debug(AC_trace_file, "embObjMotionControl::disablePidRaw()");
}

bool embObjMotionControl::enablePidRaw(int j)
{
	print_debug(AC_trace_file, "embObjMotionControl::enablePidRaw()");
}

bool embObjMotionControl::setOffsetRaw(int j, double v)
{
	print_debug(AC_trace_file, "embObjMotionControl::setOffsetRaw()");
}

///////////// Velocity control interface raw

bool embObjMotionControl::setVelocityModeRaw()
{
	print_debug(AC_trace_file, "embObjMotionControl::setVelocityModeRaw()");
}

bool embObjMotionControl::velocityMoveRaw(int j, double sp)
{
	print_debug(AC_trace_file, "embObjMotionControl::velocityMoveRaw()");
}

bool embObjMotionControl::velocityMoveRaw(const double *sp)
{
	print_debug(AC_trace_file, "embObjMotionControl::velocityMoveRaw()");
}



///////////// Calibration interface

bool embObjMotionControl::calibrate(int axis, unsigned int type, double p1, double p2, double p3)
{
	printf("embObjMotionControl::calibrate");
    return true;
}

bool embObjMotionControl::calibrateRaw(int j, double p)
{
	printf("embObjMotionControl::calibrateRaw");
    return true;
}

bool embObjMotionControl::calibrate2(int axis, unsigned int type, double p1, double p2, double p3)
{
	printf("embObjMotionControl::calibrate2");
    return true;
}

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

bool embObjMotionControl::getEncodersTimedRaw(double *encs, double *stamps) { }
bool embObjMotionControl::getEncoderTimedRaw(int j, double *encs, double *stamp) { }

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
