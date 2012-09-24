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

#include "embObjMotionControl.h"

// Boards configurations
#include "EOnv_hid.h"


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

// Utilities

void copyPid_iCub2eo(const Pid *in, eOmc_PID_t *out)
{
	out->kp = in->kp;
	out->ki = in->ki;
	out->kd = in->kd;
	out->limitonintegral = in->max_int;
	out->limitonoutput = in->max_output;
	out->offset = in->offset;
	out->scale = in->scale;
}

void copyPid_eo2iCub(eOmc_PID_t *in, Pid *out)
{
	out->kp = in->kp;
	out->ki = in->ki;
	out->kd = in->kd;
	out->max_int = in->limitonintegral;
	out->max_output = in->limitonoutput;
	out->offset = in->offset;
	out->scale = in->scale;
}


inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    fprintf(stderr, "%s not yet implemented for embObjMotionControl\n", txt);

    return false;
}

#define NV_NOT_FOUND print_debug(AC_error_file, "\n>>> ERROR \ntransceiver->getNVhandler returned NULL!! This may mean that this variable is not handled by this EMS\n");

#define SOME_WAIT_MECHANISM sleep(0)


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

//void embObjMotionControl::waitSem()
//{
//	semaphore.wait();
//}

//void embObjMotionControl::postSem()
//{
//	semaphore.post();
//}

embObjMotionControl::embObjMotionControl() :
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
	_enabledAmp = false;
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


	xtmp = Bottle(config.findGroup("ETH"));
	int boardNum=255;
	xtmp2 = xtmp.findGroup("Ems");
	printf("Ems %s - %d - %s\n", xtmp2.get(0).asString().c_str(), xtmp2.get(1).asInt(), xtmp2.get(2).asString().c_str());
	_fId.boardNum = xtmp2.get(1).asInt();

	_fId.ep = 255;

	switch(_fId.boardNum)
	{
		case 1:
			_fId.ep = endpoint_mc_leftupperarm;
			break;
		case 2:
			_fId.ep = endpoint_mc_leftlowerarm;
			break;
		case 3:
			_fId.ep = endpoint_mc_rightupperarm;
			break;
		case 4:
			_fId.ep = endpoint_mc_rightlowerarm;
			break;
		case 5:
			_fId.ep = 255;
			break;
		case 6:
			_fId.ep = 255;
			break;
		case 7:
			_fId.ep = 255;
			break;
		case 8:
			_fId.ep = 255;
			break;
		case 9:
			_fId.ep = 255;
			break;
		default:
			_fId.ep = 255;
			printf("\n ERROR: MotionControl endpoint not found!!!\n");
			return false;
			break;
	}

	// Save eo data of this board/EP
	res->transceiver->getHostData(&_fId.EPvector, &_fId.EPhash_function);
	_fId.handle  = /*dynamic_cast<IiCubFeature*>*/ (this);

	ethResCreator::instance()->addLUTelement(_fId);

	if ( (_fId.boardNum >= FIRST_BOARD) && (_fId.boardNum <= LAST_BOARD) )
	{
		NVnumber = res->transceiver->getNVnumber(_fId.boardNum, _fId.ep);
    	requestQueue = new eoRequestsQueue(NVnumber);
	}
	else
		fprintf(stderr, "Non existing board number!!\n");

	//
	//  Tell EMS which NV I want to be signalled spontaneously
	//

	init();

	//
	//	CONFIGURATION
	//

	int i;

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

	// extract info about the first joint controlled by this EMS.
	_firstJoint = _axisMap[0];
	for (i = 1; i < _njoints; i++)
		if(_axisMap[i] < _firstJoint)
			_firstJoint = _axisMap[i];

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

    // Reserve space for data stored locally. values are initialize to 0
    _ref_positions = allocAndCheck<double>(_njoints);
    _command_speeds = allocAndCheck<double>(_njoints);
    _ref_speeds = allocAndCheck<double>(_njoints);
    _ref_accs = allocAndCheck<double>(_njoints);
    _ref_torques = allocAndCheck<double>(_njoints);
    _enabledAmp = allocAndCheck<bool>(_njoints);

	return true;
}

bool embObjMotionControl::init()
{
	eOmn_ropsigcfg_command_t 	*ropsigcfgassign;
	EOarray						*array;
	eOropSIGcfg_t 				sigcfg;

	eOnvID_t nvid, nvid_ropsigcfgassign = eo_cfg_nvsEP_mn_comm_NVID_Get(endpoint_mn_comm, 0, commNVindex__ropsigcfgcommand);
	EOnv *nvRoot = res->transceiver->getNVhandler(endpoint_mn_comm, nvid_ropsigcfgassign);

	ropsigcfgassign = (eOmn_ropsigcfg_command_t*) nvRoot->loc;
	array = (EOarray*) &ropsigcfgassign->array;
	eo_array_Reset(array);
	array->head.capacity = NUMOFROPSIGCFG;
	array->head.itemsize = sizeof(eOropSIGcfg_t);
	ropsigcfgassign->cmmnd = ropsigcfg_cmd_append;

	for(int j=_firstJoint; j<_firstJoint+_njoints;j++)
	{
		nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, j, jointNVindex_jstatus__basic);
		sigcfg.ep = _fId.ep;
		sigcfg.id = nvid;
		sigcfg.plustime = 0;
		eo_array_PushBack(array, &sigcfg);

		nvid = eo_cfg_nvsEP_mc_motor_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, j, motorNVindex_mstatus__basic);
		sigcfg.ep = _fId.ep;
		sigcfg.id = nvid;
		sigcfg.plustime = 0;
		eo_array_PushBack(array, &sigcfg);
	}
}

bool embObjMotionControl::close()
{
    ImplementEncodersTimed::uninitialize();
    ImplementPositionControl<embObjMotionControl, IPositionControl>::uninitialize();
    ImplementVelocityControl<embObjMotionControl, IVelocityControl>::uninitialize();
    ImplementPidControl<embObjMotionControl, IPidControl>::uninitialize();
}

eoThreadEntry * embObjMotionControl::appendWaitRequest(int j, uint16_t nvid)
{
	eoRequest req;
	if(!requestQueue->threadPool->getId(&req.threadId) )
		fprintf(stderr, "Error: too much threads!! (embObjMotionControl)");
	req.joint = j;
	req.nvid = res->transceiver->translate_NVid2index(_fId.boardNum, _fId.ep, nvid);

	requestQueue->append(req);
	return requestQueue->threadPool->getThreadTable(req.threadId);
}

void embObjMotionControl::getMotorController(DeviceDriver *iMC)
{

}


///////////// PID INTERFACE

bool embObjMotionControl::setPidRaw(int j, const Pid &pid)
{
	print_debug(AC_trace_file, "embObjMotionControl::setPidRaw()");

	// Check if j is valid for this specific instance of embObjMotionControl, i.e. if this joint is actually controlled by
	// the EMS I'm referring to.
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jconfig__pidposition);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;		return false;
	}

	eOmc_PID_t	outPid;
	copyPid_iCub2eo(&pid, &outPid);
	if( eores_OK != eo_nv_Set(nvRoot, &outPid, eobool_true, eo_nv_upd_dontdo))
	{
		print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}
	res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid);
	return true;
}

bool embObjMotionControl::setPidsRaw(const Pid *pids)
{
	print_debug(AC_trace_file, "embObjMotionControl::setPidRaw()");

	bool ret = true;
	for(int j=_firstJoint; j<_firstJoint+_njoints;j++)
	{
		ret &= setPidRaw(j, pids[j]);
	}
	return ret;
}

bool embObjMotionControl::setReferenceRaw(int j, double ref)
{
	print_debug(AC_trace_file, "embObjMotionControl::setReferenceRaw()");
	return NOT_YET_IMPLEMENTED("setReferenceRaw");
}

bool embObjMotionControl::setReferencesRaw(const double *refs)
{
	print_debug(AC_trace_file, "embObjMotionControl::setReferencesRaw()");
	return NOT_YET_IMPLEMENTED("setReferencesRaw");
}

bool embObjMotionControl::setErrorLimitRaw(int j, double limit)
{
	print_debug(AC_trace_file, "embObjMotionControl::setErrorLimitRaw()");
	return NOT_YET_IMPLEMENTED("setErrorLimitRaw");
}

bool embObjMotionControl::setErrorLimitsRaw(const double *limits)
{
	print_debug(AC_trace_file, "embObjMotionControl::setErrorLimitsRaw()");
	return NOT_YET_IMPLEMENTED("setErrorLimitsRaw");
}

bool embObjMotionControl::getErrorRaw(int j, double *err)
{
	print_debug(AC_trace_file, "embObjMotionControl::getErrorRaw()");

	// Check if j is valid for this specific instance of embObjMotionControl, i.e. if this joint is actually controlled by
	// the EMS I'm referring to.

	eOnvID_t nvid = eo_cfg_nvsEP_mc_motor_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, motorNVindex_mstatus);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	// faster, less safe
//	eOmc_joint_status_t  *MotorStatus = (eOmc_joint_status_t*) nvRoot->rem;
//	*err = (double) MotorStatus->ofpid.error;

	eOmc_joint_status_t  tmpMotorStatus;
	uint16_t size;
	res->transceiver->getNVvalue(nvRoot, (uint8_t*) &tmpMotorStatus, &size);
	*err = (double) tmpMotorStatus.ofpid.error;
	return true;
}

bool embObjMotionControl::getErrorsRaw(double *errs)
{
	print_debug(AC_trace_file, "embObjMotionControl::getErrorRaw()");

	bool ret = true;
	for(int j=_firstJoint; j<_firstJoint+_njoints;j++)
	{
		ret &= getErrorRaw(j, &errs[j]);
	}
	return ret;
}

bool embObjMotionControl::getOutputRaw(int j, double *out)
{
	print_debug(AC_trace_file, "embObjMotionControl::getOutputRaw()");

	eOnvID_t nvid = eo_cfg_nvsEP_mc_motor_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, motorNVindex_mstatus);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

//	eOmc_joint_status_t  *JointStatus = (eOmc_joint_status_t*) nvRoot->rem;
//	*out = (double) JointStatus->ofpid.output;

	eOmc_joint_status_t  tmpJointStatus;
	uint16_t size;
	res->transceiver->getNVvalue(nvRoot, (uint8_t*) &tmpJointStatus, &size);
	*out = (double) tmpJointStatus.ofpid.output;

	return true;
}

bool embObjMotionControl::getOutputsRaw(double *outs)
{
	print_debug(AC_trace_file, "embObjMotionControl::getOutputsRaw()");

	bool ret = true;
	for(int j=_firstJoint; j<_firstJoint+_njoints;j++)
	{
		ret &= getOutputRaw(j, &outs[j]);
	}
	return ret;
}

bool embObjMotionControl::getPidRaw(int j, Pid *pid)
{
	print_debug(AC_trace_file, "embObjMotionControl::getPidRaw()");

	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jconfig__pidposition);
	EOnv	*nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}
	res->transceiver->load_occasional_rop(eo_ropcode_ask, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid);

	// Sign up for waiting the reply
	eoThreadEntry *tt = appendWaitRequest(j, nvid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
	tt->setPending(1);

	// wait here
	if(-1 == tt->synch() )
	{
		int threadId;
		printf("embObjMotionControl::getPidRaw timed out!!\n");
		if(requestQueue->threadPool->getId(&threadId))
			requestQueue->cleanTimeouts(threadId);
		return false;
	}
	// Get the value
	uint16_t size;
	res->transceiver->getNVvalue(nvRoot, (uint8_t *)pid, &size);
	return true;
}

bool embObjMotionControl::getPidsRaw(Pid *pids)
{
	print_debug(AC_trace_file, "embObjMotionControl::getPidsRaw()");

	bool ret = true;
	eoThreadEntry *tt = NULL;

	eOnvID_t  nvid[_njoints];
	EOnv	  *nvRoot[_njoints];
	eoRequest req;

	for(int j=_firstJoint, index=0; j<_firstJoint+_njoints; j++, index++)
	{
		nvid[index]   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jconfig__pidposition);
		nvRoot[index] = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid[index]);

		if(NULL == nvRoot[index])
		{
			NV_NOT_FOUND;
			ret = false;
		}
		// Sign up for waiting the reply
		tt = appendWaitRequest(j, nvid[index]);
	}

	printf("waiting for %d replies\n", _njoints);
	if( NULL != tt)
	{
		tt->setPending(_njoints);

		// wait just once for all data
		// wait here
		if(-1 == tt->synch() )
		{
			int threadId;
			printf("embObjMotionControl::getPidRaw timed out!!\n");
			if(requestQueue->threadPool->getId(&threadId))
				requestQueue->cleanTimeouts(threadId);
			return false;
		}
	}
	else
		return false;
	// copy data received to the caller
	for(int j=_firstJoint, index=0; j<_firstJoint+_njoints; j++, index++)
	{
		eOmc_PID_t *tmpPid = (eOmc_PID16_t *) nvRoot[index];  //tmpPid doesn't need to be an array, because it's just a cast of an existing data.
		copyPid_eo2iCub(&tmpPid[index], &pids[index]);
	}
	return ret;
}

bool embObjMotionControl::getReferenceRaw(int j, double *ref)
{
	print_debug(AC_trace_file, "embObjMotionControl::getReferenceRaw()");
	return NOT_YET_IMPLEMENTED("getReference");
}

bool embObjMotionControl::getReferencesRaw(double *refs)
{
	print_debug(AC_trace_file, "embObjMotionControl::getReferencesRaw()");
	return NOT_YET_IMPLEMENTED("getReference");
}

bool embObjMotionControl::getErrorLimitRaw(int j, double *limit)
{
	print_debug(AC_trace_file, "embObjMotionControl::getErrorLimitRaw()");
    return NOT_YET_IMPLEMENTED("getErrorLimit");
}

bool embObjMotionControl::getErrorLimitsRaw(double *limits)
{
	print_debug(AC_trace_file, "embObjMotionControl::getErrorLimitsRaw()");
    return NOT_YET_IMPLEMENTED("getErrorLimit");
}

bool embObjMotionControl::resetPidRaw(int j)
{
	print_debug(AC_trace_file, "embObjMotionControl::resetPidRaw()");
    return NOT_YET_IMPLEMENTED("resetPid");
}

bool embObjMotionControl::disablePidRaw(int j)
{
	print_debug(AC_trace_file, "embObjMotionControl::disablePidRaw()");
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jconfig__controlmode);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;		return false;
	}

	eOmc_controlmode_t val = eomc_controlmode_idle;
	if( eores_OK != eo_nv_Set(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))     // can I pass &eomc_controlmode_idle??
	{
		print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid);
	return true;
}

bool embObjMotionControl::enablePidRaw(int j)
{
	print_debug(AC_trace_file, "embObjMotionControl::enablePidRaw()");

	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jconfig__controlmode);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	//eOmc_controlmode_t val = *(eOmc_controlmode_t *) nvRoot->loc;

	if(false == _enabledAmp[j-_firstJoint] )
	{
		print_debug(stderr, "Error: Enable Amp before enabling Pid!!  (embObjCtrl)\n");
		return false;
	}

	eOmc_controlmode_t val = eomc_controlmode_position;

	if( eores_OK != eo_nv_Set(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid);
	return true;
}

bool embObjMotionControl::setOffsetRaw(int j, double v)
{
	print_debug(AC_trace_file, "embObjMotionControl::setOffsetRaw()");
    return NOT_YET_IMPLEMENTED("setOffset");
}

////////////////////////////////////////
//    Velocity control interface raw  //
////////////////////////////////////////

bool embObjMotionControl::setVelocityModeRaw()
{
	print_debug(AC_trace_file, "embObjMotionControl::setVelocityModeRaw()");

	bool ret = true;
	eOnvID_t  nvid[_njoints];
	EOnv	  *nvRoot[_njoints];

	eOmc_controlmode_t val = eomc_controlmode_velocity;
	for(int j=_firstJoint, index=0; j<_firstJoint+_njoints; j++, index++)
	{
		nvid[index]   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jconfig__controlmode);
		nvRoot[index] = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid[index]);

		if(NULL == nvRoot[index])
		{
			NV_NOT_FOUND;
			ret = false;
		}

		if( eores_OK != eo_nv_Set(nvRoot[index], &val, eobool_true, eo_nv_upd_dontdo))
		{
			print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
			return false;
		}

		res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid[index]);
	}
	return ret;
}

bool embObjMotionControl::velocityMoveRaw(int j, double sp)
{
	print_debug(AC_trace_file, "embObjMotionControl::velocityMoveRaw()");

	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jcmmnds__setpoint);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);
	int index = j- _firstJoint;

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

    _command_speeds[j] = sp ;   // save internally the new value of speed.

	eOmc_setpoint_t setpoint;
	setpoint.type = eomc_setpoint_velocity;
	setpoint.to.velocity.value =  _command_speeds[index];
	setpoint.to.velocity.withacceleration = _ref_accs[index];

	if( eores_OK != eo_nv_Set(nvRoot, &setpoint, eobool_true, eo_nv_upd_dontdo))
	{
		print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid);
	return true;
}

bool embObjMotionControl::velocityMoveRaw(const double *sp)
{
	print_debug(AC_trace_file, "embObjMotionControl::velocityMoveRaw()");

	bool ret = true;
	eOnvID_t  nvid[_njoints];
	EOnv	  *nvRoot[_njoints];
	eOmc_setpoint_t setpoint;

	setpoint.type = eomc_setpoint_velocity;

	for(int j=_firstJoint, index=0; j<_firstJoint+_njoints; j++, index++)
	{
		nvid[index]   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jcmmnds__setpoint);
		nvRoot[index] = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid[index]);

		if(NULL == nvRoot[index])
		{
			NV_NOT_FOUND;
			ret = false;
		}

	    _command_speeds[index] = sp[index] / 1000.0;   // save internally the new value of speed.
		setpoint.to.velocity.value =  _command_speeds[index];
		setpoint.to.velocity.withacceleration = _ref_accs[index];

		if( eores_OK != eo_nv_Set(nvRoot[index], &setpoint, eobool_true, eo_nv_upd_dontdo))
		{
			print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
			return false;
		}

		res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid[index]);
	}
	return ret;
}


////////////////////////////////////////
//    Calibration control interface   //
////////////////////////////////////////

bool embObjMotionControl::calibrate2Raw(int j, unsigned int type, double p1, double p2, double p3)
{
	print_debug(AC_trace_file, "embObjMotionControl::calibrateRaw()");

		// Check if j is valid for this specific instance of embObjMotionControl, i.e. if this joint is actually controlled by
		// the EMS I'm referring to.
		eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jcmmnds__calibration);
		EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

		if(NULL == nvRoot)
		{
			NV_NOT_FOUND;
			return false;
		}

		eOmc_calibrator_t calib;

		switch(type)
		{
			case eomc_calibration_type0_hard_stops:
				calib.type = eomc_calibration_type0_hard_stops;
				calib.params.type0.pwmlimit = p1;
				calib.params.type0.velocity = p2;
			break;

			case eomc_calibration_type1_abs_sens_analog:
				calib.type = eomc_calibration_type1_abs_sens_analog;
				calib.params.type1.position = p1;
				calib.params.type1.velocity = p2;
			break;

			case eomc_calibration_type2_hard_stops_diff:
				calib.type = eomc_calibration_type2_hard_stops_diff;
				calib.params.type2.pwmlimit = p1;
				calib.params.type2.velocity = p2;
			break;

			case eomc_calibration_type3_abs_sens_digital:
				calib.type = eomc_calibration_type3_abs_sens_digital;
				calib.params.type3.position = p1;
				calib.params.type3.velocity = p2;
				calib.params.type3.offset   = p3;
			break;

			case eomc_calibration_type4_abs_and_incremental:
				calib.type = eomc_calibration_type4_abs_and_incremental;
				calib.params.type4.position   = p1;
				calib.params.type3.velocity   = p2;
				calib.params.type4.maxencoder = p3;
			break;
			default:
				print_debug(AC_error_file, ">> ERROR: Calibration type unknown!! (embObjMotionControl)\n");
				return false;
				break;
		}

		if( eores_OK != eo_nv_Set(nvRoot, &calib, eobool_true, eo_nv_upd_dontdo))
		{
			print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
			return false;
		}
		res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid);
    return true;
}

bool embObjMotionControl::doneRaw(int axis)
{
	print_debug(AC_trace_file, "embObjMotionControl::doneRaw\n");

	// get the control mode and check its value
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)axis, jointNVindex_jstatus__basic);
	EOnv	*nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_ask, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid);

	// Sign up for waiting the reply
	eoThreadEntry *tt = appendWaitRequest(axis, nvid);
	tt->setPending(1);

	// wait here
	if(-1 == tt->synch() )
	{
		int threadId;
		printf("embObjMotionControl::getPidRaw timed out!!\n");
		if(requestQueue->threadPool->getId(&threadId))
			requestQueue->cleanTimeouts(threadId);
		return false;
	}

	uint16_t size;
	eOmc_controlmode_t type;
	res->transceiver->getNVvalue(nvRoot, (uint8_t*) &type, &size);


	// if the value I get back from the board corresponds to the calibration type that was imposed, then the calibration is done!
	// for now just check if it is any of the calibration types.
	if( ( eomc_controlmode_calib_abs_pos_sens == type) || ( eomc_controlmode_calib_hard_stops == type) ||	( eomc_controlmode_handle_hard_stops == type) || ( eomc_controlmode_margin_reached == type) || ( eomc_controlmode_calib_abs_and_inc) )
		return true;
	else
		return false;
}

////////////////////////////////////////
//     Position control interface     //
////////////////////////////////////////

bool embObjMotionControl::getAxes(int *ax)
{
	*ax=_njoints;
	return true;
}

bool embObjMotionControl::setPositionModeRaw()
{
	print_debug(AC_trace_file, "embObjMotionControl::setVelocityModeRaw()");

	bool ret = true;
	eOnvID_t  nvid[_njoints];
	EOnv	  *nvRoot[_njoints];

	eOmc_controlmode_t val = eomc_controlmode_position;
	for(int j=_firstJoint, index=0; j<_firstJoint+_njoints; j++, index++)
	{
		nvid[index]   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jconfig__controlmode);
		nvRoot[index] = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid[index]);

		if(NULL == nvRoot[index])
		{
			NV_NOT_FOUND;
			ret = false;
		}

		if( eores_OK != eo_nv_Set(nvRoot[index], &val, eobool_true, eo_nv_upd_dontdo))
		{
			print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
			return false;
		}

		res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid[index]);
	}
	return ret;
}

bool embObjMotionControl::positionMoveRaw(int j, double ref)
{
	print_debug(AC_trace_file, "embObjMotionControl::velocityMoveRaw()");

	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jcmmnds__setpoint);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);
	int index = j-_firstJoint;

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	_ref_positions[index] = ref;   // save internally the new value of speed.

	eOmc_setpoint_t setpoint;
	setpoint.type = eomc_setpoint_velocity;
	setpoint.to.position.value =  _ref_positions[index];
	setpoint.to.position.withvelocity = _ref_speeds[index];

	if( eores_OK != eo_nv_Set(nvRoot, &setpoint, eobool_true, eo_nv_upd_dontdo))
	{
		print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid);
	return true;
}

bool embObjMotionControl::positionMoveRaw(const double *refs)
{
	bool ret = true;
	for(int j=_firstJoint, index=0; j<_firstJoint+_njoints; j++, index++)
	{
		ret &= positionMoveRaw(j, refs[index]);
	}
	return ret;
}

bool embObjMotionControl::relativeMoveRaw(int j, double delta)
{
    return NOT_YET_IMPLEMENTED("positionRelative");
}

bool embObjMotionControl::relativeMoveRaw(const double *deltas)
{
    return NOT_YET_IMPLEMENTED("positionRelative");
}

bool embObjMotionControl::checkMotionDoneRaw(int j, bool *flag)
{
	eOnvID_t nvid_config = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jconfig__motionmonitormode);
	EOnv *nvRoot_config = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid_config);

	if(NULL == nvRoot_config)
	{
		NV_NOT_FOUND;
		return false;
	}

	// monitor status until set point is reached, if it wasn't already set
	// this is because the function has to be in a non blocking fashion and I want to avoid resending the same message over and over again!!

	eOmc_joint_config_t *val = (eOmc_joint_config_t *) nvRoot_config->loc; //eomc_motionmonitormode_untilreached;
	if( eomc_motionmonitormode_dontmonitor == val->motionmonitormode)
	{
		eOmc_motionmonitormode_t tmp = eomc_motionmonitormode_forever;
		if( eores_OK != eo_nv_Set(nvRoot_config, &tmp, eobool_true, eo_nv_upd_dontdo))
		{
			print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
			return false;
		}
		res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid_config);
	}


	// Read the actual current value
	eOnvID_t nvid_status = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jstatus__basic);
	EOnv *nvRoot_status = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid_status);

	uint16_t size;
	eOmc_joint_status_basic_t status;
	res->transceiver->getNVvalue(nvRoot_status, (uint8_t *) &status, &size);
	if(eomc_motionmonitorstatus_setpointisreached == status.motionmonitorstatus)
	{
		*flag = true;

	// to stop monitoring when set point is reached... this create problems when using the other version of
	// the function with all axis togheter. A for loop cannot be used. Skip it for now
//		eOmc_motionmonitormode_t tmp = eomc_motionmonitormode_dontmonitor;
//		if( eomc_motionmonitormode_dontmonitor != *nvRoot_config->motionmonitormode)
//		{
//			if( eores_OK != eo_nv_Set(nvRoot_config, &val, eobool_true, eo_nv_upd_dontdo))
//			{
//				print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
//				return false;
//			}
//			res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid_config);
//		}
	}
	else
		*flag = false;

	return true;
}

bool embObjMotionControl::checkMotionDoneRaw(bool *flag)
{
	bool ret = true;
	bool val, tot_res = true;

	for(int j=_firstJoint, index=0; j<_firstJoint+_njoints; j++, index++)
	{
		ret &= checkMotionDoneRaw(&val);
		tot_res &= val;
	}
	*flag = tot_res;
	return ret;
}

bool embObjMotionControl::setRefSpeedRaw(int j, double sp)
{
	print_debug(AC_trace_file, "embObjMotionControl::setRefSpeedRaw()");
	// Velocity is expressed in iDegrees/s
	// save internally the new value of speed; it'll be used in the positionMove
	int index = j-_firstJoint;
	_ref_speeds[index] = sp;

	return true;
}

bool embObjMotionControl::setRefSpeedsRaw(const double *spds)
{
	// Velocity is expressed in iDegrees/s
	// save internally the new value of speed; it'll be used in the positionMove
	for(int j=_firstJoint, index=0; j<_firstJoint+_njoints; j++, index++)
	{
		_ref_speeds[index] = spds[index];
	}
	return true;
}

bool embObjMotionControl::setRefAccelerationRaw(int j, double acc)
{
	// Acceleration is expressed in iDegrees/s^2
	// save internally the new value of the acceleration; it'll be used in the velocityMove command
    _ref_accs[j-_firstJoint] = acc;

    return true;
}

bool embObjMotionControl::setRefAccelerationsRaw(const double *accs)
{
	// Acceleration is expressed in iDegrees/s^2
	// save internally the new value of the acceleration; it'll be used in the velocityMove command
	for(int j=_firstJoint, index=0; j<_firstJoint+_njoints; j++, index++)
	{
		_ref_accs[index] = accs[j];
	}
	return true;
}

bool embObjMotionControl::getRefSpeedRaw(int j, double *spd)
{
    *spd = _ref_speeds[j-_firstJoint];
    return true;
}

bool embObjMotionControl::getRefSpeedsRaw(double *spds)
{
    memcpy(spds, _ref_speeds, sizeof(double) * _njoints);
    return true;
}

bool embObjMotionControl::getRefAccelerationRaw(int j, double *acc)
{
    *acc = _ref_accs[j-_firstJoint];
    return true;
}

bool embObjMotionControl::getRefAccelerationsRaw(double *accs)
{
    memcpy(accs, _ref_speeds, sizeof(double) * _njoints);
    return true;
}

bool embObjMotionControl::stopRaw(int j)
{
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jcmmnds__stoptrajectory);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	eObool_t stop = eobool_true;

	if( eores_OK != eo_nv_Set(nvRoot, &stop, eobool_true, eo_nv_upd_dontdo))
	{
		print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid);
	return true;
}

bool embObjMotionControl::stopRaw()
{
	bool ret = true;
	for(int j=_firstJoint; j<_firstJoint+_njoints; j++)
	{
		ret &= stopRaw(j);
	}
	return ret;
}
///////////// END Position Control INTERFACE  //////////////////

  // ControlMode
bool embObjMotionControl::setPositionModeRaw(int j)
{
	eOnvID_t  			nvid;
	EOnv	  			*nvRoot;
	eOmc_controlmode_t 	val = eomc_controlmode_position;

	nvid   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jconfig__controlmode);
	nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	if( eores_OK != eo_nv_Set(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid);

	return true;
}

bool embObjMotionControl::setVelocityModeRaw(int j)
{
	eOnvID_t  			nvid;
	EOnv	  			*nvRoot;
	eOmc_controlmode_t 	val = eomc_controlmode_velocity;

	nvid   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jconfig__controlmode);
	nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	if( eores_OK != eo_nv_Set(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid);

	return true;
}

bool embObjMotionControl::setTorqueModeRaw(int j)
{
	eOnvID_t  			nvid;
	EOnv	  			*nvRoot;
	eOmc_controlmode_t 	val = eomc_controlmode_torque;

	nvid   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jconfig__controlmode);
	nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	if( eores_OK != eo_nv_Set(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid);

	return true;
}

bool embObjMotionControl::setImpedancePositionModeRaw(int j)
{
	eOnvID_t  			nvid;
	EOnv	  			*nvRoot;
	eOmc_controlmode_t 	val = eomc_controlmode_impedance_pos;

	nvid   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jconfig__controlmode);
	nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	if( eores_OK != eo_nv_Set(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid);

	return true;
}

bool embObjMotionControl::setImpedanceVelocityModeRaw(int j)
{
	eOnvID_t  			nvid;
	EOnv	  			*nvRoot;
	eOmc_controlmode_t 	val = eomc_controlmode_impedance_vel;

	nvid   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jconfig__controlmode);
	nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	if( eores_OK != eo_nv_Set(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid);

	return true;
}

bool embObjMotionControl::setOpenLoopModeRaw(int j)
{
	eOnvID_t  			nvid;
	EOnv	  			*nvRoot;
	eOmc_controlmode_t 	val = eomc_controlmode_openloop;

	nvid   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jconfig__controlmode);
	nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	if( eores_OK != eo_nv_Set(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid);

	return true;
}

bool embObjMotionControl::getControlModeRaw(int j, int *v)
{
	print_debug(AC_trace_file, "embObjMotionControl::getControlModeRaw()");

	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jstatus__basic);
	EOnv	*nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_ask, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid);

	// Sign up for waiting the reply
	eoThreadEntry *tt = appendWaitRequest(j, nvid);
	tt->setPending(1);

	// wait here
	if(-1 == tt->synch() )
	{
		int threadId;
		printf("embObjMotionControl::getPidRaw timed out!!\n");
		if(requestQueue->threadPool->getId(&threadId))
			requestQueue->cleanTimeouts(threadId);
		return false;
	}

	uint16_t size;
	eOmc_controlmode_t type;
	res->transceiver->getNVvalue(nvRoot, (uint8_t*) &type, &size);

	switch(type)
	{
		case eomc_controlmode_idle:
	        *v=VOCAB_CM_IDLE;
			break;

		case eomc_controlmode_position:
	        *v=VOCAB_CM_POSITION;
			break;

		case eomc_controlmode_velocity:
	        *v=VOCAB_CM_VELOCITY;
			break;

		case eomc_controlmode_torque:
	        *v=VOCAB_CM_TORQUE;
			break;

		case eomc_controlmode_impedance_pos:
	        *v=VOCAB_CM_IMPEDANCE_POS;
			break;

		case eomc_controlmode_impedance_vel:
	        *v=VOCAB_CM_IMPEDANCE_VEL;
			break;

		case eomc_controlmode_openloop:
	        *v=VOCAB_CM_OPENLOOP;
			break;

		default:
	        *v=VOCAB_CM_UNKNOWN;
			break;
	}
	return true;
}

bool embObjMotionControl::getControlModesRaw(int* v)
{
	bool ret = true;
	for(int j=_firstJoint, index=0; j<_firstJoint+_njoints; j++, index++)
	{
		ret &= getControlModeRaw(j, &v[index]);
	}
	return ret;
}

//////////////////////// BEGIN EncoderInterface

bool embObjMotionControl::setEncoderRaw(int j, double val)
{
	return NOT_YET_IMPLEMENTED("setEncoder");
}

bool embObjMotionControl::setEncodersRaw(const double *vals)
{
	return NOT_YET_IMPLEMENTED("setEncoders");
}

bool embObjMotionControl::resetEncoderRaw(int j)
{
	return NOT_YET_IMPLEMENTED("resetEncoder");
}

bool embObjMotionControl::resetEncodersRaw()
{
	return NOT_YET_IMPLEMENTED("resetEncoders");
}

bool embObjMotionControl::getEncoderRaw(int j, double *value)
{
	// Check if j is valid for this specific instance of embObjMotionControl, i.e. if this joint is actually controlled by
	// the EMS I'm referring to.

	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jstatus__basic);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	eOmc_joint_status_basic_t  tmpJointStatus;
	uint16_t size;
	res->transceiver->getNVvalue(nvRoot, (uint8_t*) &tmpJointStatus, &size);
	*value = (double) tmpJointStatus.position;
	return true;
}

bool embObjMotionControl::getEncodersRaw(double *encs)
{
	bool ret = true;
	for(int j=_firstJoint; j<_firstJoint+_njoints;j++)
	{
		ret &= getEncoderRaw(j, &encs[j]);
	}
	return ret;
}

bool embObjMotionControl::getEncoderSpeedRaw(int j, double *sp)
{
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jstatus__basic);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	eOmc_joint_status_basic_t  tmpJointStatus;
	uint16_t size;
	res->transceiver->getNVvalue(nvRoot,  (uint8_t*) &tmpJointStatus, &size);
	*sp = (double) tmpJointStatus.velocity;
	return true;
}

bool embObjMotionControl::getEncoderSpeedsRaw(double *spds)
{
	bool ret = true;
	for(int j=_firstJoint; j<_firstJoint+_njoints;j++)
	{
		ret &= getEncoderSpeedRaw(j, &spds[j]);
	}
	return ret;
}

bool embObjMotionControl::getEncoderAccelerationRaw(int j, double *acc)
{
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jstatus__basic);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	eOmc_joint_status_basic_t  tmpJointStatus;
	uint16_t size;
	res->transceiver->getNVvalue(nvRoot,  (uint8_t*) &tmpJointStatus, &size);
	*acc = (double) tmpJointStatus.acceleration;
	return true;
}

bool embObjMotionControl::getEncoderAccelerationsRaw(double *accs)
{
	bool ret = true;
	for(int j=_firstJoint; j<_firstJoint+_njoints;j++)
	{
		ret &= getEncoderSpeedRaw(j, &accs[j]);
	}
	return ret;
}

///////////////////////// END Encoder Interface

bool embObjMotionControl::getEncodersTimedRaw(double *encs, double *stamps) { }
bool embObjMotionControl::getEncoderTimedRaw(int j, double *encs, double *stamp) { }

////// Amplifier interface
//
bool embObjMotionControl::enableAmpRaw(int j)
{
	_enabledAmp[j-_firstJoint] = true;

	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jconfig__controlmode);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	eOmc_controlmode_t val = eomc_controlmode_idle;
	if( eores_OK != eo_nv_Set(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid);
	return true;

}

bool embObjMotionControl::disableAmpRaw(int j)
{
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jconfig__controlmode);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	_enabledAmp[j-_firstJoint] = false;
	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	eOmc_controlmode_t val = eomc_controlmode_switch_everything_off;
	if( eores_OK != eo_nv_Set(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid);
	return true;
}

bool embObjMotionControl::getCurrentRaw(int j, double *value)
{
	eOnvID_t nvid = eo_cfg_nvsEP_mc_motor_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_motorNumber_t) j, motorNVindex_mstatus__basic);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	eOmc_motor_status_basic_t  tmpMotorStatus;
	uint16_t size;
	res->transceiver->getNVvalue(nvRoot, (uint8_t*) &tmpMotorStatus, &size);
	*value = (double) tmpMotorStatus.current;
	return true;
}

bool embObjMotionControl::getCurrentsRaw(double *vals)
{
	bool ret = true;
	for(int j=_firstJoint; j<_firstJoint+_njoints;j++)
	{
		ret &= getCurrentRaw(j, &vals[j]);
	}
	return ret;
}

bool embObjMotionControl::setMaxCurrentRaw(int j, double val)
{
	eOnvID_t nvid = eo_cfg_nvsEP_mc_motor_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_motorNumber_t) j, motorNVindex_mconfig__maxcurrentofmotor);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	eOmeas_current_t  maxCurrent = (eOmeas_current_t) val;

	if( eores_OK != eo_nv_Set(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid);

	return true;
}

bool embObjMotionControl::getAmpStatusRaw(int j, int *st)
{
	(_enabledAmp[j-_firstJoint]) ? *st = 1 : *st = 0;
	return true;
}

bool embObjMotionControl::getAmpStatusRaw(int *sts)
{
	bool ret = true;
	for(int j=0; j<_njoints;j++)
	{
		sts[j] = _enabledAmp[j];
	}
	return ret;
}
