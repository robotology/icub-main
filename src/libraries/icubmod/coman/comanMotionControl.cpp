// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

//#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <string.h>
#include <iostream>

#include <yarp/os/Time.h>

#include "comanMotionControl.h"

#include "Debug.h"


using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;

#warning "Macro EMS_capacityofropframeregulars defined by hand!! Find a way to have this number synchronized with EMS!!"
#define EMS_capacityofropframeregulars 1204

// Utilities

void copyPid_iCub2eo(const Pid *in, eOmc_PID_t *out)
{
	out->kp = (int16_t)in->kp;
	out->ki = (int16_t)in->ki;
	out->kd = (int16_t)in->kd;
	out->limitonintegral = (int16_t)in->max_int;
	out->limitonoutput = (int16_t)in->max_output;
	out->offset = (int16_t)in->offset;
	out->scale = (int8_t)in->scale;
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

// This will be moved in the ImplXXXInterface
double convertA2I(double angle_in_degrees, double zero, double factor)
{
	return (angle_in_degrees + zero) * factor;
}

inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    fprintf(stderr, "%s not yet implemented for comanMotionControl\n", txt);

    return false;
}

#define NV_NOT_FOUND	return nv_not_found();

bool nv_not_found(void)
{
	yError () << " nv_not_found!! This may mean that this variable is not handled by this EMS\n";
	return false;
}

void comanMotionControl::getMStatus(int j)
{
		NOT_YET_IMPLEMENTED("getMstatus");
}

//generic function that check is key1 is present in input bottle and that the result has size elements
// return true/false
bool comanMotionControl::extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        yError () << key1.c_str() << " not found\n";
        return false;
    }

    if(tmp.size()!=size)
    {
        yError () << key1.c_str() << " incorrect number of entries in board " << _fId.name << '[' << _fId.boardNum << ']';
        return false;
    }

    out=tmp;
    return true;
}


bool comanMotionControl::alloc(int nj)
{
    _axisMap = allocAndCheck<int>(nj);
    _angleToEncoder = allocAndCheck<double>(nj);
    _encoderconversionoffset = allocAndCheck<float>(nj);
    _encoderconversionfactor = allocAndCheck<float>(nj);

    _rotToEncoder = allocAndCheck<double>(nj);
    _zeros = allocAndCheck<double>(nj);
    _torqueSensorId= allocAndCheck<int>(nj);
    _torqueSensorChan= allocAndCheck<int>(nj);
    _maxTorque=allocAndCheck<double>(nj);
    _newtonsToSensor=allocAndCheck<double>(nj);

    _pids=allocAndCheck<Pid>(nj);
    _tpids=allocAndCheck<Pid>(nj);

    _impedance_params=allocAndCheck<ImpedanceParameters>(nj);
    _impedance_limits=allocAndCheck<ImpedanceLimits>(nj);
    _estim_params=allocAndCheck<SpeedEstimationParameters>(nj);

    _limitsMax=allocAndCheck<double>(nj);
    _limitsMin=allocAndCheck<double>(nj);
    _currentLimits=allocAndCheck<double>(nj);
    checking_motiondone=allocAndCheck<bool>(nj);

    _velocityShifts=allocAndCheck<int>(nj);
    _velocityTimeout=allocAndCheck<int>(nj);

    // Reserve space for data stored locally. values are initialize to 0
    _ref_positions = allocAndCheck<double>(nj);
    _command_speeds = allocAndCheck<double>(nj);
    _ref_speeds = allocAndCheck<double>(nj);
    _ref_accs = allocAndCheck<double>(nj);
    _ref_torques = allocAndCheck<double>(nj);
    _enabledAmp = allocAndCheck<bool>(nj);
    _enabledPid = allocAndCheck<bool>(nj);
    _calibrated = allocAndCheck<bool>(nj);

    //	_debug_params=allocAndCheck<DebugParameters>(nj);


    return true;
}

comanMotionControl::comanMotionControl() :
    ImplementControlCalibration2<comanMotionControl, IControlCalibration2>(this),
    ImplementAmplifierControl<comanMotionControl, IAmplifierControl>(this),
    ImplementPidControl<comanMotionControl, IPidControl>(this),
    ImplementEncodersTimed(this),
    ImplementPositionControl<comanMotionControl, IPositionControl>(this),
    ImplementVelocityControl<comanMotionControl, IVelocityControl>(this),
    ImplementControlMode(this),
    ImplementDebugInterface(this),
    ImplementControlLimits<comanMotionControl, IControlLimits>(this),
    _mutex(1)
{
		udppkt_data 	= 0x00;
    udppkt_size 	= 0x00;

    _pids			= NULL;
    _tpids			= NULL;
    _firstJoint 	= 0;
    res				= NULL;
    requestQueue 	= NULL;
    _tpidsEnabled	= false;
    _njoints 		= 0;

    _axisMap		= NULL;
    _angleToEncoder = NULL;
    _zeros			= NULL;
    _encoderconversionfactor = NULL;
    _encoderconversionoffset = NULL;

    _impedance_params	= NULL;
    _impedance_limits	= NULL;
    _estim_params		= NULL;

    _limitsMin = NULL;
    _limitsMax = NULL;
    _currentLimits = NULL;
    _velocityShifts = NULL;
    _velocityTimeout = NULL;
    _torqueSensorId = NULL;
    _torqueSensorChan = NULL;
    _maxTorque = NULL;
    _newtonsToSensor = NULL;
    _rotToEncoder	= NULL;
    _ref_accs		= NULL;
    _command_speeds = NULL;
    _ref_positions 	= NULL;
    _ref_speeds		= NULL;
    _ref_torques	= NULL;

    checking_motiondone = NULL;
    // debug connection
    tot_packet_recv	= 0;
    errors			= 0;
    start			= 0;
    end				= 0;

    // Check status of joints
    _enabledPid		= NULL;
    _enabledAmp 	= NULL;
    _calibrated		= NULL;
}

comanMotionControl::~comanMotionControl()
{
	yTrace();
}

bool comanMotionControl::open(yarp::os::Searchable &config)
{
	 yTrace();
	return NOT_YET_IMPLEMENTED("open");

//
//  		INIT ALL INTERFACES
//
//     ImplementControlCalibration2<comanMotionControl, IControlCalibration2>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
//     ImplementAmplifierControl<comanMotionControl, IAmplifierControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
//     ImplementEncodersTimed::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
//     ImplementPositionControl<comanMotionControl, IPositionControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
//     ImplementPidControl<comanMotionControl, IPidControl>:: initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
//     ImplementControlMode::initialize(_njoints, _axisMap);
//     ImplementVelocityControl<comanMotionControl, IVelocityControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
//     ImplementDebugInterface::initialize(_njoints, _axisMap, _angleToEncoder, _zeros, _rotToEncoder);
//     ImplementControlLimits<comanMotionControl, IControlLimits>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);

    return true;
}


bool comanMotionControl::fromConfig(yarp::os::Searchable &config)
{
	// yTrace();
	Bottle xtmp;
	int i;
    Bottle general = config.findGroup("GENERAL");

	// leggere i valori da file
	if (!extractGroup(general, xtmp, "AxisMap", "a list of reordered indices for the axes", _njoints+1))
		return false;

	for (i = 1; i < xtmp.size(); i++)
		_axisMap[i-1] = xtmp.get(i).asInt();

	// extract info about the first joint controlled by this EMS.
	_firstJoint = _axisMap[0];
	for (i = 1; i < _njoints; i++)
		if(_axisMap[i] <= _firstJoint)
			_firstJoint = _axisMap[i];

	double tmp_A2E;
	// Encoder scales
	if (!extractGroup(general, xtmp, "Encoder", "a list of scales for the encoders", _njoints+1))
		return false;
	else
		for (i = 1; i < xtmp.size(); i++)
		{
//			_angleToEncoder[i-1] = xtmp.get(i).asDouble();
			_angleToEncoder[i-1] = (1<<16) / 360.0;		// conversion factor from degrees to iCubDegrees
			tmp_A2E = xtmp.get(i).asDouble();
			_encoderconversionfactor[i-1] = (float) (tmp_A2E / _angleToEncoder[i-1]);
			_encoderconversionoffset[i-1] = 0;
		}


	// Rotor scales
	if (!extractGroup(general, xtmp, "Rotor", "a list of scales for the rotor encoders", _njoints+1))
	{
		fprintf(stderr, "Using default value = 1\n");
		for(i=1;i<_njoints+1; i++)
			_rotToEncoder[i-1] = 1.0;
	}
	else
	{
		int test = xtmp.size();
		for (i = 1; i < xtmp.size(); i++)
			_rotToEncoder[i-1] = xtmp.get(i).asDouble();
	}

	// Zero Values
	if (!extractGroup(general, xtmp, "Zeros","a list of offsets for the zero point", _njoints+1))
		return false;
	else
		for (i = 1; i < xtmp.size(); i++)
			_zeros[i-1] = xtmp.get(i).asDouble();


	// Torque Id
    if (!extractGroup(general, xtmp, "TorqueId","a list of associated joint torque sensor ids", _njoints+1))
    {
        fprintf(stderr, "Using default value = 0 (disabled)\n");
        for(i=1;i<_njoints+1; i++)
            _torqueSensorId[i-1] = 0;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++) _torqueSensorId[i-1] = xtmp.get(i).asInt();
    }


    if (!extractGroup(general, xtmp, "TorqueChan","a list of associated joint torque sensor channels", _njoints+1))
    {
        fprintf(stderr, "Using default value = 0 (disabled)\n");
        for(i=1;i<_njoints+1; i++)
            _torqueSensorChan[i-1] = 0;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++) _torqueSensorChan[i-1] = xtmp.get(i).asInt();
    }

    if (!extractGroup(general, xtmp, "TorqueMax","full scale value for a joint torque sensor", _njoints+1))
    {
        fprintf(stderr, "Using default value = 0\n");
        for(i=1;i<_njoints+1; i++)
        {
                _maxTorque[i-1] = 0;
                _newtonsToSensor[i-1]=1;
        }
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
        {
                _maxTorque[i-1] = xtmp.get(i).asInt();
                _newtonsToSensor[i-1] = double(0x8000)/double(_maxTorque[i-1]);
        }
    }

    ////// PIDS
    Bottle pidsGroup=config.findGroup("PIDS", "PID parameters");
    if (pidsGroup.isNull()) {
            fprintf(stderr, "Error: no PIDS group found in config file, returning\n");
            return false;
    }

    int j=0;
    for(j=0;j<_njoints;j++)
    {
        char tmp[80];
        sprintf(tmp, "Pid%d", j);

        Bottle &xtmp2 = pidsGroup.findGroup(tmp);
        _pids[j].kp = xtmp2.get(1).asDouble();
        _pids[j].kd = xtmp2.get(2).asDouble();
        _pids[j].ki = xtmp2.get(3).asDouble();

        _pids[j].max_int = xtmp2.get(4).asDouble();
        _pids[j].max_output = xtmp2.get(5).asDouble();

        _pids[j].scale = xtmp2.get(6).asDouble();
        _pids[j].offset = xtmp2.get(7).asDouble();
    }


    ////// TORQUE PIDS
    Bottle TPidsGroup=config.findGroup("TORQUE_PIDS","TORQUE_PID parameters");
       if (TPidsGroup.isNull())
       {
               fprintf(stderr, "Error: no TORQUE PIDS group found in config file, skipping\n");
//               return false;
       }
       else
       {
    	   printf("Torque Pids section found\n");
    	   _tpidsEnabled=true;
    	   for(j=0;j<_njoints;j++)
    	   {
    		   char str1[80];
    		   sprintf(str1, "TPid%d", j);
    		 //  Bottle &xtmp3 = config.findGroup("TORQUE_PIDS","TORQUE_PID parameters").findGroup(str1);

    		   _tpids[j].kp = TPidsGroup.get(1).asDouble();
    		   _tpids[j].kd = TPidsGroup.get(2).asDouble();
    		   _tpids[j].ki = TPidsGroup.get(3).asDouble();

    		   _tpids[j].max_int = TPidsGroup.get(4).asDouble();
    		   _tpids[j].max_output = TPidsGroup.get(5).asDouble();

    		   _tpids[j].scale = TPidsGroup.get(6).asDouble();
    		   _tpids[j].offset = TPidsGroup.get(7).asDouble();
    	   }
       }


#warning "What to do with impedance??"

    ////// IMPEDANCE DEFAULT VALUES
    if (!extractGroup(general, xtmp, "TorqueChan","a list of associated joint torque sensor channels", _njoints+1))
    {
        fprintf(stderr, "Using default value = 0 (disabled)\n");
        for(i=1;i<_njoints+1; i++)
            _torqueSensorChan[i-1] = 0;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++) _torqueSensorChan[i-1] = xtmp.get(i).asInt();
    }

    if (general.check("IMPEDANCE","DEFAULT IMPEDANCE parameters")==true)
    {
        fprintf(stderr, "IMPEDANCE parameters section found\n");
        for(j=0;j<_njoints;j++)
        {
            char str2[80];
            sprintf(str2, "Imp%d", j);
            if (config.findGroup("IMPEDANCE","DEFAULT IMPEDANCE parameters").check(str2)==true)
            {
                xtmp = config.findGroup("IMPEDANCE","DEFAULT IMPEDANCE parameters").findGroup(str2);
                _impedance_params[j].enabled=true;
                _impedance_params[j].stiffness = xtmp.get(1).asDouble();
                _impedance_params[j].damping   = xtmp.get(2).asDouble();
            }
        }
    }
    else
    {
        printf("Impedance section NOT enabled, skipping...\n");
    }

	////// IMPEDANCE LIMITS (UNDER TESTING)
	for(j=0;j<_njoints;j++)
	{
		_impedance_limits[j].min_damp=  0.001;
		_impedance_limits[j].max_damp=  9.888;
		_impedance_limits[j].min_stiff= 0.002;
		_impedance_limits[j].max_stiff= 9.889;
		_impedance_limits[j].param_a=   0.011;
		_impedance_limits[j].param_b=   0.012;
		_impedance_limits[j].param_c=   0.013;
	}

    /////// LIMITS
    Bottle &limits=config.findGroup("LIMITS");
    if (limits.isNull())
    {
        fprintf(stderr, "Group LIMITS not found in configuration file\n");
        return false;
    }
    	// current limit
    if (!extractGroup(limits, xtmp, "Currents","a list of current limits", _njoints+1))
        return false;
    else
    	for(i=1;i<xtmp.size(); i++) _currentLimits[i-1]=xtmp.get(i).asDouble();

    	// max limit
    if (!extractGroup(limits, xtmp, "Max","a list of maximum angles (in degrees)", _njoints+1))
        return false;
    else
    	for(i=1;i<xtmp.size(); i++) _limitsMax[i-1]=xtmp.get(i).asDouble();

    	// min limit
    if (!extractGroup(limits, xtmp, "Min","a list of minimum angles (in degrees)", _njoints+1))
        return false;
    else
    	for(i=1;i<xtmp.size(); i++) _limitsMin[i-1]=xtmp.get(i).asDouble();

    /////// [VELOCITY]
    Bottle &velocityGroup=config.findGroup("VELOCITY");
    if (!velocityGroup.isNull())
    {
    	/////// Shifts
    	if (!extractGroup(velocityGroup, xtmp, "Shifts", "a list of shifts to be used in the vmo control", _njoints+1))
    	{
    		fprintf(stderr, "Using default Shifts=4\n");
    		for(i=1;i<_njoints+1; i++)
    			_velocityShifts[i-1] = 4;   //Default value
    	}
    	else
    	{
    		for(i=1;i<xtmp.size(); i++)
    			_velocityShifts[i-1]=xtmp.get(i).asInt();
    	}

    	/////// Timeout
    	xtmp.clear();
    	if (!extractGroup(velocityGroup, xtmp, "Timeout", "a list of timeout to be used in the vmo control", _njoints+1))
    	{
    		fprintf(stderr, "Using default Timeout=1000, i.e 1s\n");
    		for(i=1;i<_njoints+1; i++)
    			_velocityTimeout[i-1] = 1000;   //Default value
    	}
    	else
    	{
    		for(i=1;i<xtmp.size(); i++)
    			_velocityTimeout[i-1]=xtmp.get(i).asInt();
    	}

    	/////// Joint Speed Estimation
    	xtmp.clear();
    	if (!extractGroup(velocityGroup, xtmp, "JNT_speed_estimation", "a list of shift factors used by the firmware joint speed estimator", _njoints+1))
    	{
    		fprintf(stderr, "Using default value=5\n");
    		for(i=1;i<_njoints+1; i++)
    			_estim_params[i-1].jnt_Vel_estimator_shift = 0;   //Default value
    	}
    	else
    	{
    		for(i=1;i<xtmp.size(); i++)
    			_estim_params[i-1].jnt_Vel_estimator_shift = xtmp.get(i).asInt();
    	}

    	/////// Motor Speed Estimation
    	xtmp.clear();
    	if (!extractGroup(velocityGroup, xtmp, "MOT_speed_estimation", "a list of shift factors used by the firmware motor speed estimator", _njoints+1))
    	{
    		fprintf(stderr, "Using default value=5\n");
    		for(i=1;i<_njoints+1; i++)
    			_estim_params[i-1].mot_Vel_estimator_shift = 0;   //Default value
    	}
    	else
    	{
    		for(i=1;i<xtmp.size(); i++)
    			_estim_params[i-1].mot_Vel_estimator_shift = xtmp.get(i).asInt();
    	}

    	/////// Joint Acceleration Estimation
    	xtmp.clear();
    	if (!extractGroup(velocityGroup, xtmp, "JNT_accel_estimation", "a list of shift factors used by the firmware joint speed estimator", _njoints+1))
    	{
    		fprintf(stderr, "Using default value=5\n");
    		for(i=1;i<_njoints+1; i++)
    			_estim_params[i-1].jnt_Acc_estimator_shift = 0;   //Default value
    	}
    	else
    	{
    		for(i=1;i<xtmp.size(); i++)
    			_estim_params[i-1].jnt_Acc_estimator_shift = xtmp.get(i).asInt();
    	}

    	/////// Motor Acceleration Estimation
    	xtmp.clear();
    	if (!extractGroup(velocityGroup, xtmp, "MOT_accel_estimation", "a list of shift factors used by the firmware motor speed estimator", _njoints+1))
    	{
    		fprintf(stderr, "Using default value=5\n");
    		for(i=1;i<_njoints+1; i++)
    			_estim_params[i-1].mot_Acc_estimator_shift = 5;   //Default value
    	}
    	else
    	{
    		for(i=1;i<xtmp.size(); i++)
    			_estim_params[i-1].mot_Acc_estimator_shift = xtmp.get(i).asInt();
    	}

    }
    else
    {
    	fprintf(stderr, "A suitable value for [VELOCITY] Shifts was not found. Using default Shifts=4\n");
    	for(i=1;i<_njoints+1; i++)
    		_velocityShifts[i-1] = 0;   //Default value // not used now!! In the future this value may (should?) be read from config file and sertnto the EMS

    	fprintf(stderr, "A suitable value for [VELOCITY] Timeout was not found. Using default Timeout=1000, i.e 1s.\n");
    	for(i=1;i<_njoints+1; i++)
    		_velocityTimeout[i-1] = 1000;   //Default value

    	fprintf(stderr, "A suitable value for [VELOCITY] speed estimation was not found. Using default shift factor=5.\n");
    	for(i=1;i<_njoints+1; i++)
    	{
    		_estim_params[i-1].jnt_Vel_estimator_shift = 0;   //Default value
    		_estim_params[i-1].jnt_Acc_estimator_shift = 0;
    		_estim_params[i-1].mot_Vel_estimator_shift = 0;
    		_estim_params[i-1].mot_Acc_estimator_shift = 0;
    	}
    }

//    Bottle& debug = config.findGroup("DEBUG");
//    xtmp.clear();
//    if (!extractGroup(debug, xtmp, "Jconf", "debug joint start", 3))
//    {
//    	start = _firstJoint;
//    	end   = _firstJoint + _njoints;
//    	// yDebug() << "NOT Found debug joint conf start " << start << "end " << end;
//    }
//    else
//    {
//    	start = xtmp.get(1).asInt();
//    	end   = xtmp.get(2).asInt();
//    	// yDebug() << "Found debug joint conf start " << start << "end " << end;
//    }

    return true;
}

bool comanMotionControl::init()
{
	return NOT_YET_IMPLEMENTED("init");
}


bool comanMotionControl::goToRun(void)
{
	NOT_YET_IMPLEMENTED("goToRun");
}

bool comanMotionControl::close()
{
    yTrace();
    ImplementControlMode::uninitialize();
    ImplementEncodersTimed::uninitialize();
    ImplementPositionControl<comanMotionControl, IPositionControl>::uninitialize();
    ImplementVelocityControl<comanMotionControl, IVelocityControl>::uninitialize();
    ImplementPidControl<comanMotionControl, IPidControl>::uninitialize();
    ImplementControlCalibration2<comanMotionControl, IControlCalibration2>::uninitialize();
    ImplementAmplifierControl<comanMotionControl, IAmplifierControl>::uninitialize();

		return NOT_YET_IMPLEMENTED("close");
    return true;
}


eoThreadEntry * comanMotionControl::appendWaitRequest(int j, uint16_t nvid)
{
	// yTrace();
	eoRequest req;
	if(!requestQueue->threadPool->getId(&req.threadId) )
		fprintf(stderr, "Error: too much threads!! (comanMotionControl)");
	req.joint = j;
	req.nvid = res->transceiver->translate_NVid2index(_fId.boardNum, _fId.ep, nvid);

	requestQueue->append(req);
	return requestQueue->threadPool->getThreadTable(req.threadId);
}

///////////// PID INTERFACE

bool comanMotionControl::setPidRaw(int j, const Pid &pid)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setPidRaw");
}

bool comanMotionControl::setPidsRaw(const Pid *pids)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setPidsRaw");
}

bool comanMotionControl::setReferenceRaw(int j, double ref)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setReferenceRaw");
}

bool comanMotionControl::setReferencesRaw(const double *refs)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setReferencesRaw");
}

bool comanMotionControl::setErrorLimitRaw(int j, double limit)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setErrorLimitRaw");
}

bool comanMotionControl::setErrorLimitsRaw(const double *limits)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setErrorLimitsRaw");
}

bool comanMotionControl::getErrorRaw(int j, double *err)
{
		yTrace();
	return NOT_YET_IMPLEMENTED("getErrorRaw");
}

bool comanMotionControl::getErrorsRaw(double *errs)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getErrorsRaw");
}

bool comanMotionControl::getOutputRaw(int j, double *out)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getOutputRaw");
}

bool comanMotionControl::getOutputsRaw(double *outs)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getOutputsRaw");
}

bool comanMotionControl::getPidRaw(int j, Pid *pid)
{
		yTrace();
	return NOT_YET_IMPLEMENTED("getPidRaw");
}

bool comanMotionControl::getPidsRaw(Pid *pids)
{
		yTrace();
	return NOT_YET_IMPLEMENTED("getPidsRaw");
}

bool comanMotionControl::getReferenceRaw(int j, double *ref)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getReferenceRaw");
}

bool comanMotionControl::getReferencesRaw(double *refs)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getReferencesRaw");
}

bool comanMotionControl::getErrorLimitRaw(int j, double *limit)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getErrorLimitRaw");
}

bool comanMotionControl::getErrorLimitsRaw(double *limits)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getErrorLimitsRaw");
}

bool comanMotionControl::resetPidRaw(int j)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("resetPidRaw");
}

bool comanMotionControl::disablePidRaw(int j)
{
		yTrace();
	return NOT_YET_IMPLEMENTED("disablePidRaw");
}

bool comanMotionControl::enablePidRaw(int j)
{
		yTrace();
	return NOT_YET_IMPLEMENTED("enablePidRaw");
}

bool comanMotionControl::setOffsetRaw(int j, double v)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setOffsetRaw");
}

////////////////////////////////////////
//    Velocity control interface raw  //
////////////////////////////////////////

bool comanMotionControl::setVelocityModeRaw()
{
		yTrace();
	return NOT_YET_IMPLEMENTED("setVelocityModeRaw");
}

bool comanMotionControl::velocityMoveRaw(int j, double sp)
{
		yTrace();
	return NOT_YET_IMPLEMENTED("velocityMoveRaw");
}

bool comanMotionControl::velocityMoveRaw(const double *sp)
{
		yTrace();
	return NOT_YET_IMPLEMENTED("velocityMoveRaw");
}


////////////////////////////////////////
//    Calibration control interface   //
////////////////////////////////////////

bool comanMotionControl::calibrate2Raw(int j, unsigned int type, double p1, double p2, double p3)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("calibrate2Raw");
}

bool comanMotionControl::doneRaw(int axis)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("doneRaw");
}

////////////////////////////////////////
//     Position control interface     //
////////////////////////////////////////

bool comanMotionControl::getAxes(int *ax)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getAxes");
}

bool comanMotionControl::setPositionModeRaw()
{
		yTrace();
	return NOT_YET_IMPLEMENTED("setPositionModeRaw");
}

bool comanMotionControl::positionMoveRaw(int j, double ref)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("positionMoveRaw");
}

bool comanMotionControl::positionMoveRaw(const double *refs)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("positionMoveRaw");
}

bool comanMotionControl::relativeMoveRaw(int j, double delta)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("relativeMoveRaw");
}

bool comanMotionControl::relativeMoveRaw(const double *deltas)
{
	yTrace();
    return NOT_YET_IMPLEMENTED("relativeMoveRaw");
}

bool comanMotionControl::checkMotionDoneRaw(int j, bool *flag)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("checkMotionDoneRaw");
}

bool comanMotionControl::setRefSpeedRaw(int j, double sp)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setRefSpeedRaw");
}

bool comanMotionControl::setRefSpeedsRaw(const double *spds)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setRefSpeedsRaw");
}

bool comanMotionControl::setRefAccelerationRaw(int j, double acc)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setRefAccelerationRaw");
}

bool comanMotionControl::setRefAccelerationsRaw(const double *accs)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setRefAccelerationsRaw");
}

bool comanMotionControl::getRefSpeedRaw(int j, double *spd)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getRefSpeedRaw");
}

bool comanMotionControl::getRefSpeedsRaw(double *spds)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getRefSpeedsRaw");
}

bool comanMotionControl::getRefAccelerationRaw(int j, double *acc)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getRefAccelerationRaw");
}

bool comanMotionControl::getRefAccelerationsRaw(double *accs)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getRefAccelerationsRaw");
}

bool comanMotionControl::stopRaw(int j)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("stopRaw");
}

bool comanMotionControl::stopRaw()
{
	yTrace();
	return NOT_YET_IMPLEMENTED("stopRaw");
}
///////////// END Position Control INTERFACE  //////////////////

  // ControlMode
bool comanMotionControl::setPositionModeRaw(int j)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setPositionModeRaw");
}

bool comanMotionControl::setVelocityModeRaw(int j)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setVelocityModeRaw");
}

bool comanMotionControl::setTorqueModeRaw(int j)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setTorqueModeRaw");
}

bool comanMotionControl::setImpedancePositionModeRaw(int j)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setImpedancePositionModeRaw");
}

bool comanMotionControl::setImpedanceVelocityModeRaw(int j)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setImpedanceVelocityModeRaw");
}

bool comanMotionControl::setOpenLoopModeRaw(int j)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setOpenLoopModeRaw");
}

bool comanMotionControl::getControlModeRaw(int j, int *v)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getControlModeRaw");
}

bool comanMotionControl::getControlModesRaw(int* v)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getControlModesRaw");
}

//////////////////////// BEGIN EncoderInterface

bool comanMotionControl::setEncoderRaw(int j, double val)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setEncoder");
}

bool comanMotionControl::setEncodersRaw(const double *vals)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setEncoders");
}

bool comanMotionControl::resetEncoderRaw(int j)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("resetEncoderRaw");
}

bool comanMotionControl::resetEncodersRaw()
{
	yTrace();
	return NOT_YET_IMPLEMENTED("resetEncodersRaw");
}

bool comanMotionControl::getEncoderRaw(int j, double *value)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getEncoderRaw");
}

bool comanMotionControl::getEncodersRaw(double *encs)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getEncodersRaw");
}

bool comanMotionControl::getEncoderSpeedRaw(int j, double *sp)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getEncoderSpeedRaw");
}

bool comanMotionControl::getEncoderSpeedsRaw(double *spds)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getEncoderSpeedsRaw");
}

bool comanMotionControl::getEncoderAccelerationRaw(int j, double *acc)
{
		yTrace();
	return NOT_YET_IMPLEMENTED("getEncoderAccelerationRaw");
}

bool comanMotionControl::getEncoderAccelerationsRaw(double *accs)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getEncoderAccelerationsRaw");
}

///////////////////////// END Encoder Interface

bool comanMotionControl::getEncodersTimedRaw(double *encs, double *stamps)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getEncodersTimedRaw");
}

bool comanMotionControl::getEncoderTimedRaw(int j, double *encs, double *stamp)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getEncoderTimedRaw");
}

////// Amplifier interface
//
bool comanMotionControl::enableAmpRaw(int j)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("enableAmpRaw");
}

bool comanMotionControl::disableAmpRaw(int j)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("disableAmpRaw");
}

bool comanMotionControl::getCurrentRaw(int j, double *value)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getCurrentRaw");
}

bool comanMotionControl::getCurrentsRaw(double *vals)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getCurrentsRaw");
}

bool comanMotionControl::setMaxCurrentRaw(int j, double val)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setMaxCurrentRaw");
}

bool comanMotionControl::getAmpStatusRaw(int j, int *st)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getAmpStatusRaw");
}

bool comanMotionControl::getAmpStatusRaw(int *sts)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("getAmpStatusRaw");
}

//----------------------------------------------\\
//	Debug interface
//----------------------------------------------\\

bool comanMotionControl::setParameterRaw(int j, unsigned int type, double value) 			
{
		yTrace();
	return NOT_YET_IMPLEMENTED("setParameterRaw");
}

bool comanMotionControl::getParameterRaw(int j, unsigned int type, double* value)			
{
		yTrace();
	return NOT_YET_IMPLEMENTED("getParameterRaw");
}

bool comanMotionControl::getDebugParameterRaw(int j, unsigned int index, double* value) 	
{
		yTrace();
	return NOT_YET_IMPLEMENTED("getDebugParameterRaw");
}

bool comanMotionControl::setDebugParameterRaw(int j, unsigned int index, double value) 	
{
		yTrace();
	return NOT_YET_IMPLEMENTED("setDebugParameterRaw");
}

bool comanMotionControl::setDebugReferencePositionRaw(int j, double value) 				
{
		yTrace();
	return NOT_YET_IMPLEMENTED("setDebugReferencePositionRaw");
}

bool comanMotionControl::getDebugReferencePositionRaw(int j, double* value) 				
{
		yTrace();
	return NOT_YET_IMPLEMENTED("getDebugReferencePositionRaw");
}

bool comanMotionControl::getRotorPositionRaw         (int j, double* value) 				
{
		yTrace();
	return NOT_YET_IMPLEMENTED("getRotorPositionRaw");
}

bool comanMotionControl::getRotorPositionsRaw        (double* value)		 				
{
		yTrace();
	return NOT_YET_IMPLEMENTED("getRotorPositionsRaw");
}

bool comanMotionControl::getRotorSpeedRaw            (int j, double* value)		 		
{
		yTrace();
	return NOT_YET_IMPLEMENTED("getRotorSpeedRaw");
}

bool comanMotionControl::getRotorSpeedsRaw           (double* value) 						
{
		yTrace();
	return NOT_YET_IMPLEMENTED("getRotorSpeedsRaw");
}

bool comanMotionControl::getRotorAccelerationRaw     (int j, double* value)		 		
{
		yTrace();
	return NOT_YET_IMPLEMENTED("getRotorAccelerationRaw");
}

bool comanMotionControl::getRotorAccelerationsRaw    (double* value) 						
{
		yTrace();
	return NOT_YET_IMPLEMENTED("getRotorAccelerationsRaw");
}

bool comanMotionControl::getJointPositionRaw         (int j, double* value) 				
{
		yTrace();
	return NOT_YET_IMPLEMENTED("getJointPositionRaw");
}

bool comanMotionControl::getJointPositionsRaw        (double* value) 						
{
		yTrace();
	return NOT_YET_IMPLEMENTED("getJointPositionsRaw");
}


// Limit interface
bool comanMotionControl::setLimitsRaw(int j, double min, double max)
{
	yTrace();
	return NOT_YET_IMPLEMENTED("setLimitsRaw");
}

bool comanMotionControl::getLimitsRaw(int j, double *min, double *max)
{
		yTrace();
	return NOT_YET_IMPLEMENTED("getLimitsRaw");
}


