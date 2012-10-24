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


#include <yarp/os/Log.h>
#include <yarp/os/impl/Logger.h>

// embObj includes

#include "embObjMotionControl.h"

// Boards configurations
#include "EOnv_hid.h"

#include "Debug.h"


using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;

#warning "Macro EMS_capacityofropframeregulars defined by hand!! Find a way to have this number synchronized with EMS!!"
#define EMS_capacityofropframeregulars 1024

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

// This will be moved in the ImplXXXInterface
double convertA2I(double angle_in_degrees)
{
	return angle_in_degrees * 1;
}

inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    fprintf(stderr, "%s not yet implemented for embObjMotionControl\n", txt);

    return false;
}

#define NV_NOT_FOUND	return nv_not_found();

bool nv_not_found(void)
{
	yError() << " nv_not_found!! This may mean that this variable is not handled by this EMS\n";
	return false;
}


//generic function that check is key1 is present in input bottle and that the result has size elements
// return true/false
bool embObjMotionControl::extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        yError() << key1.c_str() << " not found\n";
        return false;
    }

    if(tmp.size()!=size)
    {
        yError() << key1.c_str() << " incorrect number of entries in board " << _fId.name << '[' << _fId.boardNum << ']';
        return false;
    }

    out=tmp;
    return true;
}


bool embObjMotionControl::alloc(int nj)
{
    _axisMap = allocAndCheck<int>(nj);
    _angleToEncoder = allocAndCheck<double>(nj);
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
	udppkt_data 	= 0x00;
	udppkt_size 	= 0x00;

	_pids			= NULL;
	_tpids			= NULL;
	_firstJoint 	= 0;
	_njoints 		= 0;
	res				= NULL;
	requestQueue 	= NULL;
	_tpidsEnabled	= false;

    _axisMap		= NULL;
    _angleToEncoder = NULL;
    _zeros			= NULL;

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

	// debug connection
	tot_packet_recv	= 0;
	errors			= 0;

	// Check status of joints
	_enabledPid		= NULL;
	_enabledAmp 	= NULL;
	_calibrated		= NULL;

	// NV stuff
	NVnumber 		= 0;
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
	yTrace();

	// Debug info
	memset(info, 0x00, SIZE_INFO);
	Bottle xtmp2;
	ACE_TCHAR address[64] = {0};
//	Bottle xtmp = Bottle(config.findGroup("ETH"));
	strcpy(address, config.findGroup("ETH").check("IpAddress",Value(1), " EMS ip address").asString().c_str() );

//	xtmp2 = xtmp.findGroup("IpAddress");
//	strcpy(address, xtmp2.get(1).asString().c_str());
	sprintf(info, "embObjMotionControl - referred to EMS: %s", address);

	//
	// open ethResource, if needed
	//

	ethResCreator *resList = ethResCreator::instance();
	if(NULL == (res = resList->getResource(config)) )
	{
		yError() << "[embObjMotionContro] Unable to instantiate an EMS... check configuration file";
		return false;
	}

	// Defining Unique Id
	_fId.type = MotionControl;
	std::string featId = config.find("FeatId").asString().c_str();
	cout << "FeatId = " << featId << endl;
	strcpy(_fId.name, featId.c_str());


	_fId.boardNum  = 255;
//	Value val =config.findGroup("ETH").check("Ems",Value(1), "Board number");
//	if(val.isInt())
//		_fId.boardNum =val.asInt();
//	else
//		printf("No board number found!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
//
//	yDebug() << "boardNum " << _fId.boardNum;

	Bottle xtmpB = Bottle(config.findGroup("ETH"));
	int boardNum=255;
	xtmp2 = xtmpB.findGroup("Ems");
    if (xtmp2.isNull())
    {
        yError() << "[embObjMotionContro] EMS Board number identifier not found\n";
        return false;
    }

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
			_fId.ep = endpoint_mc_torso;
			break;
		case 6:
			_fId.ep = endpoint_mc_leftupperleg;
			break;
		case 7:
			_fId.ep = endpoint_mc_leftlowerleg;
			break;
		case 8:
			_fId.ep = endpoint_mc_rightupperleg;
			break;
		case 9:
			_fId.ep = endpoint_mc_rightlowerleg;
			break;
		default:
			_fId.ep = 255;
			yError() << "\n eoMotion Control: Wrong board identifier number!!!";
			return false;
			break;
	}


	// Save eo data of this board/EP
	res->transceiver->getHostData(&_fId.EPvector, &_fId.EPhash_function);
	_fId.handle  = (this);

	ethResCreator::instance()->addLUTelement(_fId);
	NVnumber = res->transceiver->getNVnumber(_fId.boardNum, _fId.ep);
	requestQueue = new eoRequestsQueue(NVnumber);


	//
	//	CONFIGURATION
	//

    // get robot parameters
	//Bottle general = config.findGroup("GENERAL");
	_njoints = config.findGroup("GENERAL").check("Joints",Value(1),   "Number of degrees of freedom").asInt();

	alloc(_njoints);
	fromConfig(config);


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



	//  Tell EMS which NV I want to be signalled spontaneously, configure joints and motors and go to running mode
    init();

	return true;
}


bool embObjMotionControl::fromConfig(yarp::os::Searchable &config)
{
	yTrace();
	Bottle xtmp, xtmp2;
	int i;
    Bottle& general = config.findGroup("GENERAL");

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

	// Encoder scales
	if (!extractGroup(general, xtmp, "Encoder", "a list of scales for the encoders", _njoints+1))
		return false;
	else
		for (i = 1; i < xtmp.size(); i++)
			_angleToEncoder[i-1] = xtmp.get(i).asDouble();

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
    Bottle &pidsGroup=config.findGroup("PIDS", "PID parameters");
    if (pidsGroup.isNull()) {
            fprintf(stderr, "Error: no PIDS group found in config file, returning\n");
            return false;
    }

    int j=0;
    for(j=0;j<_njoints;j++)
    {
        char tmp[80];
        sprintf(tmp, "Pid%d", j);

        Bottle &xtmp = pidsGroup.findGroup(tmp);
        _pids[j].kp = xtmp.get(1).asDouble();
        _pids[j].kd = xtmp.get(2).asDouble();
        _pids[j].ki = xtmp.get(3).asDouble();

        _pids[j].max_int = xtmp.get(4).asDouble();
        _pids[j].max_output = xtmp.get(5).asDouble();

        _pids[j].scale = xtmp.get(6).asDouble();
        _pids[j].offset = xtmp.get(7).asDouble();
    }


    ////// TORQUE PIDS
    if (config.check("TORQUE_PIDS","TORQUE_PID parameters")==true)
    {
        printf("Torque Pids section found\n");
        _tpidsEnabled=true;
        for(j=0;j<_njoints;j++)
        {
            char tmp[80];
            sprintf(tmp, "TPid%d", j);
            Bottle &xtmp = config.findGroup("TORQUE_PIDS","TORQUE_PID parameters").findGroup(tmp);

            _tpids[j].kp = xtmp.get(1).asDouble();
            _tpids[j].kd = xtmp.get(2).asDouble();
            _tpids[j].ki = xtmp.get(3).asDouble();

            _tpids[j].max_int = xtmp.get(4).asDouble();
            _tpids[j].max_output = xtmp.get(5).asDouble();

            _tpids[j].scale = xtmp.get(6).asDouble();
            _tpids[j].offset = xtmp.get(7).asDouble();
        }
    }
    else
    {
        fprintf(stderr, "Torque Pids section NOT enabled, skipping...\n");
    }

#warning "What to do with impedance??"

    ////// IMPEDANCE DEFAULT VALUES
    if (config.check("IMPEDANCE","DEFAULT IMPEDANCE parameters")==true)
    {
        fprintf(stderr, "IMPEDANCE parameters section found\n");
        for(j=0;j<_njoints;j++)
        {
            char tmp[80];
            sprintf(tmp, "Imp%d", j);
            if (config.findGroup("IMPEDANCE","DEFAULT IMPEDANCE parameters").check(tmp)==true)
            {
                xtmp = config.findGroup("IMPEDANCE","DEFAULT IMPEDANCE parameters").findGroup(tmp);
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
    			_estim_params[i-1].jnt_Vel_estimator_shift = 5;   //Default value
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
    			_estim_params[i-1].mot_Vel_estimator_shift = 5;   //Default value
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
    			_estim_params[i-1].jnt_Acc_estimator_shift = 5;   //Default value
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
    		_velocityShifts[i-1] = 4;   //Default value

    	fprintf(stderr, "A suitable value for [VELOCITY] Timeout was not found. Using default Timeout=1000, i.e 1s.\n");
    	for(i=1;i<_njoints+1; i++)
    		_velocityTimeout[i-1] = 1000;   //Default value

    	fprintf(stderr, "A suitable value for [VELOCITY] speed estimation was not found. Using default shift factor=5.\n");
    	for(i=1;i<_njoints+1; i++)
    	{
    		_estim_params[i-1].jnt_Vel_estimator_shift = 5;   //Default value
    		_estim_params[i-1].jnt_Acc_estimator_shift = 5;
    		_estim_params[i-1].mot_Vel_estimator_shift = 5;
    		_estim_params[i-1].mot_Acc_estimator_shift = 5;
    	}
    }

    return true;
}



bool embObjMotionControl::init()
{
	yTrace();
	eOmn_ropsigcfg_command_t 	*ropsigcfgassign;
	EOarray						*array;
	eOropSIGcfg_t 				sigcfg;
	int 						old = 0;

	eOnvID_t nvid, nvid_ropsigcfgassign = eo_cfg_nvsEP_mn_comm_NVID_Get(endpoint_mn_comm, 0, commNVindex__ropsigcfgcommand);
	EOnv *nvRoot = res->transceiver->getNVhandler(endpoint_mn_comm, nvid_ropsigcfgassign);

	ropsigcfgassign = (eOmn_ropsigcfg_command_t*) nvRoot->loc;
	array = (EOarray*) &ropsigcfgassign->array;
	eo_array_Reset(array);
	array->head.capacity = NUMOFROPSIGCFG;
	array->head.itemsize = sizeof(eOropSIGcfg_t);
	ropsigcfgassign->cmmnd = ropsigcfg_cmd_append;

	printf("\ropSigCfgcommand nvid = %d (0x%04X)\n", nvid_ropsigcfgassign, nvid_ropsigcfgassign);

	int jStatusSize = sizeof(eOmc_joint_status_t);
	int mStatusSize = sizeof(eOmc_motor_status_basic_t);
	int totSigSize	= 0;

	for(int j=_firstJoint; j<_firstJoint+_njoints; j++)
	{
		yDebug() << "configuring ropSig for joint " << j;

		// Verify that the EMS is able to handle all those data. The macro EOK_HOSTTRANSCEIVER_capacityofropframeregulars has to be the one used by the firmware!!!!
		if( ! (EMS_capacityofropframeregulars >= (totSigSize += jStatusSize)) )
		{
			yError() << "No space left on EMS device for setting new regular messages!! Skipping remaining" << _fId.name;
			break;
		}

		// basterebbero jstatus__basic e jstatus__ofpid, ma la differenza tra questi due e il jstatus completo sono 4 byte, per ora non utilizzati.
		nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, j, jointNVindex_jstatus);
//		printf("\njointNVindex_jstatus nvid = %d (0x%04X)", nvid, nvid);
		if(EOK_uint16dummy == nvid)
		{
			yError() << " NVID jointNVindex_jstatus not found for EndPoint" << _fId.ep << " joint " << j;
		}
		else
		{
			sigcfg.ep = _fId.ep;
			sigcfg.id = nvid;
			sigcfg.plustime = 0;
			if(eores_OK != eo_array_PushBack(array, &sigcfg))
				yError() << " while loading ropSig Array for joint " << j << " at line " << __LINE__;
		}

		// Verify that the EMS is able to handle all those data. The macro EOK_HOSTTRANSCEIVER_capacityofropframeregulars has to be the one used by the firmware!!!!
		if( ! (EMS_capacityofropframeregulars >= (totSigSize += mStatusSize)) )
		{
			yError() << "No space left on EMS device for setting new regular messages!! Skipping remaining on board" << _fId.name;
			break;
		}


		nvid = eo_cfg_nvsEP_mc_motor_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, j, motorNVindex_mstatus__basic);
//		printf("\nmotorNVindex_jstatus nvid = %d (0x%04X)", nvid, nvid);
		if(EOK_uint16dummy == nvid)
		{
			yError() << " NVID jointNVindex_jstatus not found for EndPoint" << _fId.ep << " joint " << j;
		}
		else
		{
			sigcfg.ep = _fId.ep;
			sigcfg.id = nvid;
			sigcfg.plustime = 0;
			if(eores_OK != eo_array_PushBack(array, &sigcfg))
				yError() << " while loading ropSig Array for joint " << j << " at line " << __LINE__;
		}

		if( (NUMOFROPSIGCFG - 1) <= ((j - old +1)*2))	// a ropSigCfg can store only 20 variables at time. If more are needed send 2 messages.
		{
			// A ropsigcfg vector can hold at max NUMOFROPSIGCFG (20) value. If more are needed, send another package,
			// so wait some time to let ethManager send this package and then start again.
			yDebug() << "Maximun number of variables reached in the ropSigCfg array, splitting it in two pieces";
			if( eores_OK != eo_nv_Set(nvRoot, array, eobool_true, eo_nv_upd_dontdo))
			{
				yError() << "ERROR eo_nv_Set !!";
				return false;
			}

			res->transceiver->load_occasional_rop(eo_ropcode_set, endpoint_mn_comm, nvid_ropsigcfgassign);
			Time::delay(1);				// Wait here, the ethManager thread will take care of sending the loaded message
			printf("\n-----------------");
			eo_array_Reset(array);
			array->head.capacity = NUMOFROPSIGCFG;
			array->head.itemsize = sizeof(eOropSIGcfg_t);
			ropsigcfgassign->cmmnd = ropsigcfg_cmd_append;
			old = j;
		}
	}
	// Send remaining stuff
	if( eores_OK != eo_nv_Set(nvRoot, array, eobool_true, eo_nv_upd_dontdo))
	{
		yError() << "ERROR eo_nv_Set !!";
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, endpoint_mn_comm, nvid_ropsigcfgassign);
	Time::delay(0.5);


#warning "TODO: check that all this stuff doesn't go beyond the ropframe size!!!"
	//
	// invia la configurazione dei giunti
	//

	int jConfigSize 	= sizeof(eOmc_joint_config_t);
	int mConfigSize 	= sizeof(eOmc_motor_config_t);
	int totConfigSize	= 0;

	yDebug() << "Sending joint configuration";
	if( EOK_HOSTTRANSCEIVER_capacityofrop < jConfigSize )
	{
		yError() << "Size of Joint Config is bigger than single ROP... cannot send it at all!! Fix it";
	}
	else
	{
		for(int j=_firstJoint, index =0; j<_firstJoint+_njoints; j++, index++)
		{
			yDebug() << " j = " << j << "index = " << index;

			if( ! (EOK_HOSTTRANSCEIVER_capacityofpacket >= (totConfigSize += jConfigSize)) )
			{
				yDebug() << "Too many stuff to be sent at once... splitting in more messages";
				Time::delay(0.1);
				totConfigSize = 0;
			}

			nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, j, jointNVindex_jconfig);
			if(EOK_uint16dummy == nvid)
			{
				yError() << " NVID not found\n";
				return false;
			}
			nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

			if(NULL == nvRoot)
			{
				yError() << " NV pointer found\n" << _fId.name << _fId.boardNum << "at line" << __LINE__;
				return false;
			}


			eOmc_joint_config_t	jconfig;
			memset(&jconfig, 0x00, sizeof(eOmc_joint_config_t));
			copyPid_iCub2eo(&_pids[index],  &jconfig.pidposition);
			copyPid_iCub2eo(&_pids[index],  &jconfig.pidvelocity);
			copyPid_iCub2eo(&_tpids[index], &jconfig.pidtorque);
			// to do
			memset(&jconfig.impedance, 0x00, sizeof(eOmc_impedance_t));

			jconfig.maxpositionofjoint = (eOmeas_position_t) convertA2I(_limitsMax[index]);
			jconfig.minpositionofjoint = (eOmeas_position_t) convertA2I(_limitsMin[index]);
			jconfig.velocitysetpointtimeout = _velocityTimeout[index];
			jconfig.motionmonitormode = eomc_motionmonitorstatus_notmonitored;
			// to do
			jconfig.encoderconversionfactor = 0x00;
			jconfig.encoderconversionoffset = 0x00;

			if( eores_OK != eo_nv_Set(nvRoot, &jconfig, eobool_true, eo_nv_upd_dontdo))
			{
				yError() << "ERROR eo_nv_Set !!";
				return false;
			}
			usleep(2000);
			res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid);
		}
	}

	Time::delay(0.01);
	totConfigSize = 0;
	//
	// invia la configurazione dei motori
	//
	yDebug() << "Sending motor configuration";
	if( EOK_HOSTTRANSCEIVER_capacityofrop < mConfigSize )
	{
		yError() << "Size of Motor Config is bigger than single ROP... cannot send it at all!! Fix it";
	}
	else
	{
		for(int j=_firstJoint, index =0; j<_firstJoint+_njoints;j++, index++)
		{
			yDebug() << " j = " << j << "index = " << index;

			if( ! (EOK_HOSTTRANSCEIVER_capacityofpacket >= (totConfigSize += mConfigSize)) )
			{
				yDebug() << "Too many stuff to be sent at once... splitting in more messages";
				Time::delay(0.1);
				totConfigSize = 0;
			}

			nvid = eo_cfg_nvsEP_mc_motor_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, j, motorNVindex_mconfig);
			if(EOK_uint16dummy == nvid)
			{
				yError() << " NVID not found\n";
				return false;
			}
			nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

			if(NULL == nvRoot)
				yError() << " NV pointer found\n";;


			eOmc_motor_config_t	mconfig;
			memset(&mconfig, 0x00, sizeof(eOmc_motor_config_t));
			// what to do here?
			//mconfig.pidcurrent = unknown;
			//mconfig.maxvelocityofmotor =  ????;
			mconfig.maxcurrentofmotor = _currentLimits[index];

			if( eores_OK != eo_nv_Set(nvRoot, &mconfig, eobool_true, eo_nv_upd_dontdo))
			{
				yError() << "ERROR eo_nv_Set !!";
				return false;
			}
			res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid);
		} }
	Time::delay(0.01);

	// invia configurazioni a caso

	// attiva il loop di controllo
	eOcfg_nvsEP_mn_applNumber_t dummy = 0;  // not used but there for API compatibility
	nvid = eo_cfg_nvsEP_mn_appl_NVID_Get(endpoint_mn_appl, dummy, applNVindex_cmmnds__go2state);
	if(EOK_uint16dummy == nvid)
	{
		yError() << " NVID not found\n";
		return false;
	}

	EOnv 	*nv_p 	= res->transceiver->getNVhandler(endpoint_mn_appl, nvid);
	if(NULL == nv_p)
		yError() << " NV pointer found\n";;

	eOmn_appl_state_t  desired 	= applstate_running;

	if( eores_OK != eo_nv_Set(nv_p, &desired, eobool_true, eo_nv_upd_dontdo))
	{
		yDebug() << "ERROR eo_nv_Set !!\n";
		return false;
	}

	// tell agent to prepare a rop to send
	res->transceiver->load_occasional_rop(eo_ropcode_set, endpoint_mn_appl, nvid);
	nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	return true;
}


bool embObjMotionControl::close()
{
	yTrace();
	ImplementControlMode::uninitialize();
	ImplementEncodersTimed::uninitialize();
	ImplementPositionControl<embObjMotionControl, IPositionControl>::uninitialize();
	ImplementVelocityControl<embObjMotionControl, IVelocityControl>::uninitialize();
	ImplementPidControl<embObjMotionControl, IPidControl>::uninitialize();
    ImplementControlCalibration2<embObjMotionControl, IControlCalibration2>::uninitialize();
    ImplementAmplifierControl<embObjMotionControl, IAmplifierControl>::uninitialize();

    ethResCreator::instance()->removeResource(res);
    return true;
}


eoThreadEntry * embObjMotionControl::appendWaitRequest(int j, uint16_t nvid)
{
	//yTrace();
	eoRequest req;
	if(!requestQueue->threadPool->getId(&req.threadId) )
		fprintf(stderr, "Error: too much threads!! (embObjMotionControl)");
	req.joint = j;
	req.nvid = res->transceiver->translate_NVid2index(_fId.boardNum, _fId.ep, nvid);

	requestQueue->append(req);
//
	return requestQueue->threadPool->getThreadTable(req.threadId);
}

void embObjMotionControl::getMotorController(DeviceDriver *iMC)
{

}


///////////// PID INTERFACE

bool embObjMotionControl::setPidRaw(int j, const Pid &pid)
{
	yTrace();

	// Check if j is valid for this specific instance of embObjMotionControl, i.e. if this joint is actually controlled by
	// the EMS I'm referring to.
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jconfig__pidposition);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	if(NULL == nvRoot)
		yError() << "[embObj Motion Control] Get nv pointer failed at line " << __LINE__;

	eOmc_PID_t	outPid;
	copyPid_iCub2eo(&pid, &outPid);
	if( eores_OK != eo_nv_Set(nvRoot, &outPid, eobool_true, eo_nv_upd_dontdo))
	{
		yError() << "[embObj Motion Control] Set nv value failed at line " << __LINE__;
		return false;
	}
	res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid);

	yDebug() << "Set pid joint " << j << "Kp " << pid.kp << " ki " << outPid.ki << " kd " << pid.kd;

	// Now set the velocity pid too...
	nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jconfig__pidvelocity);
	nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	if(NULL == nvRoot)
		yError() << "[embObj Motion Control] Get nv pointer failed at line " << __LINE__;

	if( eores_OK != eo_nv_Set(nvRoot, &outPid, eobool_true, eo_nv_upd_dontdo))
	{
		yError() << "[embObj Motion Control] Set nv value failed at line " << __LINE__;
		return false;
	}
	res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid);


	return true;
}

bool embObjMotionControl::setPidsRaw(const Pid *pids)
{
	// print_debug(AC_trace_file, "embObjMotionControl::setPidRaw()");

	bool ret = true;
	for(int j=_firstJoint; j<_firstJoint+_njoints;j++)
	{
		ret &= setPidRaw(j, pids[j]);
	}
	return ret;
}

bool embObjMotionControl::setReferenceRaw(int j, double ref)
{
	// print_debug(AC_trace_file, "embObjMotionControl::setReferenceRaw()");
	return NOT_YET_IMPLEMENTED("setReferenceRaw");
}

bool embObjMotionControl::setReferencesRaw(const double *refs)
{
	// print_debug(AC_trace_file, "embObjMotionControl::setReferencesRaw()");
	return NOT_YET_IMPLEMENTED("setReferencesRaw");
}

bool embObjMotionControl::setErrorLimitRaw(int j, double limit)
{
	// print_debug(AC_trace_file, "embObjMotionControl::setErrorLimitRaw()");
	return NOT_YET_IMPLEMENTED("setErrorLimitRaw");
}

bool embObjMotionControl::setErrorLimitsRaw(const double *limits)
{
	// print_debug(AC_trace_file, "embObjMotionControl::setErrorLimitsRaw()");
	return NOT_YET_IMPLEMENTED("setErrorLimitsRaw");
}

bool embObjMotionControl::getErrorRaw(int j, double *err)
{
	// print_debug(AC_trace_file, "embObjMotionControl::getErrorRaw()");

	// Check if j is valid for this specific instance of embObjMotionControl, i.e. if this joint is actually controlled by
	// the EMS I'm referring to.

	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jstatus__ofpid);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	if(NULL == nvRoot)
		NV_NOT_FOUND;


	eOmc_joint_status_ofpid_t  tmpJointStatus;
	uint16_t size;
	res->transceiver->getNVvalue(nvRoot, (uint8_t*) &tmpJointStatus, &size);
	*err = (double) tmpJointStatus.error;
	return true;
}

bool embObjMotionControl::getErrorsRaw(double *errs)
{
	// print_debug(AC_trace_file, "embObjMotionControl::getErrorRaw()");

	bool ret = true;
	for(int j=_firstJoint; j<_firstJoint+_njoints;j++)
	{
		ret &= getErrorRaw(j, &errs[j]);
	}
	return ret;
}

bool embObjMotionControl::getOutputRaw(int j, double *out)
{
	// print_debug(AC_trace_file, "embObjMotionControl::getOutputRaw()");

	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jstatus__ofpid);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	if(NULL == nvRoot)
		NV_NOT_FOUND

	eOmc_joint_status_ofpid_t  tmpJointStatus;
	uint16_t size;
	res->transceiver->getNVvalue(nvRoot, (uint8_t*) &tmpJointStatus, &size);
	*out = (double) tmpJointStatus.output;

	return true;
}

bool embObjMotionControl::getOutputsRaw(double *outs)
{
	// print_debug(AC_trace_file, "embObjMotionControl::getOutputsRaw()");

	bool ret = true;
	for(int j=_firstJoint; j<_firstJoint+_njoints;j++)
	{
		ret &= getOutputRaw(j, &outs[j]);
	}
	return ret;
}

bool embObjMotionControl::getPidRaw(int j, Pid *pid)
{
	yTrace();

	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jconfig__pidposition);
	EOnv	*nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid);

	if(NULL == nvRoot)
		NV_NOT_FOUND

	res->transceiver->load_occasional_rop(eo_ropcode_ask, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid);

	// Sign up for waiting the reply
	eoThreadEntry *tt = appendWaitRequest(j, nvid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
	tt->setPending(1);

	// wait here
	if(-1 == tt->synch() )
	{
		int threadId;
		yError() << "getPid timed out, joint " << j;

		if(requestQueue->threadPool->getId(&threadId))
			requestQueue->cleanTimeouts(threadId);
		return false;
	}
	// Get the value
	uint16_t size;
	eOmc_PID_t eoPID;
	res->transceiver->getNVvalue(nvRoot, (uint8_t *)&eoPID, &size);
    yDebug() << " GetPid returned values : kp = " << eoPID.kp << "kd = " <<  eoPID.kd << " ki = " << eoPID.ki;
    copyPid_eo2iCub(&eoPID, pid);

	return true;
}

bool embObjMotionControl::getPidsRaw(Pid *pids)
{
	yTrace();

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
			NV_NOT_FOUND

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
			yError() << "getPids timed out";
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
	// print_debug(AC_trace_file, "embObjMotionControl::getReferenceRaw()");
	return NOT_YET_IMPLEMENTED("getReference");
}

bool embObjMotionControl::getReferencesRaw(double *refs)
{
	// print_debug(AC_trace_file, "embObjMotionControl::getReferencesRaw()");
	return NOT_YET_IMPLEMENTED("getReference");
}

bool embObjMotionControl::getErrorLimitRaw(int j, double *limit)
{
	// print_debug(AC_trace_file, "embObjMotionControl::getErrorLimitRaw()");
    return NOT_YET_IMPLEMENTED("getErrorLimit");
}

bool embObjMotionControl::getErrorLimitsRaw(double *limits)
{
	// print_debug(AC_trace_file, "embObjMotionControl::getErrorLimitsRaw()");
    return NOT_YET_IMPLEMENTED("getErrorLimit");
}

bool embObjMotionControl::resetPidRaw(int j)
{
	yTrace();
	// print_debug(AC_trace_file, "embObjMotionControl::resetPidRaw()");

    return NOT_YET_IMPLEMENTED("resetPid");
}

bool embObjMotionControl::disablePidRaw(int j)
{
	yTrace();
	// Spegni tutto!! Setta anche Amp off
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jcmmnds__controlmode);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	_enabledAmp[j-_firstJoint] = false;
	_enabledPid[j-_firstJoint] = false;

	yDebug() << "disablePidRaw AMP status " << _enabledAmp[j-_firstJoint];

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	eOmc_controlmode_t val = eomc_controlmode_switch_everything_off;
	if( eores_OK != eo_nv_Set(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid);

	return true;
}

bool embObjMotionControl::enablePidRaw(int j)
{
	yTrace();

	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jcmmnds__controlmode);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	//eOmc_controlmode_t val = *(eOmc_controlmode_t *) nvRoot->loc;

	if(false == _enabledAmp[j-_firstJoint] )
	{
		yWarning() << "Error: Enable Amp before enabling Pid!!  (embObjCtrl)\n";
		return false;
	}

	// se giunto non è calibrato non fa nulla, se è calibrato manda il control mode position
	_enabledPid[j-_firstJoint] = true;

	if(_calibrated[j-_firstJoint])
	{
		eOmc_controlmode_t val = eomc_controlmode_position;

		if( eores_OK != eo_nv_Set(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
		{
			yError() <<  "\n>>> ERROR eo_nv_Set !!\n";
			return false;
		}

		res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid);
	}

	return true;
}

bool embObjMotionControl::setOffsetRaw(int j, double v)
{
	yTrace();
    return NOT_YET_IMPLEMENTED("setOffset");
}

////////////////////////////////////////
//    Velocity control interface raw  //
////////////////////////////////////////

bool embObjMotionControl::setVelocityModeRaw()
{
	yTrace();

	bool ret = true;
	eOnvID_t  nvid[_njoints];
	EOnv	  *nvRoot[_njoints];

	eOmc_controlmode_t val = eomc_controlmode_velocity;
	for(int j=_firstJoint, index=0; j<_firstJoint+_njoints; j++, index++)
	{
		nvid[index]   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jcmmnds__controlmode);
		nvRoot[index] = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid[index]);

		if(NULL == nvRoot[index])
		{
			NV_NOT_FOUND;
			ret = false;
		}

		if( eores_OK != eo_nv_Set(nvRoot[index], &val, eobool_true, eo_nv_upd_dontdo))
		{
			// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
			return false;
		}

		res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid[index]);
	}

	return ret;
}

bool embObjMotionControl::velocityMoveRaw(int j, double sp)
{
	yTrace();

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
		// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid);

	return true;
}

bool embObjMotionControl::velocityMoveRaw(const double *sp)
{
	yTrace();

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
			// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
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
	yTrace();

	// Tenere il check o forzare questi sottostati?
	if(!_enabledAmp[j-_firstJoint] )
	{
		yError() << "PWM not enabled";
		return false;
	}

	if(!_enabledPid[j-_firstJoint])
	{
		yError() << "PID not enabled";
		return false;
	}

	// Get controlmode NV pointer
	eOnvID_t  nvid_controlmode;
	EOnv	  *nvRoot_controlmode;
	eOmc_controlmode_t val = eomc_controlmode_margin_reached;


	nvid_controlmode   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jcmmnds__controlmode);
	nvRoot_controlmode = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid_controlmode);

	if(NULL == nvRoot_controlmode)
	{
		NV_NOT_FOUND;

		return false;
	}

	if( eores_OK != eo_nv_Set(nvRoot_controlmode, &val, eobool_true, eo_nv_upd_dontdo))
	{
		yError() << "eo nv not set!\n";
		return false;
	}


	// Get calibration command NV pointer
	eOnvID_t nvid_cmd_calib = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jcmmnds__calibration);
	EOnv *nvRoot_cmd_calib = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid_cmd_calib);

	if(NULL == nvRoot_cmd_calib)
	{
		NV_NOT_FOUND;
		return false;
	}

	eOmc_calibrator_t calib;

	switch(type)
	{
		// muove -> amp+pid, poi calib
		case eomc_calibration_type0_hard_stops:
			calib.type = eomc_calibration_type0_hard_stops;
			calib.params.type0.pwmlimit = p1;
			calib.params.type0.velocity = p2;
//			eOmc_controlmode_t val = eomc_controlmode_margin_reached;
			if( eores_OK != eo_nv_Set(nvRoot_cmd_calib, &calib, eobool_true, eo_nv_upd_dontdo))
			{
				yError() << "eo nv not set!\n";
				return false;
			}
			res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid_controlmode);
			res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid_cmd_calib);
			break;

		// fermo
		case eomc_calibration_type1_abs_sens_analog:
			calib.type = eomc_calibration_type1_abs_sens_analog;
			calib.params.type1.position = p1;
			calib.params.type1.velocity = p2;
//			eOmc_controlmode_t val = eomc_controlmode_margin_reached;
			if( eores_OK != eo_nv_Set(nvRoot_cmd_calib, &calib, eobool_true, eo_nv_upd_dontdo))
			{
				yError() << "eo nv not set!\n";
				return false;
			}
			res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid_cmd_calib);
			res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid_controlmode);
			break;

		// muove
		case eomc_calibration_type2_hard_stops_diff:
			calib.type = eomc_calibration_type2_hard_stops_diff;
			calib.params.type2.pwmlimit = p1;
			calib.params.type2.velocity = p2;
//			eOmc_controlmode_t val = eomc_controlmode_margin_reached;
			if( eores_OK != eo_nv_Set(nvRoot_cmd_calib, &calib, eobool_true, eo_nv_upd_dontdo))
			{
				yError() << "eo nv not set!\n";
				return false;
			}
			res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid_controlmode);
			res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid_cmd_calib);
			break;

		// muove
		case eomc_calibration_type3_abs_sens_digital:
			calib.type = eomc_calibration_type3_abs_sens_digital;
			calib.params.type3.position = p1;
			calib.params.type3.velocity = p2;
			calib.params.type3.offset   = p3;
//			eOmc_controlmode_t val = eomc_controlmode_margin_reached;
			if( eores_OK != eo_nv_Set(nvRoot_cmd_calib, &calib, eobool_true, eo_nv_upd_dontdo))
			{
				yError() << "eo nv not set!\n";
				return false;
			}
			res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid_cmd_calib);
			res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid_controlmode);
			break;

		// muove
		case eomc_calibration_type4_abs_and_incremental:
			calib.type = eomc_calibration_type4_abs_and_incremental;
			calib.params.type4.position   = p1;
			calib.params.type3.velocity   = p2;
			calib.params.type4.maxencoder = p3;
//			eOmc_controlmode_t val = eomc_controlmode_margin_reached;
			if( eores_OK != eo_nv_Set(nvRoot_cmd_calib, &calib, eobool_true, eo_nv_upd_dontdo))
			{
				yError() << "eo nv not set!\n";
				return false;
			}
			res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid_cmd_calib);
			res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid_controlmode);
			break;

		default:
			yError() << "Calibration type unknown!! (embObjMotionControl)\n";
			return false;
			break;
	}

	_calibrated[j-_firstJoint] = true;

	return true;
}

bool embObjMotionControl::doneRaw(int axis)
{
	// used only in calibration procedure, for normal work use the checkMotionDone
	yTrace();

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
		yError() << "[ETH] Calibration done timed out, joint " << axis;

		if(requestQueue->threadPool->getId(&threadId))
			requestQueue->cleanTimeouts(threadId);
		return false;
	}

	uint16_t size;
	eOmc_controlmode_t type;
	res->transceiver->getNVvalue(nvRoot, (uint8_t*) &type, &size);


	// if the control mode is no longer a calibration type, it means calibration ended

	if( ( eomc_controlmode_calib_abs_pos_sens == type) || ( eomc_controlmode_calib_hard_stops == type) ||	( eomc_controlmode_handle_hard_stops == type) || ( eomc_controlmode_margin_reached == type) || ( eomc_controlmode_calib_abs_and_inc) )
		return false;
	else
		return true;
}

////////////////////////////////////////
//     Position control interface     //
////////////////////////////////////////

bool embObjMotionControl::getAxes(int *ax)
{
	yTrace();
	*ax=_njoints;

	return true;
}

bool embObjMotionControl::setPositionModeRaw()
{
	yTrace();

	bool ret = true;
	eOnvID_t  nvid[_njoints];
	EOnv	  *nvRoot[_njoints];

	eOmc_controlmode_t val = eomc_controlmode_position;
	for(int j=_firstJoint, index=0; j<_firstJoint+_njoints; j++, index++)
	{
		nvid[index]   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jcmmnds__controlmode);
		nvRoot[index] = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid[index]);

		if(NULL == nvRoot[index])
		{
			NV_NOT_FOUND;
			ret = false;
		}

		if( eores_OK != eo_nv_Set(nvRoot[index], &val, eobool_true, eo_nv_upd_dontdo))
		{
			// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
			return false;
		}

		res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid[index]);
	}

	return ret;
}

bool embObjMotionControl::positionMoveRaw(int j, double ref)
{
	yTrace();

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
		// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
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
			// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
			return false;
		}
		res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid_config);
	}


	// Read the current value - it is signalled spontaneously every cicle, so we don't have to wait here
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
//				// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
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
	yTrace();
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
		// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
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
	uint32_t 	val = eomc_controlmode_position;

	nvid   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jcmmnds__controlmode);
	nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	if( eores_OK != eo_nv_Set(nvRoot, (eOmc_controlmode_t*)&val, eobool_true, eo_nv_upd_always))
	{
		printf("\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid);

	return true;
}

bool embObjMotionControl::setVelocityModeRaw(int j)
{
	eOnvID_t  			nvid;
	EOnv	  			*nvRoot;
	uint32_t 	val = eomc_controlmode_velocity;

	nvid   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jcmmnds__controlmode);
	nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	if( eores_OK != eo_nv_Set(nvRoot, (eOmc_controlmode_t*)&val, eobool_true, eo_nv_upd_always))
	{
		// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
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

	nvid   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jcmmnds__controlmode);
	nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	if( eores_OK != eo_nv_Set(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
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

	nvid   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jcmmnds__controlmode);
	nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	if( eores_OK != eo_nv_Set(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
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

	nvid   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jcmmnds__controlmode);
	nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	if( eores_OK != eo_nv_Set(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
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

	nvid   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jcmmnds__controlmode);
	nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	if( eores_OK != eo_nv_Set(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid);

	return true;
}

bool embObjMotionControl::getControlModeRaw(int j, int *v)
{
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jstatus__basic);
	EOnv	*nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

//	res->transceiver->load_occasional_rop(eo_ropcode_ask, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid);
//
//	// Sign up for waiting the reply
//	eoThreadEntry *tt = appendWaitRequest(j, nvid);
//	tt->setPending(1);
//
//	// wait here
//	if(-1 == tt->synch() )
//	{
//		int threadId;
//		yError() << "[ETH] getControlModeRaw timed out, joint " << j;
//		if(requestQueue->threadPool->getId(&threadId))
//			requestQueue->cleanTimeouts(threadId);
//		return false;
//	}

	uint16_t size;
	eOmc_joint_status_basic_t	status;
	eOmc_controlmode_t type;
	res->transceiver->getNVvalue(nvRoot, (uint8_t*) &status, &size);

	//type = (eOmc_joint_status_basic_t) status.controlmodestatus;

	uint8_t val = status.controlmodestatus;
//	printf("\nCurrent status for joint %d is ",j);
	switch(val)
	{
		case eomc_controlmode_idle:
	        *v=VOCAB_CM_IDLE;
//	        printf("IDLE\n");
			break;

		case eomc_controlmode_position:
	        *v=VOCAB_CM_POSITION;
//	        printf("POSITION\n");
			break;

		case eomc_controlmode_velocity:
	        *v=VOCAB_CM_VELOCITY;
//	        printf("VELOCITY\n");
			break;

		case eomc_controlmode_torque:
	        *v=VOCAB_CM_TORQUE;
//	        printf("TORQUE\n");
			break;

		case eomc_controlmode_calib_abs_pos_sens:
		case eomc_controlmode_calib_hard_stops:
		case eomc_controlmode_margin_reached:
		case eomc_controlmode_calib_abs_and_inc:
		case eomc_controlmode_handle_hard_stops:
			*v=VOCAB_CM_UNKNOWN;
//			printf("CALIBRATING\n");
			break;

		case eomc_controlmode_impedance_pos:
	        *v=VOCAB_CM_IMPEDANCE_POS;
//	        printf("IMPEDANCE_POS\n");
			break;

		case eomc_controlmode_impedance_vel:
	        *v=VOCAB_CM_IMPEDANCE_VEL;
//	        printf("IMPEDANCE_VEL\n");
			break;

		case eomc_controlmode_openloop:
	        *v=VOCAB_CM_OPENLOOP;
//	        printf("VOCAB_CM_OPENLOOP\n");
			break;

		case 0xf0:
	        *v=VOCAB_CM_IDLE;
//	        printf("SWITCH EVERYTHING OFF\n");
			break;

		default:
	        *v=VOCAB_CM_UNKNOWN;
//	        printf("UNKNOWN (0x%04X)\n", status.controlmodestatus);
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
	yTrace();

	// Just take note of this command. Does nothing here... wait for enable pid
	_enabledAmp[j-_firstJoint] = true;
	yDebug() << "enableAmpRaw AMP status " << _enabledAmp[j-_firstJoint];
	return true;
}

bool embObjMotionControl::disableAmpRaw(int j)
{
	yTrace();
	// Spegni tutto!! Setta anche pid off
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jcmmnds__controlmode);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid);

	_enabledAmp[j-_firstJoint] = false;
	_enabledPid[j-_firstJoint] = false;

	yDebug() << "disableAmpRaw AMP status " << _enabledAmp[j-_firstJoint];

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	eOmc_controlmode_t val = eomc_controlmode_switch_everything_off;
	if( eores_OK != eo_nv_Set(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
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
		// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid);

	return true;
}

bool embObjMotionControl::getAmpStatusRaw(int j, int *st)
{
	yTrace();
	(_enabledAmp[j-_firstJoint]) ? *st = 1 : *st = 0;
	return true;
}

bool embObjMotionControl::getAmpStatusRaw(int *sts)
{
	yTrace();
	bool ret = true;
	for(int j=0; j<_njoints;j++)
	{
		sts[j] = _enabledAmp[j];
	}

	return ret;
}
