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

#include "embObjMotionControl.h"

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
    fprintf(stderr, "%s not yet implemented for embObjMotionControl\n", txt);

    return false;
}

#define NV_NOT_FOUND	return nv_not_found();

bool nv_not_found(void)
{
	yError () << " nv_not_found!! This may mean that this variable is not handled by this EMS\n";
	return false;
}

void embObjMotionControl::getMStatus(int j)
{
	EOnv *nvRoot;
	EOnv nvtmp;
	eOnvID_t nvid;

	nvid = eo_cfg_nvsEP_mc_motor_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, j, motorNVindex_mconfig);
	res->transceiver->load_occasional_rop(eo_ropcode_ask, (uint16_t)_fId.ep, nvid);
}

//generic function that check is key1 is present in input bottle and that the result has size elements
// return true/false
bool embObjMotionControl::extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
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


bool embObjMotionControl::alloc(int nj)
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

embObjMotionControl::embObjMotionControl() :
						ImplementControlCalibration2<embObjMotionControl, IControlCalibration2>(this),
						ImplementAmplifierControl<embObjMotionControl, IAmplifierControl>(this),
						ImplementPidControl<embObjMotionControl, IPidControl>(this),
						ImplementEncodersTimed(this),
						ImplementPositionControl<embObjMotionControl, IPositionControl>(this),
				        ImplementVelocityControl<embObjMotionControl, IVelocityControl>(this),
				        ImplementControlMode(this),
#ifdef IMPLEMENT_DEBUG_INTERFACE
				        ImplementDebugInterface(this),
#endif
				        ImplementControlLimits<embObjMotionControl, IControlLimits>(this),
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

	 std::string str=config.toString().c_str();
	 Property prop;
	 prop.fromString(str.c_str());

	// get EMS ip address from config string
	ACE_TCHAR address[64] = {0};
	Bottle xtmp = Bottle(config.findGroup("ETH"));
	strcpy(address, config.findGroup("ETH").check("IpAddress",Value(1), " EMS ip address").asString().c_str() );

//	 Debug info
	memset(info, 0x00, SIZE_INFO);
	sprintf(info, "embObjMotionControl - referred to EMS: %s\n", address);

	yDebug() << "Opened " << info;

//	 open ethResource, if needed

	ethResCreator *resList = ethResCreator::instance();
	if(NULL == (res = resList->getResource(config)) )
	{
		yError () << "[embObjMotionContro] Unable to instantiate an EMS... check configuration file";
		return false;
	}

	// Defining Unique Id
	//_fId.type = MotionControl;
	std::string featId = "ciao"; //config.find("FeatId").asString().c_str();
	cout << "FeatId = " << featId << endl;
	strcpy(_fId.name, featId.c_str());


	_fId.boardNum  = 255;
	Value val =config.findGroup("ETH").check("Ems",Value(1), "Board number");
	if(val.isInt())
		_fId.boardNum =val.asInt();
	else
		printf("No board number found!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

	// yDebug() << "boardNum " << _fId.boardNum;

	Bottle &xtmpB = (config.findGroup("ETH"));
	int boardNum=255;
	Bottle &xtmp2 = xtmpB.findGroup("Ems");
    if (xtmp2.isNull())
    {
        yError () << "[embObjMotionContro] EMS Board number identifier not found\n";
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
			yError () << "\n eoMotion Control: Wrong board identifier number!!!";
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
#ifdef IMPLEMENT_DEBUG_INTERFACE
    ImplementDebugInterface::initialize(_njoints, _axisMap, _angleToEncoder, _zeros, _rotToEncoder);
#endif
    ImplementControlLimits<embObjMotionControl, IControlLimits>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);

    //  Tell EMS which NV I want to be signaled spontaneously, configure joints and motors and go to running mode

    init();

    // yTrace();
    // E meglio dopo il go to running perche' la mais inizia a sparare subito i dati
    // e rischia di riempire e incasinare le code di ricezione del can?
    configure_mais();

    goToRun();
    return true;
}


bool embObjMotionControl::fromConfig(yarp::os::Searchable &config)
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

bool embObjMotionControl::init()
{
	// yTrace();
	eOmn_ropsigcfg_command_t 	*ropsigcfgassign;
	EOarray						*array;
	eOropSIGcfg_t 				sigcfg;
	int 						old = 0;

	eOnvID_t nvid, nvid_ropsigcfgassign = eo_cfg_nvsEP_mn_comm_NVID_Get(endpoint_mn_comm, 0, commNVindex__ropsigcfgcommand);
	EOnv *nvRoot_ropsigcfgassign;
	EOnv nv_ropsigcfgassign;
	nvRoot_ropsigcfgassign = res->transceiver->getNVhandler(endpoint_mn_comm, nvid_ropsigcfgassign, &nv_ropsigcfgassign);


	ropsigcfgassign = (eOmn_ropsigcfg_command_t*) nvRoot_ropsigcfgassign->loc;
	array = (EOarray*) &ropsigcfgassign->array;
	eo_array_Reset(array);
	array->head.capacity = NUMOFROPSIGCFG;
	array->head.itemsize = sizeof(eOropSIGcfg_t);
	ropsigcfgassign->cmmnd = ropsigcfg_cmd_append;

//	printf("\ropSigCfgcommand nvid = %d (0x%04X)\n", nvid_ropsigcfgassign, nvid_ropsigcfgassign);

	int jStatusSize = sizeof(eOmc_joint_status_t);
	int mStatusSize = sizeof(eOmc_motor_status_basic_t);
	int totSigSize	= 0;

	for(int j=_firstJoint; j<_firstJoint+_njoints; j++)
	{
		 yDebug() << "configuring ropSig for joint " << j;

		// Verify that the EMS is able to handle all those data. The macro EOK_HOSTTRANSCEIVER_capacityofropframeregulars has to be the one used by the firmware!!!!
		if( ! (EMS_capacityofropframeregulars >= (totSigSize += jStatusSize)) )
		{
			yError () << "No space left on EMS device for setting new regular messages!! Skipping remaining" << _fId.name;
			break;
		}

		// basterebbero jstatus__basic e jstatus__ofpid, ma la differenza tra questi due e il jstatus completo sono 4 byte, per ora non utilizzati.
		nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, j, jointNVindex_jstatus);
//		printf("\njointNVindex_jstatus nvid = %d (0x%04X)", nvid, nvid);
		if(EOK_uint16dummy == nvid)
		{
			yError () << " NVID jointNVindex_jstatus not found for EndPoint" << _fId.ep << " joint " << j;
		}
		else
		{
			sigcfg.ep = _fId.ep;
			sigcfg.id = nvid;
			sigcfg.plustime = 0;
			if(eores_OK != eo_array_PushBack(array, &sigcfg))
				yError () << " while loading ropSig Array for joint " << j << " at line " << __LINE__;
		}

		// Verify that the EMS is able to handle all those data. The macro EOK_HOSTTRANSCEIVER_capacityofropframeregulars has to be the one used by the firmware!!!!
		if( ! (EMS_capacityofropframeregulars >= (totSigSize += mStatusSize)) )
		{
			yError () << "No space left on EMS device for setting new regular messages!! Skipping remaining on board" << _fId.name;
			break;
		}

		nvid = eo_cfg_nvsEP_mc_motor_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, j, motorNVindex_mstatus__basic);
//		printf("\nmotorNVindex_jstatus nvid = %d (0x%04X)", nvid, nvid);
		if(EOK_uint16dummy == nvid)
		{
			yError () << " NVID jointNVindex_jstatus not found for EndPoint" << _fId.ep << " joint " << j;
		}
		else
		{
			sigcfg.ep = _fId.ep;
			sigcfg.id = nvid;
			sigcfg.plustime = 0;
			if(eores_OK != eo_array_PushBack(array, &sigcfg))
				yError () << " while loading ropSig Array for joint " << j << " at line " << __LINE__;
		}

		if( (NUMOFROPSIGCFG - 1) <= ((j - old +1)*2))	// a ropSigCfg can store only 20 variables at time. Send 2 messages if more are needed.
		{
			// A ropsigcfg vector can hold at max NUMOFROPSIGCFG (20) value. If more are needed, send another package,
			// so wait some time to let ethManager send this package and then start again.
			// yDebug() << "Maximun number of variables reached in the ropSigCfg array, splitting it in two pieces";
			if( !res->transceiver->nvSetData(nvRoot_ropsigcfgassign, array, eobool_true, eo_nv_upd_dontdo))
			{
				yError () << "ERROR eo_nv_Set !!";
				return false;
			}

			if(!res->transceiver->load_occasional_rop(eo_ropcode_set, endpoint_mn_comm, nvid_ropsigcfgassign))
				return false;

			Time::delay(0.1);				// Wait here, the ethManager thread will take care of sending the loaded message
			printf("\n-----------------");
			eo_array_Reset(array);
			array->head.capacity = NUMOFROPSIGCFG;
			array->head.itemsize = sizeof(eOropSIGcfg_t);
			ropsigcfgassign->cmmnd = ropsigcfg_cmd_append;
			old = j;
		}
	}
	// Send remaining stuff
	if( !res->transceiver->nvSetData(nvRoot_ropsigcfgassign, array, eobool_true, eo_nv_upd_dontdo))
	{
		yError () << "ERROR eo_nv_Set !!";
		return false;
	}

	if( !res->transceiver->load_occasional_rop(eo_ropcode_set, endpoint_mn_comm, nvid_ropsigcfgassign))
	{
		yError () << "error load occasional rop, sig cfg\n";
		return false;
	}
	else
	{
		printf("rop sig cfg load ok\n");
	}
	// yTrace() << __LINE__;
	Time::delay(0.1);


	//////////////////////////////////////////
	// invia la configurazione dei GIUNTI   //
	//////////////////////////////////////////

	int jConfigSize 	= sizeof(eOmc_joint_config_t);
	int mConfigSize 	= sizeof(eOmc_motor_config_t);
	int totConfigSize	= 0;
	int index 			= 0;

	EOnv *nvRoot;
	EOnv nvtmp;

	// yTrace();
	 yDebug() << "Sending joint configuration";
	if( EOK_HOSTTRANSCEIVER_capacityofrop < jConfigSize )
	{
		yError () << "Size of Joint Config is bigger than single ROP... cannot send it at all!! Fix it!!";
	}
	else
	{
		for(int j=_firstJoint, index =0; j<_firstJoint + _njoints; j++, index++)
		{
			 yDebug() << " j = " << j << "index = " << index;
			printf("sending j config j = %d", j);

			if( ! (EOK_HOSTTRANSCEIVER_capacityofropframeoccasionals >= (totConfigSize += jConfigSize)) )
			{
				// yDebug() << "Too many stuff to be sent at once... splitting in more messages";
				Time::delay(0.05);
				totConfigSize = 0;
			}

			nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, j, jointNVindex_jconfig);
			if(EOK_uint16dummy == nvid)
			{
				yError () << " NVID not found\n";
				continue;
			}
			nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid, &nvtmp);

			if(NULL == nvRoot)
			{
				yError () << " NV pointer not found\n" << _fId.name << "board number " <<_fId.boardNum << "joint " << j << "at line" << __LINE__;
				continue;
			}


			eOmc_joint_config_t	jconfig;
			memset(&jconfig, 0x00, sizeof(eOmc_joint_config_t));
			copyPid_iCub2eo(&_pids[index],  &jconfig.pidposition);
			copyPid_iCub2eo(&_pids[index],  &jconfig.pidvelocity);
			copyPid_iCub2eo(&_tpids[index], &jconfig.pidtorque);

			// to do
//			memset(&jconfig.impedance, 0x00, sizeof(eOmc_impedance_t));

			jconfig.maxpositionofjoint = (eOmeas_position_t) convertA2I(_limitsMax[index], _zeros[index], _angleToEncoder[index]);
			jconfig.minpositionofjoint = (eOmeas_position_t) convertA2I(_limitsMin[index], _zeros[index], _angleToEncoder[index]);
			jconfig.velocitysetpointtimeout = (eOmeas_time_t)_velocityTimeout[index];
			jconfig.motionmonitormode = eomc_motionmonitormode_dontmonitor;
			// to do

			yDebug() << "\n>>>>Setting max and min pos for joint " << j;
			yDebug() << "From config file" ;
			yDebug() << "deg min = " << _limitsMin[index];
			yDebug() << "deg max = " << _limitsMax[index];
			yDebug() << "jConfig" ;
			yDebug() << "eomin = " << jconfig.minpositionofjoint;
			yDebug() << "eomax = " << jconfig.maxpositionofjoint;
			yDebug() << "zero deg = " <<_zeros[index];
			yDebug() << "_angleToEncoder = " <<_angleToEncoder[index];


			jconfig.encoderconversionfactor = eo_common_float_to_Q17_14(_encoderconversionfactor[index]);
			jconfig.encoderconversionoffset = eo_common_float_to_Q17_14(_encoderconversionoffset[index]);

			if( !res->transceiver->nvSetData(nvRoot, &jconfig, eobool_true, eo_nv_upd_dontdo))
				continue;

			if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid))
				 continue;

			// Debugging... to much information can overload can queue on EMS
			Time::delay(0.05);
		}
	}
	Time::delay(0.1);


	//////////////////////////////////////////
	// invia la configurazione dei MOTORI   //
	//////////////////////////////////////////

	yDebug() << "Sending motor MAX CURRENT ONLY";
	totConfigSize = 0;
	if( EOK_HOSTTRANSCEIVER_capacityofrop < mConfigSize )
	{
		yError () << "Size of Motor Config is bigger than single ROP... cannot send it at all!! Fix it";
	}
	else
	{
		for(int j=_firstJoint, index =0; j<_firstJoint+_njoints;j++, index++)
		{
			// yDebug() << " j = " << j << "index = " << index;

			if( ! (EOK_HOSTTRANSCEIVER_capacityofropframeoccasionals >= (totConfigSize += mConfigSize)) )
			{
				// yDebug() << "Too many stuff to be sent at once... splitting in more messages";
				Time::delay(0.1);
				totConfigSize = 0;
			}

//			nvid = eo_cfg_nvsEP_mc_motor_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, j, motorNVindex_mconfig);
			nvid = eo_cfg_nvsEP_mc_motor_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, j, motorNVindex_mconfig__maxcurrentofmotor);
			if(EOK_uint16dummy == nvid)
			{
				yError () << " NVID not found\n";
				continue;
			}
			nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid, &nvtmp);

			if(NULL == nvRoot)
				yError () << " NV pointer found\n";;


			eOmc_motor_config_t	mconfig;
			memset(&mconfig, 0x00, sizeof(eOmc_motor_config_t));
			// what to do here?
			//mconfig.pidcurrent = unknown;
			//mconfig.maxvelocityofmotor =  ????;
			mconfig.maxcurrentofmotor = _currentLimits[index];

			eOmeas_current_t	current = (eOmeas_current_t) _currentLimits[index];

			yDebug() << "setting maxcurrent of motor " << index << " to " << _currentLimits[index] << "(" << current << ")";
//			if( !res->transceiver->nvSetData(nvRoot, &mconfig, eobool_true, eo_nv_upd_dontdo))
			if( !res->transceiver->nvSetData(nvRoot, &current, eobool_true, eo_nv_upd_dontdo))
			{
				yError () << "ERROR eo_nv_Set !!";
				continue;
			}
			if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid))
			{
				yError () << "ERROR eo_nv_Set !!";
				continue;
			}
			// Debugging... to much information can overload can queue on EMS
			Time::delay(0.1);

			getMStatus(j);
			Time::delay(0.1);
		}
	}
	Time::delay(0.1);

	return true;
}


bool embObjMotionControl::configure_mais(void)
{
	// invia configurazioni a caso
	 yTrace();
	eOnvID_t nvid;
	EOnv *nvRoot;
	EOnv tmp;

	// Mais per lettura posizioni dita, c'e' solo sulle mani per ora
	eOcfg_nvsEP_as_endpoint_t mais_ep = (eOcfg_nvsEP_as_endpoint_t)0;

	if(_fId.ep == endpoint_mc_leftlowerarm )
			mais_ep = endpoint_as_leftlowerarm;

	if(_fId.ep == endpoint_mc_rightlowerarm )
			mais_ep = endpoint_as_rightlowerarm;


	if( mais_ep )
	{
		uint8_t						maisnum = 0;
		uint8_t                     datarate = 10; //10 milli (like in icub_right_arm_safe.ini)  // type ok
		eOsnsr_maismode_t			maismode = snsr_maismode_txdatacontinuously;

		//set mais datarate = 1millisec
		nvid = eo_cfg_nvsEP_as_mais_NVID_Get(mais_ep, maisnum, maisNVindex_mconfig__datarate);
		if(EOK_uint16dummy == nvid)
		{
			yError () << "[eomc] NVID not found( maisNVindex_mconfig__datarate, " << _fId.name << "board number " << _fId.boardNum << "at line" << __LINE__ << ")";
			return false;
		}
		nvRoot = res->transceiver->getNVhandler(mais_ep, nvid, &tmp);
		if(NULL == nvRoot)
			yError () << "[eomc] NV pointer not found\n" << _fId.name << "board number " << _fId.boardNum << "at line" << __LINE__;

		if( !res->transceiver->nvSetData(nvRoot, &datarate, eobool_true, eo_nv_upd_dontdo))
			return false;

		if(!res->transceiver->load_occasional_rop(eo_ropcode_set, mais_ep, nvid) )
			return false;

		//set tx mode continuosly
		nvid = eo_cfg_nvsEP_as_mais_NVID_Get(mais_ep, maisnum, maisNVindex_mconfig__mode);
		if(EOK_uint16dummy == nvid)
		{
			yError () << "[eomc] NVID not found( maisNVindex_mconfig__mode, " << _fId.name << "board number " << _fId.boardNum << "at line" << __LINE__ << ")";
			return false;
		}
		nvRoot = res->transceiver->getNVhandler(mais_ep, nvid, &tmp);
		if(NULL == nvRoot)
			yError () << "[eomc] NV pointer not found\n" << _fId.name << "board number " << _fId.boardNum << "at line" << __LINE__;

		if( !res->transceiver->nvSetData(nvRoot, &maismode, eobool_true, eo_nv_upd_dontdo))
			return false;

		if(!res->transceiver->load_occasional_rop(eo_ropcode_set, mais_ep, nvid) )
			return false;
	}

	Time::delay(0.1);
}

bool embObjMotionControl::goToRun(void)
{
	// yTrace();
	eOnvID_t nvid;
	EOnv tmp;

	// attiva il loop di controllo
	eOcfg_nvsEP_mn_applNumber_t dummy = 0;  // not used but there for API compatibility
	nvid = eo_cfg_nvsEP_mn_appl_NVID_Get(endpoint_mn_appl, dummy, applNVindex_cmmnds__go2state);

	EOnv 	*nvRoot 	= res->transceiver->getNVhandler(endpoint_mn_appl, nvid, &tmp);
	if(NULL == nvRoot)
	{
		yError () << "NV pointer not found at line" << __LINE__;
		return false;
	}

	eOmn_appl_state_t  desired 	= applstate_running;

	if( !res->transceiver->nvSetData(nvRoot, &desired, eobool_true, eo_nv_upd_dontdo))
		return false;

	// tell agent to prepare a rop to send
	if(!res->transceiver->load_occasional_rop(eo_ropcode_set, endpoint_mn_appl, nvid) )
		return false;
	}

bool embObjMotionControl::close()
{
	// yTrace();
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
	// yTrace();
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
	// yTrace();
	EOnv tmp;
	// Check if j is valid for this specific instance of embObjMotionControl, i.e. if this joint is actually controlled by
	// the EMS I'm referring to.
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jconfig__pidposition);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid, &tmp);

	if(NULL == nvRoot)
		yError () << "[embObj Motion Control] Get nv pointer failed at line " << __LINE__;

	eOmc_PID_t	outPid;
	copyPid_iCub2eo(&pid, &outPid);
	if( !res->transceiver->nvSetData(nvRoot, &outPid, eobool_true, eo_nv_upd_dontdo))
	//if( !res->transceiver->nvSetData(nvRoot, &outPid, eobool_true, eo_nv_upd_dontdo))
	{
		yError () << "[embObj Motion Control] Set nv value failed at line " << __LINE__;
		return false;
	}
	if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid) )
		return false;

	// yDebug() << "Set pid joint " << j << "Kp " << pid.kp << " ki " << outPid.ki << " kd " << pid.kd;

	// Now set the velocity pid too...
	nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jconfig__pidvelocity);
	nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid, &tmp);

	if(NULL == nvRoot)
		yError () << "[embObj Motion Control] Get nv pointer failed at line " << __LINE__;

	if( !res->transceiver->nvSetData(nvRoot, &outPid, eobool_true, eo_nv_upd_dontdo))
	{
		yError () << "[embObj Motion Control] Set nv value failed at line " << __LINE__;
		return false;
	}
	if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid) )
		return false;


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
	EOnv tmp;

	// Check if j is valid for this specific instance of embObjMotionControl, i.e. if this joint is actually controlled by
	// the EMS I'm referring to.

	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jstatus__ofpid);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid, &tmp);

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
	bool ret = true;
	for(int j=_firstJoint; j<_firstJoint+_njoints;j++)
	{
		ret &= getErrorRaw(j, &errs[j]);
	}
	return ret;
}

bool embObjMotionControl::getOutputRaw(int j, double *out)
{
	EOnv tmp;
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jstatus__ofpid);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid, &tmp);

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
	bool ret = true;

	for(int j=_firstJoint; j<_firstJoint+_njoints;j++)
	{
		ret &= getOutputRaw(j, &outs[j]);
	}
	return ret;
}

bool embObjMotionControl::getPidRaw(int j, Pid *pid)
{
	// yTrace();
	EOnv tmp;
	//_mutex.wait();
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jconfig__pidposition);
	EOnv	*nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid, &tmp);

	if(NULL == nvRoot)
		NV_NOT_FOUND

	if(!res->transceiver->load_occasional_rop(eo_ropcode_ask, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid) )
		return  false;

	// Sign up for waiting the reply
	eoThreadEntry *tt = appendWaitRequest(j, nvid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
	tt->setPending(1);

	// wait here
	if(-1 == tt->synch() )
	{
		int threadId;
		yError () << "getPid timed out, joint " << j;

		if(requestQueue->threadPool->getId(&threadId))
			requestQueue->cleanTimeouts(threadId);
		return false;
	}

	// Get the value
	uint16_t size;
	eOmc_PID_t eoPID;
	res->transceiver->getNVvalue(nvRoot, (uint8_t *)&eoPID, &size);
    // yDebug() << " GetPid returned values : kp = " << eoPID.kp << "kd = " <<  eoPID.kd << " ki = " << eoPID.ki;
    copyPid_eo2iCub(&eoPID, pid);
	//_mutex.post();
	return true;
}

bool embObjMotionControl::getPidsRaw(Pid *pids)
{
	// yTrace();

	bool ret = true;
	eoThreadEntry *tt = NULL;

//	EOnv tmp[_njoints];
	eOnvID_t  nvid[_njoints];
	EOnv	  nv[_njoints];
	eoRequest req;

	for(int j=_firstJoint, index=0; j<_firstJoint+_njoints; j++, index++)
	{
		nvid[index]   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jconfig__pidposition);
		if(NULL == res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid[index], &nv[index]) )
			NV_NOT_FOUND;

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
			yError () << "getPids timed out";
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
		eOmc_PID_t *tmpPid = (eOmc_PID16_t *) &nv[index];  //tmpPid doesn't need to be an array, because it's just a cast of an existing data.
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
	// yTrace();
	// print_debug(AC_trace_file, "embObjMotionControl::resetPidRaw()");

    return NOT_YET_IMPLEMENTED("resetPid");
}

bool embObjMotionControl::disablePidRaw(int j)
{
	// yTrace();
	EOnv tmp;
	// Spegni tutto!! Setta anche Amp off
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jcmmnds__controlmode);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid, &tmp);

	_enabledAmp[j-_firstJoint] = false;
	_enabledPid[j-_firstJoint] = false;

	// yDebug() << "disablePidRaw AMP status " << _enabledAmp[j-_firstJoint];
//	printf("nvid of disablePid 0x%04X\n", nvid);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	eOmc_controlmode_command_t val = eomc_controlmode_cmd_switch_everything_off;
	if( !res->transceiver->nvSetData(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid) )
		return false;

	return true;
}

bool embObjMotionControl::enablePidRaw(int j)
{
	// yTrace();
	EOnv tmp;
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jcmmnds__controlmode);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid, &tmp);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	if(false == _enabledAmp[j-_firstJoint] )
	{
		yWarning() << "Enable Amp before enabling Pid!!  (embObjCtrl)\n";
//		return false;
	}

	// se giunto non è calibrato non fa nulla, se è calibrato manda il control mode position
	_enabledPid[j-_firstJoint] = true;
	printf("nvid of enablePid 0x%04X\n", nvid);

	if(_calibrated[j-_firstJoint])
	{
		eOmc_controlmode_command_t val = eomc_controlmode_cmd_position;

		if( !res->transceiver->nvSetData(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
		{
			yError () <<  "\n>>> ERROR eo_nv_Set !!\n";
			return false;
		}

		if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid) )
			return false;
	}
	return true;
}

bool embObjMotionControl::setOffsetRaw(int j, double v)
{
	// yTrace();
    return NOT_YET_IMPLEMENTED("setOffset");
}

////////////////////////////////////////
//    Velocity control interface raw  //
////////////////////////////////////////

bool embObjMotionControl::setVelocityModeRaw()
{
	// yTrace();
	EOnv tmp;
	bool ret = true;
	eOnvID_t  nvid[_njoints];
	EOnv	  *nvRoot[_njoints];

	eOmc_controlmode_command_t val = eomc_controlmode_cmd_velocity;
	for(int j=_firstJoint, index=0; j<_firstJoint+_njoints; j++, index++)
	{
		nvid[index]   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jcmmnds__controlmode);
		nvRoot[index] = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid[index], &tmp);

		if(NULL == nvRoot[index])
		{
			NV_NOT_FOUND;
			ret = false;
		}

		if( !res->transceiver->nvSetData(nvRoot[index], &val, eobool_true, eo_nv_upd_dontdo))
		{
			// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
			return false;
		}

		if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid[index]) )
			return false;
	}

	return ret;
}

bool embObjMotionControl::velocityMoveRaw(int j, double sp)
{
	// yTrace();
	EOnv tmp;
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jcmmnds__setpoint);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid, &tmp);
	int index = j- _firstJoint;

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

    _command_speeds[j] = sp ;   // save internally the new value of speed.

	eOmc_setpoint_t setpoint;
	setpoint.type = eomc_setpoint_velocity;
	setpoint.to.velocity.value =  (eOmeas_velocity_t) _command_speeds[index];
	setpoint.to.velocity.withacceleration = (eOmeas_acceleration_t) _ref_accs[index];

	if( !res->transceiver->nvSetData(nvRoot, &setpoint, eobool_true, eo_nv_upd_dontdo))
	{
		// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid) )
		return false;

	return true;
}

bool embObjMotionControl::velocityMoveRaw(const double *sp)
{
	// yTrace();
	EOnv tmp;
	bool ret = true;
	eOnvID_t  nvid[_njoints];
	EOnv	  *nvRoot[_njoints];
	eOmc_setpoint_t setpoint;

	setpoint.type = eomc_setpoint_velocity;

	for(int j=_firstJoint, index=0; j<_firstJoint+_njoints; j++, index++)
	{
		nvid[index]   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jcmmnds__setpoint);
		nvRoot[index] = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid[index], &tmp);

		if(NULL == nvRoot[index])
		{
			NV_NOT_FOUND;
			ret = false;
		}

	    _command_speeds[index] = sp[index] / 1000.0;   // save internally the new value of speed.
		setpoint.to.velocity.value =  (eOmeas_velocity_t) _command_speeds[index];
		setpoint.to.velocity.withacceleration = (eOmeas_acceleration_t) _ref_accs[index];

		if( !res->transceiver->nvSetData(nvRoot[index], &setpoint, eobool_true, eo_nv_upd_dontdo))
		{
			// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
			return false;
		}

		if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid[index]) )
			return false;
	}

	return ret;
}


////////////////////////////////////////
//    Calibration control interface   //
////////////////////////////////////////

bool embObjMotionControl::calibrate2Raw(int j, unsigned int type, double p1, double p2, double p3)
{
	// yTrace();
	EOnv tmp_controlmode;
	// Tenere il check o forzare questi sottostati?
	if(!_enabledAmp[j-_firstJoint] )
	{
		yDebug () << "Called calibrate for joint " << j << "with PWM(AMP) not enabled";
//		return false;
	}

	if(!_enabledPid[j-_firstJoint])
	{
		yDebug () << "Called calibrate for joint " << j << "with PID not enabled";
//		return false;
	}

//   There is no explicit command "go to calibration mode" but it is implicit in the calibration command

	// Get calibration command NV pointer
	EOnv tmp_calib;
	eOnvID_t nvid_cmd_calib = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jcmmnds__calibration);
	EOnv *nvRoot_cmd_calib = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid_cmd_calib, &tmp_calib);

	if(NULL == nvRoot_cmd_calib)
	{
		NV_NOT_FOUND;
		return false;
	}

	eOmc_calibrator_t calib;
	memset(&calib, 0x00, sizeof(calib));
	calib.type = type;

	switch(type)
	{
		// muove -> amp+pid, poi calib
		case eomc_calibration_type0_hard_stops:
			calib.params.type0.pwmlimit = (int16_t) p1;
			calib.params.type0.velocity = (eOmeas_velocity_t) p2;
			break;

		// fermo
		case eomc_calibration_type1_abs_sens_analog:
			calib.params.type1.position = (int16_t) p1;
			calib.params.type1.velocity = (eOmeas_velocity_t) p2;
			break;

		// muove
		case eomc_calibration_type2_hard_stops_diff:
			calib.params.type2.pwmlimit = (int16_t) p1;
			calib.params.type2.velocity = (eOmeas_velocity_t) p2;
			break;

		// muove
		case eomc_calibration_type3_abs_sens_digital:
			calib.params.type3.position = (int16_t) p1;
			calib.params.type3.velocity = (eOmeas_velocity_t) p2;
			calib.params.type3.offset   = (int32_t) p3;
			break;

		// muove
		case eomc_calibration_type4_abs_and_incremental:
			calib.params.type4.position   = (int16_t) p1;
			calib.params.type3.velocity   = (eOmeas_velocity_t) p2;
			calib.params.type4.maxencoder = (int32_t) p3;
			break;

		default:
			yError () << "Calibration type unknown!! (embObjMotionControl)\n";
			return false;
			break;		//useless
	}

	if( !res->transceiver->nvSetData(nvRoot_cmd_calib, &calib, eobool_true, eo_nv_upd_dontdo))
		return false;

	if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid_cmd_calib) )
		return false;

	_calibrated[j-_firstJoint] = true;

	return true;
}

bool embObjMotionControl::doneRaw(int axis)
{
	// used only in calibration procedure, for normal work use the checkMotionDone
	// yTrace();
	EOnv tmp;
	//
	// get the control mode and check its value??? // e segnalato spontaneamente!! perche' fare una richiesta????
	//
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)axis, jointNVindex_jstatus__basic);
	EOnv	*nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid, &tmp);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

//	res->transceiver->load_occasional_rop(eo_ropcode_ask, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid);
//
//	// Sign up for waiting the reply
//	eoThreadEntry *tt = appendWaitRequest(axis, nvid);
//	tt->setPending(1);
//
//	// wait here
//	if(-1 == tt->synch() )
//	{
//		int threadId;
//		yError () << "[ETH] Calibration done timed out, joint " << axis;
//
//		if(requestQueue->threadPool->getId(&threadId))
//			requestQueue->cleanTimeouts(threadId);
//		return false;
//	}

	uint16_t size;
	eOmc_controlmode_t type;
	res->transceiver->getNVvalue(nvRoot, (uint8_t*) &type, &size);


	// if the control mode is no longer a calibration type, it means calibration ended
	if( eomc_controlmode_calib == type)
		return false;
	else
		return true;
}

////////////////////////////////////////
//     Position control interface     //
////////////////////////////////////////

bool embObjMotionControl::getAxes(int *ax)
{
	// yTrace();
	*ax=_njoints;

	return true;
}

bool embObjMotionControl::setPositionModeRaw()
{
	// yTrace();

	bool ret = true;
	eOnvID_t  nvid[_njoints];
	EOnv	  nv[_njoints];

	eOmc_controlmode_command_t val = eomc_controlmode_cmd_position;
	for(int j=_firstJoint, index=0; j<_firstJoint+_njoints; j++, index++)
	{
		nvid[index]   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jcmmnds__controlmode);
		if(NULL == res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid[index], &nv[index]) )
		{
			NV_NOT_FOUND;
			ret = false;
		}

		if( !res->transceiver->nvSetData(&nv[index], &val, eobool_true, eo_nv_upd_dontdo))
			return false;

		if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid[index]))
			return false;
	}

	return ret;
}

bool embObjMotionControl::positionMoveRaw(int j, double ref)
{
	// yTrace();
	EOnv tmp;
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jcmmnds__setpoint);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid, &tmp);
	int index = j-_firstJoint;

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	_ref_positions[index] = ref;   // save internally the new value of speed.

	eOmc_setpoint_t setpoint;
	setpoint.type = (eOenum08_t) eomc_setpoint_position;
	setpoint.to.position.value =  (eOmeas_position_t) _ref_positions[index];
	setpoint.to.position.withvelocity = (eOmeas_velocity_t) _ref_speeds[index];

	if( !res->transceiver->nvSetData(nvRoot, &setpoint, eobool_true, eo_nv_upd_dontdo))
	{
		// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid) )
		return false;

	yDebug() << "Position move EP" << _fId.ep << "j" << j << setpoint.to.position.value << "\tspeed " << setpoint.to.position.withvelocity  << " at time: " << (Time::now()/1e6) << "\n";
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
	// yTrace();
	EOnv tmpnv_config;
	eOnvID_t nvid_config = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jconfig__motionmonitormode);
	EOnv *nvRoot_config = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid_config, &tmpnv_config);

//	printf("nvid of check motion done 0x%04X\n", nvid_config);
	if(NULL == nvRoot_config)
	{
		NV_NOT_FOUND;
		return false;
	}

	// monitor status until set point is reached, if it wasn't already set
	// this is because the function has to be in a non blocking fashion and I want to avoid resending the same message over and over again!!

	if(!checking_motiondone[j-_firstJoint])
	{
		checking_motiondone[j-_firstJoint] = true;
		eOmc_motionmonitormode_t tmp = eomc_motionmonitormode_forever;
		if( !res->transceiver->nvSetData(nvRoot_config, &tmp, eobool_true, eo_nv_upd_dontdo))
		{
			// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
			return false;
		}
		if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid_config) )
			return false;
	}


	// Read the current value - it is signalled spontaneously every cicle, so we don't have to wait here
	EOnv tmpnv_status;
	eOnvID_t nvid_status = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jstatus__basic);
	EOnv *nvRoot_status = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid_status, &tmpnv_status);

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
//			if( !res->transceiver->nvSetData(nvRoot_config, &val, eobool_true, eo_nv_upd_dontdo))
//			{
//				// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
//				return false;
//			}
//			res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid_config);
//		}
//		checking_motiondone[j-_firstJoint]= false;
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
	// yTrace();
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
	EOnv tmpnv;
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jcmmnds__stoptrajectory);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid, &tmpnv);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	eObool_t stop = eobool_true;

	if( !res->transceiver->nvSetData(nvRoot, &stop, eobool_true, eo_nv_upd_dontdo))
	{
		// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid) )
		return false;

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
	EOnv 						tmpnv;
	eOnvID_t  					nvid;
	EOnv	  		   			*nvRoot;
	eOmc_controlmode_command_t 	val = eomc_controlmode_cmd_position;

	nvid   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jcmmnds__controlmode);
	nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid, &tmpnv);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	if( !res->transceiver->nvSetData(nvRoot, &val, eobool_true, eo_nv_upd_always))
	{
		printf("\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid) )
		return false;

	return true;
}

bool embObjMotionControl::setVelocityModeRaw(int j)
{
	eOnvID_t  						nvid;
	EOnv 							tmpnv;
	EOnv	  						*nvRoot;
	eOmc_controlmode_command_t 		val = eomc_controlmode_cmd_velocity;

	nvid   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jcmmnds__controlmode);
	nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid, &tmpnv);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	if( !res->transceiver->nvSetData(nvRoot, &val, eobool_true, eo_nv_upd_always))
	{
		yError () << "nv Set";
		return false;
	}

	if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid) )
		return false;

	return true;
}

bool embObjMotionControl::setTorqueModeRaw(int j)
{
	eOnvID_t  						nvid;
	EOnv 							tmpnv;
	EOnv	  						*nvRoot;
	eOmc_controlmode_command_t 		val =  eomc_controlmode_cmd_torque;

	nvid   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jcmmnds__controlmode);
	nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid, &tmpnv);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	if( !res->transceiver->nvSetData(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid) )
		return false;

	return true;
}

bool embObjMotionControl::setImpedancePositionModeRaw(int j)
{
	eOnvID_t  						nvid;
	EOnv 							tmpnv;
	EOnv	  						*nvRoot;
	eOmc_controlmode_command_t 		val = eomc_controlmode_cmd_impedance_pos;

	nvid   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jcmmnds__controlmode);
	nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid, &tmpnv);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	if( !res->transceiver->nvSetData(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid) )
		return false;

	return true;
}

bool embObjMotionControl::setImpedanceVelocityModeRaw(int j)
{
	eOnvID_t  						nvid;
	EOnv 							tmpnv;
	EOnv	  						*nvRoot;
	eOmc_controlmode_command_t 		val = eomc_controlmode_cmd_impedance_vel;

	nvid   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jcmmnds__controlmode);
	nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid, &tmpnv);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	if( !res->transceiver->nvSetData(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
		return false;

	if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid))
		return false;

	return true;
}

bool embObjMotionControl::setOpenLoopModeRaw(int j)
{
	eOnvID_t  						nvid;
	EOnv 							tmpnv;
	EOnv	  						*nvRoot;
	eOmc_controlmode_command_t 		val = eomc_controlmode_cmd_openloop;

	nvid   = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jcmmnds__controlmode);
	nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid, &tmpnv);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	if( !res->transceiver->nvSetData(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
		return false;

	if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid) )
		return false;

	return true;
}

bool embObjMotionControl::getControlModeRaw(int j, int *v)
{
	EOnv tmpnv;
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t)j, jointNVindex_jstatus__basic);
	EOnv	*nvRoot = res->transceiver->getNVhandler( (eOcfg_nvsEP_mc_endpoint_t)_fId.ep,  nvid, &tmpnv);

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
//		yError () << "[ETH] getControlModeRaw timed out, joint " << j;
//		if(requestQueue->threadPool->getId(&threadId))
//			requestQueue->cleanTimeouts(threadId);
//		return false;
//	}

	uint16_t size;
	eOmc_joint_status_basic_t	status;
	eOmc_controlmode_t  type;
	res->transceiver->getNVvalue(nvRoot, (uint8_t*) &status, &size);

	//type = (eOmc_joint_status_basic_t) status.controlmodestatus;

	type = (eOmc_controlmode_t) status.controlmodestatus;
//	printf("\nCurrent status for joint %d is ",j);
	switch(type)
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

		case eomc_controlmode_calib:
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
//	printf("EO getEncoderRaw\n");

	EOnv tmpnv;
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jstatus__basic);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid, &tmpnv);

	if(NULL == nvRoot)
	{
		yError () << "Nv pointer not found\n";
		return false;
	}

	eOmc_joint_status_basic_t  tmpJointStatus;
	uint16_t size;
	res->transceiver->getNVvalue(nvRoot, (uint8_t*) &tmpJointStatus, &size);
	*value = (double) tmpJointStatus.position;

// DEBUG
//	static int i = 0;
//	static bool print = true;
//
//#define PRINT_EVERY_X 500
//
//	if(_fId.ep == endpoint_mc_leftlowerarm)
//	{
//		if( (print) && (i>= PRINT_EVERY_X) && (j == 0) )
//		{
//			i = 0;
//			print = false;
//		}
//
//		if(j == 0)
//			i++;
//
//		if(i >= PRINT_EVERY_X)
//		{
//			print = true;
//		}
//
//		if(print)
//			printf("Pos joint %d = %d\n", j, tmpJointStatus.position);
//	}
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
	EOnv tmpnv;
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jstatus__basic);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid, &tmpnv);

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
	EOnv tmpnv;
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jstatus__basic);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid, &tmpnv);

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

bool embObjMotionControl::getEncodersTimedRaw(double *encs, double *stamps)
{
	getEncodersRaw(encs);
}

bool embObjMotionControl::getEncoderTimedRaw(int j, double *encs, double *stamp)
{
	getEncoderRaw(j, encs);

//	//_mutex.wait();
//	double stamp=0;
//	for (i = 0; i < r.getJoints(); i++) {
//		v[i] = double(r._bcastRecvBuffer[i]._position_joint._value);
//		t[i] = r._bcastRecvBuffer[i]._position_joint._stamp;
//
//		if (stamp<r._bcastRecvBuffer[i]._position_joint._stamp)
//			stamp=r._bcastRecvBuffer[i]._position_joint._stamp;
//	}
//
//	stampEncoders.update(stamp);
//	//_mutex.post();
}

////// Amplifier interface
//
bool embObjMotionControl::enableAmpRaw(int j)
{
	// yTrace();

	// Just take note of this command. Does nothing here... wait for enable pid
	_enabledAmp[j-_firstJoint] = true;
	// yDebug() << "enableAmpRaw AMP status " << _enabledAmp[j-_firstJoint];
	return true;
}

bool embObjMotionControl::disableAmpRaw(int j)
{
	// yTrace();
	// Spegni tutto!! Setta anche pid off
	EOnv tmpnv;
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jcmmnds__controlmode);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid, &tmpnv);

	_enabledAmp[j-_firstJoint] = false;
	_enabledPid[j-_firstJoint] = false;

	// yDebug() << "disableAmpRaw AMP status " << _enabledAmp[j-_firstJoint];

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	eOmc_controlmode_command_t val = eomc_controlmode_cmd_switch_everything_off;
	if( !res->transceiver->nvSetData(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid) )
		return false;

	return true;
}

bool embObjMotionControl::getCurrentRaw(int j, double *value)
{
	EOnv tmpnv;
	eOnvID_t nvid = eo_cfg_nvsEP_mc_motor_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_motorNumber_t) j, motorNVindex_mstatus__basic);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid, &tmpnv);

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
	EOnv tmpnv;
	eOnvID_t nvid = eo_cfg_nvsEP_mc_motor_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_motorNumber_t) j, motorNVindex_mconfig__maxcurrentofmotor);
	EOnv *nvRoot = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid, &tmpnv);

	if(NULL == nvRoot)
	{
		NV_NOT_FOUND;
		return false;
	}

	eOmeas_current_t  maxCurrent = (eOmeas_current_t) val;

	if( !res->transceiver->nvSetData(nvRoot, &val, eobool_true, eo_nv_upd_dontdo))
	{
		// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
		return false;
	}

	if(!res->transceiver->load_occasional_rop(eo_ropcode_set, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid) )
		return false;

	return true;
}

bool embObjMotionControl::getAmpStatusRaw(int j, int *st)
{
	// yTrace();
	(_enabledAmp[j-_firstJoint]) ? *st = 1 : *st = 0;
	return true;
}

bool embObjMotionControl::getAmpStatusRaw(int *sts)
{
	// yTrace();
	bool ret = true;
	for(int j=0; j<_njoints;j++)
	{
		sts[j] = _enabledAmp[j];
	}

	return ret;
}

#ifdef IMPLEMENT_DEBUG_INTERFACE
//----------------------------------------------\\
//	Debug interface
//----------------------------------------------\\

bool embObjMotionControl::setParameterRaw(int j, unsigned int type, double value) 			{ }
bool embObjMotionControl::getParameterRaw(int j, unsigned int type, double* value)			{ }
bool embObjMotionControl::getDebugParameterRaw(int j, unsigned int index, double* value) 	{ }
bool embObjMotionControl::setDebugParameterRaw(int j, unsigned int index, double value) 	{ }
bool embObjMotionControl::setDebugReferencePositionRaw(int j, double value) 				{ }
bool embObjMotionControl::getDebugReferencePositionRaw(int j, double* value) 				{ }
bool embObjMotionControl::getRotorPositionRaw         (int j, double* value) 				{ }
bool embObjMotionControl::getRotorPositionsRaw        (double* value)		 				{ }
bool embObjMotionControl::getRotorSpeedRaw            (int j, double* value)		 		{ }
bool embObjMotionControl::getRotorSpeedsRaw           (double* value) 						{ }
bool embObjMotionControl::getRotorAccelerationRaw     (int j, double* value)		 		{ }
bool embObjMotionControl::getRotorAccelerationsRaw    (double* value) 						{ }
bool embObjMotionControl::getJointPositionRaw         (int j, double* value) 				{ }
bool embObjMotionControl::getJointPositionsRaw        (double* value) 						{ }
#endif

// Limit interface
bool embObjMotionControl::setLimitsRaw(int j, double min, double max)
{

}

bool embObjMotionControl::getLimitsRaw(int j, double *min, double *max)
{
	EOnv NV_min, NV_max;
	eOnvID_t nvid_min, nvid_max;
	EOnv *nvRoot_min, *nvRoot_max;

	nvid_min = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jconfig__minpositionofjoint);
	nvid_max = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) j, jointNVindex_jconfig__maxpositionofjoint);

	nvRoot_min = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid_min, &NV_min);
	nvRoot_max = res->transceiver->getNVhandler((uint16_t)_fId.ep, nvid_max, &NV_max);

	if( (NULL == nvRoot_min) || (NULL == nvRoot_max))
	{
		yError() << "nv root not found at line " << __LINE__;
		return false;
	}

	// Sign up for waiting the reply
	eoThreadEntry *tt = appendWaitRequest(j, nvid_min);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
	appendWaitRequest(j, nvid_max);
	tt->setPending(2);

	if(!res->transceiver->load_occasional_rop(eo_ropcode_ask, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid_min) )
		return false;

	if(!res->transceiver->load_occasional_rop(eo_ropcode_ask, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, nvid_max) )
		return false;


	// wait here
	if(-1 == tt->synch() )
	{
		int threadId;
		printf("\n\n--------------------\nTIMEOUT for joint %d\n-----------------------\n", j); //yError () << "ask request timed out, joint " << j;

		if(requestQueue->threadPool->getId(&threadId))
			requestQueue->cleanTimeouts(threadId);
		//return false;
	}
	// Get the value
	uint16_t size;

	int32_t	eomin, eomax;
	res->transceiver->getNVvalue(nvRoot_min, (uint8_t *)&eomin, &size);
	res->transceiver->getNVvalue(nvRoot_max, (uint8_t *)&eomax, &size);
	*min = (double)eomin;
	*max = (double)eomax;
	printf("\n j %d\n",j);
	printf("min = %f\n", *min);
	printf("max = %f\n", *max);
	printf("size = %d\n", size);
	printf("eomin = %d\n", eomin);
	printf("eomax = %d\n", eomax);
    // yDebug() << " GetPid returned values : kp = " << eoPID.kp << "kd = " <<  eoPID.kd << " ki = " << eoPID.ki;

    return true;
}


