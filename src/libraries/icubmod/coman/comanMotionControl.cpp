// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iostream>
#include <cstdlib>

#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <string.h>
#include <ode/odeconfig.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include "comanDevicesHandler.hpp"
#include "comanMotionControl.h"
#include "Debug.h"

//#undef yDebug()
//#define yDebug() cout
//
//#undef yWarning()
//#define yWarning() cout
//
//#undef yError()
//#define yError() cout

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;

static const int VEL2POS_TIMEOUT = 50;
static const int VEL2POS_THRESHOLD = 0.01;
static const int VEL2POS_DELAY = 0.1;

// Utilities
/*
Per comandi TCP usare:      setItem(SET_TORQUE_FACTORS, &torque_factor, sizeof(torque_factor);
Per comandi UDP usare:      UDPCommPacket pkt(SET_DESIRED_POS_VEL);
                            pkt.appendData((char*)des_pos, N'di item nel vettore);   // si puó anche fare un ciclo for di 1 elemento alla volta se serve.
                            pkt.sendToUDPSocket(udp_sock, (sockaddr *)&dest_addr, sizeof(dest_addr));

Prendere il puntatore alla classe corrispondente al giunto
    McBoard *joint_p = getMCpointer(j);   //  -> giusto
Prendere l'indice corretto dell'array per il giunto j
    idx = bId2Idx(j);
*/


// This will be moved in the ImplXXXInterface
// void comanMotionControl::convertDoble2Int(double in[], int out[])
// {
//     for(int idx=0; idx<_njoints; idx++)
//         out[idx] = (int) in[idx];
// }

// void comanMotionControl::convertDoble2Short(double in[], short int out[])
// {
//     for(int idx=0; idx<_njoints; idx++)
//         out[idx] = (short int) in[idx];
// }

inline McBoard * comanMotionControl::getMCpointer(int j)
{
    return _mcs[jointTobId(j)];
}

inline int comanMotionControl::bId2Joint(int j)
{
    return _inv_bIdMap[j];
}

inline uint8_t comanMotionControl::jointTobId(int j)
{
    return (uint8_t) _bIdMap[j];
}

static inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    fprintf(stderr, "%s not yet implemented for comanMotionControl\n", txt);
    return false;
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
        yError () << key1.c_str() << " incorrect number of entries";
        return false;
    }

    out=tmp;
    return true;
}


bool comanMotionControl::alloc(int nj)
{
    _axisMap = allocAndCheck<int>(nj);
    _bIdMap = allocAndCheck<int>(nj);
//     _inv_bIdMap = allocAndCheck<int>(nj);      // this will be allocated after!!

    _angleToEncoder = allocAndCheck<double>(nj);
    _rotToEncoder = allocAndCheck<double>(nj);
    _zeros = allocAndCheck<double>(nj);
    _maxTorque=allocAndCheck<double>(nj);
    _newtonsToSensor=allocAndCheck<double>(nj);
    for(int j=0; j<nj; j++)
        _newtonsToSensor[j] = 1000.0;

    _posPids=allocAndCheck<Pid>(nj);
    _trqPids=allocAndCheck<Pid>(nj);
    _impPosPids=allocAndCheck<Pid>(nj);

    _limitsMax=allocAndCheck<double>(nj);
    _limitsMin=allocAndCheck<double>(nj);
    _currentLimits=allocAndCheck<double>(nj);

    _velocityShifts=allocAndCheck<int>(nj);
    _velocityTimeout=allocAndCheck<int>(nj);

    // Reserve space for data stored locally. values are initialize to 0
    _ref_positions = allocAndCheck<int32_t>(nj);
    _command_speeds = allocAndCheck<double>(nj);
    _ref_speeds = allocAndCheck<int16_t>(nj);
    _ref_accs = allocAndCheck<double>(nj);
    _ref_torques = allocAndCheck<int16_t>(nj);
    _pid_offset = allocAndCheck<int16_t>(nj);
    _enabledAmp = allocAndCheck<bool>(nj);
    _enabledPid = allocAndCheck<bool>(nj);
    _calibrated = allocAndCheck<bool>(nj);
    _controlMode = allocAndCheck<int>(nj);  // Cache the controlMode because boards doesn´t know

    // Store internal values of the boards so that we can change between different configuration
    motor_config_mask   = allocAndCheck<uint16_t>(nj);
    motor_config_mask2  = allocAndCheck<uint16_t>(nj);
    pid                 = allocAndCheck<Pid>(nj);
    pidTorque           = allocAndCheck<Pid>(nj);


    return true;
}

   comanMotionControl::comanMotionControl() :
    ImplementControlCalibration2<comanMotionControl, IControlCalibration2>(this),
    ImplementAmplifierControl<comanMotionControl, IAmplifierControl>(this),
    ImplementPidControl<comanMotionControl, IPidControl>(this),
    ImplementEncodersTimed(this),
    ImplementPositionControl2(this),
    ImplementVelocityControl2(this),
    ImplementControlMode(this),
    ImplementDebugInterface(this),
    ImplementControlLimits2(this),
    ImplementTorqueControl(this),
    ImplementPositionDirect(this),
    ImplementImpedanceControl(this),
     _initialPidConfigFound(false),
    _mutex(1)
{
    yTrace();
    _boards_ctrl = NULL;

    _posPids			= NULL;
    _trqPids			= NULL;
    _impPosPids         = NULL;

    _tpidsEnabled	= false;
    _njoints 		= 0;

    _axisMap		= NULL;
    _bIdMap     = NULL;
    _inv_bIdMap = NULL;

    _angleToEncoder = NULL;
    _zeros			= NULL;
    _limitsMin = NULL;
    _limitsMax = NULL;
    _currentLimits = NULL;
    _velocityShifts = NULL;
    _velocityTimeout = NULL;

    _maxTorque = NULL;
    _newtonsToSensor = NULL;
    _rotToEncoder	= NULL;
    _ref_accs		= NULL;
    _command_speeds = NULL;
    _ref_positions 	= NULL;
    _ref_speeds		= NULL;
    _ref_torques	= NULL;
    _pid_offset = NULL;
    // debug connection

    _controlMode = NULL;

    // Check status of joints
    _enabledPid		= NULL;
    _enabledAmp 	= NULL;
    _calibrated		= NULL;
    bc_policy     = 0;
    bc_rate     =   0;
}

comanMotionControl::~comanMotionControl()
{
    yTrace();

    checkAndDestroy<int>(_axisMap);
    checkAndDestroy<int>(_bIdMap);
    checkAndDestroy<int>(_inv_bIdMap);

    checkAndDestroy<double>(_angleToEncoder);
    checkAndDestroy<double>(_rotToEncoder);
    checkAndDestroy<double>(_zeros);
    checkAndDestroy<double>(_maxTorque);
    checkAndDestroy<double>(_newtonsToSensor);

    checkAndDestroy<Pid>(_posPids);
    checkAndDestroy<Pid>(_trqPids);
    checkAndDestroy<Pid>(_impPosPids);

    checkAndDestroy<double>(_limitsMax);
    checkAndDestroy<double>(_limitsMin);
    checkAndDestroy<double>(_currentLimits);

    checkAndDestroy<int>(_velocityShifts);
    checkAndDestroy<int>(_velocityTimeout);

    checkAndDestroy<int32_t>(_ref_positions);
    checkAndDestroy<double>(_command_speeds);
    checkAndDestroy<int16_t>(_ref_speeds);
    checkAndDestroy<double>(_ref_accs);
    checkAndDestroy<int16_t>(_ref_torques);
    checkAndDestroy<int16_t>(_pid_offset);

    checkAndDestroy<bool>(_enabledAmp);
    checkAndDestroy<bool>(_enabledPid);
    checkAndDestroy<bool>(_calibrated);

    checkAndDestroy<int>(_controlMode);
}


bool comanMotionControl::open(yarp::os::Searchable &config)
{
    yTrace();

    if(! fromConfig(config) )
    {
    	yError() << " Some vital parameters are missing, check log and correct conf file!";
    	return false;
    }


    // initialize control type to idle
    for(int i=0; i< _njoints; i++)
    {
        _controlMode[i] = VOCAB_CM_IDLE;
    }


//
//      INIT ALL INTERFACES
//
    ImplementControlCalibration2<comanMotionControl, IControlCalibration2>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementAmplifierControl<comanMotionControl, IAmplifierControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementEncodersTimed::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementPositionControl2::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementPidControl<comanMotionControl, IPidControl>:: initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementControlMode::initialize(_njoints, _axisMap);
    ImplementVelocityControl2::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementDebugInterface::initialize(_njoints, _axisMap, _angleToEncoder, _zeros, _rotToEncoder);
    ImplementControlLimits2::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementTorqueControl::initialize(_njoints, _axisMap, _angleToEncoder, _zeros, _newtonsToSensor);
    ImplementPositionDirect::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementImpedanceControl::initialize(_njoints, _axisMap, _angleToEncoder, _zeros, _newtonsToSensor);

    _comanHandler = comanDevicesHandler::instance();

    if(_comanHandler == NULL)
    {
        yError() << "unable to create a new Coman Handler class!";
        return false;
    }

    if(!_comanHandler->open(config) )
    {
        yError() << "unable to initialize Coman Devices Handler class... probably no boards were found. Check log.";
        return false;
    }
    _boards_ctrl = _comanHandler->getBoard_ctrl_p();

    if(_boards_ctrl == NULL)
    {
        yError() << "unable to create a new Boards_ctrl class!";
        return false;
    }

    // TODO check this!
#warning "<><> TODO: This is a copy of the mcs map. Verify that things will never change after this copy or use a pointer (better) <><>"
    _mcs = _boards_ctrl->get_mcs_map();

    // set initial velocity, torque ansd position, start controls
    init();

    return true;
}

bool comanMotionControl::fromConfig(yarp::os::Searchable &config)
{
    //yDebug() << config.toString().c_str();
    Bottle xtmp;
    int i;
    Bottle general = config.findGroup("GENERAL");

    Value &tmp= general.find("joints");
    if(tmp.isNull())
    {
    	yError() << "Missing Joints number!\n";
		return false;
    }
    _njoints = general.find("joints").asInt();
    alloc(_njoints);

    // leggere i valori da file, AxisMap is optional
    // This is a remapping for the user. It is optional because it is actually unuseful and can even add more confusion than other.
    if (extractGroup(general, xtmp, "axisMap", "a list of reordered indices for the axes", _njoints+1))
    {
    	for (i = 1; i < xtmp.size(); i++)
    		_axisMap[i-1] = xtmp.get(i).asInt();
    }
    else
    {
    	yWarning() << "No AxisMap map found, using default configuration: continuous from 0 to n";
    	for (i = 1; i < xtmp.size(); i++)
    	{
    		_axisMap[i-1] = i-1;
    	}
    }

    // leggere i valori da file, bIdMap is mandatory!!
    // this value maps the board number to the joint of the comanMotorControl device
    int maxJ = -1;
    if(!extractGroup(general, xtmp, "bIdMap", "a list of ordered bIds", _njoints+1))
    	return false;

    for (i = 1; i < xtmp.size(); i++)
	{
		_bIdMap[i-1] = xtmp.get(i).asInt();
		if(_bIdMap[i-1] > maxJ)
			maxJ = _bIdMap[i-1];
	}
    maxJ +=1;  // MaxJ has to be an addressable index, so size of vector must be maxJ+1

    _inv_bIdMap = allocAndCheck<int>(maxJ);

    for(int bId_idx=0; bId_idx<maxJ; bId_idx++)
    {
        // reset array to an invaslid value
        _inv_bIdMap[bId_idx] = -1;
    }

    for(int bId_idx=0; bId_idx<_njoints; bId_idx++)
    {
        _inv_bIdMap[_bIdMap[bId_idx]] = bId_idx;
    }

    // Encoder scales
    if (!extractGroup(general, xtmp, "encoder", "a list of scales for the encoders", _njoints+1))
        return false;
    else
        for (i = 1; i < xtmp.size(); i++)
            _angleToEncoder[i-1] = xtmp.get(i).asDouble();

    // Zero Values
    if (!extractGroup(general, xtmp, "zeros","a list of offsets for the zero point", _njoints+1))
        return false;
    else
        for (i = 1; i < xtmp.size(); i++)
            _zeros[i-1] = xtmp.get(i).asDouble();

    bc_policy = 0;
    extra_policy = 0;
    Bottle mc_board = config.findGroup("MC_BOARD");
    bc_policy = mc_board.find("policy").asInt();
    extra_policy = mc_board.find("extra_policy").asInt();
    bc_rate = mc_board.find("bc_rate").asInt();

    printf("bc policy    = %d (0x%0x)\n", bc_policy, bc_policy);
    printf("extra_policy = %d (0x%0x)\n", extra_policy, extra_policy);


    ////// POSITION PIDS  - optional
    Bottle posPidsGroup;
    posPidsGroup=config.findGroup("POS_PIDS", "Position Pid parameters");
    if (posPidsGroup.isNull()==false)
    {
       if (!parsePidsGroup(posPidsGroup, _posPids))
       {
           yError() << "POS_PIDS section: error detected in parameters syntax\n";
           return false;
       }
       else
       {
           yDebug() << "Position Pids successfully loaded\n";
           _initialPidConfigFound = true;
       }
    }
    else
    {
        yWarning() << "POS_PIDS group not found, using defaults from boards!!\n";
    }

    ////// Torque PIDS  - mandatory (?)
    Bottle trqPidGroup;
    trqPidGroup=config.findGroup("TRQ_PIDS", "Torque Pid parameters");
    if (trqPidGroup.isNull()==false)
    {
       if (!parsePidsGroup(trqPidGroup, _trqPids))
       {
           yError() << "TRQ_PIDS section: error detected in parameters syntax\n";
           return false;
       }
       else
       {
           yDebug() << "Torque Pids successfully loaded\n";
       }
    }
    else
    {
        yWarning() << "TRQ_PIDS group not found, using hard-coded values!!";
        for(int i=0; i<_njoints; i++)
        {
            // yarp store pids as double, use the setPidRaw do do the conversion.
            _trqPids[i].kp = 445.0;
            _trqPids[i].ki = 22.0;
            _trqPids[i].kd = 0.0;
        }
        std::cout  << "\tkp is " << _trqPids[0].kp << std::endl;
        std::cout  << "\tki is " << _trqPids[0].ki << std::endl;
        std::cout  << "\tkd is " << _trqPids[0].kd << "\n" << std::endl;
    }

    ////// Impedance Pos PIDS  - mandatory (?)
    Bottle impPosPidsGroup;
    impPosPidsGroup=config.findGroup("IMP_POS_PIDS", "Position Pid parameters for impedance control");
    if (impPosPidsGroup.isNull()==false)
    {
       if (!parsePidsGroup(impPosPidsGroup, _impPosPids))
       {
           yError() << "IMP_POS_PIDS section: error detected in parameters syntax\n";
           return false;
       }
       else
       {
           yDebug() << "Impedance Position Pids successfully loaded\n";
       }
    }
    else
    {
        yWarning() << "IMP_POS_PIDS group not found, using hard-coded values!!";
        for(int i=0; i<_njoints; i++)
        {
            // yarp store pids as double, use the setPidRaw do do the conversion.
            _impPosPids[i].kp = 40000.0;
            _impPosPids[i].ki = 0.0;
            _impPosPids[i].kd = 5000.0;
        }
        std::cout  << "\tkp is " << _impPosPids[0].kp << std::endl;
        std::cout  << "\tki is " << _impPosPids[0].ki << std::endl;
        std::cout  << "\tkd is " << _impPosPids[0].kd << "\n"  << std::endl;
    }
    return true;
}

bool comanMotionControl::parsePidsGroup(Bottle& pidsGroup, Pid myPid[])
{
    int j=0;
    Bottle xtmp;

    if (!extractGroup(pidsGroup, xtmp, "kp", "kp parameter", _njoints+1))  return false;
    else
    {
        for (j=0; j<_njoints; j++)
            myPid[j].kp = xtmp.get(j+1).asDouble();
    }

    if (!extractGroup(pidsGroup, xtmp, "kd", "kd parameter", _njoints+1))  return false;
    else
    {
        for (j=0; j<_njoints; j++)
            myPid[j].kd = xtmp.get(j+1).asDouble();
    }

    if (!extractGroup(pidsGroup, xtmp, "ki", "ki parameter", _njoints+1))  return false;
    else
    {
        for (j=0; j<_njoints; j++)
            myPid[j].ki = xtmp.get(j+1).asDouble();
    }

    if (!extractGroup(pidsGroup, xtmp, "maxInt", "maxInt parameter", _njoints+1))  return false;
    else
    {
        for (j=0; j<_njoints; j++)
            myPid[j].max_int = xtmp.get(j+1).asDouble();
    }

    if (!extractGroup(pidsGroup, xtmp, "maxPwm", "maxPwm parameter", _njoints+1))   return false;
    else
    {
        for (j=0; j<_njoints; j++)
            myPid[j].max_output = xtmp.get(j+1).asDouble();
    }

    if (!extractGroup(pidsGroup, xtmp, "shift", "scale parameter", _njoints+1))     return false;
    else
    {
        for (j=0; j<_njoints; j++)
            myPid[j].scale = xtmp.get(j+1).asDouble();
    }

    if (!extractGroup(pidsGroup, xtmp, "ko", "pid offset parameter", _njoints+1))   return false;
    else
    {
        for (j=0; j<_njoints; j++)
            myPid[j].offset = xtmp.get(j+1).asDouble();
    }

    if (!extractGroup(pidsGroup, xtmp, "stictionUp", "stictionUp parameter", _njoints+1))    return false;
    else
    {
        for (j=0; j<_njoints; j++)
            myPid[j].stiction_up_val = xtmp.get(j+1).asDouble();
    }

    if (!extractGroup(pidsGroup, xtmp, "stictionDwn", "stictionDwn parameter", _njoints+1))  return false;
    else
    {
        for (j=0; j<_njoints; j++)
            myPid[j].stiction_down_val = xtmp.get(j+1).asDouble();
    }

    return true;
}

bool comanMotionControl::init()
{
#warning "A firmware version check could be usefull here?"
// MUST be in this order: velocity, position and torque??
    _boards_ctrl->set_velocity(_ref_speeds, _njoints * sizeof(_ref_speeds[0]));
    _boards_ctrl->set_position(_ref_positions, _njoints * sizeof(_ref_positions[0]));
    _boards_ctrl->set_torque(_ref_torques, _njoints * sizeof(_ref_torques[0]));

    // test settings
//    _boards_ctrl->test();
    // ... WAIT  to let dsp thinking .... LEAVE HERE
//    sleep(1);

    // let the calibrator or user application start the controller through the setPositionMode command
    uint8_t stop = 0;
    _boards_ctrl->start_stop_control(stop);

    for(int i=0; i< _njoints; i++)
    {
        _controlMode[i] = VOCAB_CM_IDLE;
    }

    if(_initialPidConfigFound)
    {
        for(int i=0; i< _njoints; i++)
        {
            setPid(i, _posPids[i]);
            yarp::os::Time::delay(0.1);
        }
    }

    // Storing meaniningful parameter, in order to have a correct switch between different control modes
    bool ret = true;

    for(int i=0; i< _njoints; i++)
    {
        McBoard *joint_p = getMCpointer(i);
        if(joint_p != NULL)
        {
            ret = ret && (!joint_p->getItem(GET_MOTOR_CONFIG, NULL, 0, REPLY_MOTOR_CONFIG, &motor_config_mask[i], 2) );

            yarp::os::Time::delay(0.01);

            ret = ret && (!joint_p->getItem(GET_MOTOR_CONFIG2, NULL, 0, REPLY_MOTOR_CONFIG2, &motor_config_mask2[i], 2) );

            getPidRaw(1, &pid[i]);
            yarp::os::Time::delay(0.01);

            getTorquePidRaw(1, &pidTorque[i]);
            yarp::os::Time::delay(0.01);
        }
    }

    // set current position as target to keep the robot steady at the start-up; set initial ref speed
    double initialPosition[_njoints];
    yarp::sig::Vector initialSpeeds(_njoints, 10);

    ret = getEncoders(initialPosition);
    if(ret)
    {
        setRefSpeeds(initialSpeeds.data());
        yarp::os::Time::delay(0.01);

        setPositions(initialPosition);
        yarp::os::Time::delay(0.01);

//        std::cout << "===============================" << std::endl;
//        std::cout << " Coman MC: Inital speeds are \n" << initialSpeeds.toString() << std::endl;
//        std::cout << "===============================" << std::endl;

    }
    else
        std::cout << "Coman MC: error! Not able to read initial positions";

    return true;
}

bool comanMotionControl::close()
{
    yTrace();

    ImplementControlCalibration2<comanMotionControl, IControlCalibration2>::uninitialize();
    ImplementAmplifierControl<comanMotionControl, IAmplifierControl>::uninitialize();
    ImplementEncodersTimed::uninitialize();
    ImplementPositionControl2::uninitialize();
    ImplementPidControl<comanMotionControl, IPidControl>::uninitialize();
    ImplementControlMode::uninitialize();
    ImplementVelocityControl2::uninitialize();
    ImplementDebugInterface::uninitialize();
    ImplementControlLimits2::uninitialize();
    ImplementTorqueControl::uninitialize();
    ImplementPositionDirect::uninitialize();
    return _comanHandler->deInstance();
}

///////////// PID INTERFACE

bool comanMotionControl::setPidRaw(int j, const Pid &pid)
{
    yTrace() << "joint " << j << "(bId" << jointTobId(j) << ")";
    pid_gains_t p_i_d;
    McBoard *joint_p = getMCpointer(j);
    int ret = 0;

    if( NULL == joint_p)
    {
        yError() << "Calling SetPid on a non-existing joint j"  << j << "(bId" << jointTobId(j) << ")" ;
        return false;
    }
    else
    {
        p_i_d.p = (int32_t)pid.kp;
        p_i_d.i = (int32_t)pid.ki;
        p_i_d.d = (int32_t)pid.kd;
        p_i_d.gain_set = POSITION_GAINS;

        int32_t _max_int = (int32_t) pid.max_int;
        int16_t start_off = (int16_t) pid.stiction_up_val;
        int16_t pid_off = (int16_t) pid.offset;
        int32_t scales[3];
        scales[0] = (int32_t) pid.scale;
        scales[1] = (int32_t) pid.scale;
        scales[2] = (int32_t) pid.scale;

//        yWarning() << "setPid: maxint 32bit" << _max_int << " double" << pid.max_int << "\n";

        ret = (!joint_p->setItem(SET_PID_GAINS, &p_i_d.gain_set, sizeof(p_i_d)) );  // setItem returns 0 if ok, 2 if error
        if(!ret)
            yError() << "SetPid: pid gains returns " << ret << "0 is ok, else is error)";
        yDebug() << "Pid: kp " <<  p_i_d.p <<  "kd " <<  p_i_d.d  << "ki " << p_i_d.i;


        ret &= (!joint_p->setItem(SET_PID_GAIN_SCALE, &scales, 3*sizeof(pid.scale) ));

        //        ret &= (!joint_p->setItem(SET_ILIM_GAIN, &(_max_int), sizeof(_max_int)) );  // comment this line
        ret &= comanMotionControl::setOffsetRaw(j, (int16_t) pid_off);  // j is the yarp joint
    }
    return !ret;
}

bool comanMotionControl::setPidsRaw(const Pid *pids)
{
    yTrace();
    bool ret = true;

    for(int j=0; j<_njoints; j++)
        ret = ret && setPidRaw(j, pids[j]);
    return ret;
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
    // get pid error
    yTrace();
    int32_t error;
    McBoard *joint_p = getMCpointer(j);
    if( NULL == joint_p)
    {
        yError() << "Calling getErrorRaw on a non-existing joint j" << j;
        *err = 0;
        return false;
    }
    // TCP ONLY
    bool ret = (!joint_p->getItem(GET_PID_ERROR, NULL, 1, REPLY_PID_ERROR, &error, sizeof(error)) );
    *err = ((double) error);
    return ret;
}

bool comanMotionControl::getErrorsRaw(double *errs)
{
    bool ret = true;
    for(int bId = 1; bId <= _njoints; bId++)
        ret = ret && getErrorRaw(bId, &errs[bId]);
    return ret;
}

bool comanMotionControl::getOutputRaw(int j, double *out)
{
    int16_t error;
    McBoard *joint_p = getMCpointer(j);
    if( NULL == joint_p)
    {
        *out = 0;
        yError() << "Calling getOutputRaw on a non-existing joint j" << j;
        return false;
    }
    //TCP ONLY
    bool ret = (!joint_p->getItem(GET_PID_OUTPUT, NULL, 1, REPLY_PID_OUTPUT, &error, sizeof(error)) );
    *out = (double) error;
    return ret;
}

bool comanMotionControl::getOutputsRaw(double *outs)
{
    yTrace();
    bool ret = true;
    for(int j=0; j <_njoints; j++)
    {
        ret = ret && getOutputRaw(j, &outs[j]);
    }
    return ret;
}

bool comanMotionControl::getPidRaw(int j, Pid *pid)
{
    yTrace();
    pid_gains_t ComanPid;
    ComanPid.p = ComanPid.i = ComanPid.d = 0;

    bool ret = true;
    McBoard *joint_p = getMCpointer(j);

    if( NULL == joint_p)
    {
        pid->kp = 0;
        pid->ki = 0;
        pid->kd = 0;
        pid->max_int = 0;
        pid->max_output = 0;
        pid->offset = 0;
        pid->scale = 0;
        pid->stiction_down_val = 0;
        pid->stiction_up_val = 0;
        pid->max_int = 0;
        yError() << "Calling GetPid on a non-existing joint j" << j;
        return false;
    }

    int32_t  integral_limit;
    int16_t  pid_offset;
    int32_t  scales[3];
    char pidType = POSITION_GAINS;

     // get back PIDs from the boards  TCP ONLY
    ret &= (!joint_p->getItem(GET_PID_GAINS,      &pidType, 1, REPLY_PID_GAINS,       &ComanPid,        sizeof(ComanPid)) );
    ret &= (!joint_p->getItem(GET_PID_GAIN_SCALE, &pidType, 1, REPLY_PID_GAIN_SCALE,   scales,          sizeof(int32_t)*3) );
    ret &= (!joint_p->getItem(GET_ILIM_GAIN,      &pidType, 1, REPLY_ILIM_GAIN,       &integral_limit,  sizeof(integral_limit)) );
    ret &= (!joint_p->getItem(GET_PID_OFFSET,     NULL,     0, REPLY_PID_OFFSET,      &pid_offset,      sizeof(pid_offset)) );
//     ret &= (!joint_p->getItem(GET_START_OFFSET,   NULL,     1, REPLY_START_OFFSET,    &pid_offset,      sizeof(ComanPid)) );


	if(ret)
	{
	    pid->kp = (double) ComanPid.p;
	    pid->ki = (double) ComanPid.i;
	    pid->kd = (double) ComanPid.d;
	    pid->scale = (double) scales[0];  // using just one of them
	    pid->max_int = (double) integral_limit;
	    pid->offset = ((double) pid_offset);// * mV2V;  ?
	}
	else
	{
		yError() << "get pid failed somehow";
	}
    return ret;
}

bool comanMotionControl::getPidsRaw(Pid *pids)
{
    yTrace();
    bool ret = true;
    for(int j=0; j < _njoints; j++)
        ret = ret && getPidRaw(j, &pids[j]);
    return ret;
}

bool comanMotionControl::getReferenceRaw(int j, double *ref)
{
    yTrace();
    McBoard *joint_p = getMCpointer(j);
    if( NULL == joint_p)
    {
        yError() << "Calling getReferenceRaw on a non-existing joint j" << j;
        *ref = 0;
        return false;
    }
    int32_t ref_pos;
    bool ret = (!joint_p->getItem(GET_DESIRED_POSITION,   NULL, 0, REPLY_DESIRED_POSITION, &ref_pos, sizeof(ref_pos)) );
    *ref = (double) ref_pos;
    return ret;
}

bool comanMotionControl::getReferencesRaw(double *refs)
{
    yTrace();
    bool ret = true;
    for(int j=0; j<_njoints; j++)
        ret = ret && getReferenceRaw(j, &refs[j]);
    return ret;
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
    bool ret = true;
    uint8_t stop = 0;
    McBoard *joint_p = getMCpointer(j);

    if( NULL == joint_p)
    {
        yError() << "Calling disablePidRaw on a non-existing board" << j;
        return false;
    }

    uint8_t bId = jointTobId(j);
    ret = ret && (!_boards_ctrl->start_stop_single_control(bId, stop, POSITION_MOVE));
    ret = ret && (!_boards_ctrl->start_stop_single_control(bId, stop, VELOCITY_MOVE));
    ret = ret && (!_boards_ctrl->start_stop_single_control(bId, stop, TORQUE_MOVE));
    ret = ret && (!joint_p->setItem(SET_TORQUE_ON_OFF, &stop, sizeof(stop)) );

    if(ret)
        _controlMode[j] = VOCAB_CM_IDLE;

    return ret;
}

bool comanMotionControl::enablePidRaw(int j)
{
    setPositionModeRaw(j);
}

bool comanMotionControl::setOffsetRaw(int j, double v)
{
    yTrace();

    _pid_offset[j] = (uint16_t) v;
    return !_boards_ctrl->set_pid_offset(_pid_offset, sizeof(uint16_t) * _njoints);
}


////////////////////////////////////////
//    Velocity PID
////////////////////////////////////////

bool comanMotionControl::setVelPidRaw(int j, const Pid &pid)
{

    yTrace() << "joint " << j << "vel KP" << pid.kp;
    pid_gains_t p_i_d;
    McBoard *joint_p = getMCpointer(j);
    bool ret = true;

    if( NULL == joint_p)
    {
        yError() << "Calling SetVelPid on a non-existing joint" << j;
        return false;
    }
    else
    {
        p_i_d.p = (int32_t)pid.kp;
        p_i_d.i = (int32_t)pid.ki;
        p_i_d.d = (int32_t)pid.kd;
        p_i_d.gain_set = VELOCITY_GAINS;

        int32_t _max_int = (int32_t) pid.max_int;
        int16_t start_off = (int16_t) pid.stiction_up_val;
        int16_t pid_off = (int16_t) pid.offset;
        int32_t scales[3];
        scales[0] = (int32_t) pid.scale;
        scales[1] = (int32_t) pid.scale;
        scales[2] = (int32_t) pid.scale;

        // setItem returns 0 if ok, 2 if error
        ret &= (!joint_p->setItem(SET_PID_GAINS, &p_i_d.gain_set, sizeof(p_i_d)) );
        ret &= (!joint_p->setItem(SET_PID_GAIN_SCALE, &scales, 3*sizeof(int32_t) ) );
        ret &= (!joint_p->setItem(SET_ILIM_GAIN, &(_max_int), sizeof(pid.scale)) );
        ret &= setOffsetRaw(j, (int16_t) pid_off);
    }
    return ret;
}

bool comanMotionControl::setVelPidsRaw(const int n_joint, const int *joints, const Pid *pids)
{
    yTrace();
    bool ret = true;
    for(int j=0; j<n_joint; j++)
        ret = ret && setVelPidRaw(joints[j], pids[j]);
    return ret;
}

bool comanMotionControl::setVelPidsRaw(const Pid *pids)
{
    yTrace();
    bool ret = true;
    for(int j=0; j < _njoints; j++)
        ret = ret && setVelPidRaw(j, pids[j]);
    return ret;
}

bool comanMotionControl::getVelPidRaw(int j, Pid *pid)
{
    yTrace();
    pid_gains_t ComanPid;
    ComanPid.p = ComanPid.i = ComanPid.d = 0;

    bool ret = true;
    McBoard *joint_p = getMCpointer(j);

    if( NULL == joint_p)
    {
        pid->kp = 0;
        pid->ki = 0;
        pid->kd = 0;
        pid->max_int = 0;
        pid->max_output = 0;
        pid->offset = 0;
        pid->scale = 0;
        pid->stiction_down_val = 0;
        pid->stiction_up_val = 0;
        yError() << "Calling GetPid on a non-existing joint j" << j;
        return false;
    }

    int32_t  integral_limit;
    int16_t  pid_offset;
    int32_t  scales[3];
    char pidType = VELOCITY_GAINS;

     // get back PIDs from the boards  TCP ONLY
    ret &= (!joint_p->getItem(GET_PID_GAINS,      &pidType, 1, REPLY_PID_GAINS,       &ComanPid,        sizeof(ComanPid)) );
    ret &= (!joint_p->getItem(GET_PID_GAIN_SCALE, &pidType, 1, REPLY_PID_GAIN_SCALE,   scales,          sizeof(int32_t)*3) );
    ret &= (!joint_p->getItem(GET_ILIM_GAIN,      &pidType, 1, REPLY_ILIM_GAIN,       &integral_limit,  sizeof(integral_limit)) );
    ret &= (!joint_p->getItem(GET_PID_OFFSET,     NULL,     0, REPLY_PID_OFFSET,      &pid_offset,      sizeof(pid_offset)) );
//     ret &= (!joint_p->getItem(GET_START_OFFSET,   NULL,     1, REPLY_START_OFFSET,    &pid_offset,      sizeof(ComanPid)) );

    pid->kp = (double) ComanPid.p;
    pid->ki = (double) ComanPid.i;
    pid->kd = (double) ComanPid.d;
    pid->scale = (double) scales[0];  // using just one of them
    pid->max_int = (double) integral_limit;
    pid->offset = ((double) pid_offset);// * mV2V;  ?

    return ret;
}

bool comanMotionControl::getVelPidsRaw(const int n_joint, const int *joints, Pid *pids)
{
    yTrace();
    bool ret = true;
    for(int j=0; j< n_joint; j++)
    {
        // TCP ONLY
        ret &= getVelPidRaw(joints[j], &pids[j]);
    }
    return ret;
}

bool comanMotionControl::getVelPidsRaw(Pid *pids)
{
    yTrace();
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        // TCP ONLY
        ret &= getVelPidRaw(j, &pids[j]);
    }
    return ret;
}


////////////////////////////////////////
//    Velocity control interface raw  //
////////////////////////////////////////

bool comanMotionControl::setVelocityModeRaw(int j)
{
    yTrace();
    bool ret = true;
    uint8_t bId = jointTobId(j);
    McBoard *joint_p = getMCpointer(j);

    if( NULL == joint_p)
    {
        yError() << "Calling setVelocityModeRaw on a non-existing joint j" << j;
        return false;
    }

    if(_controlMode[j] == VOCAB_CM_VELOCITY)
    {
        // nothing to do here
        yDebug() << "joint "<< j << "already in velocity mode";
        ret = true;
    }
    else
    {
        ret = disablePidRaw(j);
        ret = ret && (!_boards_ctrl->start_stop_single_control(bId, 1, VELOCITY_MOVE));
    }

    if(ret)
        _controlMode[j] = VOCAB_CM_VELOCITY;
    else
        yError() << "in setVelocityModeRaw for joint " << j;
    return ret;
}

bool comanMotionControl::setVelocityModeRaw(const int n_joint, const int *joints)
{
    yTrace();
    int start = 0x03;
    std::vector<int> bId_set;
    bId_set.resize(n_joint);
    for(int j=0; j < n_joint; j++)
    {
        bId_set[j] = (int) jointTobId( joints[j] );
    }
    return (!_boards_ctrl->start_stop_set_control(bId_set, start, VELOCITY_MOVE));
}

bool comanMotionControl::setVelocityModeRaw()
{
    bool ret = true;
    for(int i=0; i<_njoints; i++)
    {
        ret = ret && disablePid(i);
    }
    ret = ret && (!_boards_ctrl->start_stop_control(1, VELOCITY_MOVE) );
    return ret;
}

bool comanMotionControl::velocityMoveRaw(int j, double sp)
{
    uint8_t bId = jointTobId(j);
    int16_t des_vel = (int16_t)(sp/COMAN_POS_TO_VEL_GAIN);   // differenza tra la unità di misura usata in velocità e posizione;

    bool ret = (!_boards_ctrl->set_velocity_group(&bId, &des_vel, 1));

    if(ret)
    {
        _ref_speeds[j] = des_vel;   // differenza tra la unità di misura usata in velocità e posizione;;
    }
    else
    {
        cout << "ERROR in velocityMoveRaw for joint " << j;
    }

    return ret;
}

bool comanMotionControl::velocityMoveRaw(const int n_joint, const int *joints, const double *spds)
{
    int16_t des_vel[n_joint];
    uint8_t bIds[n_joint];

    for(int i=0; i<n_joint; i++)
    {
        des_vel[i] = (int16_t)(spds[i]/COMAN_POS_TO_VEL_GAIN);
        bIds[i] = (uint8_t) jointTobId( joints[i]);
    }

    bool ret = (!_boards_ctrl->set_velocity_group(bIds, des_vel, n_joint));

    if(ret)
    {
        // cache value for future use
        for(int i=0; i<n_joint; i++)
        {
            // here is an index, therefore DO NOT convert with jointTobId()
            _ref_speeds[joints[i]] = des_vel[i];
        }
    }
    else
    {
        cout << "ERROR in velocityMoveRaw for set of joints ";
    }
    return ret;
}

bool comanMotionControl::velocityMoveRaw(const double *spds)
{
    bool ret = true;
    int16_t tmp_spds[_njoints];

    for(int j=1; j <= _njoints; j++)
    {
        tmp_spds[j] = (int16_t)(spds[j]/COMAN_POS_TO_VEL_GAIN);   // differenza tra la unità di misura usata in velocità e posizione;
    }

    ret = ret && (!_boards_ctrl->set_velocity(tmp_spds, _njoints * sizeof(int16_t)));

    if(ret)
    {
        for(int j=1; j <= _njoints; j++)
        {
            _ref_speeds[j] = tmp_spds[j];
        }
    }
    else
    {
        cout << "ERROR in velocityMoveRaw for all joint ";
    }
    return ret;
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
    *ax = _njoints;
    return true;
}

bool comanMotionControl::setPositionModeRaw()
{
    yTrace();
    bool ret = true;

    for(int i =0; i<_njoints; i++)
    {
    	ret = ret && setPositionModeRaw(i);
    }
    return ret;
/*
    int start = 1;
    bool ret = (!_boards_ctrl->start_stop_control(start, POSITION_MOVE) );

    if(ret)
    {
        for(int i=0; i<_njoints; i++)
            _controlMode[i] = VOCAB_CM_POSITION;    // I assume that if the message was sent, the boards correclty process it
    }
    else
        yError() << "while setting position mode for all joints";

    return ret;
*/
}

bool comanMotionControl::positionMoveRaw(int j, double ref)
{
//    struct timespec t_start, t_end;
//    clock_gettime(CLOCK_REALTIME, &t_start);


    yTrace();
    uint8_t bId = jointTobId(j);

    _ref_positions[j] = (int32_t) ref;

        /*clock_gettime(CLOCK_REALTIME, &t_end);
        timespec temp;
        if ((t_end.tv_nsec-t_start.tv_nsec)<0) {
            temp.tv_sec = t_end.tv_sec-t_start.tv_sec-1;
            temp.tv_nsec = 1e9+t_end.tv_nsec-t_start.tv_nsec;
        } else {
            temp.tv_sec = t_end.tv_sec-t_start.tv_sec;
            temp.tv_nsec = t_end.tv_nsec-t_start.tv_nsec;
        }
        printf("%ld.%06ld ms\n",
                 (long)(temp.tv_nsec) / 1000000,
                 (long)(temp.tv_nsec) % 1000000);*/

    return (!_boards_ctrl->set_position_velocity_group(&bId, &_ref_positions[j], &_ref_speeds[j], 1) );
}

bool comanMotionControl::positionMoveRaw(const double *refs)
{
    yTrace();
    for(int i=0; i<_njoints; i++)
        _ref_positions[i] = (int32_t) refs[i];

    // here the function wants the num of elements (joints), contiene internamente un loop su tutte le schede
    return (!_boards_ctrl->set_position_velocity(_ref_positions, _ref_speeds, _njoints) );
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

bool comanMotionControl::checkMotionDoneRaw(bool *flag)
{
    // loop su _mcs, non sul numero dei giunti!!
    bool ret = true;
    for(int i=0; i<_njoints; i++)
    {
        ret = ret && checkMotionDoneRaw(i, &flag[i]);
    }
    return ret;
}

bool comanMotionControl::checkMotionDoneRaw(int j, bool *flag)
{
    double actual_pos, delta;
    return true;

    bool ret = getEncoderRaw(j, &actual_pos);

    delta = fabs(( ((double) _ref_positions[j]) - actual_pos) /_angleToEncoder[j]);
//    cout << "check motion done : j " << j << " delta" << delta ;

    if( delta <= COMAN_POS_THRESHOLD )
        *flag = true;
    else
        *flag = false;

    return ret;
}

bool comanMotionControl::setRefSpeedRaw(int j, double sp)
{
//    yTrace();
    _ref_speeds[j] = (int16_t) (sp/COMAN_POS_TO_VEL_GAIN);  // differenza tra la unità di misura usata in velocità e posizione

    // TODO convertire nella funzione a gruppi di giunti
    bool ret = (!_boards_ctrl->set_velocity(_ref_speeds, _njoints * sizeof(int16_t)) );

    if(!ret)
    {
       yError() << "ERROR in setRefSpeedRaw for joint " << j;
    }
    return ret;
}

bool comanMotionControl::setRefSpeedsRaw(const double *spds)
{
    yTrace();
    for(int i=0; i<_njoints; i++)
        _ref_speeds[i] = (int16_t) (spds[i] /COMAN_POS_TO_VEL_GAIN);  // differenza tra la unità di misura usata in velocità e posizione;

    bool ret = (!_boards_ctrl->set_velocity(_ref_speeds, _njoints * sizeof(int16_t)) );

    if(!ret)
    {
        yError() << "ERROR in setRefSpeedsRaw for all joint ";
    }

    return ret;
}

bool comanMotionControl::setRefAccelerationRaw(int j, double acc)
{
    yTrace();
    McBoard *joint_p = getMCpointer(j);
    uint16_t tmp_acc = (int16_t) acc;

    if(NULL == joint_p)
    {
        yError() << "Trying to use setRefAccelerationsRaw on a non existing joint" << j;
        return false;
    }
    else
    {
        // TCP ONLY
        return (!joint_p->setItem(SET_ACCELERATION, &tmp_acc, sizeof(int16_t)) );  // setItem returns 0 if ok, 2 if error
    }
}

bool comanMotionControl::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    yTrace();
    bool ret = true;
    McBoard *joint_p = NULL;

    // TCP ONLY
    for(int j=0; j<n_joint; j++)
    {
        joint_p = getMCpointer(joints[j]);
        int16_t tmp_acc = (int16_t) accs[j];

        if(NULL != joint_p)
            ret = ret && (!joint_p->setItem(SET_ACCELERATION, &tmp_acc, sizeof(int16_t)));   // setItem returns 0 if ok, 2 if error
        else
        {
            yError() << "Trying to use setRefAccelerationsRaw on a non existing joint" << joints[j];
            ret = false;
        }
    }
    return ret;
}

bool comanMotionControl::setRefAccelerationsRaw(const double *accs)
{
    yTrace();
    bool ret = true;
    McBoard *joint_p = NULL;

    // TCP ONLY
    for(int j=0; j < _njoints; j++)
    {
        joint_p = getMCpointer(j);
        int16_t tmp_acc = (int16_t) accs[j];

        if(NULL != joint_p)
            ret = ret && (!joint_p->setItem(SET_ACCELERATION, &tmp_acc, sizeof(int16_t)));   // setItem returns 0 if ok, 2 if error
        else
        {
            yError() << "Trying to use setRefAccelerationsRaw on a non existing joint" << j;
            ret = false;
        }
    }
    return ret;
}

bool comanMotionControl::getRefSpeedRaw(int j, double *spd)
{
    yTrace();
    *spd = (double) _ref_speeds[j] * COMAN_POS_TO_VEL_GAIN;
    return true;
}

bool comanMotionControl::getRefSpeedsRaw(double *spds)
{
    yTrace();
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret = ret && getRefSpeedRaw(j, &spds[j]);
    }
    return ret;
}

bool comanMotionControl::getRefAccelerationRaw(int j, double *acc)
{
    yTrace();
#warning "is there a conversion factor for acceleration? which is its mes. unit?"
    McBoard *joint_p = getMCpointer(j);
    bool ret;
    int16_t tmp;

    if(NULL == joint_p)
    {
        yError() << "Trying to use getRefAccelerationRaw on a non existing joint" << j;
        ret = false;
    }
    else
    {
        // TCP ONLY
        // //     ret = (!joint_p->getItem(GET_TORQUE_ON_OFF, NULL, 0, REPLY_TORQUE_ON_OFF, &trq_on, sizeof(trq_on)));
        ret = (!joint_p->getItem(GET_ACCELERATION, NULL, 0, REPLY_ACCELERATION, &tmp, sizeof(int16_t)) );  // setItem returns 0 if ok, 2 if error
        *acc = (double) tmp;
    }
    return ret;
}

bool comanMotionControl::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    yTrace();
    bool ret = true;
    McBoard *joint_p = NULL;

    int16_t tmp;

    for(int idx=0; idx<n_joint; idx++)
    {
        joint_p = getMCpointer(joints[idx]);

        if(NULL == joint_p)
        {
            yError() << "Trying to use getRefAccelerationsRaw on a non existing joint" << joints[idx];
            ret = false;
        }
        else
        {
            // TCP ONLY
            ret = ret && (!joint_p->getItem(GET_ACCELERATION, NULL, 0, REPLY_ACCELERATION, &tmp, sizeof(uint16_t)));   // setItem returns 0 if ok, 2 if error
            accs[idx] = (double) tmp;
        }
    }
    return ret;
}

bool comanMotionControl::getRefAccelerationsRaw(double *accs)
{
    yTrace();
    int idx;
    int16_t tmp_acc;
    bool ret = true;
    McBoard *joint_p = NULL;

    for(int j=0; j < _njoints; j++)
    {
        joint_p = getMCpointer(j);

        if(NULL == joint_p)
        {
            yError() << "Trying to use getRefAccelerationsRaw on a non existing board" << j;
            ret = false;
        }
        else
        {
            // TCP ONLY
            ret = ret && (!joint_p->getItem(GET_ACCELERATION, NULL, 0, REPLY_ACCELERATION, &tmp_acc, sizeof(int16_t)));   // setItem returns 0 if ok, 2 if error
            accs[j] = (double) tmp_acc;
        }
    }
    return ret;
}

bool comanMotionControl::stopRaw(int j)
{
    yTrace();
    bool ret = true;
    uint8_t stop = 0;
    McBoard *joint_p = NULL;
    uint8_t bId = jointTobId(j);

    switch(_controlMode[j])
    {
        case VOCAB_CM_POSITION:
            ret = !_boards_ctrl->start_stop_single_control((uint8_t) bId, stop, POSITION_MOVE);
            break;

        case VOCAB_CM_VELOCITY:
            ret = !_boards_ctrl->start_stop_single_control((uint8_t) bId, stop, VELOCITY_MOVE);
            break;

        case VOCAB_CM_TORQUE:
            ret = !_boards_ctrl->start_stop_single_control((uint8_t) bId, stop, TORQUE_MOVE);
            break;

        default:
            yError() << "Calling stop from an unknown control mode (joint" << j << ")... stopping everything!";
            ret = ret && (!_boards_ctrl->start_stop_single_control((uint8_t) bId, stop, POSITION_MOVE));
            ret = ret && (!_boards_ctrl->start_stop_single_control((uint8_t) bId, stop, VELOCITY_MOVE));
            ret = ret && (!_boards_ctrl->start_stop_single_control((uint8_t) bId, stop, TORQUE_MOVE));
            ret = false;
            break;
    }
    _controlMode[j] = VOCAB_CM_IDLE;
    return ret;
}

bool comanMotionControl::stopRaw(const int n_joint, const int *joints)
{
    yTrace();
    bool ret = true;
    uint8_t stop = 0;
    std::vector<int> board_set;
    board_set.resize(n_joint);

    for(int idx=0; idx < n_joint; idx++)
    {
        board_set[idx] = jointTobId(joints[idx]);
    }

    ret = ret && (!_boards_ctrl->start_stop_set_control(board_set, stop, POSITION_MOVE));
    ret = ret && (!_boards_ctrl->start_stop_set_control(board_set, stop, VELOCITY_MOVE));
    ret = ret && (!_boards_ctrl->start_stop_set_control(board_set, stop, TORQUE_MOVE));

	if(ret)
	{
		for(int idx=0; idx < n_joint; idx++)
		{
		    _controlMode[joints[idx]] = VOCAB_CM_IDLE;
		}
	}
    return ret;
}

bool comanMotionControl::stopRaw()
{
    yTrace();
    bool ret = true;
    uint8_t stop = 0;

    ret = ret && (!_boards_ctrl->start_stop_control( stop, POSITION_MOVE));
    ret = ret && (!_boards_ctrl->start_stop_control( stop, VELOCITY_MOVE));
    ret = ret && (!_boards_ctrl->start_stop_control( stop, TORQUE_MOVE));

	if(ret)
	for(int i=0; 0<_njoints; i++)
    	_controlMode[i] = VOCAB_CM_IDLE;
    	
    return ret;
}
///////////// END Position Control INTERFACE  //////////////////

////////////////////////////////////////
//     Position control 2 interface   //
////////////////////////////////////////

bool comanMotionControl::setPositionModeRaw(const int n_joint, const int *joints)
{
    yTrace();
    int start = 0x03;
    std::vector<int> board_set;
    board_set.resize(n_joint);

    bool ret = true;
    for(int idx=0; idx < n_joint; idx++)
    {
    	ret = ret && setPositionModeRaw(joints[idx]);
//        board_set[idx] = jointTobId(joints[idx]);
    }
    return ret;
    //return (!_boards_ctrl->start_stop_set_control(board_set, start, POSITION_MOVE));
}

bool comanMotionControl::positionMoveRaw(const int n_joint, const int *joints, const double *refs)
{
    yTrace();
    int16_t *tmp_sp   = new int16_t [n_joint];
    int32_t *tmp_pos  = new int32_t [n_joint];
    uint8_t *tmp_bIds = new uint8_t [n_joint];

    for(int i=0; i<n_joint; i++)
    {
        tmp_bIds[i] = jointTobId(joints[i]);
        _ref_positions[joints[i]] = (int32_t) refs[i];
        tmp_sp[i]  = _ref_speeds[joints[i]];
        tmp_pos[i] = _ref_positions[joints[i]];
    }
    bool ret = (!_boards_ctrl->set_position_velocity_group(tmp_bIds, tmp_pos, tmp_sp, n_joint) );
    delete [] tmp_sp;
    delete [] tmp_pos;
    delete [] tmp_bIds;
    return ret;
}

bool comanMotionControl::relativeMoveRaw(const int n_joint, const int *joints, const double *deltas)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("relativeMoveRaw");
}

bool comanMotionControl::checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags)
{
    bool ret = true;
    for(int i=0; i<n_joint; i++)
    {
        ret = ret && checkMotionDoneRaw(joints[i], &flags[i]);
    }
    return ret;
}

bool comanMotionControl::setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds)
{
    int16_t tmp_sp[n_joint];
    uint8_t tmp_bId[n_joint];

    for(int idx=0; idx<n_joint; idx++)
    {
        tmp_bId[idx] = jointTobId(joints[idx]);
        // differenza tra la unità di misura usata in velocità e posizione;
        tmp_sp[idx] = (int16_t) (spds[idx] /COMAN_POS_TO_VEL_GAIN); 
        _ref_speeds[joints[idx]] = tmp_sp[idx];
    }
    bool ret = (!_boards_ctrl->set_velocity_group(tmp_bId, tmp_sp, n_joint));

    if(!ret)   yError() << "ERROR in setRefSpeedsRaw for a subset of joints";

    return ret;
}

bool comanMotionControl::getRefSpeedsRaw(const int n_joint, const int *joints, double *spds)
{
    bool ret = true;
    for(int idx=0; idx< n_joint; idx++)
    {
        ret = ret && getRefSpeedRaw(joints[idx], &spds[idx]);
    }
    return ret;
}


 ///////////// END Position Control 2 INTERFACE  //////////////////

// ControlMode
bool comanMotionControl::setPositionModeRaw(int j)
{
    yTrace();
    McBoard *joint_p = getMCpointer(j);
    uint8_t bId = jointTobId(j);
    uint8_t start = 1;
    uint8_t stop = 0;
    bool ret = true;
    double initialPosition;

    if( NULL == joint_p)
    {
        yError() << "Calling setPositionModeRaw on a non-existing joint j" << j;
        return false;
    }

    // stop what is running
    switch(_controlMode[j])
    {
        case VOCAB_CM_IDLE:
        if(getEncoder(j, &initialPosition) )
            positionMove(j, initialPosition);
        else
            std::cout << "Coman MC: error! Not able to read initial positions";

        Time::delay(0.01);
        ret = ret && (!_boards_ctrl->start_stop_single_control(bId, start, POSITION_MOVE));
        Time::delay(0.01);

        break;
        case VOCAB_CM_POSITION:
            yDebug() << "joint "<< j << "already in position mode";
            return true;
            break;

        case VOCAB_CM_VELOCITY:
            // now firmware is able to do a smooth transition from pos to vel, so nothing to do here
            if(getEncoder(j, &initialPosition) )
                positionMove(j, initialPosition);
            else
                std::cout << "Coman setPositionModeRaw failed! Not able to read encoder";
            break;

        case VOCAB_CM_TORQUE:
            yDebug() << "joint "<< j << "stopping torque mode";
            ret = ret && (!joint_p->setItem(SET_TORQUE_ON_OFF, &stop, sizeof(stop)));   // setItem returns 0 if ok, 2 if error

            ret = ret && (!joint_p->setItem(SET_MOTOR_CONFIG, &motor_config_mask[j], sizeof(motor_config_mask[j])) );
            printf("joint %d set motor config 0x%0X\n", j, motor_config_mask[j]);

            setPidRaw(j, pid[j]);
            setTorquePidRaw(j, pidTorque[j]);

            ret = ret && (!_boards_ctrl->start_stop_single_control(bId, start, POSITION_MOVE));

            if(getEncoder(j, &initialPosition) )
                positionMove(j, initialPosition);
            else
                std::cout << "Coman setPositionModeRaw failed! Not able to read encoder";

            if(!ret)
                yError() << "error while stopping torque mode";
            break;

        case VOCAB_CM_IMPEDANCE_POS:
        case VOCAB_CM_IMPEDANCE_VEL:
//            ret = ret && (!_boards_ctrl->start_stop_single_control(bId, stop, POSITION_MOVE));
//            ret = ret && (!_boards_ctrl->start_stop_single_control(bId, stop, VELOCITY_MOVE));
//            ret = ret && (!joint_p->setItem(SET_TORQUE_ON_OFF, &stop, sizeof(stop)) );

            disablePidRaw(j);
            Time::delay(0.01);

            ret = ret && (!joint_p->setItem(SET_MOTOR_CONFIG, &motor_config_mask[j], sizeof(motor_config_mask[j])) );
            printf("joint %d set motor config 0x%0X\n", j, motor_config_mask[j]);


//            	ret = ret && (!joint_p->setItem(SET_MOTOR_CONFIG2, &motor_config_mask2[j], sizeof(motor_config_mask2[j])) );

            setPidRaw(j, pid[j]);
            setTorquePidRaw(j, pidTorque[j]);

//            	ret = ret && (!joint_p->setItem(SET_TORQUE_ON_OFF, &start, sizeof(start)) );
            ret = ret && (!_boards_ctrl->start_stop_single_control(bId, start, POSITION_MOVE));


            if(getEncoder(j, &initialPosition) )
                positionMove(j, initialPosition);
            else
                std::cout << "Coman setPositionModeRaw failed! Not able to read encoder";

            break;

        default:
            yWarning() << "joint "<< j << "set position mode coming from unknown controlmode... stop everything and then enable position\n";
            if(getEncoder(j, &initialPosition) )
                positionMove(j, initialPosition);
            else
                std::cout << "Coman setPositionModeRaw failed! Not able to read encoder";

            disablePidRaw(j);
            break;
    }
    ret = ret && (!_boards_ctrl->start_stop_single_control(bId, start, POSITION_MOVE));      //  1 = ON

    if(ret)
        _controlMode[j] = VOCAB_CM_POSITION;
    else
        yError() << "while setting position mode for joint " << j;
    return ret;
}



bool comanMotionControl::setTorqueModeRaw(int j)
{
    // Chiedere info
    McBoard *joint_p = getMCpointer(j);
    uint8_t bId = jointTobId(j);
    if( NULL == joint_p)
    {
        yError() << "Calling setTorqueModeRaw on a non-existing joint j" << j;
        return false;
    }

    bool ret = true;
    uint8_t start = 1;
    uint8_t stop = 0;
    uint16_t motor_config_mask = 0;

    ret = ret && (!joint_p->setItem(SET_TORQUE_ON_OFF, &stop, sizeof(stop)) );
    ret = ret && (!_boards_ctrl->start_stop_single_control(bId, stop, POSITION_MOVE));
    ret = ret && (!_boards_ctrl->start_stop_single_control(bId, stop, VELOCITY_MOVE));

    ret = ret && (!joint_p->getItem(GET_MOTOR_CONFIG, NULL, 0, REPLY_MOTOR_CONFIG, &motor_config_mask, 2) );
    printf("joint %d got motor config 0x%0X\n", j, motor_config_mask);

    motor_config_mask |= 0x4803;   // add bit about impedance control.??? needed here???

    ret = ret && (!joint_p->setItem(SET_MOTOR_CONFIG, &motor_config_mask, sizeof(motor_config_mask)) );
    printf("joint %d set motor config 0x%0X\n", j, motor_config_mask);
/*
    uint16_t motor_config_mask2 = 0x0;       //0 Moving Average 1 ButterWorth 2 Least Square 3 Jerry Pratt -- TODO cambiano a runtime/configurazione??
    ret = ret && (!joint_p->setItem(SET_MOTOR_CONFIG2, &motor_config_mask2, sizeof(motor_config_mask2)) );
*/
    // set pids
    Pid pid;
    pid.kp = 0;
    pid.ki = 0;
    pid.kd = 0;

    ret = ret && setPidRaw(j, pid);                     yarp::os::Time::delay(0.01);
    ret = ret && setTorquePidRaw(j, _trqPids[j]);       yarp::os::Time::delay(0.01);

    ret = ret && (!joint_p->setItem(SET_TORQUE_ON_OFF, &start, sizeof(start)) );
    ret = ret && (!_boards_ctrl->start_stop_single_control(bId, start, POSITION_MOVE));

    printf("setTorqueModeRaw joint [%d]: message was sent with %s\n", j, ret? "SUCCESS" : "FAILURE");

    if(ret)
        _controlMode[j] = VOCAB_CM_TORQUE;    
    return (ret);
}

bool comanMotionControl::setTorqueModeRaw( )
{
    bool ret = true;
    for(int j=0; j<_njoints; j++)
        ret = ret && setTorqueModeRaw(j);

    return ret;
}

bool comanMotionControl::setImpedancePositionModeRaw(int j)
{
    // Chiedere info
    double initialPosition;
    McBoard *joint_p = getMCpointer(j);
    uint8_t bId = jointTobId(j);
    if( NULL == joint_p)
    {
        yError() << "Calling setImpedancePositionModeRaw on a non-existing joint j" << j;
        return false;
    }

    bool ret = true;
    uint8_t start = 1;  //1 for torque control    0 for position control
    uint8_t stop = 0;
    uint16_t motor_config_mask = 0;
//    uint16_t motor_config_mask2 = 0;

    ret = ret && (!_boards_ctrl->start_stop_single_control(bId, stop, POSITION_MOVE));
    ret = ret && (!_boards_ctrl->start_stop_single_control(bId, stop, VELOCITY_MOVE));
    ret = ret && (!joint_p->setItem(SET_TORQUE_ON_OFF, &stop, sizeof(stop)) );

    ret = ret && (!joint_p->getItem(GET_MOTOR_CONFIG, NULL, 0, REPLY_MOTOR_CONFIG, &motor_config_mask, 2) );
    printf("joint %d got motor config 0x%0X\n", j, motor_config_mask);

    motor_config_mask |= 0x4803;   // add bit about impedance control.

    ret = ret && (!joint_p->setItem(SET_MOTOR_CONFIG, &motor_config_mask, sizeof(motor_config_mask)) );
    printf("joint %d set motor config 0x%0X\n", j, motor_config_mask);
/*
    ret = ret && (!joint_p->getItem(GET_MOTOR_CONFIG2, NULL, 0, REPLY_MOTOR_CONFIG2, &motor_config_mask2, 2) );
    uint16_t motor_config_mask2 |= 0x0;       //0 Moving Average 1 ButterWorth 2 Least Square 3 Jerry Pratt -- TODO cambiano a runtime/configurazione??
    ret = ret && (!joint_p->setItem(SET_MOTOR_CONFIG2, &motor_config_mask2, sizeof(motor_config_mask2)) );
*/
    // set impedance position pids
    ret = ret && setPidRaw(j, _impPosPids[j]);      yarp::os::Time::delay(0.01);
    ret = ret && setTorquePidRaw(j, _trqPids[j]);   yarp::os::Time::delay(0.01);

    ret = ret && (!joint_p->setItem(SET_TORQUE_ON_OFF, &start, sizeof(start)) );
    ret = ret && (!_boards_ctrl->start_stop_single_control(bId, start, POSITION_MOVE));

    if(getEncoder(j, &initialPosition) )
        positionMove(j, initialPosition);
    else
        std::cout << "Coman MC: error! Not able to read initial positions";


    if(ret)
        _controlMode[j] = VOCAB_CM_IMPEDANCE_POS;
    printf("setImpedancePositionModeRaw ret value is %d\n", ret);
    return ret;
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
    // cached value must be correct!!

    bool ret = true;
    McBoard *joint_p = getMCpointer(j);   //  -> giusto

    if( NULL == joint_p)
    {
        return false;
    }

    ts_bc_data_t bc_data;
    mc_bc_data_t &data = bc_data.raw_bc_data.mc_bc_data;

    joint_p->get_bc_data(bc_data);

    uint8_t faults = data.faults();

    //    *v= ((bool)faults *  VOCAB3('e','r','r')) + ((1 - (bool)faults ) *_controlMode[j]);

    if (faults)
    {
        *v = VOCAB3('e','r','r');
        std::cout << "Joint " << j << " returned error code " << (int)faults;
    }
    else
    {
        *v = _controlMode[j];
    }
    return true;
}

bool comanMotionControl::getControlModesRaw(int* v)
{
    for(int j=0; j<_njoints; j++)
        getControlModeRaw(j, &v[j]);
    return true;
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

bool comanMotionControl::getEncoderRaw(int j, double *enc)
{
//    struct timespec t_start, t_end;
//    clock_gettime(CLOCK_REALTIME, &t_start);

    bool ret = true;
    McBoard *joint_p = getMCpointer(j);   //  -> giusto

    if( NULL == joint_p)
    {
//         yError() << "Calling getEncoderRaw on a non-existing joint j" << j;
        *enc = j;   // return the joint number!!
        return false;
    }

    ts_bc_data_t bc_data;
    mc_bc_data_t &data = bc_data.raw_bc_data.mc_bc_data;
    if(bc_policy & BC_POLICY_MOTOR_POSITION)  // se viene broadcastata... usare la ricezione udp
    {
        joint_p->get_bc_data(bc_data);
        ret = true;
    }
    else                                      // altrimenti chiedila in TCP...usare con cautela!!
    {
        yWarning() << "Asking encoder through TCP!!";
        ret = (!joint_p->getItem(GET_ENCODER_POSITION, NULL, 1, REPLY_ENCODER_POSITION, &data.Position, sizeof(int)) );
    }

    *enc = (double) data.Position;

//    clock_gettime(CLOCK_REALTIME, &t_end);
//    timespec temp;
//    if ((t_end.tv_nsec-t_start.tv_nsec)<0) {
//        temp.tv_sec = t_end.tv_sec-t_start.tv_sec-1;
//        temp.tv_nsec = 1e9+t_end.tv_nsec-t_start.tv_nsec;
//    } else {
//        temp.tv_sec = t_end.tv_sec-t_start.tv_sec;
//        temp.tv_nsec = t_end.tv_nsec-t_start.tv_nsec;
//    }
//    printf("%ld.%06ld ms",
//             (long)(temp.tv_nsec) / 1000000,
//             (long)(temp.tv_nsec) % 1000000);


    return ret;
}

bool comanMotionControl::getEncodersRaw(double *encs)
{
    double tmp;
    for(int i=0; i<_njoints; i++)
    {
        getEncoderRaw(i, &tmp);
        encs[i] = tmp;
    }
    return true;
}

bool comanMotionControl::getEncoderSpeedRaw(int j, double *spd)
{
    McBoard *joint_p = getMCpointer(j);   //  -> giusto

    if( NULL == joint_p)
    {
        yError() << "Calling getEncoderSpeedRaw on a non-existing joint j" << j;
        *spd = j;   // return the joint number!!
        return false;
    }
    // viene probabilmente broadcastata... usare la ricezione udp per questo. Così è corretto?
    ts_bc_data_t bc_data;
    mc_bc_data_t &data = bc_data.raw_bc_data.mc_bc_data;
#warning "this implies a memcopy!! To be optimized!! And add timestamp"
    joint_p->get_bc_data(bc_data);
    *spd = (double) data.Velocity * COMAN_POS_TO_VEL_GAIN; //Ottimo...

    return true;
}

bool comanMotionControl::getEncoderSpeedsRaw(double *spds)
{
    double tmp;
    bool ret = true;
    for(int i=0; i<_njoints; i++)
    {
        ret &=getEncoderSpeedRaw(i, &spds[i]);
    }
    return ret;
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
//    struct timespec t_start, t_end;
//    clock_gettime(CLOCK_REALTIME, &t_start);

    double tmp;
    bool ret = true;
    for(int i=0; i<_njoints; i++)
    {
        ret = ret && getEncoderTimedRaw(i, &encs[i], &stamps[i]);
    }

//    clock_gettime(CLOCK_REALTIME, &t_end);
//    timespec temp;
//    if ((t_end.tv_nsec-t_start.tv_nsec)<0) {
//        temp.tv_sec = t_end.tv_sec-t_start.tv_sec-1;
//        temp.tv_nsec = 1e9+t_end.tv_nsec-t_start.tv_nsec;
//    } else {
//        temp.tv_sec = t_end.tv_sec-t_start.tv_sec;
//        temp.tv_nsec = t_end.tv_nsec-t_start.tv_nsec;
//    }
//    printf("%ld.%06ld ms",
//             (long)(temp.tv_nsec) / 1000000,
//             (long)(temp.tv_nsec) % 1000000);


    return ret;
}

bool comanMotionControl::getEncoderTimedRaw(int j, double *enc, double *stamp)
{
//    struct timespec t_start, t_end;
//    clock_gettime(CLOCK_REALTIME, &t_start);

    McBoard *joint_p = getMCpointer(j);

    if( NULL == joint_p)
    {
         //yError() << "Calling getEncoderTimedRaw on a non-existing joint j" << j;
        *enc = j;   // return the joint number!!
        *stamp = 0;
        return false;
    }
    // viene probabilmente broadcastata... usare la ricezione udp per questo. Così è corretto?
    ts_bc_data_t bc_data;
    mc_bc_data_t &data = bc_data.raw_bc_data.mc_bc_data;
#warning "this implies a memcopy!! To be optimized!! And add timestamp"
    joint_p->get_bc_data(bc_data);
    *enc = (double) data.Position;

//     *stamp = data.Timestamp;     // if dspdata have theyr own timestamp
    *stamp=Time::now();           // if dps data doesn't have theyr oen timestamp

//    clock_gettime(CLOCK_REALTIME, &t_end);
//    timespec temp;
//    if ((t_end.tv_nsec-t_start.tv_nsec)<0) {
//        temp.tv_sec = t_end.tv_sec-t_start.tv_sec-1;
//        temp.tv_nsec = 1e9+t_end.tv_nsec-t_start.tv_nsec;
//    } else {
//        temp.tv_sec = t_end.tv_sec-t_start.tv_sec;
//        temp.tv_nsec = t_end.tv_nsec-t_start.tv_nsec;
//    }
//    printf("%ld.%06ld ms",
//             (long)(temp.tv_nsec) / 1000000,
//             (long)(temp.tv_nsec) % 1000000);


    return true;
}

////// Amplifier interface
//
bool comanMotionControl::enableAmpRaw(int j)
{
    yTrace();
    return setPositionModeRaw(j);
}

bool comanMotionControl::disableAmpRaw(int j)
{
    yTrace();
    McBoard *joint_p = getMCpointer(j);

    if( NULL == joint_p)
    {
        yError() << "Calling disableAmpRaw on a non-existing joint j" << j;
        return false;
    }
    return (!joint_p->setItem(CLEAR_BOARD_FAULT, 0, 0));  // setItem returns 0 if ok, 2 if error
}

bool comanMotionControl::getCurrentRaw(int j, double *curr)
{
    McBoard *joint_p = getMCpointer(j);

    if( NULL == joint_p)
    {
    	yError() << "Calling getEncoderTimedRaw on a non-existing joint j" << j;
        *curr = j;   // return the joint number!!
        return false;
    }
    // viene probabilmente broadcastata... usare la ricezione udp per questo.
    ts_bc_data_t bc_data;
    mc_bc_data_t &data = bc_data.raw_bc_data.mc_bc_data;

    joint_p->get_bc_data(bc_data);
    *curr = (double) data.Current;

    return true;
}

bool comanMotionControl::getCurrentsRaw(double *vals)
{
	bool ret = true;
    double tmp;
    for(int i=0; i<_njoints; i++)
    {
        ret = ret && getCurrentRaw(i, &tmp);
        vals[i] = tmp;
    }
    return ret;
}

bool comanMotionControl::setMaxCurrentRaw(int j, double val)
{
    yTrace();
    McBoard *joint_p = getMCpointer(j);
    if( NULL == joint_p)
    {
        return false;
    }
    int16_t tmp = (int16_t) val;
    bool ret = (!joint_p->setItem(SET_CURRENT_LIMIT, &tmp, sizeof(tmp)) );
    return ret;
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

bool comanMotionControl::getRotorPositionRaw(int j, double* value)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("getRotorPositionRaw");
}

bool comanMotionControl::getRotorPositionsRaw(double* value)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("getRotorPositionsRaw");
}

bool comanMotionControl::getRotorSpeedRaw(int j, double* value)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("getRotorSpeedRaw");
}

bool comanMotionControl::getRotorSpeedsRaw(double* value)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("getRotorSpeedsRaw");
}

bool comanMotionControl::getRotorAccelerationRaw(int j, double* value)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("getRotorAccelerationRaw");
}

bool comanMotionControl::getRotorAccelerationsRaw(double* value)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("getRotorAccelerationsRaw");
}

bool comanMotionControl::getJointPositionRaw(int j, double* value)
{
    yTrace();
    int16_t abs_pos;
    bool ret = true;
    McBoard *joint_p = getMCpointer(j);

    if( NULL == joint_p)
    {
        *value = 0;
        return false;
    }

    ts_bc_data_t bc_data;
    mc_bc_data_t &data = bc_data.raw_bc_data.mc_bc_data;
    if(bc_policy & BC_POLICY_ABSOLUTE_POSITION)  // se viene broadcastata... usare la ricezione udp
    {
        joint_p->get_bc_data(bc_data);
        ret = true;
    }
    
    ret = (!joint_p->getItem(GET_ABSOLUTE_POSITION, NULL, 0, REPLY_ABSOLUTE_POSITION, &abs_pos, sizeof(abs_pos)) );
    *value = (double) abs_pos;
    return ret;
}

bool comanMotionControl::getJointPositionsRaw        (double* value)
{
    yTrace();
    bool ret = true;
    for(int j=0; j<_njoints; j++)
        ret = ret && getJointPositionRaw(j, &value[j]);
    return ret;
}


// Limit interface
bool comanMotionControl::setLimitsRaw(int j, double min, double max)
{
    yTrace();
    bool ret = true;

    int16_t _min_pos = (int16_t) min;  // boards use int, we use double;
    int16_t _max_pos = (int16_t) max;

    McBoard *joint_p = getMCpointer(j);
    if( NULL == joint_p)
    {
        yError() << "Calling setLimitsRaw on a non-existing joint j" << j;
        return false;
    }

    ret &= (!joint_p->setItem(SET_MIN_POSITION, &_min_pos, sizeof(_min_pos)) );  // setItem returns 0 if ok, 2 if error
    ret &= (!joint_p->setItem(SET_MAX_POSITION, &_max_pos, sizeof(_max_pos)) );
    return ret;
}

bool comanMotionControl::getLimitsRaw(int j, double *min, double *max)
{
    yTrace();
    McBoard *joint_p = getMCpointer(j);
    bool ret = true;
    int _max_pos, _min_pos;  // boards use int, we use double;

    if( NULL == joint_p)
    {
        *min = 0;
        *max = 0;
        yError() << "Calling getLimitsRaw on a non-existing joint j" << j;
        return false;
    }

    ret &= (!joint_p->getItem(GET_MIN_POSITION,   NULL, 0, REPLY_MIN_POSITION, &_min_pos, sizeof(_min_pos)) );
    ret &= (!joint_p->getItem(GET_MAX_POSITION,   NULL, 0, REPLY_MAX_POSITION, &_max_pos, sizeof(_max_pos)) );

    yDebug() << "Max position " << _max_pos;
    yDebug() << "Min position " << _min_pos;

    *min = (double) _min_pos;
    *max = (double) _max_pos;
    return ret;
}

//
// Torque interface
//

bool comanMotionControl::getTorqueRaw(int j, double *t)
{
    McBoard *joint_p = getMCpointer(j);

    if( NULL == joint_p)
    {
        yError() << "Calling getTorqueRaw on a non-existing joint j" << j;
        *t = (double) j;   // return the joint number!!
        return false;
    }
    ts_bc_data_t bc_data;
    mc_bc_data_t &data = bc_data.raw_bc_data.mc_bc_data;

    joint_p->get_bc_data(bc_data);
    // la nuova versione generata da python non ha questo campo.
    // C'è un modo per verificare l'esistenza del campo e fa compilare cmq anche se il cmapo è assente??
    *t = (double) data.Torque  / _newtonsToSensor[j];
//    *t = 0;
    return true;
};

bool comanMotionControl::getTorquesRaw(double *t)
{
    yTrace();
    bool ret = true;
    for(int j=0; j<_njoints; j++)
        ret = ret && getTorqueRaw(j, &t[j]);
    return ret;
};

bool comanMotionControl::getBemfParamRaw(int j, double *bemf)
{
    return NOT_YET_IMPLEMENTED("getBemfParam");
}

bool comanMotionControl::setBemfParamRaw(int j, double bemf)
{
    return NOT_YET_IMPLEMENTED("getBemfParam");
}

bool comanMotionControl::getTorqueRangeRaw(int j, double *min, double *max)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("getTorqueRangeRaw");
};

bool comanMotionControl::getTorqueRangesRaw(double *min, double *max)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("getTorqueRangesRaw");
};

bool comanMotionControl::setRefTorquesRaw(const double *t)
{
    yTrace();
    int ret = true;
    for(int idx=0; idx<_njoints; idx++)
    {
        ret &= setRefTorqueRaw(idx, t[idx]);
    }
    return ret;
};

bool comanMotionControl::setRefTorqueRaw(int j, double t)
{
    uint8_t bId = jointTobId(j);
    _ref_torques[j] = (int16_t) t;

    UDPCommPacket pkt(SET_DESIRED_TORQUE);
    pkt.appendData((uint8_t*)_ref_torques, _njoints * sizeof(_ref_torques[0]) );
    bool ret = (!_boards_ctrl->set_torque_group(&bId, &_ref_torques[j], 1));
    return ret;
};

bool comanMotionControl::getRefTorqueRaw(int j, double *t)
{
    yTrace();
    McBoard *joint_p = getMCpointer(j);
    int16_t abs_pos;

    if( NULL == joint_p)
    {
        *t = 0;
        return false;
    }

    bool ret = (!joint_p->getItem(GET_DESIRED_TORQUE,   NULL, 0, REPLY_DESIRED_TORQUE, &abs_pos, sizeof(abs_pos)) );
    *t = (double) abs_pos;
    return ret;
}

bool comanMotionControl::getRefTorquesRaw(double *t)
{
    yTrace();
    bool ret = true;
    for(int j=0; j<_njoints; j++)
        ret = ret && getRefTorqueRaw(j, &t[j]);
    return ret;
};

bool comanMotionControl::setTorquePidRaw(int j, const Pid &pid)
{
    yTrace() << "joint " << j << "torque KP" << pid.kp;
    pid_gains_t p_i_d;
    McBoard *joint_p = getMCpointer(j);
    bool ret = true;

    if( NULL == joint_p)
    {
        yError() << "Calling Set torque Pid on a non-existing joint j" << j;
        return false;
    }
    else
    {
        p_i_d.p = (int32_t)pid.kp;
        p_i_d.i = (int32_t)pid.ki;
        p_i_d.d = (int32_t)pid.kd;
        p_i_d.gain_set = TORQUE_GAINS;

        int32_t _max_int = (int32_t) pid.max_int;
        int16_t start_off = (int16_t) pid.stiction_up_val;
        int16_t pid_off = (int16_t) pid.offset;
        int32_t scales[3];
        scales[0] = (int32_t) pid.scale;
        scales[1] = (int32_t) pid.scale;
        scales[2] = (int32_t) pid.scale;

        ret &= (!joint_p->setItem(SET_PID_GAINS, &p_i_d.gain_set, sizeof(p_i_d)) );  // setItem returns 0 if ok, 2 if error
        ret &= (!joint_p->setItem(SET_PID_GAIN_SCALE, &scales, 3 * sizeof(pid.scale) ) );  // setItem returns 0 if ok, 2 if error
        ret &= (!joint_p->setItem(SET_ILIM_GAIN, &(_max_int), sizeof(_max_int)) );  // setItem returns 0 if ok, 2 if error
        ret &= setOffsetRaw(j, (int16_t) pid_off);
    }
    return ret;
};

bool comanMotionControl::setTorquePidsRaw(const Pid *pids)
{
    bool ret = true;
    for(int j=0; j<_njoints; j++)
    {
        ret = ret && setTorquePidRaw(j, pids[j]);
    }
    return ret;
};

bool comanMotionControl::setTorqueErrorLimitRaw(int j, double limit)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("setTorqueErrorLimitRaw");
};

bool comanMotionControl::setTorqueErrorLimitsRaw(const double *limits)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("setTorqueErrorLimitsRaw");
};

bool comanMotionControl::getTorqueErrorRaw(int j, double *err)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("getTorqueErrorRaw");
};

bool comanMotionControl::getTorqueErrorsRaw(double *errs)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("getTorqueErrorsRaw");
};

bool comanMotionControl::getTorquePidOutputRaw(int j, double *out)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("getTorquePidOutputRaw");
};

bool comanMotionControl::getTorquePidOutputsRaw(double *outs)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("getTorquePidOutputsRaw");
};

bool comanMotionControl::getTorquePidRaw(int j, Pid *pid)
{
	yTrace();
	pid_gains_t ComanPid;
	ComanPid.p = ComanPid.i = ComanPid.d = 0;

	bool ret = true;
	McBoard *joint_p = getMCpointer(j);

	if( NULL == joint_p)
	{
		pid->kp = 0;
		pid->ki = 0;
		pid->kd = 0;
		pid->max_int = 0;
		pid->max_output = 0;
		pid->offset = 0;
		pid->scale = 0;
		pid->stiction_down_val = 0;
		pid->stiction_up_val = 0;
		pid->max_int = 0;
		yError() << "Calling GetPid on a non-existing joint j" << j;
		return false;
	}

	int32_t  integral_limit;
	int16_t  pid_offset;
	int32_t  scales[3];
	char pidType = TORQUE_GAINS;

	// get back PIDs from the boards  TCP ONLY
	ret &= (!joint_p->getItem(GET_PID_GAINS,      &pidType, 1, REPLY_PID_GAINS,       &ComanPid,        sizeof(ComanPid)) );
	ret &= (!joint_p->getItem(GET_PID_GAIN_SCALE, &pidType, 1, REPLY_PID_GAIN_SCALE,   scales,          sizeof(int32_t)*3) );
	ret &= (!joint_p->getItem(GET_ILIM_GAIN,      &pidType, 1, REPLY_ILIM_GAIN,       &integral_limit,  sizeof(integral_limit)) );
	ret &= (!joint_p->getItem(GET_PID_OFFSET,     NULL,     0, REPLY_PID_OFFSET,      &pid_offset,      sizeof(pid_offset)) );
	//     ret &= (!joint_p->getItem(GET_START_OFFSET,   NULL,     1, REPLY_START_OFFSET,    &pid_offset,      sizeof(ComanPid)) );

	if(ret)
	{
		pid->kp = (double) ComanPid.p;
		pid->ki = (double) ComanPid.i;
		pid->kd = (double) ComanPid.d;
		pid->scale = (double) scales[0];  // using just one of them
		pid->max_int = (double) integral_limit;
		pid->offset = ((double) pid_offset);// * mV2V;  ?
	}
	else
	{
		yError() << "get torque pid failed somehow";
	}
	return ret;
};

bool comanMotionControl::getTorquePidsRaw(Pid *pids)
{
    yTrace();
    bool ret = true;
    for(int j=0; j < _njoints; j++)
        ret = ret && getTorquePidRaw(j, &pids[j]);
    return ret;
};

bool comanMotionControl::getTorqueErrorLimitRaw(int j, double *limit)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("getTorqueErrorLimitRaw");
};

bool comanMotionControl::getTorqueErrorLimitsRaw(double *limits)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("getTorqueErrorLimitsRaw");
};

bool comanMotionControl::resetTorquePidRaw(int j)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("resetTorquePidRaw");
};

bool comanMotionControl::disableTorquePidRaw(int j)
{
    yTrace();
    
    return NOT_YET_IMPLEMENTED("disableTorquePidRaw");
};

bool comanMotionControl::enableTorquePidRaw(int j)
{    yTrace();
    return NOT_YET_IMPLEMENTED("enableTorquePidRaw");
};

bool comanMotionControl::setTorqueOffsetRaw(int j, double v)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("setTorqueOffsetRaw");
};

bool comanMotionControl::setVelLimitsRaw(int j, double min, double max)
{
    McBoard *joint_p = getMCpointer(j);
    int16_t velMax = (int16_t) max;
    int16_t velMin = (int16_t) min;
    bool ret = true;
    ret = (!joint_p->setItem(SET_MAX_VELOCITY, &velMax, sizeof(velMax)) ) && ret;
    ret = (!joint_p->setItem(SET_MIN_VELOCITY, &velMax, sizeof(velMin)) ) && ret;
    return true;
}

bool comanMotionControl::getVelLimitsRaw(int j, double *min, double *max)
{
    McBoard *joint_p = getMCpointer(j);
    int16_t velMax;
    int16_t velMin;
    bool ret = (!joint_p->setItem(GET_MAX_VELOCITY, &velMax, sizeof(velMax)) );
    ret = (!joint_p->setItem(GET_MIN_VELOCITY, &velMax, sizeof(velMin)) ) && ret;
    *min = (double) velMin;
    *max = (double) velMax;
    return true;
}


// PositionDirect Interface
bool comanMotionControl::setPositionRaw(int j, double ref)
{
    // needs to send both position and velocity as well as positionMove
//    yTrace();
    uint8_t bId = jointTobId(j);
    _ref_positions[j] = (int32_t) ref;

    return (!_boards_ctrl->set_position_velocity_group(&bId, &_ref_positions[j], &_ref_speeds[j], 1) );  // 1 is the number of joints
}

// needs to send both position and velocity as well as positionMove
bool comanMotionControl::setPositionsRaw(const int n_joint, const int *joints, double *refs)
{
    int16_t *tmp_sp   = new int16_t [n_joint];
    int32_t *tmp_pos  = new int32_t [n_joint];
    uint8_t *tmp_bIds = new uint8_t [n_joint];

    for(int i=0; i<n_joint; i++)
    {
        tmp_bIds[i] = jointTobId(joints[i]);
        _ref_positions[joints[i]] = (int32_t) refs[i];
        tmp_sp[i]  = _ref_speeds[joints[i]];
        tmp_pos[i] = _ref_positions[joints[i]];
    }
    int ret = (!_boards_ctrl->set_position_velocity_group(tmp_bIds, tmp_pos, tmp_sp, n_joint) );
    delete [] tmp_sp;
    delete [] tmp_pos;
    delete [] tmp_bIds;
    return ret;
}

// needs to send both position and velocity as well as positionMove
bool comanMotionControl::setPositionsRaw(const double *refs)
{
    for(int i=0; i<_njoints; i++)
        _ref_positions[i] = (int32_t) refs[i];

    // here the function wants the num of elements (joints), contiene internamente un loop su tutte le schede
    return (!_boards_ctrl->set_position_velocity(_ref_positions, _ref_speeds, _njoints) );
}

bool comanMotionControl::getImpedanceRaw(int j, double *stiffness, double *damping)
{
    return NOT_YET_IMPLEMENTED("getImpedanceOffsetRaw");
}

bool comanMotionControl::setImpedanceRaw(int j, double stiffness, double damping)
{
    uint8_t bId = jointTobId(j);
    int tmp_stiff = (int) stiffness;
    int tmp_damp  = (int) damping;

    if(stiffness < 5 *_newtonsToSensor[j]/_angleToEncoder[j])
    {
        yWarning() << "------------------------------------------------------";
        yWarning() << " Value of stiffnes " << "stiffness is below threshold ";
        yWarning() << "------------------------------------------------------";
    }

    bool ret = (!_boards_ctrl->set_stiffness_damping_group(&bId, &tmp_stiff, &tmp_damp, 1));

    if(!ret)
    {
        cout << "ERROR in setImpedanceRaw for joint " << j;
    }
    return ret;
}

bool comanMotionControl::setImpedanceOffsetRaw(int j, double offset)
{
    // use SET_GRAVITY_COMPENSATION_GROUP when available;
    uint8_t bId = jointTobId(j);
    return _comanHandler->setGravityOffset(bId, (int)offset);
}

bool comanMotionControl::getImpedanceOffsetRaw(int j, double* offset)
{
    // Command not present in firmware
    return NOT_YET_IMPLEMENTED("getImpedanceOffsetRaw");
}

bool comanMotionControl::getCurrentImpedanceLimitRaw(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp)
{
    // Command not present in firmware
    return NOT_YET_IMPLEMENTED("getCurrentImpedanceLimitRaw");
}
