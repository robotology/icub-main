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

#include "comanDevicesHandler.hpp"
#include "comanMotionControl.h"
#include "Debug.h"

#undef yDebug()
#define yDebug() cout

#undef yWarning()
#define yWarning() cout

#undef yError()
#define yError() cout

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
double comanMotionControl::convertDoble2Int(double in[], int out[])
{
    for(int idx=0; idx<_njoints; idx++)
        out[idx] = (int) in[idx];
}

double comanMotionControl::convertDoble2Short(double in[], short int out[])
{
    for(int idx=0; idx<_njoints; idx++)
        out[idx] = (short int) in[idx];
}

inline McBoard * comanMotionControl::getMCpointer(int j)
{
    return _mcs[j];
}

inline int comanMotionControl::bId2Idx(int j)
{
    return (j-1);
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
        //yError () << key1.c_str() << " incorrect number of entries in board " << _fId.name << '[' << _fId.boardNum << ']';
        return false;
    }

    out=tmp;
    return true;
}


bool comanMotionControl::alloc(int nj)
{
    _axisMap = allocAndCheck<int>(nj);
    _angleToEncoder = allocAndCheck<double>(nj);
//     _encoderconversionoffset = <float>(nj);
//     _encoderconversionfactor = allocAndCheck<float>(nj);

    _rotToEncoder = allocAndCheck<double>(nj);
    _zeros = allocAndCheck<double>(nj);
    _torqueSensorId= allocAndCheck<int>(nj);
    _torqueSensorChan= allocAndCheck<int>(nj);
    _maxTorque=allocAndCheck<double>(nj);
    _newtonsToSensor=allocAndCheck<double>(nj);

    _pids=allocAndCheck<Pid>(nj);
    _tpids=allocAndCheck<Pid>(nj);

    _limitsMax=allocAndCheck<double>(nj);
    _limitsMin=allocAndCheck<double>(nj);
    _currentLimits=allocAndCheck<double>(nj);
    checking_motiondone=allocAndCheck<bool>(nj);

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

    _controlMode = allocAndCheck<int>(nj);

    return true;
}

comanMotionControl::comanMotionControl() :
    ImplementControlCalibration2<comanMotionControl, IControlCalibration2>(this),
    ImplementAmplifierControl<comanMotionControl, IAmplifierControl>(this),
    ImplementPidControl<comanMotionControl, IPidControl>(this),
    ImplementEncodersTimed(this),
    ImplementPositionControl2(this),
//    ImplementVelocityControl<comanMotionControl, IVelocityControl>(this),
    ImplementVelocityControl2(this),
    ImplementControlMode(this),
    ImplementDebugInterface(this),
    ImplementControlLimits<comanMotionControl, IControlLimits>(this),
    _mutex(1)
{
    yTrace();
    _boards_ctrl = NULL;

    _pids			= NULL;
    _tpids			= NULL;
//    0 	= 0;

    _tpidsEnabled	= false;
    _njoints 		= 0;

    _axisMap		= NULL;
    _angleToEncoder = NULL;
    _zeros			= NULL;
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
    _pid_offset = NULL;
    checking_motiondone = NULL;
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
}

bool comanMotionControl::open(yarp::os::Searchable &config)
{
    yTrace();

    fromConfig(config);
    // initialize control type to idle
    for(int i=0; i< _njoints; i++)
    {
        _controlMode[i] = VOCAB_CM_IDLE;
    }

/* TODO ??
    // per tutti i giunti... non fare niente. Le schede partono già con i parametri corretti. Chiedere cosa è necessario fare all'avvio del robot
    for(int i=0; i< _njoints; i++)
    {
        int j = _axisMap[i];
        int16_t _min_pos = (int16_t) min;  // boards use int, we use double;
        int16_t _max_pos = (int16_t) max;

        McBoard *joint_p = getMCpointer(j);
        if( NULL == joint_p)
        {
            yError() << "Calling setLimitsRaw on a non-existing joint j" << j;
            return false;
        }

        // set limits for pos, vel torque
        ret &= (!joint_p->setItem(SET_MIN_POSITION, &_min_pos, sizeof(_min_pos)) );  // setItem returns 0 if ok, 2 if error
        ret &= (!joint_p->setItem(SET_MAX_POSITION, &_max_pos, sizeof(_max_pos)) );
        ret &= (!joint_p->setItem(SET_MIN_VELOCITY, &        , sizeof(        )) );
        ret &= (!joint_p->setItem(SET_MAX_VELOCITY, &        , sizeof(        )) );
        ret &= (!joint_p->setItem(SET_MIN_TORQUE,   &        , sizeof(        )) );
        ret &= (!joint_p->setItem(SET_MAX_TORQUE,   &        , sizeof(        )) );

        // set broadcast policy
        ret &= (!joint_p->setItem(SET_BROADCAST_RATE,     &        , sizeof(        )) );
        ret &= (!joint_p->setItem(SET_BROADCAST_POLICY,   &        , sizeof(        )) );
    }
*/
//
//      INIT ALL INTERFACES
//
    ImplementControlCalibration2<comanMotionControl, IControlCalibration2>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementAmplifierControl<comanMotionControl, IAmplifierControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementEncodersTimed::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
//     ImplementPositionControl<comanMotionControl, IPositionControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementPositionControl2::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementPidControl<comanMotionControl, IPidControl>:: initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementControlMode::initialize(_njoints, _axisMap);
//    ImplementVelocityControl<comanMotionControl, IVelocityControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementVelocityControl2::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementDebugInterface::initialize(_njoints, _axisMap, _angleToEncoder, _zeros, _rotToEncoder);
    ImplementControlLimits<comanMotionControl, IControlLimits>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);

    _comanHandler = comanDevicesHandler::instance();

    if(_comanHandler == NULL)
    {
        yError() << "unable to create a new Coman Handler class!";
        return false;
    }

    _comanHandler->open(config);
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

    for(int i=0; i< _njoints; i++)
    {
        _controlMode[i] = VOCAB_CM_POSITION;
    }
    return true;
}

bool comanMotionControl::fromConfig(yarp::os::Searchable &config)
{
    yError() << config.toString().c_str();
    Bottle xtmp;
    int i;
    Bottle general = config.findGroup("GENERAL");

    _njoints = general.find("Joints").asInt();
    alloc(_njoints);

    // leggere i valori da file
    if (!extractGroup(general, xtmp, "AxisMap", "a list of reordered indices for the axes", _njoints+1))
        return false;

    for (i = 1; i < xtmp.size(); i++)
        _axisMap[i-1] = xtmp.get(i).asInt();

    // Encoder scales
    if (!extractGroup(general, xtmp, "Encoder", "a list of scales for the encoders", _njoints+1))
        return false;
    else
        for (i = 1; i < xtmp.size(); i++)
            _angleToEncoder[i-1] = xtmp.get(i).asDouble();

    // Zero Values
    if (!extractGroup(general, xtmp, "Zeros","a list of offsets for the zero point", _njoints+1))
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

    printf("bc policy    = %d (0x%0x)", bc_policy, bc_policy);
    printf("extra_policy = %d (0x%0x)", extra_policy, extra_policy);
    return true;
}

bool comanMotionControl::init()
{

//   //Do some init for each board if needed... maybe check firmware version??
//     McBoard *joint_p = getMCpointer(0);
//     uint8_t boardId = 0x03;
//     if(NULL != joint_p)
//     {
//         if(0 != joint_p->setItem(SET_BOARD_NUMBER, &boardId, sizeof(uint8_t)))
//         {
//             yError() << "setting boardId";
//         }
//
//         if(0 != joint_p->setItem(SAVE_PARAMS_TO_FLASH, NULL, 0 ))
//         {
//             yError() << "saving boardId into flash memory";
//         }
//     }

//     _boards_ctrl->body_homing(_ref_positions, _ref_speeds, _ref_torques);  // calibratore
    // MUST be in this order: velocity, position and torque??
    _boards_ctrl->set_velocity(_ref_speeds, _njoints * sizeof(_ref_speeds[0]));
    _boards_ctrl->set_position(_ref_positions, _njoints * sizeof(_ref_positions[0]));
    _boards_ctrl->set_torque(_ref_torques, _njoints * sizeof(_ref_torques[0]));

    // test settings
    _boards_ctrl->test();
    // ... WAIT  to let dsp thinking .... LEAVE HERE
    sleep(1);

    uint8_t start = 1;
    _boards_ctrl->start_stop_control(start);

   // tell to ALL dps to start broadcast data
    _boards_ctrl->start_stop_bc_boards(start);
    return true;
}

bool comanMotionControl::close()
{
    yTrace();
    _comanHandler->close();

    if(_boards_ctrl != NULL)
        delete _boards_ctrl;

    ImplementControlCalibration2<comanMotionControl, IControlCalibration2>::uninitialize();
    ImplementAmplifierControl<comanMotionControl, IAmplifierControl>::uninitialize();
    ImplementEncodersTimed::uninitialize();
    ImplementPositionControl2::uninitialize();
    ImplementPidControl<comanMotionControl, IPidControl>::uninitialize();
    ImplementControlMode::uninitialize();
//    ImplementVelocityControl<comanMotionControl, IVelocityControl>::uninitialize();
    ImplementVelocityControl2::uninitialize();
    ImplementDebugInterface::uninitialize();
    ImplementControlLimits<comanMotionControl, IControlLimits>::uninitialize();
    return true;
}

///////////// PID INTERFACE

bool comanMotionControl::setPidRaw(int j, const Pid &pid)
{
    yTrace() << "joint " << j << "KP" << pid.kp;
    pid_gains_t p_i_d;
    McBoard *joint_p = getMCpointer(j);
    bool ret = true;

    if( NULL == joint_p)
    {
        yError() << "Calling SetPid on a non-existing joint j" << j;
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

        ret &= (!joint_p->setItem(SET_PID_GAINS, &p_i_d.gain_set, sizeof(p_i_d)) );  // setItem returns 0 if ok, 2 if error
        ret &= (!joint_p->setItem(SET_PID_GAIN_SCALE, &scales, 3*sizeof(int32_t) ) );  // setItem returns 0 if ok, 2 if error
        ret &= (!joint_p->setItem(SET_ILIM_GAIN, &(_max_int), sizeof(pid.scale)) );  // setItem returns 0 if ok, 2 if error
        ret &= setOffsetRaw(j, (int16_t) pid_off);
    }
    return ret;
}

bool comanMotionControl::setPidsRaw(const Pid *pids)
{
    yTrace();
    bool ret = true;

    for(int bId=1; bId<=_njoints; bId++)
        ret = ret && setPidRaw(bId, pids[bId]);
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
    for(int bId=1; bId <= _njoints; bId++)
    {
        ret = ret && getOutputRaw(bId, &outs[bId]);
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

    pid->kp = (double) ComanPid.p;
    pid->ki = (double) ComanPid.i;
    pid->kd = (double) ComanPid.d;
    pid->scale = (double) scales[0];  // using just one of them
    pid->max_int = (double) integral_limit;
    pid->offset = ((double) pid_offset);// * mV2V;  ?

    yDebug() << "Coman POS pid kp " << ComanPid.p << "ki "<< ComanPid.i << "kd " << ComanPid.d;
    yDebug() << "Coman integral limit " << integral_limit << ", pid offset " << pid_offset;
    return ret;
}

bool comanMotionControl::getPidsRaw(Pid *pids)
{
    yTrace();
    bool ret = true;
    for(int bId=1; bId <= _njoints; bId++)
        ret = ret && getPidRaw(bId, &pids[bId]);
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
    for(int bId=1; bId<=_njoints; bId++)
        ret = ret && getReferenceRaw(bId, &refs[bId]);
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

    // j is the bId, stop = 0 -> off
    ret = ret && (!_boards_ctrl->start_stop_single_control(j, stop, POSITION_MOVE));
    ret = ret && (!_boards_ctrl->start_stop_single_control(j, stop, VELOCITY_MOVE));
    ret = ret && (!_boards_ctrl->start_stop_single_control(j, stop, TORQUE_MOVE));

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
    int idx = bId2Idx(j);

    _pid_offset[idx] = (uint16_t) v;
    return !_boards_ctrl->set_pid_offset(_pid_offset, sizeof(uint16_t) * _njoints);
}


////////////////////////////////////////
//    Velocity PID
////////////////////////////////////////

bool comanMotionControl::setVelPidRaw(int j, const Pid &pid)
{

    yTrace() << "board " << j << "vel KP" << pid.kp;
    pid_gains_t p_i_d;
    McBoard *joint_p = getMCpointer(j);
    bool ret = true;

    if( NULL == joint_p)
    {
        yError() << "Calling SetPid on a non-existing board" << j;
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
    for(int bId=1; bId <= _njoints; bId++)
        ret = ret && setVelPidRaw(bId, pids[bId]);
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

    yDebug() << "Coman VEL pid kp " << ComanPid.p << "ki "<< ComanPid.i << "kd " << ComanPid.d;
    yDebug() << "Coman integral limit " << integral_limit << ", pid offset " << pid_offset;
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
    for(int j=1; j<= _njoints; j++)
    {
        // TCP ONLY
        ret &= getVelPidRaw(j, &pids[j]);
    }
    return ret;
}

bool comanMotionControl::getVelErrorRaw(int j, double *err)
{
    yTrace();
    return getErrorRaw(j, err);
}

bool comanMotionControl::getVelErrorsRaw(const int n_joint, const int *joints, double *errs)
{
    yTrace();
    bool ret = true;
    for(int j=0; j< n_joint; j++)
    {
        // TCP ONLY
        ret &= getVelErrorRaw(joints[j], &errs[j]);
    }
    return ret;
}

bool comanMotionControl::getVelErrorsRaw(double *errs)
{
    yTrace();
    return getErrorsRaw(errs);
}


////////////////////////////////////////
//    Velocity control interface raw  //
////////////////////////////////////////

bool comanMotionControl::setVelocityModeRaw(int j)
{
    yTrace();
    bool ret = true;
    int idx = bId2Idx(j);
    McBoard *joint_p = getMCpointer(j);

    if( NULL == joint_p)
    {
        yError() << "Calling setVelocityModeRaw on a non-existing joint j" << j;
        return false;
    }

    if(_controlMode[idx] == VOCAB_CM_VELOCITY)
    {
        // nothing to do here
        {yDebug() << "joint "<< j << "already in velocity mode" << endl;}
        ret = true;
    }
    else
    {
        ret = (!_boards_ctrl->start_stop_single_control(j, 1, VELOCITY_MOVE));
    }

    if(ret)
        _controlMode[idx] = VOCAB_CM_VELOCITY;
    else
        yError() << "in setVelocityModeRaw for joint " << j;
    return ret;
}

bool comanMotionControl::setVelocityModeRaw(const int n_joint, const int *joints)
{
    yTrace();
    int start = 0x03;
    std::vector<int> board_set;
    board_set.resize(n_joint);
    for(int idx=0; idx < n_joint; idx++)
    {
        board_set[idx] = joints[idx];
    }
    return (!_boards_ctrl->start_stop_set_control(board_set, start, VELOCITY_MOVE));
}

bool comanMotionControl::setVelocityModeRaw()
{
    return (!_boards_ctrl->start_stop_control(1, VELOCITY_MOVE) );
}

bool comanMotionControl::velocityMoveRaw(int j, double sp)
{
    // Here I count on j be a reasonable number, i.e. between 1 and _njoints. j=0 is NOT correct!!!
    int idx = bId2Idx(j);
    uint8_t bId = (uint8_t) j;
    int16_t des_vel = (int16_t)(sp/COMAN_POS_TO_VEL_GAIN);   // differenza tra la unità di misura usata in velocità e posizione;

    bool ret = (!_boards_ctrl->set_velocity_group(&bId, &des_vel, 1));

    if(ret)
    {
        _ref_speeds[idx] = des_vel;   // differenza tra la unità di misura usata in velocità e posizione;;
    }
    else
    {
        cout << "ERROR in velocityMoveRaw for joint " << j << endl;
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
        bIds[i] = (uint8_t) joints[i];
    }

    bool ret = (!_boards_ctrl->set_velocity_group(bIds, des_vel, n_joint));

    if(ret)
    {
        // cache value for future use
        for(int i=0; i<n_joint; i++)
        {
            _ref_speeds[joints[i]] = des_vel[i];
        }
    }
    else
    {
        cout << "ERROR in velocityMoveRaw for set of joints " << endl;
    }
    return ret;
}

bool comanMotionControl::velocityMoveRaw(const double *spds)
{
    bool ret = true;
    int16_t tmp_spds[_njoints];

    for(int bId=1; bId <= _njoints; bId++)
    {
        int idx = bId2Idx(bId);
        tmp_spds[idx] = (int16_t)(spds[idx]/COMAN_POS_TO_VEL_GAIN);   // differenza tra la unità di misura usata in velocità e posizione;
    }

    ret = ret && (!_boards_ctrl->set_velocity(tmp_spds, _njoints * sizeof(int16_t)));

    if(ret)
    {
        for(int bId=1; bId <= _njoints; bId++)
        {
            int idx = bId2Idx(bId);
            _ref_speeds[idx] = tmp_spds[idx];
        }
    }
    else
    {
        cout << "ERROR in velocityMoveRaw for all joint " << endl;
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

}

bool comanMotionControl::positionMoveRaw(int j, double ref)
{
    yTrace();
    int idx = bId2Idx(j);
    uint8_t bId = (uint8_t) j;

    _ref_positions[idx] = (int32_t) ref;


    return (!_boards_ctrl->set_position_velocity_group(&bId, &_ref_positions[idx], &_ref_speeds[idx], 1) );
    // here the function wants the num of elements, contiene internamente un loop su tutte le schede
    //return (!_boards_ctrl->set_position_velocity(_ref_positions, _ref_speeds, _njoints) );
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
    bool ret = getEncoderRaw(j, &actual_pos);

    delta = ((double) _ref_positions[j]) - actual_pos;

    if(fabs(delta <= COMAN_POS_THRESHOLD))
        *flag = true;
    else
        *flag = false;

    return ret;
}

bool comanMotionControl::setRefSpeedRaw(int j, double sp)
{
    yTrace();
    int idx = bId2Idx(j);
    _ref_speeds[idx] = (int16_t) (sp/COMAN_POS_TO_VEL_GAIN);  // differenza tra la unità di misura usata in velocità e posizione

    bool ret = (!_boards_ctrl->set_velocity(_ref_speeds, _njoints * sizeof(int16_t)) );

    if(!ret)
    {
       cout << "ERROR in setRefSpeedRaw for joint " << j << endl;
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
//         yError() non funziona??!?!?!?
        cout << "ERROR in setRefSpeedsRaw for all joint " << endl;
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
    for(int idx=0; idx<n_joint; idx++)
    {
        joint_p = getMCpointer(idx);
        int16_t tmp_acc = (int16_t) accs[idx];

        if(NULL != joint_p)
            ret = ret && (!joint_p->setItem(SET_ACCELERATION, &tmp_acc, sizeof(int16_t)));   // setItem returns 0 if ok, 2 if error
        else
        {
            yError() << "Trying to use setRefAccelerationsRaw on a non existing joint" << idx;
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
    for(int idx=1; idx <= _njoints; idx++)
    {
        joint_p = getMCpointer(idx);
        int16_t tmp_acc = (int16_t) accs[idx];

        if(NULL != joint_p)
            ret = ret && (!joint_p->setItem(SET_ACCELERATION, &tmp_acc, sizeof(int16_t)));   // setItem returns 0 if ok, 2 if error
        else
        {
            yError() << "Trying to use setRefAccelerationsRaw on a non existing joint" << idx;
            ret = false;
        }
    }
    return ret;
}

bool comanMotionControl::getRefSpeedRaw(int j, double *spd)
{
    yTrace();
    int idx = bId2Idx(j);
    *spd = (double) _ref_speeds[idx] * COMAN_POS_TO_VEL_GAIN;
    return true;

/*    McBoard *joint_p = getMCpointer(j);
    int16_t ref_speed;
    if( NULL == joint_p)
    {
        *spd = 0;
        yError() << "Calling getRefSpeedRaw on a non-existing joint j" << j;
        return false;
    }
    bool ret = (!joint_p->getItem(GET_DESIRED_VELOCITY,   NULL, 0, REPLY_DESIRED_VELOCITY, &ref_speed, sizeof(ref_speed)) );
    *spd = (double) ref_speed;
    return ret;
*/
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
        joint_p = getMCpointer(idx);

        if(NULL == joint_p)
        {
            yError() << "Trying to use getRefAccelerationsRaw on a non existing joint" << idx;
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

    for(int bId=1; bId <= _njoints; bId++)
    {
        joint_p = getMCpointer(bId);
        idx = bId2Idx(bId);

        if(NULL == joint_p)
        {
            yError() << "Trying to use getRefAccelerationsRaw on a non existing board" << bId;
            ret = false;
        }
        else
        {
            // TCP ONLY
            ret = ret && (!joint_p->getItem(GET_ACCELERATION, NULL, 0, REPLY_ACCELERATION, &tmp_acc, sizeof(int16_t)));   // setItem returns 0 if ok, 2 if error
            accs[idx] = (double) tmp_acc;
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

    switch(_controlMode[j])
    {
        case VOCAB_CM_POSITION:
            ret = !_boards_ctrl->start_stop_single_control((uint8_t) j, stop, POSITION_MOVE);
            break;

        case VOCAB_CM_VELOCITY:
            ret = !_boards_ctrl->start_stop_single_control((uint8_t) j, stop, VELOCITY_MOVE);
            break;

        case VOCAB_CM_TORQUE:
            ret = !_boards_ctrl->start_stop_single_control((uint8_t) j, stop, TORQUE_MOVE);
            break;

        default:
            yError() << "Calling stop from an unknown control mode (joint" << j << ")... stopping everything!";
            ret = ret && (!_boards_ctrl->start_stop_single_control((uint8_t) j, stop, POSITION_MOVE));
            ret = ret && (!_boards_ctrl->start_stop_single_control((uint8_t) j, stop, VELOCITY_MOVE));
            ret = ret && (!_boards_ctrl->start_stop_single_control((uint8_t) j, stop, TORQUE_MOVE));
            ret = false;
            break;
    }
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
        board_set[idx] = joints[idx];
    }

    ret = ret && (!_boards_ctrl->start_stop_set_control(board_set, stop, POSITION_MOVE));
    ret = ret && (!_boards_ctrl->start_stop_set_control(board_set, stop, VELOCITY_MOVE));
    ret = ret && (!_boards_ctrl->start_stop_set_control(board_set, stop, TORQUE_MOVE));

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

    for(int idx=0; idx < n_joint; idx++)
    {
        board_set[idx] = joints[idx];
    }
    return (!_boards_ctrl->start_stop_set_control(board_set, start, POSITION_MOVE));
}

bool comanMotionControl::positionMoveRaw(const int n_joint, const int *joints, const double *refs)
{
    int16_t *tmp_sp   = new int16_t [n_joint];
    int32_t *tmp_pos  = new int32_t [n_joint];
    uint8_t *tmp_bIds = new uint8_t [n_joint];

    for(int i=0; i<n_joint; i++)
    {
        tmp_bIds[i] = joints[i];
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
        tmp_bId[idx] = (uint8_t) joints[idx];
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
    int idx = bId2Idx(j);
    McBoard *joint_p = getMCpointer(j);
    uint8_t stop = 0;
    bool ret = true;
    int vel2pos_cycle = 0;

    if( NULL == joint_p)
    {
        yError() << "Calling setPositionModeRaw on a non-existing joint j" << j;
        return false;
    }

    // stop what is running
    switch(_controlMode[j])
    {
        case VOCAB_CM_POSITION:
            yDebug() << "joint "<< j << "already in position mode" << endl;
            return true;
            break;

        case VOCAB_CM_VELOCITY:
            // now firmware is able to bo a smooth transition from pos to vel, so nothing to do here
            break;

        case VOCAB_CM_TORQUE:
            yDebug() << "joint "<< j << "stopping torque mode";
            ret = ret && (!joint_p->setItem(SET_TORQUE_ON_OFF, &stop, sizeof(stop)));   // setItem returns 0 if ok, 2 if error

            if(!ret)
                yError() << "error while stopping torque mode" << endl;
            break;

        default:
            yDebug() << "joint "<< j << "setvelocity mode coming from unknown controlmode... stop everything and then enable position\n" << endl;
            stopRaw();
            break;
    }
    ret = ret && (!_boards_ctrl->start_stop_single_control(j, 1, POSITION_MOVE));      // j+1 per solita differenza di corrispondenze tra bId e n' giunto, 1 = ON

    if(ret)
        _controlMode[idx] = VOCAB_CM_POSITION;
    else
        yError() << "while setting position mode for joint " << j;
    return ret;
}



bool comanMotionControl::setTorqueModeRaw(int j)
{
    yTrace();
    uint8_t stop = 0;
    int idx = bId2Idx(j);
    McBoard *joint_p = getMCpointer(j);

    if( NULL == joint_p)
    {
        yError() << "Calling setTorqueModeRaw on a non-existing joint j" << j;
        return false;
    }
    uint8_t trq_on = 4;  //1 for torque control    0 for position control
    bool ret = true;

    ret = ret && (!_boards_ctrl->start_stop_single_control(j, stop, POSITION_MOVE));
    ret = ret && (!_boards_ctrl->start_stop_single_control(j, stop, VELOCITY_MOVE));

/*  // stop what is running
    if(0)
    switch(_controlMode[j])
    {
        case VOCAB_CM_POSITION:
            case VOCAB_CM_VELOCITY:
            cout << "joint "<< j << "stopping position mode";
            ret = ret && (!boards_ctrl->start_stop_single_control(j, stop, POSITION_MOVE));

            if(!ret)
                cout << "error while stopping position mode";
//             break;
//
//         case VOCAB_CM_VELOCITY:
            cout << "joint "<< j << "stopping velocity mode";
            ret = ret && (!boards_ctrl->start_stop_single_control(j, stop, VELOCITY_MOVE));

            if(!ret)
                cout << "error while stopping velocity mode";
            break;

        case VOCAB_CM_TORQUE:
            cout << "joint "<< j << "already in torque mode";
            //return true;
            break;

        default:
                cout << "joint "<< j << "TODO setvelocity mode coming from un-handled case... stop something here!!\n";
                break;
    }


//     ret = (!joint_p->getItem(GET_TORQUE_ON_OFF, NULL, 0, REPLY_TORQUE_ON_OFF, &trq_on, sizeof(trq_on)));

    if(!ret)
        yError() << "while getting torque status";
    else
        yError() << "torque control is " << (trq_on? "enabled\n" : "disabled\n");
*/

    trq_on = 1;
    ret = ret && (!joint_p->setItem(SET_TORQUE_ON_OFF, &trq_on, sizeof(trq_on)) );

    if(ret)
        _controlMode[idx] = VOCAB_CM_TORQUE;
    return (ret);
}

bool comanMotionControl::setTorqueModeRaw( )
{
    yTrace();
    return NOT_YET_IMPLEMENTED("setTorqueModeRaw all joints");
}

bool comanMotionControl::setImpedancePositionModeRaw(int j)
{
    // Chiedere info
    McBoard *joint_p = getMCpointer(j);

    if( NULL == joint_p)
    {
        yError() << "Calling setImpedancePositionModeRaw on a non-existing joint j" << j;
        return false;
    }

    bool ret = true;
    uint16_t motor_config_mask = 0;
    //
    uint8_t trq_on = 1;  //1 for torque control    0 for position control
//     ret &= (!joint_p->setItem(SET_TORQUE_ON_OFF, &trq_on, sizeof(trq_on)) );
    ret &= (!joint_p->getItem(GET_MOTOR_CONFIG, NULL, 0, REPLY_MOTOR_CONFIG, &motor_config_mask, 2) );
    printf("joint %d got motor config 0x%0X", j, motor_config_mask);

    motor_config_mask |= 0x4000;   // add bit about impedance control.

    ret &= (!joint_p->setItem(SET_MOTOR_CONFIG, &motor_config_mask, sizeof(motor_config_mask)) );
    printf("joint %d got motor config 0x%0X", j, motor_config_mask);

    uint16_t motor_config_mask2 = 0x1;       //0 Moving Average 1 ButterWorth 2 Least Square 3 Jerry Pratt -- TODO cambiano a runtime/configurazione??

    ret &= (!joint_p->setItem(SET_MOTOR_CONFIG2, &motor_config_mask2, sizeof(motor_config_mask2)) );
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
    int idx = bId2Idx(j);

    // cached value must be correct!!
    *v = _controlMode[idx];
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
    *spd = (double) data.Velocity;

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
    double tmp;
    for(int i=0; i<_njoints; i++)
    {
        getEncoderTimedRaw(i, &tmp, &stamps[i]);
        encs[i] = tmp;
    }
}

bool comanMotionControl::getEncoderTimedRaw(int j, double *enc, double *stamp)
{
    McBoard *joint_p = getMCpointer(j);

    if( NULL == joint_p)
    {
//         yError() << "Calling getEncoderTimedRaw on a non-existing joint j" << j;
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
    return NOT_YET_IMPLEMENTED("disableAmpRaw");
}

bool comanMotionControl::getCurrentRaw(int j, double *curr)
{
    McBoard *joint_p = getMCpointer(j);

    if( NULL == joint_p)
    {
//         yError() << "Calling getEncoderTimedRaw on a non-existing joint j" << j;
        *curr = j;   // return the joint number!!
        return false;
    }
    // viene probabilmente broadcastata... usare la ricezione udp per questo.
    ts_bc_data_t bc_data;
    mc_bc_data_t &data = bc_data.raw_bc_data.mc_bc_data;
#warning "this implies a memcopy!! To be optimized!! And add timestamp"
    joint_p->get_bc_data(bc_data);
    *curr = (double) data.Current;

    return true;
}

bool comanMotionControl::getCurrentsRaw(double *vals)
{
    double tmp;
    for(int i=0; i<_njoints; i++)
    {
        getCurrentRaw(i, &tmp);
        vals[i] = tmp;
    }
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
        ret &= getJointPositionRaw(j, &value[j]);
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
// Toque interface
//

bool comanMotionControl::getTorqueRaw(int j, double *t)
{
    McBoard *joint_p = getMCpointer(j);

    if( NULL == joint_p)
    {
        yError() << "Calling getTorqueRaw on a non-existing joint j" << j;
        *t = j;   // return the joint number!!
        return false;
    }
    ts_bc_data_t bc_data;
    mc_bc_data_t &data = bc_data.raw_bc_data.mc_bc_data;

    joint_p->get_bc_data(bc_data);
    // la nuova versione generata da python non ha questo campo.
    // C'è un modo per verificare l'esistenza del campo e fa compilare cmq anche se il cmapo è assente??
//     *t = (double) data.Torque;
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
    yTrace();
    _ref_torques[j] = t;

    UDPCommPacket pkt(SET_DESIRED_TORQUE);
    pkt.appendData((uint8_t*)_ref_torques, _njoints * sizeof(_ref_torques[0]) );
#warning "udp_sock is private, create a public Board_crtl method for setting offset?"
//         int ret = pkt.sendToUDPSocket(_boards_ctrl->udp_sock, (sockaddr *)& (boards_ctrl->dest_addr), sizeof(boards_ctrl->dest_addr));
    return true;
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
        ret &=getRefTorqueRaw(j, &t[j]);
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
        yError() << "Calling SetPid on a non-existing joint j" << j;
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
        ret &= (!joint_p->setItem(SET_PID_GAIN_SCALE, &scales, 3 * sizeof(int32_t) ) );  // setItem returns 0 if ok, 2 if error
        ret &= (!joint_p->setItem(SET_ILIM_GAIN, &(_max_int), sizeof(pid.scale)) );  // setItem returns 0 if ok, 2 if error
        ret &= setOffsetRaw(j, (int16_t) pid_off);
    }
    return ret;
};

bool comanMotionControl::setTorquePidsRaw(const Pid *pids)
{
    bool ret = true;
    for(int bId=1; bId<=_njoints; bId++)
    {
        ret = ret && setTorquePidRaw(bId, pids[bId]);
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
    return NOT_YET_IMPLEMENTED("getTorquePidRaw");
};

bool comanMotionControl::getTorquePidsRaw(Pid *pids)
{
    yTrace();
    return NOT_YET_IMPLEMENTED("getTorquePidsRaw");
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
