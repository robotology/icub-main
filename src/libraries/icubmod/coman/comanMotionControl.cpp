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

// Pid is in icub fashion, pidType is an input, out is the output.
static void copyPid_iCub2Coman(const Pid *iCubPid_in, GainSet Coman_pidType, pid_gains_t *ComanPid_out)
{
    ComanPid_out->p = (int16_t)iCubPid_in->kp;
    ComanPid_out->i = (int16_t)iCubPid_in->ki;
    ComanPid_out->d = (int16_t)iCubPid_in->kd;
    ComanPid_out->gain_set = Coman_pidType;
    /*out->limitonintegral = (int16_t)in->max_int;
    out->limitonoutput = (int16_t)in->max_output;
    out->offset = (int16_t)in->offset;
    out->scale = (int8_t)in->scale;
    */
}

// Coman input, pidType and out are output values, theyr memory has to be allocated externally
static void copyPid_Coman2iCub(pid_gains_t *ComanPid_in, Pid *iCubPid_out, GainSet *Coman_pidType)
{
    iCubPid_out->kp = ComanPid_in->p;
    iCubPid_out->ki = ComanPid_in->i;
    iCubPid_out->kd = ComanPid_in->d;
    *Coman_pidType = ComanPid_in->gain_set;
//     out->max_int = in->limitonintegral;
//     out->max_output = in->limitonoutput;
//     out->offset = in->offset;
//     out->scale = in->scale;
}


// This will be moved in the ImplXXXInterface
static double convertA2I(double angle_in_degrees, double zero, double factor)
{
	return (angle_in_degrees + zero) * factor;
}


static inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    fprintf(stderr, "%s not yet implemented for comanMotionControl\n", txt);

    return false;
}

#define NV_NOT_FOUND	return nv_not_found();

static bool nv_not_found(void)
{
    yError () << " nv_not_found!! This may mean that this variable is not handled by this EMS\n";
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

#if 0
    _debug_params=allocAndCheck<DebugParameters>(nj);
    _impedance_params=allocAndCheck<ImpedanceParameters>(nj);
    _impedance_limits=allocAndCheck<ImpedanceLimits>(nj);
    _estim_params=allocAndCheck<SpeedEstimationParameters>(nj);
#endif
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
//     Boards_ctrl(),
    _mutex(1)
{
    yTrace();
//     boards_ctrl = NULL;
// #ifdef _OLD_STYLE_
    udppkt_data 	= 0x00;
    udppkt_size 	= 0x00;

    _pids			= NULL;
    _tpids			= NULL;
    _firstJoint 	= 0;

    _tpidsEnabled	= false;
    _njoints 		= 0;

    _axisMap		= NULL;
    _angleToEncoder = NULL;
    _zeros			= NULL;
    _encoderconversionfactor = NULL;
    _encoderconversionoffset = NULL;

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

    controlMode = POSITION_GAINS;
    
    // Check status of joints
    _enabledPid		= NULL;
    _enabledAmp 	= NULL;
    _calibrated		= NULL;
// #endif
#if 0
    _impedance_params = NULL;
    _impedance_limits = NULL;
    _estim_params   = NULL;
    res           = NULL;
    requestQueue  = NULL;
#endif
}

comanMotionControl::~comanMotionControl()
{
    yTrace();
}

bool comanMotionControl::open(yarp::os::Searchable &config)
{
    yTrace();


    fromConfig(config);


//
//      INIT ALL INTERFACES
//
    ImplementControlCalibration2<comanMotionControl, IControlCalibration2>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementAmplifierControl<comanMotionControl, IAmplifierControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementEncodersTimed::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementPositionControl<comanMotionControl, IPositionControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementPidControl<comanMotionControl, IPidControl>:: initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementControlMode::initialize(_njoints, _axisMap);
    ImplementVelocityControl<comanMotionControl, IVelocityControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementDebugInterface::initialize(_njoints, _axisMap, _angleToEncoder, _zeros, _rotToEncoder);
    ImplementControlLimits<comanMotionControl, IControlLimits>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);

    Boards_ctrl::open(config);

    return true;
// #else
//     return NOT_YET_IMPLEMENTED("open");
// #endif
}


bool comanMotionControl::fromConfig(yarp::os::Searchable &config)
{
    yTrace();
    // yTrace();
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

    // Zero Values
    if (!extractGroup(general, xtmp, "Zeros","a list of offsets for the zero point", _njoints+1))
        return false;
    else
        for (i = 1; i < xtmp.size(); i++)
            _zeros[i-1] = xtmp.get(i).asDouble();


    return true;
}

bool comanMotionControl::init()
{
    return NOT_YET_IMPLEMENTED("init");
}


bool comanMotionControl::close()
{
    yTrace();
    Boards_ctrl::close();

#ifdef _OLD_STYLE_
    ImplementControlMode::uninitialize();
    ImplementEncodersTimed::uninitialize();
    ImplementPositionControl<comanMotionControl, IPositionControl>::uninitialize();
    ImplementVelocityControl<comanMotionControl, IVelocityControl>::uninitialize();
    ImplementPidControl<comanMotionControl, IPidControl>::uninitialize();
    ImplementControlCalibration2<comanMotionControl, IControlCalibration2>::uninitialize();
    ImplementAmplifierControl<comanMotionControl, IAmplifierControl>::uninitialize();

    return true;
#else
    return NOT_YET_IMPLEMENTED("close");
#endif
}


// eoThreadEntry * comanMotionControl::appendWaitRequest(int j, uint16_t nvid)
// {
// 	yTrace();
// #ifdef _OLD_STYLE_
// 	eoRequest req;
// 	if(!requestQueue->threadPool->getId(&req.threadId) )
// 		fprintf(stderr, "Error: too much threads!! (comanMotionControl)");
// 	req.joint = j;
// 	req.nvid = res->transceiver->translate_NVid2index(_fId.boardNum, _fId.ep, nvid);
// 
// 	requestQueue->append(req);
// 	return requestQueue->threadPool->getThreadTable(req.threadId);
// #else
//   return NULL;
// #endif 
// }

///////////// PID INTERFACE

bool comanMotionControl::setPidRaw(int j, const Pid &pid)
{
    yTrace();
    McBoard *joint_p;
    pid_gains_t p_i_d;

    copyPid_iCub2Coman(&pid, controlMode, &p_i_d);

    joint_p = _mcs[j];
    if( 0 == joint_p)
    {
        yError() << "Calling SetPid on a non-existing joint j" << j;
        return false;
    }
    else
        return (!joint_p->setItem(SET_PID_GAINS, &p_i_d.gain_set, sizeof(p_i_d)) );
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
    pid_gains_t ComanPid, P_pid, T_pid;
    GainSet pidType;
    McBoard *joint_p = _mcs[j];

    if( 0 == joint_p)
    {
        yError() << "Calling SetPid on a non-existing joint j" << j;
        return false;
    }

 // get back PIDs from the boards
    ComanPid.gain_set = controlMode;
    joint_p->getItem(GET_PID_GAINS,      &ComanPid.gain_set, 1, REPLY_PID_GAINS, &ComanPid, sizeof(ComanPid));
    copyPid_Coman2iCub(&P_pid, pid, &pidType);

    return true;
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
    {yTrace();}
    *ax = _njoints;
    return true;
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

bool comanMotionControl::checkMotionDoneRaw(bool *flag)
{
//     yTrace();
//     return NOT_YET_IMPLEMENTED("checkMotionDoneRaw");
}

bool comanMotionControl::checkMotionDoneRaw(int j, bool *flag)
{
//     yTrace();
//     return NOT_YET_IMPLEMENTED("checkMotionDoneRaw");
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
//     yTrace();
//     return NOT_YET_IMPLEMENTED("getControlModeRaw");
}

bool comanMotionControl::getControlModesRaw(int* v)
{
//     yTrace();
//     return NOT_YET_IMPLEMENTED("getControlModesRaw");
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
//     yTrace();
//     return NOT_YET_IMPLEMENTED("getEncoderRaw");
}

bool comanMotionControl::getEncodersRaw(double *encs)
{
//     yTrace();
//     return NOT_YET_IMPLEMENTED("getEncodersRaw");
}

bool comanMotionControl::getEncoderSpeedRaw(int j, double *sp)
{
//     yTrace();
//     return NOT_YET_IMPLEMENTED("getEncoderSpeedRaw");
}

bool comanMotionControl::getEncoderSpeedsRaw(double *spds)
{
//     yTrace();
//     return NOT_YET_IMPLEMENTED("getEncoderSpeedsRaw");
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
    McBoard *joint_p = _mcs[j];

    if( 0 == joint_p)
    {
//         yError() << "Calling getEncoderTimedRaw on a non-existing joint j" << j;
        *enc = j;
        *stamp = 0;
        return true;
    }
    // viene probabilmente broadcastata... usare la ricezione udp per questo. Così è corretto?
    mc_bc_data_t data;
#warning "this implies a memcopy!! To be optimized!! And add timestamp"
    joint_p->get_bc_data(&data);
    *enc = (double) data.Position;

    // fix this
    *stamp = (double) 0x00;

    return true;
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
    McBoard *joint_p = _mcs[j];

    int _max_pos, _min_pos;  // boards use int, we use double;

    if( 0 == joint_p)
    {
        yError() << "Calling SetPid on a non-existing joint j" << j;
        return false;
    }

    joint_p->getItem(GET_MIN_POSITION,   NULL, 0, REPLY_MIN_POSITION, &_min_pos, sizeof(_min_pos));
    joint_p->getItem(GET_MAX_POSITION,   NULL, 0, REPLY_MAX_POSITION, &_max_pos, sizeof(_max_pos));

    yDebug() << "Max position " << _max_pos;
    yDebug() << "Min position " << _min_pos;

    *min = (double) _min_pos;
    *max = (double) _max_pos;
    return true;
}

