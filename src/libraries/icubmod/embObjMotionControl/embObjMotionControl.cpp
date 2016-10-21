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

#include "embObjMotionControl.h"
#include <ethManager.h>
#include <FeatureInterface.h>


#include <yarp/os/LogStream.h>

#include "EoCommon.h"
#include "EOarray.h"
#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolMC.h"
#include "EoProtocolAS.h"

#include <yarp/os/NetType.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;


// macros
#define NEW_JSTATUS_STRUCT 1
#define ASK_REFERENCE_TO_FIRMWARE 1


// Utilities
torqueControlHelper::torqueControlHelper(int njoints, double* p_angleToEncoders, double* p_newtonsTosens )
{
   jointsNum=njoints;
   newtonsToSensor = new double  [jointsNum];
   angleToEncoders = new double  [jointsNum];

   if (p_angleToEncoders!=0)
       memcpy(angleToEncoders, p_angleToEncoders, sizeof(double)*jointsNum);
   else
       for (int i=0; i<jointsNum; i++) {angleToEncoders[i]=1.0;}

   if (p_newtonsTosens!=0)
       memcpy(newtonsToSensor, p_newtonsTosens, sizeof(double)*jointsNum);
   else
       for (int i=0; i<jointsNum; i++) {newtonsToSensor[i]=1.0;}


}

torqueControlHelper::torqueControlHelper(int njoints, float* p_angleToEncoders, double* p_newtonsTosens )
{
   jointsNum=njoints;
   newtonsToSensor = new double  [jointsNum];
   angleToEncoders = new double  [jointsNum];

   if (p_angleToEncoders!=0)
       for (int i=0; i<jointsNum; i++) {angleToEncoders[i] = p_angleToEncoders[i];}
   else
       for (int i=0; i<jointsNum; i++) {angleToEncoders[i]=1.0;}

   if (p_newtonsTosens!=0)
       memcpy(newtonsToSensor, p_newtonsTosens, sizeof(double)*jointsNum);
   else
       for (int i=0; i<jointsNum; i++) {newtonsToSensor[i]=1.0;}
}

bool embObjMotionControl::EncoderType_iCub2eo(const string* in, uint8_t *out)
{
    if (*in == "NONE")
    {
        *out = 0;
        return true;
    }
    else if (*in == "AEA")
    {
        *out = 1;
        return true;
    }
    else if (*in == "ROIE")
    {
        *out = 2;
        return true;
    }
    else if (*in == "HALL_ADC")
    {
        *out = 3;
        return true;
    }
    else if (*in == "MAIS")
    {
        *out = 4;
        return true;
    }
    else if (*in == "OPTICAL_QUAD")
    {
        *out = 5;
        return true;
    }
    else if (*in == "HALL_MOTOR_SENS")
    {
        *out = 6;
        return true;
    }
    *out = 0;
    return false;
}

bool embObjMotionControl::EncoderType_eo2iCub(const uint8_t *in, string* out)
{
    if (*in == 0)
    {
        *out = "NONE";
        return true;
    }
    else if (*in == 1)
    {
        *out = "AEA";
        return true;
    }
    else if (*in == 2)
    {
        *out = "ROIE";
        return true;
    }
    else if (*in == 3)
    {
        *out = "HALL_ADC";
        return true;
    }
    else if (*in == 4)
    {
        *out = "MAIS";
        return true;
    }
    else if (*in == 5)
    {
        *out = "OPTICAL_QUAD";
        return true;
    }
    else if (*in == 6)
    {
        *out = "HALL_MOTOR_SENS";
        return true;
    }
    *out = "ERROR";
    return false;
}

void embObjMotionControl::copyPid_iCub2eo(const Pid *in, eOmc_PID_t *out)
{
    memset(out, 0, sizeof(eOmc_PID_t));     // marco.accame: it is good thing to clear the out struct before copying. this prevent future members of struct not yet managed to be dirty.
    out->kp = (float) (in->kp);
    out->ki = (float) (in->ki);
    out->kd = (float) (in->kd);
    out->limitonintegral = (float)(in->max_int);
    out->limitonoutput = (float)(in->max_output);
    out->offset = (float) (in->offset);
    out->scale = (int8_t) (in->scale);
    out->kff = (float) (in->kff);
    out->stiction_down_val = (float)(in->stiction_down_val);
    out->stiction_up_val = (float)(in->stiction_up_val);
}

void embObjMotionControl::copyPid_eo2iCub(eOmc_PID_t *in, Pid *out)
{
    // marco.accame: in here i dont clear the out class because there is not a clear() method
    out->kp = (double) in->kp;
    out->ki = (double) in->ki;
    out->kd = (double) in->kd;
    out->max_int = (double) in->limitonintegral;
    out->max_output = (double) in->limitonoutput;
    out->offset = (double) in->offset;
    out->scale = (double) in->scale;
    out->setStictionValues(in->stiction_up_val, in->stiction_down_val);
    out->setKff(in->kff);
}

// This will be moved in the ImplXXXInterface
static double convertA2I(double angle_in_degrees, double zero, double factor)
{
    return (angle_in_degrees + zero) * factor;
}


static inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    yError() << txt << " is not yet implemented for embObjMotionControl";
    return true;
}

static inline bool DEPRECATED(const char *txt)
{
    yError() << txt << " has been deprecated for embObjMotionControl";
    return true;
}

#define NV_NOT_FOUND    return nv_not_found();

static bool nv_not_found(void)
{
    yError () << " nv_not_found!! This may mean that this variable is not handled by this EMS\n";
    return false;
}


bool embObjMotionControl::controlModeCommandConvert_yarp2embObj(int vocabMode, eOenum08_t &embOut)
{
    bool ret = true;

    switch(vocabMode)
    {
    case VOCAB_CM_IDLE:
        embOut = eomc_controlmode_cmd_idle;
        break;

    case VOCAB_CM_POSITION:
        embOut = eomc_controlmode_cmd_position;
        break;

    case VOCAB_CM_POSITION_DIRECT:
        embOut = eomc_controlmode_cmd_direct;
        break;

    case VOCAB_CM_VELOCITY:
        embOut = eomc_controlmode_cmd_velocity;
        break;

    case VOCAB_CM_MIXED:
        embOut = eomc_controlmode_cmd_mixed;
        break;

    case VOCAB_CM_TORQUE:
        embOut = eomc_controlmode_cmd_torque;
        break;

    case VOCAB_CM_IMPEDANCE_POS:
        embOut = eomc_controlmode_cmd_impedance_pos;
        break;

    case VOCAB_CM_IMPEDANCE_VEL:
        embOut = eomc_controlmode_cmd_impedance_vel;
        break;

    case VOCAB_CM_OPENLOOP:
        embOut = eomc_controlmode_cmd_openloop;
        break;

    case VOCAB_CM_FORCE_IDLE:
        embOut = eomc_controlmode_cmd_force_idle;
        break;

    default:
        ret = false;
        break;
    }
    return ret;
}

int embObjMotionControl::controlModeCommandConvert_embObj2yarp(eOmc_controlmode_command_t embObjMode)
{
    yError() << "embObjMotionControl::controlModeCommandConvert_embObj2yarp" << " is not yet implemented for embObjMotionControl";
    return 0;

}

bool embObjMotionControl::controlModeStatusConvert_yarp2embObj(int vocabMode, eOmc_controlmode_t &embOut)
{
    yError() << "controlModeStatusConvert_yarp2embObj" << " is not yet implemented for embObjMotionControl";
    return false;
}

int embObjMotionControl::controlModeStatusConvert_embObj2yarp(eOenum08_t embObjMode)
{
    int vocabOut;
    switch(embObjMode)
    {
    case eomc_controlmode_idle:
        vocabOut = VOCAB_CM_IDLE;
        break;

    case eomc_controlmode_position:
        vocabOut = VOCAB_CM_POSITION;
        break;

    case eomc_controlmode_velocity:
        vocabOut = VOCAB_CM_VELOCITY;
        break;

    case eomc_controlmode_direct:
        vocabOut = VOCAB_CM_POSITION_DIRECT;
        break;

    case eomc_controlmode_mixed:
    case eomc_controlmode_velocity_pos:      // they are the same, this will probably removed in the future
        vocabOut = VOCAB_CM_MIXED;
        break;

    case eomc_controlmode_torque:
        vocabOut = VOCAB_CM_TORQUE;
        break;

    case eomc_controlmode_calib:
        vocabOut = VOCAB_CM_CALIBRATING;
        break;

    case eomc_controlmode_impedance_pos:
        vocabOut = VOCAB_CM_IMPEDANCE_POS;
        break;

    case eomc_controlmode_impedance_vel:
        vocabOut = VOCAB_CM_IMPEDANCE_VEL;
        break;

    case eomc_controlmode_openloop:
    case eomc_controlmode_current:          // for the high level they are the same
        vocabOut = VOCAB_CM_OPENLOOP;
        break;

    case eomc_controlmode_hwFault:
        vocabOut = VOCAB_CM_HW_FAULT;
        break;

    case eomc_controlmode_notConfigured:
        vocabOut = VOCAB_CM_NOT_CONFIGURED;
        break;

    case eomc_controlmode_configured:
        vocabOut = VOCAB_CM_CONFIGURED;
        break;

    default:
        printf("embObj to yarp unknown controlmode %d\n", embObjMode);
        vocabOut = VOCAB_CM_UNKNOWN;
        break;
    }
    return vocabOut;
}


bool embObjMotionControl::interactionModeCommandConvert_yarp2embObj(int vocabMode, eOenum08_t &embOut)
{
    bool ret = true;

    switch(vocabMode)
    {
    case VOCAB_IM_STIFF:
        embOut = eOmc_interactionmode_stiff;
        break;

    case VOCAB_IM_COMPLIANT:
        embOut = eOmc_interactionmode_compliant;
        break;

    default:
        ret = false;
        break;
    }
    return ret;
}


bool embObjMotionControl::interactionModeStatusConvert_embObj2yarp(eOenum08_t embObjMode, int &vocabOut)
{
    bool ret = true;
    switch(embObjMode)
    {
    case eOmc_interactionmode_stiff:
        vocabOut = VOCAB_IM_STIFF;
        break;

    case eOmc_interactionmode_compliant:
        vocabOut = VOCAB_IM_COMPLIANT;
        break;

    default:
        vocabOut = 666; //VOCAB_CM_UNKNOWN;
        yError() << "Received an unknown interactionMode from the EMS boards with value " << embObjMode;
//        ret = false;
        break;
    }
    return ret;
}


//generic function that check is key1 is present in input bottle and that the result has size elements
// return true/false
bool embObjMotionControl::extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    size++;
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        yError () << key1.c_str() << " parameter not found";
        return false;
    }

    if(tmp.size()!=size)
    {
        if(NULL != res)
        {
            yError () << key1.c_str() << " incorrect number of entries in BOARD " << res->getName() << "IP" << res->getIPv4string();
        }
        else
        {
            yError () << key1.c_str() << " incorrect number of entries in BOARD IP =" << boardIPstring;
        }
        return false;
    }

    out=tmp;
    return true;
}


bool embObjMotionControl::alloc(int nj)
{
    _axisMap = allocAndCheck<int>(nj);
    _angleToEncoder = allocAndCheck<double>(nj);
    _encodersStamp = allocAndCheck<double>(nj);
    _DEPRECATED_encoderconversionoffset = allocAndCheck<float>(nj);
    _DEPRECATED_encoderconversionfactor = allocAndCheck<float>(nj);
    _jointEncoderType = allocAndCheck<uint8_t>(nj);
    _rotorEncoderType = allocAndCheck<uint8_t>(nj);
    _jointEncoderRes = allocAndCheck<int>(nj);
    _rotorEncoderRes = allocAndCheck<int>(nj);
    _gearbox = allocAndCheck<double>(nj);
    _gearboxE2J = allocAndCheck<double>(nj);
    _maxJntCmdVelocity = allocAndCheck<double>(nj);
    _maxMotorVelocity = allocAndCheck<double>(nj);
    _newtonsToSensor=allocAndCheck<double>(nj);
    _hasHallSensor = allocAndCheck<bool>(nj);
    _hasTempSensor = allocAndCheck<bool>(nj);
    _hasRotorEncoder = allocAndCheck<bool>(nj);
    _hasRotorEncoderIndex = allocAndCheck<bool>(nj);
    _hasSpeedEncoder = allocAndCheck<bool>(nj);
    _rotorIndexOffset = allocAndCheck<int>(nj);
    _motorPoles = allocAndCheck<int>(nj);
    _rotorlimits_max = allocAndCheck<double>(nj);
    _rotorlimits_min = allocAndCheck<double>(nj);

    _pids=allocAndCheck<Pid>(nj);
    _vpids=allocAndCheck<Pid>(nj);
    _tpids=allocAndCheck<Pid>(nj);
    _cpids = allocAndCheck<Pid>(nj);

    _impedance_params=allocAndCheck<ImpedanceParameters>(nj);
    _impedance_limits=allocAndCheck<ImpedanceLimits>(nj);
    _axisName = new string[nj];
    _jointType = allocAndCheck<JointTypeEnum>(nj);

    _limitsMax=allocAndCheck<double>(nj);
    _limitsMin=allocAndCheck<double>(nj);
    _hwLimitsMax=allocAndCheck<double>(nj);
    _hwLimitsMin=allocAndCheck<double>(nj);
    _kinematic_mj=allocAndCheck<double>(16);
    _currentLimits=allocAndCheck<MotorCurrentLimits>(nj);
    _motorPwmLimits=allocAndCheck<double>(nj);
    checking_motiondone=allocAndCheck<bool>(nj);

    _velocityShifts=allocAndCheck<int>(nj);
    _velocityTimeout=allocAndCheck<int>(nj);
    _kbemf=allocAndCheck<double>(nj);
    _ktau=allocAndCheck<double>(nj);
    _filterType=allocAndCheck<int>(nj);
    _last_position_move_time=allocAndCheck<double>(nj);

    // Reserve space for data stored locally. values are initialize to 0
    _ref_command_positions = allocAndCheck<double>(nj);
    _ref_positions = allocAndCheck<double>(nj);
    _ref_command_speeds = allocAndCheck<double>(nj);
    _ref_speeds = allocAndCheck<double>(nj);
    _ref_accs = allocAndCheck<double>(nj);
    _ref_torques = allocAndCheck<double>(nj);
    _enabledAmp = allocAndCheck<bool>(nj);
    _enabledPid = allocAndCheck<bool>(nj);
    _calibrated = allocAndCheck<bool>(nj);
    _cacheImpedance = allocAndCheck<eOmc_impedance_t>(nj);
    _posistionControlLaw = allocAndCheck<string>(nj);
    _velocityControlLaw = allocAndCheck<string>(nj);
    _torqueControlLaw = allocAndCheck<string>(nj);

    //debug purpose

    return true;
}

bool embObjMotionControl::dealloc()
{
    checkAndDestroy(_axisMap);
    checkAndDestroy(_angleToEncoder);
    checkAndDestroy(_encodersStamp);
    checkAndDestroy(_DEPRECATED_encoderconversionoffset);
    checkAndDestroy(_DEPRECATED_encoderconversionfactor);
    checkAndDestroy(_jointEncoderRes);
    checkAndDestroy(_rotorEncoderRes);
    checkAndDestroy(_jointEncoderType);
    checkAndDestroy(_rotorEncoderType);
    checkAndDestroy(_gearbox);
    checkAndDestroy(_gearboxE2J);
    checkAndDestroy(_maxJntCmdVelocity);
    checkAndDestroy(_maxMotorVelocity);
    checkAndDestroy(_newtonsToSensor);
    checkAndDestroy(_pids);
    checkAndDestroy(_vpids);
    checkAndDestroy(_tpids);
    checkAndDestroy(_cpids);
    checkAndDestroy(_impedance_params);
    checkAndDestroy(_impedance_limits);
    checkAndDestroy(_limitsMax);
    checkAndDestroy(_limitsMin);
    checkAndDestroy(_hwLimitsMax);
    checkAndDestroy(_hwLimitsMin);
    checkAndDestroy(_kinematic_mj);
    checkAndDestroy(_currentLimits);
    checkAndDestroy(_motorPwmLimits);
    checkAndDestroy(checking_motiondone);
    checkAndDestroy(_velocityShifts);
    checkAndDestroy(_velocityTimeout);
    checkAndDestroy(_kbemf);
    checkAndDestroy(_ktau);
    checkAndDestroy(_filterType);
    checkAndDestroy(_ref_command_positions);
    checkAndDestroy(_ref_positions);
    checkAndDestroy(_ref_command_speeds);
    checkAndDestroy(_ref_speeds);
    checkAndDestroy(_ref_accs);
    checkAndDestroy(_ref_torques);
    checkAndDestroy(_enabledAmp);
    checkAndDestroy(_enabledPid);
    checkAndDestroy(_calibrated);
    checkAndDestroy(_hasHallSensor);
    checkAndDestroy(_hasTempSensor);
    checkAndDestroy(_hasRotorEncoder);
    checkAndDestroy(_hasRotorEncoderIndex);
    checkAndDestroy(_hasSpeedEncoder);
    checkAndDestroy(_rotorIndexOffset);
    checkAndDestroy(_motorPoles);
    checkAndDestroy(_axisName);
    checkAndDestroy(_jointType);
    checkAndDestroy(_rotorlimits_max);
    checkAndDestroy(_rotorlimits_min);

    if(requestQueue)
        delete requestQueue;
    return true;
}

embObjMotionControl::embObjMotionControl() :
    ImplementControlCalibration2<embObjMotionControl, IControlCalibration2>(this),
    ImplementAmplifierControl<embObjMotionControl, IAmplifierControl>(this),
    ImplementPidControl<embObjMotionControl, IPidControl>(this),
    ImplementEncodersTimed(this),
    ImplementPositionControl2(this),
    ImplementVelocityControl<embObjMotionControl, IVelocityControl>(this),
    ImplementVelocityControl2(this),
    ImplementControlMode2(this),
    ImplementImpedanceControl(this),
    ImplementMotorEncoders(this),
#ifdef IMPLEMENT_DEBUG_INTERFACE
    ImplementDebugInterface(this),
#endif
    ImplementTorqueControl(this),
    ImplementControlLimits2(this),
    ImplementPositionDirect(this),
    ImplementOpenLoopControl(this),
    ImplementInteractionMode(this),
    ImplementMotor(this),
    ImplementRemoteVariables(this),
    ImplementAxisInfo(this),
    _mutex(1),
    SAFETY_THRESHOLD(2.0)
{
    _gearbox       = 0;
    _gearboxE2J      = 0;
    opened        = 0;
    _pids         = NULL;
    _vpids        = NULL;
    _tpids        = NULL;
    _cpids        = NULL;
    res           = NULL;
    requestQueue  = NULL;
    _njoints      = 0;
    _axisMap      = NULL;
    _encodersStamp = NULL;
    _DEPRECATED_encoderconversionfactor = NULL;
    _DEPRECATED_encoderconversionoffset = NULL;
    _angleToEncoder = NULL;
    _hasHallSensor = NULL;
    _hasTempSensor = NULL;
    _hasRotorEncoder = NULL;
    _hasRotorEncoderIndex = NULL;
    _hasSpeedEncoder = NULL;
    _rotorIndexOffset = NULL;
    _motorPoles       = NULL;
    _cacheImpedance   = NULL;
    _impedance_params = NULL;
    _impedance_limits = NULL;
    _rotorlimits_max  = NULL;
    _rotorlimits_min  = NULL;

    _axisName         = NULL;
    _jointType        = NULL;
    _limitsMin        = NULL;
    _limitsMax        = NULL;
    _hwLimitsMax      = NULL;
    _hwLimitsMin      = NULL;
    _currentLimits    = NULL;
    _motorPwmLimits   = NULL;
    _velocityShifts   = NULL;
    _velocityTimeout  = NULL;
    _maxJntCmdVelocity= NULL;
    _maxMotorVelocity = NULL;
    _newtonsToSensor  = NULL;
    _jointEncoderRes  = NULL;
    _jointEncoderType = NULL;
    _rotorEncoderRes  = NULL;
    _rotorEncoderType = NULL;
    _ref_accs         = NULL;
    _ref_command_speeds   = NULL;
    _ref_command_positions= NULL;
    _ref_positions    = NULL;
    _ref_speeds       = NULL;
    _ref_torques      = NULL;
    _kinematic_mj     = NULL;
    _kbemf            = NULL;
    _ktau             = NULL;
    _filterType       = NULL;
    _positionControlUnits = P_MACHINE_UNITS;
    _torqueControlUnits = T_MACHINE_UNITS;
    _torqueControlEnabled = false;
    _torqueControlHelper = NULL;

    checking_motiondone = NULL;
    // debug connection
    tot_packet_recv   = 0;
    errors            = 0;
    start             = 0;
    end               = 0;

    // Check status of joints
    _enabledPid       = NULL;
    _enabledAmp       = NULL;
    _calibrated       = NULL;
    _last_position_move_time = NULL;
    // NV stuff
    NVnumber          = 0;

    useRawEncoderData = false;
    _pwmIsLimited     = false;
    _currentPidsAvailables = false;

    ConstString tmp = NetworkBase::getEnvironment("ETH_VERBOSEWHENOK");
    if (tmp != "")
    {
        verbosewhenok = (bool)NetType::toInt(tmp);
    }
    else
    {
        verbosewhenok = false;
    }

    parser = NULL;

}


embObjMotionControl::~embObjMotionControl()
{
    yTrace() << "embObjMotionControl::~embObjMotionControl()";

    if(NULL != parser)
    {
        delete parser;
        parser = NULL;
    }

    dealloc();
}


bool embObjMotionControl::initialised()
{
    return opened;
}


bool embObjMotionControl::open(yarp::os::Searchable &config)
{
    // - first thing to do is verify if the eth manager is available. then i parse info about the eth board.

    ethManager = TheEthManager::instance();
    if(NULL == ethManager)
    {
        yFatal() << "embObjMotionControl::open() fails to instantiate ethManager";
        return false;
    }


    if(false == ethManager->verifyEthBoardInfo(config, NULL, boardIPstring, sizeof(boardIPstring)))
    {
        yError() << "embObjMotionControl::open(): object TheEthManager fails in parsing ETH propertiex from xml file";
        return false;
    }
    // add specific info about this device ...

#if defined(EMBOBJMC_USESERVICEPARSER)
    if(NULL == parser)
    {
        parser = new ServiceParser;
    }
#endif

    // - now all other things


    // READ CONFIGURATION
    if(!fromConfig(config))
    {
        yError() << "Missing parameters in config file";
        return false;
    }

    //  INIT ALL INTERFACES
    yarp::sig::Vector tmpZeros; tmpZeros.resize (_njoints, 0.0);
    yarp::sig::Vector tmpOnes;  tmpOnes.resize  (_njoints, 1.0);

    ImplementControlCalibration2<embObjMotionControl, IControlCalibration2>::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementAmplifierControl<embObjMotionControl, IAmplifierControl>::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementEncodersTimed::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementMotorEncoders::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementPositionControl2::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementPidControl<embObjMotionControl, IPidControl>::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementControlMode2::initialize(_njoints, _axisMap);
    ImplementVelocityControl<embObjMotionControl, IVelocityControl>::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementVelocityControl2::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementControlLimits2::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementImpedanceControl::initialize(_njoints, _axisMap, _angleToEncoder, NULL, _newtonsToSensor);
    ImplementTorqueControl::initialize(_njoints, _axisMap, _angleToEncoder, NULL, _newtonsToSensor);
    ImplementPositionDirect::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementOpenLoopControl::initialize(_njoints, _axisMap);
    ImplementInteractionMode::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementMotor::initialize(_njoints, _axisMap);
    ImplementRemoteVariables::initialize(_njoints, _axisMap);
    ImplementAxisInfo::initialize(_njoints, _axisMap);


    // -- instantiate EthResource etc.

    res = ethManager->requestResource2(this, config);
    if(NULL == res)
    {
        yError() << "embObjMotionControl::open() fails because could not instantiate the ethResource for BOARD w/ IP = " << boardIPstring << " ... unable to continue";
        return false;
    }


    if(!res->verifyEPprotocol(eoprot_endpoint_motioncontrol))
    {
        cleanup();
        return false;
    }

//    uint32_t addr = res->getIPv4remoteAddress();

//    serviceConfig.id = (addr & 0xff000000) >> 24;
//    yError() << " ************* board ID =" << serviceConfig.id;
//    eOmn_serv_parameter_t* servparam = NULL;

//    if(serviceConfig.id == 3)
//    {
//        if(false == parser->parseService(config, serviceConfig))
//        {
//            return false;
//        }
//     servparam= &serviceConfig.theservice;

//     yError () << "*** board config addr .3  has been read from parser";
//     }


#if defined(EMBOBJMC_USESERVICEPARSER)
    const eOmn_serv_parameter_t* servparam = &serviceConfig.ethservice;
    if(eomn_serv_MC_generic == serviceConfig.ethservice.configuration.type)
    {
        servparam = NULL;
    }
#else
    const eOmn_serv_parameter_t* servparam = NULL;
#endif

    if(false == res->serviceVerifyActivate(eomn_serv_category_mc, servparam))
    {
        yError() << "embObjMotionControl::open() has an error in call of ethResources::serviceVerifyActivate() for BOARD" << res->getName() << "IP" << res->getIPv4string();
        cleanup();
        return false;
    }


    NVnumber = res->getNVnumber(eoprot_endpoint_motioncontrol);
    requestQueue = new eoRequestsQueue(NVnumber);

    if(!init() )
    {
        yError() << "embObjMotionControl::open() has an error in call of embObjMotionControl::init() for BOARD" << res->getName() << "IP" << res->getIPv4string();
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjMotionControl::init() has succesfully initted BOARD" << res->getName() << "IP" << res->getIPv4string();
        }
    }


    if(false == res->serviceStart(eomn_serv_category_mc))
    {
        yError() << "embObjMotionControl::open() fails to start mc service for board" << res->getName() << "IP" << res->getIPv4string() << ": cannot continue";
        cleanup();
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjMotionControl::open() correctly starts mc service of board" << res->getName() << "IP" << res->getIPv4string();
        }
    }


    opened = true;
    return true;
}


bool embObjMotionControl::isEpManagedByBoard()
{
    return res->isEPsupported(eoprot_endpoint_motioncontrol);
}


bool embObjMotionControl::parseImpedanceGroup_NewFormat(Bottle& pidsGroup, ImpedanceParameters vals[])
{
    int j=0;
    Bottle xtmp;
    if (!extractGroup(pidsGroup, xtmp, "stiffness", "stiffness parameter", _njoints))  return false; for (j=0; j<_njoints; j++) vals[j].stiffness = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "damping", "damping parameter", _njoints))      return false; for (j=0; j<_njoints; j++) vals[j].damping = xtmp.get(j+1).asDouble();
    return true;
}


bool embObjMotionControl::parsePositionPidsGroup(Bottle& pidsGroup, Pid myPid[])
{
    int j=0;
    Bottle xtmp;
    if (!extractGroup(pidsGroup, xtmp, "kp", "Pid kp parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].kp = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "kd", "Pid kd parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].kd = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "ki", "Pid kp parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].ki = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "maxInt", "Pid maxInt parameter", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].max_int = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "maxOutput", "Pid maxOutput parameter", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].max_output = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "shift", "Pid shift parameter", _njoints))     return false; for (j=0; j<_njoints; j++) myPid[j].scale = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "ko", "Pid ko parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].offset = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "stictionUp", "Pid stictionUp", _njoints))     return false; for (j=0; j<_njoints; j++) myPid[j].stiction_up_val = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "stictionDwn", "Pid stictionDwn", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].stiction_down_val = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "kff", "Pid kff parameter", _njoints))         return false; for (j=0; j<_njoints; j++) myPid[j].kff = xtmp.get(j+1).asDouble();

    //conversion from metric to machine units (if applicable)
    if (_positionControlUnits==P_METRIC_UNITS)
    {
        for (j=0; j<_njoints; j++)
        {
            myPid[j].kp = myPid[j].kp / _angleToEncoder[j];  //[PWM/deg]
            myPid[j].ki = myPid[j].ki / _angleToEncoder[j];  //[PWM/deg]
            myPid[j].kd = myPid[j].kd / _angleToEncoder[j];  //[PWM/deg]
        }
    }
    else
    {
        //do nothing
    }

    return true;
}

// bool embObjMotionControl::parseVelocityPidsGroup(Bottle& pidsGroup, Pid myPid[])
// {
//     int j=0;
//     Bottle xtmp;
//     if (!extractGroup(pidsGroup, xtmp, "kp", "Pid kp parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].kp = xtmp.get(j+1).asDouble();
//     if (!extractGroup(pidsGroup, xtmp, "kd", "Pid kd parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].kd = xtmp.get(j+1).asDouble();
//     if (!extractGroup(pidsGroup, xtmp, "ki", "Pid kp parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].ki = xtmp.get(j+1).asDouble();
//     if (!extractGroup(pidsGroup, xtmp, "maxInt", "Pid maxInt parameter", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].max_int = xtmp.get(j+1).asDouble();
//     if (!extractGroup(pidsGroup, xtmp, "maxOutput", "Pid maxOutput parameter", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].max_output = xtmp.get(j+1).asDouble();
//     if (!extractGroup(pidsGroup, xtmp, "shift", "Pid shift parameter", _njoints))     return false; for (j=0; j<_njoints; j++) myPid[j].scale = xtmp.get(j+1).asDouble();
//     if (!extractGroup(pidsGroup, xtmp, "ko", "Pid ko parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].offset = xtmp.get(j+1).asDouble();
//     if (!extractGroup(pidsGroup, xtmp, "stictionUp", "Pid stictionUp", _njoints))     return false; for (j=0; j<_njoints; j++) myPid[j].stiction_up_val = xtmp.get(j+1).asDouble();
//     if (!extractGroup(pidsGroup, xtmp, "stictionDwn", "Pid stictionDwn", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].stiction_down_val = xtmp.get(j+1).asDouble();
//     if (!extractGroup(pidsGroup, xtmp, "kff", "Pid kff parameter", _njoints))         return false; for (j=0; j<_njoints; j++) myPid[j].kff = xtmp.get(j+1).asDouble();
//
//     //conversion from metric to machine units (if applicable)
//     if (_positionControlUnits==P_METRIC_UNITS)
//     {
//         for (j=0; j<_njoints; j++)
//         {
//             myPid[j].kp = myPid[j].kp / _angleToEncoder[j];  //[PWM/deg]
//             myPid[j].ki = myPid[j].ki / _angleToEncoder[j];  //[PWM/deg]
//             myPid[j].kd = myPid[j].kd / _angleToEncoder[j];  //[PWM/deg]
//         }
//     }
//     else
//     {
//         //do nothing
//     }
//
//     return true;
// }


bool embObjMotionControl::parsePidsGroup(Bottle& pidsGroup, Pid myPid[], string prefix)
{
    int j=0;
    Bottle xtmp;

    if (!extractGroup(pidsGroup, xtmp,  prefix + string("kp"), "Pid kp parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].kp = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp,  prefix + string("kd"), "Pid kd parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].kd = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp,  prefix + string("ki"), "Pid kp parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].ki = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp,  prefix + string("maxInt"), "Pid maxInt parameter", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].max_int = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp,  prefix + string("maxOutput"), "Pid maxOutput parameter", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].max_output = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp,  prefix + string("shift"), "Pid shift parameter", _njoints))     return false; for (j=0; j<_njoints; j++) myPid[j].scale = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp,  prefix + string("ko"), "Pid ko parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].offset = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp,  prefix + string("stictionUp"), "Pid stictionUp", _njoints))     return false; for (j=0; j<_njoints; j++) myPid[j].stiction_up_val = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp,  prefix + string("stictionDwn"), "Pid stictionDwn", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].stiction_down_val = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp,  prefix + string("kff"), "Pid kff parameter", _njoints))         return false; for (j=0; j<_njoints; j++) myPid[j].kff = xtmp.get(j+1).asDouble();

#error VALE: metti in funzione separata ma conversione dep pid per il torque devo usare una conversione differente
    //conversion from metric to machine units (if applicable)
    if (_positionControlUnits==P_METRIC_UNITS)
    {
        for (j=0; j<_njoints; j++)
        {
            myPid[j].kp = myPid[j].kp / _angleToEncoder[j];  //[PWM/deg]
            myPid[j].ki = myPid[j].ki / _angleToEncoder[j];  //[PWM/deg]
            myPid[j].kd = myPid[j].kd / _angleToEncoder[j];  //[PWM/deg]
        }
    }
    else
    {
        //do nothing
    }

    return true;
}

bool embObjMotionControl::parseTorquePidsGroup(Bottle& pidsGroup, Pid myPid[], double kbemf[], double ktau[], int filterType[])
{
    int j=0;
    Bottle xtmp;
    if (!extractGroup(pidsGroup, xtmp, "kp", "Pid kp parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].kp = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "kd", "Pid kd parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].kd = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "ki", "Pid kp parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].ki = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "maxInt", "Pid maxInt parameter", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].max_int = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "maxOutput", "Pid maxOutput parameter", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].max_output = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "shift", "Pid shift parameter", _njoints))     return false; for (j=0; j<_njoints; j++) myPid[j].scale = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "ko", "Pid ko parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].offset = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "stictionUp", "Pid stictionUp", _njoints))     return false; for (j=0; j<_njoints; j++) myPid[j].stiction_up_val = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "stictionDwn", "Pid stictionDwn", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].stiction_down_val = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "kff",   "Pid kff parameter", _njoints))       return false; for (j=0; j<_njoints; j++) myPid[j].kff  = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "kbemf", "kbemf parameter", _njoints))         return false; for (j=0; j<_njoints; j++) kbemf[j]      = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "ktau", "ktau parameter", _njoints))           return false; for (j=0; j<_njoints; j++) ktau[j]       = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "filterType", "filterType param", _njoints))   return false; for (j=0; j<_njoints; j++) filterType[j] = xtmp.get(j+1).asInt();

    //conversion from metric to machine units (if applicable)
    for (j=0; j<_njoints; j++)
    {
        myPid[j].kp = myPid[j].kp / _torqueControlHelper->getNewtonsToSensor(j);  //[PWM/Nm]
        myPid[j].ki = myPid[j].ki / _torqueControlHelper->getNewtonsToSensor(j);  //[PWM/Nm]
        myPid[j].kd = myPid[j].kd / _torqueControlHelper->getNewtonsToSensor(j);  //[PWM/Nm]
        myPid[j].stiction_up_val   = myPid[j].stiction_up_val   * _torqueControlHelper->getNewtonsToSensor(j); //[Nm]
        myPid[j].stiction_down_val = myPid[j].stiction_down_val * _torqueControlHelper->getNewtonsToSensor(j); //[Nm]
    }

    return true;
}


bool embObjMotionControl::parseCurrentPidsGroup(Bottle& pidsGroup, Pid myPid[])
{
    int j=0;
    Bottle xtmp;

    if (!extractGroup(pidsGroup, xtmp, "kp", "Pid kp parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].kp = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "kd", "Pid kd parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].kd = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "ki", "Pid kp parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].ki = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "scale", "Pid scale parameter", _njoints))     return false; for (j=0; j<_njoints; j++) myPid[j].scale = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "maxInt", "Pid maxInt parameter", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].max_int = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "maxOutput", "Pid maxOutput parameter", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].max_output = xtmp.get(j+1).asDouble();

    //conversion from metric to machine units (if applicable)
//     if (_positionControlUnits==P_METRIC_UNITS)
//     {
//         for (j=0; j<_njoints; j++)
//         {
//             myPid[j].kp = myPid[j].kp / _angleToEncoder[j];  //[PWM/deg]
//             myPid[j].ki = myPid[j].ki / _angleToEncoder[j];  //[PWM/deg]
//             myPid[j].kd = myPid[j].kd / _angleToEncoder[j];  //[PWM/deg]
//         }
//     }
//     else
//     {
//         //do nothing
//     }

    return true;
}


int embObjMotionControl::fromConfig_NumOfJoints(yarp::os::Searchable &config)
{
    //
    //  Read Configuration params from file
    //
    int jn = config.findGroup("GENERAL").check("Joints", Value(1), "Number of degrees of freedom").asInt();

    return(jn);
}


bool embObjMotionControl::fromConfig_getGeneralInfo(yarp::os::Searchable &config)
{
    std::string str;

    if (!config.findGroup("GENERAL").find("MotioncontrolVersion").isInt())
    {
        yError() << "Missing MotioncontrolVersion parameter. RobotInterface cannot start. Please contact icub-support@iit.it";
        return false;
    }
    else
    {
        int mcv = config.findGroup("GENERAL").find("MotioncontrolVersion").asInt();
        if (mcv != 2)
        {
            yError() << "Wrong MotioncontrolVersion parameter. RobotInterface cannot start. Please contact icub-support@iit.it";
            return false;
        }
    }

    if(!config.findGroup("GENERAL").find("verbose").isBool())
    {
        yError() << "embObjMotionControl::open() detects that general->verbose bool param is different from accepted values (true / false). Assuming false";
        str=" ";
    }
    else
    {
        if(config.findGroup("GENERAL").find("verbose").asBool())
            str=config.toString().c_str();
        else
            str=" ";
    }
    yTrace() << str;


    // Check useRawEncoderData = do not use calibration data!
    Value use_raw = config.findGroup("GENERAL").find("useRawEncoderData");

    if(use_raw.isNull())
    {
        useRawEncoderData = false;
    }
    else
    {
        if(!use_raw.isBool())
        {
            yWarning() << "embObjMotionControl::open() detected that useRawEncoderData bool param is different from accepted values (true / false). Assuming false";
            useRawEncoderData = false;
        }
        else
        {
            useRawEncoderData = use_raw.asBool();
            if(useRawEncoderData)
            {
                yWarning() << "embObjMotionControl::open() detected that it is using raw data from encoders! Be careful  See 'useRawEncoderData' param in config file";
                yWarning() << "DO NOT USE OR CALIBRATE THE ROBOT IN THIS CONFIGURATION!";
                yWarning() << "CHECK IF THE FAULT BUTTON IS PRESSED and press ENTER to continue";
                getchar();
            }
        }
    }

    // Check useRawEncoderData = do not use calibration data!
    Value use_limitedPWM = config.findGroup("GENERAL").find("useLimitedPWM");
    if(use_limitedPWM.isNull())
    {
        _pwmIsLimited = false;
    }
    else
    {
        if(!use_limitedPWM.isBool())
        {
            _pwmIsLimited = false;
        }
        else
        {
            _pwmIsLimited = use_limitedPWM.asBool();
        }
    }

    return true;
}
bool embObjMotionControl::parseGeneralMecGroup(Bottle &general)
{
    Bottle xtmp;
    int i;

    if (!extractGroup(general, xtmp, "AxisMap", "a list of reordered indices for the axes", _njoints))
        return false;

    for (i = 1; i < xtmp.size(); i++)
        _axisMap[i-1] = xtmp.get(i).asInt();

    if (!extractGroup(general, xtmp, "AxisName", "a list of strings representing the axes names", _njoints))
        return false;
    //beware: axis name has to be remapped here because they are not set using the toHw() helper function
    for (i = 1; i < xtmp.size(); i++)
        _axisName[_axisMap[i - 1]] = xtmp.get(i).asString();

    if (!extractGroup(general, xtmp, "AxisType", "a list of strings representing the axes type (revolute/prismatic)", _njoints))
        return false;
    //beware: axis type has to be remapped here because they are not set using the toHw() helper function
    for (i = 1; i < xtmp.size(); i++)
    {
        string s = xtmp.get(i).asString();
        if (s == "revolute")  _jointType[_axisMap[i - 1]] = VOCAB_JOINTTYPE_REVOLUTE;
        else if (s == "prismatic")  _jointType[_axisMap[i - 1]] = VOCAB_JOINTTYPE_PRISMATIC;
        else
        {
            yError("Unknown AxisType value %s!", s.c_str());
            _jointType[_axisMap[i - 1]] = VOCAB_JOINTTYPE_UNKNOWN;
            return false;
        }
    }

    double tmp_A2E;
    // Encoder scales
    if (!extractGroup(general, xtmp, "Encoder", "a list of scales for the encoders", _njoints))
    {
        return false;
    }

    for (i = 1; i < xtmp.size(); i++)
    {
        tmp_A2E = xtmp.get(i).asDouble();
        if (tmp_A2E<0)
        {
            yWarning("Encoder parameter should be positive!");
        }

        if (useRawEncoderData)
        {
            _angleToEncoder[i - 1] = 1;
        }
        else
        {
            _angleToEncoder[i - 1] = tmp_A2E;
        }
    }


    // Gearbox
    if (!extractGroup(general, xtmp, "Gearbox_M2J", "The gearbox reduction ratio", _njoints))
    {
        return false;
    }

    int test = xtmp.size();
    for (i = 1; i < xtmp.size(); i++)
    {
        _gearbox[i-1] = xtmp.get(i).asDouble();
        if (_gearbox[i-1]==0) {yError() << "Using a gearbox value = 0 may cause problems! Check your configuration files"; return false;}
    }


    //Gearbox2
    if (!extractGroup(general, xtmp, "Gearbox_E2J", "The gearbox reduction ratio between encoder and joint", _njoints))
    {
        yWarning() << "Missing Gearbox_E2J param. I use default value (1) " ;
        for(int i=0; i<_njoints; i++)
        {
            _gearboxE2J[i] = 1;
        }
    }
    else
    {
        int test = xtmp.size();
        for (i = 1; i < xtmp.size(); i++)
        {
            _gearboxE2J[i-1] = xtmp.get(i).asDouble();
            if (_gearboxE2J[i-1]==0) {yError() << "Using a gearbox value = 0 may cause problems! Check your configuration files"; return false;}
        }
    }

    return true;

}

bool embObjMotionControl::parsePosPid(Bottle &posPidsGroup)
{
    Value &controlUnits=posPidsGroup.find("controlUnits");
    if  (controlUnits.isNull() == false && controlUnits.isString() == true)
    {
        if      (controlUnits.toString()==string("metric_units"))  {yDebug("POSITION_CONTROL: using metric_units");  _positionControlUnits=P_METRIC_UNITS;}
        else if (controlUnits.toString()==string("machine_units")) {yDebug("POSITION_CONTROL: using machine_units"); _positionControlUnits=P_MACHINE_UNITS;}
        else    {yError() << "embObjMotionControl::fromConfig(): POSITION_CONTROL section: invalid controlUnits value";
                    return false;}
    }
    else
    {
        yError() << "embObjMotionControl::fromConfig(): POSITION_CONTROL section: missing controlUnits parameter. Assuming machine_units. Please fix your configuration file.";
        _positionControlUnits=P_MACHINE_UNITS;
    }

    Value &controlLaw=posPidsGroup.find("controlLaw");
    if (controlLaw.isNull() == false && controlLaw.isString() == true)
    {
        string s_controlaw = controlLaw.toString();
        if (s_controlaw==string("joint_pid_v1"))
        {
            if (!parsePositionPidsGroup (posPidsGroup, _pids))
            {
                yError() << "embObjMotionControl::fromConfig(): POSITION_CONTROL section: error detected in parameters syntax";
                return false;
            }
            else
            {
                yDebug("POSITION_CONTROL: using control law joint_pid_v1");
            }
        }
        else if (s_controlaw==string("not_implemented"))
        {
            yDebug() << "found 'not_impelemented' in position control_law. This will terminate robotInterface execution.";
            return false;
        }
        else if (s_controlaw==string("disabled"))
        {
            yDebug() << "found 'disabled' in position control_law. This will terminat robotInterface execution.";
            return false;
        }
        else
        {
            yError() << "Unable to use control law " << s_controlaw << " por position control. Quitting.";
            return false;
        }
    }

}

bool embObjMotionControl::parseVelPid(Bottle &velPidsGroup)
{
    Value &controlUnits=velPidsGroup.find("controlUnits");
    if  (controlUnits.isNull() == false && controlUnits.isString() == true)
    {
        if      (controlUnits.toString()==string("metric_units"))  {yDebug("VELOCITY_CONTROL: using metric_units");  _velocityControlUnits=V_METRIC_UNITS;}
        else if (controlUnits.toString()==string("machine_units")) {yDebug("VELOCITY_CONTROL: using machine_units"); _velocityControlUnits=V_MACHINE_UNITS;}
        else    {yError() << "embObjMotionControl::fromConfig(): VELOCITY_CONTROL section: invalid controlUnits value";
                    return false;}
    }
    else
    {
        yError() << "embObjMotionControl::fromConfig(): VELOCITY_CONTROL section: missing controlUnits parameter. Assuming machine_units. Please fix your configuration file.";
        _velocityControlUnits=V_MACHINE_UNITS;
    }

    Value &controlLaw=velPidsGroup.find("controlLaw");
    if (controlLaw.isNull() == false && controlLaw.isString() == true)
    {
        string s_controlaw = controlLaw.toString();
        if (s_controlaw==string("joint_pid_v1"))
        {
            if (!parseVelocityPidsGroup (velPidsGroup, _vpids))
            {
                yError() << "embObjMotionControl::fromConfig(): VELOCITY_CONTROL section: error detected in parameters syntax";
                return false;
            }
            else
            {
                yDebug("VELOCITY_CONTROL: using control law joint_pid_v1");
            }
        }
        else if (s_controlaw==string("not_implemented"))
        {
            yDebug() << "found 'not_impelemented' in velocity control_law. This will terminate robotInterface execution.";
            return false;
        }
        else if (s_controlaw==string("disabled"))
        {
            yDebug() << "found 'disabled' in velocity control_law. This will terminat robotInterface execution.";
            return false;
        }
        else
        {
            yError() << "Unable to use control law " << s_controlaw << " por velocity control. Quitting.";
            return false;
        }
    }

    return true;


}

bool embObjMotionControl::parseTrqPid(Bottle &trqPidsGroup)
{
    Value &controlUnits=trqPidsGroup.find("controlUnits");
    if  (controlUnits.isNull() == false && controlUnits.isString() == true)
    {
        if      (controlUnits.toString()==string("metric_units"))  {yDebug("TORQUE_CONTROL: using metric_units"); _torqueControlUnits=T_METRIC_UNITS;}
        else if (controlUnits.toString()==string("machine_units")) {yDebug("TORQUE_CONTROL: using metric_units"); _torqueControlUnits=T_MACHINE_UNITS;}
        else    {yError() << "embObjMotionControl::fromConfig(): TORQUE_CONTROL section: invalid controlUnits value";
                    return false;}
    }
    else
    {
        yError() << "embObjMotionControl::fromConfig(): TORQUE_CONTROL section: missing controlUnits parameter. Assuming machine_units. Please fix your configuration file.";
        _torqueControlUnits=T_MACHINE_UNITS;
    }

    if(_torqueControlUnits==T_MACHINE_UNITS)
    {
        yarp::sig::Vector tmpOnes; tmpOnes.resize(_njoints,1.0);
        _torqueControlHelper = new torqueControlHelper(_njoints, tmpOnes.data(), tmpOnes.data());
    }
    else if (_torqueControlUnits==T_METRIC_UNITS)
    {
        _torqueControlHelper = new torqueControlHelper(_njoints, _angleToEncoder, _newtonsToSensor);
    }
    else
    {
        yError() << "embObjMotionControl::fromConfig(): TORQUE_CONTROL section: invalid controlUnits value (_torqueControlUnits=" << _torqueControlUnits << ")";
        return false;
    }

    Value &controlLaw=trqPidsGroup.find("controlLaw");
    if (controlLaw.isNull() == false && controlLaw.isString() == true)
    {
        string s_controlaw = controlLaw.toString();
        if (s_controlaw==string("motor_pid_with_friction_v1"))
        {
            yDebug("TORQUE_CONTROL: using control law motor_pid_with_friction_v1");
            if (!parseTorquePidsGroup (trqPidsGroup, _tpids, _kbemf, _ktau, _filterType))
            {
                yError() << "embObjMotionControl::fromConfig(): TORQUE_CONTROL section: error detected in parameters syntax";
                _torqueControlEnabled = false;
                return false;
            }
            else
            {
                _torqueControlEnabled = true;
            }
        }
        else if (s_controlaw==string("joint_pid_v1"))
        {
            yDebug("TORQUE_CONTROL: using control law joint_pid_v1");
            if (!parseTorquePidsGroup (trqPidsGroup, _tpids, _kbemf, _ktau, _filterType))
            {
                yError() << "embObjMotionControl::fromConfig(): TORQUE_CONTROL section: error detected in parameters syntax";
                _torqueControlEnabled = false;
                return false;
            }
            else
            {
                _torqueControlEnabled = true;
            }
        }
        else if (s_controlaw==string("not_implemented"))
        {
            yDebug() << "torque control not not_implemented on this robot part. Disabling.";
            _torqueControlEnabled = false;
        }
        else if (s_controlaw==string("disabled"))
        {
            yDebug() << "torque control disabled on this robot part.";
            _torqueControlEnabled = false;
        }
        else
        {
            yError() << "Unable to use control law " << s_controlaw << ". Disabling torque control";
            _torqueControlEnabled = false;
        }
    }
    else
    {
        yError() << "Unable to find a valid control law parameter. Disabling torque control";
        _torqueControlEnabled = false;
    }

    return true;

}


bool embObjMotionControl::parseCurrPid(Bottle &currentPidsGroup)
{
     _currentPidsAvailables = true;
    Value &controlUnits=currentPidsGroup.find("controlUnits");
    if  (controlUnits.isNull() == false && controlUnits.isString() == true)
    {
        if (controlUnits.toString()==string("metric_units"))
        {
            yDebug("CURRENT_CONTROL: using metric_units");
        }
        else if (controlUnits.toString()==string("machine_units"))
        {
            yError("embObjMotionControl::fromConfig(): CURRENT_CONTROL: is not possible to use machine_units");
            return false;

        }
        else
        {
            yError() << "embObjMotionControl::fromConfig(): CURRENT_CONTROL section: invalid controlUnits value";
            return false;

        }
    }
    else
    {
        yError() << "embObjMotionControl::fromConfig(): CURRENT_CONTROL section: missing controlUnits parameter. Assuming metric_units. Please fix your configuration file.";
    }



    Value &controlLaw=currentPidsGroup.find("controlLaw");
    if (controlLaw.isNull() == false && controlLaw.isString() == true)
    {
        string s_controlaw = controlLaw.toString();
        if (s_controlaw==string("2foc_feedback"))
        {
            yDebug("CURRENT_CONTROL: using control law motor 2foc_feedback");
            if (!parseCurrentPidsGroup (currentPidsGroup, _cpids))
            {
                yError() << "embObjMotionControl::fromConfig(): CURRENT_CONTROL section: error detected in parameters syntax";
                return false;
            }
        }
        else if (s_controlaw==string("not_implemented"))
        {
            yDebug() << "current control not not_implemented on this robot part. Disabling.";
        }
        else if (s_controlaw==string("disabled"))
        {
            yDebug() << "current control disabled on this robot part.";
        }
        else
        {
            yError() << "Unable to use control law " << s_controlaw << ". Disabling current control";
        }
    }
    else
    {
        yDebug() << "Unable to find a valid control law parameter. Disabling current control";
    }

    return true;

}

bool embObjMotionControl::parseLimitsGroup( Bottle &limits)
{
    int i;
    Bottle xtmp;

    // current limit
    if (!extractGroup(limits, xtmp, "motorOverloadCurrents","a list of current limits", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) _currentLimits[i-1].overloadCurrent=xtmp.get(i).asDouble();

    // nominal current
    if (!extractGroup(limits, xtmp, "motorNominalCurrents","a list of nominal current limits", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) _currentLimits[i-1].nominalCurrent =xtmp.get(i).asDouble();

    // peak current
    if (!extractGroup(limits, xtmp, "motorPeakCurrents","a list of peak current limits", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) _currentLimits[i-1].peakCurrent=xtmp.get(i).asDouble();

    // max limit
    if (!extractGroup(limits, xtmp, "jntPosMax","a list of user maximum angles (in degrees)", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) _limitsMax[i-1]=xtmp.get(i).asDouble();

    // min limit
    if (!extractGroup(limits, xtmp, "jntPosMin","a list of user minimum angles (in degrees)", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) _limitsMin[i-1]=xtmp.get(i).asDouble();

    // max hardware limit
    if (!extractGroup(limits, xtmp, "hardwareJntPosMax","a list of hardware maximum angles (in degrees)", _njoints))
    {
         yWarning() << "embObjMotionControl: missing hardwareJntPosMax param. Values of jntPosMax param will be used like hardware limits ";
         for(i=0; i<_njoints; i++) _hwLimitsMax[i]=_limitsMax[i];
    }
    else
    {
        for(i=1; i<xtmp.size(); i++) _hwLimitsMax[i-1]=xtmp.get(i).asDouble();

        //check hardware limits are bigger then user limits
        for(i=0; i<_njoints; i++)
        {
            if(_limitsMax[i] > _hwLimitsMax[i])
            {
                yError() << "embObjMotionControl: user has set a limit  bigger then hardware limit!. Please check jntPosMax.";
                return false;
            }
        }
    }

     // min hardware limit
    if (!extractGroup(limits, xtmp, "hardwareJntPosMin","a list of hardware minimum angles (in degrees)", _njoints))
    {
         yWarning() << "embObjMotionControl: missing hardwareJntPosMin param. Values of jntPosMin param will be used like hardware limits ";
         for(i=0; i<_njoints; i++) _hwLimitsMin[i]=_limitsMin[i];
    }
    else
    {
        for(i=1; i<xtmp.size(); i++) _hwLimitsMin[i-1]=xtmp.get(i).asDouble();

        //check hardware limits are bigger then user limits
        for(i=0; i<_njoints; i++)
        {
            if(_limitsMin[i] < _hwLimitsMin[i])
            {
                yError() << "embObjMotionControl: user has set a limit  bigger then hardware limit!. Please check jntPosMin.";
                return false;
            }
        }

    }
    // Rotor max limit
    if (!extractGroup(limits, xtmp, "rotorPosMax","a list of maximum rotor angles (in degrees)", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) _rotorlimits_max[i-1]=xtmp.get(i).asDouble();

    // joint Velocity command max limit
    if (!extractGroup(limits, xtmp, "jntVelMax", "a list of maximum velocities for the joints (in degrees/s)", _njoints))
        return false;
    else
        for (i = 1; i<xtmp.size(); i++) _maxJntCmdVelocity[i - 1] = xtmp.get(i).asDouble();

    // Rotor min limit
    if (!extractGroup(limits, xtmp, "rotorPosMin","a list of minimum roto angles (in degrees)", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) _rotorlimits_min[i-1]=xtmp.get(i).asDouble();

    // Motor pwm limit
    if (!extractGroup(limits, xtmp, "motorPwmLimit","a list of motor PWM limits", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++)
        {
            _motorPwmLimits[i-1]=xtmp.get(i).asDouble();
            if(_motorPwmLimits[i-1]<0)
            {
                yError() << "motorPwmLimit should be a positive value";
                return false;
            }
        }
    return true;

}

bool embObjMotionControl::parseTimeoutsGroup( Bottle &timeouts)
{
    Bottle xtmp;
    int i;

    xtmp.clear();
    if (!extractGroup(timeouts, xtmp, "Velocity", "a list of timeout to be used in the vmo control", _njoints))
    {
        yWarning() << "Using default velocity Timeout=100, i.e 0.1s\n";
        for(i=1; i<_njoints+1; i++)
            _velocityTimeout[i-1] = 100;   //Default value
    }
    else
    {
        for(i=1; i<xtmp.size(); i++)
            _velocityTimeout[i-1]=xtmp.get(i).asInt();
    }

    return true;
}

bool embObjMotionControl::parse2FocGroup(Bottle &focGroup)
{
    Bottle xtmp;
    int i;

    if (!extractGroup(focGroup, xtmp, "HasHallSensor", "HasHallSensor 0/1 ", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            _hasHallSensor[i - 1] = xtmp.get(i).asInt();
    }
    if (!extractGroup(focGroup, xtmp, "HasTempSensor", "HasTempSensor 0/1 ", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            _hasTempSensor[i - 1] = xtmp.get(i).asInt();
    }
    if (!extractGroup(focGroup, xtmp, "HasRotorEncoder", "HasRotorEncoder 0/1 ", _njoints))
    {
        return false;
    }
    else
    {

        for (i = 1; i < xtmp.size(); i++)
            _hasRotorEncoder[i - 1] = xtmp.get(i).asInt();
    }
    if (!extractGroup(focGroup, xtmp, "HasRotorEncoderIndex", "HasRotorEncoderIndex 0/1 ", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            _hasRotorEncoderIndex[i - 1] = xtmp.get(i).asInt();
    }

    if (!extractGroup(focGroup, xtmp, "RotorIndexOffset", "RotorIndexOffset", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            _rotorIndexOffset[i - 1] = xtmp.get(i).asInt();
    }

    // Number of motor poles
    if (!extractGroup(focGroup, xtmp, "MotorPoles", "MotorPoles", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            _motorPoles[i - 1] = xtmp.get(i).asInt();
    }

    if (!extractGroup(focGroup, xtmp, "HasSpeedEncoder", "HasSpeedEncoder 0/1 ", _njoints))
    {
        yWarning () << "missing param HasSpeedEncoder";
        // optional by now
        //return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            _hasSpeedEncoder[i - 1] = xtmp.get(i).asInt();
    }

    return true;

}


bool embObjMotionControl::parseCouplingGroup(Bottle &coupling)
{

    Bottle xtmp;
    int  fixedMatrix4X4Size = 16;
    int  fixedMatrix4X6Size = 24;
    bool formaterror =false;

    // matrix J2M
    if (!extractGroup(coupling, xtmp, "matrixJ2M", "matrixJ2M ", fixedMatrix4X4Size))
    {
        return false;
    }

    if(false == parser->convert(xtmp, controller.matrixJ2M, formaterror, fixedMatrix4X4Size))
    {
        yError() << "embObjMotionControl::parseCouplingGroup() has detected an illegal format for some of the values of CONTROLLER.matrixJ2M";
        return false;
    }


    // matrix E2J
    if (!extractGroup(coupling, xtmp, "matrixE2J", "matrixE2J ", fixedMatrix4X6Size))
    {
        return false;
    }

    formaterror = false;
    if(false == parser->convert(xtmp, controller.matrixE2J, formaterror, fixedMatrix4X6Size))
    {
        yError() << "embObjMotionControl::parseCouplingGroup() has detected an illegal format for some of the values of CONTROLLER.matrixE2J";
        return false;
    }


    // matrix M2J
    if (!extractGroup(coupling, xtmp, "matrixM2J", "matrixM2J ", fixedMatrix4X4Size))
    {
        return false;
    }

    formaterror = false;
    if( false == parser->convert(xtmp, controller.matrixM2J, formaterror, fixedMatrix4X4Size))
    {
        yError() << "embObjMotionControl::parseCouplingGroup() has detected an illegal format for some of the values of CONTROLLER.matrixM2J";
        return false;
    }

    return true;

}

bool embObjMotionControl::parseJointsetCfgGroup(Bottle &jointsetcfg)
{
    Bottle xtmp;
    int numofsets = 0;
    vector<int> listofjoints;

    if(!extractGroup(jointsetcfg, xtmp, "numberofsets", "number of sets ", 1))
    {
        return  false;
    }

    numofsets = xtmp.get(1).asInt();

    if((0 == numofsets) || (numofsets > _njoints))
    {
        yError() << "Number of jointsets is not correct. it should belong to (1, " << _njoints << ")";
        return false;
    }

    //reset joint2jointset map
    jointsets_prop.numofjointsets = numofsets;
    jointsets_prop.joint2set.resize(numofsets, eomc_jointSetNum_none);

    //reset jointsets' cfg
    eOmc_jointset_configuration_t cfg_reset = {0};
    jointsets_prop.jointset_cfgs.resize(numofsets, cfg_reset);

    set2joint.resize(numofsets);
    for(unsigned int j=0;j<numofsets;j++)
    {
        char jointset_string[80];
        sprintf(jointset_string, "JOINTSET_%d", j);
        Bottle  js_cfg;
        bool formaterror = false;

        if(!extractGroup(jointsetcfg, js_cfg, jointset_string, jointset_string, 1))
        {
            return  false;
        }

        //list of joints
        Bottle b_listofjoints=js_cfg.findGroup("listofjoints", "list of joints");
        if (b_listofjoints.isNull())
        {
            yError () << "listofjoints parameter not found";
            return false;
        }

        set2joint[j].resize(b_listofjoints.size()-1);


        for (int k = 1; k < b_listofjoints.size(); k++)
        {
            int jointofthisset = b_listofjoints.get(k).asInt();
            jointsets_prop.joint2set[jointofthisset] = j;
            set2joint[j][k-1] = jointofthisset;
        }

        //constraints
        if(!extractGroup(js_cfg, xtmp, "constraint", "type of jointset constraint ", 1))
        {
            return  false;
        }

        eOmc_jsetconstraint_t constraint;
        if(!parser->convert(xtmp.get(1).asString(), constraint, formaterror))
        {
            return false;
        }
        jointsets_prop.jointset_cfgs[j].constraints.type = constraint;


        //param1
        if(!extractGroup(js_cfg, xtmp, "param1", "param1 of jointset constraint ", 1))
        {
            return  false;
        }
        jointsets_prop.jointset_cfgs[j].constraints.param1 = xtmp.get(1).asDouble();

        //param2
        if(!extractGroup(js_cfg, xtmp, "param2", "param2 of jointset constraint ", 1))
        {
            return  false;
        }
        jointsets_prop.jointset_cfgs[j].constraints.param2 = xtmp.get(1).asDouble();

    }

    //debug prints
    for (int xx=0; xx<parser->mc_service.properties.numofjoints; xx++)
        yError() << " JOINT SET MAP: J " << xx << "SET " << parser->mc_service.properties.joint2set[xx];
}

bool embObjMotionControl::verifyControlLawconsistencyinJointSet(string *controlLaw)
{
    for(int s=0; s<set2joint.size(); s++)
    {
        for(int k=1; k<set2joint[s].size(); k++)
        {
            int otherjoint = set2joint[set][k];
            if(controlLaw[0] != controlLaw[k])
            {
                yError() << "Joints beloning to same set must be have same control law";
                return false;
            }
        }
    }
    return true;
}

bool embObjMotionControl::parserControlsGroup(Bottle &controlsGroup)
{
    Bottle xtmp;
    int i;

    if (!extractGroup(controlsGroup, xtmp, "positionControl", "Position Control ", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            _posistionControlLaw[i - 1] = xtmp.get(i).asString();

        // verify that joint beloning to same jointset have same controlLaw
        if(!verifyControlLawconsistencyinJointSet(_posistionControlLaw))
        {
             yError() << "Joints beloning to same set must be have same position control law";
             return false;
        }

    }

    if (!extractGroup(controlsGroup, xtmp, "velocityControl", "Velocity Control ", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            _velocityControlLaw[i - 1] = xtmp.get(i).asString();

        // verify that joint beloning to same jointset have same controlLaw
        if(!verifyControlLawconsistencyinJointSet(_velocityControlLaw))
        {
             yError() << "Joints beloning to same set must be have same velocity control law";
             return false;
        }
    }

    if (!extractGroup(controlsGroup, xtmp, "torqueControl", "Torque Control ", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            _torqueControlLaw[i - 1] = xtmp.get(i).asString();

        if(!verifyControlLawconsistencyinJointSet(_torqueControlLaw))
        {
             yError() << "Joints beloning to same set must be have same torque control law";
             return false;
        }
    }
    return true;

}

bool embObjMotionControl::parsercontrolUnitsType(Bottle &bPid, GenericControlUnitsType_t &unitstype)
{

    Value &controlUnits=bPid.find("controlUnits");
    if(controlUnits.isNull())
    {
        yError() << "embObjMotioncontrol: missing controlUnits parameter";
        return false;
    }
    if(!controlUnits.isString())
    {
        yError() << "embObjMotioncontrol: controlUnits parameter is not a string";
        return false;
    }


    if(controlUnits.toString()==string("metric_units"))
    {
        unitstype = METRIC_UNITS;
        return true;
    }
    else if(controlUnits.toString()==string("machine_units"))
    {
        unitstype = MACHINE_UNITS;
        return true;
    }
    else
    {
        yError() << "embObjMotionControl:invalid controlUnits value: " <<controlUnits.toString().c_str();
        return false;
    }

}


bool embObjMotionControl::parsePid_inPos_outPwm(Bottle &b_pid, string controlLaw)
{
    bool alreadyParsed = false;
    map<string, Pid_Algorithm>::iterator it = posAlgoMap.find(controlLaw);
    if(it != posAlgoMap.end())
        alreadyParsed = true;

    if(alreadyParsed)
        return;

    Pid_Algorithm_simple *pidSimple_ptr = new Pid_Algorithm_simple(_njoints);
    pidSimple_ptr->type = PidAlgo_simple;

    GenericControlUnitsType_t unitstype;
    if(!parsercontrolUnitsType(b_pid, unitstype))
        retrun false;
    pidSimple_ptr->ctrlUnitsType = unitstype;

    if(!parsePidsGroup(b_pid, pidSimple_ptr->pid, string("pos_")))
        return false;

    posAlgoMap[controlLaw] = pidSimple_ptr;
}


bool embObjMotionControl::parsePid_inTrq_outPwm(Bottle &b_pid, string controlLaw)
{
    bool alreadyParsed = false;
    map<string, Pid_Algorithm>::iterator it = trqAlgoMap.find(controlLaw);
    if(it != trqAlgoMap.end())
        alreadyParsed = true;

    if(alreadyParsed)
        return true;

    Pid_Algorithm_simple *pidSimple_ptr = new Pid_Algorithm_simple(_njoints);
    pidSimple_ptr->type = PidAlgo_simple;

    GenericControlUnitsType_t unitstype;
    if(!parsercontrolUnitsType(b_pid, unitstype))
        retrun false;
    pidSimple_ptr->ctrlUnitsType = unitstype;

    if(!parsePidsGroup(b_pid, pidSimple_ptr->pid, string("trq_")))
        return false;

    Bottle xtmp;
    //torque specific params
    if (!extractGroup(b_pid, xtmp, "kbemf", "kbemf parameter", _njoints))         return false; for (j=0; j<_njoints; j++) kbemf[j]      = xtmp.get(j+1).asDouble();
    if (!extractGroup(b_pid, xtmp, "ktau", "ktau parameter", _njoints))           return false; for (j=0; j<_njoints; j++) ktau[j]       = xtmp.get(j+1).asDouble();
    if (!extractGroup(b_pid, xtmp, "filterType", "filterType param", _njoints))   return false; for (j=0; j<_njoints; j++) filterType[j] = xtmp.get(j+1).asInt();

    //conversion from metric to machine units (if applicable)
    for (j=0; j<_njoints; j++)
    {
        pidSimple_ptr->pid[j].kp = pidSimple_ptr->pid[j].kp / _torqueControlHelper->getNewtonsToSensor(j);  //[PWM/Nm]
        pidSimple_ptr->pid[j].ki = pidSimple_ptr->pid[j].ki / _torqueControlHelper->getNewtonsToSensor(j);  //[PWM/Nm]
        pidSimple_ptr->pid[j].kd = pidSimple_ptr->pid[j].kd / _torqueControlHelper->getNewtonsToSensor(j);  //[PWM/Nm]
        pidSimple_ptr->pid[j].stiction_up_val   = pidSimple_ptr->pid[j].stiction_up_val   * _torqueControlHelper->getNewtonsToSensor(j); //[Nm]
        pidSimple_ptr->pid[j].stiction_down_val = pidSimple_ptr->pid[j].stiction_down_val * _torqueControlHelper->getNewtonsToSensor(j); //[Nm]
    }


    trqAlgoMap[controlLaw] = pidSimple_ptr;

    return true;
}

bool embObjMotionControl::parsePidPos_withInnerVelPid(Bottle &b_pid, string controlLaw)
{

    bool alreadyParsed = false;
    map<string, Pid_Algorithm>::iterator it = posAlgoMap.find(controlLaw);
    if(it != posAlgoMap.end())
        alreadyParsed = true;

    if(alreadyParsed)
        return;

    PidAlgorithm_VelocityInnerLoop *pidInnerVel_ptr = new PidAlgorithm_VelocityInnerLoop(_njoints);
    pidInnerVel_ptr->type = PIdAlgo_velocityInnerLoop;

    GenericControlUnitsType_t unitstype;
    if(!parsercontrolUnitsType(b_pid, unitstype))
        return false;
    pidSimple_ptr->ctrlUnitsType = unitstype;

    if(!parsePidsGroup(b_pid, pidInnerVel_ptr->extPid, string("pos_")))
        return false;
    if(!parsePidsGroup(b_pid, pidInnerVel_ptr->innerVelPid, string("vel_")))
        return false;

    posAlgoMap[controlLaw] = *pidInnerVel_ptr;

    return true;
}



bool embObjMotionControl::parseSelectedPositionControl(Bottle config)
{
    for(int i=0; i<_njoints; i++)
    {
        // 1) verify that selected control law is defined in file
        Bottle posControlLaw = config.findGroup(_posistionControlLaw[i]);
        if (posControlLaw.isNull())
        {
            yError() << "Missing " << _posistionControlLaw[i].c_str();
            return false;
        }

        // 2) read control_law
        Value &controlLaw=posControlLaw.find("controlLaw");
        if( (controlLaw.isNull()) || (! controlLaw.isString()) )
        {
            yError() << "Unable read control law parameter for " << _posistionControlLaw[i].c_str() <<". Quitting.";
            return false;
        }

        string s_controlaw = controlLaw.toString();
        if (s_controlaw==string("Pid_inPos_outPwm"))
        {
            if (!parsePid_inPos_outPwm(posControlLaw, _posistionControlLaw[i]))
            {
                yError() << "embObjMotionControl::fromConfig(): format error in Pid_inPos_outPwm";
                return false;
            }

        }
        else if (s_controlaw==string("PidPos_withInnerVelPid"))
        {
            if (!parsePidPos_withInnerVelPid(posControlLaw))
            {
                yError() << "embObjMotionControl::fromConfig(): format error in PidPos_withInnerVelPid";
                return false;
            }

        }
        else
        {
            yError() << "Unable to use control law " << s_controlaw << " por position control. Quitting.";
            return false;
        }

    }
    return true;

}


bool embObjMotionControl::parseSelectedTorqueControl(Bottle config)
{
    int set;
    for(int i=0; i<_njoints; i++)
    {
        if(_torqueControlLaw[i] == "none")
        {
            jointsets_prop.jointset_cfgs[set].candotorquecontrol = false;
            continue;
        }
        // 1) verify that selected control law is defined in file
        Bottle trqControlLaw = config.findGroup(_torqueControlLaw[i]);
        if (trqControlLaw.isNull())
        {
            yError() << "Missing " << _torqueControlLaw[i].c_str();
            return false;
        }

        set = jointsets_prop.joint2set[i];
        jointsets_prop.jointset_cfgs[set].candotorquecontrol = true;

        // 2) read control_law
        Value &controlLaw=trqControlLaw.find("controlLaw");
        if( (controlLaw.isNull()) || (! controlLaw.isString()) )
        {
            yError() << "Unable read control law parameter for " << _torqueControlLaw[i].c_str() <<". Quitting.";
            return false;
        }

        string s_controlaw = controlLaw.toString();
        if (s_controlaw==string("Pid_inPos_outPwm"))
        {
            if (!parsePid_inPos_outPwm(trqControlLaw, _torqueControlLaw[i]))
            {
                yError() << "embObjMotionControl::fromConfig(): format error in Pid_inPos_outPwm";
                return false;
            }

        }
        else if (s_controlaw==string("PidPos_withInnerVelPid"))
        {
            if (!parsePidPos_withInnerVelPid(trqControlLaw))
            {
                yError() << "embObjMotionControl::fromConfig(): format error in PidPos_withInnerVelPid";
                return false;
            }

        }
        else
        {
            yError() << "Unable to use control law " << s_controlaw << " por position control. Quitting.";
            return false;
        }

    }
    return true;

}


bool embObjMotionControl::getCorrectPidForEachJoint(void)
{
    Pid_Algorithm *pidAlgo_ptr;
    Pid_Algorithm *vpidAlgo_ptr;
    Pid_Algorithm *tpidAlgo_ptr;

    map<string, Pid_Algorithm>::iterator it;

    for(int j=0; j<_njoints; j++)
    {
        //get position pid
        it = posAlgoMap.find(_posistionControlLaw[i]);
        if(it == posAlgoMap.end())
        {
            yError() << "Enxpected error!";
            return false;
        }

        Pid_Algorithm *pidAlgo_ptr = posAlgoMap[_posistionControlLaw[i]];

        //get velocity pid
        if(_velocityControlLaw[i] == "none")
            Pid_Algorithm *vpidAlgo_ptr = NULL;
        else
        {
            it = velAlgoMap.find(_velocityControlLaw[i]);
            if(it == velAlgoMap.end())
            {
                yError() << "Enxpected error!";
                return false;
            }

            vpidAlgo_ptr = velAlgoMap[_velocityControlLaw[i]];
        }

        //get torque pid
        if(_torqueControlLaw[i] == "none")
            Pid_Algorithm *tpidAlgo_ptr = NULL;
        else
        {
            it = trqAlgoMap.find(_torqueControlLaw[i]);
            if(it == trqAlgoMap.end())
            {
                yError() << "Enxpected error!";
                return false;
            }

            tpidAlgo_ptr = trqAlgoMap[_torqueControlLaw[i]];
        }

        //verifico che i giunti abbiamo lo stesso algoritmo per pid posizione, velocita' e torque
        bool allAlgoAreEqual = false;
        if( (vpidAlgo_ptr) && (pidAlgo_ptr->type == vpidAlgo_ptr->type) &&
            (tpidAlgo_ptr) && (pidAlgo_ptr->type == tpidAlgo_ptr->type)
          )
            allAlgoAreEqual = false;

         if(!allAlgoAreEqual)
         {
             yError() << "Not all algo are equal!";
             return false;
         }


 ///////////////// PID POSITION
        switch(pidAlgo_ptr->type )
        {
            case(PidAlgo_simple):
            {
                Pid_Algorithm_simple *pidAlgo_simple_ptr = (Pid_Algorithm_simple*)pidAlgo_ptr;
                _pids[j] = pidAlgo_simple_ptr->pid[j];

                if(vpidAlgo_simple_ptr)
                    _vpids[j] = vpidAlgo_simple_ptr->pid[j];
                else
                    _vpids[j] = 0;

                if(tpidAlgo_simple_ptr)
                    _tpids[j] = tpidAlgo_simple_ptr->pid[j];
                else
                    _tpids[j] = 0;


            }break;

            case(PIdAlgo_velocityInnerLoop):
            {
                if(!verifyAllVelocityPIdOfSameJointAreEqual())
                    return false;
                PidAlgorithm_VelocityInnerLoop *pidAlgo_innerVelLoop_ptr = (PidAlgorithm_VelocityInnerLoop*)pidAlgo_ptr;
                _pids[j] = pidAlgo_innerVelLoop_ptr->extPid[j];
                _vpids[j] = pidAlgo_simple_ptr->innerVelPid[j];
                if(tpidAlgo_simple_ptr)
                    _tpids[j] = tpidAlgo_simple_ptr->pid[j];
                else
                    _tpids[j] = 0;
            }break;

            default:
                yError() << "Enxpected error!";
                return false;
        }
    }


}

bool embObjMotionControl::saveCouplingsData(void)
{
    switch(serviceConfig.ethservice.configuration.type)
    {
        case eomn_serv_MC_foc:
        {
            eOmc_4jomo_coupling_t *jc_dest = &(serviceConfig.ethservice.configuration.data.mc.foc_based.jomocoupling);
        } break;

        case eomn_serv_MC_mc4plus:
        {
            eOmc_4jomo_coupling_t *jc_dest = &(serviceConfig.ethservice.configuration.data.mc.mc4plus_based.jomocoupling);
        } break;

        case eomn_serv_MC_mc4plusmais:
        {
            eOmc_4jomo_coupling_t *jc_dest = &(serviceConfig.ethservice.configuration.data.mc.mc4plusmais_based.jomocoupling);

        } break;
        default:
        {
            return false;
        }
    }

    memset(jc_dest, 0, sizeof(eOmc_4jomo_coupling_t));

    // very important: so far, the fw in Controller.c must find eomc_jointSetNum_none in un-used entries, even if we have less than 4 joints.
    memset(jc_dest->joint2set, eomc_jointSetNum_none, sizeof(jc_dest->joint2set));

    for(int i=0; i<jointsets_prop.joint2set.size(); i++)
    {
        jc_dest->joint2set[i] = jointsets_prop.joint2set[i];
    }

    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            jc_dest->joint2motor[i][j] = eo_common_float_to_Q17_14(controller.matrixJ2M[4*i+j]);
            jc_dest->motor2joint[i][j] = eo_common_float_to_Q17_14(controller.matrixM2J[4*i+j]);
        }
    }


    for(int r=0; r<4; r++)
    {
        for(int c=0; c<6; c++)
        {
            jc_dest->encoder2joint[r][c] = eo_common_float_to_Q17_14(controller.matrixE2J[6*r+c]);
        }
    }

    for(int s=0; s< jointsets_prop.numofjointsets; s++)
    {
        memcpy(&jc_dest->jsetcfg[s]  , &jointsets_prop.jointset_cfgs[s], sizeof(eOmc_jointset_configuration_t));
    }

    return true;

}


bool embObjMotionControl::fromConfig_Step2(yarp::os::Searchable &config)
{
    Bottle xtmp;
    int i,j;

#warning VALE: leggi valore  mc_service.properties.jointset_cfgs[i].pidoutputtype  e candotorquecontrol

    Bottle general = config.findGroup("GENERAL");
    if (general.isNull())
    {
       yError() << "Missing General group";
       return false;
    }

    if(!parseGeneralMecGroup(general))
        return false;

     //_newtonsToSensor not depends more on joint. Since now we use float number to change torque values with firmware, we can use micro Nm in order to have a good sensitivity.
    for (i = 0; i < _njoints; i++)
    {
        _newtonsToSensor[i] = 1000000.0f; // conversion from Nm into microNm

    }

    eOmn_serv_type_t mc_serv_type = serviceConfig.ethservice.configuration.type;
    if( (mc_serv_type == eomn_serv_MC_foc) ||
        (mc_serv_type == eomn_serv_MC_mc4plus) ||
        (mc_serv_type == eomn_serv_MC_mc4plusmais)
      )
    {

        ////// COUPLINGS
        {
            Bottle couplings = config.findGroup("COUPLINGS");
            if (couplings.isNull())
            {
                yError() << "Missing Coupling group";
                return false;
            }

            if(!parseCouplingGroup(couplings))
                return false;

        }

        ////// JOINTSET_CFG
        {
            Bottle jointsetCfg = config.findGroup("JOINTSET_CFG");
            if (jointsetCfg.isNull())
            {
                yError() << "Missing JOINTSET_CFG group";
                return false;
            }

            if(!parseJointsetCfgGroup(jointsetCfg))
                return false;

        }

        if(!saveCouplingsData())
            return false;
    }

    ///// CONTROLS
    {
        Bottle controlsGroup = config.findGroup("CONTROLS", "Configuration of used control laws ");
        if(controlsGroup.isNull())
        {
            yError() <<"embObjMotionControl::fromConfig(): Error: no CONTROLS group found in config file, returning";
            return false;
        }
        if(!parserControlsGroup(controlsGroup))
            return false;

        if(!parseSelectedPositionControl(config))
            return false;
        if(!parseSelectedVelocityControl(config))
            return false;
        if(!parseSelectedTorqueControl(config))
            return false;

        //quando arrivo qui ho popolato tutte le mie mappe
        if(!getCorrectPidForEachJoint())
            return false;
    }

    ////// POSITION PIDS
    {
        Bottle posPidsGroup;
        posPidsGroup=config.findGroup("POSITION_CONTROL", "Position Pid parameters new format");
        if (posPidsGroup.isNull())
        {
            yError() <<"embObjMotionControl::fromConfig(): Error: no POS_PIDS group found in config file, returning";
            return false;
        }
        if(!parsePosPid(posPidsGroup))
            return false;
    }


    ////// VELOCITY PIDS
    {
        Bottle velPidsGroup;
        velPidsGroup=config.findGroup("VELOCITY_CONTROL", "VelocityPid parameters new format");
        if (velPidsGroup.isNull())
        {
            yWarning() <<"embObjMotionControl::fromConfig(): Warning: no VEL_PIDS group found in config file, skipping";
            //return false; // not mandatory by now
        }
        else
        {
            if(!parseVelPid(velPidsGroup))
                return false;
        }
    }

    ////// TORQUE PIDS
    {
        Bottle trqPidsGroup;
        trqPidsGroup=config.findGroup("TORQUE_CONTROL", "Torque control parameters new format");
        if (trqPidsGroup.isNull())
        {
            yError() <<"embObjMotionControl::fromConfig(): Error: no TORQUE_CONTROL group found in config file";
            _torqueControlEnabled = false;
            return false; //torque control group is mandatory
        }

        if(!parseTrqPid(trqPidsGroup))
            return false;

    }


    ////// CURRENT PIDS
    {
        Bottle currentPidsGroup;
        currentPidsGroup=config.findGroup("CURRENT_CONTROL", "Current control parameters");
        if(currentPidsGroup.isNull())
        {
             yDebug() <<"embObjMotionControl::fromConfig(): no CURRENT_CONTROL group found in config file";
            //return true; //current control group is not mandatory
            _currentPidsAvailables = false;
        }
        else
        {
            if(!parseCurrPid(currentPidsGroup))
                return false;
        }

    }

    ////// IMPEDANCE PARAMETERS
    {
        Bottle impedanceGroup;
        impedanceGroup=config.findGroup("IMPEDANCE","IMPEDANCE parameters");
        if (impedanceGroup.isNull()==false)
        {
            if(verbosewhenok)
            {
                yDebug() << "embObjMotionControl::fromConfig() detected that IMPEDANCE parameters section is found";
            }
            if (!parseImpedanceGroup_NewFormat (impedanceGroup, _impedance_params))
            {
                yError("IMPEDANCE section: error detected in parameters syntax\n");
                return false;
            }
            else
            {
                yInfo("IMPEDANCE section: parameters successfully loaded\n");
            }
        }
        else
        {
            yError() <<"embObjMotionControl::fromConfig(): Error: no IMPEDANCE group found in config file, returning";
            return false;
        }

    }

    ////// IMPEDANCE LIMITS DEFAULT VALUES (UNDER TESTING)
    for(j=0; j<_njoints; j++)
    {
        // got from canBusMotionControl, ask to Randazzo Marco
        _impedance_limits[j].min_damp=  0.001;
        _impedance_limits[j].max_damp=  9.888;
        _impedance_limits[j].min_stiff= 0.002;
        _impedance_limits[j].max_stiff= 9.889;
        _impedance_limits[j].param_a=   0.011;
        _impedance_limits[j].param_b=   0.012;
        _impedance_limits[j].param_c=   0.013;
    }



    /////// LIMITS
    {
    Bottle &limits=config.findGroup("LIMITS");
    if (limits.isNull())
    {
        yWarning() << "embObjMotionControl::fromConfig() detected that Group LIMITS is not found in configuration file";
        return false;
    }
    if(!parseLimitsGroup(limits))
        return false;


    }

     /////// [2FOC]
     {
         bool mcservice_is_2foc = true;
        #warning defnisci mcservice_is_2foc!!!
        if(mcservice_is_2foc)
        {
            Bottle &focGroup=config.findGroup("2FOC");
            if (focGroup.isNull() )
            {
                yWarning() << "embObjMotionControl::fromConfig() detected that Group 2FOC is not found in configuration file";
                return false;
            }
            if(!parse2FocGroup(focGroup))
                return false;
        }
     }

    /////// [TIMEOUTS]
    {
        Bottle timeoutsGroup =config.findGroup("TIMEOUTS");
        if(timeoutsGroup.isNull())
        {
            yWarning() << "embObjMotionControl::fromConfig(): Warning: no TIMEOUTS group found in config file, default values will be used.";
            for(i=1; i<_njoints+1; i++)
                _velocityTimeout[i-1] = 1000;   //Default value
        }
        else
        {
            if(!parseTimeoutsGroup(timeoutsGroup))
                return false;
        }
    }

    return true;
}


// use this one for ... service configuration
bool embObjMotionControl::fromConfig_readServiceCfg(yarp::os::Searchable &config)
{

#if defined(EMBOBJMC_USESERVICEPARSER)
    if(false == parser->parseService(config, serviceConfig))
    {
        yError("embObjMotionControl::fromConfig() cannot parse service");
        return false;
    }

    //now parser read encoders' resolutions also.
    //so here I save in embObMotioncontrol memory encoders's resolution

    for(int i=0; i<_njoints; i++)
    {
        _jointEncoderRes[i]  = parser->mc_service.properties.encoder1s[i].resolution;
        _jointEncoderType[i] = parser->mc_service.properties.encoder1s[i].desc.type;
        _rotorEncoderRes[i]  = parser->mc_service.properties.encoder2s[i].resolution;
        _rotorEncoderType[i] = parser->mc_service.properties.encoder2s[i].desc.type;
    }

#endif

    return true;
}



bool embObjMotionControl::fromConfig(yarp::os::Searchable &config)
{
    _njoints = fromConfig_NumOfJoints(config);

    if(0 == _njoints)
    {
        yError() << "embObjMotionControl::fromConfig(): detected _njoints = " << _njoints;
        return false;
    }

    // we have number of joints inside _njoints. we allocate all required buffers
    if(!alloc(_njoints))
    {
        yError() << "embObjMotionControl::fromConfig(): alloc() failed for _njoints = " << _njoints;
        return false;
    }

    // first step of configuration
    if(false == fromConfig_readServiceCfg(config))
    {
        return false;
    }

    if(false == fromConfig_getGeneralInfo(config)) //get general info: useRawEncoderData, useLiitedPwm, etc....
    {
        return false;
    }

    // second step of configuration
    if(false == fromConfig_Step2(config))
    {
        return false;
    }

    // third step of configuration


    return true;
}


bool embObjMotionControl::init()
{
    eOprotID32_t protid = 0;

    /////////////////////////////////////////////////
    //SEND DISABLE TO ALL JOINTS
    /////////////////////////////////////////////////

    for(int logico=0; logico< _njoints; logico++)
    {
        int fisico = _axisMap[logico];
        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, fisico, eoprot_tag_mc_joint_cmmnds_controlmode);
        eOenum08_t controlMode = eomc_controlmode_cmd_idle;

        if(!res->addSetMessage(protid, (uint8_t *) &controlMode))
        {
            yError() << "embObjMotionControl::init() had an error while setting eomc_controlmode_cmd_idle in BOARD" << res->getName() << "with IP" << res->getIPv4string();
            // return(false); i dont return false. because even if a failure, that is not a severe error.
            // MOREOVER: to verify we must read the status of the joint and NOT the command ... THINK OF IT
        }
    }

    Time::delay(0.010);


    ////////////////////////////////////////////////
    // configure the regular rops
    ////////////////////////////////////////////////

    vector<eOprotID32_t> id32v(0);
    for(int n=0; n<_njoints; n++)
    {
        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, n, eoprot_tag_mc_joint_status_core);
        id32v.push_back(protid);
        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, n, eoprot_tag_mc_motor_status_basic);
        id32v.push_back(protid);
    }


    if(false == res->serviceSetRegulars(eomn_serv_category_mc, id32v))
    {
        yError() << "embObjMotionControl::init() fails to add its variables to regulars in BOARD" << res->getName() << "with IP" << res->getIPv4string() << ": cannot proceed any further";
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjMotionControl::init() added" << id32v.size() << "regular rops to BOARD" << res->getName() << "with IP" << res->getIPv4string();
            char nvinfo[128];
            for(unsigned int r=0; r<id32v.size(); r++)
            {
                uint32_t id32 = id32v.at(r);
                eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
                yDebug() << "\t it added regular rop for" << nvinfo;
            }
        }
    }

    Time::delay(0.005);  // 5 ms


    //////////////////////////////////////////
    // invia la configurazione dei GIUNTI   //
    //////////////////////////////////////////
    for(int logico=0; logico< _njoints; logico++)
    {
        int fisico = _axisMap[logico];
        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, fisico, eoprot_tag_mc_joint_config);

        eOmc_joint_config_t jconfig = {0};
        memset(&jconfig, 0, sizeof(eOmc_joint_config_t));
        copyPid_iCub2eo(&_pids[logico],  &jconfig.pidposition);
        copyPid_iCub2eo(&_vpids[logico], &jconfig.pidvelocity);
        copyPid_iCub2eo(&_tpids[logico], &jconfig.pidtorque);

        //stiffness and damping read in xml file are in Nm/deg and Nm/(Deg/sec), so we need to convert before send to fw.
        jconfig.impedance.damping   = (eOmeas_damping_t) _torqueControlHelper->convertImpN2S(logico, _impedance_params[logico].damping);
        jconfig.impedance.stiffness = (eOmeas_stiffness_t) _torqueControlHelper->convertImpN2S(logico,  _impedance_params[logico].stiffness);
        jconfig.impedance.offset    = 0; //impedance_params[j];

        _cacheImpedance[logico].stiffness = jconfig.impedance.stiffness;
        _cacheImpedance[logico].damping   = jconfig.impedance.damping;
        _cacheImpedance[logico].offset    = jconfig.impedance.offset;

        jconfig.userlimits.max = (eOmeas_position_t) S_32(convertA2I(_limitsMax[logico], 0.0, _angleToEncoder[logico]));
        jconfig.userlimits.min = (eOmeas_position_t) S_32(convertA2I(_limitsMin[logico], 0.0, _angleToEncoder[logico]));

        jconfig.hardwarelimits.max = (eOmeas_position_t) S_32(convertA2I(_hwLimitsMax[logico], 0.0, _angleToEncoder[logico]));
        jconfig.hardwarelimits.min = (eOmeas_position_t) S_32(convertA2I(_hwLimitsMin[logico], 0.0, _angleToEncoder[logico]));


        jconfig.maxvelocityofjoint = S_32(_maxJntCmdVelocity[logico] * _angleToEncoder[logico]); //icubdeg/s
        jconfig.velocitysetpointtimeout = (eOmeas_time_t) U_16(_velocityTimeout[logico]);

        jconfig.jntEncoderResolution = _jointEncoderRes[logico];
        jconfig.jntEncoderType = _jointEncoderType[logico];
        jconfig.motor_params.bemf_value = 0;
        jconfig.motor_params.bemf_scale = 0;
        jconfig.motor_params.ktau_value = 0;
        jconfig.motor_params.ktau_scale = 0;

        jconfig.tcfiltertype=_filterType[logico];


        if(false == res->setRemoteValueUntilVerified(protid, &jconfig, sizeof(jconfig), 10, 0.010, 0.050, 2))
        {
            yError() << "FATAL: embObjMotionControl::init() had an error while calling setRemoteValueUntilVerified() for joint config fisico #" << fisico << "in BOARD" << res->getName() << "with IP" << res->getIPv4string();
            return false;
        }
        else
        {
            if(verbosewhenok)
            {
                yDebug() << "embObjMotionControl::init() correctly configured joint config fisico #" << fisico << "in BOARD" << res->getName() << "with IP" << res->getIPv4string();
            }
        }
    }

    /////////////////////////////////////////////////////////
    // invia la configurazione dei parametri di stiction   //
    /////////////////////////////////////////////////////////
    for(int logico=0; logico< _njoints; logico++)
    {
        MotorTorqueParameters params;
        params.bemf = _kbemf[logico];
        params.bemf_scale = 0;
        params.ktau = _ktau[logico];
        params.ktau_scale = 0;
        //use the yarp method to get the values properly converted from [SI] to HW units (if necessary)
        setMotorTorqueParams(logico,params);
    }

    //////////////////////////////////////////
    // invia la configurazione dei MOTORI   //
    //////////////////////////////////////////

//    yDebug() << "Sending motor MAX CURRENT ONLY";

    for(int logico=0; logico<_njoints; logico++)
    {
        int fisico = _axisMap[logico];

        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, fisico, eoprot_tag_mc_motor_config);
        eOmc_motor_config_t motor_cfg = {0};
        motor_cfg.maxvelocityofmotor = 0;//_maxMotorVelocity[logico]; //unused yet!
        motor_cfg.currentLimits.nominalCurrent = _currentLimits[logico].nominalCurrent;
        motor_cfg.currentLimits.overloadCurrent = _currentLimits[logico].overloadCurrent;
        motor_cfg.currentLimits.peakCurrent = _currentLimits[logico].peakCurrent;
        motor_cfg.gearboxratio = _gearbox[logico];
        motor_cfg.gearboxratio2 = _gearboxE2J[logico];
        motor_cfg.rotorEncoderResolution = _rotorEncoderRes[logico];
        motor_cfg.hasHallSensor = _hasHallSensor[logico];
        motor_cfg.hasRotorEncoder = _hasRotorEncoder[logico];
        motor_cfg.hasTempSensor = _hasTempSensor[logico];
        motor_cfg.hasRotorEncoderIndex = _hasRotorEncoderIndex[logico];
        motor_cfg.hasSpeedEncoder = _hasSpeedEncoder[logico];
        motor_cfg.motorPoles = _motorPoles[logico];
        motor_cfg.rotorIndexOffset = _rotorIndexOffset[logico];
        motor_cfg.rotorEncoderType = _rotorEncoderType[logico];
        motor_cfg.pwmLimit =_motorPwmLimits[logico];
        motor_cfg.limitsofrotor.max = (eOmeas_position_t) S_32(convertA2I(_rotorlimits_max[logico], 0.0, _angleToEncoder[logico]));
        motor_cfg.limitsofrotor.min = (eOmeas_position_t) S_32(convertA2I(_rotorlimits_min[logico], 0.0, _angleToEncoder[logico]));
        
        if(_currentPidsAvailables)
        {
            copyPid_iCub2eo(&_cpids[logico],  &motor_cfg.pidcurrent);
        }
        else
        {
            motor_cfg.pidcurrent.kp = 8;
            motor_cfg.pidcurrent.ki = 2;
            motor_cfg.pidcurrent.scale = 10;
        }
        
        if (false == res->setRemoteValueUntilVerified(protid, &motor_cfg, sizeof(motor_cfg), 10, 0.010, 0.050, 2))
        {
            yError() << "FATAL: embObjMotionControl::init() had an error while calling setRemoteValueUntilVerified() for motor config fisico #" << fisico << "in BOARD" << res->getName() << "with IP" << res->getIPv4string();
            return false;
        }
        else
        {
            if (verbosewhenok)
            {
                yDebug() << "embObjMotionControl::init() correctly configured motor config fisico #" << fisico << "in BOARD" << res->getName() << "with IP" << res->getIPv4string();
            }
        }


    }

    /////////////////////////////////////////////
    // invia la configurazione del controller  //
    /////////////////////////////////////////////

    //to be done

    yTrace() << "embObjMotionControl::init(): correctly instantiated for BOARD" << res->getName() << "with IP" << res->getIPv4string();
    return true;
}



bool embObjMotionControl::close()
{
    yTrace() << " embObjMotionControl::close()";

    ImplementControlMode2::uninitialize();
    ImplementEncodersTimed::uninitialize();
    ImplementMotorEncoders::uninitialize();
    ImplementPositionControl2::uninitialize();
    ImplementVelocityControl<embObjMotionControl, IVelocityControl>::uninitialize();
    ImplementVelocityControl2::uninitialize();
    ImplementPidControl<embObjMotionControl, IPidControl>::uninitialize();
    ImplementControlCalibration2<embObjMotionControl, IControlCalibration2>::uninitialize();
    ImplementAmplifierControl<embObjMotionControl, IAmplifierControl>::uninitialize();
    ImplementImpedanceControl::uninitialize();
    ImplementControlLimits2::uninitialize();
    ImplementTorqueControl::uninitialize();
    ImplementPositionDirect::uninitialize();
    ImplementOpenLoopControl::uninitialize();
    ImplementInteractionMode::uninitialize();
    ImplementRemoteVariables::uninitialize();
    ImplementAxisInfo::uninitialize();

    if (_torqueControlHelper)  {delete _torqueControlHelper; _torqueControlHelper=0;}


    // in cleanup, at date of 23feb2016 there is a call to ethManager->releaseResource() which ...
    // send to config all the boards and stops tx and rx treads.
    // thus, in here we cannot call serviceStop(mc) because there will be tx/rx activity only for the first call of ::close().
    // i termporarily put serviceStop(eomn_serv_category_all) inside releaseResource()
    // todo: later on: clear regulars of mc, stop(mc), inside releaseresource() DO NOT stop tx/rx activity and DO NOT stop all services
    // res->serviceStop(eomn_serv_category_mc);
    // #warning TODO: clear the regulars imposed by motion-control.

    cleanup();

    return true;
}

void embObjMotionControl::cleanup(void)
{
    if(ethManager == NULL) return;

    int ret = ethManager->releaseResource2(res, this);
    res = NULL;
    if(ret == -1)
        ethManager->killYourself();
}



eoThreadEntry * embObjMotionControl::appendWaitRequest(int j, uint32_t protoid)
{
    eoRequest req;
    if(!requestQueue->threadPool->getId(&req.threadId) )
        yError() << "Error: too much threads!! (embObjMotionControl)";
    req.joint = j;
    req.prognum = res->translate_NVid2index(protoid);
    requestQueue->append(req);
    return requestQueue->threadPool->getThreadTable(req.threadId);
}

iethresType_t embObjMotionControl::type()
{
    return iethres_motioncontrol;
}

bool embObjMotionControl::update(eOprotID32_t id32, double timestamp, void *rxdata)
{
    // use this function to update the values cached in the class using data received by the remote boards via the network callbacks
    // in embObjMotionControl it is updated only the timestamp of the encoders, thuus i dont used rxdata
    int joint = eoprot_ID2index(id32);

    rxdata = rxdata;

    // marco.accame: pay attention using rxdata. the rxdata depends on the id32.
    // now the function update() is called with rxdata of different types.
    // if the tag is eoprot_tag_mc_joint_status, then rxdata is of type eOmc_joint_status_t*
    // if the tag is eoprot_tag_mc_joint_status_basic, then rxdata is of type eOmc_joint_status_basic_t*


    // for the case of id32 which contains an encoder value .... we refresh the timestamp of that encoder

    if(true == initialised())
    {   // do it only if we already have opened the device
        _mutex.wait();
        _encodersStamp[joint] = timestamp;
        _mutex.post();
    }

    return true;
}

eoThreadFifo * embObjMotionControl::getFifo(uint32_t variableProgNum)
{
    return requestQueue->getFifo(variableProgNum);
}

eoThreadEntry *embObjMotionControl::getThreadTable(int threadId)
{
    return requestQueue->threadPool->getThreadTable(threadId);
}


///////////// PID INTERFACE

bool embObjMotionControl::setPidRaw(int j, const Pid &pid)
{
    eOprotID32_t protoId = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_pidposition);
    eOmc_PID_t  outPid;
    Pid hwPid = pid;

    if (_positionControlUnits==P_METRIC_UNITS)
    {
        hwPid.kp = hwPid.kp / _angleToEncoder[j];  //[PWM/deg]
        hwPid.ki = hwPid.ki / _angleToEncoder[j];  //[PWM/deg]
        hwPid.kd = hwPid.kd / _angleToEncoder[j];  //[PWM/deg]
    }
    else if (_positionControlUnits==P_MACHINE_UNITS)
    {
        hwPid.kp = hwPid.kp;  //[PWM/icubdegrees]
        hwPid.ki = hwPid.ki;  //[PWM/icubdegrees]
        hwPid.kd = hwPid.kd;  //[PWM/icubdegrees]
    }
    else
    {
        yError() << "Unknown _positionControlUnits";
    }

    copyPid_iCub2eo(&hwPid, &outPid);

    if(!res->addSetMessage(protoId, (uint8_t *) &outPid))
    {
        yError() << "while setting position PIDs for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j;
        return false;
    }

    return true;
}

bool embObjMotionControl::setPidsRaw(const Pid *pids)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= setPidRaw(j, pids[j]);
    }
    return ret;
}

bool embObjMotionControl::setReferenceRaw(int j, double ref)
{
    int mode = 0;
    getControlModeRaw(j, &mode);
    if (mode != VOCAB_CM_POSITION_DIRECT &&
        mode != VOCAB_CM_IDLE)
    {
        #ifdef ICUB_AUTOMATIC_MODE_SWITCHING
        yWarning() << "setReferenceRaw: Deprecated automatic switch to VOCAB_CM_POSITION_DIRECT, BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint "<< j;
        setControlModeRaw(j,VOCAB_CM_POSITION_DIRECT);
        #else
        yError() << "setReferenceRaw: skipping command because BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " is not in VOCAB_CM_POSITION_DIRECT mode";
        #endif
    }

    eOprotID32_t protoId = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);
    eOmc_setpoint_t setpoint = {0};

    _ref_positions[j] = ref;   // save internally the new value of pos.
    setpoint.type = (eOenum08_t) eomc_setpoint_positionraw;
    setpoint.to.position.value = (eOmeas_position_t) S_32(ref);
    setpoint.to.position.withvelocity = 0;

    return res->addSetMessage(protoId, (uint8_t*) &setpoint);
}

bool embObjMotionControl::setReferencesRaw(const double *refs)
{
    bool ret = true;
    for(int j=0, index=0; j< _njoints; j++, index++)
    {
        ret &= setReferenceRaw(j, refs[index]);
    }
    return ret;
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
    uint16_t size = 0;
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core);
    eOmc_joint_status_core_t  jcore = {0};
    *err = 0;
    if(!res->readBufferedValue(id32, (uint8_t *)&jcore, &size))
        return false;


#if NEW_JSTATUS_STRUCT
    if((eomc_controlmode_torque == jcore.modes.controlmodestatus)   ||
       (eomc_controlmode_openloop == jcore.modes.controlmodestatus) ||
       (eomc_controlmode_current == jcore.modes.controlmodestatus))
            return true;

    *err = (double) jcore.ofpid.generic.error1  ;
    return true;


#else

    int mycontrolMode;
    /* Values in pid.XXX fields are valid ONLY IF we are in the corresponding control mode.
    Read it from the signalled message so we are sure that mode and pid values are coherent to each other */
    getControlModeRaw(j, &mycontrolMode);
    if(VOCAB_CM_POSITION != mycontrolMode )
    {
        //yWarning() << "Asked for Position PID Error while not in Position control mode. Returning zeros";
        err = 0;//bug??
        return false;
    }
    *err = (double) jstatus.ofpid.legacy.error;
#endif

    return true;
}

bool embObjMotionControl::getErrorsRaw(double *errs)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getErrorRaw(j, &errs[j]);
    }
    return ret;
}

bool embObjMotionControl::getPidRaw(int j, Pid *pid)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_pidposition);
    // Sign up for waiting the reply

    eoThreadEntry *tt = appendWaitRequest(j, protid);
    tt->setPending(1);

    if(!res->addGetMessage(protid) )
    {
        yError() << "Can't send get pid request for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j;
        return false;
    }

    // wait here
    if(-1 == tt->synch() )
    {
        int threadId;
        yError () << "embObjMotionControl::getPidRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j;

        if(requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_PID_t eoPID;
    res->readBufferedValue(protid, (uint8_t *)&eoPID, &size);

    copyPid_eo2iCub(&eoPID, pid);

    if (_positionControlUnits==P_METRIC_UNITS)
    {
        pid->kp = pid->kp * _angleToEncoder[j];  //[PWM/deg]
        pid->ki = pid->ki * _angleToEncoder[j];  //[PWM/deg]
        pid->kd = pid->kd * _angleToEncoder[j];  //[PWM/deg]
    }
    else if (_positionControlUnits==P_MACHINE_UNITS)
    {
        pid->kp = pid->kp;  //[PWM/icubdegrees]
        pid->ki = pid->ki;  //[PWM/icubdegrees]
        pid->kd = pid->kd;  //[PWM/icubdegrees]
    }
    else
    {
        yError() << "Unknown _positionControlUnits";
    }

    return true;
}

bool embObjMotionControl::getPidsRaw(Pid *pids)
{
    bool ret = true;

    // just one joint at time, wait answer before getting to the next.
    // This is because otherwise too many msg will be placed into can queue
    for(int j=0, index=0; j<_njoints; j++, index++)
    {
        ret &=getPidRaw(j, &pids[j]);
    }
    return ret;
}

bool embObjMotionControl::getReferenceRaw(int j, double *ref)
{
    uint16_t size = 0;
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core);
    eOmc_joint_status_core_t jcore = {0};
    *ref = 0;
    if(!res->readBufferedValue(id32, (uint8_t *)&jcore, &size))
        return false;


#if NEW_JSTATUS_STRUCT

    if((eomc_controlmode_torque == jcore.modes.controlmodestatus)   ||
       (eomc_controlmode_openloop == jcore.modes.controlmodestatus) ||
       (eomc_controlmode_current == jcore.modes.controlmodestatus))
            return true;

    *ref = (double) jcore.ofpid.generic.reference1;

    return true;

#else
    *ref = jstatus.ofpid.legacy.positionreference;
    return true;
#endif

}

bool embObjMotionControl::getReferencesRaw(double *refs)
{
    bool ret = true;

    // just one joint at time, wait answer before getting to the next.
    // This is because otherwise too many msg will be placed into can queue
    for(int j=0; j< _njoints; j++)
    {
        ret &= getReferenceRaw(j, &refs[j]);
    }
    return ret;
}

bool embObjMotionControl::getErrorLimitRaw(int j, double *limit)
{
    return NOT_YET_IMPLEMENTED("getErrorLimit");
}

bool embObjMotionControl::getErrorLimitsRaw(double *limits)
{
    return NOT_YET_IMPLEMENTED("getErrorLimit");
}

bool embObjMotionControl::resetPidRaw(int j)
{
    return NOT_YET_IMPLEMENTED("resetPid");
}

bool embObjMotionControl::disablePidRaw(int j)
{
    return DEPRECATED("disablePidRaw");
}

bool embObjMotionControl::enablePidRaw(int j)
{
    return DEPRECATED("enablePidRaw");
}

bool embObjMotionControl::setOffsetRaw(int j, double v)
{
    return NOT_YET_IMPLEMENTED("setOffset");
}

////////////////////////////////////////
//    Velocity control interface raw  //
////////////////////////////////////////

bool embObjMotionControl::setVelocityModeRaw()
{
    bool ret = true;

    eOprotID32_t protid;

    eOmc_controlmode_command_t val = eomc_controlmode_cmd_velocity;
    for(int j=0, index=0; j< _njoints; j++, index++)
    {
        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_controlmode);

        if(! res->addSetMessage(protid, (uint8_t *) &val))
        {
            yError() << "while setting velocity mode";
            return false;
        }
    }
    return ret;
}

bool embObjMotionControl::velocityMoveRaw(int j, double sp)
{
    int mode=0;
    getControlModeRaw(j, &mode);
    if( (mode != VOCAB_CM_VELOCITY) &&
        (mode != VOCAB_CM_MIXED) &&
        (mode != VOCAB_CM_IMPEDANCE_VEL) &&
        (mode != VOCAB_CM_IDLE))
    {
        #ifdef ICUB_AUTOMATIC_MODE_SWITCHING
        yWarning() << "velocityMoveRaw: Deprecated automatic switch to VOCAB_CM_VELOCITY, BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j;
        setControlModeRaw(j, VOCAB_CM_VELOCITY);
        #else
        yError() << "velocityMoveRaw: skipping command because BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " is not in VOCAB_CM_VELOCITY mode";
        #endif
    }

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);

    _ref_command_speeds[j] = sp ;   // save internally the new value of speed.

    eOmc_setpoint_t setpoint;
    setpoint.type = eomc_setpoint_velocity;
    setpoint.to.velocity.value =  (eOmeas_velocity_t) S_32(_ref_command_speeds[j]);
    setpoint.to.velocity.withacceleration = (eOmeas_acceleration_t) S_32(_ref_accs[j]);


    if(! res->addSetMessage(protid, (uint8_t *) &setpoint))
    {
        yError() << "while setting velocity mode";
        return false;
    }
    return true;
}

bool embObjMotionControl::velocityMoveRaw(const double *sp)
{
    bool ret = true;
    eOmc_setpoint_t setpoint;

    setpoint.type = eomc_setpoint_velocity;

    for(int j=0; j<_njoints; j++)
    {
        ret = velocityMoveRaw(j, sp[j]) && ret;
    }

    return ret;
}


////////////////////////////////////////
//    Calibration control interface   //
////////////////////////////////////////

bool embObjMotionControl::setCalibrationParametersRaw(int j, const CalibrationParameters& params)
{
    yTrace() << "setCalibrationParametersRaw for BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint" << j;

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_calibration);
    eOmc_calibrator_t calib;
    memset(&calib, 0x00, sizeof(calib));
    calib.type = params.type;

    switch (calib.type)
    {
        // muove -> amp+pid, poi calib
    case eomc_calibration_type0_hard_stops:
        calib.params.type0.pwmlimit = (int16_t)S_16(params.param1);
        calib.params.type0.velocity = (eOmeas_velocity_t)S_32(params.param2);
        calib.params.type0.calibrationZero = (int32_t)S_32(params.paramZero * _angleToEncoder[j]);
        break;

        // fermo
    case eomc_calibration_type1_abs_sens_analog:
        calib.params.type1.position = (int16_t)S_16(params.param1);
        calib.params.type1.velocity = (eOmeas_velocity_t)S_32(params.param2);
        calib.params.type1.calibrationZero = (int32_t)S_32(params.paramZero * _angleToEncoder[j]);
        break;

        // muove
    case eomc_calibration_type2_hard_stops_diff:
        calib.params.type2.pwmlimit = (int16_t)S_16(params.param1);
        calib.params.type2.velocity = (eOmeas_velocity_t)S_32(params.param2);
        calib.params.type2.calibrationZero = (int32_t)S_32(params.paramZero * _angleToEncoder[j]);
        break;

        // muove
    case eomc_calibration_type3_abs_sens_digital:
        calib.params.type3.position = (int16_t)S_16(params.param1);
        calib.params.type3.velocity = (eOmeas_velocity_t)S_32(params.param2);
        calib.params.type3.offset = (int32_t)S_32(params.param3);
        calib.params.type3.calibrationZero = (int32_t)S_32(params.paramZero * _angleToEncoder[j]);
        break;

        // muove
    case eomc_calibration_type4_abs_and_incremental:
        calib.params.type4.position = (int16_t)S_16(params.param1);
        calib.params.type4.velocity = (eOmeas_velocity_t)S_32(params.param2);
        calib.params.type4.maxencoder = (int32_t)S_32(params.param3);
        calib.params.type4.calibrationZero = (int32_t)S_32(params.paramZero * _angleToEncoder[j]);
        break;

        // muove
    case eomc_calibration_type5_hard_stops:
        calib.params.type5.pwmlimit   = (int32_t) S_32(params.param1);
        calib.params.type5.calibrationZero = (int32_t)S_32(params.paramZero * _angleToEncoder[j]);
        break;

        // muove
    case eomc_calibration_type6_mais:
        calib.params.type6.position = (int32_t)S_32(params.param1);
        calib.params.type6.velocity = (int32_t)S_32(params.param2);
        calib.params.type6.current = (int32_t)S_32(params.param3);
        calib.params.type6.vmin = (int32_t)S_32(params.param4);
        calib.params.type6.vmax = (int32_t)S_32(params.param5);
        calib.params.type6.calibrationZero = (int32_t)S_32(params.paramZero * _angleToEncoder[j]);
        break;

        // muove
    case eomc_calibration_type7_hall_sensor:
        calib.params.type7.position = (int32_t)S_32(params.param1);
        calib.params.type7.velocity = (int32_t)S_32(params.param2);
        //param3 is not used
        calib.params.type7.vmin = (int32_t)S_32(params.param4);
        calib.params.type7.vmax = (int32_t)S_32(params.param5);
        calib.params.type7.calibrationZero = (int32_t)S_32(params.paramZero * _angleToEncoder[j]);
        break;

        //muove
    case eomc_calibration_type8_tripod_internal_hard_stop:
        calib.params.type8.pwmlimit   = (int32_t) S_32(params.param1);
        calib.params.type8.max_delta  = (int32_t) S_32(params.param2);
        calib.params.type8.calibrationZero = (int32_t)S_32(params.paramZero /* * _angleToEncoder[j] */);
        break;

    case eomc_calibration_type9_tripod_external_hard_stop:
        calib.params.type9.pwmlimit   = (int32_t) S_32(params.param1);
        calib.params.type9.max_delta  = (int32_t) S_32(params.param2);
        calib.params.type9.calibrationZero = (int32_t)S_32(params.paramZero /* * _angleToEncoder[j] */);
        break;

    case eomc_calibration_type10_abs_hard_stop:
        calib.params.type10.pwmlimit   = (int32_t) S_32(params.param1);
        calib.params.type10.calibrationZero = (int32_t)S_32(params.paramZero * _angleToEncoder[j]);
        break;

    case eomc_calibration_type11_cer_hands:
        calib.params.type11.offset0     = (int32_t)S_32(params.param1);
        calib.params.type11.offset1     = (int32_t)S_32(params.param2);
        calib.params.type11.offset2     = (int32_t)S_32(params.param3);
        calib.params.type11.cable_range = (int32_t)S_32(params.param4);
        calib.params.type11.pwm         = (int32_t)S_32(params.param5);
        //calib.params.type11.calibrationZero = 32767;//(int32_t)S_32(params.paramZero * _angleToEncoder[j]);
        break;

    default:
        yError() << "Calibration type unknown!! (embObjMotionControl)\n";
        return false;
        break;
    }

    if (!res->addSetMessage(protid, (uint8_t *)&calib))
    {
        yError() << "while setting velocity mode";
        return false;
    }

    _calibrated[j] = true;

    return true;
}

bool embObjMotionControl::calibrate2Raw(int j, unsigned int type, double p1, double p2, double p3)
{
    yTrace() << "calibrate2Raw for BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint" << j;

    // Tenere il check o forzare questi sottostati?
//    if(!_enabledAmp[j ] )
//    {
//        yWarning () << "Called calibrate for joint " << j << "with PWM(AMP) not enabled, forcing it!!";
//        //        return false;
//    }

//    if(!_enabledPid[j ])
//    {
//        yWarning () << "Called calibrate for joint " << j << "with PID not enabled, forcing it!!";
//        //        return false;
//    }

    //   There is no explicit command "go to calibration mode" but it is implicit in the calibration command


    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_calibration);
    eOmc_calibrator_t calib;
    memset(&calib, 0x00, sizeof(calib));
    calib.type = type;

    switch(type)
    {
        // muove -> amp+pid, poi calib
    case eomc_calibration_type0_hard_stops:
        calib.params.type0.pwmlimit = (int16_t) S_16(p1);
        calib.params.type0.velocity = (eOmeas_velocity_t) S_32(p2);
        break;

        // fermo
    case eomc_calibration_type1_abs_sens_analog:
        calib.params.type1.position = (int16_t) S_16(p1);
        calib.params.type1.velocity = (eOmeas_velocity_t)  S_32(p2);
        break;

        // muove
    case eomc_calibration_type2_hard_stops_diff:
        calib.params.type2.pwmlimit = (int16_t) S_16(p1);
        calib.params.type2.velocity = (eOmeas_velocity_t)  S_32(p2);
        break;

        // muove
    case eomc_calibration_type3_abs_sens_digital:
        calib.params.type3.position = (int16_t) S_16(p1);
        calib.params.type3.velocity = (eOmeas_velocity_t)  S_32(p2);
        calib.params.type3.offset   = (int32_t) S_32(p3);
        break;

        // muove
    case eomc_calibration_type4_abs_and_incremental:
        calib.params.type4.position   = (int16_t) S_16(p1);
        calib.params.type4.velocity   = (eOmeas_velocity_t)  S_32(p2);
        calib.params.type4.maxencoder = (int32_t) S_32(p3);
        break;

    default:
        yError () << "Calibration type unknown!! (embObjMotionControl)\n";
        return false;
        break;
    }

    if(! res->addSetMessage(protid, (uint8_t *) &calib))
    {
        yError() << "while setting velocity mode";
        return false;
    }

    _calibrated[j ] = true;

    return true;
}


bool embObjMotionControl::doneRaw(int axis)
{
    bool result = false;
    eOenum08_t temp = 0;
    uint16_t size = 0;
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, axis, eoprot_tag_mc_joint_status_core_modes_controlmodestatus);
    if(false == askRemoteValue(id32, &temp, size))
    {
        yError ("Failure of askRemoteValue() inside embObjMotionControl::doneRaw(axis=%d) for BOARD %s IP %s", axis, res->getName(), res->getIPv4string());
        return false;
    }

    eOmc_controlmode_t type = (eOmc_controlmode_t) temp;


    // if the control mode is no longer a calibration type, it means calibration ended
    if (eomc_controlmode_idle == type)
    {
        result = false;
    }
    else if (eomc_controlmode_calib == type)
    {
        result = false;
    }
    else if (eomc_controlmode_hwFault == type)
    {
        yError("unable to complete calibration: joint %d in 'hw_fault status' inside doneRaw() function", axis);
        result = false;
    }
    else if (eomc_controlmode_notConfigured == type)
    {
        yError("unable to complete calibration: joint %d in 'not_configured' status inside doneRaw() function", axis);
        result = false;
    }
    else if (eomc_controlmode_unknownError == type)
    {
        yError("unable to complete calibration: joint %d in 'unknownError' status inside doneRaw() function", axis);
        result = false;
    }
    else if (eomc_controlmode_configured == type)
    {
        yError("unable to complete calibration: joint %d in 'configured' status inside doneRaw() function", axis);
        result = false;
    }
    else
    {
        result = true;
    }
    return result;
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
    return DEPRECATED("setPositionModeRaw");
}

bool embObjMotionControl::positionMoveRaw(int j, double ref)
{
    if (yarp::os::Time::now()-_last_position_move_time[j]<MAX_POSITION_MOVE_INTERVAL)
    {
        yWarning() << "Performance warning: You are using positionMove commands at high rate (<"<< MAX_POSITION_MOVE_INTERVAL*1000.0 <<" ms). Probably position control mode is not the right control mode to use.";
    }
    _last_position_move_time[j] = yarp::os::Time::now();

    int mode = 0;
    getControlModeRaw(j, &mode);
    if( (mode != VOCAB_CM_POSITION) &&
        (mode != VOCAB_CM_MIXED) &&
        (mode != VOCAB_CM_IMPEDANCE_POS) &&
        (mode != VOCAB_CM_IDLE))
    {
        #ifdef ICUB_AUTOMATIC_MODE_SWITCHING
        yDebug() << "positionMoveRaw: Deprecated automatic switch to VOCAB_CM_POSITION, BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j;
        setControlModeRaw(j, VOCAB_CM_POSITION);
        #else
        yError() << "positionMoveRaw: skipping command because BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " is not in VOCAB_CM_POSITION mode";
        #endif
    }

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);
    _ref_command_positions[j] = ref;   // save internally the new value of pos.

    eOmc_setpoint_t setpoint;

    setpoint.type = (eOenum08_t) eomc_setpoint_position;
    setpoint.to.position.value =  (eOmeas_position_t) S_32(_ref_command_positions[j]);
    setpoint.to.position.withvelocity = (eOmeas_velocity_t) S_32(_ref_speeds[j]);

    return res->addSetMessage(protid, (uint8_t*) &setpoint);
}

bool embObjMotionControl::positionMoveRaw(const double *refs)
{
    bool ret = true;

    for(int j=0, index=0; j< _njoints; j++, index++)
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
    eObool_t ismotiondone = eobool_false;
    uint16_t size = 0;

    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core_modes_ismotiondone);
    if(false == askRemoteValue(id32, &ismotiondone, size))
    {
        yError ("Failure of askRemoteValue() inside embObjMotionControl::checkMotionDoneRaw(j=%d) for BOARD %s IP %s", j, res->getName(), res->getIPv4string());
        return false;
    }


    *flag = ismotiondone; // eObool_t can have values only amongst: eobool_true (1) or eobool_false (0).

    return true;
}

bool embObjMotionControl::checkMotionDoneRaw(bool *flag)
{
    bool ret = true;
    bool val, tot_res = true;

    for(int j=0, index=0; j< _njoints; j++, index++)
    {
        ret &= checkMotionDoneRaw(&val);
        tot_res &= val;
    }
    *flag = tot_res;
    return ret;
}

bool embObjMotionControl::setRefSpeedRaw(int j, double sp)
{
    // Velocity is expressed in iDegrees/s
    // save internally the new value of speed; it'll be used in the positionMove
    int index = j ;
    _ref_speeds[index] = sp;
    return true;
}

bool embObjMotionControl::setRefSpeedsRaw(const double *spds)
{
    // Velocity is expressed in iDegrees/s
    // save internally the new value of speed; it'll be used in the positionMove
    for(int j=0, index=0; j< _njoints; j++, index++)
    {
        _ref_speeds[index] = spds[index];
    }
    return true;
}

bool embObjMotionControl::setRefAccelerationRaw(int j, double acc)
{
    // Acceleration is expressed in iDegrees/s^2
    // save internally the new value of the acceleration; it'll be used in the velocityMove command

    if (acc > 1e6)
    {
        _ref_accs[j ] =  1e6;
    }
    else if (acc < -1e6)
    {
        _ref_accs[j ] = -1e6;
    }
    else
    {
        _ref_accs[j ] = acc;
    }

    return true;
}

bool embObjMotionControl::setRefAccelerationsRaw(const double *accs)
{
    // Acceleration is expressed in iDegrees/s^2
    // save internally the new value of the acceleration; it'll be used in the velocityMove command
    for(int j=0, index=0; j< _njoints; j++, index++)
    {
        if (accs[j] > 1e6)
        {
            _ref_accs[index] =  1e6;
        }
        else if (accs[j] < -1e6)
        {
            _ref_accs[index] = -1e6;
        }
        else
        {
            _ref_accs[index] = accs[j];
        }
    }
    return true;
}

bool embObjMotionControl::getRefSpeedRaw(int j, double *spd)
{
    if (j<0 || j>_njoints) return false;
#if ASK_REFERENCE_TO_FIRMWARE
    *spd = _ref_speeds[j];
    //return NOT_YET_IMPLEMENTED("getRefSpeedRaw");
#else
    *spd = _ref_speeds[j];
#endif
    return true;
}

bool embObjMotionControl::getRefSpeedsRaw(double *spds)
{
    memcpy(spds, _ref_speeds, sizeof(double) * _njoints);
    return true;
}

bool embObjMotionControl::getRefAccelerationRaw(int j, double *acc)
{
    *acc = _ref_accs[j];
    return true;
}

bool embObjMotionControl::getRefAccelerationsRaw(double *accs)
{
    memcpy(accs, _ref_accs, sizeof(double) * _njoints);
    return true;
}

bool embObjMotionControl::stopRaw(int j)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_stoptrajectory);

    eObool_t stop = eobool_true;

    return res->addSetMessage(protid, (uint8_t*) &stop);
}

bool embObjMotionControl::stopRaw()
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= stopRaw(j);
    }
    return ret;
}
///////////// END Position Control INTERFACE  //////////////////

////////////////////////////////////////
//     Position control2 interface    //
////////////////////////////////////////

bool embObjMotionControl::positionMoveRaw(const int n_joint, const int *joints, const double *refs)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret &&positionMoveRaw(joints[j], refs[j]);
    }
    return ret;
}

bool embObjMotionControl::relativeMoveRaw(const int n_joint, const int *joints, const double *deltas)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret &&relativeMoveRaw(joints[j], deltas[j]);
    }
    return ret;
}

bool embObjMotionControl::checkMotionDoneRaw(const int n_joint, const int *joints, bool *flag)
{
    bool ret = true;
    bool val = true;
    bool tot_val = true;

    for(int j=0; j<n_joint; j++)
    {
        ret = ret && checkMotionDoneRaw(joints[j], &val);
        tot_val &= val;
    }
    *flag = tot_val;
    return ret;
}

bool embObjMotionControl::setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret &&setRefSpeedRaw(joints[j], spds[j]);
    }
    return ret;
}

bool embObjMotionControl::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret &&setRefAccelerationRaw(joints[j], accs[j]);
    }
    return ret;
}

bool embObjMotionControl::getRefSpeedsRaw(const int n_joint, const int *joints, double *spds)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret && getRefSpeedRaw(joints[j], &spds[j]);
    }
    return ret;
}

bool embObjMotionControl::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret && getRefAccelerationRaw(joints[j], &accs[j]);
    }
    return ret;
}

bool embObjMotionControl::stopRaw(const int n_joint, const int *joints)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret &&stopRaw(joints[j]);
    }
    return ret;
}

///////////// END Position Control INTERFACE  //////////////////

// ControlMode
bool embObjMotionControl::setPositionModeRaw(int j)
{
    return DEPRECATED("setPositionModeRaw");
}

bool embObjMotionControl::setVelocityModeRaw(int j)
{
    return DEPRECATED("setVelocityModeRaw");
}

bool embObjMotionControl::setTorqueModeRaw(int j)
{
    return DEPRECATED("setTorqueModeRaw");
}

bool embObjMotionControl::setImpedancePositionModeRaw(int j)
{
    return DEPRECATED("setImpedancePositionModeRaw");
}

bool embObjMotionControl::setImpedanceVelocityModeRaw(int j)
{
    return DEPRECATED("setImpedanceVelocityModeRaw");
}

bool embObjMotionControl::setOpenLoopModeRaw(int j)
{
    return DEPRECATED("setOpenLoopModeRaw");
}
// puo' essere richiesto con get
bool embObjMotionControl::getControlModeRaw(int j, int *v)
{
    uint16_t size = 0;
    eOmc_joint_status_core_t jcore = {0};

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core);
    if(! res->readBufferedValue(protid, (uint8_t *)&jcore, &size))
        return false;

    eOmc_controlmode_t type = (eOmc_controlmode_t) jcore.modes.controlmodestatus;

    *v = controlModeStatusConvert_embObj2yarp(type);
    return true;
}

// IControl Mode 2
bool embObjMotionControl::getControlModesRaw(int* v)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret = ret && getControlModeRaw(j, &v[j]);
    }
    return ret;
}

bool embObjMotionControl::getControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    bool ret = true;
    for(int j=0; j< n_joint; j++)
    {
        ret = ret && getControlModeRaw(joints[j], &modes[j]);
    }
    return ret;
}



// marco.accame: con alberto cardellino abbiamo parlato della correttezza di effettuare la verifica di quanto imposto (in setControlModeRaw() ed affini)
// andando a rileggere il valore nella scheda eth fino a che esso non sia quello atteso. si deve fare oppure no?
// con il control mode il can ora lo fa ma e' giusto? era cosi' anche in passato?
bool embObjMotionControl::setControlModeRaw(const int j, const int _mode)
{
    bool ret = true;
    eOenum08_t controlmodecommand = 0;

    if((_mode == VOCAB_CM_TORQUE) && (_torqueControlEnabled == false))
    {
        yError()<<"Torque control is disabled. Check your configuration parameters";
        return false;
    }

    if(!controlModeCommandConvert_yarp2embObj(_mode, controlmodecommand) )
    {
        yError() << "SetControlMode: received unknown control mode for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " mode " << Vocab::decode(_mode);
        return false;
    }

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_controlmode);
    if(! res->addSetMessage(protid, (uint8_t*) &controlmodecommand) )
    {
        yError() << "setControlModeRaw failed for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " mode " << Vocab::decode(_mode);
        return false;
    }


    ret = checkRemoteControlModeStatus(j, _mode);

    if(false == ret)
    {
        yError("In embObjMotionControl::setControlModeRaw(j=%d, mode=%s) for BOARD %s IP %s has failed checkRemoteControlModeStatus()", j, yarp::os::Vocab::decode(_mode).c_str(), res->getName(), res->getIPv4string());
    }

    return ret;
}


bool embObjMotionControl::setControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    bool ret = true;
    eOenum08_t controlmodecommand = 0;


    for(int i=0; i<n_joint; i++)
    {
        if ((modes[i] == VOCAB_CM_TORQUE) && (_torqueControlEnabled == false)) {yError()<<"Torque control is disabled. Check your configuration parameters"; continue;}

        if(!controlModeCommandConvert_yarp2embObj(modes[i], controlmodecommand) )
        {
            yError() << "SetControlModesRaw(): received unknown control mode for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << joints[i] << " mode " << Vocab::decode(modes[i]);

            return false;
        }

        eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, joints[i], eoprot_tag_mc_joint_cmmnds_controlmode);
        if(! res->addSetMessage(protid, (uint8_t*) &controlmodecommand) )
        {
            yError() << "setControlModesRaw() could not send set<cmmnds_controlmode> for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << joints[i] << " mode " << Vocab::decode(modes[i]);

            return false;
        }

        bool tmpresult = checkRemoteControlModeStatus(i, modes[i]);
        if(false == tmpresult)
        {
             yError() << "setControlModesRaw(const int n_joint, const int *joints, int *modes) could not check with checkRemoteControlModeStatus() for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << joints[i] << " mode " << Vocab::decode(modes[i]);
        }

        ret = ret && tmpresult;

    }

    return ret;
}

bool embObjMotionControl::setControlModesRaw(int *modes)
{
    bool ret = true;
    eOenum08_t controlmodecommand = 0;

    for(int i=0; i<_njoints; i++)
    {

        if ((modes[i] == VOCAB_CM_TORQUE) && (_torqueControlEnabled == false))
        {
            yError()<<"Torque control is disabled. Check your configuration parameters";
            continue;
        }

        if(!controlModeCommandConvert_yarp2embObj(modes[i], controlmodecommand) )
        {
            yError() << "SetControlMode: received unknown control mode forBOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << i << " mode " << Vocab::decode(modes[i]);
            return false;
        }

        eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, i, eoprot_tag_mc_joint_cmmnds_controlmode);
        if(! res->addSetMessage(protid, (uint8_t*) &controlmodecommand) )
        {
            yError() << "setControlModesRaw failed for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << i << " mode " << Vocab::decode(modes[i]);
            return false;
        }

        bool tmpresult = checkRemoteControlModeStatus(i, modes[i]);
        if(false == tmpresult)
        {
             yError() << "setControlModesRaw(int *modes) could not check with checkRemoteControlModeStatus() for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << i << " mode " << Vocab::decode(modes[i]);
        }

        ret = ret && tmpresult;

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
    uint16_t      size;
    eOmc_joint_status_core_t     core;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core);

    bool ret = res->readBufferedValue(protid, (uint8_t *)&core, &size);

    if(ret)
    {
        *value = (double) core.measures.meas_position;
    }
    else
    {
        yError() << "embObjMotionControl while reading encoder";
        *value = 0;
    }

    return ret;
}

bool embObjMotionControl::getEncodersRaw(double *encs)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getEncoderRaw(j, &encs[j]);

    }
    return ret;
}

bool embObjMotionControl::getEncoderSpeedRaw(int j, double *sp)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core);
    uint16_t      size;
    eOmc_joint_status_core_t  core;
    *sp = 0;
    if(!res->readBufferedValue(protid, (uint8_t *)&core, &size))
    {
        return false;
    }
    // extract requested data from status
    *sp = (double) core.measures.meas_velocity;
    return true;
}

bool embObjMotionControl::getEncoderSpeedsRaw(double *spds)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getEncoderSpeedRaw(j, &spds[j]);
    }
    return ret;
}

bool embObjMotionControl::getEncoderAccelerationRaw(int j, double *acc)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core);
    uint16_t      size;
    eOmc_joint_status_core_t  core;
    *acc = 0;
    if(! res->readBufferedValue(protid, (uint8_t *)&core, &size))
    {
        return false;
    }
    *acc = (double) core.measures.meas_acceleration;
    return true;
}

bool embObjMotionControl::getEncoderAccelerationsRaw(double *accs)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getEncoderAccelerationRaw(j, &accs[j]);
    }
    return ret;
}

///////////////////////// END Encoder Interface

bool embObjMotionControl::getEncodersTimedRaw(double *encs, double *stamps)
{
    bool ret = getEncodersRaw(encs);
    _mutex.wait();
    for(int i=0; i<_njoints; i++)
        stamps[i] = _encodersStamp[i];
    _mutex.post();

    return ret;
}

bool embObjMotionControl::getEncoderTimedRaw(int j, double *encs, double *stamp)
{
    bool ret = getEncoderRaw(j, encs);
    _mutex.wait();
    *stamp = _encodersStamp[j];
    _mutex.post();

    return ret;
}

//////////////////////// BEGIN EncoderInterface

bool embObjMotionControl::getNumberOfMotorEncodersRaw(int* num)
{
    *num=_njoints;
    return true;
}

bool embObjMotionControl::setMotorEncoderRaw(int m, const double val)
{
    return NOT_YET_IMPLEMENTED("setMotorEncoder");
}

bool embObjMotionControl::setMotorEncodersRaw(const double *vals)
{
    return NOT_YET_IMPLEMENTED("setMotorEncoders");
}

bool embObjMotionControl::setMotorEncoderCountsPerRevolutionRaw(int m, const double cpr)
{
    return NOT_YET_IMPLEMENTED("setMotorEncoderCountsPerRevolutionRaw");
}

bool embObjMotionControl::getMotorEncoderCountsPerRevolutionRaw(int m, double *cpr)
{
    return NOT_YET_IMPLEMENTED("getMotorEncoderCountsPerRevolutionRaw");
}

bool embObjMotionControl::resetMotorEncoderRaw(int mj)
{
    return NOT_YET_IMPLEMENTED("resetMotorEncoder");
}

bool embObjMotionControl::resetMotorEncodersRaw()
{
    return NOT_YET_IMPLEMENTED("reseMotortEncoders");
}

bool embObjMotionControl::getMotorEncoderRaw(int m, double *value)
{
    uint16_t      size;
    eOmc_motor_status_basic_t     status;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, m, eoprot_tag_mc_motor_status_basic);

    bool ret = res->readBufferedValue(protid, (uint8_t *)&status, &size);
    if(ret)
    {
        *value = (double) status.mot_position;
    }
    else
    {
        yError() << "embObjMotionControl while reading motor encoder position";
        *value = 0;
    }

    return ret;
}

bool embObjMotionControl::getMotorEncodersRaw(double *encs)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getMotorEncoderRaw(j, &encs[j]);

    }
    return ret;
}

bool embObjMotionControl::getMotorEncoderSpeedRaw(int m, double *sp)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, m, eoprot_tag_mc_motor_status_basic);
    uint16_t      size;
    eOmc_motor_status_basic_t  tmpMotorStatus;
    bool ret = res->readBufferedValue(protid, (uint8_t *)&tmpMotorStatus, &size);
    if(ret)
    {
        *sp = (double) tmpMotorStatus.mot_velocity;
    }
    else
    {
        yError() << "embObjMotionControl while reading motor encoder speed";
        *sp = 0;
    }
    return true;
}

bool embObjMotionControl::getMotorEncoderSpeedsRaw(double *spds)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getMotorEncoderSpeedRaw(j, &spds[j]);
    }
    return ret;
}

bool embObjMotionControl::getMotorEncoderAccelerationRaw(int m, double *acc)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, m, eoprot_tag_mc_motor_status_basic);
    uint16_t      size;
    eOmc_motor_status_basic_t  tmpMotorStatus;
    bool ret = res->readBufferedValue(protid, (uint8_t *)&tmpMotorStatus, &size);
    if(ret)
    {
        *acc = (double) tmpMotorStatus.mot_acceleration;
    }
    else
    {
        yError() << "embObjMotionControl while reading motor encoder acceleration";
        *acc = 0;
    }
    return true;
}

bool embObjMotionControl::getMotorEncoderAccelerationsRaw(double *accs)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getMotorEncoderAccelerationRaw(j, &accs[j]);
    }
    return ret;
}

bool embObjMotionControl::getMotorEncodersTimedRaw(double *encs, double *stamps)
{
    bool ret = getMotorEncodersRaw(encs);
    _mutex.wait();
    for(int i=0; i<_njoints; i++)
        stamps[i] = _encodersStamp[i];
    _mutex.post();

    return ret;
}

bool embObjMotionControl::getMotorEncoderTimedRaw(int m, double *encs, double *stamp)
{
    bool ret = getMotorEncoderRaw(m, encs);
    _mutex.wait();
    *stamp = _encodersStamp[m];
    _mutex.post();

    return ret;
}
///////////////////////// END Motor Encoder Interface

////// Amplifier interface

bool embObjMotionControl::enableAmpRaw(int j)
{
    return DEPRECATED("enableAmpRaw");
}

bool embObjMotionControl::disableAmpRaw(int j)
{
    return DEPRECATED("disableAmpRaw");
}

bool embObjMotionControl::getCurrentRaw(int j, double *value)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_status_basic);
    uint16_t size;
    eOmc_motor_status_basic_t  tmpMotorStatus;
    bool ret = res->readBufferedValue(protid, (uint8_t *)&tmpMotorStatus, &size);

    *value = (double) tmpMotorStatus.mot_current;
    return true;
}

bool embObjMotionControl::getCurrentsRaw(double *vals)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getCurrentRaw(j, &vals[j]);
    }
    return ret;
}

bool embObjMotionControl::setMaxCurrentRaw(int j, double val)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config_currentlimits);
    eoThreadEntry *tt = appendWaitRequest(j, protid);
    tt->setPending(1);

    if(!res->addGetMessage(protid) )
    {
        yError() << "embObjMotionControl::setMaxCurrentRaw() can't send get current limit for board" << res->getName() << "IP" << res->getIPv4string() << " joint " << j;
        return false;
    }

    // wait here
    if(-1 == tt->synch() )
    {
        int threadId;
        yError () << "embObjMotionControl::setMaxCurrentRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j;

        if(requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    //get current limit params
    uint16_t size;
    eOmc_current_limits_params_t currentlimits = {0};
    res->readBufferedValue(protid, (uint8_t *)&currentlimits, &size);

    //set current overload
    currentlimits.overloadCurrent = (eOmeas_current_t) S_16(val);

    //send new values
    return res->addSetMessage(protid, (uint8_t*) &currentlimits);
}

bool embObjMotionControl::getMaxCurrentRaw(int j, double *val)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config_currentlimits);
    uint16_t size;
    eOmc_current_limits_params_t currentlimits = {0};
    *val = 0;

    if(!askRemoteValue(protid, (uint8_t *)&currentlimits, size))
    {
        yError() << "embObjMotionControl::getMaxCurrentRaw() could not read max current for  BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;
        return false;
    }

    *val = (double) currentlimits.overloadCurrent;

    return true;
}

bool embObjMotionControl::getAmpStatusRaw(int j, int *st)
{
    (_enabledAmp[j ]) ? *st = 1 : *st = 0;
    return true;
}

bool embObjMotionControl::getAmpStatusRaw(int *sts)
{
    bool ret = true;
    for(int j=0; j<_njoints; j++)
    {
        sts[j] = _enabledAmp[j];
    }

    return ret;
}

#ifdef IMPLEMENT_DEBUG_INTERFACE
//----------------------------------------------\\
//    Debug interface
//----------------------------------------------\\

bool embObjMotionControl::setParameterRaw(int j, unsigned int type, double value)   {       return NOT_YET_IMPLEMENTED("setParameterRaw"); }
bool embObjMotionControl::getParameterRaw(int j, unsigned int type, double* value)  {       return NOT_YET_IMPLEMENTED("getParameterRaw"); }
bool embObjMotionControl::getDebugParameterRaw(int j, unsigned int index, double* value)  { return NOT_YET_IMPLEMENTED("getDebugParameterRaw"); }
bool embObjMotionControl::setDebugParameterRaw(int j, unsigned int index, double value)   { return NOT_YET_IMPLEMENTED("setDebugParameterRaw"); }
bool embObjMotionControl::setDebugReferencePositionRaw(int j, double value)         {       return NOT_YET_IMPLEMENTED("setDebugReferencePositionRaw"); }
bool embObjMotionControl::getDebugReferencePositionRaw(int j, double* value)        {       return NOT_YET_IMPLEMENTED("getDebugReferencePositionRaw");}

#endif //IMPLEMENT_DEBUG_INTERFACE

// Limit interface
bool embObjMotionControl::setLimitsRaw(int j, double min, double max)
{
    bool ret = true;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_userlimits);

    eOmeas_position_limits_t    limits;
    limits.max = (eOmeas_position_t) S_32(max);
    limits.min = (eOmeas_position_t) S_32(min);

    ret = res->addSetMessage(protid, (uint8_t *) &limits);


    if(!ret)
    {
        yError() << "while setting position limits for joint" << j << " \n";
    }
    return ret;
}

bool embObjMotionControl::getLimitsRaw(int j, double *min, double *max)
{
    eOmeas_position_limits_t limits;
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_userlimits);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protoid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if(!res->addGetMessage(protoid) )
    {
        yError() << "Can't send get min position limit request for board" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;
        return false;
    }

    // wait here
    if(-1 == tt->synch() )
    {
        int threadId;
        yError () << "embObjMotionControl::getLimitsRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j;

        if(requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }
    // Get the value
    uint16_t size;

    bool ret = res->readBufferedValue(protoid, (uint8_t *)&limits, &size);

    *min = (double)limits.min + SAFETY_THRESHOLD;
    *max = (double)limits.max - SAFETY_THRESHOLD;
    return ret;
}

bool embObjMotionControl::getGearboxRatioRaw(int j, double *gearbox)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protoid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if (!res->addGetMessage(protoid))
        return false;

    // wait here
    if (-1 == tt->synch())
    {
        int threadId;
        yError() << "embObjMotionControl::getGearbox() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;

        if (requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    res->readBufferedValue(protoid, (uint8_t *)&motor_cfg, &size);

    // refresh cached value when reading data from the EMS
    *gearbox = (double)motor_cfg.gearboxratio;

    return true;
}

bool embObjMotionControl::getRotorLimitsRaw(int j, double *rotorMin, double *rotorMax)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protoid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if (!res->addGetMessage(protoid))
        return false;

    // wait here
    if (-1 == tt->synch())
    {
        int threadId;
        yError() << "embObjMotionControl::getGearbox() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;

        if (requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    res->readBufferedValue(protoid, (uint8_t *)&motor_cfg, &size);

    // refresh cached value when reading data from the EMS
    *rotorMax = (double)motor_cfg.limitsofrotor.max/_angleToEncoder[j];
    *rotorMin = (double)motor_cfg.limitsofrotor.min/_angleToEncoder[j];

    return true;
}

bool embObjMotionControl::getTorqueControlFilterType(int j, int& type)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protoid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if (!res->addGetMessage(protoid))
        return false;

    // wait here
    if (-1 == tt->synch())
    {
        int threadId;
        yError() << "embObjMotionControl::getRotorIndexOffsetRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;

        if (requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_joint_config_t    joint_cfg;
    res->readBufferedValue(protoid, (uint8_t *)&joint_cfg, &size);

    // refresh cached value when reading data from the EMS
    type = (int)joint_cfg.tcfiltertype;
    return true;
}

bool embObjMotionControl::getRotorEncoderResolutionRaw(int j, double &rotres)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protoid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if (!res->addGetMessage(protoid))
        return false;

    // wait here
    if (-1 == tt->synch())
    {
        int threadId;
        yError() << "embObjMotionControl::getRotorEncoderResolutionRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;

        if (requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    res->readBufferedValue(protoid, (uint8_t *)&motor_cfg, &size);

    // refresh cached value when reading data from the EMS
    rotres = (double)motor_cfg.rotorEncoderResolution;

    return true;
}

bool embObjMotionControl::getJointEncoderResolutionRaw(int j, double &jntres)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protoid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if (!res->addGetMessage(protoid))
        return false;

    // wait here
    if (-1 == tt->synch())
    {
        int threadId;
        yError() << "embObjMotionControl::getJointEncoderResolutionRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;

        if (requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_joint_config_t    joint_cfg;
    res->readBufferedValue(protoid, (uint8_t *)&joint_cfg, &size);

    // refresh cached value when reading data from the EMS
    jntres = (double)joint_cfg.jntEncoderResolution;

    return true;
}

bool embObjMotionControl::getJointEncoderTypeRaw(int j, int &type)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protoid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if (!res->addGetMessage(protoid))
        return false;

    // wait here
    if (-1 == tt->synch())
    {
        int threadId;
        yError() << "embObjMotionControl::getJointEncoderResolutionRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;

        if (requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_joint_config_t    joint_cfg;
    res->readBufferedValue(protoid, (uint8_t *)&joint_cfg, &size);

    // refresh cached value when reading data from the EMS
    type = (int)joint_cfg.jntEncoderType;

    return true;
}

bool embObjMotionControl::getRotorEncoderTypeRaw(int j, int &type)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protoid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if (!res->addGetMessage(protoid))
        return false;

    // wait here
    if (-1 == tt->synch())
    {
        int threadId;
        yError() << "embObjMotionControl::getJointEncoderResolutionRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;

        if (requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    res->readBufferedValue(protoid, (uint8_t *)&motor_cfg, &size);

    // refresh cached value when reading data from the EMS
    type = (int)motor_cfg.rotorEncoderType;

    return true;
}

bool embObjMotionControl::getKinematicMJRaw(int j, double &rotres)
{
    yError("getKinematicMJRaw not yet  implemented");
    return false;
}

bool embObjMotionControl::getHasTempSensorsRaw(int j, int& ret)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protoid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if (!res->addGetMessage(protoid))
        return false;

    // wait here
    if (-1 == tt->synch())
    {
        int threadId;
        yError() << "embObjMotionControl::getHasTempSensorsRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;

        if (requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    res->readBufferedValue(protoid, (uint8_t *)&motor_cfg, &size);

    // refresh cached value when reading data from the EMS
    ret = (int)motor_cfg.hasTempSensor;

    return true;
}

bool embObjMotionControl::getHasHallSensorRaw(int j, int& ret)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protoid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if (!res->addGetMessage(protoid))
        return false;

    // wait here
    if (-1 == tt->synch())
    {
        int threadId;
        yError() << "embObjMotionControl::getHasHallSensorsRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;

        if (requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    res->readBufferedValue(protoid, (uint8_t *)&motor_cfg, &size);

    // refresh cached value when reading data from the EMS
    ret = (int)motor_cfg.hasHallSensor;

    return true;
}

bool embObjMotionControl::getHasRotorEncoderRaw(int j, int& ret)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protoid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if (!res->addGetMessage(protoid))
        return false;

    // wait here
    if (-1 == tt->synch())
    {
        int threadId;
        yError() << "embObjMotionControl::getHasRotorEncoderRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;

        if (requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    res->readBufferedValue(protoid, (uint8_t *)&motor_cfg, &size);

    // refresh cached value when reading data from the EMS
    ret = (int)motor_cfg.hasRotorEncoder;

    return true;
}

bool embObjMotionControl::getHasRotorEncoderIndexRaw(int j, int& ret)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protoid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if (!res->addGetMessage(protoid))
        return false;

    // wait here
    if (-1 == tt->synch())
    {
        int threadId;
        yError() << "embObjMotionControl::getHasRotorEncoderIndexRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;

        if (requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    res->readBufferedValue(protoid, (uint8_t *)&motor_cfg, &size);

    // refresh cached value when reading data from the EMS
    ret = (int)motor_cfg.hasRotorEncoderIndex;

    return true;
}

bool embObjMotionControl::getMotorPolesRaw(int j, int& poles)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protoid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if (!res->addGetMessage(protoid))
        return false;

    // wait here
    if (-1 == tt->synch())
    {
        int threadId;
        yError() << "embObjMotionControl::getMotorPolesRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;

        if (requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    res->readBufferedValue(protoid, (uint8_t *)&motor_cfg, &size);

    // refresh cached value when reading data from the EMS
    poles = (int)motor_cfg.motorPoles;

    return true;
}

bool embObjMotionControl::getRotorIndexOffsetRaw(int j, double& rotorOffset)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protoid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if (!res->addGetMessage(protoid))
        return false;

    // wait here
    if (-1 == tt->synch())
    {
        int threadId;
        yError() << "embObjMotionControl::getRotorIndexOffsetRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;

        if (requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    res->readBufferedValue(protoid, (uint8_t *)&motor_cfg, &size);

    // refresh cached value when reading data from the EMS
    rotorOffset = (double)motor_cfg.rotorIndexOffset;

    return true;
}

bool embObjMotionControl::getCurrentPidRaw(int j, Pid *pid)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protoid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if (!res->addGetMessage(protoid))
        return false;

    // wait here
    if (-1 == tt->synch())
    {
        int threadId;
        yError() << "embObjMotionControl::getRotorIndexOffsetRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;

        if (requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    res->readBufferedValue(protoid, (uint8_t *)&motor_cfg, &size);

    // refresh cached value when reading data from the EMS
    eOmc_PID_t tmp = (eOmc_PID_t)motor_cfg.pidcurrent;
    copyPid_eo2iCub(&tmp, pid);

    return true;
}

bool embObjMotionControl::getAxisNameRaw(int axis, yarp::os::ConstString& name)
{
    if (axis >= 0 && axis < _njoints)
    {
        name = _axisName[axis];
        return true;
    }
    else
    {
        name = "ERROR";
        return false;
    }
}

bool embObjMotionControl::getJointTypeRaw(int axis, yarp::dev::JointTypeEnum& type)
{
    if (axis >= 0 && axis < _njoints)
    {
        type = _jointType[axis];
        return true;
    }
    else
    {
        return false;
    }
}

// IRemoteVariables
bool embObjMotionControl::getRemoteVariableRaw(yarp::os::ConstString key, yarp::os::Bottle& val)
{
    val.clear();
    if (key == "kinematic_mj")
    {
        Bottle& r = val.addList(); for (int i = 0; i<16; i++) { r.addDouble(_kinematic_mj[i]); }
        return true;
    }
    else if (key == "encoders")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { r.addDouble(_angleToEncoder[i]); }
        return true;
    }
    else if (key == "rotorEncoderResolution")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp = 0; getRotorEncoderResolutionRaw(i, tmp);  r.addDouble(tmp); }
        return true;
    }
    else if (key == "jointEncoderResolution")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp = 0; getJointEncoderResolutionRaw(i, tmp);  r.addDouble(tmp); }
        return true;
    }
    else if (key == "gearbox")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp=0; getGearboxRatioRaw(i, &tmp);  r.addDouble(tmp); }
        return true;
    }
    else if (key == "hasHallSensor")
    {
        Bottle& r = val.addList(); for (int i = 0; i < _njoints; i++) { int tmp = 0; getHasHallSensorRaw(i, tmp); r.addInt(tmp); }
        return true;
    }
    else if (key == "hasTempSensor")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { int tmp = 0; getHasTempSensorsRaw(i, tmp); r.addInt(tmp); }
        return true;
    }
    else if (key == "hasRotorEncoder")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { int tmp = 0; getHasRotorEncoderRaw(i, tmp); r.addInt(tmp); }
        return true;
    }
    else if (key == "hasRotorEncoderIndex")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { int tmp = 0; getHasRotorEncoderIndexRaw(i, tmp); r.addInt(tmp); }
        return true;
    }
    else if (key == "rotorIndexOffset")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp = 0; getRotorIndexOffsetRaw(i, tmp);  r.addDouble(tmp); }
        return true;
    }
    else if (key == "motorPoles")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { int tmp = 0; getMotorPolesRaw(i, tmp); r.addInt(tmp); }
        return true;
    }
    else if (key == "pidCurrentKp")
    {
        Bottle& r = val.addList(); for (int i = 0; i < _njoints; i++) { Pid p; getCurrentPidRaw(i, &p); r.addDouble(p.kp); }
        return true;
    }
    else if (key == "pidCurrentKi")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { Pid p; getCurrentPidRaw(i, &p); r.addDouble(p.ki); }
        return true;
    }
    else if (key == "pidCurrentShift")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++)  { Pid p; getCurrentPidRaw(i, &p); r.addDouble(p.scale); }
        return true;
    }
    else if (key == "pidCurrentOutput")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++)  { Pid p; getCurrentPidRaw(i, &p); r.addDouble(p.max_output); }
        return true;
    }
    else if (key == "jointEncoderType")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++)
        {
            int t; string s;
            getJointEncoderTypeRaw(i, t); uint8_t tt = t; bool b = EncoderType_eo2iCub(&tt, &s);
            if (b == false)
            {
                yError("Invalid jointEncoderType");
            }
            r.addString(s);
        }
        return true;
    }
    else if (key == "rotorEncoderType")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++)
        {
            int t; string s;
            getRotorEncoderTypeRaw(i, t); uint8_t tt = t; bool b = EncoderType_eo2iCub(&tt, &s);
            if (b == false)
            {
                yError("Invalid motorEncoderType");
            }
            r.addString(s);
        }
        return true;
    }
    else if (key == "coulombThreshold")
    {
        val.addString("not implemented yet");
        return true;
    }
    else if (key == "torqueControlFilterType")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++)  { int t; getTorqueControlFilterType(i, t); r.addDouble(t); }
        return true;
    }
    else if (key == "torqueControlEnabled")
    {

        Bottle& r = val.addList();
        for(int i = 0; i<_njoints; i++)
        {
            r.addInt((int)_torqueControlEnabled);
        }
        return true;
    }
    else if (key == "PWMLimit")
    {
        Bottle& r = val.addList(); for (int i = 0; i< _njoints; i++) { double tmp = 0; getPWMLimitRaw(i, &tmp);  r.addDouble(tmp); }
        return true;
    }
    else if (key == "motOverloadCurr")
    {
        Bottle& r = val.addList(); for (int i = 0; i< _njoints; i++) { double tmp = 0; getMaxCurrentRaw(i, &tmp);  r.addDouble(tmp); }
        return true;
    }
    else if (key == "motNominalCurr")
    {
        Bottle& r = val.addList(); for (int i = 0; i< _njoints; i++) { double tmp = 0; getNominalCurrentRaw(i, &tmp);  r.addDouble(tmp); }
        return true;
    }
    else if (key == "motPeakCurr")
    {
        Bottle& r = val.addList(); for (int i = 0; i< _njoints; i++) { double tmp = 0; getPeakCurrentRaw(i, &tmp);  r.addDouble(tmp); }
        return true;
    }
    else if (key == "PowerSuppVoltage")
    {
        Bottle& r = val.addList(); for (int i = 0; i< _njoints; i++) { double tmp = 0; getPowerSupplyVoltageRaw(i, &tmp);  r.addDouble(tmp); }
        return true;
    }
    else if (key == "rotorMax")
    {
        double tmp1, tmp2;
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp = 0; getRotorLimitsRaw(i, &tmp1, &tmp2);  r.addDouble(tmp2); }
        return true;
    }
    else if (key == "rotorMin")
    {
        double tmp1, tmp2;
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp = 0; getRotorLimitsRaw(i, &tmp1, &tmp2);  r.addDouble(tmp1); }
        return true;
    }
    else if (key == "jointMax")
    {
        double tmp1, tmp2;
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp = 0; getLimitsRaw(i, &tmp1, &tmp2);  r.addDouble(tmp2); }
        return true;
    }
    else if (key == "jointMin")
    {
        double tmp1, tmp2;
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp = 0; getLimitsRaw(i, &tmp1, &tmp2);  r.addDouble(tmp1); }
        return true;
    }
    yWarning("getRemoteVariable(): Unknown variable %s", key.c_str());
    return false;
}

bool embObjMotionControl::setRemoteVariableRaw(yarp::os::ConstString key, const yarp::os::Bottle& val)
{
    string s1 = val.toString();
    if (val.size() != _njoints)
    {
        yWarning("setRemoteVariable(): Protocol error %s", s1.c_str());
        return false;
    }

    if (key == "kinematic_mj")
    {
        return true;
    }
    else if (key == "rotor")
    {
        for (int i = 0; i < _njoints; i++) _rotorEncoderRes[i] = val.get(i).asInt();
        return true;
    }
    else if (key == "gearbox")
    {
        for (int i = 0; i < _njoints; i++) _gearbox[i] = val.get(i).asDouble();
        return true;
    }
    else if (key == "PWMLimit")
    {
        for (int i = 0; i < _njoints; i++) setPWMLimitRaw(i, val.get(i).asDouble());
        return true;
    }
    //disabled for used safety
#if 0
    else if (key == "jointMax")
    {
        double min, max;
        for (int i = 0; i < _njoints; i++)
        {
            getLimitsRaw(i, &min, &max);
            setLimitsRaw(i, min, val.get(i).asDouble());
        }
        return true;
    }
    else if (key == "jointMin")
    {
        double min, max;
        for (int i = 0; i < _njoints; i++)
        {
            getLimitsRaw(i, &min, &max);
            setLimitsRaw(i, val.get(i).asDouble(), max);
        }
    }
#endif
    yWarning("setRemoteVariable(): Unknown variable %s", key.c_str());
    return false;
}

bool embObjMotionControl::getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys)
{
    listOfKeys->clear();
    listOfKeys->addString("kinematic_mj");
    listOfKeys->addString("encoders");
    listOfKeys->addString("gearbox");
    listOfKeys->addString("hasHallSensor");
    listOfKeys->addString("hasTempSensor");
    listOfKeys->addString("hasRotorEncoder");
    listOfKeys->addString("hasRotorEncoderIndex");
    listOfKeys->addString("rotorIndexOffset");
    listOfKeys->addString("rotorEncoderResolution");
    listOfKeys->addString("jointEncoderResolution");
    listOfKeys->addString("motorPoles");
    listOfKeys->addString("pidCurrentKp");
    listOfKeys->addString("pidCurrentKi");
    listOfKeys->addString("pidCurrentShift");
    listOfKeys->addString("pidCurrentOutput");
    listOfKeys->addString("coulombThreshold");
    listOfKeys->addString("torqueControlFilterType");
    listOfKeys->addString("jointEncoderType");
    listOfKeys->addString("rotorEncoderType");
    listOfKeys->addString("PWMLimit");
    listOfKeys->addString("motOverloadCurr");
    listOfKeys->addString("motNominalCurr");
    listOfKeys->addString("motPeakCurr");
    listOfKeys->addString("PowerSuppVoltage");
    listOfKeys->addString("rotorMax");
    listOfKeys->addString("rotorMin");
    listOfKeys->addString("jointMax");
    listOfKeys->addString("jointMin");
    return true;
}

// IControlLimits2
bool embObjMotionControl::setVelLimitsRaw(int axis, double min, double max)
{
    return NOT_YET_IMPLEMENTED("setVelLimitsRaw");
}

bool embObjMotionControl::getVelLimitsRaw(int axis, double *min, double *max)
{
    eOmc_joint_config_t jconfig;
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, axis, eoprot_tag_mc_joint_config);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(axis, protoid);
    tt->setPending(1);

    if (!res->addGetMessage(protoid))
    {
        yError() << "Can't send getVelLimits request for BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << axis;
        return false;
    }

    // wait here
    if (-1 == tt->synch())
    {
        int threadId;
        yError() << "embObjMotionControl::getVelLimitsRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << axis;

        if (requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }
    // Get the value
    uint16_t size;

    bool ret = res->readBufferedValue(protoid, (uint8_t *)&jconfig, &size);

    *max = jconfig.maxvelocityofjoint;
    *min = 0;

    return ret;
}


/*
 * IVirtualAnalogSensor Interface
 *
 *  DEPRECATED!! WILL BE REMOVED IN THE NEAR FUTURE!!
 *
 */

int embObjMotionControl::getState(int ch)
{
    return VAS_OK;
};

int embObjMotionControl::getChannels()
{
    return _njoints;
};

bool embObjMotionControl::updateMeasure(yarp::sig::Vector &fTorques)
{
    bool ret = true;

    for(int j=0; j< _njoints; j++)
    {
        ret = ret && updateMeasure(j, fTorques[j]);
    }
    return ret;
}

bool embObjMotionControl::updateMeasure(int userLevel_jointNumber, double &fTorque)
{
    int j = _axisMap[userLevel_jointNumber];

    eOmeas_torque_t meas_torque = 0;
    static double curr_time = Time::now();
    static int    count_saturation=0;

    meas_torque = (eOmeas_torque_t) S_32(_newtonsToSensor[j]*fTorque);

    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_inputs_externallymeasuredtorque);
    return res->addSetMessageAndCacheLocally(protoid, (uint8_t*) &meas_torque);
}

// end  IVirtualAnalogSensor //


// Torque control
bool embObjMotionControl::setTorqueModeRaw()
{
    bool ret = true;
    eOmc_controlmode_command_t val = eomc_controlmode_cmd_torque;
    for(int j=0; j<_njoints; j++)
    {
        eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_controlmode);
        ret &= res->addSetMessage(protid, (uint8_t*) &val);
    }
    return ret;
}

bool embObjMotionControl::getTorqueRaw(int j, double *t)
{
    eOmeas_torque_t meas_torque = 0;
    uint16_t size;
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_inputs_externallymeasuredtorque);
    bool ret = res->readSentValue(protoid, (uint8_t*) &meas_torque, &size);
    *t = (double) meas_torque / _newtonsToSensor[j];
    return ret;
}

bool embObjMotionControl::getTorquesRaw(double *t)
{
    bool ret = true;
    for(int j=0; j<_njoints; j++)
        ret = ret && getTorqueRaw(j, &t[j]);
    return true;
}

bool embObjMotionControl::getTorqueRangeRaw(int j, double *min, double *max)
{
    return NOT_YET_IMPLEMENTED("getTorqueRangeRaw");
}

bool embObjMotionControl::getTorqueRangesRaw(double *min, double *max)
{
    return NOT_YET_IMPLEMENTED("getTorqueRangesRaw");
}

bool embObjMotionControl::setRefTorquesRaw(const double *t)
{
    bool ret = true;
    for(int j=0; j<_njoints && ret; j++)
        ret &= setRefTorqueRaw(j, t[j]);
    return ret;
}

bool embObjMotionControl::setRefTorqueRaw(int j, double t)
{
    eOmc_setpoint_t setpoint;
    setpoint.type = (eOenum08_t) eomc_setpoint_torque;
    setpoint.to.torque.value =  (eOmeas_torque_t) S_32(t);

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);
    return res->addSetMessage(protid, (uint8_t*) &setpoint);
}

bool embObjMotionControl::setRefTorquesRaw(const int n_joint, const int *joints, const double *t)
{
    bool ret = true;
    for(int j=0; j< n_joint; j++)
    {
        ret &= setRefTorqueRaw(joints[j], t[j]);
    }
    return ret;
}

bool embObjMotionControl::getRefTorquesRaw(double *t)
{
    bool ret = true;
    for(int j=0; j<_njoints && ret; j++)
        ret &= getRefTorqueRaw(j, &t[j]);
    return ret;
}

bool embObjMotionControl::getRefTorqueRaw(int j, double *t)
{
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core);
    uint16_t size;
    eOmc_joint_status_core_t jcore = {0};
    *t =0 ;


    if(!res->readBufferedValue(id32, (uint8_t *)&jcore, &size))
    {
        yError() << "embObjMotionControl::getRefTorqueRaw() could not read pid torque reference pos for  BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;
        return false;
    }
#if NEW_JSTATUS_STRUCT

    if ((eOmc_interactionmode_compliant == jcore.modes.interactionmodestatus) &&
        (eomc_controlmode_position == jcore.modes.controlmodestatus))
    {
        *t = (double) jcore.ofpid.complpos.reftrq;
    }

    if(eomc_controlmode_torque == jcore.modes.controlmodestatus)
    {
        *t = (double) jcore.ofpid.torque.reftrq;
    }

    return true;

#else
    *t = (double) jstatus.ofpid.legacy.torquereference;
    return true;
#endif


}

bool embObjMotionControl::setTorquePidRaw(int j, const Pid &pid)
{
    eOmc_PID_t  outPid;
    Pid hwPid = pid;
    hwPid.kp = hwPid.kp / _torqueControlHelper->getNewtonsToSensor(j);  //[PWM/Nm]
    hwPid.ki = hwPid.ki / _torqueControlHelper->getNewtonsToSensor(j);  //[PWM/Nm]
    hwPid.kd = hwPid.kd / _torqueControlHelper->getNewtonsToSensor(j);  //[PWM/Nm]
    hwPid.stiction_up_val   = hwPid.stiction_up_val   * _torqueControlHelper->getNewtonsToSensor(j);  //[Nm]
    hwPid.stiction_down_val = hwPid.stiction_down_val * _torqueControlHelper->getNewtonsToSensor(j);  //[Nm]
    //printf("DEBUG setTorquePidRaw: %f %f %f %f %f\n",hwPid.kp ,  hwPid.ki, hwPid.kd , hwPid.stiction_up_val , hwPid.stiction_down_val );

    copyPid_iCub2eo(&hwPid, &outPid);
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_pidtorque);
    return res->addSetMessage(protid, (uint8_t *)&outPid);
}

bool embObjMotionControl::setTorquePidsRaw(const Pid *pids)
{
    bool ret = true;
    for(int j=0; j<_njoints && ret; j++)
        ret &= setTorquePidRaw(j, pids[j]);
    return ret;
}

bool embObjMotionControl::getTorqueErrorRaw(int j, double *err)
{
    uint16_t size = 0;
    bool ret = true;
    eOmc_joint_status_core_t jcore = {0};

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core);
    *err = 0;
    if(!res->readBufferedValue(protid, (uint8_t *)&jcore, &size))
        return false;


#if NEW_JSTATUS_STRUCT


    if ((eOmc_interactionmode_compliant == jcore.modes.interactionmodestatus) &&
        (eomc_controlmode_position == jcore.modes.controlmodestatus))
    {
        *err = (double) jcore.ofpid.complpos.errtrq;
    }

    if(eomc_controlmode_torque == jcore.modes.controlmodestatus)
    {
        *err = (double) jcore.ofpid.torque.errtrq;
    }

    return true;


#else
    int mycontrolMode;
    getControlModeRaw(j, &mycontrolMode);
    if(VOCAB_CM_TORQUE != mycontrolMode)
    {
        //yWarning() << "Asked for Torque PID Error while not in Torque control mode. Returning zeros";
        err = 0;
        return false;
    }
    *err = (double) jstatus.ofpid.legacy.error;
    return true;
#endif

}

bool embObjMotionControl::getTorqueErrorsRaw(double *errs)
{
    bool ret = true;
    for(int j=0; j<_njoints && ret; j++)
        ret &= getTorqueErrorRaw(j, &errs[j]);
    return ret;
}

bool embObjMotionControl::getTorquePidRaw(int j, Pid *pid)
{
    //_mutex.wait();
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_pidtorque);

    // Sign up for waiting the reply FIRST OF ALL!!
    eoThreadEntry *tt = appendWaitRequest(j, protid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if(!res->addGetMessage(protid) )
        return false;

    // wait here
    if(-1 == tt->synch() )
    {
        int threadId;
        yError () << "embObjMotionControl::getTorquePidRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;


        if(requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_PID_t eoPID;
    bool ret = res->readBufferedValue(protid, (uint8_t *)&eoPID, &size);
    copyPid_eo2iCub(&eoPID, pid);
    //printf("DEBUG getTorquePidRaw: %f %f %f %f %f\n",pid->kp , pid->ki, pid->kd , pid->stiction_up_val , pid->stiction_down_val );

    pid->kp = pid->kp * _torqueControlHelper->getNewtonsToSensor(j); //[PWM/Nm]
    pid->ki = pid->ki * _torqueControlHelper->getNewtonsToSensor(j); //[PWM/Nm]
    pid->kd = pid->kd * _torqueControlHelper->getNewtonsToSensor(j); //[PWM/Nm]
    pid->stiction_up_val   = pid->stiction_up_val   / _torqueControlHelper->getNewtonsToSensor(j); //[Nm]
    pid->stiction_down_val = pid->stiction_down_val / _torqueControlHelper->getNewtonsToSensor(j); //[Nm]

    return ret;
}

bool embObjMotionControl::getTorquePidsRaw(Pid *pids)
{
    // first set is done in the open function because the whole joint config is sent to the EMSs
    bool ret = true;
       // just one joint at time, wait for the answer before getting to the next.
    // This is because otherwise too many msg will be placed into EMS can queue
    for(int j=0, index=0; j<_njoints; j++, index++)
    {
        ret &=getTorquePidRaw(j, &pids[j]);
    }
    return ret;
}

bool embObjMotionControl::getImpedanceRaw(int j, double *stiffness, double *damping)
{
    // first set is done in the open function because the whole joint config is sent to the EMSs
    eOmc_impedance_t val;

    if(!getWholeImpedanceRaw(j, val))
        return false;

    *stiffness = (double) (val.stiffness);
    *damping = (double) (val.damping);
    return true;
}

bool embObjMotionControl::getWholeImpedanceRaw(int j, eOmc_impedance_t &imped)
{
    // first set is done in the open function because the whole joint config is sent to the EMSs

    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_impedance);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protoid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if(!res->addGetMessage(protoid) )
        return false;

    // wait here
    if(-1 == tt->synch() )
    {
        int threadId;
        yError () << "embObjMotionControl::getWholeImpedanceRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;

        if(requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    res->readBufferedValue(protoid, (uint8_t *)&imped, &size);

    // refresh cached value when reading data from the EMS
    _cacheImpedance->damping   = (double) imped.damping;
    _cacheImpedance->stiffness = (double) imped.stiffness;
    _cacheImpedance->offset    = (double) imped.offset;
    return true;
}

bool embObjMotionControl::setImpedanceRaw(int j, double stiffness, double damping)
{
    bool ret = true;
    eOmc_impedance_t val;

    // Need to read the whole struct and modify just 2 of them -> now aching the old values and re-using them.
    // first set is done in the open function because the whole joint config is sent to the EMSs
    // cleaner solution, split the impedance structure into 2 separeted nework variables
//    if(!getWholeImpedanceRaw(j, val))
//        return false;

    _cacheImpedance[j].stiffness = (eOmeas_stiffness_t) stiffness;
    _cacheImpedance[j].damping   = (eOmeas_damping_t) damping;

    val.stiffness   = _cacheImpedance[j].stiffness;
    val.damping     = _cacheImpedance[j].damping;
    val.offset      = _cacheImpedance[j].offset;

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_impedance);


    ret &= res->addSetMessage(protid, (uint8_t *) &val);
    return ret;
}

bool embObjMotionControl::setImpedanceOffsetRaw(int j, double offset)
{
    bool ret = true;
    eOmc_impedance_t val;

    // first set is done in the open function because the whole joint config is sent to the EMSs
//    if(!getWholeImpedanceRaw(j, val))
//        return false;

    _cacheImpedance[j].offset   = (eOmeas_torque_t) S_32(offset);
    val.stiffness     = _cacheImpedance[j].stiffness;
    val.damping     = _cacheImpedance[j].damping;
    val.offset      = _cacheImpedance[j].offset;

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_impedance);


    ret &= res->addSetMessage(protid, (uint8_t *) &val);

    return ret;
}

bool embObjMotionControl::getImpedanceOffsetRaw(int j, double *offset)
{
    eOmc_impedance_t val;

    if(!getWholeImpedanceRaw(j, val))
        return false;

    *offset = val.offset;
    return true;
}

bool embObjMotionControl::getCurrentImpedanceLimitRaw(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp)
{
    *min_stiff = _impedance_limits[j].min_stiff;
    *max_stiff = _impedance_limits[j].max_stiff;
    *min_damp  = _impedance_limits[j].min_damp;
    *max_damp  = _impedance_limits[j].max_damp;
    return true;
}

bool embObjMotionControl::getBemfParamRaw(int j, double *bemf)
{
    return DEPRECATED("getBemfParamRaw");
}

bool embObjMotionControl::setBemfParamRaw(int j, double bemf)
{
    return DEPRECATED("setBemfParamRaw");
}

bool embObjMotionControl::setTorqueErrorLimitRaw(int j, double limit)
{
    return NOT_YET_IMPLEMENTED("setTorqueErrorLimitRaw");
}

bool embObjMotionControl::setTorqueErrorLimitsRaw(const double *limits)
{
    return NOT_YET_IMPLEMENTED("setTorqueErrorLimitsRaw");
}

bool embObjMotionControl::getTorquePidOutputRaw(int j, double *out)
{
        uint16_t size = 0;
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core);
    eOmc_joint_status_core_t  jcore = {0};
    *out = 0;
    if(!res->readBufferedValue(id32, (uint8_t *)&jcore, &size))
        return false;



    if((eomc_controlmode_torque == jcore.modes.controlmodestatus)   ||
       ((eomc_controlmode_position == jcore.modes.controlmodestatus) && (eOmc_interactionmode_compliant == jcore.modes.interactionmodestatus)))
       *out = jcore.ofpid.generic.output;

    return true;
}

bool embObjMotionControl::getTorquePidOutputsRaw(double *outs)
{
    return NOT_YET_IMPLEMENTED("getTorquePidOutputsRaw");
}

bool embObjMotionControl::getTorqueErrorLimitRaw(int j, double *limit)
{
    return NOT_YET_IMPLEMENTED("getTorqueErrorLimitRaw");
}

bool embObjMotionControl::getTorqueErrorLimitsRaw(double *limits)
{
    return NOT_YET_IMPLEMENTED("getTorqueErrorLimitsRaw");
}

bool embObjMotionControl::resetTorquePidRaw(int j)
{
    return NOT_YET_IMPLEMENTED("resetTorquePidRaw");
}

bool embObjMotionControl::disableTorquePidRaw(int j)
{
    return NOT_YET_IMPLEMENTED("disableTorquePidRaw");
}

bool embObjMotionControl::enableTorquePidRaw(int j)
{
    return NOT_YET_IMPLEMENTED("enableTorquePidRaw");
}

bool embObjMotionControl::setTorqueOffsetRaw(int j, double v)
{
    return NOT_YET_IMPLEMENTED("setTorqueOffsetRaw");
}

bool embObjMotionControl::getMotorTorqueParamsRaw(int j, MotorTorqueParameters *params)
{
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_motor_params);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, id32);
    tt->setPending(1);

    if(!res->addGetMessage(id32) )
    {
        yError() << "embObjMotionControl::getMotorTorqueParamsRaw() could not send get message for BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;
        return false;
    }

    // wait here
    if(-1 == tt->synch() )
    {
        int threadId;
        yError () << "embObjMotionControl::getMotorTorqueParamsRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;

        if(requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_motor_params_t eo_params = {0};
    res->readBufferedValue(id32, (uint8_t *)&eo_params, &size);

    params->bemf       = eo_params.bemf_value / _torqueControlHelper->getNewtonsToSensor(j) *  _torqueControlHelper->getAngleToEncoders(j);  //[Nm/deg/s]
    params->bemf_scale = eo_params.bemf_scale;
    params->ktau       = eo_params.ktau_value * _torqueControlHelper->getNewtonsToSensor(j);  //[PWM/Nm]
    params->ktau_scale = eo_params.ktau_scale;
    //printf("debug getMotorTorqueParamsRaw %f %f %f %f\n",  params->bemf, params->bemf_scale, params->ktau,params->ktau_scale);

    return true;
}

bool embObjMotionControl::setMotorTorqueParamsRaw(int j, const MotorTorqueParameters params)
{
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_motor_params);
    eOmc_motor_params_t eo_params = {0};

    //printf("getAngleToEncoders: %f\n",_torqueControlHelper->getAngleToEncoders(j));

    eo_params.bemf_value    = (float) params.bemf * _torqueControlHelper->getNewtonsToSensor(j) /  _torqueControlHelper->getAngleToEncoders(j); //[Nm/deg/s]
    eo_params.bemf_scale    = (uint8_t) params.bemf_scale;
    eo_params.ktau_value    = (float) params.ktau / _torqueControlHelper->getNewtonsToSensor(j); //[PWM/Nm]
    eo_params.ktau_scale    = (uint8_t) params.ktau_scale;
    //printf("DEBUG setMotorTorqueParamsRaw: %f %f %f %f\n",  params.bemf, params.bemf_scale, params.ktau,params.ktau_scale);

    if(!res->addSetMessage(id32, (uint8_t *) &eo_params))
    {
        yError() << "embObjMotionControl::setMotorTorqueParamsRaw() could not send set message for BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;
        return false;
    }

    return true;
}

// IVelocityControl2
bool embObjMotionControl::velocityMoveRaw(const int n_joint, const int *joints, const double *spds)
{
    bool ret = true;

    for(int j=0; j< n_joint; j++)
    {
        ret &= velocityMoveRaw(joints[j], spds[j]);
    }
    return ret;
}

bool embObjMotionControl::setVelPidRaw(int j, const Pid &pid)
{
    eOprotID32_t protoId = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_pidvelocity);
    eOmc_PID_t  outPid;
    Pid hwPid = pid;

    if (_positionControlUnits == P_METRIC_UNITS)
    {
        hwPid.kp = hwPid.kp / _angleToEncoder[j];  //[PWM/deg]
        hwPid.ki = hwPid.ki / _angleToEncoder[j];  //[PWM/deg]
        hwPid.kd = hwPid.kd / _angleToEncoder[j];  //[PWM/deg]
    }
    else if (_positionControlUnits == P_MACHINE_UNITS)
    {
        hwPid.kp = hwPid.kp;  //[PWM/icubdegrees]
        hwPid.ki = hwPid.ki;  //[PWM/icubdegrees]
        hwPid.kd = hwPid.kd;  //[PWM/icubdegrees]
    }
    else
    {
        yError() << "Unknown _positionControlUnits, needed by setVelPidRaw()";
    }

    copyPid_iCub2eo(&hwPid, &outPid);

    if (!res->addSetMessage(protoId, (uint8_t *)&outPid))
    {
        yError() << "while setting velocity PIDs for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j;
        return false;
    }

    return NOT_YET_IMPLEMENTED("Our boards do not have a Velocity Pid");
}

bool embObjMotionControl::setVelPidsRaw(const Pid *pids)
{
    bool ret = true;
    for (int j = 0; j< _njoints; j++)
    {
        ret &= setVelPidRaw(j, pids[j]);
    }
    return ret;
}

bool embObjMotionControl::getVelPidRaw(int j, Pid *pid)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_pidvelocity);
    // Sign up for waiting the reply

    eoThreadEntry *tt = appendWaitRequest(j, protid);
    tt->setPending(1);

    if (!res->addGetMessage(protid))
    {
        yError() << "Can't send getVelPidRaw() request for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j;
        return false;
    }

    // wait here
    if (-1 == tt->synch())
    {
        int threadId;
        yError() << "embObjMotionControl::getVelPidRaw() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j;

        if (requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_PID_t eoPID;
    res->readBufferedValue(protid, (uint8_t *)&eoPID, &size);

    copyPid_eo2iCub(&eoPID, pid);

    if (_positionControlUnits == P_METRIC_UNITS)
    {
        pid->kp = pid->kp * _angleToEncoder[j];  //[PWM/deg]
        pid->ki = pid->ki * _angleToEncoder[j];  //[PWM/deg]
        pid->kd = pid->kd * _angleToEncoder[j];  //[PWM/deg]
    }
    else if (_positionControlUnits == P_MACHINE_UNITS)
    {
        pid->kp = pid->kp;  //[PWM/icubdegrees]
        pid->ki = pid->ki;  //[PWM/icubdegrees]
        pid->kd = pid->kd;  //[PWM/icubdegrees]
    }
    else
    {
        yError() << "Unknown _positionControlUnits needed by getVelPid()";
    }

    return NOT_YET_IMPLEMENTED("Our boards do not have a Velocity Pid");
}

bool embObjMotionControl::getVelPidsRaw(Pid *pids)
{
    bool ret = true;

    // just one joint at time, wait answer before getting to the next.
    // This is because otherwise too many msg will be placed into can queue
    for (int j = 0, index = 0; j<_njoints; j++, index++)
    {
        ret &= getVelPidRaw(j, &pids[j]);
    }
    return ret;
}

// PositionDirect Interface
bool embObjMotionControl::setPositionDirectModeRaw()
{
    return DEPRECATED("setPositionDirectModeRaw");
}

bool embObjMotionControl::setPositionRaw(int j, double ref)
{
    // does the same as setReferenceRaw, with some more misterious (missing) checks.
    int mode = 0;
    getControlModeRaw(j, &mode);
    if (mode != VOCAB_CM_POSITION_DIRECT &&
        mode != VOCAB_CM_IDLE)
    {
        #ifdef ICUB_AUTOMATIC_MODE_SWITCHING
        yWarning() << "setPositionRaw: Deprecated automatic switch to VOCAB_CM_POSITION_DIRECT, BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j;
        setControlModeRaw(j,VOCAB_CM_POSITION_DIRECT);
        #else
        yError() << "setPositionRaw: skipping command because BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " is not in VOCAB_CM_POSITION_DIRECT mode";
        #endif
    }

    return setReferenceRaw(j, ref);
}

bool embObjMotionControl::setPositionsRaw(const int n_joint, const int *joints, double *refs)
{
    bool ret = true;
    for(int i=0; i<n_joint; i++)
    {
        ret &= setPositionRaw(joints[i], refs[i]);
    }
    return ret;
}

bool embObjMotionControl::setPositionsRaw(const double *refs)
{
    bool ret = true;
    for (int i = 0; i<_njoints; i++)
    {
        ret &= setPositionRaw(i, refs[i]);
    }
    return ret;
}


bool embObjMotionControl::getTargetPositionRaw(int axis, double *ref)
{
    if (axis<0 || axis>_njoints) return false;
#if ASK_REFERENCE_TO_FIRMWARE
   eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, axis, eoprot_tag_mc_joint_status_target);
   *ref = 0;

    // Get the value
    uint16_t size;
    eOmc_joint_status_target_t  target = {0};
    if(!askRemoteValue(id32, (uint8_t *)&target, size))
    {
        yError() << "embObjMotionControl::getTargetPositionRaw() could not read reference pos for  BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << axis;
        return false;
    }

    *ref = (double) target.trgt_position;
    //yError() << "embObjMotionControl::getTargetPositionRaw()  BOARD" << _fId.boardNumber << "joint " << axis << "pos=" << target.trgt_position;
    return true;
#else
    *ref = _ref_command_positions[axis];
    return true;
#endif
}

bool embObjMotionControl::getTargetPositionsRaw(double *refs)
{
    bool ret = true;
    for (int i = 0; i<_njoints; i++)
    {
        ret &= getTargetPositionRaw(i, &refs[i]);
    }
    return ret;
}

bool embObjMotionControl::getTargetPositionsRaw(int nj, const int * jnts, double *refs)
{
    bool ret = true;
    for (int i = 0; i<nj; i++)
    {
        ret &= getTargetPositionRaw(jnts[i], &refs[i]);
    }
    return ret;
}

bool embObjMotionControl::getRefVelocityRaw(int axis, double *ref)
{
    if (axis<0 || axis>_njoints) return false;
#if ASK_REFERENCE_TO_FIRMWARE
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, axis, eoprot_tag_mc_joint_status_target);
    *ref = 0;

    // Get the value
    uint16_t size;
    eOmc_joint_status_target_t  target = {0};
    if(!askRemoteValue(id32, (uint8_t *)&target, size))
    {
        yError() << "embObjMotionControl::getRefVelocityRaw() could not read reference vel for  BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << axis;
        return false;
    }
    *ref = (double) target.trgt_velocity;
    return true;
#else
    *ref = _ref_command_speeds[axis];
    return true;
#endif
}

bool embObjMotionControl::getRefVelocitiesRaw(double *refs)
{
    bool ret = true;
    for (int i = 0; i<_njoints; i++)
    {
        ret &= getRefVelocityRaw(i, &refs[i]);
    }
    return ret;
}

bool embObjMotionControl::getRefVelocitiesRaw(int nj, const int * jnts, double *refs)
{
    bool ret = true;
    for (int i = 0; i<nj; i++)
    {
        ret &= getRefVelocityRaw(jnts[i], &refs[i]);
    }
    return ret;
}

bool embObjMotionControl::getRefPositionRaw(int axis, double *ref)
{
    if (axis<0 || axis>_njoints) return false;
#if ASK_REFERENCE_TO_FIRMWARE
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, axis, eoprot_tag_mc_joint_status_target);
    *ref = 0;
    // Get the value
    uint16_t size;
    eOmc_joint_status_target_t  target = {0};
    if(!askRemoteValue(id32, (uint8_t *)&target, size))
    {
        yError() << "embObjMotionControl::getRefPositionRaw() could not read reference pos for  BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << axis;
        return false;
    }

    *ref = (double) target.trgt_positionraw;
    return true;
#else
    *ref = _ref_positions[axis];
    return true;
#endif
}

bool embObjMotionControl::getRefPositionsRaw(double *refs)
{
    bool ret = true;
    for (int i = 0; i<_njoints; i++)
    {
        ret &= getRefPositionRaw(i, &refs[i]);
    }
    return ret;
}

bool embObjMotionControl::getRefPositionsRaw(int nj, const int * jnts, double *refs)
{
    bool ret = true;
    for (int i = 0; i<nj; i++)
    {
        ret &= getRefPositionRaw(jnts[i], &refs[i]);
    }
    return ret;
}

// InteractionMode



bool embObjMotionControl::getInteractionModeRaw(int j, yarp::dev::InteractionModeEnum* _mode)
{
    uint16_t     size;
    eOenum08_t   interactionmodestatus;
//    std::cout << "eoMC getInteractionModeRaw SINGLE joint " << j << std::endl;

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core_modes_interactionmodestatus);
    if(! res->readBufferedValue(protid, (uint8_t *)&interactionmodestatus, &size)) // it is broadcasted toghether with the jointStatus full
        return false;

    int tmp = (int) *_mode;
    if(!interactionModeStatusConvert_embObj2yarp(interactionmodestatus, tmp) )
        return false;

    *_mode = (yarp::dev::InteractionModeEnum) tmp;
    return true;
}

bool embObjMotionControl::getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
//    std::cout << "eoMC getInteractionModeRaw GROUP joints" << std::endl;
    bool ret = true;
    for(int idx=0; idx<n_joints; idx++)
    {
        ret =  getInteractionModeRaw(joints[idx], &modes[idx]);
        std::cout << " joint " <<  joints[idx] << " says " << yarp::os::Vocab::decode(modes[idx]) << std::endl;
    }
    return ret;
}

bool embObjMotionControl::getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
//    std::cout << "eoMC getInteractionModeRaw ALL joints" << std::endl;
    bool ret = true;
    for(int j=0; j<_njoints; j++)
        ret = ret && getInteractionModeRaw(j, &modes[j]);
    return ret;
}

// marco.accame: con alberto cardellino abbiamo parlato della correttezza di effettuare la verifica di quanto imposto (in setInteractionModeRaw() ed affini)
// andando a rileggere il valore nella scheda eth fino a che esso non sia quello atteso. si deve fare oppure no?
// con il interaction mode il can ora non lo fa. mentre lo fa per il control mode. perche' diverso?
bool embObjMotionControl::setInteractionModeRaw(int j, yarp::dev::InteractionModeEnum _mode)
{
    eOenum08_t interactionmodecommand = 0;


//    yDebug() << "received setInteractionModeRaw command (SINGLE) for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " mode " << Vocab::decode(_mode);

    if (_mode == VOCAB_IM_COMPLIANT && _torqueControlEnabled == false) {yError()<<"Torque control is disabled. Check your configuration parameters"; return false;}

    if(!interactionModeCommandConvert_yarp2embObj(_mode, interactionmodecommand) )
    {
        yError() << "setInteractionModeRaw: received unknown mode for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " mode " << Vocab::decode(_mode);
    }

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_interactionmode);

    if(! res->addSetMessage(protid, (uint8_t*) &interactionmodecommand) )
    {
        yError() << "setInteractionModeRaw failed for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " mode " << Vocab::decode(_mode);
        return false;
    }

    // marco.accame: use the following if you want to check the value of interactionmode on the remote board
#if 0
    eOenum08_t interactionmodestatus = 0;
    uint16_t size = 0;
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_interactionmodestatus);
    bool ret = askRemoteValue(id32, &interactionmodestatus, size);

    if((false == ret) || (interactionmodecommand != interactionmodestatus))
    {
        yError() << "check of embObjMotionControl::setInteractionModeRaw() failed for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " mode " << Vocab::decode(_mode);
        return false;
    }
#endif

    return true;
}


bool embObjMotionControl::setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
//    std::cout << "setInteractionModeRaw GROUP " << std::endl;

    eOenum08_t interactionmodecommand = 0;

    for(int j=0; j<n_joints; j++)
    {
        if (modes[j] == VOCAB_IM_COMPLIANT && _torqueControlEnabled == false) {yError()<<"Torque control is disabled. Check your configuration parameters"; continue;}

        if(!interactionModeCommandConvert_yarp2embObj(modes[j], interactionmodecommand) )
        {
            yError() << "embObjMotionControl::setInteractionModesRaw(): received unknown interactionMode for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " mode " << Vocab::decode(modes[j]) << " " << modes[j];
            return false;
        }

        eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_interactionmode);
        if(! res->addSetMessage(protid, (uint8_t*) &interactionmodecommand) )
        {
            yError() << "embObjMotionControl::setInteractionModesRaw() failed for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " mode " << Vocab::decode(modes[j]);
            return false;
        }

        // marco.accame: use the following if you want to check the value of interactionmode on the remote board
#if 0
        eOenum08_t interactionmodestatus = 0;
        uint16_t size = 0;
        eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_interactionmodestatus);
        bool ret = askRemoteValue(id32, &interactionmodestatus, size);

        if((false == ret) || (interactionmodecommand != interactionmodestatus))
        {
            if(false == ret)
            {
                yError() << "check of embObjMotionControl::setInteractionModesRaw() failed for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " mode " << Vocab::decode(modes[j]);
                return false;
            }

            int tmp;
            if(interactionModeStatusConvert_embObj2yarp(interactionmodestatus, tmp) )
                yError() << "setInteractionModeRaw failed for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " because of interactionMode mismatching \n\tSet " \
                         << Vocab::decode(modes[j]) << " Got " << Vocab::decode(tmp);
            else
                yError() << "setInteractionModeRaw failed for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " because of interactionMode mismatching \n\tSet " \
                         << Vocab::decode(modes[j]) << " Got an unknown value!";
            return false;
        }
#endif

    }

    return true;
}

bool embObjMotionControl::setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{

    eOenum08_t interactionmodecommand = 0;

    for(int j=0; j<_njoints; j++)
    {
        if ((modes[j] == VOCAB_IM_COMPLIANT) && (_torqueControlEnabled == false))
        {
            yError()<<"Torque control is disabled. Check your configuration parameters";
            continue;
        }

        if(!interactionModeCommandConvert_yarp2embObj(modes[j], interactionmodecommand) )
        {
            yError() << "setInteractionModeRaw: received unknown interactionMode for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " mode " << Vocab::decode(modes[j]);
            return false;
        }

        eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_interactionmode);
        if(! res->addSetMessage(protid, (uint8_t*) &interactionmodecommand) )
        {
            yError() << "setInteractionModeRaw failed for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " mode " << Vocab::decode(modes[j]);
            return false;
        }

        // marco.accame: use the following if you want to check the value of interactionmode on the remote board
#if 0
        eOenum08_t interactionmodestatus = 0;
        uint16_t size = 0;
        eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_interactionmodestatus);
        bool ret = askRemoteValue(id32, &interactionmodestatus, size);

        if((false == ret) || (interactionmodecommand != interactionmodestatus))
        {
            if(false == ret)
            {
                yError() << "check of embObjMotionControl::setInteractionModesRaw() failed for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " mode " << Vocab::decode(modes[j]);
                return false;
            }

            int tmp;
            if(interactionModeStatusConvert_embObj2yarp(interactionmodestatus, tmp) )
                yError() << "setInteractionModeRaw failed for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " because of interactionMode mismatching \n\tSet " \
                         << Vocab::decode(modes[j]) << " Got " << Vocab::decode(tmp);
            else
                yError() << "setInteractionModeRaw failed for BOARD" << res->getName() << "IP" << res->getIPv4string() << " joint " << j << " because of interactionMode mismatching \n\tSet " \
                         << Vocab::decode(modes[j]) << " Got an unknown value!";
            return false;
        }
#endif

    }

    return true;
}


//
// OPENLOOP interface
//
bool embObjMotionControl::setOpenLoopModeRaw()
{
    return DEPRECATED("setOpenLoopModeRaw");
}

bool embObjMotionControl::setRefOutputRaw(int j, double v)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);

    eOmc_setpoint_t setpoint;

    setpoint.type = (eOenum08_t) eomc_setpoint_openloop;
    setpoint.to.openloop.value =  (eOmeas_pwm_t) S_16(v);

    return res->addSetMessage(protid, (uint8_t*) &setpoint);
}

bool embObjMotionControl::setRefOutputsRaw(const double *v)
{
    bool ret = true;
    for(int j=0; j<_njoints; j++)
    {
        ret = ret && setRefOutputRaw(j, v[j]);
    }
    return ret;
}

bool embObjMotionControl::getRefOutputRaw(int j, double *out)
{
    eOprotID32_t protoId = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_target);
    uint16_t size = 0;
    *out = 0;
    eOmc_joint_status_target_t  target = {0};


    if(!askRemoteValue(protoId, (uint8_t *)&target, size))
    {
        yError() << "embObjMotionControl::getRefOutputRaw() could not read openloop reference for  BOARD" << res->getName() << "IP" << res->getIPv4string() << "joint " << j;
        return false;
    }



#if NEW_JSTATUS_STRUCT

    *out = (double) target.trgt_openloop;

    return true;

#else
    *out = (double) jstatus.ofpid.legacy.positionreference;
    return true;
#endif

}

bool embObjMotionControl::getRefOutputsRaw(double *outs)
{
    bool ret = true;
    for(int j=0; j<_njoints; j++)
    {
        ret = ret && getRefOutputRaw(j, &outs[j]);
    }
    return ret;
}

bool embObjMotionControl::getOutputRaw(int j, double *out)
{
    eOprotID32_t protoId = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core);
    uint16_t size = 0;
    eOmc_joint_status_core_t jcore = {0};
    *out = 0;
    if(!res->readBufferedValue(protoId, (uint8_t *)&jcore, &size) )
        return false;

#if NEW_JSTATUS_STRUCT

    if((eomc_controlmode_torque == jcore.modes.controlmodestatus)   ||
      // (eomc_controlmode_openloop == jcore.modes.controlmodestatus) || both IPidControl and IOpenLoop inetrfaces have getOutput function
       (eomc_controlmode_current == jcore.modes.controlmodestatus))
            return true;

    *out = (double) jcore.ofpid.generic.output;

    return true;

#else
    *out = (double) jstatus.ofpid.legacy.output;
    return true;
#endif

}

bool embObjMotionControl::getOutputsRaw(double *outs)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getOutputRaw(j, &outs[j]);
    }
    return ret;
}

bool embObjMotionControl::getNumberOfMotorsRaw(int* num)
{
    *num=_njoints;
    return true;
}

bool embObjMotionControl::getTemperatureRaw(int m, double* val)
{
    uint16_t      size;
    eOmc_motor_status_basic_t     status;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, m, eoprot_tag_mc_motor_status_basic);

    bool ret = res->readBufferedValue(protid, (uint8_t *)&status, &size);
    if(ret)
    {
        *val = (double) status.mot_temperature;
    }
    else
    {
        yError() << "embObjMotionControl::getTemperatureRaw failed for BOARD" << res->getName() << "IP" << res->getIPv4string() << " motor " << m ;
        *val = 0;
    }

    return ret;
}

bool embObjMotionControl::getTemperaturesRaw(double *vals)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getTemperatureRaw(j, &vals[j]);
    }
    return ret;
}

bool embObjMotionControl::getTemperatureLimitRaw(int m, double *temp)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, m, eoprot_tag_mc_motor_config_temperaturelimit);
    uint16_t size;
    eOmeas_temperature_t temperaturelimit = {0};
    *temp = 0;
    if(!askRemoteValue(protid, (uint8_t *)&temperaturelimit, size))
    {
        yError() << "embObjMotionControl::getTemperatureLimitRaw() can't read temperature limits  for BOARD" << res->getName() << "IP" << res->getIPv4string() << " motor " << m;
        return false;
    }

    *temp = (double) temperaturelimit;

    return true;
}

bool embObjMotionControl::setTemperatureLimitRaw(int m, const double temp)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, m, eoprot_tag_mc_motor_config_temperaturelimit);
    eOmeas_temperature_t  temperatureLimit = (eOmeas_pwm_t) S_16(temp);
    return res->addSetMessage(protid, (uint8_t*) &temperatureLimit);

}

bool embObjMotionControl::getPeakCurrentRaw(int m, double *val)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, m, eoprot_tag_mc_motor_config_currentlimits);
    uint16_t size;
    eOmc_current_limits_params_t currentlimits = {0};
    *val = 0;
    if(!askRemoteValue(protid, (uint8_t *)&currentlimits, size))
    {
        yError() << "embObjMotionControl::getPeakCurrentRaw() can't read current limits  for BOARD" << res->getName() << "IP" << res->getIPv4string() << " motor " << m;
        return false;
    }

    *val = (double) currentlimits.peakCurrent ;
    return true;
}

bool embObjMotionControl::setPeakCurrentRaw(int m, const double val)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, m, eoprot_tag_mc_motor_config_currentlimits);
    //get current limit params
    uint16_t size;
    eOmc_current_limits_params_t currentlimits = {0};
    if(!askRemoteValue(protid, (uint8_t *)&currentlimits, size))
    {
        yError() << "embObjMotionControl::setPeakCurrentRaw can't read current limits for BOARD" << res->getName() << "IP" << res->getIPv4string() << " motor " << m ;
        return false;
    }

    //set current overload
    currentlimits.peakCurrent = (eOmeas_current_t) S_16(val);

    //send new values
    bool ret = res->addSetMessage(protid, (uint8_t*) &currentlimits);
    if(!ret)
    {
         yError() << "embObjMotionControl::setPeakCurrentRaw failed sending new value for BOARD" << res->getName() << "IP" << res->getIPv4string() << " motor " << m ;
    }
    return ret;
}

bool embObjMotionControl::getNominalCurrentRaw(int m, double *val)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, m, eoprot_tag_mc_motor_config_currentlimits);
    uint16_t size;
    eOmc_current_limits_params_t currentlimits = {0};
    *val = 0;
    if(!askRemoteValue(protid, (uint8_t *)&currentlimits, size))
    {
        yError() << "embObjMotionControl::getNominalCurrentRaw() can't read current limits  for BOARD" << res->getName() << "IP" << res->getIPv4string() << " motor " << m;
        return false;
    }

    *val = (double) currentlimits.nominalCurrent ;
    return true;
}

bool embObjMotionControl::setNominalCurrentRaw(int m, const double val)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, m, eoprot_tag_mc_motor_config_currentlimits);

    //get current limit params
    uint16_t size;
    eOmc_current_limits_params_t currentlimits = {0};
    if(!askRemoteValue(protid, (uint8_t *)&currentlimits, size))
    {
        yError() << "embObjMotionControl::setNominalCurrentRaw can't read current limits for BOARD" << res->getName() << "IP" << res->getIPv4string() << " motor " << m ;
        return false;
    }

    //set current overload
    currentlimits.nominalCurrent = (eOmeas_current_t) S_16(val);

    //send new values
    bool ret = res->addSetMessage(protid, (uint8_t*) &currentlimits);
    if(!ret)
    {
         yError() << "embObjMotionControl::setNominalCurrentRaw failed sending new value for BOARD" << res->getName() << "IP" << res->getIPv4string() << " motor " << m ;
    }

    return ret;
}

bool embObjMotionControl::getPWMRaw(int j, double* val)
{
    uint16_t      size;
    eOmc_motor_status_basic_t     status;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_status_basic);

    bool ret = res->readBufferedValue(protid, (uint8_t *)&status, &size);
    if(ret)
    {
        *val = (double) status.mot_pwm;
    }
    else
    {
        yError() << "embObjMotionControl::getPWMRaw failed for BOARD" << res->getName() << "IP" << res->getIPv4string() << " motor " << j ;
        *val = 0;
    }

    return ret;
}

bool embObjMotionControl::getPWMLimitRaw(int j, double* val)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config_pwmlimit);
    uint16_t size;
    eOmeas_pwm_t  motorPwmLimit;

    bool ret = askRemoteValue(protid, (uint8_t *)&motorPwmLimit, size);
    if(ret)
    {
        *val = (double) motorPwmLimit;
    }
    else
    {
        yError() << "embObjMotionControl::getPWMLimitRaw failed for BOARD" << res->getName() << "IP" << res->getIPv4string() << " motor " << j ;
        *val = 0;
    }

    return ret;
}

bool embObjMotionControl::setPWMLimitRaw(int j, const double val)
{
    if (val < 0)
    {
        yError() << "embObjMotionControl::setPWMLimitRaw failed because pwmLimit is negative for BOARD" << res->getName() << "IP" << res->getIPv4string() << " motor " << j ;
        return true; //return true because the error ios not due to communication error
    }
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config_pwmlimit);
    eOmeas_pwm_t  motorPwmLimit = (eOmeas_pwm_t) S_16(val);
    return res->addSetMessage(protid, (uint8_t*) &motorPwmLimit);
}

bool embObjMotionControl::getPowerSupplyVoltageRaw(int j, double* val)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_controller, 0, eoprot_tag_mc_controller_status);
    uint16_t size;
    eOmc_controller_status_t  controllerStatus;

    bool ret = askRemoteValue(protid, (uint8_t *)&controllerStatus, size);
    if(ret)
    {
        *val = (double) controllerStatus.supplyVoltage;
    }
    else
    {
        yError() << "embObjMotionControl::getPowerSupplyVoltageRaw failed for BOARD" << res->getName() << "IP" << res->getIPv4string() << " motor " << j ;
        *val = 0;
    }

    return ret;
}

bool embObjMotionControl::askRemoteValue(eOprotID32_t id32, void* value, uint16_t& size)
{
    // marco.accame: this is a private methods, thus it is responsibility of the called to pass value pointer of suitable size. we dont do controls.

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(eoprot_ID2index(id32), id32);

    if(NULL == tt)
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "embObjMotionControl::askRemoteValue() has a NULL eoThreadEntry for BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;
    }

    tt->setPending(1);

    if (!res->addGetMessage(id32))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "embObjMotionControl::askRemoteValue() cannot send ask<> to BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;
        return false;
    }



    // wait here
    if (-1 == tt->synch())
    {
        int threadId;
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "embObjMotionControl::askRemoteValue() timed out the wait of reply from BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;

        if (requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    bool ret = res->readBufferedValue(id32, (uint8_t *)value, &size);

    if(false == ret)
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "embObjMotionControl::askRemoteValue() fails res->readBufferedValue() for BOARD" << res->getName() << "IP" << res->getIPv4string() << "and nv" << nvinfo;
    }
    return ret;
}




bool embObjMotionControl::checkRemoteControlModeStatus(int joint, int target_mode)
{
    bool ret = false;
    eOenum08_t temp = 0;
    uint16_t size = 0;

    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, joint, eoprot_tag_mc_joint_status_core_modes_controlmodestatus);
    const double timeout = 0.250f;  // 250 msec
    const int maxretries = 25;
    const double delaybetweenqueries = 0.010f; // 10 msec

    // now i repeat the query until i am satisfied. how many times? for maximum time timeout seconds and with a gap of delaybetweenqueries

    double timeofstart = yarp::os::Time::now();
    int attempt = 0;

    for( attempt = 0; attempt < maxretries; attempt++)
    {
        ret = askRemoteValue(id32, &temp, size);
        if(ret == false)
        {
            yError ("An error occurred inside embObjMotionControl::checkRemoteControlModeStatus(j=%d, targetmode=%s) for BOARD %s IP %s", joint, yarp::os::Vocab::decode(target_mode).c_str(), res->getName(), res->getIPv4string());
            break;
        }
        int current_mode = controlModeStatusConvert_embObj2yarp(temp);
        if(current_mode == target_mode)
        {
            ret = true;
            break;
        }
        if((current_mode == VOCAB_CM_IDLE) && (target_mode == VOCAB_CM_FORCE_IDLE))
        {
            ret = true;
            break;
        }
        if(current_mode == VOCAB_CM_HW_FAULT)
        {
            if(target_mode != VOCAB_CM_FORCE_IDLE) { yError ("embObjMotionControl::checkRemoteControlModeStatus(%d, %d) is unable to check the control mode of BOARD %s IP %s because it is now in HW_FAULT", joint, target_mode, res->getName(), res->getIPv4string()); }
            ret = true;
            break;
        }

        if((yarp::os::Time::now()-timeofstart) > timeout)
        {
            ret = false;
            yError ("A %f sec timeout occured in embObjMotionControl::checkRemoteControlModeStatus(), BOARD %s IP %s, joint %d, current mode: %s, requested: %s", timeout, res->getName(), res->getIPv4string(), joint, yarp::os::Vocab::decode(current_mode).c_str(), yarp::os::Vocab::decode(target_mode).c_str());
            break;
        }
        if(attempt > 0)
        {   // i print the warning only after at least one retry.
            yWarning ("embObjMotionControl::checkRemoteControlModeStatus() has done %d attempts and will retry again after a %f sec delay. (BOARD %s IP %s, joint %d) -> current mode = %s, requested = %s", attempt+1, delaybetweenqueries, res->getName() , res->getIPv4string(), joint, yarp::os::Vocab::decode(current_mode).c_str(), yarp::os::Vocab::decode(target_mode).c_str());
        }
        yarp::os::Time::delay(delaybetweenqueries);
    }

    if(false == ret)
    {
        yError("failure of embObjMotionControl::checkRemoteControlModeStatus(j=%d, targetmode=%s) for BOARD %s IP %s after %d attempts and %f seconds", joint, yarp::os::Vocab::decode(target_mode).c_str(), res->getName(), res->getIPv4string(), attempt, yarp::os::Time::now()-timeofstart);
    }


    return ret;
}


// eof
