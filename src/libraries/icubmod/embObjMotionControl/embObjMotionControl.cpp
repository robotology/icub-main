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
        yError () << key1.c_str() << " incorrect number of entries in board " << _fId.name << '[' << _fId.boardNumber << ']';
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
    _encoderconversionoffset = allocAndCheck<float>(nj);
    _encoderconversionfactor = allocAndCheck<float>(nj);
    _jointEncoderType = allocAndCheck<string>(nj);
    _rotorEncoderType = allocAndCheck<string>(nj);
    _jointEncoderRes = allocAndCheck<double>(nj);
    _rotorEncoderRes = allocAndCheck<double>(nj);
    _gearbox = allocAndCheck<double>(nj);
    _zeros = allocAndCheck<double>(nj);
    _torqueSensorId= allocAndCheck<int>(nj);
    _torqueSensorChan= allocAndCheck<int>(nj);
    _maxTorque=allocAndCheck<double>(nj);
    _newtonsToSensor=allocAndCheck<double>(nj);
    _hasHallSensor = allocAndCheck<bool>(nj);
    _hasTempSensor = allocAndCheck<bool>(nj);
    _hasRotorEncoder = allocAndCheck<bool>(nj);
    _hasRotorEncoderIndex = allocAndCheck<bool>(nj);
    _rotorIndexOffset = allocAndCheck<int>(nj);
    _motorPoles = allocAndCheck<int>(nj);

    _pids=allocAndCheck<Pid>(nj);
    _tpids=allocAndCheck<Pid>(nj);
    _cpids = allocAndCheck<Pid>(nj);

    _impedance_params=allocAndCheck<ImpedanceParameters>(nj);
    _impedance_limits=allocAndCheck<ImpedanceLimits>(nj);
    _estim_params=allocAndCheck<SpeedEstimationParameters>(nj);

    _limitsMax=allocAndCheck<double>(nj);
    _limitsMin=allocAndCheck<double>(nj);
    _kinematic_mj=allocAndCheck<double>(16);
    _currentLimits=allocAndCheck<double>(nj);
    checking_motiondone=allocAndCheck<bool>(nj);

    _velocityShifts=allocAndCheck<int>(nj);
    _velocityTimeout=allocAndCheck<int>(nj);
    _kbemf=allocAndCheck<double>(nj);
    _ktau=allocAndCheck<double>(nj);
    _filterType=allocAndCheck<int>(nj);
    _last_position_move_time=allocAndCheck<double>(nj);

    // Reserve space for data stored locally. values are initialize to 0
    _ref_positions = allocAndCheck<double>(nj);
    _command_speeds = allocAndCheck<double>(nj);
    _ref_speeds = allocAndCheck<double>(nj);
    _ref_accs = allocAndCheck<double>(nj);
    _ref_torques = allocAndCheck<double>(nj);
    _enabledAmp = allocAndCheck<bool>(nj);
    _enabledPid = allocAndCheck<bool>(nj);
    _calibrated = allocAndCheck<bool>(nj);
    _cacheImpedance = allocAndCheck<eOmc_impedance_t>(nj);

    //debug purpose




    return true;
}

bool embObjMotionControl::dealloc()
{
    checkAndDestroy(_axisMap);
    checkAndDestroy(_angleToEncoder);
    checkAndDestroy(_encodersStamp);
    checkAndDestroy(_encoderconversionoffset);
    checkAndDestroy(_encoderconversionfactor);
    checkAndDestroy(_jointEncoderRes);
    checkAndDestroy(_rotorEncoderRes);
    checkAndDestroy(_jointEncoderType);
    checkAndDestroy(_rotorEncoderType);
    checkAndDestroy(_gearbox);
    checkAndDestroy(_zeros);
    checkAndDestroy(_torqueSensorId);
    checkAndDestroy(_torqueSensorChan);
    checkAndDestroy(_maxTorque);
    checkAndDestroy(_newtonsToSensor);
    checkAndDestroy(_pids);
    checkAndDestroy(_tpids);
    checkAndDestroy(_cpids);
    checkAndDestroy(_impedance_params);
    checkAndDestroy(_impedance_limits);
    checkAndDestroy(_estim_params);
    checkAndDestroy(_limitsMax);
    checkAndDestroy(_limitsMin);
    checkAndDestroy(_kinematic_mj);
    checkAndDestroy(_currentLimits);
    checkAndDestroy(checking_motiondone);
    checkAndDestroy(_velocityShifts);
    checkAndDestroy(_velocityTimeout);
    checkAndDestroy(_kbemf);
    checkAndDestroy(_ktau);
    checkAndDestroy(_filterType);
    checkAndDestroy(_ref_positions);
    checkAndDestroy(_command_speeds);
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
    checkAndDestroy(_rotorIndexOffset);
    checkAndDestroy(_motorPoles);

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
    _mutex(1),
    SAFETY_THRESHOLD(2.0)
{
    _gearbox       = 0;
    opened        = 0;
    _pids         = NULL;
    _tpids        = NULL;
    _cpids        = NULL;
    res           = NULL;
    requestQueue  = NULL;
    _njoints      = 0;
    _axisMap      = NULL;
    _zeros        = NULL;
    _encodersStamp = NULL;
    _encoderconversionfactor = NULL;
    _encoderconversionoffset = NULL;
    _angleToEncoder = NULL;
    _hasHallSensor = NULL;
    _hasTempSensor = NULL;
    _hasRotorEncoder = NULL;
    _hasRotorEncoderIndex = NULL;
    _rotorIndexOffset = NULL;
    _motorPoles       = NULL;
    _cacheImpedance   = NULL;
    _impedance_params = NULL;
    _impedance_limits = NULL;
    _estim_params     = NULL;

    _limitsMin        = NULL;
    _limitsMax        = NULL;
    _currentLimits    = NULL;
    _velocityShifts   = NULL;
    _velocityTimeout  = NULL;
    _torqueSensorId   = NULL;
    _torqueSensorChan = NULL;
    _maxTorque        = NULL;
    _newtonsToSensor  = NULL;
    _jointEncoderRes  = NULL;
    _jointEncoderType = NULL;
    _rotorEncoderRes  = NULL;
    _rotorEncoderType = NULL;
    _ref_accs         = NULL;
    _command_speeds   = NULL;
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
#if !defined(EMBOBJMC_DONT_USE_MAIS)
    numberofmaisboards = 0;
#endif // defined(EMBOBJMC_DONT_USE_MAIS)

    ConstString tmp = NetworkBase::getEnvironment("ETH_VERBOSEWHENOK");
    if (tmp != "")
    {
        verbosewhenok = (bool)NetType::toInt(tmp);
    }
    else
    {
        verbosewhenok = false;
    }

}

embObjMotionControl::~embObjMotionControl()
{
    yTrace() << "embObjMotionControl::~embObjMotionControl()";
    dealloc();
}

bool embObjMotionControl::initialised()
{
    return opened;
}

bool embObjMotionControl::verifyMotionControlProtocol(Bottle groupProtocol )
{
    if(false == res->isEPmanaged(eoprot_endpoint_motioncontrol))
    {
        yError() << "embObjMotionControl::open() detected that EMS "<< _fId.boardNumber << " does not support motion control";
        return false;
    }


    if(false == res->verifyBoard(groupProtocol))
    {
        yError() << "embObjMotionControl::open() fails in function verifyBoard() for board " << _fId.boardNumber << ": CANNOT PROCEED ANY FURTHER";
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjMotionControl::open() has verified that BOARD "<< _fId.boardNumber << " is communicating correctly";
        }
    }

    if(false == res->verifyEPprotocol(groupProtocol, eoprot_endpoint_motioncontrol))
    {
        yError() << "embObjMotionControl::open() fails in function verifyProtocol() for BOARD "<< _fId.boardNumber << ": probably it does not have the same eoprot_endpoint_management and/or eoprot_endpoint_motioncontrol protocol version: DO A FW UPGRADE";
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjMotionControl::open() has succesfully verified that BOARD "<< _fId.boardNumber << " has same protocol version for motioncontrol as robotInterface";
        }
    }


    if(false == res->configureENDPOINT(groupProtocol, eoprot_endpoint_motioncontrol))
    {
        yError() << "embObjMotionControl::open() fails in function configureENDPOINT() for BOARD "<< _fId.boardNumber << " and endpoint eoprot_endpoint_motioncontrol: VERIFY XML files";
        return false;
    }


    if(false == res->verifyENTITYnumber(groupProtocol, eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, _njoints))
    {
        yError() << "embObjMotionControl::open() fails in function verifyENTITYnumber() for BOARD "<< _fId.boardNumber << " and entity eoprot_entity_mc_joint: VERIFY their number in board, and in XML files";
        return false;
    }
//    else
//    {
//        yDebug() << "embObjMotionControl::open() has succesfully verified that board "<< _fId.boardNumber << " has multiplicity" << _njoints << "for entity eoprot_entity_mc_joint";
//    }


    if(false == res->verifyENTITYnumber(groupProtocol, eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, _njoints))
    {
        yError() << "embObjMotionControl::open() fails in function verifyENTITYnumber() for board "<< _fId.boardNumber << " and entity eoprot_entity_mc_motor: VERIFY their number in board, and in XML files";
        return false;
    }
//    else
//    {
//        yDebug() << "embObjMotionControl::open() has succesfully verified that board "<< _fId.boardNumber << " has multiplicity" << _njoints << "for entity eoprot_entity_mc_motor";
//    }

    return true;
}

#if !defined(EMBOBJMC_DONT_USE_MAIS)

bool embObjMotionControl::verifyMaisProtocol(Bottle groupProtocol)
{
    numberofmaisboards = eoprot_entity_numberof_get(featIdBoardNum2nvBoardNum(_fId.boardNumber), eoprot_endpoint_analogsensors, eoprot_entity_as_mais);

    if(0 != numberofmaisboards)
    {
        // must verify protocol and then the entity number

        if(false == res->verifyEPprotocol(groupProtocol, eoprot_endpoint_analogsensors))
        {
            yError() << "embObjMotionControl::open() fails in function verifyProtocol() of eoprot_endpoint_analogsensors for BOARD "<< _fId.boardNumber << ": probably it does not have the same eoprot_endpoint_analogsensors protocol version: DO A FW UPGRADE";
            return false;
        }
        else
        {
            if(verbosewhenok)
            {
                yDebug() << "embObjMotionControl::open() detected that there is a mais, thus called verifyProtocol() of eoprot_endpoint_analogsensors for board "<< _fId.boardNumber;
            }
        }



        if(false == res->verifyENTITYnumber(groupProtocol, eoprot_endpoint_analogsensors, eoprot_entity_as_mais))
        {
            yError() << "embObjMotionControl::open() fails in function verifyENTITYnumber() for board "<< _fId.boardNumber << " and entity eoprot_entity_as_mais: VERIFY their number in board, and in XML files";
            return false;
        }


        if(false == configure_mais(groupProtocol))
        {
            yError() << "embObjMotionControl::open() had an error while configuring mais for board " << _fId.boardNumber;
            return false;
        }
        else
        {
            if(verbosewhenok)
            {
                yDebug() << "embObjMotionControl::open() succesfully configured the MAIS attached to board" << _fId.boardNumber;
            }
        }
    }
    return true;
}

#endif // !defined(EMBOBJMC_DONT_USE_MAIS)


bool embObjMotionControl::open(yarp::os::Searchable &config)
{
    std::string str;
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


    Bottle          groupEth;
    ACE_UINT16      port;

    Bottle groupTransceiver = Bottle(config.findGroup("TRANSCEIVER"));
    if(groupTransceiver.isNull())
    {
        yError() << "embObjMotionControl::open() cannot find TRANSCEIVER group in xml config files";
        return false;
    }

    Bottle groupProtocol = Bottle(config.findGroup("PROTOCOL"));
    if(groupProtocol.isNull())
    {
        yError() << "embObjMotionControl::open() cannot find PROTOCOL group in xml config files";
        return false;
    }

    // Get both PC104 and EMS ip addresses and port from config file
    groupEth  = Bottle(config.findGroup("ETH"));
    if(groupEth.isNull())
    {
        yError() << "embObjMotionControl::open() cannot find ETH group in config files";
        return false;
    }
    Value *PC104IpAddress_p;
    if(!groupEth.check("PC104IpAddress", PC104IpAddress_p))
    {
        yError() << "missing PC104IpAddress";
        return false;
    }
    Bottle parameter1( groupEth.find("PC104IpAddress").asString() );
    port      = groupEth.find("CmdPort").asInt();              // .get(1).asInt();
    strcpy(_fId.pc104IPaddr.string, parameter1.toString().c_str());
    _fId.pc104IPaddr.port = port;

    Bottle parameter2( groupEth.find("IpAddress").asString() );    // .findGroup("IpAddress");
    strcpy(_fId.boardIPaddr.string, parameter2.toString().c_str());
    _fId.boardIPaddr.port = port;

    sscanf(_fId.boardIPaddr.string,"\"%d.%d.%d.%d", &_fId.boardIPaddr.ip1, &_fId.boardIPaddr.ip2, &_fId.boardIPaddr.ip3, &_fId.boardIPaddr.ip4);
    sscanf(_fId.pc104IPaddr.string,"\"%d.%d.%d.%d", &_fId.pc104IPaddr.ip1, &_fId.pc104IPaddr.ip2, &_fId.pc104IPaddr.ip3, &_fId.pc104IPaddr.ip4);

    snprintf(_fId.boardIPaddr.string,sizeof(_fId.boardIPaddr.string), "%u.%u.%u.%u:%u", _fId.boardIPaddr.ip1, _fId.boardIPaddr.ip2, _fId.boardIPaddr.ip3, _fId.boardIPaddr.ip4, _fId.boardIPaddr.port);
    snprintf(_fId.pc104IPaddr.string, sizeof(_fId.pc104IPaddr.string), "%u.%u.%u.%u:%u", _fId.pc104IPaddr.ip1, _fId.pc104IPaddr.ip2, _fId.pc104IPaddr.ip3, _fId.pc104IPaddr.ip4, _fId.pc104IPaddr.port);

    // Check input parameters
    snprintf(info, sizeof(info), "embObjMotionControl - referred to EMS: %s", _fId.boardIPaddr.string);

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


    // Saving User Friendly Id
    memset(_fId.name, 0x00, sizeof(_fId.name));
    snprintf(_fId.name, sizeof(_fId.name), "%s", info);

    _fId.boardNumber  = FEAT_boardnumber_dummy;
    Value val = config.findGroup("ETH").check("Ems",Value(1), "Board number");
    if(val.isInt())
    {
        _fId.boardNumber = val.asInt();
    }
    else
    {
        yError () << "embObjMotionControl: EMS Board number identifier not found";
        return false;
    }

    _fId.endpoint = eoprot_endpoint_motioncontrol;
    //
    //  Read Configuration params from file
    //
    _njoints = config.findGroup("GENERAL").check("Joints",Value(1),   "Number of degrees of freedom").asInt();

    if(!alloc(_njoints))
    {
        yError() << "Malloc failed";
        return false;
    }
    if(!fromConfig(config))
    {
        yError() << "Missing parameters in config file";
        return false;
    }

    //  INIT ALL INTERFACES
    yarp::sig::Vector tmpZeros; tmpZeros.resize (_njoints, 0.0);
    yarp::sig::Vector tmpOnes;  tmpOnes.resize  (_njoints, 1.0);
    
    ImplementControlCalibration2<embObjMotionControl, IControlCalibration2>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementAmplifierControl<embObjMotionControl, IAmplifierControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementEncodersTimed::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementMotorEncoders::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementPositionControl2::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementPidControl<embObjMotionControl, IPidControl>:: initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementControlMode2::initialize(_njoints, _axisMap);
    ImplementVelocityControl<embObjMotionControl, IVelocityControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementVelocityControl2::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
#ifdef IMPLEMENT_DEBUG_INTERFACE
    ImplementDebugInterface::initialize(_njoints, _axisMap, _angleToEncoder, _zeros, _rotorEncoderRes);
#endif
    ImplementControlLimits2::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementImpedanceControl::initialize(_njoints, _axisMap, _angleToEncoder, _zeros, _newtonsToSensor);
    ImplementTorqueControl::initialize(_njoints, _axisMap, _angleToEncoder, _zeros, _newtonsToSensor);
    ImplementPositionDirect::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementOpenLoopControl::initialize(_njoints, _axisMap);
    ImplementInteractionMode::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementMotor::initialize(_njoints, _axisMap);
    ImplementRemoteVariables::initialize(_njoints, _axisMap);
    
    /*
    *  Once I'm sure every input data required is present and correct, instantiate the EMS, transceiver etc...
    */

    ethManager = TheEthManager::instance();
    if(NULL == ethManager)
    {
        yFatal() << "embObjMotionControl::open() fails to instantiate ethManager";
        return false;
    }

    _fId.interface  = this;
    _fId.type = ethFeatType_MotionControl;

    res = ethManager->requestResource(config, groupTransceiver, groupProtocol, _fId);
    if(NULL == res)
    {
        yError() << "embObjMotionControl::open() fails because could not instantiate the ethResource for board" << _fId.boardNumber << " ... unable to continue";
        return false;
    }


    if(!verifyMotionControlProtocol(groupProtocol) )
    {
        cleanup();
        return false;
    }


    NVnumber = res->getNVnumber(eoprot_endpoint_motioncontrol);
    requestQueue = new eoRequestsQueue(NVnumber);

    if(!init() )
    {
        yError() << "embObjMotionControl::open() has an error in call of embObjMotionControl::init() for board" << _fId.boardNumber;
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjMotionControl::init() has succesfully initted board "<< _fId.boardNumber;
        }
    }


#if !defined(EMBOBJMC_DONT_USE_MAIS)

    // now if this device has a mais ... we configure it

    if(!verifyMaisProtocol(groupProtocol))
    {
        cleanup();
        return false;
    }

#endif // !defined(EMBOBJMC_DONT_USE_MAIS)

    if(false == res->goToRun())
    {
        yError() << "embObjMotionControl::open() fails to start control loop of board" << _fId.boardNumber << ": cannot continue";
        cleanup();
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjMotionControl::open() correctly activated control loop of BOARD" << _fId.boardNumber;
        }
    }


    opened = true;
    return true;
}

bool embObjMotionControl::isEpManagedByBoard()
{
    if(eobool_true == eoprot_endpoint_configured_is(res->get_protBRDnumber(), eoprot_endpoint_motioncontrol))
    {
        return true;
    }
    
    return false;
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
    if (!extractGroup(pidsGroup, xtmp, "maxPwm", "Pid maxPwm parameter", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].max_output = xtmp.get(j+1).asDouble();
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

    //optional PWM limit
    if(_pwmIsLimited)
    {   // check for value in the file
        if (!extractGroup(pidsGroup, xtmp, "limPwm", "Limited PWD", _njoints))
        {
            yError() << "The PID parameter limPwm was requested but was not correctly set in the configuration file, please fill it.";
            return false;
        }

        fprintf(stderr,  "embObjMotionControl using LIMITED PWM!! \n");
        for (j=0; j<_njoints; j++) myPid[j].max_output = xtmp.get(j+1).asDouble();
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
    if (!extractGroup(pidsGroup, xtmp, "maxPwm", "Pid maxPwm parameter", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].max_output = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "shift", "Pid shift parameter", _njoints))     return false; for (j=0; j<_njoints; j++) myPid[j].scale = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "ko", "Pid ko parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].offset = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "stictionUp", "Pid stictionUp", _njoints))     return false; for (j=0; j<_njoints; j++) myPid[j].stiction_up_val = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "stictionDwn", "Pid stictionDwn", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].stiction_down_val = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "kff",   "Pid kff parameter", _njoints))       return false; for (j=0; j<_njoints; j++) myPid[j].kff  = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "kbemf", "kbemf parameter", _njoints))         return false; for (j=0; j<_njoints; j++) kbemf[j]      = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp, "ktau", "ktau parameter", _njoints))           return false; for (j=0; j<_njoints; j++) ktau[j]       = xtmp.get(j+1).asDouble(); 
    if (!extractGroup(pidsGroup, xtmp, "filterType", "filterType param", _njoints))   return false; for (j=0; j<_njoints; j++) filterType[j] = xtmp.get(j+1).asDouble();
 
    //conversion from metric to machine units (if applicable)
    for (j=0; j<_njoints; j++)
    {
        myPid[j].kp = myPid[j].kp / _torqueControlHelper->getNewtonsToSensor(j);  //[PWM/Nm]
        myPid[j].ki = myPid[j].ki / _torqueControlHelper->getNewtonsToSensor(j);  //[PWM/Nm]
        myPid[j].kd = myPid[j].kd / _torqueControlHelper->getNewtonsToSensor(j);  //[PWM/Nm]
        myPid[j].stiction_up_val   = myPid[j].stiction_up_val   * _torqueControlHelper->getNewtonsToSensor(j); //[Nm]
        myPid[j].stiction_down_val = myPid[j].stiction_down_val * _torqueControlHelper->getNewtonsToSensor(j); //[Nm]
    }

    //optional PWM limit
    if(_pwmIsLimited)
    {   // check for value in the file
        if (!extractGroup(pidsGroup, xtmp, "limPwm", "Limited PWD", _njoints))
        {
            yError() << "The PID parameter limPwm was requested but was not correctly set in the configuration file, please fill it.";
            return false;
        }

        fprintf(stderr,  "embObjMotionControl using LIMITED PWM!! \n");
        for (j=0; j<_njoints; j++) myPid[j].max_output = xtmp.get(j+1).asDouble();
    }

    return true;
}

bool embObjMotionControl::fromConfig(yarp::os::Searchable &config)
{
    Bottle xtmp;
    int i,j;
    Bottle general = config.findGroup("GENERAL");

    // leggere i valori da file
    if (!extractGroup(general, xtmp, "AxisMap", "a list of reordered indices for the axes", _njoints))
        return false;

    for (i = 1; i < xtmp.size(); i++)
        _axisMap[i-1] = xtmp.get(i).asInt();

    double tmp_A2E;
    // Encoder scales
    if (!extractGroup(general, xtmp, "Encoder", "a list of scales for the encoders", _njoints))
    {
        return false;
    }
    else
        for (i = 1; i < xtmp.size(); i++)
        {
            tmp_A2E = xtmp.get(i).asDouble();

            if(useRawEncoderData)   // do not use any configuration, this is intended for doing the very first calibration
            {
                tmp_A2E > 0 ? _encoderconversionfactor[i-1] = 1 : _encoderconversionfactor[i-1] = -1;
                _angleToEncoder[i-1] = 1;
            }
            else
            {
                _angleToEncoder[i-1] =  (1<<16) / 360.0;        // conversion factor from degrees to iCubDegrees
                _encoderconversionfactor[i-1] = float((tmp_A2E  ) / _angleToEncoder[i-1]);
            }

            _encoderconversionoffset[i-1] = 0;
        }

    // Joint encoder type
    if (!extractGroup(general, xtmp, "JointEncoderType", "JointEncoderType", _njoints))
    {
        return false;
    }
    else
    {
        int test = xtmp.size();
        for (i = 1; i < xtmp.size(); i++)
            _jointEncoderType[i - 1] = xtmp.get(i).asString();
    }

    // Motor capabilities
    if (!extractGroup(general, xtmp, "HasHallSensor", "HasHallSensor 0/1 ", _njoints))
    {
        return false;
    }
    else
    {
        int test = xtmp.size();
        for (i = 1; i < xtmp.size(); i++)
            _hasHallSensor[i - 1] = xtmp.get(i).asInt();
    }
    if (!extractGroup(general, xtmp, "HasTempSensor", "HasTempSensor 0/1 ", _njoints))
    {
        return false;
    }
    else
    {
        int test = xtmp.size();
        for (i = 1; i < xtmp.size(); i++)
            _hasTempSensor[i - 1] = xtmp.get(i).asInt();
    }
    if (!extractGroup(general, xtmp, "HasRotorEncoder", "HasRotorEncoder 0/1 ", _njoints))
    {
        return false;
    }
    else
    {
        int test = xtmp.size();
        for (i = 1; i < xtmp.size(); i++)
            _hasRotorEncoder[i - 1] = xtmp.get(i).asInt();
    }
    if (!extractGroup(general, xtmp, "HasRotorEncoderIndex", "HasRotorEncoderIndex 0/1 ", _njoints))
    {
        return false;
    }
    else
    {
        int test = xtmp.size();
        for (i = 1; i < xtmp.size(); i++)
            _hasRotorEncoderIndex[i - 1] = xtmp.get(i).asInt();
    }

    // Rotor encoder res
    if (!extractGroup(general, xtmp, "RotorEncoderRes", "a list of scales for the rotor encoders", _njoints))
    {
        return false;
    }
    else
    {
        int test = xtmp.size();
        for (i = 1; i < xtmp.size(); i++)
            _rotorEncoderRes[i - 1] = xtmp.get(i).asDouble();
    }

    // joint encoder res
    if (!extractGroup(general, xtmp, "JointEncoderRes", "a list of scales for the joint encoders", _njoints))
    {
        return false;
    }
    else
    {
        int test = xtmp.size();
        for (i = 1; i < xtmp.size(); i++)
            _jointEncoderRes[i - 1] = xtmp.get(i).asDouble();
    }

    // Number of motor poles
    if (!extractGroup(general, xtmp, "MotorPoles", "MotorPoles", _njoints))
    {
        return false;
    }
    else
    {
        int test = xtmp.size();
        for (i = 1; i < xtmp.size(); i++)
            _motorPoles[i - 1] = xtmp.get(i).asInt();
    }

    // Rotor encoder index
    if (!extractGroup(general, xtmp, "RotorIndexOffset", "RotorIndexOffset", _njoints))
    {
        return false;
    }
    else
    {
        int test = xtmp.size();
        for (i = 1; i < xtmp.size(); i++)
            _rotorIndexOffset[i - 1] = xtmp.get(i).asInt();
    }

    // Rotor encoder type
    if (!extractGroup(general, xtmp, "RotorEncoderType", "RotorEncoderType", _njoints))
    {
        return false;
    }
    else
    {
        int test = xtmp.size();
        for (i = 1; i < xtmp.size(); i++)
            _rotorEncoderType[i - 1] = xtmp.get(i).asString();
    }

    // Gearbox
    if (!extractGroup(general, xtmp, "Gearbox", "The gearbox reduction ratio", _njoints))
    {
        return false;
    }
    else
    {
        int test = xtmp.size();
        for (i = 1; i < xtmp.size(); i++)
        {
            _gearbox[i-1] = xtmp.get(i).asDouble();
            if (_gearbox[i-1]==0) {yError() << "Using a gearbox value = 0 may cause problems! Check your configuration files"; return false;}
        }
    }
    
    // Zero Values
    if (!extractGroup(general, xtmp, "Zeros","a list of offsets for the zero point", _njoints))
        return false;
    else
        for (i = 1; i < xtmp.size(); i++)
            if(useRawEncoderData)   // do not use any configuration, this is intended for doing the very first calibration
                _zeros[i-1] = 0;
            else
                _zeros[i-1] = xtmp.get(i).asDouble();


    // Torque sensors stuff
    if (!extractGroup(general, xtmp, "TorqueId","a list of associated joint torque sensor ids", _njoints))
    {
        fprintf(stderr, "Using default value = 0 (disabled)\n");
        for(i=1; i<_njoints+1; i++)
            _torqueSensorId[i-1] = 0;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++) _torqueSensorId[i-1] = xtmp.get(i).asInt();
    }


    if (!extractGroup(general, xtmp, "TorqueChan","a list of associated joint torque sensor channels", _njoints))
    {
        yWarning() <<  "embObjMotionControl::fromConfig() detected that TorqueChan is not present: using default value = 0 (disabled)";
        for(i=1; i<_njoints+1; i++)
            _torqueSensorChan[i-1] = 0;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++) _torqueSensorChan[i-1] = xtmp.get(i).asInt();
    }

    if (!extractGroup(general, xtmp, "TorqueMax","full scale value for a joint torque sensor", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
        {
            _maxTorque[i-1] = xtmp.get(i).asInt();
            _newtonsToSensor[i-1] = double(0x8000)/double(_maxTorque[i-1]);
        }
    }


    ////// POSITION PIDS
    {
        Bottle posPidsGroup;
        posPidsGroup=config.findGroup("POSITION_CONTROL", "Position Pid parameters new format");
        if (posPidsGroup.isNull()==false)
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
        else
        {
            yError() <<"embObjMotionControl::fromConfig(): Error: no POS_PIDS group found in config file, returning";
            return false;
        }
    }


    ////// TORQUE PIDS
    {
        Bottle trqPidsGroup;
        trqPidsGroup=config.findGroup("TORQUE_CONTROL", "Torque control parameters new format");
        if (trqPidsGroup.isNull()==false)
        {
           Value &controlUnits=trqPidsGroup.find("controlUnits");
           if  (controlUnits.isNull() == false && controlUnits.isString() == true)
           {
                if      (controlUnits.toString()==string("metric_units"))  {yDebug("TORQUE_CONTROL: using metric_units"); _torqueControlUnits=T_METRIC_UNITS;}
                else if (controlUnits.toString()==string("machine_units")) {yDebug("TORQUE_CONTROL: using metric_units"); _torqueControlUnits=T_MACHINE_UNITS;}
                else    {yError() << "embObjMotionControl::fromConfig(): TORQUE_CONTROL section: invalid controlUnits value";
                         return false;}
                if      (_torqueControlUnits==T_MACHINE_UNITS) {yarp::sig::Vector tmpOnes; tmpOnes.resize(_njoints,1.0); _torqueControlHelper = new torqueControlHelper(_njoints, tmpOnes.data(), tmpOnes.data());}
                else if (_torqueControlUnits==T_METRIC_UNITS)  {_torqueControlHelper = new torqueControlHelper(_njoints, _angleToEncoder, _newtonsToSensor);}
           }
           else
           {
                yError() << "embObjMotionControl::fromConfig(): TORQUE_CONTROL section: missing controlUnits parameter. Assuming machine_units. Please fix your configuration file.";
                _torqueControlUnits=T_MACHINE_UNITS;
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
        }
        else
        {
            yError() <<"embObjMotionControl::fromConfig(): Error: no TORQUE_CONTROL group found in config file";
            _torqueControlEnabled = false;
        }
    }

    ////// IMPEDANCE PARAMETERS
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

    /////// JOINTS_COUPLING
    if (_njoints<=4)
    {
        Bottle &coupling=config.findGroup("JOINTS_COUPLING");
        if (coupling.isNull())
        {
            yWarning() << "embObjMotionControl::fromConfig() detected that Group JOINTS_COUPLING is not found in configuration file";
            //return false;
        }
        // current limit
        if (!extractGroup(coupling, xtmp, "kinematic_mj","the kinematic matrix 4x4 which tranforms from joint space to motor space", 16))
        {
            for(i=1; i<xtmp.size(); i++) _kinematic_mj[i-1]=0.0;
        }
        else
            for(i=1; i<xtmp.size(); i++) _kinematic_mj[i-1]=xtmp.get(i).asDouble();
    }
    else
    {
        //we are skipping JOINTS_COUPLING for EMS boards which control MC4 boards (for now)
    }
    
    /////// LIMITS
    Bottle &limits=config.findGroup("LIMITS");
    if (limits.isNull())
    {
        yWarning() << "embObjMotionControl::fromConfig() detected that Group LIMITS is not found in configuration file";
        return false;
    }
    // current limit
    if (!extractGroup(limits, xtmp, "Currents","a list of current limits", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) _currentLimits[i-1]=xtmp.get(i).asDouble();

    // max limit
    if (!extractGroup(limits, xtmp, "Max","a list of maximum angles (in degrees)", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) _limitsMax[i-1]=xtmp.get(i).asDouble();

    // min limit
    if (!extractGroup(limits, xtmp, "Min","a list of minimum angles (in degrees)", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) _limitsMin[i-1]=xtmp.get(i).asDouble();

    /////// [VELOCITY]
    Bottle &velocityGroup=config.findGroup("VELOCITY");
    if (!velocityGroup.isNull())
    {
        /////// Shifts
        if (!extractGroup(velocityGroup, xtmp, "Shifts", "a list of shifts to be used in the vmo control", _njoints))
        {
            fprintf(stderr, "Using default Shifts=4\n");
            for(i=1; i<_njoints+1; i++)
                _velocityShifts[i-1] = 4;   //Default value
        }
        else
        {
            for(i=1; i<xtmp.size(); i++)
                _velocityShifts[i-1]=xtmp.get(i).asInt();
        }

        /////// Timeout
        xtmp.clear();
        if (!extractGroup(velocityGroup, xtmp, "Timeout", "a list of timeout to be used in the vmo control", _njoints))
        {
            fprintf(stderr, "Using default Timeout=100, i.e 0.1s\n");
            for(i=1; i<_njoints+1; i++)
                _velocityTimeout[i-1] = 100;   //Default value
        }
        else
        {
            for(i=1; i<xtmp.size(); i++)
                _velocityTimeout[i-1]=xtmp.get(i).asInt();
        }

        /////// Joint Speed Estimation
        xtmp.clear();
        if (!extractGroup(velocityGroup, xtmp, "JNT_speed_estimation", "a list of shift factors used by the firmware joint speed estimator", _njoints))
        {
            fprintf(stderr, "Using default value=5\n");
            for(i=1; i<_njoints+1; i++)
                _estim_params[i-1].jnt_Vel_estimator_shift = 0;   //Default value
        }
        else
        {
            for(i=1; i<xtmp.size(); i++)
                _estim_params[i-1].jnt_Vel_estimator_shift = xtmp.get(i).asInt();
        }

        /////// Motor Speed Estimation
        xtmp.clear();
        if (!extractGroup(velocityGroup, xtmp, "MOT_speed_estimation", "a list of shift factors used by the firmware motor speed estimator", _njoints))
        {
            fprintf(stderr, "Using default value=5\n");
            for(i=1; i<_njoints+1; i++)
                _estim_params[i-1].mot_Vel_estimator_shift = 0;   //Default value
        }
        else
        {
            for(i=1; i<xtmp.size(); i++)
                _estim_params[i-1].mot_Vel_estimator_shift = xtmp.get(i).asInt();
        }

        /////// Joint Acceleration Estimation
        xtmp.clear();
        if (!extractGroup(velocityGroup, xtmp, "JNT_accel_estimation", "a list of shift factors used by the firmware joint speed estimator", _njoints))
        {
            fprintf(stderr, "Using default value=5\n");
            for(i=1; i<_njoints+1; i++)
                _estim_params[i-1].jnt_Acc_estimator_shift = 0;   //Default value
        }
        else
        {
            for(i=1; i<xtmp.size(); i++)
                _estim_params[i-1].jnt_Acc_estimator_shift = xtmp.get(i).asInt();
        }

        /////// Motor Acceleration Estimation
        xtmp.clear();
        if (!extractGroup(velocityGroup, xtmp, "MOT_accel_estimation", "a list of shift factors used by the firmware motor speed estimator", _njoints))
        {
            fprintf(stderr, "Using default value=5\n");
            for(i=1; i<_njoints+1; i++)
                _estim_params[i-1].mot_Acc_estimator_shift = 5;   //Default value
        }
        else
        {
            for(i=1; i<xtmp.size(); i++)
                _estim_params[i-1].mot_Acc_estimator_shift = xtmp.get(i).asInt();
        }

    }
    else
    {
        fprintf(stderr, "A suitable value for [VELOCITY] Shifts was not found. Using default Shifts=4\n");
        for(i=1; i<_njoints+1; i++)
            _velocityShifts[i-1] = 0;   //Default value // not used now!! In the future this value may (should?) be read from config file and sertnto the EMS

        fprintf(stderr, "A suitable value for [VELOCITY] Timeout was not found. Using default Timeout=1000, i.e 1s.\n");
        for(i=1; i<_njoints+1; i++)
            _velocityTimeout[i-1] = 1000;   //Default value

        fprintf(stderr, "A suitable value for [VELOCITY] speed estimation was not found. Using default shift factor=5.\n");
        for(i=1; i<_njoints+1; i++)
        {
            _estim_params[i-1].jnt_Vel_estimator_shift = 0;   //Default value
            _estim_params[i-1].jnt_Acc_estimator_shift = 0;
            _estim_params[i-1].mot_Vel_estimator_shift = 0;
            _estim_params[i-1].mot_Acc_estimator_shift = 0;
        }
    }
    return true;
}


bool embObjMotionControl::init()
{
    eOprotID32_t protid = 0;
    bool result = true;

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
            yError() << "embObjMotionControl::init() had an error while setting eomc_controlmode_cmd_idle in BOARD" << res->get_protBRDnumber()+1;
            // return(false); i dont return false. because even if a failure, that is not a severe error.
            // MOREOVER: to verify we must read the status of the joint and NOT the command ... THINK OF IT
        }
        //Time::delay(0.001);
    }

    Time::delay(0.010);  // 10 ms (m.a.a-delay: before it was 0.01)



    ////////////////////////////////////////////////
    // configure the regular rops
    ////////////////////////////////////////////////

    vector<eOprotID32_t> id32v(0);
    for(int n=0; n<_njoints; n++)
    {
        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, n, eoprot_tag_mc_joint_status);
        id32v.push_back(protid);
        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, n, eoprot_tag_mc_motor_status_basic);
        id32v.push_back(protid);
    }

    if(false == res->addRegulars(id32v, true))
    {
        yError() << "embObjMotionControl::init() fails to add its variables to regulars in BOARD" << res->get_protBRDnumber()+1 << ": cannot proceed any further";
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjMotionControl::init() added" << id32v.size() << "regular rops to BOARD" << res->get_protBRDnumber()+1;
            char nvinfo[128];
            for(int r=0; r<id32v.size(); r++)
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

        eOmc_joint_config_t     jconfig;
        memset(&jconfig, 0, sizeof(eOmc_joint_config_t));
        copyPid_iCub2eo(&_pids[logico],  &jconfig.pidposition);
        copyPid_iCub2eo(&_pids[logico],  &jconfig.pidvelocity);
        copyPid_iCub2eo(&_tpids[logico], &jconfig.pidtorque);

        jconfig.impedance.damping   = (eOmeas_damping_t)   U_32(_impedance_params[logico].damping * 1000);
        jconfig.impedance.stiffness = (eOmeas_stiffness_t) U_32(_impedance_params[logico].stiffness * 1000);
        jconfig.impedance.offset    = 0; //impedance_params[j];

        _cacheImpedance[logico].stiffness = jconfig.impedance.stiffness;
        _cacheImpedance[logico].damping   = jconfig.impedance.damping;
        _cacheImpedance[logico].offset    = 0;

        jconfig.limitsofjoint.max = (eOmeas_position_t) S_32(convertA2I(_limitsMax[logico], _zeros[logico], _angleToEncoder[logico]));
        jconfig.limitsofjoint.min = (eOmeas_position_t) S_32(convertA2I(_limitsMin[logico], _zeros[logico], _angleToEncoder[logico]));
        jconfig.velocitysetpointtimeout = (eOmeas_time_t) U_16(_velocityTimeout[logico]);
        jconfig.motionmonitormode = eomc_motionmonitormode_dontmonitor;

        jconfig.encoderconversionfactor = eo_common_float_to_Q17_14(_encoderconversionfactor[logico]);
        jconfig.encoderconversionoffset = eo_common_float_to_Q17_14(_encoderconversionoffset[logico]);

        jconfig.motor_params.bemf_value = 0;
        jconfig.motor_params.bemf_scale = 0;
        jconfig.motor_params.ktau_value = 0;
        jconfig.motor_params.ktau_scale = 0;
        
        jconfig.tcfiltertype=_filterType[logico];


        if(false == res->setRemoteValueUntilVerified(protid, &jconfig, sizeof(jconfig), 10, 0.010, 0.050, 2))
        {
            yError() << "FATAL: embObjMotionControl::init() had an error while calling setRemoteValueUntilVerified() for joint config fisico #" << fisico << "in BOARD" << res->get_protBRDnumber()+1;
            return false;
        }
        else
        {
            if(verbosewhenok)
            {
                yDebug() << "embObjMotionControl::init() correctly configured joint config fisico #" << fisico << "in BOARD" << res->get_protBRDnumber()+1;
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
        eOmc_motor_config_t    motor_cfg;
        motor_cfg.maxcurrentofmotor = _currentLimits[logico];
        motor_cfg.gearboxratio = _gearbox[logico];
        motor_cfg.rotorEncoderResolution = _rotorEncoderRes[logico];
        motor_cfg.hasHallSensor = _hasHallSensor[logico];
        motor_cfg.hasRotorEncoder = _hasRotorEncoder[logico];
        motor_cfg.hasTempSensor = _hasTempSensor[logico];
        motor_cfg.hasRotorEncoderIndex = _hasRotorEncoderIndex[logico];
        motor_cfg.motorPoles = _motorPoles[logico];
        motor_cfg.rotorIndexOffset = _rotorIndexOffset[logico];
        motor_cfg.filler01 = 0;
        motor_cfg.pidcurrent.kp = 8;
        motor_cfg.pidcurrent.ki = 2;
        motor_cfg.pidcurrent.scale = 10;
        if (false == res->setRemoteValueUntilVerified(protid, &motor_cfg, sizeof(motor_cfg), 10, 0.010, 0.050, 2))
        {
            yError() << "FATAL: embObjMotionControl::init() had an error while calling setRemoteValueUntilVerified() for motor config fisico #" << fisico << "in BOARD" << res->get_protBRDnumber() + 1;
            return false;
        }
        else
        {
            if (verbosewhenok)
            {
                yDebug() << "embObjMotionControl::init() correctly configured motor config fisico #" << fisico << "in BOARD" << res->get_protBRDnumber() + 1;
            }
        }

        /*
        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, fisico, eoprot_tag_mc_motor_config_maxcurrentofmotor);
        eOmeas_current_t    current = (eOmeas_current_t) S_16(_currentLimits[logico]);
        if(false == res->setRemoteValueUntilVerified(protid, &current, sizeof(current), 10, 0.010, 0.050, 2))
        {
            yError() << "FATAL: embObjMotionControl::init() had an error while calling setRemoteValueUntilVerified() for motor config fisico #" << fisico << "in BOARD" << res->get_protBRDnumber()+1;
            return false;
        }
        else
        {
            if(verbosewhenok)
            {
                yDebug() << "embObjMotionControl::init() correctly configured motor config fisico #" << fisico << "in BOARD" << res->get_protBRDnumber()+1;
            }
        }

        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, fisico, eoprot_tag_mc_motor_config_gearboxratio);
        int32_t gearbox = (int32_t) (_gearbox[logico]);
        if(false == res->setRemoteValueUntilVerified(protid, &gearbox, sizeof(gearbox), 10, 0.010, 0.050, 2))
        {
            yError() << "FATAL: embObjMotionControl::init() had an error while calling setRemoteValueUntilVerified() for motor config fisico #" << fisico << "in BOARD" << res->get_protBRDnumber()+1;
            return false;
        }
        else
        {
            if(verbosewhenok)
            {
                yDebug() << "embObjMotionControl::init() correctly configured motor config fisico #" << fisico << "in BOARD" << res->get_protBRDnumber()+1;
            }
        }

        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, fisico, eoprot_tag_mc_motor_config_rotorencoder);
        int32_t rotor = (int32_t) (_rotorEncoderRes[logico]);
        if(false == res->setRemoteValueUntilVerified(protid, &rotor, sizeof(rotor), 10, 0.010, 0.050, 2))
        {
            yError() << "FATAL: embObjMotionControl::init() had an error while calling setRemoteValueUntilVerified() for motor config fisico #" << fisico << "in BOARD" << res->get_protBRDnumber()+1;
            return false;
        }
        else
        {
            if(verbosewhenok)
            {
                yDebug() << "embObjMotionControl::init() correctly configured motor config fisico #" << fisico << "in BOARD" << res->get_protBRDnumber()+1;
            }
        }
        */
    }

    /////////////////////////////////////////////
    // invia la configurazione del controller  //
    /////////////////////////////////////////////
    
    {   // configuration of the joint coupling matrix inside the controller

        eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_controller, 0, eoprot_tag_mc_controller_config_jointcoupling);

        eOq17_14_t protmatrix[4][4] = {0};
 
        for(int i=0; i<4; i++)
        {
            for(int j=0; j<4; j++)
            {
                int k=i*4+j;
                protmatrix[i][j] = eo_common_float_to_Q17_14(_kinematic_mj[k]);
                //printf("pos = %d %d: xml=%f, prot=%x, conv = %f\n", i, j, _kinematic_mj[k], protmatrix[i][j], eo_common_Q17_14_to_float(protmatrix[i][j]));
            }
        }

        if(false == res->setRemoteValueUntilVerified(id32, protmatrix, sizeof(protmatrix), 10, 0.010, 0.050, 2))
        {
            yError() << "FATAL: embObjMotionControl::init() had an error while calling setRemoteValueUntilVerified() for mc controller config joint coupling matrix" << "in BOARD" << res->get_protBRDnumber()+1;
            return false;
        }
        else
        {
            if(verbosewhenok)
            {
                yDebug() << "embObjMotionControl::init() correctly configured mc controller config joint coupling matrix" << "in BOARD" << res->get_protBRDnumber()+1;
            }
        }

    }

    yTrace() << "EmbObj Motion Control for board " << _fId.boardNumber << " istantiated correctly\n";
    return true;
}

#if !defined(EMBOBJMC_DONT_USE_MAIS)

bool embObjMotionControl::configure_mais(yarp::os::Searchable &config)
{
    // Mais per lettura posizioni dita, c'e' solo sulle mani per ora

#if 1
    // version with read-back

    uint8_t datarate = 10;    // 10 milli (like in icub_right_arm_safe.ini)  // type ok
    eOenum08_t maismode  = eoas_maismode_txdatacontinuously; // use eOas_maismode_t for value BUT USE   for type (their sizes can be different !!)

    eOprotID32_t id32 = eo_prot_ID32dummy;


    if(0 == eoprot_entity_numberof_get(featIdBoardNum2nvBoardNum(_fId.boardNumber), eoprot_endpoint_analogsensors, eoprot_entity_as_mais))
    {
        return false;
    }

    // ok, we have a mais


    // -- mais datarate

    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_config_datarate);

    if(false == res->setRemoteValueUntilVerified(id32, &datarate, sizeof(datarate), 10, 0.010, 0.050, 2))
    {
        yError() << "FATAL: embObjMotionControl::configure_mais() had an error while calling setRemoteValueUntilVerified() for mais datarate in BOARD" << res->get_protBRDnumber()+1;
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjMotionControl::configure_mais() correctly configured mais datarate at value" << datarate << "in BOARD" << res->get_protBRDnumber()+1;
        }
    }

    // -- mais tx mode


    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_config_mode);

    if(false == res->setRemoteValueUntilVerified(id32, &maismode, sizeof(maismode), 10, 0.010, 0.050, 2))
    {
        yError() << "FATAL: embObjMotionControl::configure_mais() had an error while calling setRemoteValueUntilVerified() for mais mode in BOARD" << res->get_protBRDnumber()+1;
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjMotionControl::configure_mais() correctly configured mais mode at value" << maismode << "in BOARD" << res->get_protBRDnumber()+1;
        }
    }

    return true;

#else

    uint8_t datarate = 10;    // 10 milli (like in icub_right_arm_safe.ini)  // type ok
    eOenum08_t maismode  = eoas_maismode_txdatacontinuously;
    
    if(0 == eoprot_entity_numberof_get(featIdBoardNum2nvBoardNum(_fId.boardNumber), eoprot_endpoint_analogsensors, eoprot_entity_as_mais))
    {
        return false;
    }

    // ok, we have a mais


    // set mais datarate = 1millisec
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_config_datarate);

    if(!res->addSetMessage(protid, &datarate))
    {
        yError() << "embObjMotionControl::configure_mais() had send error while setting mais datarate";
        return false;
    }

    Time::delay(0.010);

    if(false == res->verifyRemoteValue(protid, (uint8_t *) &datarate, sizeof(datarate)))
    {
        yError() << "embObjMotionControl::configure_mais() had an error while verifying datarate =" << datarate << "in BOARD" << res->get_protBRDnumber()+1;
    }

    // set tx mode continuosly
    protid = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_config_mode);

    if(!res->addSetMessage(protid, (uint8_t *) &maismode))
    {
        yError() << "embObjMotionControl::configure_mais() had send error while setting mais maismode";
        return false;
    }

    Time::delay(0.010);  // (m.a.a-delay: before it was 0.01)

    if(false == res->verifyRemoteValue(protid, (uint8_t *) &maismode, sizeof(maismode)))
    {
        yError() << "embObjMotionControl::configure_mais() had an error while verifying maismode =" << maismode << "in BOARD" << res->get_protBRDnumber()+1;
        return false;
    }


    return true;

#endif
}

#endif // !defined(EMBOBJMC_DONT_USE_MAIS)


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
    ImplementPositionDirect::uninitialize();
    ImplementOpenLoopControl::uninitialize();
    ImplementInteractionMode::uninitialize();
    ImplementRemoteVariables::uninitialize();
    
    if (_torqueControlHelper)  {delete _torqueControlHelper; _torqueControlHelper=0;}
    
    cleanup();

    return true;
}

void embObjMotionControl::cleanup(void)
{
    if(ethManager == NULL) return;

    int ret = ethManager->releaseResource(_fId);
    res = NULL;
    if(ret == -1)
        ethManager->killYourself();
}


#if 0
ethFeature_t embObjMotionControl::getFeat_id()
{
    return(this->_fId);
}
#endif

// TODO da eliminare?
eoThreadEntry * embObjMotionControl::appendWaitRequest(int j, uint32_t protoid)
{
    eoRequest req;
    if(!requestQueue->threadPool->getId(&req.threadId) )
        yError() << "Error: too much threads!! (embObjMotionControl)";
    req.joint = j;
    req.nvid = res->translate_NVid2index(protoid);
    requestQueue->append(req);
    return requestQueue->threadPool->getThreadTable(req.threadId);
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


void embObjMotionControl::refreshEncoderTimeStamp(int joint)
{
    static long int count = 0;
    count++;

    if(true == initialised())
    {   // do it only if we already have opened the device
        _mutex.wait();
        _encodersStamp[joint] = Time::now();
        _mutex.post();
    }
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
        yError() << "while setting position PIDs for board " << _fId.boardNumber << " joint " << j;
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
        yWarning() << "setReferenceRaw: Deprecated automatic switch to VOCAB_CM_POSITION_DIRECT, board " << _fId.boardNumber << " joint "<< j;
        setControlModeRaw(j,VOCAB_CM_POSITION_DIRECT);
        #else
        yError() << "setReferenceRaw: skipping command because board " << _fId.boardNumber << " joint " << j << " is not in VOCAB_CM_POSITION_DIRECT mode";
        #endif
    }

    eOprotID32_t protoId = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);
    eOmc_setpoint_t setpoint = {0};

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
    int mycontrolMode;
    /* Values in pid.XXX fields are valid ONLY IF we are in the corresponding control mode.
    Read it from the signalled message so we are sure that mode and pid values are coherent to each other */
    getControlModeRaw(j, &mycontrolMode);
    if(VOCAB_CM_POSITION != mycontrolMode )
    {
        //yWarning() << "Asked for Position PID Error while not in Position control mode. Returning zeros";
        err = 0;
        return false;
    }

    eOprotID32_t protoId = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_ofpid);
    uint16_t size;
    eOmc_joint_status_ofpid_t  tmpJointStatus;
    res->readBufferedValue(protoId, (uint8_t *)&tmpJointStatus, &size);
    *err = (double) tmpJointStatus.error;
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
        yError() << "Can't send get pid request for board " << _fId.boardNumber << " joint " << j;
        return false;
    }

    // wait here
    if(-1 == tt->synch() )
    {
        int threadId;
        yError () << "embObjMotionControl::getPidRaw() timed out the wait of reply from board " << _fId.boardNumber << " joint " << j;

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
    eOprotID32_t protoId = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_ofpid);
    uint16_t size;
    eOmc_joint_status_ofpid_t  tmpJointStatus;
    res->readBufferedValue(protoId, (uint8_t *)&tmpJointStatus, &size);
    *ref = tmpJointStatus.positionreference;
    return true;
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
        yWarning() << "velocityMoveRaw: Deprecated automatic switch to VOCAB_CM_VELOCITY, board " << _fId.boardNumber << " joint " << j;
        setControlModeRaw(j, VOCAB_CM_VELOCITY);
        #else
        yError() << "velocityMoveRaw: skipping command because board " << _fId.boardNumber << " joint " << j << " is not in VOCAB_CM_VELOCITY mode";
        #endif
    }

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);

    _command_speeds[j] = sp ;   // save internally the new value of speed.

    eOmc_setpoint_t setpoint;
    setpoint.type = eomc_setpoint_velocity;
    setpoint.to.velocity.value =  (eOmeas_velocity_t) S_32(_command_speeds[j]);
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

bool embObjMotionControl::calibrate2Raw(int j, unsigned int type, double p1, double p2, double p3)
{
    yTrace() << "calibrate2Raw for BOARD " << _fId.boardNumber << "joint" << j;

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
    // used only in calibration procedure, for normal work use the checkMotionDone

    bool result = false;
    uint16_t size;
    eOmc_controlmode_t type;
    eOmc_joint_status_basic_t status;

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, axis, eoprot_tag_mc_joint_status_basic);
    //EO_WARNING("acemor-> see wait of 100 msec in embObjMotionControl::doneRaw(int axis)")
    // acemor on 21 may 2013: the delay is not necessary in normal cases when the joint cleanly starts in eomc_controlmode_idle and the calibrator sets it to eomc_controlmode_calib.
    // however we keep it because: ... sometimes the boards are not started cleanly (see later note1) and the controlmodestatus can contain an invalid value.
    //                                 the value becomes eomc_controlmode_calib only after the calibration command has arrived to the ems (and if relevant then to the can board and it status
    //                                 is signalled back). 
    // thus: al least for the first call after a calibration command on that joint we should keep the delay. Moreover: a delay of 1 sec is 
    // in parametricCalibrator::checkCalibrateJointEnded() just before calling doneRaw().

    Time::delay(0.1);

    //#warning --> marco.accame: embObjMotionControl::doneRaw() uses delay of 100ms to wait for something ... review this thing.

    res->readBufferedValue(protid, (uint8_t*) &status, &size);
    type = (eOmc_controlmode_t) status.controlmodestatus;

    // if the control mode is no longer a calibration type, it means calibration ended
    if   (eomc_controlmode_idle == type)
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
        yDebug() << "positionMoveRaw: Deprecated automatic switch to VOCAB_CM_POSITION, board " << _fId.boardNumber << " joint " << j;
        setControlModeRaw(j, VOCAB_CM_POSITION);
        #else
        yError() << "positionMoveRaw: skipping command because board " << _fId.boardNumber << " joint " << j << " is not in VOCAB_CM_POSITION mode";
        #endif
    }
    
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);
    _ref_positions[j] = ref;   // save internally the new value of pos.

    eOmc_setpoint_t setpoint;

    setpoint.type = (eOenum08_t) eomc_setpoint_position;
    setpoint.to.position.value =  (eOmeas_position_t) S_32(_ref_positions[j]);
    setpoint.to.position.withvelocity = (eOmeas_velocity_t) S_32(_ref_speeds[j]);


//    yDebug() << "Position move EP" << _fId.endpoint << "j" << j << setpoint.to.position.value << "\tspeed " << setpoint.to.position.withvelocity  << " at time: " << (Time::now()/1e6);

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
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_motionmonitormode);

//    EOnv *nvRoot_config = res->getNVhandler(protid, &tmpnv_config);
//
//    //    printf("nvid of check motion done 0x%04X\n", nvid_config);
//    if(NULL == nvRoot_config)
//    {
//        NV_NOT_FOUND;
//        return false;
//    }

    // monitor status until set point is reached, if it wasn't already set
    // this is because the function has to be in a non blocking fashion and I want to avoid resending the same message over and over again!!

    if(!checking_motiondone[j ])
    {
        checking_motiondone[j ] = true;

        eOmc_motionmonitormode_t tmp = eomc_motionmonitormode_forever;

        res->addSetMessage(protid,(uint8_t *) &tmp);
//         if( !res->nvSetData(nvRoot_config, &tmp, eobool_true, eo_nv_upd_dontdo))
//         {
//             // print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
//             return false;
//         }
//         if(!res->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.endpoint, nvid_config) )
//             return false;
    }


    // Read the current value - it is signalled spontaneously every cycle, so we don't have to wait here
    protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_basic);

    uint16_t size;
    eOmc_joint_status_basic_t status;

    res->readBufferedValue(protid,(uint8_t *) &status, &size);
    if(eomc_motionmonitorstatus_setpointisreached == status.motionmonitorstatus)
    {
        *flag = true;

        // to stop monitoring when set point is reached... this create problems when using the other version of
        // the function with all axis togheter. A for loop cannot be used. Skip it for now
        //        eOmc_motionmonitormode_t tmp = eomc_motionmonitormode_dontmonitor;
        //        if( eomc_motionmonitormode_dontmonitor != *nvRoot_config->motionmonitormode)
        //        {
        //            if( !res->nvSetData(nvRoot_config, &val, eobool_true, eo_nv_upd_dontdo))
        //            {
        //                // print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
        //                return false;
        //            }
        //            res->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.endpoint, nvid_config);
        //        }
        //        checking_motiondone[j ]= false;
    }
    else
        *flag = false;
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
    *spd = _ref_speeds[j];
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

bool embObjMotionControl::getControlModeRaw(int j, int *v)
{
    uint16_t                      size;
    eOmc_joint_status_basic_t     status;

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_basic);
    if(! res->readBufferedValue(protid, (uint8_t *)&status, &size))
        return false;

    eOmc_controlmode_t type = (eOmc_controlmode_t) status.controlmodestatus;

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

bool embObjMotionControl::getStatusBasic_withWait(const int n_joint, const int *joints, eOenum08_t *_modes)
{
    std::vector<eoThreadEntry *>  tt;
    eOmc_joint_status_basic_t     status;
    tt.resize(n_joint);

    eOprotID32_t *protid = new eOprotID32_t[n_joint];

    // Sign up for waiting the replies
    for(int idx=0; idx<n_joint; idx++)
    {
        protid[idx] = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, joints[idx], eoprot_tag_mc_joint_status_basic);
        tt[idx] = appendWaitRequest(joints[idx], protid[idx]);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
        tt[idx]->setPending(1);  // 1 (not n_joint) because it is related to the specific variable

        if(!res->addGetMessage(protid[idx]) )
        {
            yError() << "Can't send getStatusBasic_withWait request for board " << _fId.boardNumber << "joint " << joints[idx];
            return false;
        }
    }

    // wait for data to arrive and read it when available
    for(int idx=0; idx<n_joint; idx++)
    {
        // wait here
        if(-1 == tt[idx]->synch() )
        {
            int threadId;
            yError () << "embObjMotionControl::getStatusBasic_withWait() timed out the wait of reply from BOARD" <<  _fId.boardNumber << " joint " << joints[idx];

            if(requestQueue->threadPool->getId(&threadId))
                requestQueue->cleanTimeouts(threadId);
            return false;
        }
        else
        {
            // Get the value
            uint16_t size;
            res->readBufferedValue(protid[idx], (uint8_t*)&status, &size);
            _modes[idx] = status.controlmodestatus;
        }
    }
    return true;
}


bool embObjMotionControl::setControlModeRaw(const int j, const int _mode)
{
    eOenum08_t      valSet;
    eOenum08_t      valGot;
    bool ret = true;

    //yDebug() << "SetControlMode: received setControlMode command (SINGLE) for board " << _fId.boardNumber << " joint " << j << " mode " << Vocab::decode(_mode);

    if (_mode == VOCAB_CM_TORQUE && _torqueControlEnabled == false) {yError()<<"Torque control is disabled. Check your configuration parameters"; return false;}

    if(!controlModeCommandConvert_yarp2embObj(_mode, valSet) )
    {
        yError() << "SetControlMode: received unknown control mode for board " << _fId.boardNumber << " joint " << j << " mode " << Vocab::decode(_mode);
        return false;
    }

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_controlmode);
    if(! res->addSetMessage(protid, (uint8_t*) &valSet) )
    {
        yError() << "setControlModeRaw failed for board " << _fId.boardNumber << " joint " << j << " mode " << Vocab::decode(_mode);
        return false;
    }
    
    int timeout = 0;
    int current_mode = VOCAB_CM_UNKNOWN;

    do
    {
        bool ret = getStatusBasic_withWait(1, &j, &valGot);
        if (ret == false) {yError ("An error occurred inside setControlModesRaw()"); break;}
        current_mode = controlModeStatusConvert_embObj2yarp(valGot);
        if (current_mode==_mode) {ret = true; break;}
        if (current_mode==VOCAB_CM_IDLE     && _mode==VOCAB_CM_FORCE_IDLE) {ret = true; break;}
        if (current_mode==VOCAB_CM_HW_FAULT)
        {
            if (_mode!=VOCAB_CM_FORCE_IDLE) {yError ("Unable to set the control mode of board %d joint %d in HW_FAULT", _fId.boardNumber, j);}
            ret = true; break;
        }
        yarp::os::Time::delay(0.010);
        if (timeout>0) yWarning ("setControlModeRaw delay (board %d joint %d), current mode: %s, requested: %s", _fId.boardNumber, j, yarp::os::Vocab::decode(current_mode).c_str(), yarp::os::Vocab::decode(_mode).c_str());
        timeout++;
    }
    while (timeout < 10);
    if (timeout>=10)
    {
        ret = false;
        yError ("100ms Timeout occured in setControlModeRaw (board %d joint %d), current mode: %s, requested: %s", _fId.boardNumber, j, yarp::os::Vocab::decode(current_mode).c_str(), yarp::os::Vocab::decode(_mode).c_str());
    }
    return ret;
}

bool embObjMotionControl::setControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    eOenum08_t          *valSet = new eOenum08_t[n_joint];
    eOenum08_t          *valGot = new eOenum08_t[n_joint];
    bool ret = true;

    //yDebug() << "SetControlMode: received setControlMode (GROUP) command for board " << _fId.boardNumber << " mode " << Vocab::decode(modes[0]);

    for(int i=0; i<n_joint; i++)
    {
        if (modes[i] == VOCAB_CM_TORQUE && _torqueControlEnabled == false) {yError()<<"Torque control is disabled. Check your configuration parameters"; continue;}

        if(!controlModeCommandConvert_yarp2embObj(modes[i], valSet[i]) )
        {
            yError() << "SetControlMode: received unknown control mode for board " << _fId.boardNumber << " joint " << joints[i] << " mode " << Vocab::decode(modes[i]);
            delete valSet; delete valGot;
            return false;
        }

        eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, joints[i], eoprot_tag_mc_joint_cmmnds_controlmode);
        if(! res->addSetMessage(protid, (uint8_t*) &valSet[i]) )
        {
            yError() << "setControlModeRaw failed for board " << _fId.boardNumber << " joint " << joints[i] << " mode " << Vocab::decode(modes[i]);
            delete valSet; delete valGot;
            return false;
        }

        int current_mode = VOCAB_CM_UNKNOWN;
        int timeout = 0;
        do
        {
            bool ret = getStatusBasic_withWait(1, &joints[i], &valGot[i]);
            if (ret == false) {yError ("An error occurred inside setControlModesRaw()"); break;}
            current_mode = controlModeStatusConvert_embObj2yarp(valGot[i]);
            if (current_mode==modes[i]) {ret = true; break;}
            if (current_mode==VOCAB_CM_IDLE     && modes[i]==VOCAB_CM_FORCE_IDLE) {ret = true; break;}
            if (current_mode==VOCAB_CM_HW_FAULT)
            {
                if (modes[i]!=VOCAB_CM_FORCE_IDLE) {yError ("Unable to set the control mode of board %d joint %d in HW_FAULT", _fId.boardNumber, joints[i]);}
                ret = true; break;
            }
            yarp::os::Time::delay(0.010);
            if (timeout>0) yWarning ("setControlModesRaw delay (board %d joint %d), current mode: %s, requested: %s", _fId.boardNumber, joints[i], yarp::os::Vocab::decode(current_mode).c_str(), yarp::os::Vocab::decode(modes[i]).c_str());
            timeout++;
        }
        while (timeout < 10);
        if (timeout>=10)
        {
            ret = false;
            yError ("100ms Timeout occured in setControlModesRaw (board %d joint %d), current mode: %s, requested: %s", _fId.boardNumber, joints[i], yarp::os::Vocab::decode(current_mode).c_str(), yarp::os::Vocab::decode(modes[i]).c_str());
        }
    }

    delete valSet; delete valGot;
    return ret;
}

bool embObjMotionControl::setControlModesRaw(int *modes)
{
    int  *jointVector = new int[_njoints];

    eOenum08_t          *valSet = new eOenum08_t[_njoints];
    eOenum08_t          *valGot = new eOenum08_t[_njoints];
    bool ret = true;

    for(int i=0; i<_njoints; i++)
    {
        jointVector[i] = i;
        
        if (modes[i] == VOCAB_CM_TORQUE && _torqueControlEnabled == false) {yError()<<"Torque control is disabled. Check your configuration parameters"; continue;}
            
        if(!controlModeCommandConvert_yarp2embObj(modes[i], valSet[i]) )
        {
            yError() << "SetControlMode: received unknown control mode for board " << _fId.boardNumber << " joint " << i << " mode " << Vocab::decode(modes[i]);
            delete jointVector; delete valSet; delete valGot;
            return false;
        }

        eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, i, eoprot_tag_mc_joint_cmmnds_controlmode);
        if(! res->addSetMessage(protid, (uint8_t*) &valSet[i]) )
        {
            yError() << "setControlModesRaw failed for board " << _fId.boardNumber << " joint " << i << " mode " << Vocab::decode(modes[i]);
            delete jointVector; delete valSet; delete valGot;
            return false;
        }

        int current_mode = VOCAB_CM_UNKNOWN;
        int timeout = 0;
        do
        {
            bool ret = getStatusBasic_withWait(1, &i, &valGot[i]);
            if (ret == false) {yError ("An error occurred inside setControlModesRaw()"); break;}
            current_mode = controlModeStatusConvert_embObj2yarp(valGot[i]);
            if (current_mode==modes[i]) {ret = true; break;}
            if (current_mode==VOCAB_CM_IDLE     && modes[i]==VOCAB_CM_FORCE_IDLE) {ret = true; break;}
            if (current_mode==VOCAB_CM_HW_FAULT)
            {
                if (modes[i]!=VOCAB_CM_FORCE_IDLE) {yError ("Unable to set the control mode of board %d joint %d in HW_FAULT", _fId.boardNumber, i);}
                ret = true; break;
            }
            yarp::os::Time::delay(0.010);
            if (timeout>0) yWarning ("setControlModesRaw delay (board %d joint %d), current mode: %s, requested: %s", _fId.boardNumber, i, yarp::os::Vocab::decode(current_mode).c_str(), yarp::os::Vocab::decode(modes[i]).c_str());
            timeout++;
        }
        while (timeout < 10);
        if (timeout>=10)
        {
            ret = false;
            yError ("100ms Timeout occured in setControlModesRaw (board %d joint %d), current mode: %s, requested: %s", _fId.boardNumber, i, yarp::os::Vocab::decode(current_mode).c_str(), yarp::os::Vocab::decode(modes[i]).c_str());
        }
    }

    delete jointVector; delete valSet; delete valGot;
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
    eOmc_joint_status_basic_t     status;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_basic);

    bool ret = res->readBufferedValue(protid, (uint8_t *)&status, &size);

    if(ret)
    {
        eOmc_controlmode_t type = (eOmc_controlmode_t) status.controlmodestatus;
        *value = (double) status.jnt_position;
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
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_basic);
    uint16_t      size;
    eOmc_joint_status_basic_t  tmpJointStatus;
    bool ret = res->readBufferedValue(protid, (uint8_t *)&tmpJointStatus, &size);
    // extract requested data from status
    *sp = (double) tmpJointStatus.jnt_velocity;
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
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_basic);
    uint16_t      size;
    eOmc_joint_status_basic_t  tmpJointStatus;
    bool ret = res->readBufferedValue(protid, (uint8_t *)&tmpJointStatus, &size);
    *acc = (double) tmpJointStatus.jnt_acceleration;
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
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config_maxcurrentofmotor);
    eOmeas_current_t  maxCurrent = (eOmeas_current_t) S_16(val);
    return res->addSetMessage(protid, (uint8_t*) &val);
}

bool embObjMotionControl::getMaxCurrentRaw(int j, double *val)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config_maxcurrentofmotor);
    uint16_t size;
    eOmeas_current_t  maxCurrent;
    bool ret = res->readBufferedValue(protid, (uint8_t *)&maxCurrent, &size);
    *val = (double) maxCurrent;
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

bool embObjMotionControl::getRotorPositionRaw         (int j, double* value)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_status_basic);
    uint16_t size;
    eOmc_motor_status_basic_t  tmpMotorStatus;
    bool ret = res->readBufferedValue(protid, (uint8_t *)&tmpMotorStatus, &size);

    *value = (double) tmpMotorStatus.mot_position;
    return true;
}

bool embObjMotionControl::getRotorPositionsRaw        (double* value)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
        ret = getRotorPositionRaw(j, &value[j]) && ret;

    return ret;
}

bool embObjMotionControl::getRotorSpeedRaw            (int j, double* value)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_status_basic);
    uint16_t size;
    eOmc_motor_status_basic_t  tmpMotorStatus;
    bool ret = res->readBufferedValue(protid, (uint8_t *)&tmpMotorStatus, &size);

    *value = (double) tmpMotorStatus.mot_velocity;
    return true;
}

bool embObjMotionControl::getRotorSpeedsRaw           (double* value)
{
    bool ret = true;
     for(int j=0; j< _njoints; j++)
         ret = getRotorSpeedRaw(j, &value[j]) && ret;

     return ret;
}
bool embObjMotionControl::getRotorAccelerationRaw     (int j, double* value)        { return NOT_YET_IMPLEMENTED("getRotorAccelerationRaw");  }
bool embObjMotionControl::getRotorAccelerationsRaw    (double* value)               { return NOT_YET_IMPLEMENTED("getRotorAccelerationsRaw");  }
bool embObjMotionControl::getJointPositionRaw         (int j, double* value)        { return NOT_YET_IMPLEMENTED("getJointPositionRaw");  }
bool embObjMotionControl::getJointPositionsRaw        (double* value)               { return NOT_YET_IMPLEMENTED("getJointPositionsRaw");  }
#endif

// Limit interface
bool embObjMotionControl::setLimitsRaw(int j, double min, double max)
{
    bool ret = true;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_limitsofjoint);

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
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_limitsofjoint);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protoid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if(!res->addGetMessage(protoid) )
    {
        yError() << "Can't send get min position limit request for board" << _fId.boardNumber << "joint " << j;
        return false;
    }

    // wait here
    if(-1 == tt->synch() )
    {
        int threadId;
        yError () << "embObjMotionControl::getLimitsRaw() timed out the wait of reply from BOARD" <<  _fId.boardNumber << " joint " << j;

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

// IRemoteVariables
bool embObjMotionControl::getRemoteVariableRaw(yarp::os::ConstString key, yarp::os::Bottle& val)
{
    val.clear();
    if (key == "kinematic_mj")
    {
        val.addString("1 2 3"); return true;
    }
    else if (key == "rotor")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) r.addDouble(_rotorEncoderRes[i]); return true;
    }
    else if (key == "gearbox")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) r.addDouble(_gearbox[i]); return true;
    }
    else if (key == "zeros")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) r.addDouble(_zeros[i]); return true;
    }
    else if (key == "hasHallSensor")
    {
        Bottle& r = val.addList(); for (int i = 0; i < _njoints; i++) r.addInt(_hasHallSensor[i]); return true;
    }
    else if (key == "hasTempSensor")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) r.addInt(_hasTempSensor[i]); return true;
    }
    else if (key == "hasRotorEncoder")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) r.addInt(_hasRotorEncoder[i]); return true;
    }
    else if (key == "hasRotorEncoderIndex")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) r.addInt(_hasRotorEncoderIndex[i]); return true;
    }
    else if (key == "rotorIndexOffset")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) r.addInt(_rotorIndexOffset[i]); return true;
    }
    else if (key == "motorPoles")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) r.addInt(_motorPoles[i]); return true;
    }
    else if (key == "pidCurrentKp")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) r.addDouble(_cpids[i].kp); return true;
    }
    else if (key == "pidCurrentKi")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) r.addDouble(_cpids[i].ki); return true;
    }
    else if (key == "pidCurrentShift")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) r.addDouble(_cpids[i].scale); return true;
    }
    yWarning("getRemoteVariable(): Unknown variable %s", key.c_str());
    return false;
}

bool embObjMotionControl::setRemoteVariableRaw(yarp::os::ConstString key, const yarp::os::Bottle& val)
{
    string s1 = val.toString();
    Bottle* bval = val.get(0).asList();
    if (bval == 0)
    {
        yWarning("setRemoteVariable(): Protocol error %s", s1.c_str());
        return false;
    }

    string s2 = bval->toString();
    if (key == "kinematic_mj")
    {
        return true;
    }
    else if (key == "rotor")
    {
        for (int i = 0; i < _njoints; i++)
            _rotorEncoderRes[i] = bval->get(i).asDouble();
        return true;
    }
    else if (key == "gearbox")
    {
        for (int i = 0; i < _njoints; i++) _gearbox[i] = bval->get(i).asDouble(); return true;
    }
    else if (key == "zeros")
    {
        for (int i = 0; i < _njoints; i++) _zeros[i] = bval->get(i).asDouble(); return true;
    }
    yWarning("setRemoteVariable(): Unknown variable %s", key.c_str());
    return false;
}

bool embObjMotionControl::getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys)
{
    listOfKeys->clear();
    listOfKeys->addString("kinematic_mj");
    listOfKeys->addString("rotor");
    listOfKeys->addString("gearbox");
    listOfKeys->addString("hasHallSensor");
    listOfKeys->addString("hasTempSensor");
    listOfKeys->addString("hasRotorEncoder");
    listOfKeys->addString("hasRotorEncoderIndex");
    listOfKeys->addString("rotorIndexOffset");
    listOfKeys->addString("motorPoles");
    listOfKeys->addString("pidCurrentKp");
    listOfKeys->addString("pidCurrentKi");
    listOfKeys->addString("pidCurrentShift");
    return true;
}

// IControlLimits2
bool embObjMotionControl::setVelLimitsRaw(int axis, double min, double max)
{
    return NOT_YET_IMPLEMENTED("setVelLimitsRaw");
}

bool embObjMotionControl::getVelLimitsRaw(int axis, double *min, double *max)
{
    return NOT_YET_IMPLEMENTED("getVelLimitsRaw");
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
    double NEWTON2SCALE=32767.0/_maxTorque[j];

    eOmeas_torque_t meas_torque = 0;
    static double curr_time = Time::now();
    static int    count_saturation=0;
       
    if(0 != _maxTorque[j])
    {
        if(fTorque < (- _maxTorque[j] ))
        {
            if (Time::now() - curr_time > 2.0)
            {
                yWarning ("embObjMotionControl::updateMeasure() torque measure saturated to %+4.4f on joint %d, count: %d", _maxTorque[j], userLevel_jointNumber, count_saturation);
                curr_time = Time::now();
            }
            fTorque = (- _maxTorque[j]);
            count_saturation++;
        }
        if(fTorque > _maxTorque[j])
        {
            if (Time::now() - curr_time > 2.0)
            {
                yWarning ("embObjMotionControl::updateMeasure() torque measure saturated to %+4.4f on joint %d, count: %d", _maxTorque[j], userLevel_jointNumber, count_saturation);
                curr_time = Time::now();
            }
            fTorque = _maxTorque[j];
            count_saturation++;
        }

        meas_torque = (eOmeas_torque_t) S_16(NEWTON2SCALE*fTorque);
    }

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
    double NEWTON2SCALE=32767.0/_maxTorque[j];
    eOmeas_torque_t meas_torque;
    uint16_t size;
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_inputs_externallymeasuredtorque);
    bool ret = res->readSentValue(protoid, (uint8_t*) &meas_torque, &size);
    *t = ( (double) meas_torque / NEWTON2SCALE);
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
    setpoint.to.torque.value =  (eOmeas_torque_t) S_16(t);

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);
    return res->addSetMessage(protid, (uint8_t*) &setpoint);
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

    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_ofpid);
    uint16_t size;
    eOmc_joint_status_ofpid_t  jstatuspid;
    res->readBufferedValue(id32, (uint8_t *)&jstatuspid, &size);
    *t = (double) jstatuspid.torquereference;
    return true;

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
    uint16_t size;
    bool ret = true;
    eOmc_joint_status_ofpid_t pid_status;
    int mycontrolMode;
    /* Values in pid.XXX fields are valid ONLY IF we are in the corresponding control mode.
    Read it from the signalled message so we are sure that mode and pid values are coherent to each other
    In realtà potrebbe arrivare un nuovo msg tra la lettura del controlmode e la lettura dello status del pid
    Approfondire!! TODO*/

    getControlModeRaw(j, &mycontrolMode);
    if(VOCAB_CM_TORQUE != mycontrolMode)
    {
        //yWarning() << "Asked for Torque PID Error while not in Torque control mode. Returning zeros";
        err = 0;
        return false;
    }
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_ofpid);

    ret = res->readBufferedValue(protid, (uint8_t *)&pid_status, &size);
    *err = (double) pid_status.error;
    return ret;
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
        yError () << "embObjMotionControl::getTorquePidRaw() timed out the wait of reply from BOARD" << _fId.boardNumber << "joint " << j;


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

    *stiffness = (double) (val.stiffness * 0.001);
    *damping = (double) (val.damping * 0.001);
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
        yError () << "embObjMotionControl::getWholeImpedanceRaw() timed out the wait of reply from BOARD" << _fId.boardNumber << "joint " << j;

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

    // EMS will divide stiffness value by 1000 because the cycle is 1KHz. It is done on the EMS since it manage the cycle and knows the real Rate.
    _cacheImpedance[j].stiffness = (eOmeas_stiffness_t) U_32(stiffness * 1000.0);
    _cacheImpedance[j].damping   = (eOmeas_damping_t) U_32(damping * 1000.0);

    val.stiffness     = _cacheImpedance[j].stiffness;
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

    _cacheImpedance[j].offset   = (eOmeas_torque_t) S_16(offset);
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
    return NOT_YET_IMPLEMENTED("getTorquePidOutputRaw");
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
        yError() << "embObjMotionControl::getMotorTorqueParamsRaw() could not send get message for BOARD" << _fId.boardNumber << "joint " << j;
        return false;
    }

    // wait here
    if(-1 == tt->synch() )
    {
        int threadId;
        yError () << "embObjMotionControl::getMotorTorqueParamsRaw() timed out the wait of reply from BOARD" << _fId.boardNumber << "joint " << j;

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
        yError() << "embObjMotionControl::setMotorTorqueParamsRaw() could not send set message for BOARD" << _fId.boardNumber << "joint " << j;
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
        yError() << "while setting velocity PIDs for board " << _fId.boardNumber << " joint " << j;
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
        yError() << "Can't send getVelPidRaw() request for board " << _fId.boardNumber << " joint " << j;
        return false;
    }

    // wait here
    if (-1 == tt->synch())
    {
        int threadId;
        yError() << "embObjMotionControl::getVelPidRaw() timed out the wait of reply from board " << _fId.boardNumber << " joint " << j;

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
    // needs to send both position and velocit as well as positionMove
    // does the same as setReferenceRaw, with some more misterious (missing) checks.
    int mode = 0;
    getControlModeRaw(j, &mode);
    if (mode != VOCAB_CM_POSITION_DIRECT &&
        mode != VOCAB_CM_IDLE)
    {
        #ifdef ICUB_AUTOMATIC_MODE_SWITCHING
        yWarning() << "setPositionRaw: Deprecated automatic switch to VOCAB_CM_POSITION_DIRECT, board " << _fId.boardNumber << " joint " << j;
        setControlModeRaw(j,VOCAB_CM_POSITION_DIRECT);
        #else
        yError() << "setPositionRaw: skipping command because board " << _fId.boardNumber << " joint " << j << " is not in VOCAB_CM_POSITION_DIRECT mode";
        #endif
    }

    return setReferenceRaw(j, ref);
}

bool embObjMotionControl::setPositionsRaw(const int n_joint, const int *joints, double *refs)
{
    // needs to send both position and velocit as well as positionMove
    bool ret = true;    
    for(int i=0; i<n_joint; i++)
    {
        ret &= setReferenceRaw(joints[i], refs[i]);
    }
    return ret;
}

bool embObjMotionControl::setPositionsRaw(const double *refs)
{
    // needs to send both position and velocit as well as positionMove
    return setReferencesRaw(refs);
}



// InteractionMode


bool embObjMotionControl::getInteractionMode_withWait(const int n_joint, const int *joints, eOenum08_t *_modes)
{
    std::vector<eoThreadEntry *>tt;
    tt.resize(n_joint);

    eOprotID32_t *protid = new eOprotID32_t[n_joint];

    // Sign up for waiting the replies
    for(int idx=0; idx<n_joint; idx++)
    {
        protid[idx] = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, joints[idx], eoprot_tag_mc_joint_status_interactionmodestatus);
        tt[idx] = appendWaitRequest(joints[idx], protid[idx]);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
        tt[idx]->setPending(1);  // 1 (not n_joint) because it is related to the specific variable

        if(!res->addGetMessage(protid[idx]) )
        {
            yError() << "Can't send getInteractionMode_withWait request for board " << _fId.boardNumber << " joint " << joints[idx];
            delete protid;
            return false;
        }
    }

    // wait for data to arrive and read it when available
    for(int idx=0; idx<n_joint; idx++)
    {
        // wait here
        if(-1 == tt[idx]->synch() )
        {
            int threadId;
            yError () << "embObjMotionControl::getInteractionMode_withWait() timed out the reply from board "<< _fId.boardNumber << " joint " << joints[idx];

            if(requestQueue->threadPool->getId(&threadId))
                requestQueue->cleanTimeouts(threadId);

            delete protid;
            return false;
        }
        else
        {
            yWarning () << "getInteractionMode_withWait happily got reply board "<< _fId.boardNumber << " joint " << joints[idx];

            // Get the value
            uint16_t size;
            res->readBufferedValue(protid[idx], (uint8_t*)&_modes[idx], &size);
        }
    }

    delete protid;
    return true;
}

bool embObjMotionControl::getInteractionModeRaw(int j, yarp::dev::InteractionModeEnum* _mode)
{
    uint16_t     size;
    eOenum08_t   interactionmodestatus;
//    std::cout << "eoMC getInteractionModeRaw SINGLE joint " << j << std::endl;

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_interactionmodestatus);
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

bool embObjMotionControl::setInteractionModeRaw(int j, yarp::dev::InteractionModeEnum _mode)
{
    eOenum08_t      valSet;
    eOenum08_t      valGot;

//    yDebug() << "received setInteractionModeRaw command (SINGLE) for board " << _fId.boardNumber << " joint " << j << " mode " << Vocab::decode(_mode);

    if (_mode == VOCAB_IM_COMPLIANT && _torqueControlEnabled == false) {yError()<<"Torque control is disabled. Check your configuration parameters"; return false;}

    if(!interactionModeCommandConvert_yarp2embObj(_mode, valSet) )
    {
        yError() << "setInteractionModeRaw: received unknown mode for board " << _fId.boardNumber << " joint " << j << " mode " << Vocab::decode(_mode);
    }

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_interactionmode);

    if(! res->addSetMessage(protid, (uint8_t*) &valSet) )
    {
        yError() << "setInteractionModeRaw failed for board " << _fId.boardNumber << " joint " << j << " mode " << Vocab::decode(_mode);
        return false;
    }

//    // ask back the controlMode to verify everything went correctly (some board could not support compliant mode)
//    yarp::os::Time::delay(0.1);
//    bool ret = getInteractionMode_withWait(1, &j, &valGot);

//    if( (!ret) || (valGot != valSet))
//    {
//        yError() << "check of setInteractionModeRaw failed for board " << _fId.boardNumber << " joint " << j << " mode " << Vocab::decode(_mode);
//        return false;
//    }
    return true;
}

bool embObjMotionControl::setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
//    std::cout << "setInteractionModeRaw GROUP " << std::endl;

    int  *jointVector = new int[n_joints];

    eOenum08_t  *valSet = new eOenum08_t [n_joints];
    eOenum08_t  *valGot = new eOenum08_t [n_joints];

    for(int j=0; j<n_joints; j++)
    {
        if (modes[j] == VOCAB_IM_COMPLIANT && _torqueControlEnabled == false) {yError()<<"Torque control is disabled. Check your configuration parameters"; continue;}

        if(!interactionModeCommandConvert_yarp2embObj(modes[j], valSet[j]) )
        {
            yError() << "setInteractionModeRaw: received unknown interactionMode for board " << _fId.boardNumber << " joint " << j << " mode " << Vocab::decode(modes[j]) << " " << modes[j];
            delete jointVector; delete valSet; delete valGot;
            return false;
        }

        eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_interactionmode);
        if(! res->addSetMessage(protid, (uint8_t*) &valSet[j]) )
        {
            yError() << "setInteractionModeRaw failed for board " << _fId.boardNumber << " joint " << j << " mode " << Vocab::decode(modes[j]);
            delete jointVector; delete valSet; delete valGot;
            return false;
        }
    }

//    // ask back the controlMode to verify everything went correctly (some board could not support compliant mode)
//    if(!getInteractionMode_withWait(n_joints, joints, valGot) )
//    {
//        yError() << "setInteractionModeRaw failed while checking if the new interactionMode was correctly set";
//        return false;
//    }

//    for(int j=0; j<n_joints; j++)
//    {
//        if( valGot[j] != valSet[j])
//        {
//            int tmp;
//            if(interactionModeStatusConvert_embObj2yarp(valGot[j], tmp) )
//                yError() << "setInteractionModeRaw failed for board " << _fId.boardNumber << " joint " << j << " because of interactionMode mismatching \n\tSet " \
//                         << Vocab::decode(modes[j]) << " Got " << Vocab::decode(tmp);
//            else
//                yError() << "setInteractionModeRaw failed for board " << _fId.boardNumber << " joint " << j << " because of interactionMode mismatching \n\tSet " \
//                         << Vocab::decode(modes[j]) << " Got an unknown value!";
//            return false;
//        }
//    }

    delete jointVector; delete valSet; delete valGot;
    return true;
}

bool embObjMotionControl::setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    int  *jointVector = new int[_njoints];

    eOenum08_t     *valSet = new eOenum08_t [_njoints];
    eOenum08_t     *valGot = new eOenum08_t [_njoints];

    for(int j=0; j<_njoints; j++)
    {
        if (modes[j] == VOCAB_IM_COMPLIANT && _torqueControlEnabled == false) {yError()<<"Torque control is disabled. Check your configuration parameters"; continue;}

        jointVector[j] = j;
        if(!interactionModeCommandConvert_yarp2embObj(modes[j], valSet[j]) )
        {
            yError() << "setInteractionModeRaw: received unknown interactionMode for board " << _fId.boardNumber << " joint " << j << " mode " << Vocab::decode(modes[j]);
            delete jointVector; delete valSet; delete valGot;
            return false;
        }

        eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_interactionmode);
        if(! res->addSetMessage(protid, (uint8_t*) &valSet[j]) )
        {
            yError() << "setInteractionModeRaw failed for board " << _fId.boardNumber << " joint " << j << " mode " << Vocab::decode(modes[j]);
            delete jointVector; delete valSet; delete valGot;
            return false;
        }
    }

//    // ask back the controlMode to verify everything went correctly (some board could not support compliant mode)
//    if(!getInteractionMode_withWait(_njoints, jointVector, valGot) )
//    {
//        yError() << "setInteractionModeRaw failed while checking if the new interactionMode was correctly set";
//        return false;
//    }

//    for(int j=0; j<_njoints; j++)
//    {
//        if( valGot[j] != valSet[j])
//        {
//            int tmp;
//            if(interactionModeStatusConvert_embObj2yarp(valGot[j], tmp) )
//                yError() << "setInteractionModeRaw failed for board " << _fId.boardNumber << " joint " << j << " because of interactionMode mismatching \n\tSet " \
//                         << Vocab::decode(modes[j]) << " Got " << Vocab::decode(tmp);
//            else
//                yError() << "setInteractionModeRaw failed for board " << _fId.boardNumber << " joint " << j << " because of interactionMode mismatching \n\tSet " \
//                         << Vocab::decode(modes[j]) << " Got an unknown value!";            return false;
//        }
//    }

    delete jointVector; delete valSet; delete valGot;
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

    setpoint.type = (eOenum08_t) eomc_setpoint_current;
    setpoint.to.current.value =  (eOmeas_current_t) S_16(v);

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
    bool ret = true;
    eOprotID32_t protoId = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_ofpid);
    uint16_t size;
    eOmc_joint_status_ofpid_t  tmpJointStatus;
    if(res->readBufferedValue(protoId, (uint8_t *)&tmpJointStatus, &size) )
    {
        *out = (double) tmpJointStatus.positionreference;
    }
    else
    {
        *out = 0;
        ret = false;
    }
    return ret;
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
    bool ret = true;
    eOprotID32_t protoId = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_ofpid);
    uint16_t size;
    eOmc_joint_status_ofpid_t  tmpJointStatus;
    if(res->readBufferedValue(protoId, (uint8_t *)&tmpJointStatus, &size) )
    {
        *out = (double) tmpJointStatus.output;
    }
    else
    {
        *out = 0;
        ret = false;
    }
    return ret;
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
    return NOT_YET_IMPLEMENTED("getTemperatureRaw");
}

bool embObjMotionControl::getTemperaturesRaw(double *vals)
{
    return NOT_YET_IMPLEMENTED("getTemperaturesRaw");
}

bool embObjMotionControl::getTemperatureLimitRaw(int m, double *temp)
{
    return NOT_YET_IMPLEMENTED("getTemperatureLimitRaw");
}

bool embObjMotionControl::setTemperatureLimitRaw(int m, const double temp)
{
    return NOT_YET_IMPLEMENTED("setTemperatureLimitRaw");
}

bool embObjMotionControl::getMotorOutputLimitRaw(int m, double *limit)
{
    return NOT_YET_IMPLEMENTED("getMotorOutputLimitRaw");
}

bool embObjMotionControl::setMotorOutputLimitRaw(int m, const double limit)
{
    return NOT_YET_IMPLEMENTED("setMotorOutputLimitRaw");
}


// eof
