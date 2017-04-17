    // -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2015 iCub Facility, Istituto Italiano di Tecnologia
* Authors: Alberto Cardellino
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*
*/

//#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>
#include <tripodMotionControl.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;
using namespace icub::dev;
using namespace icub::dev::impl;
// using namespace yarp::dev::impl;

HW_deviceHelper::HW_deviceHelper() : pid(NULL),
                                     pos(NULL),
                                     pos2(NULL),
                                     vel(NULL),
                                     vel2(NULL),
                                     iJntEnc(NULL),
                                     iMotEnc(NULL),
                                     amp(NULL),
                                     lim2(NULL),
                                     calib(NULL),
                                     calib2(NULL),
                                     iTorque(NULL),
                                     iImpedance(NULL),
                                     iOpenLoop(NULL),
                                     iMode(NULL),
                                     iMode2(NULL),
                                     info(NULL),
                                     posDir(NULL),
                                     iInteract(NULL),
                                     configured(false),
                                     _subDevVerbose(false) { }

HW_deviceHelper::~HW_deviceHelper()
{
    detach();
}

void HW_deviceHelper::detach()
{
    _subDevVerbose = false;
    configured = false;
    pid = NULL;
    pos = NULL;
    pos2 = NULL;
    vel = NULL;
    vel2 = NULL;
    iJntEnc = NULL;
    iMotEnc = NULL;
    amp = NULL;
    lim2 = NULL;
    calib = NULL;
    calib2 = NULL;
    iTorque = NULL;
    iImpedance = NULL;
    iOpenLoop = NULL;
    iMode = NULL;
    iMode2 = NULL;
    info = NULL;
    posDir = NULL;
    iInteract = NULL;
    imotor = NULL;
    iVar = NULL;
};

bool HW_deviceHelper::attach(PolyDriver* subdevice)
{
    if (subdevice == NULL)
    {
        yError() << "tripodMotionControl helper: device passed to attach function is NULL";
        return false;
    }

    if (subdevice->isValid())
    {
        subdevice->view(pid);
        subdevice->view(pos);
        subdevice->view(pos2);
        subdevice->view(posDir);
        subdevice->view(vel);
        subdevice->view(vel2);
        subdevice->view(amp);
        subdevice->view(lim2);
        subdevice->view(calib);
        subdevice->view(calib2);
        subdevice->view(info);
        subdevice->view(iTorque);
        subdevice->view(iImpedance);
        subdevice->view(iMode);
        subdevice->view(iMode2);
        subdevice->view(iOpenLoop);
        subdevice->view(iJntEnc);
        subdevice->view(iMotEnc);
        subdevice->view(iInteract);
        subdevice->view(imotor);
        subdevice->view(iVar);
    }
    else
    {
        yError()<<"tripodMotionControl helper: device to attach to is not valid ( isValid() function returned false)";
        return false;
    }

    if ( ((iMode==0) || (iMode2==0)) && (_subDevVerbose ))
        yWarning() << "tripodMotionControl:  Warning iMode not valid interface";

    if ((iTorque==0) && (_subDevVerbose))
        yWarning() << "tripodMotionControl:  Warning iTorque not valid interface";

    if ((iImpedance==0) && (_subDevVerbose))
        yWarning() << "tripodMotionControl:  Warning iImpedance not valid interface";

    if ((iOpenLoop==0) && (_subDevVerbose))
        yWarning() << "tripodMotionControl:  Warning iOpenLoop not valid interface";

    if ((iInteract==0) && (_subDevVerbose))
        yWarning() << "tripodMotionControl:  Warning iInteractionMode not valid interface";

    if ((iMotEnc==0) && (_subDevVerbose))
        yWarning() << "tripodMotionControl:  Warning iMotorEncoder not valid interface";

    if ((imotor==0) && (_subDevVerbose))
        yWarning() << "tripodMotionControl:  Warning iMotor not valid interface";

    if ((iVar == 0) && (_subDevVerbose))
        yWarning() << "tripodMotionControl:  Warning iVar not valid interface";

    int deviceJoints=0;

    // checking minimum set of intefaces required
    if( ! (pos || pos2) ) // One of the 2 is enough, therefore if both are missing I raise an error
    {
        yError("tripodMotionControl: neither IPositionControl nor IPositionControl2 interface was not found in subdevice. Quitting");
        return false;
    }

    configured = true;
    return true;
}

bool HW_deviceHelper::isConfigured()
{
    return configured;
}

bool tripodMotionControl::tripod_user2HW(double *user, double *robot)
{
    for(int i=0; i < _njoints; i++)
        robot[i] = user[i] + 5;
    return true;
}

bool tripodMotionControl::tripod_HW2user(double *robot, double *user)
{
    for(int i=0; i < _njoints; i++)
        user[i] = robot[i] + - 5;
    return true;
}


#if 0
void tripodMotionControl::copyPid_iCub2eo(const Pid *in, Pid *out)
{
    memset(out, 0, sizeof(Pid));     // marco.accame: it is good thing to clear the out struct before copying. this prevent future members of struct not yet managed to be dirty.
    out->kp = (float) (in->kp);
    out->ki = (float) (in->ki);
    out->kd = (float) (in->kd);
//     out->limitonintegral = (float)(in->max_int);
//     out->limitonoutput = (float)(in->max_output);
    out->offset = (float) (in->offset);
    out->scale = (int8_t) (in->scale);
    out->kff = (float) (in->kff);
    out->stiction_down_val = (float)(in->stiction_down_val);
    out->stiction_up_val = (float)(in->stiction_up_val);
}

void tripodMotionControl::copyPid_eo2iCub(Pid *in, Pid *out)
{
    // marco.accame: in here i dont clear the out class because there is not a clear() method
    out->kp = (double) in->kp;
    out->ki = (double) in->ki;
    out->kd = (double) in->kd;
//     out->max_int = (double) in->limitonintegral;
//     out->max_output = (double) in->limitonoutput;
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

#endif

bool tripodMotionControl::NOT_YET_IMPLEMENTED(const char *txt)
{
    if(verbose)
        yError() << txt << " is not yet implemented for tripodMotionControl";
    return false;
}

bool tripodMotionControl::DEPRECATED(const char *txt)
{
    if(verbose)
        yError() << txt << " has been deprecated for tripodMotionControl";
    return false;
}


//generic function that check is key1 is present in input bottle and that the result has size elements
// return true/false
bool tripodMotionControl::extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
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
        yError () << key1.c_str() << " incorrect number of entries for param" << key1.c_str();
        return false;
    }

    out=tmp;
    return true;
}


bool tripodMotionControl::alloc(int nj)
{
    _axisMap = allocAndCheck<int>(nj);
    _angleToEncoder = allocAndCheck<double>(nj);
    _encodersStamp = allocAndCheck<double>(nj);
    _limitsMax=allocAndCheck<double>(nj);
    _limitsMin=allocAndCheck<double>(nj);
    _kinematic_mj=allocAndCheck<double>(16);
    _currentLimits=allocAndCheck<double>(nj);
    checking_motiondone=allocAndCheck<bool>(nj);

    // Reserve space for data stored locally. values are initialize to 0
    _userRef_positions = allocAndCheck<double>(nj);
    _robotRef_positions = allocAndCheck<double>(nj);
    _lastUser_encoders = allocAndCheck<double>(nj);
    _lastRobot_encoders = allocAndCheck<double>(nj);
    _command_speeds = allocAndCheck<double>(nj);
    _ref_speeds = allocAndCheck<double>(nj);
    _ref_accs = allocAndCheck<double>(nj);
    _calibrated = allocAndCheck<bool>(nj);
#if 0
    _pids=allocAndCheck<Pid>(nj);
    _velocityShifts=allocAndCheck<int>(nj);
    _velocityTimeout=allocAndCheck<int>(nj);
    _kbemf=allocAndCheck<double>(nj);
    _ktau=allocAndCheck<double>(nj);
    _filterType=allocAndCheck<int>(nj);
    _last_position_move_time=allocAndCheck<double>(nj);
    _tpids=allocAndCheck<Pid>(nj);
    _cpids = allocAndCheck<Pid>(nj);
    _DEPRECATED_encoderconversionoffset = allocAndCheck<float>(nj);
    _DEPRECATED_encoderconversionfactor = allocAndCheck<float>(nj);
    _jointEncoderType = allocAndCheck<uint8_t>(nj);
    _rotorEncoderType = allocAndCheck<uint8_t>(nj);
    _jointEncoderRes = allocAndCheck<int>(nj);
    _rotorEncoderRes = allocAndCheck<int>(nj);
    _gearbox = allocAndCheck<double>(nj);
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
#endif
    return true;
}

bool tripodMotionControl::dealloc()
{
    checkAndDestroy(_axisMap);
    checkAndDestroy(_angleToEncoder);
    checkAndDestroy(_encodersStamp);
    checkAndDestroy(_limitsMax);
    checkAndDestroy(_limitsMin);
    checkAndDestroy(_kinematic_mj);
    checkAndDestroy(_currentLimits);
    checkAndDestroy(checking_motiondone);
    checkAndDestroy(_userRef_positions);
    checkAndDestroy(_robotRef_positions);
    checkAndDestroy(_lastUser_encoders);
    checkAndDestroy(_lastRobot_encoders);
    checkAndDestroy(_command_speeds);
    checkAndDestroy(_ref_speeds);
    checkAndDestroy(_ref_accs);
    checkAndDestroy(_calibrated);
#if 0
    checkAndDestroy(_pids);
    checkAndDestroy(_hasHallSensor);
    checkAndDestroy(_hasTempSensor);
    checkAndDestroy(_hasRotorEncoder);
    checkAndDestroy(_hasRotorEncoderIndex);
    checkAndDestroy(_rotorIndexOffset);
    checkAndDestroy(_motorPoles);
    checkAndDestroy(_velocityShifts);
    checkAndDestroy(_velocityTimeout);
    checkAndDestroy(_kbemf);
    checkAndDestroy(_ktau);
    checkAndDestroy(_filterType);
    checkAndDestroy(_DEPRECATED_encoderconversionoffset);
    checkAndDestroy(_DEPRECATED_encoderconversionfactor);
    checkAndDestroy(_jointEncoderRes);
    checkAndDestroy(_rotorEncoderRes);
    checkAndDestroy(_jointEncoderType);
    checkAndDestroy(_rotorEncoderType);
    checkAndDestroy(_gearbox);
    checkAndDestroy(_torqueSensorId);
    checkAndDestroy(_torqueSensorChan);
    checkAndDestroy(_maxTorque);
    checkAndDestroy(_newtonsToSensor);
    checkAndDestroy(_tpids);
    checkAndDestroy(_cpids);

    #endif
    return true;
}

tripodMotionControl::tripodMotionControl() :
    ImplementControlCalibration2<tripodMotionControl, IControlCalibration2>(this),
//     ImplementPidControl<tripodMotionControl, IPidControl>(this),
//     ImplementVelocityControl<tripodMotionControl, IVelocityControl>(this),
    ImplementEncodersTimed(this),
    ImplementPositionControl2(this),
    ImplementVelocityControl2(this),
    ImplementControlMode2(this),
    ImplementMotorEncoders(this),
//     ImplementControlLimits2(this),
    ImplementPositionDirect(this),
//     ImplementOpenLoopControl(this),
    ImplementInteractionMode(this),
//     ImplementMotor(this),
    _mutex(1)
//     ,SAFETY_THRESHOLD(2.0)
{
    verbose             = false;
    _polyDriverDevice   = NULL;
    useRemoteCB         = false;
    _angleToEncoder     = NULL;
    _njoints            = 0;
    _axisMap            = NULL;
    _encodersStamp      = NULL;
    _limitsMin          = NULL;
    _limitsMax          = NULL;
    _currentLimits      = NULL;
    _ref_accs           = NULL;
    _command_speeds     = NULL;
    _userRef_positions  = NULL;
    _robotRef_positions = NULL;
    _lastUser_encoders  = NULL;
    _lastRobot_encoders = NULL;
    _ref_speeds         = NULL;
    _kinematic_mj       = NULL;
    checking_motiondone = NULL;

    // Check status of joints
    _calibrated         = NULL;
    useRawEncoderData   = false;

#if 0
    _pids               = NULL;
    _tpids          = NULL;
    _cpids          = NULL;
    _pwmIsLimited     = false;
    _last_position_move_time = NULL;
    _kbemf            = NULL;
    _ktau             = NULL;
    _filterType       = NULL;
    _positionControlUnits = P_MACHINE_UNITS;
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
    _DEPRECATED_encoderconversionfactor = NULL;
    _DEPRECATED_encoderconversionoffset = NULL;
    _hasHallSensor      = NULL;
    _hasTempSensor      = NULL;
    _hasRotorEncoder    = NULL;
    _hasRotorEncoderIndex = NULL;
    _rotorIndexOffset = NULL;
    _motorPoles       = NULL;
    _gearbox        = 0;
#endif
}

tripodMotionControl::~tripodMotionControl()
{
    yTrace();
    dealloc();
}


bool tripodMotionControl::open(yarp::os::Searchable &config)
{
    Bottle &general = config.findGroup("GENERAL");
    if(general.isNull())
    {
        yError() << "No general group found in the config file!";
        return false;
    }

    if(general.find("verbose").asBool())
    {
        verbose = true;
        _device._subDevVerbose = verbose;
    }

    //
    //  Read Configuration params from file
    //
    if(!general.check("Joints") )
    {
        yError() << "Missing 'Joints' param in 'GENERAL' group from config file!";
        return false;
    }
    else
       _njoints = general.find("Joints").asInt();

    if(!alloc(_njoints))
    {
        yError() << "Malloc failed";
        return false;
    }

    if(!fromConfig(config))
    {
        yError() << "tripodMotionControl: Missing parameters in config file";
        return false;
    }

    // This device probably does not need implementXXX layer and we can use directly functions without the raw version.
    // For now I'll implement with conversion factor to 1, after discussion if we decide it is not needed I'll remove it.
    // Any HW dependant conversion can be done by attached device or remote device.
    // Also axis map can always be straight from 0 to n
    for(int i=0; i<_njoints; i++)
    {
        _angleToEncoder[i] = 1.0;
    }

    //  INIT ALL INTERFACES
    ImplementControlCalibration2<tripodMotionControl, IControlCalibration2>::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementEncodersTimed::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementMotorEncoders::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementPositionControl2::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
//     ImplementPidControl<tripodMotionControl, IPidControl>::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementControlMode2::initialize(_njoints, _axisMap);
//     ImplementVelocityControl<tripodMotionControl, IVelocityControl>::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementVelocityControl2::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
//
//     ImplementControlLimits2::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementPositionDirect::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
//     ImplementOpenLoopControl::initialize(_njoints, _axisMap);
    ImplementInteractionMode::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
//     ImplementMotor::initialize(_njoints, _axisMap);
    
   /*
    *  Init the IK library somehow if needed
    */


   /*
    *  Check if we have to create a remoteControlBoard
    */
   Bottle &connectionGroup = config.findGroup("CONNECTION");
   if(connectionGroup.isNull())
   {
        yTrace() << "No connection group found, waiting for 'attachAll' action to be called";
        useRemoteCB = false;
   }
   else
   {
       _polyDriverDevice = new yarp::dev::PolyDriver;
       yarp::os::Property remoteCB_config;
       remoteCB_config.fromString(connectionGroup.tail().toString().c_str());
       remoteCB_config.put("device", "remote_controlboard");

       bool ret = _polyDriverDevice->open(remoteCB_config);

       if(remoteCB_config.check("debug"))   // if debugging, fake the open and always return true here
       {
            yWarning() << "tripodMotionControl: debugging. Ignoring opening errors from RemoteControlBoard";
            ret = true;
       }

       if(!ret)
       {
            yError() << "tripodMotionControl: error while opening remoteControlBoard, check previous error messages and the config files or verify remote device is up and running";
            useRemoteCB = false;
            return false;
       }
       else
       {
            yDebug() << "Successfully connected to remote device, acquiring interfaces";
            ret = _device.attach(_polyDriverDevice);

            if(!ret && !remoteCB_config.check("debug"))
                return false;
            useRemoteCB = true;
       }
   }
    return true;
}


bool tripodMotionControl::attachAll(const PolyDriverList& p)
{
    return _device.attach(_polyDriverDevice);
}

bool tripodMotionControl::detachAll()
{
    _device.detach();
    return true;
}


#if 0
bool tripodMotionControl::parsePositionPidsGroup(Bottle& pidsGroup, Pid myPid[])
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

bool tripodMotionControl::parseTorquePidsGroup(Bottle& pidsGroup, Pid myPid[], double kbemf[], double ktau[], int filterType[])
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
    if (!extractGroup(pidsGroup, xtmp, "filterType", "filterType param", _njoints))   return false; for (j=0; j<_njoints; j++) filterType[j] = xtmp.get(j+1).asInt();
 

    //optional PWM limit
    if(_pwmIsLimited)
    {   // check for value in the file
        if (!extractGroup(pidsGroup, xtmp, "limPwm", "Limited PWD", _njoints))
        {
            yError() << "The PID parameter limPwm was requested but was not correctly set in the configuration file, please fill it.";
            return false;
        }

        fprintf(stderr,  "tripodMotionControl using LIMITED PWM!! \n");
        for (j=0; j<_njoints; j++) myPid[j].max_output = xtmp.get(j+1).asDouble();
    }

    return true;
}
#endif

bool tripodMotionControl::fromConfig(yarp::os::Searchable &config)
{
    Bottle xtmp;
    int i,j;
    Bottle general = config.findGroup("GENERAL");

    // leggere i valori da file
    if (!extractGroup(general, xtmp, "AxisMap", "a list of reordered indices for the axes", _njoints))
        return false;

    for (i = 1; i < xtmp.size(); i++)
        _axisMap[i-1] = xtmp.get(i).asInt();

/*    double tmp_A2E;
    // Encoder scales
    if (!extractGroup(general, xtmp, "Encoder", "a list of scales for the encoders", _njoints))
    {
        return false;
    }
    else
    {
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
    }

    /////// JOINTS_COUPLING
    if (_njoints<=4)
    {
        Bottle &coupling=config.findGroup("JOINTS_COUPLING");
        if (coupling.isNull())
        {
            yWarning() << "tripodMotionControl::fromConfig() detected that Group JOINTS_COUPLING is not found in configuration file";
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
        yWarning() << "tripodMotionControl::fromConfig() detected that Group LIMITS is not found in configuration file";
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
*/
    return true;
}


bool tripodMotionControl::init()
{
    yTrace();
    return true;
}

void tripodMotionControl::cleanup(void)
{
    yTrace();
}

bool tripodMotionControl::close()
{
    yTrace();

    ImplementControlMode2::uninitialize();
    ImplementEncodersTimed::uninitialize();
    ImplementMotorEncoders::uninitialize();
    ImplementPositionControl2::uninitialize();
//     ImplementVelocityControl<tripodMotionControl, IVelocityControl>::uninitialize();
    ImplementVelocityControl2::uninitialize();
//     ImplementPidControl<tripodMotionControl, IPidControl>::uninitialize();
    ImplementControlCalibration2<tripodMotionControl, IControlCalibration2>::uninitialize();
//     ImplementControlLimits2::uninitialize();
    ImplementPositionDirect::uninitialize();
//     ImplementOpenLoopControl::uninitialize();
    ImplementInteractionMode::uninitialize();

    cleanup();
    return true;
}

bool tripodMotionControl::refreshEncoders(double *times)
{
    bool ret = false;
    _mutex.wait();
    if(times !=  NULL)
        ret = _device.iJntEnc->getEncodersTimed(_lastRobot_encoders, times);
    else
        ret = _device.iJntEnc->getEncoders(_lastRobot_encoders);

    if(ret)
    {
        ret = tripod_HW2user(_lastRobot_encoders, _lastUser_encoders);
    }

    if(!ret)
    {
        // Becareful of overflowing of error messages
        yError() << "TripodMotionControl: error updating encoders";
    }
    _mutex.post();
    return ret;
}

#if 0

///////////// PID INTERFACE

bool tripodMotionControl::setPidRaw(int j, const Pid &pid)
{
    Pid  outPid;
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

//     copyPid_iCub2eo(&hwPid, &outPid);

    return true;
}

bool tripodMotionControl::setPidsRaw(const Pid *pids)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= setPidRaw(j, pids[j]);
    }
    return ret;
}

bool tripodMotionControl::setReferenceRaw(int j, double ref)
{
return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::setReferencesRaw(const double *refs)
{
    bool ret = true;
    for(int j=0, index=0; j< _njoints; j++, index++)
    {
        ret &= setReferenceRaw(j, refs[index]);
    }
    return ret;
}

bool tripodMotionControl::setErrorLimitRaw(int j, double limit)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::setErrorLimitsRaw(const double *limits)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getErrorRaw(int j, double *err)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getErrorsRaw(double *errs)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getErrorRaw(j, &errs[j]);
    }
    return ret;
}

bool tripodMotionControl::getPidRaw(int j, Pid *pid)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getPidsRaw(Pid *pids)
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

bool tripodMotionControl::getReferenceRaw(int j, double *ref)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getReferencesRaw(double *refs)
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

bool tripodMotionControl::getErrorLimitRaw(int j, double *limit)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getErrorLimitsRaw(double *limits)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::resetPidRaw(int j)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::disablePidRaw(int j)
{
    return DEPRECATED(__YFUNCTION__);
}

bool tripodMotionControl::enablePidRaw(int j)
{
    return DEPRECATED(__YFUNCTION__);
}

bool tripodMotionControl::setOffsetRaw(int j, double v)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}
#endif

////////////////////////////////////////
//    Velocity control interface raw  //
////////////////////////////////////////

bool tripodMotionControl::setVelocityModeRaw()
{
    // I guess this is too dangerous to be used with this device.
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::velocityMoveRaw(int j, double sp)
{
    // I guess this is too dangerous to be used with this device.
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::velocityMoveRaw(const double *sp)
{
    // I guess this is too dangerous to be used with this device.
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

////////////////////////////////////////
//    Calibration control interface   //
////////////////////////////////////////

bool tripodMotionControl::setCalibrationParametersRaw(int j, const CalibrationParameters& params)
{
    return _device.calib2->setCalibrationParameters(j, params);
}

bool tripodMotionControl::calibrate2Raw(int j, unsigned int type, double p1, double p2, double p3)
{
    return _device.calib2->calibrate2(j, type, p1, p2, p3);
}

bool tripodMotionControl::doneRaw(int axis)
{
    return _device.calib2->done(axis);
}

////////////////////////////////////////
//     Position control interface     //
////////////////////////////////////////
bool tripodMotionControl::getAxes(int *ax)
{
    *ax=_njoints;
    return true;
}

bool tripodMotionControl::setPositionModeRaw()
{
    return _device.pos->setPositionMode();
}

bool tripodMotionControl::positionMoveRaw(int j, double ref)
{
    // calling IK library before propagate the command to HW
    _mutex.wait();
    _userRef_positions[j] = ref;
    if(!tripod_user2HW(_userRef_positions, _robotRef_positions))
    {
        yError() << "Requested position is not reachable";
        return false;
    }
    _mutex.post();

    // all joints may need to move in order to achieve the new requested position
    // even if only one user virtual joint has got new reference
    return _device.pos->positionMove(_robotRef_positions);
}

bool tripodMotionControl::positionMoveRaw(const double *refs)
{
    _mutex.wait();
    for(int i=0, index=0; i< _njoints; i++, index++)
    {
        _userRef_positions[i] = refs[i];
    }

    if(!tripod_user2HW(_userRef_positions, _robotRef_positions))
    {
        yError() << "Requested position is not reachable";
        return false;
    }
    _mutex.post();

    // all joints may need to move in order to achieve the new requested position
    // even if only one user virtual joint has got new reference
    return _device.pos->positionMove(_robotRef_positions);
}

bool tripodMotionControl::relativeMoveRaw(int j, double delta)
{
    _mutex.wait();
    // TODO does it make any sense to add values likt this?? Those could be angle or quaternion or whatever!!!!
    // How to sum those up depends on the chosen representation!! Verify and maybe add a parameter in config files
    // to specify the type and choose correct sum procedure
    _userRef_positions[j] += delta;

    // calling IK library before propagate the command to HW
    if(!tripod_user2HW(_userRef_positions, _robotRef_positions))
    {
        yError() << "Requested position is not reachable";
        return false;
    }
    _mutex.post();

    // all joints may need to move in order to achieve the new requested position
    // even if only one user virtual joint has got new reference
    return _device.pos->positionMove(_robotRef_positions);
}

bool tripodMotionControl::relativeMoveRaw(const double *deltas)
{
    // TODO does it make any sense to add values likt this?? Those could be angle or quaternion or whatever!!!!
    // How to sum those up depends on the chosen representation!! Verify and maybe add a parameter in config files
    // to specify the type and choose correct sum procedure
    _mutex.wait();
    for(int i=0; i<_njoints; i++)
        _userRef_positions[i] += deltas[i];

    // calling IK library before propagate the command to HW
    if(!tripod_user2HW(_userRef_positions, _robotRef_positions))
    {
        yError() << "Requested position is not reachable";
        return false;
    }
    _mutex.post();

    // all joints may need to move in order to achieve the new requested position
    // even if only one user virtual joint has got new reference
    return _device.pos->positionMove(_robotRef_positions);
}

bool tripodMotionControl::checkMotionDoneRaw(int j, bool *flag)
{
    return _device.pos->checkMotionDone(j, flag);
}

bool tripodMotionControl::checkMotionDoneRaw(bool *flag)
{
    return _device.pos->checkMotionDone(flag);
}

bool tripodMotionControl::setRefSpeedRaw(int j, double sp)
{
    return _device.pos->setRefSpeed(j, sp);
}

bool tripodMotionControl::setRefSpeedsRaw(const double *spds)
{
    return _device.pos->setRefSpeeds(spds);
}

bool tripodMotionControl::setRefAccelerationRaw(int j, double acc)
{
    return _device.pos->setRefAcceleration(j, acc);
}

bool tripodMotionControl::setRefAccelerationsRaw(const double *accs)
{
    return _device.pos->setRefAccelerations(accs);
}

bool tripodMotionControl::getRefSpeedRaw(int j, double *spd)
{
    return _device.pos->getRefSpeed(j, spd);
}

bool tripodMotionControl::getRefSpeedsRaw(double *spds)
{
    return _device.pos->getRefSpeeds(spds);
}

bool tripodMotionControl::getRefAccelerationRaw(int j, double *acc)
{
    return _device.pos->getRefAcceleration(j, acc);
}

bool tripodMotionControl::getRefAccelerationsRaw(double *accs)
{
    return _device.pos->getRefAccelerations(accs);
}

bool tripodMotionControl::stopRaw(int j)
{
    return _device.pos->stop(j);
}

bool tripodMotionControl::stopRaw()
{
    return _device.pos->stop();
}
///////////// END Position Control INTERFACE  //////////////////

////////////////////////////////////////
//     Position control2 interface    //
////////////////////////////////////////

bool tripodMotionControl::positionMoveRaw(const int n_joint, const int *joints, const double *refs)
{
    _mutex.wait();
    for(int i=0, index=0; i< n_joint; i++, index++)
    {
        _userRef_positions[joints[i]] = refs[i];
    }

    if(!tripod_user2HW(_userRef_positions, _robotRef_positions))
    {
        yError() << "Requested position is not reachable";
        return false;
    }
    _mutex.post();

    // all joints may need to move in order to achieve the new requested position
    // even if only one user virtual joint has got new reference
    return _device.pos->positionMove(_robotRef_positions);
}

bool tripodMotionControl::relativeMoveRaw(const int n_joint, const int *joints, const double *deltas)
{
    _mutex.wait();
    for(int i=0, index=0; i< n_joint; i++, index++)
    {
        _userRef_positions[joints[i]] += deltas[i];
    }

    if(!tripod_user2HW(_userRef_positions, _robotRef_positions))
    {
        yError() << "Requested position is not reachable";
        return false;
    }
    _mutex.post();

    // all joints may need to move in order to achieve the new requested position
    // even if only one user virtual joint has got new reference
    return _device.pos->positionMove(_robotRef_positions);
}

bool tripodMotionControl::checkMotionDoneRaw(const int n_joint, const int *joints, bool *flag)
{
    return _device.pos2->checkMotionDone(n_joint, joints, flag);
}

bool tripodMotionControl::setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds)
{
    return _device.pos2->setRefSpeeds(n_joint, joints, spds);
}

bool tripodMotionControl::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    return _device.pos2->setRefAccelerations(n_joint, joints, accs);
}

bool tripodMotionControl::getRefSpeedsRaw(const int n_joint, const int *joints, double *spds)
{
    return _device.pos2->getRefSpeeds(n_joint, joints, spds);
}

bool tripodMotionControl::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    return _device.pos2->getRefAccelerations(n_joint, joints, accs);
}

bool tripodMotionControl::stopRaw(const int n_joint, const int *joints)
{
    return _device.pos2->stop(n_joint, joints);
}
///////////// END Position Control INTERFACE  //////////////////

// ControlMode
bool tripodMotionControl::setPositionModeRaw(int j)
{
    return DEPRECATED("setPositionModeRaw");
}

bool tripodMotionControl::setVelocityModeRaw(int j)
{
    return DEPRECATED("setVelocityModeRaw");
}

bool tripodMotionControl::setTorqueModeRaw(int j)
{
    return DEPRECATED("setTorqueModeRaw");
}

bool tripodMotionControl::setImpedancePositionModeRaw(int j)
{
    return DEPRECATED("setImpedancePositionModeRaw");
}

bool tripodMotionControl::setImpedanceVelocityModeRaw(int j)
{
    return DEPRECATED("setImpedanceVelocityModeRaw");
}

bool tripodMotionControl::setOpenLoopModeRaw(int j)
{
    return DEPRECATED("setOpenLoopModeRaw");
}

bool tripodMotionControl::getControlModeRaw(int j, int *v)
{
    return _device.iMode2->getControlMode(j,v);
}

// IControl Mode 2
bool tripodMotionControl::getControlModesRaw(int* v)
{
    return _device.iMode2->getControlModes(v);
}

bool tripodMotionControl::getControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    return _device.iMode2->getControlModes(n_joint, joints, modes);
}

bool tripodMotionControl::setControlModeRaw(const int j, const int mode)
{
    return _device.iMode2->setControlMode(j, mode);
}

bool tripodMotionControl::setControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    return _device.iMode2->setControlModes(n_joint, joints, modes);
}

bool tripodMotionControl::setControlModesRaw(int *modes)
{
    return _device.iMode2->setControlModes(modes);
}

//////////////////////// BEGIN EncoderInterface

bool tripodMotionControl::setEncoderRaw(int j, double val)
{
    return NOT_YET_IMPLEMENTED("setEncoder");
}

bool tripodMotionControl::setEncodersRaw(const double *vals)
{
    return NOT_YET_IMPLEMENTED("setEncoders");
}

bool tripodMotionControl::resetEncoderRaw(int j)
{
    return NOT_YET_IMPLEMENTED("resetEncoder");
}

bool tripodMotionControl::resetEncodersRaw()
{
    return NOT_YET_IMPLEMENTED("resetEncoders");
}

bool tripodMotionControl::getEncoderRaw(int j, double *value)
{
    bool ret = refreshEncoders(NULL);
   *value = _lastUser_encoders[j];
   return ret;
}

bool tripodMotionControl::getEncodersRaw(double *encs)
{
    bool ret = refreshEncoders(NULL);
    memcpy(encs, _lastUser_encoders, _njoints*sizeof(double));
    return ret;
}

bool tripodMotionControl::getEncoderSpeedRaw(int j, double *sp)
{
    return _device.iJntEnc->getEncoderSpeed(j, sp);  // TODO does it make sense??
}

bool tripodMotionControl::getEncoderSpeedsRaw(double *spds)
{
    return _device.iJntEnc->getEncoderSpeeds(spds);  // TODO does it make sense??
}

bool tripodMotionControl::getEncoderAccelerationRaw(int j, double *acc)
{
    return _device.iJntEnc->getEncoderAcceleration(j, acc);  // TODO does it make sense??
}

bool tripodMotionControl::getEncoderAccelerationsRaw(double *accs)
{
    return _device.iJntEnc->getEncoderAccelerations(accs);  // TODO does it make sense??
}

///////////////////////// END Encoder Interface

bool tripodMotionControl::getEncodersTimedRaw(double *encs, double *stamps)
{
    bool ret = refreshEncoders(stamps);
    memcpy(encs, _lastUser_encoders, _njoints*sizeof(double));
    return ret;
}

bool tripodMotionControl::getEncoderTimedRaw(int j, double *value, double *stamp)
{
    double tmp_stamps[_njoints];
    bool ret = refreshEncoders(tmp_stamps);
    *value = _lastUser_encoders[j];
    *stamp = tmp_stamps[j];
    return ret;
}


//////////////////////// BEGIN motor EncoderInterface

bool tripodMotionControl::getNumberOfMotorEncodersRaw(int* num)
{
    *num=_njoints;  // TODO probably not true
    return true;
}

bool tripodMotionControl::setMotorEncoderRaw(int m, const double val)
{
    return NOT_YET_IMPLEMENTED("setMotorEncoder");
}

bool tripodMotionControl::setMotorEncodersRaw(const double *vals)
{
    return NOT_YET_IMPLEMENTED("setMotorEncoders");
}

bool tripodMotionControl::setMotorEncoderCountsPerRevolutionRaw(int m, const double cpr)
{
    return NOT_YET_IMPLEMENTED("setMotorEncoderCountsPerRevolutionRaw");
}

bool tripodMotionControl::getMotorEncoderCountsPerRevolutionRaw(int m, double *cpr)
{
    return NOT_YET_IMPLEMENTED("getMotorEncoderCountsPerRevolutionRaw");
}

bool tripodMotionControl::resetMotorEncoderRaw(int mj)
{
    return NOT_YET_IMPLEMENTED("resetMotorEncoder");
}

bool tripodMotionControl::resetMotorEncodersRaw()
{
    return NOT_YET_IMPLEMENTED("reseMotortEncoders");
}

bool tripodMotionControl::getMotorEncoderRaw(int m, double *value)
{
    return _device.iMotEnc->getMotorEncoder(m, value);
}

bool tripodMotionControl::getMotorEncodersRaw(double *encs)
{
    return _device.iMotEnc->getMotorEncoders(encs);
}

bool tripodMotionControl::getMotorEncoderSpeedRaw(int m, double *sp)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getMotorEncoderSpeedsRaw(double *spds)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getMotorEncoderAccelerationRaw(int m, double *acc)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getMotorEncoderAccelerationsRaw(double *accs)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getMotorEncodersTimedRaw(double *encs, double *stamps)
{
    return _device.iMotEnc->getMotorEncodersTimed(encs, stamps);
}

bool tripodMotionControl::getMotorEncoderTimedRaw(int m, double *encs, double *stamp)
{
    return _device.iMotEnc->getMotorEncoderTimed(m, encs, stamp);
}
///////////////////////// END Motor Encoder Interface

#if 0
////// Amplifier interface

bool tripodMotionControl::enableAmpRaw(int j)
{
    return DEPRECATED("enableAmpRaw");
}

bool tripodMotionControl::disableAmpRaw(int j)
{
    return DEPRECATED("disableAmpRaw");
}

bool tripodMotionControl::getCurrentRaw(int j, double *value)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}


// Iamplifier control -- not in the list of interfaces implemented by this device right now
bool tripodMotionControl::getCurrentsRaw(double *vals)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getCurrentRaw(j, &vals[j]);
    }
    return ret;
}

bool tripodMotionControl::setMaxCurrentRaw(int j, double val)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getMaxCurrentRaw(int j, double *val)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getAmpStatusRaw(int *sts)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

// Limit interface
bool tripodMotionControl::setLimitsRaw(int j, double min, double max)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getLimitsRaw(int j, double *min, double *max)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getGearboxRatioRaw(int j, double *gearbox)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getTorqueControlFilterType(int j, int& type)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getRotorEncoderResolutionRaw(int j, double &rotres)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getJointEncoderResolutionRaw(int j, double &jntres)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getJointEncoderTypeRaw(int j, int &type)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getRotorEncoderTypeRaw(int j, int &type)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getKinematicMJRaw(int j, double &rotres)
{
    yError("getKinematicMJRaw not yet  implemented");
    return false;
}

bool tripodMotionControl::getHasTempSensorsRaw(int j, int& ret)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getHasHallSensorRaw(int j, int& ret)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getHasRotorEncoderRaw(int j, int& ret)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getHasRotorEncoderIndexRaw(int j, int& ret)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getMotorPolesRaw(int j, int& poles)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getRotorIndexOffsetRaw(int j, double& rotorOffset)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getCurrentPidRaw(int j, Pid *pid)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

// IControlLimits2
bool tripodMotionControl::setVelLimitsRaw(int axis, double min, double max)
{
    return NOT_YET_IMPLEMENTED("setVelLimitsRaw");
}

bool tripodMotionControl::getVelLimitsRaw(int axis, double *min, double *max)
{
    return NOT_YET_IMPLEMENTED("getVelLimitsRaw");
}

bool tripodMotionControl::getBemfParamRaw(int j, double *bemf)
{
    return DEPRECATED("getBemfParamRaw");
}

bool tripodMotionControl::setBemfParamRaw(int j, double bemf)
{
    return DEPRECATED("setBemfParamRaw");
}

bool tripodMotionControl::setTorqueErrorLimitRaw(int j, double limit)
{
    return NOT_YET_IMPLEMENTED("setTorqueErrorLimitRaw");
}

bool tripodMotionControl::setTorqueErrorLimitsRaw(const double *limits)
{
    return NOT_YET_IMPLEMENTED("setTorqueErrorLimitsRaw");
}

bool tripodMotionControl::getTorquePidOutputRaw(int j, double *out)
{
    return NOT_YET_IMPLEMENTED("getTorquePidOutputRaw");
}

bool tripodMotionControl::getTorquePidOutputsRaw(double *outs)
{
    return NOT_YET_IMPLEMENTED("getTorquePidOutputsRaw");
}

bool tripodMotionControl::getTorqueErrorLimitRaw(int j, double *limit)
{
    return NOT_YET_IMPLEMENTED("getTorqueErrorLimitRaw");
}

bool tripodMotionControl::getTorqueErrorLimitsRaw(double *limits)
{
    return NOT_YET_IMPLEMENTED("getTorqueErrorLimitsRaw");
}

bool tripodMotionControl::resetTorquePidRaw(int j)
{
    return NOT_YET_IMPLEMENTED("resetTorquePidRaw");
}

bool tripodMotionControl::disableTorquePidRaw(int j)
{
    return NOT_YET_IMPLEMENTED("disableTorquePidRaw");
}

bool tripodMotionControl::enableTorquePidRaw(int j)
{
    return NOT_YET_IMPLEMENTED("enableTorquePidRaw");
}

bool tripodMotionControl::setTorqueOffsetRaw(int j, double v)
{
    return NOT_YET_IMPLEMENTED("setTorqueOffsetRaw");
}

bool tripodMotionControl::getMotorTorqueParamsRaw(int j, MotorTorqueParameters *params)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::setMotorTorqueParamsRaw(int j, const MotorTorqueParameters params)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}
#endif

// IVelocityControl2
bool tripodMotionControl::velocityMoveRaw(const int n_joint, const int *joints, const double *spds)
{
    // I guess this is too dangerous to be used with this device.
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::setVelPidRaw(int j, const Pid &pid)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::setVelPidsRaw(const Pid *pids)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getVelPidRaw(int j, Pid *pid)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getVelPidsRaw(Pid *pids)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

// PositionDirect Interface
bool tripodMotionControl::setPositionDirectModeRaw()
{
    return DEPRECATED("setPositionDirectModeRaw");
}

bool tripodMotionControl::setPositionRaw(int j, double ref)
{
    // calling IK library before propagate the command to HW
    _mutex.wait();
    _userRef_positions[j] = ref;
    if(!tripod_user2HW(_userRef_positions, _robotRef_positions))
    {
        yError() << "Requested position is not reachable";
        return false;
    }
    _mutex.post();

    // all joints may need to move in order to achieve the new requested position
    // even if only one user virtual joint has got new reference
    return _device.posDir->setPositions(_robotRef_positions);
}

bool tripodMotionControl::setPositionsRaw(const int n_joint, const int *joints, double *refs)
{
    // calling IK library before propagate the command to HW
    _mutex.wait();
    for(int i=0; i<n_joint; i++)
        _userRef_positions[joints[i]] = refs[i];

    if(!tripod_user2HW(_userRef_positions, _robotRef_positions))
    {
        yError() << "Requested position is not reachable";
        return false;
    }
    _mutex.post();

    // all joints may need to move in order to achieve the new requested position
    // even if only one user virtual joint has got new reference
    return _device.posDir->setPositions(_robotRef_positions);
}

bool tripodMotionControl::setPositionsRaw(const double *refs)
{
    // calling IK library before propagate the command to HW
    _mutex.wait();
    memcpy(_userRef_positions, refs, _njoints*sizeof(double));

    if(!tripod_user2HW(_userRef_positions, _robotRef_positions))
    {
        yError() << "Requested position is not reachable";
        return false;
    }
    _mutex.post();

    // all joints may need to move in order to achieve the new requested position
    // even if only one user virtual joint has got new reference
    return _device.posDir->setPositions(_robotRef_positions);
}

// InteractionMode
bool tripodMotionControl::getInteractionModeRaw(int j, yarp::dev::InteractionModeEnum* mode)
{
    return _device.iInteract->getInteractionMode(j, mode);
}

bool tripodMotionControl::getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    return _device.iInteract->getInteractionModes(n_joints, joints, modes);
}

bool tripodMotionControl::getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    return _device.iInteract->getInteractionModes(modes);
}

bool tripodMotionControl::setInteractionModeRaw(int j, yarp::dev::InteractionModeEnum mode)
{
    return _device.iInteract->setInteractionMode(j, mode);
}

bool tripodMotionControl::setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    return _device.iInteract->setInteractionModes(n_joints, joints, modes);
}

bool tripodMotionControl::setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    return _device.iInteract->setInteractionModes(modes);
}

#if 0
//
// OPENLOOP interface
//
bool tripodMotionControl::setOpenLoopModeRaw()
{
    return DEPRECATED("setOpenLoopModeRaw");
}

bool tripodMotionControl::setRefOutputRaw(int j, double v)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::setRefOutputsRaw(const double *v)
{
    bool ret = true;
    for(int j=0; j<_njoints; j++)
    {
        ret = ret && setRefOutputRaw(j, v[j]);
    }
    return ret;
}

bool tripodMotionControl::getRefOutputRaw(int j, double *out)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getRefOutputsRaw(double *outs)
{
    bool ret = true;
    for(int j=0; j<_njoints; j++)
    {
        ret = ret && getRefOutputRaw(j, &outs[j]);
    }
    return ret;
}

bool tripodMotionControl::getOutputRaw(int j, double *out)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getOutputsRaw(double *outs)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getOutputRaw(j, &outs[j]);
    }
    return ret;
}

bool tripodMotionControl::getNumberOfMotorsRaw(int* num)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool tripodMotionControl::getTemperatureRaw(int m, double* val)
{
    return NOT_YET_IMPLEMENTED("getTemperatureRaw");
}

bool tripodMotionControl::getTemperaturesRaw(double *vals)
{
    return NOT_YET_IMPLEMENTED("getTemperaturesRaw");
}

bool tripodMotionControl::getTemperatureLimitRaw(int m, double *temp)
{
    return NOT_YET_IMPLEMENTED("getTemperatureLimitRaw");
}

bool tripodMotionControl::setTemperatureLimitRaw(int m, const double temp)
{
    return NOT_YET_IMPLEMENTED("setTemperatureLimitRaw");
}

bool tripodMotionControl::getMotorOutputLimitRaw(int m, double *limit)
{
    return NOT_YET_IMPLEMENTED("getMotorOutputLimitRaw");
}

bool tripodMotionControl::setMotorOutputLimitRaw(int m, const double limit)
{
    return NOT_YET_IMPLEMENTED("setMotorOutputLimitRaw");
}
#endif

// eof
