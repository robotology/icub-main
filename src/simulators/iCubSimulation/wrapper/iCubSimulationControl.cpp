// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Vadim Tikhanoff, Paul Fitzpatrick, Giorgio Metta
* email:   vadim.tikhanoff@iit.it, paulfitz@alum.mit.edu, giorgio.metta@iit.it
* website: www.robotcub.org
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#ifdef _MSC_VER
#pragma warning(disable:4355)  //for VC++, no precision loss complaints
#endif
/// general purpose stuff.
#include <yarp/os/Time.h>
#include <yarp/os/ConstString.h>

#include <math.h>
#include <string>

///specific to this device driver.
#include "iCubSimulationControl.h"
#include "OdeInit.h"
#include <yarp/dev/ControlBoardInterfacesImpl.inl>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace yarp::dev;

static bool NOT_YET_IMPLEMENTED(const char *txt)
{
    yError("%s not yet implemented for iCubSimulationControl\n", txt);

    return false;
}

static inline bool DEPRECATED(const char *txt)
{
    yError() << txt << " has been deprecated for embObjMotionControl";
    return true;
}

//////////////////////////////////

iCubSimulationControl::iCubSimulationControl() : 
    //RateThread(10),
    ImplementPositionControl2(this),
    ImplementVelocityControl2(this),
    ImplementPidControl(this),
    ImplementEncodersTimed(this),
    ImplementTorqueControl(this),
    ImplementControlMode2(this),
    ImplementControlCalibration<iCubSimulationControl, IControlCalibration>(this),
    ImplementAmplifierControl<iCubSimulationControl, IAmplifierControl>(this),
    ImplementControlLimits2(this),
    ImplementInteractionMode(this),
    ImplementPositionDirect(this),
    ImplementMotorEncoders(this),
    ImplementRemoteVariables(this),
    ImplementAxisInfo(this),
    ImplementMotor(this),
    ImplementPWMControl(this),
    ImplementCurrentControl(this),
    _done(0),
    _mutex(1)
{
    _opened = false;
    manager = NULL;
}


iCubSimulationControl::~iCubSimulationControl ()
{
    OdeInit& odeinit = OdeInit::get();
    odeinit.mutex.wait();
    odeinit.removeSimulationControl(partSelec);
    odeinit.mutex.post();
}

bool iCubSimulationControl::open(yarp::os::Searchable& config) {

    Searchable& p = config;

    if (!p.check("GENERAL","section for general motor control parameters")) {
        yError("Cannot understand configuration parameters");
        return false;
    }

    int TypeArm = p.findGroup("GENERAL").check("Type",Value(1),
                                          "what did the user select?").asInt();

    int numTOTjoints = p.findGroup("GENERAL").check("TotalJoints",Value(1),
                                          "Number of total joints").asInt();

    double velocity = p.findGroup("GENERAL").check("Vel",Value(1),
                                          "Default velocity").asDouble();
    _mutex.wait();
    partSelec = TypeArm;

    njoints = numTOTjoints;

    vel = velocity;

    angleToEncoder = allocAndCheck<double>(njoints);
    zeros = allocAndCheck<double>(njoints);
    newtonsToSensor = allocAndCheck<double>(njoints);
    ampsToSensor = allocAndCheck<double>(njoints);
    dutycycleToPwm = allocAndCheck<double>(njoints);

    controlMode = allocAndCheck<int>(njoints);
    interactionMode = allocAndCheck<int>(njoints);
    maxCurrent = allocAndCheck<double>(njoints);
    
    limitsMin = allocAndCheck<double>(njoints);
    limitsMax = allocAndCheck<double>(njoints);
    velLimitsMin = allocAndCheck<double>(njoints);
    velLimitsMax = allocAndCheck<double>(njoints);
    torqueLimits = allocAndCheck<double>(njoints);
    
    refSpeed = allocAndCheck<double>(njoints);
    refAccel = allocAndCheck<double>(njoints);
    controlP = allocAndCheck<double>(njoints); 

    axisMap = allocAndCheck<int>(njoints);
//  jointNames = new ConstString[njoints];

    current_jnt_pos = allocAndCheck<double>(njoints);
    current_jnt_vel = allocAndCheck<double>(njoints);
    current_jnt_torques = allocAndCheck<double>(njoints);
    current_mot_pos = allocAndCheck<double>(njoints);
    current_mot_vel = allocAndCheck<double>(njoints);
    current_mot_torques = allocAndCheck<double>(njoints);
    pwm = allocAndCheck<double>(njoints);
    pwm_ref = allocAndCheck<double>(njoints);
    current_ampere = allocAndCheck<double>(njoints);
    current_ampere_ref = allocAndCheck<double>(njoints);
    next_pos = allocAndCheck<double>(njoints);
    next_vel = allocAndCheck<double>(njoints);
    next_torques = allocAndCheck<double>(njoints);
    inputs = allocAndCheck<int>(njoints);
    vels = allocAndCheck<double>(njoints);
    ref_command_positions = allocAndCheck<double>(njoints);
    ref_positions = allocAndCheck<double>(njoints);
    ref_command_speeds = allocAndCheck<double>(njoints);
    ref_speeds = allocAndCheck<double>(njoints);
    ref_torques = allocAndCheck<double>(njoints);

    error_tol = allocAndCheck<double>(njoints);
    position_pid = allocAndCheck <Pid>(njoints);
    torque_pid = allocAndCheck  <Pid>(njoints);
    current_pid = allocAndCheck  <Pid>(njoints);
    motor_torque_params = allocAndCheck <MotorTorqueParameters>(njoints);
    rotToEncoder = allocAndCheck<double>(njoints);
    gearbox = allocAndCheck<double>(njoints);
    hasHallSensor = allocAndCheck<bool>(njoints);
    hasTempSensor = allocAndCheck<bool>(njoints);
    hasRotorEncoder = allocAndCheck<bool>(njoints);
    rotorIndexOffset = allocAndCheck<int>(njoints);
    motorPoles = allocAndCheck<int>(njoints);

//  joint_dev = new DeviceTag[njoints];

    motor_on = allocAndCheck<bool>(njoints);
    for(int axis = 0;axis<njoints;axis++)
        motor_on[axis] = false;

    /////////////////////////
    /*   GENERAL           */
    /////////////////////////

    Bottle& xtmp = p.findGroup("GENERAL").findGroup("AxisMap","a list of reordered indices for the axes");
    
    if (xtmp.size() != njoints+1) {
        yError("AxisMap does not have the right number of entries\n");
        return false;
    }
    for (int i = 1; i < xtmp.size(); i++) axisMap[i-1] = xtmp.get(i).asInt();

    xtmp = p.findGroup("GENERAL").findGroup("Encoder","a list of scales for the encoders");
    if (xtmp.size() != njoints+1) {
        yError("Encoder does not have the right number of entries\n");
        return false;
    }
    for (int i = 1; i < xtmp.size(); i++) angleToEncoder[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("GENERAL").findGroup("fullscalePWM", "a list of scales for the fullscalePWM param");
    if (xtmp.size() != njoints + 1) {
        yError("fullscalePWM does not have the right number of entries\n");
        return false;
    }
    for (int i = 1; i < xtmp.size(); i++) dutycycleToPwm[i - 1] = xtmp.get(i).asDouble()/100.0;

    xtmp = p.findGroup("GENERAL").findGroup("ampsToSensor", "a list of scales for the ampsToSensor param");
    if (xtmp.size() != njoints + 1) {
        yError("ampsToSensor does not have the right number of entries\n");
        return false;
    }
    for (int i = 1; i < xtmp.size(); i++) ampsToSensor[i - 1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("GENERAL").findGroup("Zeros","a list of offsets for the zero point");
    if (xtmp.size() != njoints+1) {
        yError("Zeros does not have the right number of entries\n");
        return false;
    }
    for (int i = 1; i < xtmp.size(); i++) zeros[i-1] = xtmp.get(i).asDouble();


    int mj_size = p.findGroup("GENERAL").check("Kinematic_mj_size",Value(0),"Default velocity").asInt();
    if (mj_size>0)
    {
         xtmp = p.findGroup("GENERAL").findGroup("Kinematic_mj");
         if (xtmp.size() != mj_size*mj_size) {
             yError("Invalid Kinematic_mj size\n");
         }
         kinematic_mj.resize(mj_size,mj_size);
         for (int c = 0; c < mj_size; c++)
            for (int r = 0; r < mj_size; r++)
            {
                int e=r+c*mj_size+1;
                kinematic_mj[r][c] = xtmp.get(e).asDouble();
            }
    }


    //fake position pid
    for (int i = 1; i < njoints + 1; i++) position_pid[i - 1].max_output = 100;

    //torque sensor
    for (int i = 1; i < njoints+1; i++) newtonsToSensor[i-1] = 1.0;

    ////////////////////////
    /*   LIMITS           */
    ////////////////////////
    xtmp = p.findGroup("LIMITS").findGroup("jntPosMax","access the joint limits max");
    if(xtmp.size() != njoints+1)
        {
            yError("Not enough max joint limits\n");
            return false;
        }
    for( int i =1;i<xtmp.size();i++ ) 
        limitsMax[i-1] = xtmp.get(i).asDouble()*angleToEncoder[i-1];
    
    //max velocity
    for (int i = 1; i < xtmp.size(); i++)
    {
        velLimitsMin[i - 1] = 0;
        velLimitsMax[i - 1] = 100 * angleToEncoder[i - 1];
    }
        
    xtmp = p.findGroup("LIMITS").findGroup("jntPosMin","access the joint limits min");
    if(xtmp.size() != njoints+1)
        {
            yError("Not enough min joint limits\n");
            return false;
        }
    for(int i =1;i<xtmp.size();i++)
        limitsMin[i-1] = xtmp.get(i).asDouble()*angleToEncoder[i-1];

    xtmp = p.findGroup("LIMITS").findGroup("error_tol","error tolerance during tracking");
    if(xtmp.size() != njoints+1)
        {
            yError("Not enough error_tol\n");
            return false;
        }
    for(int i=1;i<xtmp.size();i++)
        error_tol[i-1] = xtmp.get(i).asDouble()*angleToEncoder[i-1];


    for(int axis =0;axis<njoints;axis++)
    {
        current_jnt_pos[axis] = 0.0;
        current_mot_pos[axis] = 0.0;
        ErrorPos[axis] = 0.0;
        double v = 0.0;     
        if (v<limitsMin[axis]) v = limitsMin[axis];
        if (v>limitsMax[axis]) v = limitsMax[axis];
        //else v = next_pos[i];
        next_pos[axis] = M_PI*(v/angleToEncoder[axis])/180;//removed (v/angleToEncoder[i]-1)/180
        next_vel[axis] = 10.0*angleToEncoder[axis];
        next_torques[axis] = 0.0;
        ref_speeds[axis] = 10.0*angleToEncoder[axis];
        ref_command_speeds[axis] = 0.0;
        input = 0;
        inputs[axis] = 0;
        vels[axis] = 1;
        maxCurrent[axis] = 1000;
        controlMode[axis] = MODE_POSITION;
        interactionMode[axis] = VOCAB_IM_STIFF;
   }

    ImplementPositionControl2::initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementVelocityControl2::initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementPidControl::initialize(njoints, axisMap, angleToEncoder, zeros,newtonsToSensor,ampsToSensor);
    ImplementEncodersTimed::initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementMotorEncoders::initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementControlCalibration<iCubSimulationControl, IControlCalibration>::
        initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementAmplifierControl<iCubSimulationControl, IAmplifierControl>::
        initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementControlLimits2::initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementTorqueControl::initialize(njoints, axisMap, angleToEncoder, zeros, newtonsToSensor);
    ImplementControlMode2::initialize(njoints, axisMap);
    ImplementInteractionMode::initialize(njoints, axisMap);
    ImplementPositionDirect::initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementRemoteVariables::initialize(njoints, axisMap);
    ImplementAxisInfo::initialize(njoints, axisMap);
    ImplementMotor::initialize(njoints, axisMap);
    ImplementCurrentControl::initialize(njoints, axisMap, ampsToSensor);
    ImplementPWMControl::initialize(njoints, axisMap, dutycycleToPwm);

    if (!p.check("joint_device")) {
        yError("Need a device to access the joints\n");
        return false;
    }
    if (!joints.open(p.find("joint_device").asString().c_str())) {
        yError("Failed to create a device to access the joints\n");
        return false;
    }
    manager = NULL;
    joints.view(manager);
    if (manager==NULL) {
        yError("Wrong type for device to access the joints\n");
        return false;
    }
    
    OdeInit& odeinit = OdeInit::get();
    odeinit.mutex.wait();
    odeinit.setSimulationControl(this, partSelec);
    odeinit.mutex.post();
    //RateThread::start();
    //_done.wait ();
    _mutex.post();
    _opened = true;

    verbosity = odeinit.verbosity;
    return true;
}

bool iCubSimulationControl::close (void)
{
    if (_opened) {

        //if (RateThread::isRunning())
        //    {
        //    }
        
        //RateThread::stop();/// stops the thread first (joins too).
        
        ImplementPositionControl2::uninitialize();
        ImplementVelocityControl2::uninitialize();
        ImplementTorqueControl::uninitialize();
        ImplementPidControl::uninitialize();
        ImplementEncodersTimed::uninitialize();
        ImplementMotorEncoders::uninitialize();
        ImplementControlCalibration<iCubSimulationControl, IControlCalibration>::uninitialize();
        ImplementAmplifierControl<iCubSimulationControl, IAmplifierControl>::uninitialize();
        ImplementControlLimits2::uninitialize();
        ImplementControlMode2::uninitialize();
        ImplementInteractionMode::uninitialize();
        ImplementPositionDirect::uninitialize();
        ImplementRemoteVariables::uninitialize();
        ImplementAxisInfo::uninitialize();
        ImplementMotor::uninitialize();
        ImplementCurrentControl::uninitialize();
        ImplementPWMControl::uninitialize();
    }

    checkAndDestroy<double>(current_jnt_pos);
    checkAndDestroy<double>(current_jnt_torques);
    checkAndDestroy<double>(current_mot_pos);
    checkAndDestroy<double>(current_mot_torques);
    checkAndDestroy<double>(pwm);
    checkAndDestroy<double>(pwm_ref);
    checkAndDestroy<double>(current_ampere);
    checkAndDestroy<double>(current_ampere_ref);
    checkAndDestroy<double>(current_jnt_vel);
    checkAndDestroy<double>(current_mot_vel);
    checkAndDestroy<double>(next_pos);
    checkAndDestroy<double>(next_vel);
    checkAndDestroy<double>(next_torques);
   // delete[] joint_dev;
    checkAndDestroy<double>(ref_command_positions);
    checkAndDestroy<double>(ref_positions);
    checkAndDestroy<double>(ref_command_speeds);
    checkAndDestroy<double>(ref_speeds);
    checkAndDestroy<double>(ref_torques);
    checkAndDestroy<double>(angleToEncoder);
    checkAndDestroy<double>(zeros);
    checkAndDestroy<double>(newtonsToSensor);
    checkAndDestroy<int>(controlMode);
    checkAndDestroy<int>(interactionMode);
    checkAndDestroy<double>(limitsMin);
    checkAndDestroy<double>(limitsMax);
    checkAndDestroy<double>(velLimitsMin);
    checkAndDestroy<double>(velLimitsMax);
    checkAndDestroy<int>(axisMap);
    checkAndDestroy<int>(inputs);
    checkAndDestroy<double>(vels);
    checkAndDestroy<double>(torqueLimits);
    checkAndDestroy<double>(maxCurrent);
    
    checkAndDestroy<double>(refSpeed);
    checkAndDestroy<double>(refAccel);
    checkAndDestroy<double>(controlP);   

    checkAndDestroy<double>(error_tol);
    checkAndDestroy<bool>(motor_on);
    checkAndDestroy<Pid>(position_pid);
    checkAndDestroy<Pid>(torque_pid);
    checkAndDestroy<MotorTorqueParameters>(motor_torque_params);
    checkAndDestroy<double>(rotToEncoder);
    checkAndDestroy<double>(gearbox);
    checkAndDestroy<bool>(hasHallSensor);
    checkAndDestroy<bool>(hasTempSensor);
    checkAndDestroy<bool>(hasRotorEncoder);
    checkAndDestroy<int>(rotorIndexOffset);
    checkAndDestroy<int>(motorPoles);
    checkAndDestroy<double>(ampsToSensor);
    checkAndDestroy<double>(dutycycleToPwm);
    //  delete[] jointNames;

    _opened = false;
    return true;
}

void iCubSimulationControl::compute_mot_vel(double *mot, double *jnt)
{
    compute_mot_pos(mot,jnt);
}
void iCubSimulationControl::compute_mot_pos(double *mot, double *jnt)
{

}

void iCubSimulationControl::jointStep() {
    _mutex.wait();
    if (manager==NULL) {
        _mutex.post();
        return;
    }
    if (partSelec<=6)
    {   
        for (int axis=0; axis<njoints; axis++)
        {
            LogicalJoint& ctrl = manager->control(partSelec,axis); 
            if (!ctrl.isValid()) continue;
            current_jnt_pos[axis] = ctrl.getAngle();
            current_jnt_vel[axis] = ctrl.getVelocity();
            current_jnt_torques[axis] = (controlMode[axis]==MODE_TORQUE) ? ctrl.getTorque() : 0.0;  // if not torque ctrl, set torque feedback to 0
            current_mot_torques[axis]=0;

            if (maxCurrent[axis]<=0) 
            {
                controlMode[axis]= VOCAB_CM_HW_FAULT;
                motor_on[axis] = false;
            }
            
            //motor_on[axis] = true; // no reason to turn motors off, for now

            if (controlMode[axis]==MODE_VELOCITY || controlMode[axis]==VOCAB_CM_MIXED || controlMode[axis]==MODE_IMPEDANCE_VEL)
            {
                if(((current_jnt_pos[axis]<limitsMin[axis])&&(next_vel[axis]<0)) || ((current_jnt_pos[axis]>limitsMax[axis])&&(next_vel[axis]>0)))
                {
                    ctrl.setVelocity(0.0);
                }
                else
                {
                    ctrl.setVelocity(next_vel[axis]);
                }
            }
            else if (controlMode[axis]==MODE_POSITION || controlMode[axis]==MODE_IMPEDANCE_POS)
            {
                ctrl.setControlParameters(vels[axis],1);
                ctrl.setPosition(next_pos[axis]);
            }
            else if (controlMode[axis]==VOCAB_CM_POSITION_DIRECT)
            {
                ctrl.setControlParameters(5,1);
                ctrl.setPosition(next_pos[axis]);
            }
            else if (controlMode[axis]==MODE_TORQUE)
            {
                ctrl.setTorque(next_torques[axis]);
            }
            else if (controlMode[axis]==MODE_PWM)
            {
                pwm[axis] = pwm_ref[axis];
                //currently identical to velocity control, with fixed velocity
                if (((current_jnt_pos[axis]<limitsMin[axis]) && (pwm_ref[axis]<0)) || ((current_jnt_pos[axis]>limitsMax[axis]) && (pwm_ref[axis]>0)))
                {
                    ctrl.setVelocity(0.0);
                }
                else
                {
                    if (pwm_ref[axis]>0.001)
                    {
                        ctrl.setVelocity(3);
                    }
                    else if (pwm_ref[axis]<-0.001)
                    {
                        ctrl.setVelocity(-3);
                    }
                    else
                    {
                        ctrl.setVelocity(0.0);
                    }
                }
            }
            else if (controlMode[axis] == MODE_CURRENT)
            {
                current_ampere[axis] = current_ampere_ref[axis];
                //currently identical to velocity control, with fixed velocity
                if (((current_jnt_pos[axis]<limitsMin[axis]) && (current_ampere_ref[axis]<0)) || ((current_jnt_pos[axis]>limitsMax[axis]) && (current_ampere_ref[axis]>0)))
                {
                    ctrl.setVelocity(0.0);
                }
                else
                {
                    if (current_ampere_ref[axis]>0.001)
                    {
                        ctrl.setVelocity(3);
                    }
                    else if (current_ampere_ref[axis]<-0.001)
                    {
                        ctrl.setVelocity(-3);
                    }
                    else
                    {
                        ctrl.setVelocity(0.0);
                    }
                }
            }
        }
        compute_mot_pos     (current_mot_pos,current_jnt_pos);
        compute_mot_vel     (current_mot_vel,current_jnt_vel);
    }
    _mutex.post();
}

bool iCubSimulationControl::getAxes(int *ax)
{
    *ax = njoints;
    return true;
}

bool iCubSimulationControl::setPidRaw (const PidControlTypeEnum& pidtype, int axis, const Pid &pid)
{
   if( (axis>=0) && (axis<njoints) )
   {
       position_pid[axis]=pid;
   }
   NOT_YET_IMPLEMENTED("setPidRaw");
   return true; //fake
}

bool iCubSimulationControl::getPidRaw (const PidControlTypeEnum& pidtype, int axis, Pid *out)
{
   if( (axis>=0) && (axis<njoints) )
   {
       *out=position_pid[axis];
   }
   NOT_YET_IMPLEMENTED("getPidRaw");
   return true; //fake
}

bool iCubSimulationControl::getPidsRaw (const PidControlTypeEnum& pidtype, Pid *out)
{
    for (int i=0; i<njoints; i++)
    {
        out[i]=position_pid[i];
    }
    NOT_YET_IMPLEMENTED("getPidsRaw");
    return true; //fake
}

bool iCubSimulationControl::setPidsRaw(const PidControlTypeEnum& pidtype, const Pid *pids)
{
    switch (pidtype)
    {
        case VOCAB_PIDTYPE_POSITION:
        for (int i=0; i<njoints; i++)
        {
            position_pid[i]=pids[i];
        }
        break;
        case VOCAB_PIDTYPE_CURRENT:
        case VOCAB_PIDTYPE_VELOCITY:
        case VOCAB_PIDTYPE_TORQUE:
            NOT_YET_IMPLEMENTED("setPidsRaw");
        break;
        default:
            NOT_YET_IMPLEMENTED("setPidsRaw");
        break;
    }
    return true; //fake
}

/// cmd is a SingleAxis pointer with 1 double arg
bool iCubSimulationControl::setPidReferenceRaw (const PidControlTypeEnum& pidtype, int axis, double ref)
{
    int mode = 0;
    getControlModeRaw(axis, &mode);
    if (mode != VOCAB_CM_POSITION_DIRECT)
    {
        #ifdef ICUB_AUTOMATIC_MODE_SWITCHING
        yWarning() << "setReferenceRaw: Deprecated automatic switch to VOCAB_CM_POSITION_DIRECT, part " << partSelec << " joint: " << axis;
        setControlModeRaw(j,VOCAB_CM_POSITION_DIRECT);
        #else
        yError() << "setReferenceRaw: skipping command because part " << partSelec << " joint" << axis << "is not in VOCAB_CM_POSITION_DIRECT mode";
        return false;
        #endif
    }

    if( (axis>=0) && (axis<njoints) )
        {
            _mutex.wait();
            next_pos[axis] = ref;
            _mutex.post();
            return true;
        }
    if (verbosity)
        yError("setReferenceRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::setPidReferencesRaw (const PidControlTypeEnum& pidtype, const double *refs)
{
   bool ret = true;
   for(int axis = 0; axis<njoints; axis++)
   {
      ret &= setPidReferenceRaw(pidtype, axis, refs[axis]);
   }
   return ret;
}

bool iCubSimulationControl::setPidErrorLimitRaw(const PidControlTypeEnum& pidtype, int axis, double limit)
{
    return NOT_YET_IMPLEMENTED("setErrorLimitRaw");
}

bool iCubSimulationControl::setPidErrorLimitsRaw(const PidControlTypeEnum& pidtype, const double *limit)
{
    return NOT_YET_IMPLEMENTED("setErrorLimitsRaw");
}

bool iCubSimulationControl::getPidErrorRaw(const PidControlTypeEnum& pidtype, int axis, double *err)
{
    if ((axis >= 0) && (axis<njoints))
    {
        _mutex.wait();
        switch (pidtype)
        {
            case VOCAB_PIDTYPE_POSITION:
            *err = current_jnt_pos[axis] - next_pos[axis];
            break;
        }
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("getErrorRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::getPidErrorsRaw(const PidControlTypeEnum& pidtype, double *errs)
{
    bool ret = true;
    for (int axis = 0; axis<njoints; axis++)
    {
        ret &= getPidErrorRaw(pidtype, axis, &errs[axis]);
    }
    return ret;
}

bool iCubSimulationControl::getPidOutputRaw(const PidControlTypeEnum& pidtype, int axis, double *out)
{
    if( (axis>=0) && (axis<njoints) )
        {
            _mutex.wait();
            *out = pwm[axis];
            _mutex.post();
            return true;
        }
    if (verbosity)
        yError("getOutputRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::getPidOutputsRaw(const PidControlTypeEnum& pidtype, double *outs)
{
    _mutex.wait();
    for(int axis = 0; axis<njoints; axis++)
        outs[axis] = pwm[axis];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::isPidEnabledRaw(const PidControlTypeEnum& pidtype, int j, bool* enabled)
{
    return NOT_YET_IMPLEMENTED("isPidEnabled");
}

bool iCubSimulationControl::getNumberOfMotorsRaw(int* num)
{
    *num = njoints;
    return true;
}

bool iCubSimulationControl::getTemperatureRaw(int m, double* val)
{
    return NOT_YET_IMPLEMENTED("getTemperatureRaw");
}

bool iCubSimulationControl::getTemperaturesRaw(double *vals)
{
    return NOT_YET_IMPLEMENTED("getTemperaturesRaw");
}

bool iCubSimulationControl::getTemperatureLimitRaw(int m, double *temp)
{
    return NOT_YET_IMPLEMENTED("getTemperatureLimitRaw");
}

bool iCubSimulationControl::setTemperatureLimitRaw(int m, const double temp)
{
    return NOT_YET_IMPLEMENTED("setTemperatureLimitRaw");
}

bool iCubSimulationControl::getMotorOutputLimitRaw(int m, double *limit)
{
    return NOT_YET_IMPLEMENTED("getMotorOutputLimitRaw");
}

bool iCubSimulationControl::setMotorOutputLimitRaw(int m, const double limit)
{
    return NOT_YET_IMPLEMENTED("setMotorOutputLimitRaw");
}

bool iCubSimulationControl::getPeakCurrentRaw(int m, double *val)
{
    return NOT_YET_IMPLEMENTED("getPeakCurrentRaw");
}

bool iCubSimulationControl::setPeakCurrentRaw(int m, const double val)
{
    return NOT_YET_IMPLEMENTED("setPeakCurrentRaw");
}

bool iCubSimulationControl::getNominalCurrentRaw(int m, double *val)
{
    return NOT_YET_IMPLEMENTED("getNominalCurrentRaw");
}

bool iCubSimulationControl::setNominalCurrentRaw(int m, const double val)
{
    return NOT_YET_IMPLEMENTED("setNominalCurrentRaw");
}

bool iCubSimulationControl::getPWMRaw(int j, double* val)
{
    return NOT_YET_IMPLEMENTED("getPWMRaw");
}

bool iCubSimulationControl::getPWMLimitRaw(int j, double* val)
{
    return NOT_YET_IMPLEMENTED("getPWMLimitRaw");
}

bool iCubSimulationControl::setPWMLimitRaw(int j, const double val)
{
    return NOT_YET_IMPLEMENTED("setPWMLimitRaw");
}

bool iCubSimulationControl::getPowerSupplyVoltageRaw(int j, double* val)
{
    return NOT_YET_IMPLEMENTED("getPowerSupplyVoltageRaw");
}

bool iCubSimulationControl::getPidReferenceRaw(const PidControlTypeEnum& pidtype, int axis, double *ref)
{
    if( (axis>=0) && (axis<njoints) )
        {
            _mutex.wait();
            switch(pidtype)
            {
                case VOCAB_PIDTYPE_POSITION:
                     *ref = next_pos[axis];
                break;
            }
            _mutex.post();
            return true;
        }
    if (verbosity)
        yError("getReferenceRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::getPidReferencesRaw(const PidControlTypeEnum& pidtype, double *ref)
{
    for(int axis = 0; axis<njoints; axis++)
    {
        getPidReferenceRaw(pidtype, axis, &ref[axis]);
    }
    return true;
}

bool iCubSimulationControl::getPidErrorLimitRaw(const PidControlTypeEnum& pidtype, int axis, double *err)
{
     return NOT_YET_IMPLEMENTED("getErrorLimitRaw");
}

bool iCubSimulationControl::getPidErrorLimitsRaw(const PidControlTypeEnum& pidtype, double *errs)
{
   return NOT_YET_IMPLEMENTED("getErrorLimitsRaw");
}

bool iCubSimulationControl::resetPidRaw(const PidControlTypeEnum& pidtype, int axis)
{
    return NOT_YET_IMPLEMENTED("resetPidRaw");
}

bool iCubSimulationControl::enablePidRaw(const PidControlTypeEnum& pidtype, int axis)
{
    return DEPRECATED("enablePidRaw");
}

bool iCubSimulationControl::setPidOffsetRaw(const PidControlTypeEnum& pidtype, int axis, double v)
{
    return NOT_YET_IMPLEMENTED("setOffsetRaw");
}

bool iCubSimulationControl::disablePidRaw(const PidControlTypeEnum& pidtype, int axis)
{
    return DEPRECATED("disablePidRaw");
}

bool iCubSimulationControl::setPositionModeRaw()
{
    return DEPRECATED("setPositionModeRaw");
}

bool iCubSimulationControl::setVelocityModeRaw()
{
    return DEPRECATED("setVelocityModeRaw");
}

bool iCubSimulationControl::positionMoveRaw(int axis, double ref)
{
    if( (axis >=0) && (axis<njoints) )
    {
        int mode = 0;
        getControlModeRaw(axis, &mode);
        if (mode != VOCAB_CM_POSITION &&
            mode != VOCAB_CM_MIXED  )
        {
            #ifdef ICUB_AUTOMATIC_MODE_SWITCHING
            yWarning() << "positionMoveRaw: Deprecated automatic switch to VOCAB_CM_POSITION, part " << partSelec << " joint: " << axis;
            setControlModeRaw(axis,VOCAB_CM_POSITION);
            #else
            yError() << "positionMoveRaw: skipping command because part " << partSelec << " joint " << axis << "is not in VOCAB_CM_POSITION mode";
            return false;
            #endif
        }

        _mutex.wait();
        if(ref< limitsMin[axis])
        {
            if (njoints == 16)
            {
                if ( axis == 7 )
                    next_pos[axis] = limitsMax[axis];
            }
            else
            {
                next_pos[axis] = limitsMin[axis];
            }
            ref_command_positions[axis] = next_pos[axis];
        }
        else if(ref > limitsMax[axis])
        {
            if (njoints == 16)
            {
                if ( axis == 7 )
                    next_pos[axis] = fabs(limitsMax[axis] - limitsMax[axis]);
            }
            else
            {
                next_pos[axis] = limitsMax[axis];
            }
            ref_command_positions[axis] = next_pos[axis];
        }
        else
        {
            ref_command_positions[axis] = ref;
            if (njoints == 16)
            {
                if ( axis == 10 ||  axis == 12 || axis == 14 ) 
                    next_pos[axis] = ref/2;
                else if ( axis == 15 ) 
                    next_pos[axis] = ref/3;
                else if ( axis == 7 )
                    next_pos[axis] = fabs(limitsMax[axis] - ref);
                else
                    next_pos[axis] = ref;
            }
            else
            {
                next_pos[axis] = ref;
            }
        }

        motor_on[axis]=true;

        if (verbosity)
            yDebug("moving joint %d of part %d to pos %f\n",axis, partSelec, next_pos[axis]);
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("positionMoveRaw joint access too high %d \n",axis);
    return false;
}

bool iCubSimulationControl::positionMoveRaw(const double *refs)
{
    bool ret = true;
    for(int axis = 0; axis<njoints; axis++)
    {
        ret &= positionMoveRaw(axis, refs[axis]);
    }
    return ret;
}

bool iCubSimulationControl::relativeMoveRaw(int axis, double delta)
{
    return positionMoveRaw(axis,next_pos[axis]+delta);
}

bool iCubSimulationControl::relativeMoveRaw(const double *deltas)
{
    for(int axis = 0; axis<njoints; axis++) {
        relativeMoveRaw(axis,deltas[axis]);
    }
    return true;
}

bool iCubSimulationControl::checkMotionDoneRaw (bool *ret)
{
    _mutex.wait();
    bool fin = true;
    for(int axis = 0;axis<njoints;axis++)
    {
        if(! (fabs( current_jnt_pos[axis]-next_pos[axis])<error_tol[axis]))
            {
                fin = false;
            }
    }
    if (verbosity)
        yDebug("motion finished error tol %f %f %f\n",error_tol[0],current_jnt_pos[0],next_pos[0]);
    *ret = fin;
    _mutex.post();
    return true;
}

bool iCubSimulationControl::checkMotionDoneRaw(int axis, bool *ret)
{
    if( (axis >=0) && (axis<njoints) )
        {
            _mutex.wait();
            if(fabs(current_jnt_pos[axis]-next_pos[axis])<error_tol[axis])
                *ret = true;
            else
                *ret = false;
            _mutex.post();
            return true;
        }
    if (verbosity)
        yError("checkMotionDoneRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}
bool iCubSimulationControl::setRefSpeedRaw(int axis, double sp)
{
    if( (axis >=0) && (axis<njoints) )
        {
            _mutex.wait();
            //vel = sp;// *180/M_PI ;
            vels[axis] = sp;//vel/20;
            ref_speeds[axis] = sp;
            if (verbosity)
                yDebug("setting joint %d of part %d to reference velocity %f\n",axis,partSelec,vels[axis]);
            _mutex.post();
            return true;
        }
    return false;
}

bool iCubSimulationControl::setRefSpeedsRaw(const double *spds)
{
    bool ret = true;
    for (int axis = 0; axis<njoints; axis++)
    {
        ret &= velocityMoveRaw(axis, spds[axis]);
    }
    return ret;
}
bool iCubSimulationControl::setRefAccelerationRaw(int axis, double acc)
{
    return NOT_YET_IMPLEMENTED("setRefAccelerationRaw");
}
bool iCubSimulationControl::setRefAccelerationsRaw(const double *accs)
{
    return NOT_YET_IMPLEMENTED("setRefAccelerationsRaw");
}
bool iCubSimulationControl::getRefSpeedRaw(int axis, double *ref)
{
    if((axis>=0) && (axis<njoints)) {
        _mutex.wait();
        *ref = ref_speeds[axis];
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("getRefSpeedRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}
bool iCubSimulationControl::getRefSpeedsRaw(double *spds)
{
    bool ret = true;
    for (int i = 0; i<njoints; i++)
    {
        ret &= getRefSpeedRaw(i, &spds[i]);
    }
    return ret;
}
bool iCubSimulationControl::getRefAccelerationRaw(int axis, double *acc)
{
    return NOT_YET_IMPLEMENTED("getRefAccelerationRaw");
}
bool iCubSimulationControl::getRefAccelerationsRaw(double *accs)
{
    return NOT_YET_IMPLEMENTED("getRefAccelerationsRaw");
}
bool iCubSimulationControl::stopRaw(int axis)
{
    if( (axis>=0) && (axis<njoints) )
    {
        _mutex.wait();
        next_pos[axis] = current_jnt_pos[axis];
        next_vel[axis] = 0.0;
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("stopRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}
bool iCubSimulationControl::stopRaw()
{
    _mutex.wait();
    for(int axis=0;axis<njoints;axis++)
    {
        next_pos[axis] = current_jnt_pos[axis];
        next_vel[axis] = 0.0;
    }
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getTargetPositionRaw(int axis, double *ref)
{
    if ((axis >= 0) && (axis<njoints)) {
        _mutex.wait();
        *ref = ref_command_positions[axis];
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("getTargetPositionRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::getTargetPositionsRaw(double *refs)
{
    bool ret = true;
    for (int i = 0; i<njoints; i++)
    {
        ret &= getTargetPositionRaw(i, &refs[i]);
    }
    return ret;
}

bool iCubSimulationControl::getTargetPositionsRaw(int nj, const int * jnts, double *refs)
{
    bool ret = true;
    for (int i = 0; i<nj; i++)
    {
        ret &= getTargetPositionRaw(jnts[i], &refs[i]);
    }
    return ret;
}

bool iCubSimulationControl::getRefVelocityRaw(int axis, double *ref)
{
    _mutex.wait();
    *ref = ref_command_speeds[axis];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getRefVelocitiesRaw(double *refs)
{
    bool ret = true;
    for (int i = 0; i<njoints; i++)
    {
        ret &= getRefVelocityRaw(i, &refs[i]);
    }
    return ret;
}

bool iCubSimulationControl::getRefVelocitiesRaw(int nj, const int * jnts, double *refs)
{
    bool ret = true;
    for (int i = 0; i<nj; i++)
    {
        ret &= getRefVelocityRaw(jnts[i], &refs[i]);
    }
    return ret;
}

bool iCubSimulationControl::getRefPositionRaw(int axis, double *ref)
{
    _mutex.wait();
    *ref = ref_positions[axis];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getRefPositionsRaw(double *refs)
{
    bool ret = true;
    for (int i = 0; i<njoints; i++)
    {
        ret &= getRefPositionRaw(i, &refs[i]);
    }
    return ret;
}


bool iCubSimulationControl::getRefPositionsRaw(int nj, const int * jnts, double *refs)
{
    bool ret = true;
    for (int i = 0; i<nj; i++)
    {
        ret &= getRefPositionRaw(jnts[i], &refs[i]);
    }
    return ret;
}

bool iCubSimulationControl::positionMoveRaw(int nj, const int * jnts, const double *refs)
{
    bool ret = true;
    for (int i = 0; i<nj; i++)
    {
        ret &= positionMoveRaw(jnts[i], refs[i]);
    }
    return ret;
}

bool iCubSimulationControl::relativeMoveRaw(int nj, const int * jnts, const double *refs)
{
    bool ret = true;
    for (int i = 0; i<nj; i++)
    {
        ret &= relativeMoveRaw(jnts[i], refs[i]);
    }
    return ret;
}

bool iCubSimulationControl::checkMotionDoneRaw(int nj, const int * jnts, bool *done)
{
    bool ret = true;
    bool jDone(true), tmpDone(true);

    for (int i = 0; i<nj; i++)
    {
        ret &= checkMotionDoneRaw(jnts[i], &jDone);
        tmpDone &= jDone;
    }
    *done = tmpDone;
    return ret;
}

bool iCubSimulationControl::setRefSpeedsRaw(const int nj, const int * jnts, const double *refs)
{
    bool ret = true;
    for (int i = 0; i<nj; i++)
    {
        ret &= setRefSpeedRaw(jnts[i], refs[i]);
    }
    return ret;
}

bool iCubSimulationControl::getRefSpeedsRaw(const int nj, const int * jnts, double *refs)
{
    bool ret = true;
    for (int i = 0; i<nj; i++)
    {
        ret &= getRefSpeedRaw(jnts[i], &refs[i]);
    }
    return ret;
}

// cmd is an array of double of length njoints specifying speed 
/// for each axis
bool iCubSimulationControl::velocityMoveRaw (int axis, double sp)
{
    if( (axis >=0) && (axis<njoints) )
    {
        int mode = 0;
        getControlModeRaw(axis, &mode);
        if (mode != VOCAB_CM_VELOCITY &&
        mode != VOCAB_CM_MIXED)
        {
            #ifdef ICUB_AUTOMATIC_MODE_SWITCHING
            yWarning() << "velocityMoveRaw: Deprecated automatic switch to VOCAB_CM_VELOCITY, part " << partSelec << " joint: " << axis;
            setControlModeRaw(axis,VOCAB_CM_VELOCITY);
            #else
            yError() << "velocityMoveRaw: skipping command because part " << partSelec << " joint " << axis << "is not in VOCAB_CM_VELOCITY mode";
            return false;
            #endif
        }
        _mutex.wait();
        next_vel[axis] = sp;
        ref_command_speeds[axis] = sp;
        motor_on[axis] = true;
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("velocityMoveRaw: joint with index %d does not exist, valis joints indices are between 0 and %d \n",axis,njoints);
    return false;    
}

/// cmd is an array of double of length njoints specifying speed 
/// for each axis
bool iCubSimulationControl::velocityMoveRaw (const double *sp)
{
    bool ret = true;
    for (int axis=0; axis<njoints; axis++)
    {
        ret &= velocityMoveRaw(axis,sp[axis]);
    }
    return ret;
}

bool iCubSimulationControl::setEncoderRaw(int axis, double val)
{
    return NOT_YET_IMPLEMENTED("setEncoderRaw");
}

bool iCubSimulationControl::setEncodersRaw(const double *vals)
{
    return NOT_YET_IMPLEMENTED("setEncodersRaw");
}

bool iCubSimulationControl::resetEncoderRaw(int axis)
{
    return NOT_YET_IMPLEMENTED("resetEncoderRaw");
}

bool iCubSimulationControl::resetEncodersRaw()
{
   return NOT_YET_IMPLEMENTED("resetEncodersRaw");
}

bool iCubSimulationControl::getEncodersRaw(double *v)
{
   _mutex.wait();
    for(int axis = 0;axis<njoints;axis++)
    {
        if ( axis == 10 ||  axis == 12 || axis == 14 ) 
            v[axis] = current_jnt_pos[axis]*2;
        else if ( axis == 15 ) 
            v[axis] = current_jnt_pos[axis]*3;
        else if ( axis == 7 ) 
            v[axis] = limitsMax[axis] - current_jnt_pos[axis]; 
        else 
            v[axis] = current_jnt_pos[axis];
    }
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getEncoderRaw(int axis, double *v)
{
    if((axis>=0) && (axis<njoints)) {
        _mutex.wait();
        
        if ( axis == 10 ||  axis == 12 || axis == 14 ) 
            *v = current_jnt_pos[axis]*2;
        else if ( axis == 15 ) 
            *v = current_jnt_pos[axis]*3;
        else if ( axis == 7 ) 
            *v = limitsMax[axis] - current_jnt_pos[axis];
        else 
            *v = current_jnt_pos[axis];

        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("getEncoderRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::getEncodersTimedRaw(double *encs, double *stamps)
{
    double timeNow = Time::now();
    for(int axis = 0;axis<njoints;axis++)
    {
        stamps[axis] = timeNow;
    }
    return getEncodersRaw(encs);
}

bool iCubSimulationControl::getEncoderTimedRaw(int axis, double *enc, double *stamp)
{
    *stamp = Time::now();
    return getEncoderRaw(axis, enc);
}


bool iCubSimulationControl::getEncoderSpeedsRaw(double *v)
{
   _mutex.wait();
    for(int axis = 0; axis<njoints; axis++)
        v[axis] = current_jnt_vel[axis];//* 10;
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getEncoderSpeedRaw(int axis, double *v)
{
    if( (axis>=0) && (axis<njoints) ) {
        _mutex.wait();
        *v = current_jnt_vel[axis];// * 10;
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("getEncoderSpeedRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::getEncoderAccelerationsRaw(double *v)
{
  	return NOT_YET_IMPLEMENTED("getEncoderAccelerationsRaw");
}

bool iCubSimulationControl::getEncoderAccelerationRaw(int axis, double *v)
{
    *v=0.0;
    return false; //NOT_YET_IMPLEMENTED("getEncoderAcc");
}

bool iCubSimulationControl::setMotorEncoderRaw(int axis, const double val)
{
    return NOT_YET_IMPLEMENTED("setMotorEncoderRaw");
}

bool iCubSimulationControl::setMotorEncodersRaw(const double *vals)
{
    return NOT_YET_IMPLEMENTED("setMotorEncodersRaw");
}

bool iCubSimulationControl::resetMotorEncoderRaw(int axis)
{
    return NOT_YET_IMPLEMENTED("resetMotorEncoderRaw");
}

bool iCubSimulationControl::resetMotorEncodersRaw()
{
   return NOT_YET_IMPLEMENTED("resetMotorEncodersRaw");
}

bool iCubSimulationControl::getMotorEncodersRaw(double *v)
{
   _mutex.wait();
    for(int axis = 0;axis<njoints;axis++)
    {
        getMotorEncoderRaw(axis,&v[axis]);
    }
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getMotorEncoderRaw(int axis, double *v)
{
    if((axis>=0) && (axis<njoints))
    {
        _mutex.wait();
        *v = current_mot_pos[axis];
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("getMotorEncoderRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::getNumberOfMotorEncodersRaw(int* num)
{
    *num=njoints;
    return true;
}

bool iCubSimulationControl::setMotorEncoderCountsPerRevolutionRaw(int m, const double cpr)
{
    return NOT_YET_IMPLEMENTED("setMotorEncoderCountsPerRevolutionRaw");
}

bool iCubSimulationControl::getMotorEncoderCountsPerRevolutionRaw(int m, double* cpr)
{
   return NOT_YET_IMPLEMENTED("getMotorEncoderCountsPerRevolutionRaw");
}


bool iCubSimulationControl::getMotorEncodersTimedRaw(double *encs, double *stamps)
{
    double timeNow = Time::now();
    for(int axis = 0;axis<njoints;axis++)
    {
        stamps[axis] = timeNow;
    }
    return getMotorEncodersRaw(encs);
}

bool iCubSimulationControl::getMotorEncoderTimedRaw(int axis, double *enc, double *stamp)
{
    *stamp = Time::now();
    return getMotorEncoderRaw(axis, enc);
}


bool iCubSimulationControl::getMotorEncoderSpeedsRaw(double *v)
{
   _mutex.wait();
    for(int axis = 0; axis<njoints; axis++)
    {
        getMotorEncoderSpeedRaw(axis,&v[axis]);
    }
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getMotorEncoderSpeedRaw(int axis, double *v)
{
    if( (axis>=0) && (axis<njoints) ) {
        _mutex.wait();
        *v = current_mot_vel[axis];
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("getEncoderSpeedRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::getMotorEncoderAccelerationsRaw(double *v)
{
  	return NOT_YET_IMPLEMENTED("getMotorEncoderAccelerationsRaw");
}

bool iCubSimulationControl::getMotorEncoderAccelerationRaw(int axis, double *v)
{
    *v=0.0;
    return false; //NOT_YET_IMPLEMENTED("getEncoderAcc");
}

bool iCubSimulationControl::disableAmpRaw(int axis)
{
    return DEPRECATED ("disableAmpRaw");
}

bool iCubSimulationControl::enableAmpRaw(int axis)
{
    return DEPRECATED ("enableAmpRaw");
}

// bcast
bool iCubSimulationControl::getCurrentsRaw(double *cs)
{
   _mutex.wait();
    for(int axis = 0; axis<njoints; axis++)
        cs[axis] = current_ampere[axis];
    _mutex.post();
    return true;
}

// bcast currents
bool iCubSimulationControl::getCurrentRaw(int axis, double *c)
{
    if( (axis>=0) && (axis<njoints) )
    {
        _mutex.wait();
        *c = current_ampere[axis];
        _mutex.post();
    }
    else
    {  
        if (verbosity)
            yError("getCurrentRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    }
    return true;
}

bool iCubSimulationControl::setMaxCurrentRaw(int axis, double v)
{
    if( (axis>=0) && (axis<njoints) )
    {
        _mutex.wait();
        maxCurrent[axis]=v;
        _mutex.post();
    }
    else
    {  
        if (verbosity)
            yError("setMaxCurrentRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    }
    return true;
}

bool iCubSimulationControl::getMaxCurrentRaw(int axis, double* v)
{
    if( (axis>=0) && (axis<njoints) )
    {
        _mutex.wait();
        *v=maxCurrent[axis];
        _mutex.post();
    }
    else
    {  
        if (verbosity)
            yError("getMaxCurrentRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    }
    return true;
}

bool iCubSimulationControl::calibrateRaw(int axis, double p)
{
    return NOT_YET_IMPLEMENTED("calibrateRaw");
}

bool iCubSimulationControl::doneRaw(int axis)
{
    return NOT_YET_IMPLEMENTED("doneRaw");
}


bool iCubSimulationControl::getAmpStatusRaw(int *st)
{
    _mutex.wait();
    for(int axis =0;axis<njoints;axis++)
        st[axis] = (int)motor_on[axis];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getAmpStatusRaw(int axis, int *st)
{
    if( (axis>=0) && (axis<njoints)) 
    {
        _mutex.wait();
        st[axis] = (int)motor_on[axis];
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("getAmpStatusRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::setLimitsRaw(int axis, double min, double max)
{
    if( (axis >=0) && (axis < njoints) ){
        _mutex.wait();
        limitsMax[axis] = max;
        limitsMin[axis] = min;
        _mutex.post();
       return true;
    }
    if (verbosity)
        yError("setLimitsRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::getLimitsRaw(int axis, double *min, double *max)
{
    if( (axis >=0) && (axis < njoints)) {
        _mutex.wait();
         *min = limitsMin[axis];
         *max = limitsMax[axis];
         _mutex.post();
         return true;
     }
     //else
     return false;
}

// IRemoteVariables
bool iCubSimulationControl::getRemoteVariableRaw(yarp::os::ConstString key, yarp::os::Bottle& val)
{
    val.clear();
    if (key == "kinematic_mj")
    {
        val.addString("1 2 3"); return true;
    }
    else if (key == "rotor")
    {
        Bottle& r = val.addList(); for (int i=0; i<njoints; i++) r.addDouble(rotToEncoder[i]); return true;
    }
    else if (key == "gearbox")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addDouble(gearbox[i]); return true;
    }
    else if (key == "zeros")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addDouble(zeros[i]); return true;
    }
    else if (key == "hasHallSensor")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addInt(hasHallSensor[i]); return true;
    }
    else if (key == "hasTempSensor")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addInt(hasTempSensor[i]); return true;
    }
    else if (key == "hasRotorEncoder")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addInt(hasRotorEncoder[i]); return true;
    }
    else if (key == "rotorIndexOffset")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addInt(rotorIndexOffset[i]); return true;
    }
    else if (key == "motorPoles")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addInt(motorPoles[i]); return true;
    }
    else if (key == "pidCurrentKp")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addDouble(current_pid[i].kp); return true;
    }
    else if (key == "pidCurrentKi")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addDouble(current_pid[i].ki); return true;
    }
    else if (key == "pidCurrentShift")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addDouble(current_pid[i].scale); return true;
    }
    yWarning("getRemoteVariable(): Unknown variable %s", key.c_str());
    return false;
}

bool iCubSimulationControl::setRemoteVariableRaw(yarp::os::ConstString key, const yarp::os::Bottle& val)
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
        for (int i = 0; i < njoints; i++)
            rotToEncoder[i] = bval->get(i).asDouble();
        return true;
    }
    else if (key == "gearbox")
    {
        for (int i = 0; i < njoints; i++) gearbox[i] = bval->get(i).asDouble(); return true;
    }
    else if (key == "zeros")
    {
        for (int i = 0; i < njoints; i++) zeros[i] = bval->get(i).asDouble(); return true;
    }
    yWarning("setRemoteVariable(): Unknown variable %s", key.c_str());
    return false;
}

bool iCubSimulationControl::getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys)
{
    listOfKeys->clear();
    listOfKeys->addString("kinematic_mj");
    listOfKeys->addString("rotor");
    listOfKeys->addString("gearbox");
    listOfKeys->addString("hasHallSensor");
    listOfKeys->addString("hasTempSensor");
    listOfKeys->addString("hasRotorEncoder");
    listOfKeys->addString("rotorIndexOffset");
    listOfKeys->addString("motorPoles");
    listOfKeys->addString("pidCurrentKp");
    listOfKeys->addString("pidCurrentKi");
    listOfKeys->addString("pidCurrentShift");
    return true;
}

// IControlLimits2
bool iCubSimulationControl::setVelLimitsRaw(int axis, double min, double max)
{
    if ((axis >= 0) && (axis < njoints)){
        _mutex.wait();
        velLimitsMax[axis] = max;
        velLimitsMin[axis] = min;
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("setVelLimitsRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::getVelLimitsRaw(int axis, double *min, double *max)
{
    if ((axis >= 0) && (axis < njoints)) {
        _mutex.wait();
        *min = velLimitsMin[axis];
        *max = velLimitsMax[axis];
        _mutex.post();
        return true;
    }
    //else
    return false;
}

bool iCubSimulationControl::velocityMoveRaw(const int n_joint, const int *joints, const double *spds)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret && velocityMoveRaw(joints[j], spds[j]);
    }
    return ret;
}

bool iCubSimulationControl::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret && setRefAccelerationRaw(joints[j], accs[j]);
    }
    return ret;
}

bool iCubSimulationControl::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret && getRefAccelerationRaw(joints[j], &accs[j]);
    }
    return ret;
}

bool iCubSimulationControl::stopRaw(const int n_joint, const int *joints)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret && stopRaw(joints[j]);
    }
    return ret;
}

bool iCubSimulationControl::setVelPidRaw(int j, const yarp::dev::Pid &pid)
{
    return NOT_YET_IMPLEMENTED("setVelPidRaw");
}

bool iCubSimulationControl::setVelPidsRaw(const yarp::dev::Pid *pids)
{
    return NOT_YET_IMPLEMENTED("setVelPidsRaw");
}

bool iCubSimulationControl::getVelPidRaw(int j, yarp::dev::Pid *pid)
{
    return NOT_YET_IMPLEMENTED("getVelPidRaw");
}

bool iCubSimulationControl::getVelPidsRaw(yarp::dev::Pid *pids)
{
    return NOT_YET_IMPLEMENTED("getVelPidsRaw");
}

bool iCubSimulationControl::getAxisNameRaw(int axis, yarp::os::ConstString& name)
{
    if ((axis >= 0) && (axis < njoints))
    {
        _mutex.wait();
        char buff[100];
        sprintf(buff,"JOINT%d", axis);
        name.assign(buff,strlen(buff));
        _mutex.post();
        return true;
    }
    
    return false;
}

bool iCubSimulationControl::getJointTypeRaw(int axis, yarp::dev::JointTypeEnum& type)
{
    if ((axis >= 0) && (axis < njoints))
    {
        _mutex.wait();
        type = yarp::dev::VOCAB_JOINTTYPE_REVOLUTE;
        _mutex.post();
        return true;
    }

    return false;
}

bool iCubSimulationControl::setTorqueModeRaw( )
{
    return NOT_YET_IMPLEMENTED("setTorqueModeRaw");
}
bool iCubSimulationControl::getTorqueRaw(int axis, double *sp)
{
    if( (axis >=0) && (axis < njoints)) {
        _mutex.wait();
         *sp = current_jnt_torques[axis];
         _mutex.post();
         return true;
    }
    return false;
}
bool iCubSimulationControl::getTorquesRaw(double *sp)
{
   _mutex.wait();
    for(int axis = 0;axis<njoints;axis++)
        sp[axis] = current_jnt_torques[axis];
    _mutex.post();
    return true;
}
bool iCubSimulationControl::getTorqueRangeRaw(int axis, double *a,double *b)
{
    return NOT_YET_IMPLEMENTED("getTorqueRangeRaw");
}
bool iCubSimulationControl::getTorqueRangesRaw(double *a,double *b)
{
    return NOT_YET_IMPLEMENTED("getTorqueRangesRaw");
}
bool iCubSimulationControl::setRefTorquesRaw(const double *sp)
{
    bool ret = true;
    for (int i = 0; i<njoints; i++)
    {
        ret &= setRefTorqueRaw(i, sp[i]);
    }
    return ret;
}

bool iCubSimulationControl::setRefTorqueRaw(int axis,double ref)
{
    if( (axis >=0) && (axis<njoints) ) {
        _mutex.wait();
        next_torques[axis] = ref;
        ref_torques[axis] = ref;
        motor_on[axis] = true;
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("setRefTorqueRaw: joint with index %d does not exist, valis joints indices are between 0 and %d \n",axis,njoints);
    return false;
}

bool iCubSimulationControl::setRefTorquesRaw(const int n_joint, const int *joints, const double *t)
{
    bool ret = true;
    for (int j = 0; j<n_joint; j++)
    {
        ret = ret && setRefTorqueRaw(joints[j], t[j]);
    }
    return ret;
}

bool iCubSimulationControl::getRefTorquesRaw(double *refs)
{
    bool ret = true;
    for (int i = 0; i<njoints; i++)
    {
        ret &= getRefTorqueRaw(i, &refs[i]);
    }
    return ret;
}
bool iCubSimulationControl::getRefTorqueRaw(int axis,double *ref)
{
    if( (axis >=0) && (axis<njoints) ) {
        _mutex.wait();
        *ref = next_torques[axis];
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("getRefTorqueRaw: joint with index %d does not exist, valis joints indices are between 0 and %d \n",axis,njoints);
    return false;
}
bool iCubSimulationControl::getBemfParamRaw(int axis,double *bemf)
{
    return NOT_YET_IMPLEMENTED("getBemfParamRaw");
}
bool iCubSimulationControl::setBemfParamRaw(int axis,double bemf)
{
    return NOT_YET_IMPLEMENTED("setBemfParamRaw");
}
bool iCubSimulationControl::setTorquePidRaw(int axis, const yarp::dev::Pid &pid)
{
    if( (axis >=0) && (axis<njoints) )
    {
        torque_pid[axis]=pid;
    }
    NOT_YET_IMPLEMENTED("setTorquePidRaw");
    return true; //fake
}
bool iCubSimulationControl::setTorquePidsRaw(const yarp::dev::Pid *pid)
{
    for (int i=0; i<njoints; i++)
    {
         torque_pid[i]=pid[i];
    }
    NOT_YET_IMPLEMENTED("setTorquePidsRaw");
    return true; //fake
}
bool iCubSimulationControl::getMotorTorqueParamsRaw(int axis, MotorTorqueParameters *params)
{
    if( (axis >=0) && (axis<njoints) )
    {
         params->bemf=motor_torque_params[axis].bemf;
         params->bemf_scale=motor_torque_params[axis].bemf_scale;
         params->ktau=motor_torque_params[axis].ktau;
         params->ktau_scale=motor_torque_params[axis].ktau_scale;
    }
    NOT_YET_IMPLEMENTED("getMotorTorqueParamsRaw");
    return true; //fake

}
bool iCubSimulationControl::setMotorTorqueParamsRaw(int axis, const MotorTorqueParameters params)
{
    if( (axis >=0) && (axis<njoints) )
    {
        motor_torque_params[axis].bemf=params.bemf;
        motor_torque_params[axis].bemf_scale=params.bemf_scale;
        motor_torque_params[axis].ktau=params.ktau;
        motor_torque_params[axis].ktau_scale=params.ktau_scale;
    }
    NOT_YET_IMPLEMENTED("setMotorTorqueParamsRaw");
    return true; //fake
}

bool iCubSimulationControl::setTorqueErrorLimitRaw(int axis, double lim)
{
    return NOT_YET_IMPLEMENTED("setTorqueErrorLimitRaw");
}
bool iCubSimulationControl::setTorqueErrorLimitsRaw(const double *lims)
{
    return NOT_YET_IMPLEMENTED("setTorqueErrorLimitsRaw");
}
bool iCubSimulationControl::getTorqueErrorRaw(int axis,double *err)
{
    return NOT_YET_IMPLEMENTED("getTorqueErrorRaw");
}
bool iCubSimulationControl::getTorqueErrorsRaw(double *err)
{
    return NOT_YET_IMPLEMENTED("getTorqueErrorsRaw");
}
bool iCubSimulationControl::getTorquePidOutputRaw(int axis, double *out)
{
    *out=0.0;
    return false;
}
bool iCubSimulationControl::getTorquePidOutputsRaw(double *out)
{
    return NOT_YET_IMPLEMENTED("getTorquePidOutputsRaw");
}
bool iCubSimulationControl::getTorquePidRaw(int axis, yarp::dev::Pid *pid)
{
    if( (axis >=0) && (axis<njoints) )
    {
        *pid=torque_pid[axis];
    }
    NOT_YET_IMPLEMENTED("getTorquePidRaw");
    return true; //fake
}
bool iCubSimulationControl::getTorquePidsRaw(yarp::dev::Pid *pid)
{
    for (int i=0; i<njoints; i++)
    {
        pid[i]=torque_pid[i];
    }
    NOT_YET_IMPLEMENTED("getTorquePidRaw");
    return true; //fake
}
bool iCubSimulationControl::getTorqueErrorLimitRaw(int axis, double *err)
{
    return NOT_YET_IMPLEMENTED("getTorqueErrorLimitRaw");
}
bool iCubSimulationControl::getTorqueErrorLimitsRaw(double *err)
{
    return NOT_YET_IMPLEMENTED("getTorqueErrorLimitsRaw");
}
bool iCubSimulationControl::resetTorquePidRaw(int axis)
{
    return NOT_YET_IMPLEMENTED("resetTorquePidRaw");
}
bool iCubSimulationControl::disableTorquePidRaw(int axis)
{
    return DEPRECATED("disableTorquePidRaw");
}
bool iCubSimulationControl::enableTorquePidRaw(int axis)
{
    return DEPRECATED("enableTorquePidRaw");
}
bool iCubSimulationControl::setTorqueOffsetRaw(int axis,double offset)
{
    return NOT_YET_IMPLEMENTED("setTorqueOffsetRaw");
}

bool iCubSimulationControl::setPositionModeRaw(int j)
{
    return DEPRECATED("setPositionModeRaw");
}
bool iCubSimulationControl::setVelocityModeRaw(int j)
{
    return DEPRECATED("setVelocityModeRaw");
}
bool iCubSimulationControl::setTorqueModeRaw(int j)
{
    return DEPRECATED("setTorqueModeRaw");
}
bool iCubSimulationControl::setImpedancePositionModeRaw(int j)
{    
    return DEPRECATED("setImpedancePositionModeRaw");
}
bool iCubSimulationControl::setImpedanceVelocityModeRaw(int j)
{
    return DEPRECATED("setImpedanceVelocityModeRaw");
}

int iCubSimulationControl::ControlModes_yarp2iCubSIM(int yarpMode)
{
    int ret = VOCAB_CM_UNKNOWN;
    switch(yarpMode)
    {
        case VOCAB_CM_FORCE_IDLE:
        case VOCAB_CM_IDLE:
            ret = MODE_IDLE;
            break;
        case VOCAB_CM_POSITION:
            ret = MODE_POSITION;
            break;
        case VOCAB_CM_VELOCITY:
            ret = MODE_VELOCITY;
            break;
        case VOCAB_CM_TORQUE:
            ret = MODE_TORQUE;
            break;
        case VOCAB_CM_IMPEDANCE_POS:
            ret = MODE_IMPEDANCE_POS;
            break;
        case VOCAB_CM_IMPEDANCE_VEL:
            ret = MODE_IMPEDANCE_VEL;
            break;
        case VOCAB_CM_PWM:
            ret = MODE_PWM;
            break;
        case VOCAB_CM_CURRENT:
            ret = MODE_CURRENT;
            break;

        // for new modes I´ll use directly the vocabs, so they are the same
        case VOCAB_CM_MIXED:
        case VOCAB_CM_POSITION_DIRECT:
        case VOCAB_CM_HW_FAULT:
        case VOCAB_CM_CALIBRATING:
        case VOCAB_CM_CALIB_DONE:
        case VOCAB_CM_NOT_CONFIGURED:
        case VOCAB_CM_CONFIGURED:
            ret = yarpMode;
            break;

        default:
            ret = MODE_UNKNOWN;
            break;
    }
    return ret;
}

int iCubSimulationControl::ControlModes_iCubSIM2yarp(int iCubMode)
{
    int ret = VOCAB_CM_UNKNOWN;
    switch(iCubMode)
    {
        case MODE_IDLE:
            ret=VOCAB_CM_IDLE;
            break;
        case MODE_POSITION:
            ret=VOCAB_CM_POSITION;
            break;
        case MODE_VELOCITY:
            ret=VOCAB_CM_VELOCITY;
            break;
        case MODE_TORQUE:
            ret=VOCAB_CM_TORQUE;
            break;
        case MODE_IMPEDANCE_POS:
            ret=VOCAB_CM_IMPEDANCE_POS;
            break;
        case MODE_IMPEDANCE_VEL:
            ret=VOCAB_CM_IMPEDANCE_VEL;
            break;
        case MODE_PWM:
            ret=VOCAB_CM_PWM;
            break;
        case MODE_CURRENT:
            ret = VOCAB_CM_CURRENT;
            break;

        // for new modes I´ll use directly the vocabs, so they are the same
        case MODE_MIXED:
        case MODE_HW_FAULT:
        case VOCAB_CM_POSITION_DIRECT:
        case MODE_CALIBRATING:
        case MODE_CALIB_DONE:
        case MODE_NOT_CONFIGURED:
        case MODE_CONFIGURED:
            ret = iCubMode;
            break;

        default:
            ret=VOCAB_CM_UNKNOWN;
            break;
    }
    return ret;
}

bool iCubSimulationControl::getControlModeRaw(int j, int *mode)
{    
    if( (j >=0) && (j < njoints)) {
        _mutex.wait();
        *mode = ControlModes_iCubSIM2yarp(controlMode[j]);
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("getControlModeRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", j, njoints);
    return false; 
}

bool iCubSimulationControl::getControlModesRaw(int* modes)
{
   _mutex.wait();
    for(int axis = 0;axis<njoints;axis++)
        modes[axis] = ControlModes_iCubSIM2yarp(controlMode[axis]);
    _mutex.post();
    return true;
}

/////// Control Mode2 Interface
bool iCubSimulationControl::getControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    bool ret = true;
    for(int idx=0; idx< n_joint; idx++)
        ret = ret && getControlModeRaw(joints[idx], &modes[idx]);
    return ret;
}

bool iCubSimulationControl::setControlModeRaw(const int j, const int mode)
{
    int tmp;
    if( (j >=0) && (j < njoints) )
    {
        tmp = ControlModes_yarp2iCubSIM(mode);
        if(tmp == MODE_UNKNOWN)
        {
            yError() << "setControlModeRaw: unknown control mode " << yarp::os::Vocab::decode(mode);
        }
        else if (controlMode[j] == VOCAB_CM_HW_FAULT && mode != VOCAB_CM_FORCE_IDLE)
        {
            yError() << "setControlModeRaw: unable to reset an HW_FAULT without a VOCAB_CM_FORCE_IDLE command";
        }
        else
        {
            _mutex.wait();
            controlMode[j] = ControlModes_yarp2iCubSIM(mode);
            next_pos[j]=current_jnt_pos[j];
            if (controlMode[j] != MODE_PWM) pwm_ref[j] = 0;
            if (controlMode[j] != MODE_CURRENT) current_ampere_ref[j] = 0;
            _mutex.post();
        }
       return true;
    }
    if (verbosity)
        yError("setControlModeRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", j, njoints);
    return false;
}

bool iCubSimulationControl::setControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    bool ret = true;
    for(int idx=0; idx<n_joint; idx++)
    {
        ret = ret && setControlModeRaw(joints[idx], modes[idx]);
    }
    return ret;
}

bool iCubSimulationControl::setControlModesRaw(int *modes)
{
    bool ret = true;
    for(int j=0; j<njoints; j++)
    {
        ret = ret && setControlModeRaw(j, modes[j]);
    }
    return ret;
}

/////// InteractionMode
bool iCubSimulationControl::getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum* mode)
{
    *mode = (yarp::dev::InteractionModeEnum) interactionMode[axis];
    return true;
}

bool iCubSimulationControl::getInteractionModesRaw(int n_joints, int *__joints, yarp::dev::InteractionModeEnum* modes)
{
    bool ret = true;
    for(int idx=0; idx<n_joints; idx++)
    {
        ret = ret && getInteractionModeRaw(__joints[idx], &modes[idx]);
    }
    return ret;
}

bool iCubSimulationControl::getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    bool ret = true;
    for(int j=0; j<njoints; j++)
    {
        ret = ret && getInteractionModeRaw(j, &modes[j]);
    }
    return ret;
}

bool iCubSimulationControl::setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode)
{
    interactionMode[axis] = (int)mode;
    next_pos[axis]=current_jnt_pos[axis];
    return true;
}

bool iCubSimulationControl::setInteractionModesRaw(int n_joints, int *__joints, yarp::dev::InteractionModeEnum* modes)
{
    bool ret = true;
    for(int idx=0; idx<n_joints; idx++)
    {
        ret = ret && setInteractionModeRaw(__joints[idx], modes[idx]);
    }
    return ret;
}

bool iCubSimulationControl::setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    bool ret = true;
    for(int j=0; j<njoints; j++)
    {
        ret = ret && setInteractionModeRaw(j, modes[j]);
    }
    return ret;
}

// Position direct interface
bool iCubSimulationControl::setPositionDirectModeRaw()
{
    bool ret = true;
    for(int j=0; j<njoints; j++)
    {
        ret = ret && setControlModeRaw(j, VOCAB_POSITION_DIRECT);
    }
    return ret;
}

bool iCubSimulationControl::setPositionRaw(int axis, double ref)
{
    // This function has been copy pasted from positionMoveRaw
    if( (axis >=0) && (axis<njoints) )
    {
        int mode = 0;
        getControlModeRaw(axis, &mode);
        if (mode != VOCAB_CM_POSITION_DIRECT)
        {
            #ifdef ICUB_AUTOMATIC_MODE_SWITCHING
            yWarning() << "setPositionRaw: Deprecated automatic switch to VOCAB_CM_POSITION_DIRECT, part:" << partSelec << " joint: " << axis;
            setControlModeRaw(j,VOCAB_CM_POSITION_DIRECT);
            #else
            yError() << "setPositionRaw: skipping command because part " << partSelec << " joint" << axis << "is not in VOCAB_CM_POSITION_DIRECT mode";
            return false;
            #endif
        }
        _mutex.wait();
        if(ref< limitsMin[axis])
        {
            if (njoints == 16)
            {
                if ( axis == 7 )
                    next_pos[axis] = limitsMax[axis];
            }
            else
            {
                next_pos[axis] = limitsMin[axis];
            }
            ref_positions[axis] = next_pos[axis];
        }
        else if(ref > limitsMax[axis])
        {
            if (njoints == 16)
            {
                if ( axis == 7 )
                    next_pos[axis] = fabs(limitsMax[axis] - limitsMax[axis]);
            }
            else
            {
                next_pos[axis] = limitsMax[axis];
            }
            ref_positions[axis] = next_pos[axis];
        }
        else
        {
            ref_positions[axis] = ref;
            if (njoints == 16)
            {
                if ( axis == 10 ||  axis == 12 || axis == 14 )
                    next_pos[axis] = ref/2;
                else if ( axis == 15 )
                    next_pos[axis] = ref/3;
                else if ( axis == 7 )
                    next_pos[axis] = fabs(limitsMax[axis] - ref);
                else
                    next_pos[axis] = ref;
            }
            else
            {
                next_pos[axis] = ref;
            }
        }

        motor_on[axis]=true;

        if (verbosity)
            yDebug("moving joint %d of part %d to pos %f (pos direct)\n",axis, partSelec, next_pos[axis]);
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("setPositionRaw joint access too high %d \n",axis);
    return false;
}

bool iCubSimulationControl::setPositionsRaw(const int n_joint, const int *joints, double *refs)
{
    bool ret = true;
    for(int i=0; i<n_joint; i++)
    {
        ret = ret && setPositionRaw(joints[i], refs[i]);
    }
    return ret;
}

bool iCubSimulationControl::setPositionsRaw(const double *refs)
{
    bool ret = true;
    for(int j=0; j<njoints; j++)
    {
        ret = ret && setPositionRaw(j, refs[j]);
    }
    return ret;
}

//PWM interface
bool iCubSimulationControl::setRefDutyCycleRaw(int j, double v)
{
    if ((j >= 0) && (j<njoints))
    {
        _mutex.wait();
        pwm_ref[j] = v;
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("setRefDutyCycleRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", j, njoints);
    return false;
}

bool iCubSimulationControl::setRefDutyCyclesRaw(const double *v)
{
    _mutex.wait();
    for (int axis = 0; axis<njoints; axis++)
        pwm_ref[axis] = v[axis];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getRefDutyCycleRaw(int j, double *v)
{
    if ((j >= 0) && (j<njoints))
    {
        _mutex.wait();
        *v = pwm_ref[j];
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("getRefDutyCycleRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", j, njoints);
    return false;
}

bool iCubSimulationControl::getRefDutyCyclesRaw(double *v)
{
    _mutex.wait();
    for (int axis = 0; axis<njoints; axis++)
        v[axis] = pwm_ref[axis];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getDutyCycleRaw(int j, double *v)
{
    if ((j >= 0) && (j<njoints))
    {
        _mutex.wait();
        *v = pwm[j];
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("getDutyCycleRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", j, njoints);
    return false;
}

bool iCubSimulationControl::getDutyCyclesRaw(double *v)
{
    _mutex.wait();
    for (int axis = 0; axis<njoints; axis++)
        v[axis] = pwm[axis];
    _mutex.post();
    return true;
}

// Current interface
/*bool iCubSimulationControl::getCurrentRaw(int j, double *t)
{
return NOT_YET_IMPLEMENTED("getCurrentRaw");
}

bool iCubSimulationControl::getCurrentsRaw(double *t)
{
return NOT_YET_IMPLEMENTED("getCurrentsRaw");
}
*/

bool iCubSimulationControl::getCurrentRangeRaw(int j, double *min, double *max)
{
    if ((j >= 0) && (j<njoints))
    {
        _mutex.wait();
        *min = -3;
        *max = 3;
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("setRefCurrentRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", j, njoints);
    return false;
}

bool iCubSimulationControl::getCurrentRangesRaw(double *min, double *max)
{
    _mutex.wait();
    for (int axis = 0; axis < njoints; axis++)
    {
        min[axis] = -3;
        max[axis] = 3;
    }
    _mutex.post();
    return true;
}

bool iCubSimulationControl::setRefCurrentsRaw(const double *t)
{
    _mutex.wait();
    for (int axis = 0; axis<njoints; axis++)
        current_ampere_ref[axis] = t[axis];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::setRefCurrentRaw(int j, double t)
{
    if ((j >= 0) && (j<njoints))
    {
        _mutex.wait();
        current_ampere_ref[j] = t;
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("setRefCurrentRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", j, njoints);
    return false;
}

bool iCubSimulationControl::setRefCurrentsRaw(const int n_joint, const int *joints, const double *t)
{
    bool ret = true;
    for (int j = 0; j<n_joint; j++)
    {
        ret = ret && setRefCurrentRaw(joints[j], t[j]);
    }
    return ret;
}

bool iCubSimulationControl::getRefCurrentsRaw(double *t)
{
    _mutex.wait();
    for (int axis = 0; axis<njoints; axis++)
        t[axis] = current_ampere_ref[axis];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getRefCurrentRaw(int j, double *t)
{
    if ((j >= 0) && (j<njoints))
    {
        _mutex.wait();
        *t = current_ampere_ref[j];
        _mutex.post();
        return true;
    }
    if (verbosity)
        yError("getRefCurrentRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", j, njoints);
    return false;
}

bool iCubSimulationControl::setCurrentPidRaw(int j, const Pid &pid)
{
    if ((j >= 0) && (j<njoints))
    {
        current_pid[j] = pid;
    }
    NOT_YET_IMPLEMENTED("setCurrentPidRaw");
    return true; //fake
}

bool iCubSimulationControl::setCurrentPidsRaw(const Pid *pids)
{
    for (int i = 0; i<njoints; i++)
    {
        current_pid[i] = pids[i];
    }
    NOT_YET_IMPLEMENTED("setCurrentPidsRaw");
    return true; //fake
}

bool iCubSimulationControl::getCurrentErrorRaw(int j, double *err)
{
    return NOT_YET_IMPLEMENTED("getCurrentErrorRaw");
}

bool iCubSimulationControl::getCurrentErrorsRaw(double *errs)
{
    return NOT_YET_IMPLEMENTED("getCurrentErrorsRaw");
}

bool iCubSimulationControl::getCurrentPidOutputRaw(int j, double *out)
{
    return NOT_YET_IMPLEMENTED("getCurrentPidOutputRaw");
}

bool iCubSimulationControl::getCurrentPidOutputsRaw(double *outs)
{
    return NOT_YET_IMPLEMENTED("getCurrentPidOutputsRaw");
}

bool iCubSimulationControl::getCurrentPidRaw(int j, Pid *pid)
{
    if ((j >= 0) && (j<njoints))
    {
        *pid = current_pid[j];
    }
    NOT_YET_IMPLEMENTED("getCurrentPidRaw");
    return true; //fake
}

bool iCubSimulationControl::getCurrentPidsRaw(Pid *pids)
{
    for (int i = 0; i<njoints; i++)
    {
        pids[i] = current_pid[i];
    }
    NOT_YET_IMPLEMENTED("getCurrentPidsRaw");
    return true; //fake
}

bool iCubSimulationControl::resetCurrentPidRaw(int j)
{
    return NOT_YET_IMPLEMENTED("resetCurrentPidRaw");
}

bool iCubSimulationControl::disableCurrentPidRaw(int j)
{
    return NOT_YET_IMPLEMENTED("disableCurrentPidRaw");
}

bool iCubSimulationControl::enableCurrentPidRaw(int j)
{
    return NOT_YET_IMPLEMENTED("enableCurrentPidRaw");
}
