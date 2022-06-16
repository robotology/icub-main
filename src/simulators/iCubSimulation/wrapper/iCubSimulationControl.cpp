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

#include <cmath>
#include <string>

///specific to this device driver.
#include "iCubSimulationControl.h"
#include "OdeInit.h"
#include <yarp/dev/ControlBoardHelper.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
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
    //PeriodicThread(0.01),
    ImplementPositionControl(this),
    ImplementVelocityControl(this),
    ImplementPidControl(this),
    ImplementEncodersTimed(this),
    ImplementTorqueControl(this),
    ImplementControlMode(this),
    ImplementControlCalibration(this),
    ImplementAmplifierControl(this),
    ImplementControlLimits(this),
    ImplementInteractionMode(this),
    ImplementPositionDirect(this),
    ImplementMotorEncoders(this),
    ImplementRemoteVariables(this),
    ImplementAxisInfo(this),
    ImplementMotor(this),
    ImplementPWMControl(this),
    ImplementCurrentControl(this)
{
    _opened = false;
    manager = NULL;
}


iCubSimulationControl::~iCubSimulationControl ()
{
    OdeInit& odeinit = OdeInit::get();
    lock_guard<mutex> lck(odeinit.mtx);
    odeinit.removeSimulationControl(partSelec);
}

bool iCubSimulationControl::open(yarp::os::Searchable& config) {

    Searchable& p = config;

    if (!p.check("GENERAL","section for general motor control parameters")) {
        yError("Cannot understand configuration parameters");
        return false;
    }

    int TypeArm = p.findGroup("GENERAL").check("Type",Value(1),
                                          "what did the user select?").asInt32();

    int numTOTjoints = p.findGroup("GENERAL").check("TotalJoints",Value(1),
                                          "Number of total joints").asInt32();

    double velocity = p.findGroup("GENERAL").check("Vel",Value(1),
                                          "Default velocity").asFloat64();
    _mutex.lock();
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
//  jointNames = new string[njoints];

    current_jnt_pos = allocAndCheck<double>(njoints);
    current_jnt_vel = allocAndCheck<double>(njoints);
    current_jnt_acc = allocAndCheck<double>(njoints);
    estimated_jnt_vel = allocAndCheck<double>(njoints);
    estimated_jnt_acc = allocAndCheck<double>(njoints);
    current_jnt_torques = allocAndCheck<double>(njoints);
    current_mot_pos = allocAndCheck<double>(njoints);
    current_mot_vel = allocAndCheck<double>(njoints);
    current_mot_acc = allocAndCheck<double>(njoints);
    estimated_mot_vel = allocAndCheck<double>(njoints);
    estimated_mot_acc = allocAndCheck<double>(njoints);
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
    velocity_pid = allocAndCheck <Pid>(njoints);
    motor_torque_params = allocAndCheck <MotorTorqueParameters>(njoints);
    rotToEncoder = allocAndCheck<double>(njoints);
    gearbox = allocAndCheck<double>(njoints);
    hasHallSensor = allocAndCheck<bool>(njoints);
    hasTempSensor = allocAndCheck<bool>(njoints);
    hasRotorEncoder = allocAndCheck<bool>(njoints);
    rotorIndexOffset = allocAndCheck<int>(njoints);
    motorPoles = allocAndCheck<int>(njoints);
    linEstJnt = new iCub::ctrl::AWLinEstimator(25, 2.0);
    quadEstJnt = new iCub::ctrl::AWQuadEstimator(50, 2.0);
    linEstMot = new iCub::ctrl::AWLinEstimator(25, 2.0);
    quadEstMot = new iCub::ctrl::AWQuadEstimator(50, 2.0);

//  joint_dev = new DeviceTag[njoints];

    motor_on = allocAndCheck<bool>(njoints);
    for(int axis = 0;axis<njoints;axis++)
        motor_on[axis] = false;

    /////////////////////////
    /*   GENERAL           */
    ///////////////////////// 

    for (int i = 0; i < njoints; i++) axisMap[i] = i;

    Bottle& xtmp = p.findGroup("GENERAL").findGroup("Encoder","a list of scales for the encoders");
    if (xtmp.size() != njoints+1) {
        yError("Encoder does not have the right number of entries\n");
        return false;
    }
    for (int i = 1; i < xtmp.size(); i++) angleToEncoder[i-1] = xtmp.get(i).asFloat64();

    xtmp = p.findGroup("GENERAL").findGroup("fullscalePWM", "a list of scales for the fullscalePWM param");
    if (xtmp.size() != njoints + 1) {
        yError("fullscalePWM does not have the right number of entries\n");
        return false;
    }
    for (int i = 1; i < xtmp.size(); i++) dutycycleToPwm[i - 1] = xtmp.get(i).asFloat64()/100.0;

    xtmp = p.findGroup("GENERAL").findGroup("ampsToSensor", "a list of scales for the ampsToSensor param");
    if (xtmp.size() != njoints + 1) {
        yError("ampsToSensor does not have the right number of entries\n");
        return false;
    }
    for (int i = 1; i < xtmp.size(); i++) ampsToSensor[i - 1] = xtmp.get(i).asFloat64();

    xtmp = p.findGroup("GENERAL").findGroup("Zeros","a list of offsets for the zero point");
    if (xtmp.size() != njoints+1) {
        yError("Zeros does not have the right number of entries\n");
        return false;
    }
    for (int i = 1; i < xtmp.size(); i++) zeros[i-1] = xtmp.get(i).asFloat64();

    int mj_size = p.findGroup("GENERAL").check("Kinematic_mj_size",Value(0),"Default velocity").asInt32();
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
                kinematic_mj[r][c] = xtmp.get(e).asFloat64();
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
        limitsMax[i-1] = xtmp.get(i).asFloat64()*angleToEncoder[i-1];
    
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
        limitsMin[i-1] = xtmp.get(i).asFloat64()*angleToEncoder[i-1];

    xtmp = p.findGroup("LIMITS").findGroup("error_tol","error tolerance during tracking");
    if(xtmp.size() != njoints+1)
        {
            yError("Not enough error_tol\n");
            return false;
        }
    for(int i=1;i<xtmp.size();i++)
        error_tol[i-1] = xtmp.get(i).asFloat64()*angleToEncoder[i-1];

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

    ImplementPositionControl::initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementVelocityControl::initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementPidControl::initialize(njoints, axisMap, angleToEncoder, zeros,newtonsToSensor,ampsToSensor, dutycycleToPwm);
    ImplementEncodersTimed::initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementMotorEncoders::initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementControlCalibration::initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementAmplifierControl::initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementControlLimits::initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementTorqueControl::initialize(njoints, axisMap, angleToEncoder, zeros, newtonsToSensor, ampsToSensor, dutycycleToPwm,nullptr,nullptr);
    ImplementControlMode::initialize(njoints, axisMap);
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
    odeinit.mtx.lock();
    odeinit.setSimulationControl(this, partSelec);
    odeinit.mtx.unlock();
    //PeriodicThread::start();
    _mutex.unlock();
    _opened = true;

    verbosity = odeinit.verbosity;
    return true;
}

bool iCubSimulationControl::close (void)
{
    if (_opened) {

        //if (PeriodicThread::isRunning())
        //    {
        //    }
        
        //PeriodicThread::stop();/// stops the thread first (joins too).
        
        ImplementPositionControl::uninitialize();
        ImplementVelocityControl::uninitialize();
        ImplementTorqueControl::uninitialize();
        ImplementPidControl::uninitialize();
        ImplementEncodersTimed::uninitialize();
        ImplementMotorEncoders::uninitialize();
        ImplementControlCalibration::uninitialize();
        ImplementAmplifierControl::uninitialize();
        ImplementControlLimits::uninitialize();
        ImplementControlMode::uninitialize();
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
    checkAndDestroy<double>(current_jnt_acc);
    checkAndDestroy<double>(current_mot_acc);
    checkAndDestroy<double>(estimated_jnt_vel);
    checkAndDestroy<double>(estimated_mot_vel);
    checkAndDestroy<double>(estimated_jnt_acc);
    checkAndDestroy<double>(estimated_mot_acc);
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
    checkAndDestroy<Pid>(velocity_pid);
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

    delete linEstJnt;
    delete quadEstJnt;
    delete linEstMot;
    delete quadEstMot;

    _opened = false;
    return true;
}

void iCubSimulationControl::compute_mot_pos_from_jnt_pos(double *mot_pos, const double *jnt_pos, int size_joints)
{
    for (int i = 0; i < size_joints; i++)
    {
        mot_pos[i] = jnt_pos[i]; //use coupling matrix here
    }
}

void iCubSimulationControl::compute_mot_vel_and_acc(double *mot_vel, double *mot_acc, const double *mot_pos, int size_joints)
{
    iCub::ctrl::AWPolyElement el(yarp::sig::Vector(size_joints, mot_pos), Time::now());
    yarp::sig::Vector speeds = linEstMot->estimate(el);
    yarp::sig::Vector accs = quadEstMot->estimate(el);
    for (int i = 0; i < size_joints; i++) mot_vel[i] = speeds[i];
    for (int i = 0; i < size_joints; i++) mot_acc[i] = accs[i];
}

void iCubSimulationControl::compute_jnt_vel_and_acc(double *jnt_vel, double *jnt_acc, const double *jnt_pos, int size_joints)
{
    iCub::ctrl::AWPolyElement el(yarp::sig::Vector(size_joints, jnt_pos), Time::now());
    yarp::sig::Vector speeds = linEstJnt->estimate(el);
    yarp::sig::Vector accs = quadEstJnt->estimate(el);
    for (int i = 0; i < size_joints; i++) jnt_vel[i] = speeds[i];
    for (int i = 0; i < size_joints; i++) jnt_acc[i] = accs[i];
}

void iCubSimulationControl::jointStep() {
    lock_guard<mutex> lck(_mutex);
    if (manager==NULL) {
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
        compute_mot_pos_from_jnt_pos(current_mot_pos, current_jnt_pos, njoints);
        compute_mot_vel_and_acc(estimated_mot_vel, estimated_mot_acc, current_mot_pos, njoints);
        compute_jnt_vel_and_acc(estimated_jnt_vel, estimated_jnt_acc, current_jnt_pos, njoints);
        for (int axis = 0; axis < njoints; axis++)
        {
            current_mot_acc[axis] = estimated_mot_acc[axis];
            current_jnt_acc[axis] = estimated_jnt_acc[axis];
        }
    }
}

bool iCubSimulationControl::getAxes(int *ax)
{
    *ax = njoints;
    return true;
}

bool iCubSimulationControl::setPidRaw (const PidControlTypeEnum& pidtype, int j, const Pid &pid)
{
   if( (j>=0) && (j<njoints) )
   {
       switch (pidtype)
       {
           case VOCAB_PIDTYPE_POSITION:
               position_pid[j]=pid;
           break;
           case VOCAB_PIDTYPE_VELOCITY:
               velocity_pid[j]=pid;
           break;
           case VOCAB_PIDTYPE_CURRENT:
               current_pid[j]=pid;
           break;
           case VOCAB_PIDTYPE_TORQUE:
              torque_pid[j]=pid;
           break;
           default:
           break;
       }
   }
   return true;
}

bool iCubSimulationControl::getPidRaw (const PidControlTypeEnum& pidtype, int j, Pid *out)
{
   if( (j>=0) && (j<njoints) )
   {
       switch (pidtype)
       {
           case VOCAB_PIDTYPE_POSITION:
               *out=position_pid[j];
           break;
           case VOCAB_PIDTYPE_VELOCITY:
               *out=velocity_pid[j];
           break;
           case VOCAB_PIDTYPE_CURRENT:
               *out=current_pid[j];
           break;
           case VOCAB_PIDTYPE_TORQUE:
               *out=torque_pid[j];
           break;
           default:
           break;
       }
   }
   return true;
}

bool iCubSimulationControl::getPidsRaw (const PidControlTypeEnum& pidtype, Pid *out)
{
    for (int i=0; i<njoints; i++)
    {
       getPidRaw(pidtype, i, &out[i]);
    }
    return true;
}

bool iCubSimulationControl::setPidsRaw(const PidControlTypeEnum& pidtype, const Pid *pids)
{
    for (int i=0; i<njoints; i++)
    {
        setPidRaw(pidtype,i,pids[i]);
    }
    return true;
}

/// cmd is a SingleAxis pointer with 1 double arg
bool iCubSimulationControl::setPidReferenceRaw (const PidControlTypeEnum& pidtype, int axis, double ref)
{
    if( (axis>=0) && (axis<njoints) )
    {
       int mode = 0;
       getControlModeRaw(axis, &mode);
       switch (pidtype)
       {
           case VOCAB_PIDTYPE_POSITION:
               NOT_YET_IMPLEMENTED("setPidReferenceRaw");
           break;
           case VOCAB_PIDTYPE_VELOCITY:
               NOT_YET_IMPLEMENTED("setPidReferenceRaw");
           break;
           case VOCAB_PIDTYPE_CURRENT:
               NOT_YET_IMPLEMENTED("setPidReferenceRaw");
           break;
           case VOCAB_PIDTYPE_TORQUE:
               NOT_YET_IMPLEMENTED("setPidReferenceRaw");
           break;
           default:
           break;
       }
       return true;
    }
    if (verbosity)
    {
        yError("setReferenceRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    }
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
        lock_guard<mutex> lck(_mutex);
        switch (pidtype)
        {
            case VOCAB_PIDTYPE_POSITION:
            *err = current_jnt_pos[axis] - next_pos[axis];
            break;
            case VOCAB_PIDTYPE_TORQUE:
            *err = current_jnt_torques[axis] - next_torques[axis];
            break;
            case VOCAB_PIDTYPE_VELOCITY:
            *err = current_jnt_vel[axis] - next_vel[axis];
            break;
            case VOCAB_PIDTYPE_CURRENT:
            *err = current_ampere[axis] - current_ampere_ref[axis];
            break;
        }
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
       int mode = 0;
       getControlModeRaw(axis, &mode);
       switch (pidtype)
       {
           case VOCAB_PIDTYPE_POSITION:
           if (mode == VOCAB_CM_POSITION_DIRECT ||
               mode == VOCAB_CM_POSITION ||
               mode == VOCAB_CM_MIXED)
           {
               lock_guard<mutex> lck(_mutex);
               *out = pwm[axis];
           }
           else
           {
               lock_guard<mutex> lck(_mutex);
               *out = 0;
           }
           break;
           case VOCAB_PIDTYPE_VELOCITY:
           if (mode == VOCAB_CM_VELOCITY)
           {
               lock_guard<mutex> lck(_mutex);
               *out = pwm[axis];
           }
           else
           {
               lock_guard<mutex> lck(_mutex);
               *out = 0;
           }
           break;
           case VOCAB_PIDTYPE_CURRENT:
           if (mode == VOCAB_CM_CURRENT)
           {
               lock_guard<mutex> lck(_mutex);
               *out = pwm[axis];
           }
           else
           {
               lock_guard<mutex> lck(_mutex);
               *out = 0;
           }
           break;
           case VOCAB_PIDTYPE_TORQUE:
           if (mode == VOCAB_CM_TORQUE)
           {
               lock_guard<mutex> lck(_mutex);
               *out = pwm[axis];
           }
           else
           {
               lock_guard<mutex> lck(_mutex);
               *out = 0;
           }
           break;
           default:
           {
               lock_guard<mutex> lck(_mutex);
               *out = 0;
           }
           break;
       }
    }
    if (verbosity)
    {
        yError("getOutputRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    }
    return false;
}

bool iCubSimulationControl::getPidOutputsRaw(const PidControlTypeEnum& pidtype, double *outs)
{
    lock_guard<mutex> lck(_mutex);
    for(int axis = 0; axis<njoints; axis++)
        outs[axis] = pwm[axis];
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
       int mode = 0;
       getControlModeRaw(axis, &mode);
       lock_guard<mutex> lck(_mutex);
       switch (pidtype)
       {
           case VOCAB_PIDTYPE_POSITION:
               *ref = next_pos[axis];
           break;
           case VOCAB_PIDTYPE_VELOCITY:
               *ref = next_vel[axis];
           break;
           case VOCAB_PIDTYPE_CURRENT:
               *ref = current_ampere_ref[axis];
           break;
           case VOCAB_PIDTYPE_TORQUE:
               *ref = next_torques[axis];
           break;
           default:
           break;
       }
       return true;
    }
    if (verbosity)
    {
        yError("getReferenceRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    }
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

bool iCubSimulationControl::positionMoveRaw(int axis, double ref)
{
    if( (axis >=0) && (axis<njoints) )
    {
        int mode = 0;
        getControlModeRaw(axis, &mode);
        if (mode != VOCAB_CM_POSITION &&
            mode != VOCAB_CM_MIXED  )
        {
            yError() << "positionMoveRaw: skipping command because part " << partSelec << " joint " << axis << "is not in VOCAB_CM_POSITION mode";
            return false;
        }

        lock_guard<mutex> lck(_mutex);
        if(ref< limitsMin[axis])
        {
            if (njoints == 16)
            {
                if ( axis == 7 ) //fingers abduction
                    next_pos[axis] = limitsMax[axis]; //add explanation here
                else
                    next_pos[axis] = limitsMin[axis];
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
                if ( axis == 7 ) //fingers abduction
                    next_pos[axis] = fabs(limitsMax[axis] - limitsMax[axis]); //add explanation here
                else
                    next_pos[axis] = limitsMax[axis];
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
    lock_guard<mutex> lck(_mutex);
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
    return true;
}

bool iCubSimulationControl::checkMotionDoneRaw(int axis, bool *ret)
{
    if( (axis >=0) && (axis<njoints) )
        {
            lock_guard<mutex> lck(_mutex);
            if(fabs(current_jnt_pos[axis]-next_pos[axis])<error_tol[axis])
                *ret = true;
            else
                *ret = false;
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
            lock_guard<mutex> lck(_mutex);
            //vel = sp;// *180/M_PI ;
            vels[axis] = sp;//vel/20;
            ref_speeds[axis] = sp;
            if (verbosity)
                yDebug("setting joint %d of part %d to reference velocity %f\n",axis,partSelec,vels[axis]);
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
        lock_guard<mutex> lck(_mutex);
        *ref = ref_speeds[axis];
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
        lock_guard<mutex> lck(_mutex);
        next_pos[axis] = current_jnt_pos[axis];
        next_vel[axis] = 0.0;
        return true;
    }
    if (verbosity)
        yError("stopRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}
bool iCubSimulationControl::stopRaw()
{
    lock_guard<mutex> lck(_mutex);
    for(int axis=0;axis<njoints;axis++)
    {
        next_pos[axis] = current_jnt_pos[axis];
        next_vel[axis] = 0.0;
    }
    return true;
}

bool iCubSimulationControl::getTargetPositionRaw(int axis, double *ref)
{
    if ((axis >= 0) && (axis<njoints)) {
        lock_guard<mutex> lck(_mutex);
        *ref = ref_command_positions[axis];
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
    lock_guard<mutex> lck(_mutex);
    *ref = ref_command_speeds[axis];
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
    lock_guard<mutex> lck(_mutex);
    *ref = ref_positions[axis];
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
            yError() << "velocityMoveRaw: skipping command because part " << partSelec << " joint " << axis << "is not in VOCAB_CM_VELOCITY mode";
            return false;
        }
        lock_guard<mutex> lck(_mutex);
        next_vel[axis] = sp;
        ref_command_speeds[axis] = sp;
        motor_on[axis] = true;
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
   lock_guard<mutex> lck(_mutex);
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
    return true;
}

bool iCubSimulationControl::getEncoderRaw(int axis, double *v)
{
    if((axis>=0) && (axis<njoints)) {
        lock_guard<mutex> lck(_mutex);
        
        if ( axis == 10 ||  axis == 12 || axis == 14 ) 
            *v = current_jnt_pos[axis]*2;
        else if ( axis == 15 ) 
            *v = current_jnt_pos[axis]*3;
        else if ( axis == 7 ) 
            *v = limitsMax[axis] - current_jnt_pos[axis];
        else 
            *v = current_jnt_pos[axis];

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
   lock_guard<mutex> lck(_mutex);
   for(int axis = 0; axis<njoints; axis++)
       v[axis] = current_jnt_vel[axis];//* 10;
   return true;
}

bool iCubSimulationControl::getEncoderSpeedRaw(int axis, double *v)
{
    if( (axis>=0) && (axis<njoints) ) {
        lock_guard<mutex> lck(_mutex);
        *v = current_jnt_vel[axis];// * 10;
        return true;
    }
    if (verbosity)
        yError("getEncoderSpeedRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::getEncoderAccelerationsRaw(double *v)
{
    lock_guard<mutex> lck(_mutex);
    for (int axis = 0; axis<njoints; axis++)
        v[axis] = current_jnt_acc[axis];//* 10;
    return true;
}

bool iCubSimulationControl::getEncoderAccelerationRaw(int axis, double *v)
{
    if ((axis >= 0) && (axis<njoints)) {
        lock_guard<mutex> lck(_mutex);
        *v = current_jnt_acc[axis];// * 10;
        return true;
    }
    if (verbosity)
        yError("getEncoderSpeedRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
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
    for(int axis = 0;axis<njoints;axis++)
    {
        getMotorEncoderRaw(axis,&v[axis]);
    }
    return true;
}

bool iCubSimulationControl::getMotorEncoderRaw(int axis, double *v)
{
    if((axis>=0) && (axis<njoints))
    {
        lock_guard<mutex> lck(_mutex);
        *v = current_mot_pos[axis];
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
    for(int axis = 0; axis<njoints; axis++)
    {
        getMotorEncoderSpeedRaw(axis,&v[axis]);
    }
    return true;
}

bool iCubSimulationControl::getMotorEncoderSpeedRaw(int axis, double *v)
{
    if( (axis>=0) && (axis<njoints) ) {
        lock_guard<mutex> lck(_mutex);
        *v = current_mot_vel[axis];
        return true;
    }
    if (verbosity)
        yError("getEncoderSpeedRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::getMotorEncoderAccelerationsRaw(double *v)
{
    for (int axis = 0; axis<njoints; axis++)
    {
        getMotorEncoderAccelerationRaw(axis, &v[axis]);
    }
    return true;
}

bool iCubSimulationControl::getMotorEncoderAccelerationRaw(int axis, double *v)
{
    if ((axis >= 0) && (axis<njoints)) {
        lock_guard<mutex> lck(_mutex);
        *v = current_mot_acc[axis];
        return true;
    }
    if (verbosity)
        yError("getEncoderSpeedRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
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
   lock_guard<mutex> lck(_mutex);
   for(int axis = 0; axis<njoints; axis++)
       cs[axis] = current_ampere[axis];
   return true;
}

// bcast currents
bool iCubSimulationControl::getCurrentRaw(int axis, double *c)
{
    if( (axis>=0) && (axis<njoints) )
    {
        lock_guard<mutex> lck(_mutex);
        *c = current_ampere[axis];
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
        lock_guard<mutex> lck(_mutex);
        maxCurrent[axis]=v;
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
        lock_guard<mutex> lck(_mutex);
        *v=maxCurrent[axis];
    }
    else
    {  
        if (verbosity)
            yError("getMaxCurrentRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    }
    return true;
}

bool iCubSimulationControl::calibrateAxisWithParamsRaw(int axis, unsigned int type, double p1, double p2, double p3)
{
    return NOT_YET_IMPLEMENTED("calibrateRaw");
}

bool iCubSimulationControl::calibrationDoneRaw(int axis)
{
    return NOT_YET_IMPLEMENTED("doneRaw");
}


bool iCubSimulationControl::getAmpStatusRaw(int *st)
{
    lock_guard<mutex> lck(_mutex);
    for(int axis =0;axis<njoints;axis++)
        st[axis] = (int)motor_on[axis];
    return true;
}

bool iCubSimulationControl::getAmpStatusRaw(int axis, int *st)
{
    if( (axis>=0) && (axis<njoints)) 
    {
        lock_guard<mutex> lck(_mutex);
        st[axis] = (int)motor_on[axis];
        return true;
    }
    if (verbosity)
        yError("getAmpStatusRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::setLimitsRaw(int axis, double min, double max)
{
    if( (axis >=0) && (axis < njoints) ){
        lock_guard<mutex> lck(_mutex);
        limitsMax[axis] = max;
        limitsMin[axis] = min;
       return true;
    }
    if (verbosity)
        yError("setLimitsRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::getLimitsRaw(int axis, double *min, double *max)
{
    if( (axis >=0) && (axis < njoints)) {
        lock_guard<mutex> lck(_mutex);
        *min = limitsMin[axis];
        *max = limitsMax[axis];
        return true;
     }
     //else
     return false;
}

// IRemoteVariables
bool iCubSimulationControl::getRemoteVariableRaw(string key, yarp::os::Bottle& val)
{
    val.clear();
    if (key == "kinematic_mj")
    {
        val.addString("1 2 3"); return true;
    }
    else if (key == "rotor")
    {
        Bottle& r = val.addList(); for (int i=0; i<njoints; i++) r.addFloat64(rotToEncoder[i]); return true;
    }
    else if (key == "gearbox")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addFloat64(gearbox[i]); return true;
    }
    else if (key == "zeros")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addFloat64(zeros[i]); return true;
    }
    else if (key == "hasHallSensor")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addInt32(hasHallSensor[i]); return true;
    }
    else if (key == "hasTempSensor")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addInt32(hasTempSensor[i]); return true;
    }
    else if (key == "hasRotorEncoder")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addInt32(hasRotorEncoder[i]); return true;
    }
    else if (key == "rotorIndexOffset")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addInt32(rotorIndexOffset[i]); return true;
    }
    else if (key == "motorPoles")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addInt32(motorPoles[i]); return true;
    }
    else if (key == "pidCurrentKp")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addFloat64(current_pid[i].kp); return true;
    }
    else if (key == "pidCurrentKi")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addFloat64(current_pid[i].ki); return true;
    }
    else if (key == "pidCurrentShift")
    {
        Bottle& r = val.addList(); for (int i = 0; i<njoints; i++) r.addFloat64(current_pid[i].scale); return true;
    }
    yWarning("getRemoteVariable(): Unknown variable %s", key.c_str());
    return false;
}

bool iCubSimulationControl::setRemoteVariableRaw(string key, const yarp::os::Bottle& val)
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
            rotToEncoder[i] = bval->get(i).asFloat64();
        return true;
    }
    else if (key == "gearbox")
    {
        for (int i = 0; i < njoints; i++) gearbox[i] = bval->get(i).asFloat64(); return true;
    }
    else if (key == "zeros")
    {
        for (int i = 0; i < njoints; i++) zeros[i] = bval->get(i).asFloat64(); return true;
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

bool iCubSimulationControl::setVelLimitsRaw(int axis, double min, double max)
{
    if ((axis >= 0) && (axis < njoints)){
        lock_guard<mutex> lck(_mutex);
        velLimitsMax[axis] = max;
        velLimitsMin[axis] = min;
        return true;
    }
    if (verbosity)
        yError("setVelLimitsRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::getVelLimitsRaw(int axis, double *min, double *max)
{
    if ((axis >= 0) && (axis < njoints)) {
        lock_guard<mutex> lck(_mutex);
        *min = velLimitsMin[axis];
        *max = velLimitsMax[axis];
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

bool iCubSimulationControl::getAxisNameRaw(int axis, string& name)
{
    if ((axis >= 0) && (axis < njoints))
    {
        lock_guard<mutex> lck(_mutex);
        char buff[100];
        sprintf(buff,"JOINT%d", axis);
        name.assign(buff,strlen(buff));
        return true;
    }
    
    return false;
}

bool iCubSimulationControl::getJointTypeRaw(int axis, yarp::dev::JointTypeEnum& type)
{
    if ((axis >= 0) && (axis < njoints))
    {
        lock_guard<mutex> lck(_mutex);
        type = yarp::dev::VOCAB_JOINTTYPE_REVOLUTE;
        return true;
    }

    return false;
}

bool iCubSimulationControl::getTorqueRaw(int axis, double *sp)
{
    if( (axis >=0) && (axis < njoints)) {
        lock_guard<mutex> lck(_mutex);
        *sp = current_jnt_torques[axis];
        return true;
    }
    return false;
}
bool iCubSimulationControl::getTorquesRaw(double *sp)
{
   lock_guard<mutex> lck(_mutex);
   for(int axis = 0;axis<njoints;axis++)
       sp[axis] = current_jnt_torques[axis];
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
        lock_guard<mutex> lck(_mutex);
        next_torques[axis] = ref;
        ref_torques[axis] = ref;
        motor_on[axis] = true;
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
        lock_guard<mutex> lck(_mutex);
        *ref = next_torques[axis];
        return true;
    }
    if (verbosity)
        yError("getRefTorqueRaw: joint with index %d does not exist, valis joints indices are between 0 and %d \n",axis,njoints);
    return false;
}

bool iCubSimulationControl::getMotorTorqueParamsRaw(int axis, MotorTorqueParameters *params)
{
    if( (axis >=0) && (axis<njoints) )
    {
         params->bemf=motor_torque_params[axis].bemf;
         params->bemf_scale=motor_torque_params[axis].bemf_scale;
         params->ktau=motor_torque_params[axis].ktau;
         params->ktau_scale=motor_torque_params[axis].ktau_scale;
         params->viscousPos=motor_torque_params[axis].viscousPos;
         params->viscousNeg=motor_torque_params[axis].viscousNeg;
         params->coulombPos=motor_torque_params[axis].coulombPos;
         params->coulombNeg=motor_torque_params[axis].coulombNeg;
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
        motor_torque_params[axis].viscousPos=params.viscousPos;
        motor_torque_params[axis].viscousNeg=params.viscousNeg;
        motor_torque_params[axis].coulombPos=params.coulombPos;
        motor_torque_params[axis].coulombNeg=params.coulombNeg;
    }
    NOT_YET_IMPLEMENTED("setMotorTorqueParamsRaw");
    return true; //fake
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
        lock_guard<mutex> lck(_mutex);
        *mode = ControlModes_iCubSIM2yarp(controlMode[j]);
        return true;
    }
    if (verbosity)
        yError("getControlModeRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", j, njoints);
    return false; 
}

bool iCubSimulationControl::getControlModesRaw(int* modes)
{
   lock_guard<mutex> lck(_mutex);
   for(int axis = 0;axis<njoints;axis++)
       modes[axis] = ControlModes_iCubSIM2yarp(controlMode[axis]);
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
            yError() << "setControlModeRaw: unknown control mode " << yarp::os::Vocab32::decode(mode);
        }
        else if (controlMode[j] == VOCAB_CM_HW_FAULT && mode != VOCAB_CM_FORCE_IDLE)
        {
            yError() << "setControlModeRaw: unable to reset an HW_FAULT without a VOCAB_CM_FORCE_IDLE command";
        }
        else
        {
            lock_guard<mutex> lck(_mutex);
            controlMode[j] = ControlModes_yarp2iCubSIM(mode);
            next_pos[j]=current_jnt_pos[j];
            if (controlMode[j] != MODE_PWM) pwm_ref[j] = 0;
            if (controlMode[j] != MODE_CURRENT) current_ampere_ref[j] = 0;
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
bool iCubSimulationControl::setPositionRaw(int axis, double ref)
{
    // This function has been copy pasted from positionMoveRaw
    if( (axis >=0) && (axis<njoints) )
    {
        int mode = 0;
        getControlModeRaw(axis, &mode);
        if (mode != VOCAB_CM_POSITION_DIRECT)
        {
            yError() << "setPositionRaw: skipping command because part " << partSelec << " joint" << axis << "is not in VOCAB_CM_POSITION_DIRECT mode";
            return false;
        }
        lock_guard<mutex> lck(_mutex);
        if(ref< limitsMin[axis])
        {
            if (njoints == 16)
            {
                if ( axis == 7 ) //fingers abduction
                    next_pos[axis] = limitsMax[axis]; //add explanation here
                else
                    next_pos[axis] = limitsMin[axis];
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
                if ( axis == 7 ) //fingers abduction
                    next_pos[axis] = fabs(limitsMax[axis] - limitsMax[axis]); //add explanation here
                else
                    next_pos[axis] = limitsMax[axis];
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
        return true;
    }
    if (verbosity)
        yError("setPositionRaw joint access too high %d \n",axis);
    return false;
}

bool iCubSimulationControl::setPositionsRaw(const int n_joint, const int *joints, const double *refs)
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
        lock_guard<mutex> lck(_mutex);
        pwm_ref[j] = v;
        return true;
    }
    if (verbosity)
        yError("setRefDutyCycleRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", j, njoints);
    return false;
}

bool iCubSimulationControl::setRefDutyCyclesRaw(const double *v)
{
    lock_guard<mutex> lck(_mutex);
    for (int axis = 0; axis<njoints; axis++)
        pwm_ref[axis] = v[axis];
    return true;
}

bool iCubSimulationControl::getRefDutyCycleRaw(int j, double *v)
{
    if ((j >= 0) && (j<njoints))
    {
        lock_guard<mutex> lck(_mutex);
        *v = pwm_ref[j];
        return true;
    }
    if (verbosity)
        yError("getRefDutyCycleRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", j, njoints);
    return false;
}

bool iCubSimulationControl::getRefDutyCyclesRaw(double *v)
{
    lock_guard<mutex> lck(_mutex);
    for (int axis = 0; axis<njoints; axis++)
        v[axis] = pwm_ref[axis];
    return true;
}

bool iCubSimulationControl::getDutyCycleRaw(int j, double *v)
{
    if ((j >= 0) && (j<njoints))
    {
        lock_guard<mutex> lck(_mutex);
        *v = pwm[j];
        return true;
    }
    if (verbosity)
        yError("getDutyCycleRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", j, njoints);
    return false;
}

bool iCubSimulationControl::getDutyCyclesRaw(double *v)
{
    lock_guard<mutex> lck(_mutex);
    for (int axis = 0; axis<njoints; axis++)
        v[axis] = pwm[axis];
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
        lock_guard<mutex> lck(_mutex);
        *min = -3;
        *max = 3;
        return true;
    }
    if (verbosity)
        yError("setRefCurrentRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", j, njoints);
    return false;
}

bool iCubSimulationControl::getCurrentRangesRaw(double *min, double *max)
{
    lock_guard<mutex> lck(_mutex);
    for (int axis = 0; axis < njoints; axis++)
    {
        min[axis] = -3;
        max[axis] = 3;
    }
    return true;
}

bool iCubSimulationControl::setRefCurrentsRaw(const double *t)
{
    lock_guard<mutex> lck(_mutex);
    for (int axis = 0; axis<njoints; axis++)
        current_ampere_ref[axis] = t[axis];
    return true;
}

bool iCubSimulationControl::setRefCurrentRaw(int j, double t)
{
    if ((j >= 0) && (j<njoints))
    {
        lock_guard<mutex> lck(_mutex);
        current_ampere_ref[j] = t;
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
    lock_guard<mutex> lck(_mutex);
    for (int axis = 0; axis<njoints; axis++)
        t[axis] = current_ampere_ref[axis];
    return true;
}

bool iCubSimulationControl::getRefCurrentRaw(int j, double *t)
{
    if ((j >= 0) && (j<njoints))
    {
        lock_guard<mutex> lck(_mutex);
        *t = current_ampere_ref[j];
        return true;
    }
    if (verbosity)
        yError("getRefCurrentRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", j, njoints);
    return false;
}
