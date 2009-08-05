// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006 Paul Fitzpatrick, Giorgio Metta
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */
///                                                                   ///
/// The complete license description is contained in the              ///
/// licence.template file included in this distribution in            ///
/// $YARP_ROOT/conf. Please refer to this file for complete           ///
/// information about the licensing of YARP                           ///
///                                                                   ///
/// DISCLAIMERS: LICENSOR WARRANTS THAT THE COPYRIGHT IN AND TO THE   ///
/// SOFTWARE IS OWNED BY THE LICENSOR OR THAT THE SOFTWARE IS         ///
/// DISTRIBUTED BY LICENSOR UNDER A VALID CURRENT LICENSE. EXCEPT AS  ///
/// EXPRESSLY STATED IN THE IMMEDIATELY PRECEDING SENTENCE, THE       ///
/// SOFTWARE IS PROVIDED BY THE LICENSOR, CONTRIBUTORS AND COPYRIGHT  ///
/// OWNERS "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, ///
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   ///
/// FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO      ///
/// EVENT SHALL THE LICENSOR, CONTRIBUTORS OR COPYRIGHT OWNERS BE     ///
/// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   ///
/// ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN ///
/// CONNECTION WITH THE SOFTWARE.                                     ///
///                                                                   ///
/////////////////////////////////////////////////////////////////////////

///
/// $Id: iCubWebotsMotionControl.cpp,v 1.1 2007/07/24 12:25:10 tdahl Exp $
///
///

/// general purpose stuff.
#include <yarp/os/Time.h>
#include <ace/config.h>
#include <ace/OS.h>
#include <ace/Log_Msg.h>
#include <ace/Sched_Params.h>
#include <ace/String_Base.h>

#include <math.h>

/// specific to this device driver.
#include "iCubWebotsMotionControl.h"

#include "../src/ControlBoardInterfacesImpl.inl"


using namespace yarp::os;
using namespace yarp::dev;

const int POSITION_MODE = 0;
const int VELOCITY_MODE = 1;
// The three values below should be defined in Webots but I couldn't find them
// The Webots manual says maxVelocity should be 10, but I found that limiting
// From inspection 360 seems to be top speed
// The Position is in rads, so I aproxed two full rotations 8*pi ~= 25
// as max and min for position
const int maxVelocity = 360; 
const double maxPosition = SERVO_INFINITY;
const double minPosition = -SERVO_INFINITY;

bool NOT_YET_IMPLEMENTED(const char *txt)
{
    ACE_OS::fprintf(stderr, "%s not yet implemented for iCubWebotsMotionControl\n", txt);

    return false;
}


//////////////////////////////////

iCubWebotsMotionControl::iCubWebotsMotionControl() : 
    ImplementPositionControl<iCubWebotsMotionControl, IPositionControl>(this),
    ImplementVelocityControl<iCubWebotsMotionControl, IVelocityControl>(this),
    ImplementPidControl<iCubWebotsMotionControl, IPidControl>(this),
    ImplementEncoders<iCubWebotsMotionControl, IEncoders>(this),
    ImplementControlCalibration<iCubWebotsMotionControl, IControlCalibration>(this),
    ImplementAmplifierControl<iCubWebotsMotionControl, IAmplifierControl>(this),
    ImplementControlLimits<iCubWebotsMotionControl, IControlLimits>(this),
    _done(0),
    _mutex(1)
{
    _opened = false;
    ifmode = POSITION_MODE;
}


iCubWebotsMotionControl::~iCubWebotsMotionControl ()
{
}



bool iCubWebotsMotionControl::open(yarp::os::Searchable& config) {
    Searchable& p = config;

    
    if (!p.check("GENERAL","section for general motor control parameters")) {
        ACE_OS::fprintf(stderr, "Cannot understand configuration parameters\n");
        return false;
    }

    int nj = p.findGroup("GENERAL").check("Joints",Value(1),
                                          "Number of degrees of freedom").asInt();

    int i;

    _mutex.wait();
    

    njoints = nj;
    
    angleToEncoder = allocAndCheck<double>(njoints);
    zeros = allocAndCheck<double>(njoints);
    
    limitsMin = allocAndCheck<double>(njoints);
    limitsMax = allocAndCheck<double>(njoints);
    torqueLimits = allocAndCheck<double>(njoints);
    
    refSpeed = allocAndCheck<double>(njoints);
    refAccel = allocAndCheck<double>(njoints);
    controlP = allocAndCheck<double>(njoints);    

    axisMap = allocAndCheck<int>(njoints);
    jointNames = new ConstString[njoints];

    current_pos = allocAndCheck<double>(njoints);
    next_pos = allocAndCheck<double>(njoints);

    next_sp = allocAndCheck<double>(njoints);
    for(int i = 0;i<njoints;i++)
        next_sp[i] = maxVelocity;

    error_tol = allocAndCheck<double>(njoints);
    
    joint_dev = new DeviceTag[njoints];

    motor_on = allocAndCheck<bool>(njoints);
    for(int i = 0;i<njoints;i++)
        motor_on[i] = false;

    ///////////////////
    /*GENERAL READING*/
    ///////////////////

    Bottle& xtmp = p.findGroup("GENERAL").findGroup("AxisMap","a list of reordered indices for the axes");
    
	if (xtmp.size() != nj+1) {
        ACE_OS::printf("AxisMap does not have the right number of entries\n");
        return false;
    }
    for (i = 1; i < xtmp.size(); i++) axisMap[i-1] = xtmp.get(i).asInt();
    
    xtmp = p.findGroup("GENERAL").findGroup("Encoder","a list of scales for the encoders");
	if (xtmp.size() != nj+1) {
        ACE_OS::printf("Encoder does not have the right number of entries\n");
        return false;
    }
    for (i = 1; i < xtmp.size(); i++) angleToEncoder[i-1] = xtmp.get(i).asDouble();
    xtmp = p.findGroup("GENERAL").findGroup("Zeros","a list of offsets for the zero point");
	if (xtmp.size() != nj+1) {
        ACE_OS::printf("Zeros does not have the right number of entries\n");
        return false;
    }
    for (i = 1; i < xtmp.size(); i++) zeros[i-1] = xtmp.get(i).asDouble();

    //////////////////////
    /*     WEBOTS       */
    //////////////////////

    xtmp = p.findGroup("WEBOTS").findGroup("JointNames","name of the joints to acces webots");
    if(xtmp.size() != nj+1)
        {
            ACE_OS::printf("Not enough joint names to access the webots devices\n");
            return false;
        }
   
    for(i=1;i<xtmp.size();i++) jointNames[i-1] = xtmp.get(i).asString();



    xtmp = p.findGroup("WEBOTS").findGroup("PositionUpdateRate","rate of position update");
    if(xtmp.size() != 2)
        {
            ACE_OS::printf("Position update rate is not set properly\n");
            return false;
        }
    positionUpdateRate = xtmp.get(1).asInt();

    
    xtmp = p.findGroup("WEBOTS").findGroup("SemaphoreNum","access number to the semaphore for webots run function synchro");
    if(xtmp.size() != 2)
        {
            ACE_OS::printf("SemaphoreNum is not set correctly\n");
            return false;
        }
    semaphoreNum = xtmp.get(1).asInt();
    
    ////////////////////////
    /*   LIMITS           */
    ////////////////////////
    xtmp = p.findGroup("LIMITS").findGroup("Max","access the joint limits max");
    if(xtmp.size() != nj+1)
        {
            ACE_OS::printf("Not enough max joint limits\n");
            return false;
        }
    for(int i =1;i<xtmp.size();i++) 
        limitsMax[i-1] = xtmp.get(i).asDouble()*angleToEncoder[i-1];
        
    xtmp = p.findGroup("LIMITS").findGroup("Min","access the joint limits min");
    if(xtmp.size() != nj+1)
        {
            ACE_OS::printf("Not enough min joint limits\n");
            return false;
        }
    for(int i =1;i<xtmp.size();i++)
        limitsMin[i-1] = xtmp.get(i).asDouble()*angleToEncoder[i-1];
        

    xtmp = p.findGroup("LIMITS").findGroup("MaxForce","maximum torque available");
    if(xtmp.size() != nj+1)
        {
            ACE_OS::printf("Not enough torque limit\n");
            return false;
        }
    for(int i=1;i<xtmp.size();i++)
        torqueLimits[i-1] = xtmp.get(i).asDouble();


    xtmp = p.findGroup("LIMITS").findGroup("refAccel","maximum acceleration");
    if(xtmp.size() != nj+1)
        {
            ACE_OS::printf("Not enough ref accel\n");
            return false;
        }
    for(int i=1;i<xtmp.size();i++)
        refAccel[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("LIMITS").findGroup("controlP","control P value");
    if(xtmp.size() != nj+1)
        {
            ACE_OS::printf("Not enough controlP\n");
            return false;
        }
    for(int i=1;i<xtmp.size();i++)
        controlP[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("LIMITS").findGroup("error_tol","error tolerance during tracking");
    if(xtmp.size() != nj+1)
        {
            ACE_OS::printf("Not enough error_tol\n");
            return false;
        }
    for(int i=1;i<xtmp.size();i++)
        error_tol[i-1] = xtmp.get(i).asDouble()*angleToEncoder[i-1];


    ImplementPositionControl<iCubWebotsMotionControl, IPositionControl>::
        initialize(njoints, axisMap, angleToEncoder, zeros);
    
    ImplementVelocityControl<iCubWebotsMotionControl, IVelocityControl>::
        initialize(njoints, axisMap, angleToEncoder, zeros);
  
    ImplementPidControl<iCubWebotsMotionControl, IPidControl>::
        initialize(njoints, axisMap, angleToEncoder, zeros);

    ImplementEncoders<iCubWebotsMotionControl, IEncoders>::
        initialize(njoints, axisMap, angleToEncoder, zeros);

    ImplementControlCalibration<iCubWebotsMotionControl, IControlCalibration>::
        initialize(njoints, axisMap, angleToEncoder, zeros);

    ImplementAmplifierControl<iCubWebotsMotionControl, IAmplifierControl>::
        initialize(njoints, axisMap, angleToEncoder, zeros);

    ImplementControlLimits<iCubWebotsMotionControl, IControlLimits>::
        initialize(njoints, axisMap, angleToEncoder, zeros);
    

    

    for(int i =0;i<njoints;i++)
        {
            current_pos[i] = 0.0;
            next_pos[i] = 0.0;
            
            joint_dev[i] = robot_get_device(jointNames[i].c_str());
            if(joint_dev[i]==0)
                {
                    ACE_OS::printf("could not find device %s\n",jointNames[i].c_str());
                    return false;
                }
            else
                servo_enable_position(joint_dev[i],positionUpdateRate);
        }

    //we point to the webots common instance
    webots_instance = WebotsCommon::getInstance();

    Thread::start();
	//_done.wait ();

	_mutex.post();

	// default initialization for this device driver.
    //setPids(p._pids);


    // set limits, on encoders and max current
    //for(i = 0; i < p._njoints; i++) {
    //    setLimits(i, p._limitsMin[i], p._limitsMax[i]);
    //    setMaxCurrent(i, p._currentLimits[i]);
    //}

	// disable the controller, cards will start with the pid controller & pwm off
	//for (i = 0; i < p._njoints; i++) {
	//	disablePid(i);
	//	disableAmp(i);
	//}

    _opened = true;

    return true;
}


bool iCubWebotsMotionControl::close (void)
{
    if (_opened) {
        // disable the controller, pid controller & pwm off
        
        if (Thread::isRunning())
            {
            }
        
        Thread::stop ();	/// stops the thread first (joins too).
        
        ImplementPositionControl<iCubWebotsMotionControl, IPositionControl>::uninitialize ();
        ImplementVelocityControl<iCubWebotsMotionControl, IVelocityControl>::uninitialize();
        ImplementPidControl<iCubWebotsMotionControl, IPidControl>::uninitialize();
        ImplementEncoders<iCubWebotsMotionControl, IEncoders>::uninitialize();
        ImplementControlCalibration<iCubWebotsMotionControl, IControlCalibration>::uninitialize();
        ImplementAmplifierControl<iCubWebotsMotionControl, IAmplifierControl>::uninitialize();
        ImplementControlLimits<iCubWebotsMotionControl, IControlLimits>::uninitialize();
    }
    
    checkAndDestroy<double>(current_pos);
    checkAndDestroy<double>(next_pos);
    delete[] joint_dev;

    checkAndDestroy<double>(angleToEncoder);
    checkAndDestroy<double>(zeros);
    checkAndDestroy<double>(limitsMin);
    checkAndDestroy<double>(limitsMax);
    checkAndDestroy<int>(axisMap);
    checkAndDestroy<double>(torqueLimits);
    
    checkAndDestroy<double>(refSpeed);
    checkAndDestroy<double>(refAccel);
    checkAndDestroy<double>(controlP);   

    checkAndDestroy<double>(error_tol);
    checkAndDestroy<bool>(motor_on);
    
    delete[] jointNames;



    _opened = false;

	return true;
}


///
///
///
void iCubWebotsMotionControl::run ()
{
    //the run loop
    while(!isStopping())
        {
            //we synch with the webots run function
            webots_instance->sem_control_wait(semaphoreNum);

            //we take control on the variables
            _mutex.wait();

            for(int i = 0;i<njoints;i++)
                {
                    current_pos[i] = (double)servo_get_position(joint_dev[i]);
                    if(motor_on[i]){
                        servo_set_velocity(joint_dev[i],next_sp[i]); 
                        servo_set_position(joint_dev[i],next_pos[i]);
                    }
                }

            //ACE_OS::printf("next_pos of joint 0 = %f\n",next_pos[0]);

            //we don't need the variables anymore
            _mutex.post();

            
            //we finish sync with webots (webots run can continue)
            webots_instance->sem_webots_post(semaphoreNum);
        }

   
}

// return the number of controlled axes.
bool iCubWebotsMotionControl::getAxes(int *ax)
{
    *ax = njoints;
    return true;
}

// LATER: can be optimized.
bool iCubWebotsMotionControl::setPidRaw (int axis, const Pid &pid)
{
    if(axis<njoints)
        {
            _mutex.wait();
            servo_set_control_p(joint_dev[axis],(float)pid.kp);
            controlP[axis] = pid.kp;
            _mutex.post();
            return true;
        }
    ACE_OS::printf("setPidRaw axis num too high %d\n",axis);
    return false;
}

bool iCubWebotsMotionControl::getPidRaw (int axis, Pid *out)
{
    if(axis<njoints)
        {
            _mutex.wait();
            out->kp = controlP[axis];
            out->kd = 0.0;
            out->ki = 0.0;
            out->max_int = 0.0;
            out->scale = 0.0;
            out->max_output = 0.0;
            out->offset = 0.0;
            _mutex.post();
            return true;
        }
    ACE_OS::printf("getPidRaw axis num too high %d\n",axis);
    return false;
}

bool iCubWebotsMotionControl::getPidsRaw (Pid *out)
{
    _mutex.wait();
    for(int i =0;i<njoints;i++)
        {
            out[i].kp = controlP[i];
            out[i].kd = 0.0;
            out[i].ki = 0.0;
            out[i].max_int = 0.0;
            out[i].scale = 0.0;
            out[i].max_output = 0.0;
            out[i].offset = 0.0;
        }
    _mutex.post();
    return true;
}


bool iCubWebotsMotionControl::setPidsRaw(const Pid *pids)
{
    _mutex.wait();
    for(int i=0;i<njoints;i++)
        {
            servo_set_control_p(joint_dev[i],(float)pids[i].kp);
            controlP[i] = pids[i].kp;
        }
    _mutex.post();
    return true;
}

/// cmd is a SingleAxis poitner with 1 double arg
bool iCubWebotsMotionControl::setReferenceRaw (int j, double ref)
{
    if(j<njoints)
        {
            _mutex.wait();
            next_pos[j] = ref;
            _mutex.post();
            return true;
        }
    ACE_OS::printf("trying to set ref of an unknown joint number %d\n",j);
    return false;
}

/// cmd is an array of double (LATER: to be optimized).
bool iCubWebotsMotionControl::setReferencesRaw (const double *refs)
{
    _mutex.wait();
    for(int i = 0;i<njoints;i++)
        next_pos[i] = refs[i];
    _mutex.post();
    return true;
}

bool iCubWebotsMotionControl::setErrorLimitRaw(int j, double limit)
{
    if(j<njoints)
        {
            _mutex.wait();
            error_tol[j] = limit;
            _mutex.post();
            return true;
        }
    ACE_OS::printf("setErrorLimitRaw joint number too high %d\n",j);
    return false;
}

bool iCubWebotsMotionControl::setErrorLimitsRaw(const double *limit)
{
    _mutex.wait();
    for(int i=0;i<njoints;i++)
        error_tol[i] = limit[i];
    _mutex.post();
    return true;
}

bool iCubWebotsMotionControl::getErrorRaw(int axis, double *err)
{
	/*if(axis<njoints)
        {
            _mutex.wait();
            *err = error_tol[axis];
            _mutex.post();
            return true;
        }
    ACE_OS::printf("getErrorRaw axis number too high %d \n",axis);
    return false;*/
    return NOT_YET_IMPLEMENTED("getErrorRaw");
}

bool iCubWebotsMotionControl::getErrorsRaw(double *errs)
{
    /*_mutex.wait();
    for(int i = 0;i<njoints;i++)
        errs[i] = error_tol[i];
    _mutex.post();
    return true;*/
    return NOT_YET_IMPLEMENTED("getErrorsRaw");
}

bool iCubWebotsMotionControl::getOutputRaw(int axis, double *out)
{
   return NOT_YET_IMPLEMENTED("getOutputRaw");
}

bool iCubWebotsMotionControl::getOutputsRaw(double *outs)
{
    return NOT_YET_IMPLEMENTED("getOutputsRaw");
}

bool iCubWebotsMotionControl::getReferenceRaw(int j, double *ref)
{
    if(j<njoints)
        {
            _mutex.wait();
            *ref = next_pos[j];
            _mutex.post();
            return true;
        }
    ACE_OS::printf("getReferenceRaw axis number too high %d\n",j);
    return false;
}

bool iCubWebotsMotionControl::getReferencesRaw(double *ref)
{
    _mutex.wait();
    for(int i = 0;i<njoints;i++)
        ref[i] = next_pos[i];
    _mutex.post();
    return true;
}

bool iCubWebotsMotionControl::getErrorLimitRaw(int j, double *err)
{
    if(j<njoints)
        {
            _mutex.wait();
            *err = error_tol[j];
            _mutex.post();
            return true;
        }
    ACE_OS::printf("getErrorLimiRaw joint number too high %d\n",j);
    return false;
}

bool iCubWebotsMotionControl::getErrorLimitsRaw(double *errs)
{
    for(int i = 0;i<njoints;i++)
        errs[i] = error_tol[i];
    return true;
}

bool iCubWebotsMotionControl::resetPidRaw(int j)
{
    return NOT_YET_IMPLEMENTED("resetPidRaw");
}

bool iCubWebotsMotionControl::enablePidRaw(int axis)
{
    return NOT_YET_IMPLEMENTED("enablePidRaw");
}

bool iCubWebotsMotionControl::setOffsetRaw(int axis, double v)
{
    return NOT_YET_IMPLEMENTED("setOffsetRaw");
}


bool iCubWebotsMotionControl::disablePidRaw(int axis)
{
	return NOT_YET_IMPLEMENTED("disablePidRaw");
}

bool iCubWebotsMotionControl::setPositionMode()
{
    _mutex.wait();
        ifmode = POSITION_MODE;
        for(int i=0;i<njoints;i++)
            next_sp[i]=maxVelocity;
    _mutex.post();
    return true;
}

bool iCubWebotsMotionControl::setVelocityMode()
{
    _mutex.wait();
        ifmode = VELOCITY_MODE;
    _mutex.post();
    return true;
}

bool iCubWebotsMotionControl::positionMoveRaw(int axis, double ref)
{
    if(axis<njoints)
        {
            setPositionMode();
            _mutex.wait();
            if(ref< limitsMin[axis])
                next_pos[axis] = limitsMin[axis];
            else if(ref > limitsMax[axis])
                next_pos[axis] = limitsMax[axis];
            else
                next_pos[axis] = ref;
            ACE_OS::printf("changed position %d to %f",axis,next_pos[axis]);
            _mutex.post();
            return true;
        }
    ACE_OS::printf("positionMoveRaw joint access too high %d \n",axis);
    return false;    
}

bool iCubWebotsMotionControl::positionMoveRaw(const double *refs)
{
    _mutex.wait();
 	for(int i = 0; i<njoints; i++)
        {
            double ref = refs[i];
            if(ref< limitsMin[i])
                next_pos[i] = limitsMin[i];
            else if(ref > limitsMax[i])
                next_pos[i] = limitsMax[i];
            else
                next_pos[i] = ref;
            ACE_OS::printf("moving joint %d to pos %f\n",i,next_pos[i]);
        }
    _mutex.post();
    return true;
}

bool iCubWebotsMotionControl::relativeMoveRaw(int j, double delta)
{
    if(j<njoints)
        {
            _mutex.wait();
            double ref = current_pos[j] + delta;
            if(ref< limitsMin[j])
                next_pos[j] = limitsMin[j];
            else if(ref > limitsMax[j])
                next_pos[j] = limitsMax[j];
            else
                next_pos[j] = ref;
            _mutex.post();
            return true;
        }
    ACE_OS::printf("relativeMoveRaw axis too high %d\n",j);
    return false;
}

bool iCubWebotsMotionControl::relativeMoveRaw(const double *deltas)
{
    _mutex.wait();
 	for(int i = 0; i<njoints; i++)
        {
            double ref = current_pos[i] + deltas[i];
            if(ref< limitsMin[i])
                next_pos[i] = limitsMin[i];
            else if(ref > limitsMax[i])
                next_pos[i] = limitsMax[i];
            else
                next_pos[i] = ref;
        }
    _mutex.post();
    return true;
}

/// check motion done, single axis.
bool iCubWebotsMotionControl::checkMotionDoneRaw(int axis, bool *ret)
{
    if(axis<njoints)
        {
            _mutex.wait();
            if(fabs(current_pos[axis]-next_pos[axis])<error_tol[axis])
                *ret = true;
            else
                *ret = false;
            _mutex.post();
            return true;
        }
    ACE_OS::printf("checkMotionDoneRaw axis too high %d\n",axis);
    return false;
}

/// cmd is a pointer to a bool
bool iCubWebotsMotionControl::checkMotionDoneRaw (bool *ret)
{
    _mutex.wait();
    bool fin = true;
    for(int i = 0;i<njoints;i++)
        {
            if(!(fabs(current_pos[i]-next_pos[i])<error_tol[i]))
                {
                    fin = false;
                    // ACE_OS::printf("axes %d unfinished\n");
                }
        }
    //if(fin)
    ACE_OS::printf("motion finished error tol %f %f %f\n",error_tol[0],current_pos[0],next_pos[0]);
    *ret = fin;
    _mutex.post();
    return true;
}

bool iCubWebotsMotionControl::setRefSpeedRaw(int axis, double sp)
{
    if(axis<njoints)
        {
            _mutex.wait();
            servo_set_velocity(joint_dev[axis],(float)sp);
            refSpeed[axis] = sp;
            _mutex.post();
            return true;
        }
    ACE_OS::printf("setRefSpeedRaw axis number is too high %d \n",axis);
    return false;
}

bool iCubWebotsMotionControl::setRefSpeedsRaw(const double *spds)
{
    _mutex.wait();
    for(int i =0;i<njoints;i++)
        {
            servo_set_velocity(joint_dev[i],(float)spds[i]);
            refSpeed[i] = spds[i];
        }
    _mutex.post();
    return true;
}

bool iCubWebotsMotionControl::setRefAccelerationRaw(int axis, double acc)
{
    if(axis<njoints)
        {
            _mutex.wait();
            servo_set_acceleration(joint_dev[axis],(float)acc);
            refAccel[axis] = acc;
            _mutex.post();
            return true;
        }
    ACE_OS::printf("setRefAccelerationRaw axis number is too high %d\n",axis);
    return false;
}

bool iCubWebotsMotionControl::setRefAccelerationsRaw(const double *accs)
{
    _mutex.wait();
    for(int i = 0;i<njoints;i++)
        {
            servo_set_acceleration(joint_dev[i],(float)accs[i]);
            refAccel[i] = accs[i];
        }
    _mutex.post();
    return true;
}

/// cmd is an array of double (LATER: to be optimized).
bool iCubWebotsMotionControl::getRefSpeedsRaw (double *spds)
{
    _mutex.wait();
    for(int i=0;i<njoints;i++)
        spds[i] = refSpeed[i];
    _mutex.post();
    return true;
}

bool iCubWebotsMotionControl::getRefSpeedRaw (int axis, double *spd)
{
    if(axis<njoints)
        {
            _mutex.wait();
            *spd = refSpeed[axis];
            _mutex.post();
            return true;
        }
    ACE_OS::printf("getRefSpeedRaw axis number is too high %d\n",axis);
    return false;
}

/// cmd is an array of double (LATER: to be optimized).
bool iCubWebotsMotionControl::getRefAccelerationsRaw (double *accs)
{
    _mutex.wait();
    for(int i=0;i<njoints;i++)
        accs[i] = refAccel[i];
    _mutex.post();
    return true;
}

/// cmd is an array of double (LATER: to be optimized).
bool iCubWebotsMotionControl::getRefAccelerationRaw (int axis, double *accs)
{
    if(axis<njoints)
        {
            _mutex.wait();
            *accs = refAccel[axis];
            _mutex.post();
            return true;
        }
    ACE_OS::printf("getRefAccelerationRaw axis number is too high %d\n",axis);
    return false;
}

bool iCubWebotsMotionControl::stopRaw(int j)
{
    if(j<njoints)
        {
            _mutex.wait();
            next_sp[j] = 0.0;
            next_pos[j] = current_pos[j];
            _mutex.post();
            return true;
        }
    ACE_OS::printf("stopRaw joint num too high %d \n",j);
    return false;
}

bool iCubWebotsMotionControl::stopRaw()
{
    _mutex.wait();
    for(int i=0;i<njoints;i++){
        if(ifmode == POSITION_MODE)
            next_pos[i] = current_pos[i];
        else
            next_sp[i] = 0.0;
    }
    _mutex.post();
    return true;
}

/// cmd is an array of double of length njoints specifying speed 
/// for each axis
bool iCubWebotsMotionControl::velocityMoveRaw (int axis, double sp)
{
    if(axis<njoints){
        _mutex.wait();
        if(sp>=0)
            next_pos[axis] = maxPosition;
        else{
            next_pos[axis] = minPosition;
            sp *= -1.0;
        }
        if(sp > maxVelocity)
            next_sp[axis] = maxVelocity;
        else
            next_sp[axis] = sp;
        _mutex.post();
    }
    return true;
}

/// cmd is an array of double of length njoints specifying speed 
/// for each axis
bool iCubWebotsMotionControl::velocityMoveRaw (const double *sp)
{
    _mutex.wait();
    if(ifmode == VELOCITY_MODE){
 	    for(int i = 0; i<njoints; i++){
            double ref = sp[i];
            if(ref >= 0.0)
                next_pos[i] = maxPosition;
            else{
                next_pos[i] = minPosition;
                ref *= -1.0;
            }
            if(ref > maxVelocity)
                next_sp[i] = maxVelocity;
            else
                next_sp[i] = ref;
            ACE_OS::printf("setting servo %d speed to %f\n",i,next_sp[i]);
        }
    }
    _mutex.post();
    return true;
}

bool iCubWebotsMotionControl::setEncoderRaw(int j, double val)
{
    return NOT_YET_IMPLEMENTED("setEncoderRaw");
}

bool iCubWebotsMotionControl::setEncodersRaw(const double *vals)
{
    return NOT_YET_IMPLEMENTED("setEncodersRaw");
}

bool iCubWebotsMotionControl::resetEncoderRaw(int j)
{
    return NOT_YET_IMPLEMENTED("resetEncoderRaw");
}

bool iCubWebotsMotionControl::resetEncodersRaw()
{
   return NOT_YET_IMPLEMENTED("resetEncodersRaw");
}

bool iCubWebotsMotionControl::getEncodersRaw(double *v)
{
    _mutex.wait();
    for(int i = 0;i<njoints;i++)
        v[i] = current_pos[i];
    _mutex.post();
    return true;
}

bool iCubWebotsMotionControl::getEncoderRaw(int axis, double *v)
{
    _mutex.wait();
    *v = current_pos[axis];
    _mutex.post();
    return true;
}

bool iCubWebotsMotionControl::getEncoderSpeedsRaw(double *v)
{
  	return NOT_YET_IMPLEMENTED("getEncoderSpeeds");
}

bool iCubWebotsMotionControl::getEncoderSpeedRaw(int j, double *v)
{
    return NOT_YET_IMPLEMENTED("getEncoderSpeed");
}

bool iCubWebotsMotionControl::getEncoderAccelerationsRaw(double *v)
{
  	return NOT_YET_IMPLEMENTED("getEncoderAccs");
}

bool iCubWebotsMotionControl::getEncoderAccelerationRaw(int j, double *v)
{
    return NOT_YET_IMPLEMENTED("getEncoderAcc");
}

bool iCubWebotsMotionControl::disableAmpRaw(int axis)
{
     if(axis<njoints)
        {
            _mutex.wait();
            motor_on[axis] = false;
            servo_motor_off(joint_dev[axis]);
            _mutex.post();
            return true;            
        }
    ACE_OS::printf("disableAmpRaw axis num too high %d\n",axis);
    return false;
}

bool iCubWebotsMotionControl::enableAmpRaw(int axis)
{
    if(axis<njoints)
        {
            _mutex.wait();
            motor_on[axis] = true;
            _mutex.post();
            return true;            
        }
    ACE_OS::printf("enableAmpRaw axis num too high %d\n",axis);
    return false;
}

// bcast
bool iCubWebotsMotionControl::getCurrentsRaw(double *cs)
{
    return NOT_YET_IMPLEMENTED("getCurrentsRaw");
}

// bcast currents
bool iCubWebotsMotionControl::getCurrentRaw(int axis, double *c)
{
    return NOT_YET_IMPLEMENTED("getCurrentRaw");
}

bool iCubWebotsMotionControl::setMaxCurrentRaw(int axis, double v)
{
    return NOT_YET_IMPLEMENTED("setMaxCurrentRaw");
}

bool iCubWebotsMotionControl::calibrateRaw(int axis, double p)
{
    return NOT_YET_IMPLEMENTED("calibrateRaw");
}

bool iCubWebotsMotionControl::doneRaw(int axis)
{
    return NOT_YET_IMPLEMENTED("doneRaw");
}


bool iCubWebotsMotionControl::getAmpStatusRaw(int *st)
{
	_mutex.wait();
    for(int i =0;i<njoints;i++)
        st[i] = (int)motor_on[i];
    _mutex.post();
    return true;
}

bool iCubWebotsMotionControl::setLimitsRaw(int axis, double min, double max)
{
	return NOT_YET_IMPLEMENTED("setLimitsRaw");
}

bool iCubWebotsMotionControl::getLimitsRaw(int axis, double *min, double *max)
{
	return NOT_YET_IMPLEMENTED("getLimitsRaw");
}
