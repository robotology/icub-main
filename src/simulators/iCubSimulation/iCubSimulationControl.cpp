// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Vadim Tikhanoff, Paul Fitzpatrick, Giorgio Metta
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * Thanks to Ruben Smits for the Velocity Control limits patch
 *
 */

/**
 * \file iCubSimulationControl.cpp
 * \brief This file is the yarp interface of the iCubSimulation. It initializes the joints number and names, and deals with the PID motion (position control velocity control)
 * \author Vadim Tikhanoff, Paul Fitzpatrick, Giorgio Metta
 * \date 2007
 * \note Release under GNU GPL v2.0
 **/
#ifdef _MSC_VER
#pragma warning(disable:4355)  //for VC++, no precision loss complaints
#define _USE_MATH_DEFINES	   // for M_PI
#endif
/// general purpose stuff.
#include <yarp/os/Time.h>
#include <yarp/os/ConstString.h>

#include <math.h>
#include <string>

///specific to this device driver.
#include "iCubSimulationControl.h"

#include <yarp/dev/ControlBoardInterfacesImpl.inl>

using namespace yarp::os;
using namespace yarp::dev;
static bool NOT_YET_IMPLEMENTED(const char *txt)
{
    fprintf(stderr, "%s not yet implemented for iCubSimulationControl\n", txt);

    return false;
}

//////////////////////////////////

iCubSimulationControl::iCubSimulationControl() : 
    RateThread(10),
    ImplementPositionControl<iCubSimulationControl, IPositionControl>(this),
   ImplementVelocityControl<iCubSimulationControl, IVelocityControl>(this),
    ImplementPidControl<iCubSimulationControl, IPidControl>(this),
    ImplementEncoders<iCubSimulationControl, IEncoders>(this),
    ImplementControlCalibration<iCubSimulationControl, IControlCalibration>(this),
    ImplementAmplifierControl<iCubSimulationControl, IAmplifierControl>(this),
    ImplementControlLimits<iCubSimulationControl, IControlLimits>(this),/* */
    _done(0),
    _mutex(1)
{
    _opened = false;
    manager = NULL;
}


iCubSimulationControl::~iCubSimulationControl ()
{
}

bool iCubSimulationControl::open(yarp::os::Searchable& config) {

    Searchable& p = config;

	if (!p.check("GENERAL","section for general motor control parameters")) {
        fprintf(stderr, "Cannot understand configuration parameters\n");
        return false;
    }
	
	int TypeArm = p.findGroup("GENERAL").check("Type",Value(1),
                                          "what did the user select?").asInt();

	int numTOTjoints = p.findGroup("GENERAL").check("TotalJoints",Value(1),
                                          "Number of total joints").asInt();

	double velocity = p.findGroup("GENERAL").check("Vel",Value(1),
                                          "Default velocity").asDouble();
	int i;

	_mutex.wait();
	partSelec = TypeArm;
	
	njoints = numTOTjoints;

	vel = velocity;

	angleToEncoder = allocAndCheck<double>(njoints);
    zeros = allocAndCheck<double>(njoints);
    
    limitsMin = allocAndCheck<double>(njoints);
    limitsMax = allocAndCheck<double>(njoints);
    torqueLimits = allocAndCheck<double>(njoints);
    
    refSpeed = allocAndCheck<double>(njoints);
    refAccel = allocAndCheck<double>(njoints);
    controlP = allocAndCheck<double>(njoints);    

    axisMap = allocAndCheck<int>(njoints);
//  jointNames = new ConstString[njoints];

    current_pos = allocAndCheck<double>(njoints);
    current_vel = allocAndCheck<double>(njoints);
    next_pos = allocAndCheck<double>(njoints);
    next_vel = allocAndCheck<double>(njoints);
    inputs = allocAndCheck<int>(njoints);
    vels = allocAndCheck<double>(njoints);

    error_tol = allocAndCheck<double>(njoints);
    
//  joint_dev = new DeviceTag[njoints];

    motor_on = allocAndCheck<bool>(njoints);
    for(int i = 0;i<njoints;i++)
        motor_on[i] = false;

	/////////////////////////
    /*   GENERAL           */
    /////////////////////////

	Bottle& xtmp = p.findGroup("GENERAL").findGroup("AxisMap","a list of reordered indices for the axes");
    
	if (xtmp.size() != njoints+1) {
        printf("AxisMap does not have the right number of entries\n");
        return false;
    }
    for (i = 1; i < xtmp.size(); i++) axisMap[i-1] = xtmp.get(i).asInt();

	 xtmp = p.findGroup("GENERAL").findGroup("Encoder","a list of scales for the encoders");
	if (xtmp.size() != njoints+1) {
        printf("Encoder does not have the right number of entries\n");
        return false;
    }
    for (i = 1; i < xtmp.size(); i++) angleToEncoder[i-1] = xtmp.get(i).asDouble();
    xtmp = p.findGroup("GENERAL").findGroup("Zeros","a list of offsets for the zero point");
	if (xtmp.size() != njoints+1) {
        printf("Zeros does not have the right number of entries\n");
        return false;
    }
    for (i = 1; i < xtmp.size(); i++) zeros[i-1] = xtmp.get(i).asDouble();
	////////////////////////
    /*   LIMITS           */
    ////////////////////////
    xtmp = p.findGroup("LIMITS").findGroup("Max","access the joint limits max");
    if(xtmp.size() != njoints+1)
        {
            printf("Not enough max joint limits\n");
            return false;
        }
    for( i =1;i<xtmp.size();i++) 
        limitsMax[i-1] = xtmp.get(i).asDouble()*angleToEncoder[i-1];
        
    xtmp = p.findGroup("LIMITS").findGroup("Min","access the joint limits min");
    if(xtmp.size() != njoints+1)
        {
            printf("Not enough min joint limits\n");
            return false;
        }
    for(int i =1;i<xtmp.size();i++)
        limitsMin[i-1] = xtmp.get(i).asDouble()*angleToEncoder[i-1];

	xtmp = p.findGroup("LIMITS").findGroup("error_tol","error tolerance during tracking");
    if(xtmp.size() != njoints+1)
        {
            printf("Not enough error_tol\n");
            return false;
        }
    for(int i=1;i<xtmp.size();i++)
        error_tol[i-1] = xtmp.get(i).asDouble()*angleToEncoder[i-1];


	for(int i =0;i<njoints;i++)
    {
        current_pos[i] = 0.0;
     	ErrorPos[i] = 0.0;
		double v = 0.0;
		if (v<limitsMin[i]) v = limitsMin[i];
		if (v>limitsMax[i]) v = limitsMax[i];
		//else v = next_pos[i];
        next_pos[i] = M_PI*(v/angleToEncoder[i])/180;//removed (v/angleToEncoder[i]-1)/180
        next_vel[i] = 0.0;
		input = 0;
        inputs[i] = 0;
        vels[i] = 1;
   }
	

	ImplementPositionControl<iCubSimulationControl, IPositionControl>::
        initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementVelocityControl<iCubSimulationControl, IVelocityControl>::
		initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementPidControl<iCubSimulationControl, IPidControl>::
		initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementEncoders<iCubSimulationControl, IEncoders>::
        initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementControlCalibration<iCubSimulationControl, IControlCalibration>::
        initialize(njoints, axisMap, angleToEncoder, zeros);
	ImplementAmplifierControl<iCubSimulationControl, IAmplifierControl>::
        initialize(njoints, axisMap, angleToEncoder, zeros);
	ImplementControlLimits<iCubSimulationControl, IControlLimits>::
        initialize(njoints, axisMap, angleToEncoder, zeros);
		
    velocityMode = false;

    if (!p.check("joint_device")) {
        printf("Need a device to access the joints\n");
        return false;
    }
    if (!joints.open(p.find("joint_device").asString().c_str())) {
        printf("Failed to create a device to access the joints\n");
        return false;
    }
    manager = NULL;
    joints.view(manager);
    if (manager==NULL) {
        printf("Wrong type for device to access the joints\n");
        return false;
    }

	RateThread::start();
	//_done.wait ();
	_mutex.post();
	_opened = true;
    return true;
}

bool iCubSimulationControl::close (void)
{
    if (_opened) {

        if (RateThread::isRunning())
            {
            }
        
        RateThread::stop();	/// stops the thread first (joins too).
        
        ImplementPositionControl<iCubSimulationControl, IPositionControl>::uninitialize ();
        ImplementVelocityControl<iCubSimulationControl, IVelocityControl>::uninitialize();
        ImplementPidControl<iCubSimulationControl, IPidControl>::uninitialize();
        ImplementEncoders<iCubSimulationControl, IEncoders>::uninitialize();
        ImplementControlCalibration<iCubSimulationControl, IControlCalibration>::uninitialize();
        ImplementAmplifierControl<iCubSimulationControl, IAmplifierControl>::uninitialize();
        ImplementControlLimits<iCubSimulationControl, IControlLimits>::uninitialize(); /**/
    }

	checkAndDestroy<double>(current_pos);
	checkAndDestroy<double>(current_vel);
    checkAndDestroy<double>(next_pos);
    checkAndDestroy<double>(next_vel);
   // delete[] joint_dev;

    checkAndDestroy<double>(angleToEncoder);
    checkAndDestroy<double>(zeros);
    checkAndDestroy<double>(limitsMin);
    checkAndDestroy<double>(limitsMax);
    checkAndDestroy<int>(axisMap);
    checkAndDestroy<int>(inputs);
    checkAndDestroy<double>(vels);
    checkAndDestroy<double>(torqueLimits);
    
    checkAndDestroy<double>(refSpeed);
    checkAndDestroy<double>(refAccel);
    checkAndDestroy<double>(controlP);   

    checkAndDestroy<double>(error_tol);
    checkAndDestroy<bool>(motor_on);
    
  //  delete[] jointNames;

	_opened = false;
	return true;
}

void iCubSimulationControl::run() {
    _mutex.wait();
    if (manager==NULL) {
        _mutex.post();
        return;
    }
    int lengths[] = { 0, 16, 16, 6, 6 , 6, 3};
    if (partSelec<=6) {
        int len = lengths[partSelec];
        int i;
        
        for (i=0; i<len; i++) {
            LogicalJoint& ctrl = manager->control(partSelec,i); 
            if (!ctrl.isValid()) continue;
            current_pos[i] = ctrl.getAngle();
            current_vel[i] = ctrl.getVelocity();
        }
        
        for (i=0; i<len; i++) {
            LogicalJoint& ctrl = manager->control(partSelec,i); 
            if (!ctrl.isValid()) continue;
            motor_on[i] = true; // no reason to turn motors off, for now
            if (velocityMode) {
                //ctrl.setVelocity(next_vel[i]);
                if(((current_pos[i]<limitsMin[i])&&(next_vel[i]<0)) || ((current_pos[i]>limitsMax[i])&&(next_vel[i]>0)))
                    ctrl.setVelocity(0.0);
                else{
                    ctrl.setVelocity(next_vel[i]);
                }
				
            } else {
                // no acceleration control right now, just velocity
                ctrl.setControlParameters(vels[i],1);
                ctrl.setPosition(next_pos[i]);
            }
        }
    }
    _mutex.post();
}


bool iCubSimulationControl::getAxes(int *ax)
{
    *ax = njoints;
    return true;
}

bool iCubSimulationControl::setPidRaw (int axis, const Pid &pid)
{
   return NOT_YET_IMPLEMENTED("setPidRaw");
}
bool iCubSimulationControl::getPidRaw (int axis, Pid *out)
{
   return NOT_YET_IMPLEMENTED("getPidRaw");
}

bool iCubSimulationControl::getPidsRaw (Pid *out)
{
    return NOT_YET_IMPLEMENTED("getPidsRaw");
}

bool iCubSimulationControl::setPidsRaw(const Pid *pids)
{
    return NOT_YET_IMPLEMENTED("setPidsRaw");
}

/// cmd is a SingleAxis pointer with 1 double arg
bool iCubSimulationControl::setReferenceRaw (int j, double ref)
{
    if(j<njoints)
        {
	//		printf(" GETTING THE REFERENCE??? 1\n");
            _mutex.wait();
            next_pos[j] = ref;
            _mutex.post();
            return true;
        }
    printf("trying to set ref of an unknown joint number %d\n",j);
    return false;
}

/// cmd is an array of double (LATER: to be optimized).
bool iCubSimulationControl::setReferencesRaw (const double *refs)
{
   _mutex.wait();
   for(int i = 0;i<njoints;i++){
        next_pos[i] = refs[i];
   }
    _mutex.post();
    return true;
}

bool iCubSimulationControl::setErrorLimitRaw(int j, double limit)
{
    return NOT_YET_IMPLEMENTED("setErrorLimitRaw");
}

bool iCubSimulationControl::setErrorLimitsRaw(const double *limit)
{
    return NOT_YET_IMPLEMENTED("setErrorLimitsRaw");
}

bool iCubSimulationControl::getErrorRaw(int axis, double *err)
{
    return NOT_YET_IMPLEMENTED("getErrorRaw");
}

bool iCubSimulationControl::getErrorsRaw(double *errs)
{
    return NOT_YET_IMPLEMENTED("getErrorsRaw");
}

bool iCubSimulationControl::getOutputRaw(int axis, double *out)
{
   return NOT_YET_IMPLEMENTED("getOutputRaw");
}

bool iCubSimulationControl::getOutputsRaw(double *outs)
{
    return NOT_YET_IMPLEMENTED("getOutputsRaw");
}

bool iCubSimulationControl::getReferenceRaw(int j, double *ref)
{
    if(j<njoints)
        {
            _mutex.wait();
            *ref = next_pos[j];
            _mutex.post();
            return true;
        }
    printf("getReferenceRaw axis number too high %d\n",j);
    return false;
}

bool iCubSimulationControl::getReferencesRaw(double *ref)
{
    _mutex.wait();
    for(int i = 0;i<njoints;i++)
        ref[i] = next_pos[i];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getErrorLimitRaw(int j, double *err)
{
     return NOT_YET_IMPLEMENTED("getErrorLimitRaw");
}

bool iCubSimulationControl::getErrorLimitsRaw(double *errs)
{
   return NOT_YET_IMPLEMENTED("getErrorLimitsRaw");
}

bool iCubSimulationControl::resetPidRaw(int j)
{
    return NOT_YET_IMPLEMENTED("resetPidRaw");
}

bool iCubSimulationControl::enablePidRaw(int axis)
{
    return NOT_YET_IMPLEMENTED("enablePidRaw");
}

bool iCubSimulationControl::setOffsetRaw(int axis, double v)
{
    return NOT_YET_IMPLEMENTED("setOffsetRaw");
}

bool iCubSimulationControl::disablePidRaw(int axis)
{
	return NOT_YET_IMPLEMENTED("disablePidRaw");
}

bool iCubSimulationControl::setPositionModeRaw()
{
    velocityMode = false;
    return true;
}

bool iCubSimulationControl::setVelocityModeRaw()
{
    velocityMode = true;
    return true;
}

bool iCubSimulationControl::positionMoveRaw(int axis, double ref)
{
    velocityMode = false;
	if(axis<njoints)
        {
            _mutex.wait();
			if(ref< limitsMin[axis])
                next_pos[axis] = limitsMin[axis];
			else if(ref > limitsMax[axis])
                next_pos[axis] = limitsMax[axis];
			else
                next_pos[axis] = ref;
            motor_on[axis]=true;
				
            printf("changed position %d to %f\n",axis,next_pos[axis]);
            _mutex.post();
            return true;
        }
    printf("positionMoveRaw joint access too high %d \n",axis);
    return false;    
}

bool iCubSimulationControl::positionMoveRaw(const double *refs)
{
    velocityMode = false;
      _mutex.wait();
 	for(int i = 0; i<njoints; i++)
        {
            double ref = refs[i];
			if(ref< limitsMin[i]){
                next_pos[i] = limitsMin[i];
			}
			else if(ref > limitsMax[i]){
                next_pos[i] = limitsMax[i];
			}
            else
			 next_pos[i] = ref;
            motor_on[i]=true;
            printf("moving joint %d to pos %f\n",i,next_pos[i]);
        }
    _mutex.post();
    return true;
}

bool iCubSimulationControl::relativeMoveRaw(int j, double delta)
{
    velocityMode = false;
	return positionMoveRaw(j,next_pos[j]+delta);
}

bool iCubSimulationControl::relativeMoveRaw(const double *deltas)
{
    velocityMode = false;
 	for(int i = 0; i<njoints; i++) {
        relativeMoveRaw(i,deltas[i]);
    }
    return true;
}

bool iCubSimulationControl::checkMotionDoneRaw (bool *ret)
{
    _mutex.wait();
    bool fin = true;
    for(int i = 0;i<njoints;i++)
        {
            if(!(fabs(current_pos[i]-next_pos[i])<error_tol[i]))
                {
                    fin = false;
                    // printf("axes %d unfinished\n");
                }
        }
    //if(fin)
    printf("motion finished error tol %f %f %f\n",error_tol[0],current_pos[0],next_pos[0]);
    *ret = fin;
    _mutex.post();
    return true;
}

bool iCubSimulationControl::checkMotionDoneRaw(int axis, bool *ret)
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
    printf("checkMotionDoneRaw axis too high %d\n",axis);
    return false;
}
bool iCubSimulationControl::setRefSpeedRaw(int j, double sp)
{
	if(j<njoints)
        {
            _mutex.wait();
			 vel = sp *180/M_PI ;
             vels[j] = vel/20;
			 printf("the velocity: %lf %lf\n",vel, sp );
            _mutex.post();
            return true;
        }
    return false;
}

bool iCubSimulationControl::setRefSpeedsRaw(const double *spds)
{
	_mutex.wait();
	for(int i = 0; i<njoints; i++)
        {
             vel = spds[i]/20;
			 
	}
	_mutex.post();
    return true;
}
bool iCubSimulationControl::setRefAccelerationRaw(int j, double acc)
{
	return NOT_YET_IMPLEMENTED("setRefAccelerationRaw");
}
bool iCubSimulationControl::setRefAccelerationsRaw(const double *accs)
{
	return NOT_YET_IMPLEMENTED("setRefAccelerationsRaw");
}
bool iCubSimulationControl::getRefSpeedRaw(int j, double *ref)
{
	return NOT_YET_IMPLEMENTED("getRefSpeedRaw");
}
bool iCubSimulationControl::getRefSpeedsRaw(double *spds)
{
	return NOT_YET_IMPLEMENTED("getRefSpeedsRaw");
}
bool iCubSimulationControl::getRefAccelerationRaw(int j, double *acc)
{
	return NOT_YET_IMPLEMENTED("getRefAccelerationRaw");
}
bool iCubSimulationControl::getRefAccelerationsRaw(double *accs)
{
	return NOT_YET_IMPLEMENTED("getRefAccelerationsRaw");
}
bool iCubSimulationControl::stopRaw(int j)
{
	if(j<njoints)
        {
            _mutex.wait();
            next_pos[j] = current_pos[j];
            next_vel[j] = 0;
            _mutex.post();
            return true;
        }
    printf("stopRaw joint num too high %d \n",j);
    return false;
}
bool iCubSimulationControl::stopRaw()
{
	_mutex.wait();
	for(int i=0;i<njoints;i++){
        next_pos[i] = current_pos[i];
        next_vel[i] = 0;
	}
    _mutex.post();
    return true;
}
// cmd is an array of double of length njoints specifying speed 
/// for each axis
bool iCubSimulationControl::velocityMoveRaw (int axis, double sp)
{
    velocityMode = true;
	if(axis<njoints) {
        _mutex.wait();
        next_vel[axis] = sp;
        motor_on[axis] = true;
        _mutex.post();
        return true;
    }
    return false;    
}

/// cmd is an array of double of length njoints specifying speed 
/// for each axis
bool iCubSimulationControl::velocityMoveRaw (const double *sp)
{
    velocityMode = true;
    for (int i=0; i<njoints; i++) {
        velocityMoveRaw(i,sp[i]);
    }
    return true;
}

bool iCubSimulationControl::setEncoderRaw(int j, double val)
{
    return NOT_YET_IMPLEMENTED("setEncoderRaw");
}

bool iCubSimulationControl::setEncodersRaw(const double *vals)
{
    return NOT_YET_IMPLEMENTED("setEncodersRaw");
}

bool iCubSimulationControl::resetEncoderRaw(int j)
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
    for(int i = 0;i<njoints;i++)
        v[i] = current_pos[i];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getEncoderRaw(int axis, double *v)
{
	_mutex.wait();
    *v = current_pos[axis];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getEncoderSpeedsRaw(double *v)
{
   _mutex.wait();
    for(int i = 0;i<njoints;i++)
        v[i] = current_vel[i] * 10;
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getEncoderSpeedRaw(int j, double *v)
{
	_mutex.wait();
    *v = current_vel[j] * 10;
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getEncoderAccelerationsRaw(double *v)
{
  	return NOT_YET_IMPLEMENTED("getEncoderAccelerationsRaw");
}

bool iCubSimulationControl::getEncoderAccelerationRaw(int j, double *v)
{
    return NOT_YET_IMPLEMENTED("getEncoderAcc");
}

bool iCubSimulationControl::disableAmpRaw(int axis)
{
    if(axis<njoints)
        {
            _mutex.wait();
            motor_on[axis] = false;
            _mutex.post();
            return true;            
        }
    printf("disableAmpRaw axis num too high %d\n",axis);
    return false;  
}

bool iCubSimulationControl::enableAmpRaw(int axis)
{
   
	if(axis<njoints)
        {
            _mutex.wait();
            motor_on[axis] = true;
            _mutex.post();
            return true;            
        }
    printf("enableAmpRaw axis num too high %d\n",axis);
    return false;
}

// bcast
bool iCubSimulationControl::getCurrentsRaw(double *cs)
{
    return NOT_YET_IMPLEMENTED("getCurrentsRaw");
}

// bcast currents
bool iCubSimulationControl::getCurrentRaw(int axis, double *c)
{
    return NOT_YET_IMPLEMENTED("getCurrentRaw");
}

bool iCubSimulationControl::setMaxCurrentRaw(int axis, double v)
{
    return NOT_YET_IMPLEMENTED("setMaxCurrentRaw");
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
    for(int i =0;i<njoints;i++)
        st[i] = (int)motor_on[i];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getAmpStatusRaw(int i, int *st)
{
    bool ret=false;
	_mutex.wait();
 
    if (i<njoints) 
    {
        st[i] = (int)motor_on[i];
        ret=true;
    }
   
    _mutex.post();
    return ret;
}

bool iCubSimulationControl::setLimitsRaw(int axis, double min, double max)
{
    if(axis > -1 && axis < njoints){
        _mutex.wait();
        limitsMax[axis] = max;
        limitsMin[axis] = min;
        _mutex.post();
       return true;
    }
    return false;
}

bool iCubSimulationControl::getLimitsRaw(int axis, double *min, double *max)
{
	if(axis > -1 && axis < njoints){
         *min = limitsMin[axis];
         *max = limitsMax[axis];
         return true;
     }
     else
         return false;
}

