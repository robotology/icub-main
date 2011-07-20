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
#define _USE_MATH_DEFINES      // for M_PI
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

using namespace yarp::os;
using namespace yarp::dev;

static bool NOT_YET_IMPLEMENTED(const char *txt)
{
    fprintf(stderr, "%s not yet implemented for iCubSimulationControl\n", txt);

    return false;
}

//////////////////////////////////

iCubSimulationControl::iCubSimulationControl() : 
    //RateThread(10),
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
    OdeInit& odeinit = OdeInit::get();
    odeinit.mutex.wait();
    odeinit.removeSimulationControl(partSelec);
    odeinit.mutex.post();
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
    current_torques = allocAndCheck<double>(njoints);
    next_pos = allocAndCheck<double>(njoints);
    next_vel = allocAndCheck<double>(njoints);
    inputs = allocAndCheck<int>(njoints);
    vels = allocAndCheck<double>(njoints);

    error_tol = allocAndCheck<double>(njoints);
    
//  joint_dev = new DeviceTag[njoints];

    motor_on = allocAndCheck<bool>(njoints);
    for(int axis = 0;axis<njoints;axis++)
        motor_on[axis] = false;

    /////////////////////////
    /*   GENERAL           */
    /////////////////////////

    Bottle& xtmp = p.findGroup("GENERAL").findGroup("AxisMap","a list of reordered indices for the axes");
    
    if (xtmp.size() != njoints+1) {
        printf("AxisMap does not have the right number of entries\n");
        return false;
    }
    for (int i = 1; i < xtmp.size(); i++) axisMap[i-1] = xtmp.get(i).asInt();

    xtmp = p.findGroup("GENERAL").findGroup("Encoder","a list of scales for the encoders");
    if (xtmp.size() != njoints+1) {
        printf("Encoder does not have the right number of entries\n");
        return false;
    }
    for (int i = 1; i < xtmp.size(); i++) angleToEncoder[i-1] = xtmp.get(i).asDouble();
    xtmp = p.findGroup("GENERAL").findGroup("Zeros","a list of offsets for the zero point");
    if (xtmp.size() != njoints+1) {
        printf("Zeros does not have the right number of entries\n");
        return false;
    }
    for (int i = 1; i < xtmp.size(); i++) zeros[i-1] = xtmp.get(i).asDouble();
    ////////////////////////
    /*   LIMITS           */
    ////////////////////////
    xtmp = p.findGroup("LIMITS").findGroup("Max","access the joint limits max");
    if(xtmp.size() != njoints+1)
        {
            printf("Not enough max joint limits\n");
            return false;
        }
    for( int i =1;i<xtmp.size();i++ ) 
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


    for(int axis =0;axis<njoints;axis++)
    {
        current_pos[axis] = 0.0;
        ErrorPos[axis] = 0.0;
        double v = 0.0;     
        if (v<limitsMin[axis]) v = limitsMin[axis];
        if (v>limitsMax[axis]) v = limitsMax[axis];
        //else v = next_pos[i];
        next_pos[axis] = M_PI*(v/angleToEncoder[axis])/180;//removed (v/angleToEncoder[i]-1)/180
        next_vel[axis] = 0.0;      
        input = 0;
        inputs[axis] = 0;
        vels[axis] = 1;
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
    
    OdeInit& odeinit = OdeInit::get();
    odeinit.mutex.wait();
    odeinit.setSimulationControl(this, partSelec);
    odeinit.mutex.post();
    //RateThread::start();
    //_done.wait ();
    _mutex.post();
    _opened = true;

    verbosity = odeinit.verbose;
    return true;
}

bool iCubSimulationControl::close (void)
{
    if (_opened) {

        //if (RateThread::isRunning())
        //    {
        //    }
        
        //RateThread::stop();/// stops the thread first (joins too).
        
        ImplementPositionControl<iCubSimulationControl, IPositionControl>::uninitialize ();
        ImplementVelocityControl<iCubSimulationControl, IVelocityControl>::uninitialize();
        ImplementPidControl<iCubSimulationControl, IPidControl>::uninitialize();
        ImplementEncoders<iCubSimulationControl, IEncoders>::uninitialize();
        ImplementControlCalibration<iCubSimulationControl, IControlCalibration>::uninitialize();
        ImplementAmplifierControl<iCubSimulationControl, IAmplifierControl>::uninitialize();
        ImplementControlLimits<iCubSimulationControl, IControlLimits>::uninitialize(); /**/
    }

    checkAndDestroy<double>(current_pos);
    checkAndDestroy<double>(current_torques);
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


void iCubSimulationControl::jointStep() {
    _mutex.wait();
    if (manager==NULL) {
        _mutex.post();
        return;
    }
    if (partSelec<=6) {   
        for (int axis=0; axis<njoints; axis++) {
            LogicalJoint& ctrl = manager->control(partSelec,axis); 
            if (!ctrl.isValid()) continue;
            current_pos[axis] = ctrl.getAngle();
            current_vel[axis] = ctrl.getVelocity();

            current_torques[axis] = ctrl.getTorque();

            //fprintf(stdout,"torques %lf \n",current_torques[axis]);
        
            motor_on[axis] = true; // no reason to turn motors off, for now

            if (velocityMode) {
                if(((current_pos[axis]<limitsMin[axis])&&(next_vel[axis]<0)) || ((current_pos[axis]>limitsMax[axis])&&(next_vel[axis]>0)))
                    ctrl.setVelocity(0.0);
                else{
                    ctrl.setVelocity(next_vel[axis]);
                }
            } else {
                // no acceleration control right now, just velocity
                ctrl.setControlParameters(vels[axis],1);
                ctrl.setPosition(next_pos[axis]);
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
bool iCubSimulationControl::setReferenceRaw (int axis, double ref)
{
    if( (axis>=0) && (axis<njoints) )
        {
        //printf(" GETTING THE REFERENCE??? 1\n");
            _mutex.wait();
            next_pos[axis] = ref;
            _mutex.post();
            return true;
        }
    if (verbosity)
        printf("setReferenceRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

/// cmd is an array of double (LATER: to be optimized).
bool iCubSimulationControl::setReferencesRaw (const double *refs)
{
   _mutex.wait();
   for(int axis = 0; axis<njoints; axis++){
        next_pos[axis] = refs[axis];
   }
    _mutex.post();
    return true;
}

bool iCubSimulationControl::setErrorLimitRaw(int axis, double limit)
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

bool iCubSimulationControl::getReferenceRaw(int axis, double *ref)
{
    if( (axis>=0) && (axis<njoints) )
        {
            _mutex.wait();
            *ref = next_pos[axis];
            _mutex.post();
            return true;
        }
    if (verbosity)
        printf("getReferenceRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::getReferencesRaw(double *ref)
{
    _mutex.wait();
    for(int axis = 0; axis<njoints; axis++)
        ref[axis] = next_pos[axis];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getErrorLimitRaw(int axis, double *err)
{
     return NOT_YET_IMPLEMENTED("getErrorLimitRaw");
}

bool iCubSimulationControl::getErrorLimitsRaw(double *errs)
{
   return NOT_YET_IMPLEMENTED("getErrorLimitsRaw");
}

bool iCubSimulationControl::resetPidRaw(int axis)
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
    if( (axis >=0) && (axis<njoints) )
    {
        _mutex.wait();
        if(ref< limitsMin[axis])
            next_pos[axis] = limitsMin[axis];
        else if(ref > limitsMax[axis])
            next_pos[axis] = limitsMax[axis];
        else
            next_pos[axis] = ref;
        motor_on[axis]=true;

        if (verbosity)
            printf("moving joint %d of part %d to pos %f\n",axis, partSelec, next_pos[axis]);
        _mutex.post();
        return true;
    }
    if (verbosity)
        printf("positionMoveRaw joint access too high %d \n",axis);
    return false;    
}

bool iCubSimulationControl::positionMoveRaw(const double *refs)
{
    velocityMode = false;
    _mutex.wait();
    for(int axis = 0; axis<njoints; axis++)
        {
            double ref = refs[axis];
            if(ref< limitsMin[axis]){
                next_pos[axis] = limitsMin[axis];
            }
            else if(ref > limitsMax[axis]){
                next_pos[axis] = limitsMax[axis];
            }
            else
                next_pos[axis] = ref;
            motor_on[axis]=true;
            if (verbosity)
                printf("moving joint %d of part %d to pos %f\n",axis,partSelec,next_pos[axis]);
        }
    _mutex.post();
    return true;
}

bool iCubSimulationControl::relativeMoveRaw(int axis, double delta)
{
    velocityMode = false;
    return positionMoveRaw(axis,next_pos[axis]+delta);
}

bool iCubSimulationControl::relativeMoveRaw(const double *deltas)
{
    velocityMode = false;
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
            if(! (fabs( current_pos[axis]-next_pos[axis])<error_tol[axis]))
                {
                    fin = false;
                    // printf("axes %d unfinished\n");
                }
        }
    //if(fin)
    if (verbosity)
        printf("motion finished error tol %f %f %f\n",error_tol[0],current_pos[0],next_pos[0]);
    *ret = fin;
    _mutex.post();
    return true;
}

bool iCubSimulationControl::checkMotionDoneRaw(int axis, bool *ret)
{
    if( (axis >=0) && (axis<njoints) )
        {
            _mutex.wait();
            if(fabs(current_pos[axis]-next_pos[axis])<error_tol[axis])
                *ret = true;
            else
                *ret = false;
            _mutex.post();
            return true;
        }
    if (verbosity)
        printf("checkMotionDoneRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}
bool iCubSimulationControl::setRefSpeedRaw(int axis, double sp)
{
    if( (axis >=0) && (axis<njoints) )
        {
            _mutex.wait();
            //vel = sp;// *180/M_PI ;
            vels[axis] = sp;//vel/20;
            if (verbosity)
                printf("setting joint %d of part %d to reference velocity %f\n",axis,partSelec,vels[axis]);
            _mutex.post();
            return true;
        }
    return false;
}

bool iCubSimulationControl::setRefSpeedsRaw(const double *spds)
{
    _mutex.wait();
    for(int axis = 0; axis<njoints; axis++)
    {
        vels[axis] = spds[axis];//(spds[i]*180/M_PI)/20;
        //vels[i] = spds[i]/20;
        if (verbosity)
            printf("setting joint %d of part %d to reference velocity %f\n",axis,partSelec,vels[axis]);
    }
    _mutex.post();
    return true;
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
        *ref = vels[axis];
        _mutex.post();
        return true;
    }
    if (verbosity)
        printf("getRefSpeedRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}
bool iCubSimulationControl::getRefSpeedsRaw(double *spds)
{
    _mutex.wait();
    for(int axis = 0;axis<njoints;axis++)
        spds[axis] = vels[axis];
    _mutex.post();
    return true;
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
            next_pos[axis] = current_pos[axis];
            next_vel[axis] = 0;
            _mutex.post();
            return true;
        }
    if (verbosity)
        printf("stopRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}
bool iCubSimulationControl::stopRaw()
{
    _mutex.wait();
    for(int axis=0;axis<njoints;axis++){
        next_pos[axis] = current_pos[axis];
        next_vel[axis] = 0.0;
    }
    _mutex.post();
    return true;
}
// cmd is an array of double of length njoints specifying speed 
/// for each axis
bool iCubSimulationControl::velocityMoveRaw (int axis, double sp)
{
    velocityMode = true;
    if( (axis >=0) && (axis<njoints) ) {
        _mutex.wait();
        next_vel[axis] = sp;
        motor_on[axis] = true;
        _mutex.post();
        return true;
    }
    if (verbosity)
        printf("velocityMoveRaw: joint with index %d does not exist, valis joints indices are between 0 and %d \n",axis,njoints);
    return false;    
}

/// cmd is an array of double of length njoints specifying speed 
/// for each axis
bool iCubSimulationControl::velocityMoveRaw (const double *sp)
{
    velocityMode = true;
    _mutex.wait();
    for (int axis=0; axis<njoints; axis++) {
        //velocityMoveRaw(i,sp[i]);
        next_vel[axis] = sp[axis];
        motor_on[axis] = true;
        if (verbosity)
            printf("setting joint %d of part %d to velocity %f\n",axis,partSelec,next_vel[axis]);
    }
    _mutex.post();
    return true;
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
        v[axis] = current_pos[axis];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getEncoderRaw(int axis, double *v)
{
    if((axis>=0) && (axis<njoints)) {
        _mutex.wait();
        *v = current_pos[axis];
        _mutex.post();
        return true;
    }
    if (verbosity)
        printf("getEncoderRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::getEncoderSpeedsRaw(double *v)
{
   _mutex.wait();
    for(int axis = 0; axis<njoints; axis++)
        v[axis] = current_vel[axis];//* 10;
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getEncoderSpeedRaw(int axis, double *v)
{
    if( (axis>=0) && (axis<njoints) ) {
        _mutex.wait();
        *v = current_vel[axis];// * 10;
        _mutex.post();
        return true;
    }
    if (verbosity)
        printf("getEncoderSpeedRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;
}

bool iCubSimulationControl::getEncoderAccelerationsRaw(double *v)
{
  	return NOT_YET_IMPLEMENTED("getEncoderAccelerationsRaw");
}

bool iCubSimulationControl::getEncoderAccelerationRaw(int axis, double *v)
{
    return NOT_YET_IMPLEMENTED("getEncoderAcc");
}

bool iCubSimulationControl::disableAmpRaw(int axis)
{
    if( (axis >=0) && (axis<njoints) )
        {
            _mutex.wait();
            motor_on[axis] = false;
            _mutex.post();
            return true;            
        }
    if (verbosity)
        printf("disableAmpRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
    return false;  
}

bool iCubSimulationControl::enableAmpRaw(int axis)
{
   
    if( (axis>=0) && (axis<njoints) )
        {
            _mutex.wait();
            motor_on[axis] = true;
            _mutex.post();
            return true;            
        }
    if (verbosity)
        printf("enableAmpRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
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
        printf("getAmpStatusRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
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
        printf("setLimitsRaw: joint with index %d does not exist; valid joint indices are between 0 and %d\n", axis, njoints);
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

bool iCubSimulationControl::setTorqueModeRaw( )
{
    return NOT_YET_IMPLEMENTED("setTorqueModeRaw");
}
bool iCubSimulationControl::getTorqueRaw(int axis, double *sp)
{
    return NOT_YET_IMPLEMENTED("getTorqueRaw");
}
bool iCubSimulationControl::getTorquesRaw(double *sp)
{
    return NOT_YET_IMPLEMENTED("getTorquesRaw");
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
    return NOT_YET_IMPLEMENTED("setRefTorquesRaw");
}
bool iCubSimulationControl::setRefTorqueRaw(int axis,double ref)
{
    return NOT_YET_IMPLEMENTED("setRefTorqueRaw");
}
bool iCubSimulationControl::getRefTorquesRaw(double *ref)
{
    return NOT_YET_IMPLEMENTED("getRefTorquesRaw");
}
bool iCubSimulationControl::getRefTorqueRaw(int axis,double *ref)
{
    return NOT_YET_IMPLEMENTED("getRefTorqueRaw");
}
bool iCubSimulationControl::setTorquePidRaw(int axis, const yarp::dev::Pid &pid)
{
    return NOT_YET_IMPLEMENTED("setTorquePidRaw");
}
bool iCubSimulationControl::setTorquePidsRaw(const yarp::dev::Pid *pid)
{
    return NOT_YET_IMPLEMENTED("setTorquePidsRaw");
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
    return NOT_YET_IMPLEMENTED("getTorquePidOutputRaw");
}
bool iCubSimulationControl::getTorquePidOutputsRaw(double *out)
{
    return NOT_YET_IMPLEMENTED("getTorquePidOutputsRaw");
}
bool iCubSimulationControl::getTorquePidRaw(int axis, yarp::dev::Pid *pid)
{
    return NOT_YET_IMPLEMENTED("getTorquePidRaw");
}
bool iCubSimulationControl::getTorquePidsRaw(yarp::dev::Pid *pid)
{
    return NOT_YET_IMPLEMENTED("getTorquePidsRaw");
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
    return NOT_YET_IMPLEMENTED("disableTorquePidRaw");
}
bool iCubSimulationControl::enableTorquePidRaw(int axis)
{
    return NOT_YET_IMPLEMENTED("enableTorquePidRaw");
}
bool iCubSimulationControl::setTorqueOffsetRaw(int axis,double offset)
{
    return NOT_YET_IMPLEMENTED("setTorqueOffsetRaw");
}

