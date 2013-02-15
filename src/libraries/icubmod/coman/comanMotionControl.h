// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/* Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Alberto Cardellino
 * email: alberto.cardellino@iit.it
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

// update comment hereafter

/**
 * @ingroup icub_hardware_modules
 *
 * Copyright (C) 2012 iCubFacility - Istituto Italiano di Tecnologia
 *
 * Author: Alberto Cardellino
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/modules/libraries/icubmod/coman.h
 *
 */

//
// $Id: comanMotionControl.h,v 1.0 2013/02/5 $
//

#ifndef __comanMotionControl_h__
#define __comanMotionControl_h__

//#undef __cplucplus

using namespace std;
using namespace yarp::os;

/////  Yarp stuff
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include "yarp/dev/ControlBoardInterfacesImpl.inl" //ControlBoardHelper
///////////////////

#include <iCub/FactoryInterface.h>


// debug interface
#include <iCub/DebugInterfaces.h>

// ACE udp socket
#include <ace/ACE.h>
#include <ace/SOCK_Dgram_Bcast.h>

#include "comanFTsensor.h"

#include "DSP_board.h"
#include "Boards_iface.h"

#define SIZE_INFO   128




#ifdef _OLD_STYLE_
//
//   Help structure
//

struct ImpedanceLimits
{
    double min_stiff;
    double max_stiff;
    double min_damp;
    double max_damp;
    double param_a;
    double param_b;
    double param_c;

public:
    ImpedanceLimits()
    {
        min_stiff=0;
        max_stiff=0;
        min_damp=0;
        max_damp=0;
        param_a=0;
        param_b=0;
        param_c=0;
    }

    double get_min_stiff() {
        return min_stiff;
    }
    double get_max_stiff() {
        return max_stiff;
    }
    double get_min_damp()  {
        return min_damp;
    }
    double get_max_damp()  {
        return max_damp;
    }
};

struct ImpedanceParameters
{
    double stiffness;
    double damping;
    bool   enabled;
    ImpedanceLimits limits;
    ImpedanceParameters() {
        stiffness=0;
        damping=0;
        enabled=false;
    }
};

struct SpeedEstimationParameters
{
    double jnt_Vel_estimator_shift;
    double jnt_Acc_estimator_shift;
    double mot_Vel_estimator_shift;
    double mot_Acc_estimator_shift;

    SpeedEstimationParameters()
    {
        jnt_Vel_estimator_shift=0;
        jnt_Acc_estimator_shift=0;
        mot_Vel_estimator_shift=0;
        mot_Acc_estimator_shift=0;
    }
};
#endif


namespace yarp {
    namespace dev {
        class comanMotionControl;
        }
    }

static void copyPid_iCub2Coman(const Pid *iCubPid_in, GainSet Coman_pidType, pid_gains_t *ComanPid_out);
static void copyPid_Coman2iCub(pid_gains_t *ComanPid_in, Pid *iCubPid_out, GainSet *Coman_pidType);


class yarp::dev::comanMotionControl:  public DeviceDriver,
    public IPidControlRaw,
    public IControlCalibration2Raw,
    public IAmplifierControlRaw,
    public IEncodersTimedRaw,
    public ImplementEncodersTimed,
    public IPositionControlRaw,
    public IVelocityControlRaw,
    public IControlModeRaw,
    public IControlLimitsRaw,
    public ImplementControlLimits<comanMotionControl, IControlLimits>,
    public ImplementControlMode,
    public ImplementAmplifierControl<comanMotionControl, IAmplifierControl>,
    public ImplementPositionControl<comanMotionControl, IPositionControl>,
    public ImplementControlCalibration2<comanMotionControl, IControlCalibration2>,
    public ImplementPidControl<comanMotionControl, IPidControl>,
    public ImplementVelocityControl<comanMotionControl, IVelocityControl>,
    public ImplementDebugInterface,
    public IDebugInterfaceRaw
{
private:

     ///////////// coman specific  ///////////////
    // TODO separate the FT sensor hanfling from MC!!
    comanFTsensor         *FTsensor;
    Boards_ctrl           *boards_ctrl;
    mcs_map_t             _mcs;
    GainSet               controlMode;                        // memorize the type of control currently running

    ////////  canonical
    yarp::os::Semaphore   _mutex;

    int                   *_axisMap;                          /** axis remapping lookup-table */
    double                *_angleToEncoder;                   /** angle to iCubDegrees conversion factors */
    float                 *_encoderconversionfactor;          /** iCubDegrees to encoder conversion factors */
    float                 *_encoderconversionoffset;          /** iCubDegrees offset */
    double                *_rotToEncoder;                     /** angle to rotor conversion factors */
    double                *_zeros;                            /** encoder zeros */
    Pid                   *_pids;                             /** initial gains */
    Pid                   *_tpids;                            /** initial torque gains */
    bool                  _tpidsEnabled;                      /** abilitation for torque gains */


    double                *_limitsMin;                        /** joint limits, max*/
    double                *_limitsMax;                        /** joint limits, min*/
    double                *_currentLimits;                    /** current limits */
    int                   *_velocityShifts;                   /** velocity shifts */
    int                   *_velocityTimeout;                  /** velocity shifts */
    int                   *_torqueSensorId;                   /** Id of associated Joint Torque Sensor */
    int                   *_torqueSensorChan;                 /** Channel of associated Joint Torque Sensor */
    double                *_maxTorque;                        /** Max torque of a joint */
    double                *_newtonsToSensor;                  /** Newtons to force sensor units conversion factors */
    bool                  *checking_motiondone;               /* flag if I' m already waiting for motion done */

// basic knowledge of my joints
    int                   _njoints;                           // Number of joints handled by this EMS; this values will be extracted by the config file
    int                   _firstJoint;                        // in case the EMS controls joints from x to y where x is not 0, functions like setpidS need to know how to run the for loop


    // internal stuff
    bool                  *_enabledAmp;             // Middle step toward a full enabled motor controller. Amp (pwm) plus Pid enable command must be sent in order to get the joint into an active state.
    bool                  *_enabledPid;             // Depends on enabledAmp. When both are set, the joint exits the idle mode and goes into position mode. If one of them is disabled, it falls to idle.
    bool                  *_calibrated;             // Flag to know if the calibrate function has been called for the joint
    double                *_ref_positions;          // used for position control.
    double                *_ref_speeds;             // used for position control.
    double                *_command_speeds;         // used for velocity control.
    double                *_ref_accs;               // for velocity control, in position min jerk eq is used.
    double                *_ref_torques;            // for torque control.

#if 0
    DebugParameters *_debug_params;             /** debug parameters */
    ImpedanceParameters *_impedance_params;   /** impedance parameters */
    ImpedanceLimits     *_impedance_limits;     /** impedancel imits */
    SpeedEstimationParameters *_estim_params;   /** parameters for speed/acceleration estimation */
#endif

private:
    bool extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size);

public:
    comanMotionControl();
    ~comanMotionControl();

    char                     info[SIZE_INFO];
    yarp::os::Semaphore         semaphore;

    /*Device Driver*/
    virtual bool open(yarp::os::Searchable &par);
    virtual bool close();
    bool fromConfig(yarp::os::Searchable &config);

//    void waitSem();
//    void postSem();
    bool alloc(int njoints);
    bool init(void);

    ///////// 	PID INTERFACE		/////////
    virtual bool setPidRaw(int j, const Pid &pid);
    virtual bool setPidsRaw(const Pid *pids);
    virtual bool setReferenceRaw(int j, double ref);
    virtual bool setReferencesRaw(const double *refs);
    virtual bool setErrorLimitRaw(int j, double limit);
    virtual bool setErrorLimitsRaw(const double *limits);
    virtual bool getErrorRaw(int j, double *err);
    virtual bool getErrorsRaw(double *errs);
    virtual bool getOutputRaw(int j, double *out);
    virtual bool getOutputsRaw(double *outs);
    virtual bool getPidRaw(int j, Pid *pid);
    virtual bool getPidsRaw(Pid *pids);
    virtual bool getReferenceRaw(int j, double *ref);
    virtual bool getReferencesRaw(double *refs);
    virtual bool getErrorLimitRaw(int j, double *limit);
    virtual bool getErrorLimitsRaw(double *limits);
    virtual bool resetPidRaw(int j);
    virtual bool disablePidRaw(int j);
    virtual bool enablePidRaw(int j);
    virtual bool setOffsetRaw(int j, double v);

    /// POSITION CONTROL INTERFACE RAW
    virtual bool getAxes(int *ax);
    virtual bool setPositionModeRaw();
    virtual bool positionMoveRaw(int j, double ref);
    virtual bool positionMoveRaw(const double *refs);
    virtual bool relativeMoveRaw(int j, double delta);
    virtual bool relativeMoveRaw(const double *deltas);
    virtual bool checkMotionDoneRaw(bool *flag);
    virtual bool checkMotionDoneRaw(int j, bool *flag);
    virtual bool setRefSpeedRaw(int j, double sp);
    virtual bool setRefSpeedsRaw(const double *spds);
    virtual bool setRefAccelerationRaw(int j, double acc);
    virtual bool setRefAccelerationsRaw(const double *accs);
    virtual bool getRefSpeedRaw(int j, double *ref);
    virtual bool getRefSpeedsRaw(double *spds);
    virtual bool getRefAccelerationRaw(int j, double *acc);
    virtual bool getRefAccelerationsRaw(double *accs);
    virtual bool stopRaw(int j);
    virtual bool stopRaw();

    ///////// 	Velocity control interface raw	/////////
    virtual bool setVelocityModeRaw();
    virtual bool velocityMoveRaw(int j, double sp);
    virtual bool velocityMoveRaw(const double *sp);


    // calibration2raw
    virtual bool calibrate2Raw(int axis, unsigned int type, double p1, double p2, double p3);
    virtual bool doneRaw(int j);


    /////////////////////////////// END Position Control INTERFACE

    // ControlMode
    virtual bool setPositionModeRaw(int j);
    virtual bool setVelocityModeRaw(int j);
    virtual bool setTorqueModeRaw(int j);
    virtual bool setImpedancePositionModeRaw(int j);
    virtual bool setImpedanceVelocityModeRaw(int j);
    virtual bool setOpenLoopModeRaw(int j);
    virtual bool getControlModeRaw(int j, int *v);
    virtual bool getControlModesRaw(int* v);

    //////////////////////// BEGIN EncoderInterface
    //
    virtual bool resetEncoderRaw(int j);
    virtual bool resetEncodersRaw();
    virtual bool setEncoderRaw(int j, double val);
    virtual bool setEncodersRaw(const double *vals);
    virtual bool getEncoderRaw(int j, double *v);
    virtual bool getEncodersRaw(double *encs);
    virtual bool getEncoderSpeedRaw(int j, double *sp);
    virtual bool getEncoderSpeedsRaw(double *spds);
    virtual bool getEncoderAccelerationRaw(int j, double *spds);
    virtual bool getEncoderAccelerationsRaw(double *accs);
    ///////////////////////// END Encoder Interface

    virtual bool getEncodersTimedRaw(double *encs, double *stamps);
    virtual bool getEncoderTimedRaw(int j, double *encs, double *stamp);

    ////// Amplifier interface
    virtual bool enableAmpRaw(int j);
    virtual bool disableAmpRaw(int j);
    virtual bool getCurrentsRaw(double *vals);
    virtual bool getCurrentRaw(int j, double *val);
    virtual bool setMaxCurrentRaw(int j, double val);
    virtual bool getAmpStatusRaw(int *st);
    virtual bool getAmpStatusRaw(int j, int *st);
    /////////////// END AMPLIFIER INTERFACE

    //----------------------------------------------\\
    //	Debug interface
    //----------------------------------------------\\

    /* Set a generic parameter (for debug)
     * @param type is the CAN code representing the command message
     * @return true/false on success/failure
     */
    bool setParameterRaw(int j, unsigned int type, double value);

    /* Get a generic parameter (for debug)
     * @param type is the CAN code representing the command message
     * @return true/false on success/failure
     */
    bool getParameterRaw(int j, unsigned int type, double* value);

    /* Set a generic parameter (for debug)
     * @param index is the number of the debug parameter
     * @return true/false on success/failure
     */
    bool setDebugParameterRaw(int j, unsigned int index, double value);

    /* Set an instantaneous reference postion (for debug), bypassing the minimum jerk
     * @param index is the number of the debug parameter
        * @return true/false on success/failure
     */
    bool setDebugReferencePositionRaw(int j, double value);

    /* Get an instantaneous reference postion (for debug), bypassing the minimum jerk
     * @param index is the number of the debug parameter
     * @return true/false on success/failure
     */
    bool getDebugParameterRaw(int j, unsigned int index, double* value);
    bool getDebugReferencePositionRaw(int j, double* value);
    bool getRotorPositionRaw         (int j, double* value);
    bool getRotorPositionsRaw        (double* value);
    bool getRotorSpeedRaw            (int j, double* value);
    bool getRotorSpeedsRaw           (double* value);
    bool getRotorAccelerationRaw     (int j, double* value);
    bool getRotorAccelerationsRaw    (double* value);
    bool getJointPositionRaw         (int j, double* value);
    bool getJointPositionsRaw        (double* value);


    /////// Limits
    bool setLimitsRaw(int axis, double min, double max);
    bool getLimitsRaw(int axis, double *min, double *max);
};

#endif // include guard
