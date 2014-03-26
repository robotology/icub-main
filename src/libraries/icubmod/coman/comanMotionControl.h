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

using namespace std;
using namespace yarp::os;

/////  Yarp stuff
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/dev/DeviceDriver.h>

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

#include <DSP_board.h>
#include <Boards_iface.h>

#include <comanDevicesHandler.hpp>

#define SIZE_INFO   128
#define BC_POLICY_MOTOR_POSITION          (1 << 0)
#define BC_POLICY_MOTOR_VELOCITY          (1 << 1)
#define BC_POLICY_TORQUE_READING          (1 << 2)
#define BC_POLICY_OUTOUT_VOLTAGE          (1 << 3)
#define BC_POLICY_PID_ERROR               (1 << 4)
#define BC_POLICY_AVERAGE_MOTOR_CURRENT   (1 << 5)
#define BC_POLICY_TEMP_VOLTAGE_SUPPLY     (1 << 6)
#define BC_POLICY_TIMESTAMP               (1 << 7)

#define BC_POLICY_FAULT_FLAG              (1 << 8)
#define BC_POLICY_ANALOG_INPUT1           (1 << 9)
#define BC_POLICY_ANALOG_INPUT2           (1 << 10)
#define BC_POLICY_ABSOLUTE_POSITION       (1 << 11)
#define BC_POLICY_RAW_VELOCITY            (1 << 12)
#define BC_POLICY_MOTOR_STATE             (1 << 13)
#define BC_POLICY_RAW_MOTOR_CURRENT       (1 << 14)
#define BC_POLICY_RELATIVE_POSITION       (1 << 15)

static const double COMAN_POS_THRESHOLD     = 0.1f;
static const double COMAN_POS_TO_VEL_GAIN   = 100.0f;

namespace yarp {
    namespace dev {
        class comanMotionControl;
        }
    }

class yarp::dev::comanMotionControl:  public DeviceDriver,
    public IPidControlRaw,
    public IControlCalibration2Raw,
    public IAmplifierControlRaw,
    public IEncodersTimedRaw,
    public ImplementEncodersTimed,
    public IPositionControl2Raw,
    public ImplementPositionControl2,
    public IVelocityControl2Raw,
    public ImplementVelocityControl2,
    public IControlModeRaw,
//    public IControlLimitsRaw,
//    public ImplementControlLimits<comanMotionControl, IControlLimits>,
    public IControlLimits2Raw,
    public ImplementControlLimits2,
    public ImplementControlMode,
    public ImplementAmplifierControl<comanMotionControl, IAmplifierControl>,
    public ImplementControlCalibration2<comanMotionControl, IControlCalibration2>,
    public ImplementPidControl<comanMotionControl, IPidControl>,
    public ImplementDebugInterface,
    public ITorqueControlRaw,
    public ImplementTorqueControl,
    public IDebugInterfaceRaw,
    public IPositionDirectRaw,
    public ImplementPositionDirect,
    public IImpedanceControlRaw,
    public ImplementImpedanceControl
{
private:

     ///////////// coman specific  ///////////////
    int32_t               *pos_array;
    int32_t               *vel_array;
    int32_t               *trq_array;
    int32_t               *off_array;           // pid offset, used as a feedforward term in control
    uint16_t              bc_policy;
    uint16_t              extra_policy;
    int                   bc_rate;

    uint16_t              motor_config_mask_j1;     // motor configuration for joint 1 (as test on this joint)
    uint16_t              motor_config_mask2_j1;    // motor configuration for joint 1 (as test on this joint)
    Pid                   pid_j1;
    Pid                   pidTorque_j1;

    bool                  pos_changed;
    bool                  vel_changed;
    bool                  trq_changed;
    bool                  off_changed;

        ////////  CHECK FOR DUPLICATED!!  buffers array for multi-joints command
    int32_t               *_ref_positions;                    // used for position control.
    int16_t               *_ref_speeds;                       // used for position control.
    int16_t               *_ref_torques;                      // for torque control.
    double                *_command_speeds;                   // used for velocity control.
    double                *_ref_accs;                         // for velocity control, in position min jerk eq is used.
    int16_t               *_pid_offset;                       // for feedforward control.

    double                *_homePos;                          // initial position (usually it's stored in the calibrator)


    // handling classes
    comanDevicesHandler     *_comanHandler;
    Boards_ctrl             *_boards_ctrl;
    Boards_ctrl::mcs_map_t  _mcs;
    int                     *_controlMode;                        // memorize the type of control currently running... safe??
    ////////  canonical
    yarp::os::Semaphore   _mutex;

    int                   *_axisMap;                          /** axis remapping lookup-table */
    int                   *_bIdMap;                           /* conversion from joint number to bId */
    int                   *_inv_bIdMap;                       /* conversion back from bId to joint number */
    double                *_angleToEncoder;                   /** angle to iCubDegrees conversion factors */
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
    double                *_maxTorque;                        /** Max torque of a joint */
    double                *_newtonsToSensor;                  /** Newtons to force sensor units conversion factors */

// basic knowledge of my joints
    int                   _njoints;                           // Number of joints handled by this class; this values will be extracted by the config file

    // internal stuff
    bool                  *_enabledAmp;             // Middle step toward a full enabled motor controller. Amp (pwm) plus Pid enable command must be sent in order to get the joint into an active state.
    bool                  *_enabledPid;             // Depends on enabledAmp. When both are set, the joint exits the idle mode and goes into position mode. If one of them is disabled, it falls to idle.
    bool                  *_calibrated;             // Flag to know if the calibrate function has been called for the joint
    bool                   _initialPidConfigFound;

private:
    bool extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size);
    bool parsePidsGroup(Bottle& pidsGroup, Pid myPid[]);

    McBoard *getMCpointer(int j);
    int bId2Joint(int j);
    uint8_t jointTobId(int j);

//     double convertDoble2Int(double in[], int out[]);
//     double convertDoble2Short(double in[], short int out[]);
    uint16_t strtouli(ConstString asString, int arg2, int arg3);
public:
    comanMotionControl();
    virtual ~comanMotionControl();

    char                        info[SIZE_INFO];
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

    //// POSITION CONTROL INTERFACE RAW
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

    ////    Position Control 2 interface
    virtual bool setPositionModeRaw(const int n_joint, const int *joints);
    virtual bool positionMoveRaw(const int n_joint, const int *joints, const double *refs);
    virtual bool relativeMoveRaw(const int n_joint, const int *joints, const double *deltas);
    virtual bool checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags);
    virtual bool setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds);
    virtual bool getRefSpeedsRaw(const int n_joint, const int *joints, double *spds);
    virtual bool setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs);
    virtual bool getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs);
    virtual bool stopRaw(const int n_joint, const int *joints);

    ////    Velocity control interface  /////////
    virtual bool setVelocityModeRaw();
    virtual bool velocityMoveRaw(int j, double sp);
    virtual bool velocityMoveRaw(const double *sp);

    ////    Velocity Control 2 interface
    virtual bool setVelocityModeRaw(const int n_joint, const int *joints);
    virtual bool velocityMoveRaw(const int n_joint, const int *joints, const double *spds);
    virtual bool setVelPidRaw(int j, const Pid &pid);
    virtual bool setVelPidsRaw(const Pid *pids);
    virtual bool setVelPidsRaw(const int n_joint, const int *joints, const Pid *pids);
    virtual bool getVelPidRaw(int j, Pid *pid);
    virtual bool getVelPidsRaw(const int n_joint, const int *joints, Pid *pids);
    virtual bool getVelPidsRaw(Pid *pids);

    // calibration2raw
    virtual bool calibrate2Raw(int axis, unsigned int type, double p1, double p2, double p3);
    virtual bool doneRaw(int j);


    // ControlMode
    virtual bool setPositionModeRaw(int j);
    virtual bool setVelocityModeRaw(int j);
    virtual bool setTorqueModeRaw(int j);
    virtual bool setImpedancePositionModeRaw(int j);
    virtual bool setImpedanceVelocityModeRaw(int j);
    virtual bool setOpenLoopModeRaw(int j);
    virtual bool getControlModeRaw(int j, int *v);
    virtual bool getControlModesRaw(int* v);

    //////// BEGIN EncoderInterface
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
    //  Debug interface
    //----------------------------------------------\\

    bool setParameterRaw(int j, unsigned int type, double value);
    bool getParameterRaw(int j, unsigned int type, double* value);
    bool setDebugParameterRaw(int j, unsigned int index, double value);

    bool setDebugReferencePositionRaw(int j, double value);
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


    //----------------------------------------------\\
    //  Limits interface
    //----------------------------------------------\\

    bool setLimitsRaw(int axis, double min, double max);
    bool getLimitsRaw(int axis, double *min, double *max);
    bool setVelLimitsRaw(int axis, double min, double max);
    bool getVelLimitsRaw(int axis, double *min, double *max);

    //----------------------------------------------\\
    //  Torque interface
    //----------------------------------------------\\

    bool setTorqueModeRaw();
    bool getTorqueRaw(int j, double *t);
    bool getTorquesRaw(double *t);
    bool getBemfParamRaw(int j, double *bemf);
    bool setBemfParamRaw(int j, double bemf);
    bool getTorqueRangeRaw(int j, double *min, double *max);
    bool getTorqueRangesRaw(double *min, double *max);
    bool setRefTorquesRaw(const double *t);
    bool setRefTorqueRaw(int j, double t);
    bool getRefTorquesRaw(double *t);
    bool getRefTorqueRaw(int j, double *t);
    bool setTorquePidRaw(int j, const Pid &pid);
    bool setTorquePidsRaw(const Pid *pids);
    bool setTorqueErrorLimitRaw(int j, double limit);
    bool setTorqueErrorLimitsRaw(const double *limits);
    bool getTorqueErrorRaw(int j, double *err);
    bool getTorqueErrorsRaw(double *errs);
    bool getTorquePidOutputRaw(int j, double *out);
    bool getTorquePidOutputsRaw(double *outs);
    bool getTorquePidRaw(int j, Pid *pid);
    bool getTorquePidsRaw(Pid *pids);
    bool getTorqueErrorLimitRaw(int j, double *limit);
    bool getTorqueErrorLimitsRaw(double *limits);
    bool resetTorquePidRaw(int j);
    bool disableTorquePidRaw(int j);
    bool enableTorquePidRaw(int j);
    bool setTorqueOffsetRaw(int j, double v);


    bool setPositionRaw(int j, double ref);
    bool setPositionsRaw(const int n_joint, const int *joints, double *refs);
    bool setPositionsRaw(const double *refs);


    //----------------------------------------------\\
    //  Impedance Control interface
    //----------------------------------------------\\

    bool getImpedanceRaw(int j, double *stiffness, double *damping);
    bool setImpedanceRaw(int j, double stiffness, double damping);
    bool setImpedanceOffsetRaw(int j, double offset);
    bool getImpedanceOffsetRaw(int j, double* offset);
    bool getCurrentImpedanceLimitRaw(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp);
};

#endif // include guard
