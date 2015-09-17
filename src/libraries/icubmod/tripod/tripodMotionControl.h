// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/* Copyright (C) 2015  iCub Facility, Istituto Italiano di Tecnologia
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


/**
 * @ingroup icub_hardware_modules
 * \defgroup tripod motion device
 *
 *
 * Copyright (C) 2015  iCub Facility, Istituto Italiano di Tecnologia
 *
 * Author: Alberto Cardellino
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This device is designed to interface the user input expressed in torso elevation plus
 * orientation into 3 elongation of the real actuator. This conversion is done by
 * a Inverse Kinematic (IK) library. The result will then be sent to the electronic
 * boards controlling the motors, using needed protocol, could it be can or ethernet.
 */


#ifndef __tripodMotionControlh__
#define __tripodMotionControlh__

//  Yarp stuff
#include <stdint.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/dev/ControlBoardInterfacesImpl.inl>


using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

namespace icub {
    namespace dev  {
    class tripodMotionControl;
        namespace impl {
            class HW_deviceHelper;
        }
    }
}

using namespace yarp::dev;

class icub::dev::impl::HW_deviceHelper
{
public:
    bool _subDevVerbose;

    yarp::dev::IPidControl           *pid;
    yarp::dev::IPositionControl      *pos;
    yarp::dev::IPositionControl2     *pos2;
    yarp::dev::IVelocityControl      *vel;
    yarp::dev::IVelocityControl2     *vel2;
    yarp::dev::IEncodersTimed        *iJntEnc;
    yarp::dev::IMotorEncoders        *iMotEnc;
    yarp::dev::IAmplifierControl     *amp;
    yarp::dev::IControlLimits2       *lim2;
    yarp::dev::IControlCalibration   *calib;
    yarp::dev::IControlCalibration2  *calib2;
    yarp::dev::ITorqueControl        *iTorque;
    yarp::dev::IImpedanceControl     *iImpedance;
    yarp::dev::IOpenLoopControl      *iOpenLoop;
    yarp::dev::IControlMode          *iMode;
    yarp::dev::IControlMode2         *iMode2;
    yarp::dev::IAxisInfo             *info;
    yarp::dev::IPositionDirect       *posDir;
    yarp::dev::IInteractionMode      *iInteract;
    yarp::dev::IMotor                *imotor;
    yarp::dev::IRemoteVariables      *iVar;

    HW_deviceHelper();
    ~HW_deviceHelper();

    bool isConfigured();
    bool attach(yarp::dev::PolyDriver *d);
    void detach();

private:
    bool configured;
};


class icub::dev::tripodMotionControl:   public DeviceDriver,
                                        public IMultipleWrapper,
//                                         public IPidControlRaw,
                                        public IControlCalibration2Raw,
                                        public ImplementControlCalibration2<tripodMotionControl, IControlCalibration2>,
                                        public IEncodersTimedRaw,
                                        public ImplementEncodersTimed,
                                        public IMotorEncodersRaw,
                                        public ImplementMotorEncoders,
//                                         public IMotorRaw,
//                                         public ImplementMotor,
                                        public IPositionControl2Raw,
                                        public ImplementPositionControl2,
                                        public IVelocityControl2Raw,
                                        public ImplementVelocityControl2,
                                        public IControlMode2Raw,
                                        public ImplementControlMode2,
//                                         public IControlLimits2Raw,
//                                         public IImpedanceControlRaw,
//                                         public ImplementControlLimits2,
//                                         public ImplementPidControl<tripodMotionControl, IPidControl>,
//                                         public ImplementVelocityControl<tripodMotionControl, IVelocityControl>,
                                        public IPositionDirectRaw,
                                        public ImplementPositionDirect,
                                        public IInteractionModeRaw,
                                        public ImplementInteractionMode
//                                         public IOpenLoopControlRaw,
//                                         public ImplementOpenLoopControl
{
private:
    bool verbose;
    bool useRemoteCB;                           /** if TRUE it means we want to connect the tripodMotionControl to real HW device using yarp network. This allows also to connect to a simulator                                                 * if FALSE then we wait for the 'attachAll' function to be called in order to get the pointer to the low-level device like canBus/embObjMotionControl. */

    yarp::os::Semaphore                      _mutex;
    icub::dev::impl::HW_deviceHelper         _device;
    yarp::dev::PolyDriver                   *_polyDriverDevice;

    int *_axisMap;                              /** axis remapping lookup-table */
    double *_angleToEncoder;                    /** angle conversion factor, if any */
    double  *_encodersStamp;                    /** keep information about acquisition time for encoders read */

    double *_limitsMin;                         /** joint limits, max*/
    double *_limitsMax;                         /** joint limits, min*/
    double *_kinematic_mj;                      /** the kinematic coupling matrix from joints space to motor space */
    double *_currentLimits;                     /** current limits */
    bool  *checking_motiondone;                 /* flag telling if I'm already waiting for motion done */
    bool        useRawEncoderData;

    // basic knowledge of my joints
    int   _njoints;                             // Number of joints handled by this EMS; this values will be extracted by the config file


    // internal stuff
    bool    *_calibrated;       // Flag to know if the calibrate function has been called for the joint
    double  *_userRef_positions;     // used for position control.
    double  *_robotRef_positions;    // used for position control.
    double  *_ref_speeds;       // used for position control.
    double  *_command_speeds;   // used for velocity control.
    double  *_ref_accs;         // for velocity control, in position min jerk eq is used.
    double  *_lastUser_encoders;     // used for position control.
    double  *_lastRobot_encoders;    // used for position control.

#if 0
    float *_DEPRECATED_encoderconversionfactor;            /** iCubDegrees to encoder conversion factors */
    float *_DEPRECATED_encoderconversionoffset;            /** iCubDegrees offset */
    uint8_t *_jointEncoderType;                 /** joint encoder type*/
    int    *_jointEncoderRes;                   /** joint encoder resolution */
    int    *_rotorEncoderRes;                   /** rotor encoder resolution */
    uint8_t *_rotorEncoderType;                 /** rotor encoder type*/
    double *_gearbox;                           /** the gearbox ratio */
    bool   *_hasHallSensor;                     /** */
    bool   *_hasTempSensor;                     /** */
    bool   *_hasRotorEncoder;                   /** */
    bool   *_hasRotorEncoderIndex;              /** */
    int    *_rotorIndexOffset;                  /** */
    int    *_motorPoles;                        /** */
    bool   _pwmIsLimited;                       /** set to true if pwm is limited */
    int *_velocityShifts;                       /** velocity shifts */
    int *_velocityTimeout;                      /** velocity shifts */
    double *_kbemf;                             /** back-emf compensation parameter */
    double *_ktau;                              /** motor torque constant */
    int * _filterType;                          /** the filter type (int value) used by the force control algorithm */
    int *_torqueSensorId;                       /** Id of associated Joint Torque Sensor */
    int *_torqueSensorChan;                     /** Channel of associated Joint Torque Sensor */
    double *_maxTorque;                         /** Max torque of a joint */
    double *_newtonsToSensor;                   /** Newtons to force sensor units conversion factors */
    enum       positionControlUnitsType {P_MACHINE_UNITS=0, P_METRIC_UNITS=1};
    positionControlUnitsType _positionControlUnits;
    double       SAFETY_THRESHOLD;
    Pid *_pids;                                 /** initial gains */
    Pid *_tpids;                                /** initial torque gains */
    Pid *_cpids;                                /** initial current gains */
    double *_last_position_move_time;           /** time stamp for last received position move command*/
#endif

private:

    inline bool NOT_YET_IMPLEMENTED(const char *txt);
    inline bool DEPRECATED(const char *txt);

    bool extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size);
    bool parsePositionPidsGroup(Bottle& pidsGroup, Pid myPid[]);
    bool parseTorquePidsGroup(Bottle& pidsGroup, Pid myPid[], double kbemf[], double ktau[], int filterType[]);

    bool alloc(int njoints);
    bool dealloc();

    bool fromConfig(yarp::os::Searchable &config);
    void cleanup(void);
    bool init(void);

    void copyPid_iCub2eo(const Pid *in, Pid *out);
    void copyPid_eo2iCub(Pid *in, Pid *out);

    bool tripod_user2HW(double *user,  double *robot);
    bool tripod_HW2user(double *robot, double *user);

public:

    tripodMotionControl();
    ~tripodMotionControl();

    // Device Driver
    virtual bool open(yarp::os::Searchable &par);
    virtual bool close();

    // IMultipleWrapper interface
    virtual bool attachAll(const PolyDriverList &p);
    virtual bool detachAll();

    bool refreshEncoders(double *times);
    void refreshEncoderTimeStamp(int joint);
    Semaphore               semaphore;
    yarp::os::ConstString   deviceDescription;

#if 0
    /////////   PID INTERFACE   /////////
    virtual bool setPidRaw(int j, const Pid &pid);
    virtual bool setPidsRaw(const Pid *pids);
    virtual bool setReferenceRaw(int j, double ref);
    virtual bool setReferencesRaw(const double *refs);
    virtual bool setErrorLimitRaw(int j, double limit);
    virtual bool setErrorLimitsRaw(const double *limits);
    virtual bool getErrorRaw(int j, double *err);
    virtual bool getErrorsRaw(double *errs);
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
#endif

    /////////// POSITION CONTROL INTERFACE RAW
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

    // Position Control2 Interface
    virtual bool positionMoveRaw(const int n_joint, const int *joints, const double *refs);
    virtual bool relativeMoveRaw(const int n_joint, const int *joints, const double *deltas);
    virtual bool checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags);
    virtual bool setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds);
    virtual bool setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs);
    virtual bool getRefSpeedsRaw(const int n_joint, const int *joints, double *spds);
    virtual bool getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs);
    virtual bool stopRaw(const int n_joint, const int *joints);

    //  Velocity control interface raw
    virtual bool setVelocityModeRaw();
    virtual bool velocityMoveRaw(int j, double sp);
    virtual bool velocityMoveRaw(const double *sp);

    // calibration2raw
    virtual bool setCalibrationParametersRaw(int axis, const CalibrationParameters& params);
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
    virtual bool getControlModesRaw(int *v);

    // ControlMode 2
    virtual bool getControlModesRaw(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModeRaw(const int j, const int mode);
    virtual bool setControlModesRaw(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModesRaw(int *modes);

    //////////////////////// BEGIN EncoderInterface
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


    //////////////////////// BEGIN MotorEncoderInterface
    virtual bool getNumberOfMotorEncodersRaw(int * num);
    virtual bool resetMotorEncoderRaw(int m);
    virtual bool resetMotorEncodersRaw();
    virtual bool setMotorEncoderRaw(int m, const double val);
    virtual bool setMotorEncodersRaw(const double *vals);
    virtual bool getMotorEncoderRaw(int m, double *v);
    virtual bool getMotorEncodersRaw(double *encs);
    virtual bool getMotorEncoderSpeedRaw(int m, double *sp);
    virtual bool getMotorEncoderSpeedsRaw(double *spds);
    virtual bool getMotorEncoderAccelerationRaw(int m, double *spds);
    virtual bool getMotorEncoderAccelerationsRaw(double *accs);
    virtual bool getMotorEncodersTimedRaw(double *encs, double *stamps);
    virtual bool getMotorEncoderTimedRaw(int m, double *encs, double *stamp);
    virtual bool getMotorEncoderCountsPerRevolutionRaw(int m, double *v);
    virtual bool setMotorEncoderCountsPerRevolutionRaw(int m, const double cpr);
    ///////////////////////// END MotorEncoder Interface

#if 0
    //////////////////////// BEGIN RemoteVariables Interface
    virtual bool getRemoteVariableRaw(yarp::os::ConstString key, yarp::os::Bottle& val);
    virtual bool setRemoteVariableRaw(yarp::os::ConstString key, const yarp::os::Bottle& val);
    virtual bool getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys);
    ///////////////////////// END RemoteVariables Interface

    //Internal use, not exposed by Yarp (yet)
    virtual bool getGearboxRatioRaw(int m, double *gearbox);
    virtual bool getRotorEncoderResolutionRaw(int m, double &rotres);
    virtual bool getJointEncoderResolutionRaw(int m, double &jntres);
    virtual bool getJointEncoderTypeRaw(int j, int &type);
    virtual bool getRotorEncoderTypeRaw(int j, int &type);
    virtual bool getKinematicMJRaw(int j, double &rotres);
    virtual bool getHasTempSensorsRaw(int j, int& ret);
    virtual bool getHasHallSensorRaw(int j, int& ret);
    virtual bool getHasRotorEncoderRaw(int j, int& ret);
    virtual bool getHasRotorEncoderIndexRaw(int j, int& ret);
    virtual bool getMotorPolesRaw(int j, int& poles);
    virtual bool getRotorIndexOffsetRaw(int j, double& rotorOffset);
    virtual bool getCurrentPidRaw(int j, Pid *pid);
    virtual bool getTorqueControlFilterType(int j, int& type);

    ////// Amplifier interface
    virtual bool enableAmpRaw(int j);
    virtual bool disableAmpRaw(int j);
    virtual bool getCurrentsRaw(double *vals);
    virtual bool getCurrentRaw(int j, double *val);
    virtual bool setMaxCurrentRaw(int j, double val);
    virtual bool getMaxCurrentRaw(int j, double *val);
    virtual bool getAmpStatusRaw(int *st);
    virtual bool getAmpStatusRaw(int j, int *st);
    /////////////// END AMPLIFIER INTERFACE

    // Limits
    bool setLimitsRaw(int axis, double min, double max);
    bool getLimitsRaw(int axis, double *min, double *max);
    // Limits 2
    bool setVelLimitsRaw(int axis, double min, double max);
    bool getVelLimitsRaw(int axis, double *min, double *max);

    // Torque control
    bool setTorqueModeRaw();
    bool getTorqueRaw(int j, double *t);
    bool getTorquesRaw(double *t);
    bool getBemfParamRaw(int j, double *bemf);
    bool setBemfParamRaw(int j, double bemf);
    bool getTorqueRangeRaw(int j, double *min, double *max);
    bool getTorqueRangesRaw(double *min, double *max);
    bool setRefTorquesRaw(const double *t);
    bool setRefTorqueRaw(int j, double t);
    bool setRefTorquesRaw(const int n_joint, const int *joints, const double *t);
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
    bool getMotorTorqueParamsRaw(int j, MotorTorqueParameters *params);
    bool setMotorTorqueParamsRaw(int j, const MotorTorqueParameters params);
#endif

    // IVelocityControl2
    bool velocityMoveRaw(const int n_joint, const int *joints, const double *spds);
    bool setVelPidRaw(int j, const Pid &pid);
    bool setVelPidsRaw(const Pid *pids);
    bool getVelPidRaw(int j, Pid *pid);
    bool getVelPidsRaw(Pid *pids);

#if 0
    bool getImpedanceRaw(int j, double *stiffness, double *damping);

    /** Set current impedance parameters (stiffness,damping) for a specific joint.
     * @return success/failure
     */
    bool setImpedanceRaw(int j, double stiffness, double damping);

    /** Set current force Offset for a specific joint.
     * @return success/failure
     */
    bool setImpedanceOffsetRaw(int j, double offset);

    /** Get current force Offset for a specific joint.
     * @return success/failure
     */
    bool getImpedanceOffsetRaw(int j, double *offset);

    /** Get the current impedandance limits for a specific joint.
     * @return success/failure
     */
    bool getCurrentImpedanceLimitRaw(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp);
#endif
    // PositionDirect Interface
    bool setPositionDirectModeRaw();
    bool setPositionRaw(int j, double ref);
    bool setPositionsRaw(const int n_joint, const int *joints, double *refs);
    bool setPositionsRaw(const double *refs);

    // InteractionMode interface
    bool getInteractionModeRaw(int j, yarp::dev::InteractionModeEnum* _mode);
    bool getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    bool getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);
    bool setInteractionModeRaw(int j, yarp::dev::InteractionModeEnum _mode);
    bool setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    bool setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);

#if 0
    // IMotor interface
    bool getNumberOfMotorsRaw(int * num);
    bool getTemperatureRaw(int m, double* val);
    bool getTemperaturesRaw(double *vals);
    bool getTemperatureLimitRaw(int m, double *temp);
    bool setTemperatureLimitRaw(int m, const double temp);
    bool getMotorOutputLimitRaw(int m, double *limit);
    bool setMotorOutputLimitRaw(int m, const double limit);
    
    // OPENLOOP interface
    bool setRefOutputRaw(int j, double v);
    bool setRefOutputsRaw(const double *v);
    bool getRefOutputRaw(int j, double *out);
    bool getRefOutputsRaw(double *outs);
    bool getOutputRaw(int j, double *out);
    bool getOutputsRaw(double *outs);
    bool setOpenLoopModeRaw();
#endif
};

#endif // include guard

