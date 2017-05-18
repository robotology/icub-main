// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Vadim Tikhanoff, Paul Fitzpatrick, Giorgio Metta
* email:    vadim.tikhanoff@iit.it, paulfitz@alum.mit.edu, giorgio.metta@iit.it
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

/**
 * \file iCubSimulationControl.h
 * \brief This is the header file for the yarp interface of the iCubSimulation.
 * \author Vadim Tikhanoff, Paul Fitzpatrick, Giorgio Metta
 * \date 2007
 * \note Release under GNU GPL v2.0
 **/
#ifndef __iCubSimulationControlh__
#define __iCubSimulationControlh__

#include <yarp/os/Bottle.h>
#include <yarp/os/Semaphore.h>
//#include <yarp/os/RateThread.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Matrix.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>

#include "LogicalJoints.h"

//control mode definitions
#define MODE_IDLE                       0x00
#define MODE_POSITION                   0x01
#define MODE_VELOCITY                   0x02
#define MODE_TORQUE                     0x03
#define MODE_IMPEDANCE_POS              0x04
#define MODE_IMPEDANCE_VEL              0x05
#define MODE_PWM                        0x50
#define MODE_CURRENT                    0x60
#define MODE_MIXED                      VOCAB_CM_MIXED
#define MODE_FORCE_IDLE                 VOCAB_CM_FORCE_IDLE
#define MODE_HW_FAULT                   VOCAB_CM_HW_FAULT
#define MODE_CALIBRATING                VOCAB_CM_CALIBRATING
#define MODE_CALIB_DONE                 VOCAB_CM_CALIB_DONE
#define MODE_NOT_CONFIGURED             VOCAB_CM_NOT_CONFIGURED
#define MODE_CONFIGURED                 VOCAB_CM_CONFIGURED
#define MODE_UNKNOWN                    VOCAB_CM_UNKNOWN

// not used
//#define MODE_CALIB_ABS_POS_SENS         0x10
//#define MODE_CALIB_HARD_STOPS           0x20
//#define MODE_HANDLE_HARD_STOPS          0x30
//#define MODE_MARGIN_REACHED             0x40
//#define MODE_CALIB_ABS_AND_INCREMENTAL  0x41


namespace yarp{
    namespace dev{
        class iCubSimulationControl;
    }
}
class yarp::dev::iCubSimulationControl :
    public DeviceDriver,
    //public yarp::os::RateThread, 
    public IPositionControl2Raw,
    public ImplementPositionControl2,
    public IVelocityControl2Raw,
    public ImplementVelocityControl2,
    public ITorqueControlRaw,
    public ImplementTorqueControl,
    public IAmplifierControlRaw,
    public ImplementAmplifierControl<iCubSimulationControl, IAmplifierControl>,
    public IControlCalibrationRaw,
    public ImplementControlCalibration<iCubSimulationControl, IControlCalibration>,
    public IControlLimits2Raw,
    public ImplementControlLimits2,
    public IControlMode2Raw,
    public ImplementControlMode2,
    public IInteractionModeRaw,
    public ImplementInteractionMode,
    public IPidControlRaw,
    public ImplementPidControl,
    public IEncodersTimedRaw,
    public ImplementEncodersTimed,
    public IMotorEncodersRaw,
    public ImplementMotorEncoders,
    public IMotorRaw,
    public ImplementMotor,
    public IPositionDirectRaw,
    public ImplementPositionDirect,
    public IRemoteVariablesRaw,
    public ImplementRemoteVariables,
    public IAxisInfoRaw,
    public ImplementAxisInfo,
    public IPWMControlRaw,
    public ImplementPWMControl,
    public ICurrentControlRaw,
    public ImplementCurrentControl
{
 private:
  iCubSimulationControl(const iCubSimulationControl&);
  void operator=(const iCubSimulationControl&);
  int ControlModes_yarp2iCubSIM(int yarpMode);
  int ControlModes_iCubSIM2yarp(int iCubMode);

 public:
  /**
   * Default constructor. Construction is done in two stages, first build the
   * object and then open the device driver.
   */
  iCubSimulationControl();
  
  /**
   * Destructor.
   */
  virtual ~iCubSimulationControl();
  
  /**
   * Open the device driver and start communication with the hardware.
   * @param config is a Searchable object containing the list of parameters.
   * @return true on success/failure.
   */
  virtual bool open(yarp::os::Searchable& config);

  virtual bool close(void);

  // IEncoderTimedRaw

  virtual bool getEncodersTimedRaw(double *encs, double *stamps);
  virtual bool getEncoderTimedRaw(int j, double *encs, double *stamp);


  ///////////// PID INTERFACE
  //
  virtual bool setPidRaw(const PidControlTypeEnum& pidtype, int j, const Pid &pid);
  virtual bool setPidsRaw(const PidControlTypeEnum& pidtype, const Pid *pids);
  virtual bool setPidReferenceRaw(const PidControlTypeEnum& pidtype, int j, double ref);
  virtual bool setPidReferencesRaw(const PidControlTypeEnum& pidtype, const double *refs);
  virtual bool setPidErrorLimitRaw(const PidControlTypeEnum& pidtype, int j, double limit);
  virtual bool setPidErrorLimitsRaw(const PidControlTypeEnum& pidtype, const double *limits);
  virtual bool getPidErrorRaw(const PidControlTypeEnum& pidtype, int j, double *err);
  virtual bool getPidErrorsRaw(const PidControlTypeEnum& pidtype, double *errs);
  virtual bool getPidOutputRaw(const PidControlTypeEnum& pidtype, int j, double *out);
  virtual bool getPidOutputsRaw(const PidControlTypeEnum& pidtype, double *outs);
  virtual bool getPidRaw(const PidControlTypeEnum& pidtype, int j, Pid *pid);
  virtual bool getPidsRaw(const PidControlTypeEnum& pidtype, Pid *pids);
  virtual bool getPidReferenceRaw(const PidControlTypeEnum& pidtype, int j, double *ref);
  virtual bool getPidReferencesRaw(const PidControlTypeEnum& pidtype, double *refs);
  virtual bool getPidErrorLimitRaw(const PidControlTypeEnum& pidtype, int j, double *limit);
  virtual bool getPidErrorLimitsRaw(const PidControlTypeEnum& pidtype, double *limits);
  virtual bool resetPidRaw(const PidControlTypeEnum& pidtype, int j);
  virtual bool disablePidRaw(const PidControlTypeEnum& pidtype, int j);
  virtual bool enablePidRaw(const PidControlTypeEnum& pidtype, int j);
  virtual bool setPidOffsetRaw(const PidControlTypeEnum& pidtype, int j, double v);
  virtual bool isPidEnabledRaw(const PidControlTypeEnum& pidtype, int j, bool* enabled);
  //
  /////////////////////////////// END PID INTERFACE

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

  // Position Control2 Interface
  virtual bool positionMoveRaw(const int n_joint, const int *joints, const double *refs);
  virtual bool relativeMoveRaw(const int n_joint, const int *joints, const double *deltas);
  virtual bool checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags);
  virtual bool setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds);
  virtual bool setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs);
  virtual bool getRefSpeedsRaw(const int n_joint, const int *joints, double *spds);
  virtual bool getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs);
  virtual bool stopRaw(const int n_joint, const int *joints);
  virtual bool getTargetPositionRaw(const int joint, double *ref);
  virtual bool getTargetPositionsRaw(double *refs);
  virtual bool getTargetPositionsRaw(const int n_joint, const int *joints, double *refs);

  // Velocity Control
  virtual bool setVelocityModeRaw();
  virtual bool velocityMoveRaw(int j, double sp);
  virtual bool velocityMoveRaw(const double *sp);
  
  // Velocity Control2 Interface
  virtual bool velocityMoveRaw(const int n_joint, const int *joints, const double *spds);
  virtual bool setVelPidRaw(int j, const yarp::dev::Pid &pid);
  virtual bool setVelPidsRaw(const yarp::dev::Pid *pids);
  virtual bool getVelPidRaw(int j, yarp::dev::Pid *pid);
  virtual bool getVelPidsRaw(yarp::dev::Pid *pids);
  virtual bool getRefVelocityRaw(const int joint, double *ref);
  virtual bool getRefVelocitiesRaw(double *refs);
  virtual bool getRefVelocitiesRaw(const int n_joint, const int *joints, double *refs);


  //////////////////////// BEGIN Encoder Interface
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

  //////////////////////// BEGIN RemoteVariables Interface
  //
  virtual bool getRemoteVariableRaw(yarp::os::ConstString key, yarp::os::Bottle& val);
  virtual bool setRemoteVariableRaw(yarp::os::ConstString key, const yarp::os::Bottle& val);
  virtual bool getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys);

  ///////////////////////// END RemoteVariables Interface

  //////////////////////// BEGIN MotorEncoderInterface
  //
  virtual bool getNumberOfMotorEncodersRaw(int* num);
  virtual bool resetMotorEncoderRaw(int m);
  virtual bool resetMotorEncodersRaw();
  virtual bool setMotorEncoderRaw(int m, const double val);
  virtual bool setMotorEncodersRaw(const double *vals);
  virtual bool getMotorEncoderRaw(int m, double *v);
  virtual bool setMotorEncoderCountsPerRevolutionRaw(int m, const double cpr);
  virtual bool getMotorEncoderCountsPerRevolutionRaw(int m, double *cpr);
  virtual bool getMotorEncodersRaw(double *encs);
  virtual bool getMotorEncoderSpeedRaw(int m, double *sp);
  virtual bool getMotorEncoderSpeedsRaw(double *spds);
  virtual bool getMotorEncoderAccelerationRaw(int m, double *spds);
  virtual bool getMotorEncoderAccelerationsRaw(double *accs);
  virtual bool getMotorEncodersTimedRaw(double *encs, double *stamps);
  virtual bool getMotorEncoderTimedRaw(int m, double *encs, double *stamp);

  ///////////////////////// END Encoder Interface
  ////// Amplifier interface
  //
  virtual bool enableAmpRaw(int j);
  virtual bool disableAmpRaw(int j);
  virtual bool getCurrentsRaw(double *vals);
  virtual bool getCurrentRaw(int j, double *val);
  virtual bool setMaxCurrentRaw(int j, double val);
  virtual bool getMaxCurrentRaw(int j, double* val);
  virtual bool getAmpStatusRaw(int *st);
  virtual bool getAmpStatusRaw(int k, int *st);
  virtual bool getPWMRaw(int j, double* val);
  virtual bool getPWMLimitRaw(int j, double* val);
  virtual bool setPWMLimitRaw(int j, const double val);
  virtual bool getPowerSupplyVoltageRaw(int j, double* val);
  //
  /////////////// END AMPLIFIER INTERFACE

  ////// calibration
  virtual bool calibrateRaw(int j, double p);
  virtual bool doneRaw(int j);

  /////// Limits
  virtual bool setLimitsRaw(int axis, double min, double max);
  virtual bool getLimitsRaw(int axis, double *min, double *max);
  virtual bool setVelLimitsRaw(int axis, double min, double max);
  virtual bool getVelLimitsRaw(int axis, double *min, double *max);

  /////// Axis Info
  virtual bool getAxisNameRaw(int axis, yarp::os::ConstString& name);
  virtual bool getJointTypeRaw(int axis, yarp::dev::JointTypeEnum& type);

  /// IMotor
  virtual bool getNumberOfMotorsRaw(int* m);
  virtual bool getTemperatureRaw(int m, double* val);
  virtual bool getTemperaturesRaw(double *vals);
  virtual bool getTemperatureLimitRaw(int m, double *temp);
  virtual bool setTemperatureLimitRaw(int m, const double temp);
  virtual bool getMotorOutputLimitRaw(int m, double *limit);
  virtual bool setMotorOutputLimitRaw(int m, const double limit);
  virtual bool getPeakCurrentRaw(int m, double *val);
  virtual bool setPeakCurrentRaw(int m, const double val);
  virtual bool getNominalCurrentRaw(int m, double *val);
  virtual bool setNominalCurrentRaw(int m, const double val);

   /////// Torque Control
  virtual bool setTorqueModeRaw(void);
  virtual bool getTorqueRaw(int, double *);
  virtual bool getTorquesRaw(double *);
  virtual bool getTorqueRangeRaw(int,double *,double *);
  virtual bool getTorqueRangesRaw(double *,double *);
  virtual bool setRefTorquesRaw(const double *);
  virtual bool setRefTorqueRaw(int,double);
  virtual bool setRefTorquesRaw(const int n_joint, const int *joints, const double *t);
  virtual bool getRefTorquesRaw(double *);
  virtual bool getRefTorqueRaw(int,double *);
  virtual bool getBemfParamRaw(int,double *);
  virtual bool setBemfParamRaw(int,double );
  virtual bool setTorquePidRaw(int,const yarp::dev::Pid &);
  virtual bool setTorquePidsRaw(const yarp::dev::Pid *);
  virtual bool setTorqueErrorLimitRaw(int,double);
  virtual bool setTorqueErrorLimitsRaw(const double *);
  virtual bool getTorqueErrorRaw(int,double *);
  virtual bool getTorqueErrorsRaw(double *);
  virtual bool getTorquePidOutputRaw(int,double *);
  virtual bool getTorquePidOutputsRaw(double *);
  virtual bool getTorquePidRaw(int,yarp::dev::Pid *);
  virtual bool getTorquePidsRaw(yarp::dev::Pid *);
  virtual bool getTorqueErrorLimitRaw(int,double *);
  virtual bool getTorqueErrorLimitsRaw(double *);
  virtual bool resetTorquePidRaw(int);
  virtual bool disableTorquePidRaw(int);
  virtual bool enableTorquePidRaw(int);
  virtual bool setTorqueOffsetRaw(int,double);
  virtual bool getMotorTorqueParamsRaw(int j, MotorTorqueParameters *params);
  virtual bool setMotorTorqueParamsRaw(int j, const MotorTorqueParameters params);

  /////// Control Mode Interface
  virtual bool setPositionModeRaw(int j);
  virtual bool setVelocityModeRaw(int j);
  virtual bool setTorqueModeRaw(int j);
  virtual bool setImpedancePositionModeRaw(int j);
  virtual bool setImpedanceVelocityModeRaw(int j);
  virtual bool getControlModeRaw(int j, int *mode);
  virtual bool getControlModesRaw(int* modes);

  /////// Control Mode2 Interface
  virtual bool getControlModesRaw(const int n_joint, const int *joints, int *modes);
  virtual bool setControlModeRaw(const int j, const int mode);
  virtual bool setControlModesRaw(const int n_joint, const int *joints, int *modes);
  virtual bool setControlModesRaw(int *modes);


  /////// InteractionMode
  virtual bool getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum* mode);
  virtual bool getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
  virtual bool getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);
  virtual bool setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode);
  virtual bool setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
  virtual bool setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);

  /////// PositionDirect
  virtual bool setPositionDirectModeRaw();
  virtual bool setPositionRaw(int j, double ref);
  virtual bool setPositionsRaw(const int n_joint, const int *joints, double *refs);
  virtual bool setPositionsRaw(const double *refs);
  virtual bool getRefPositionRaw(const int joint, double *ref);
  virtual bool getRefPositionsRaw(double *refs);
  virtual bool getRefPositionsRaw(const int n_joint, const int *joints, double *refs);

  /////// PWMControl
  virtual bool setRefDutyCycleRaw(int j, double v);
  virtual bool setRefDutyCyclesRaw(const double *v);
  virtual bool getRefDutyCycleRaw(int j, double *v);
  virtual bool getRefDutyCyclesRaw(double *v);
  virtual bool getDutyCycleRaw(int j, double *v);
  virtual bool getDutyCyclesRaw(double *v);

  /////// CurrentControl
  // virtual bool getAxes(int *ax);
  //virtual bool getCurrentRaw(int j, double *t);
  //virtual bool getCurrentsRaw(double *t);
  virtual bool getCurrentRangeRaw(int j, double *min, double *max);
  virtual bool getCurrentRangesRaw(double *min, double *max);
  virtual bool setRefCurrentsRaw(const double *t);
  virtual bool setRefCurrentRaw(int j, double t);
  virtual bool setRefCurrentsRaw(const int n_joint, const int *joints, const double *t);
  virtual bool getRefCurrentsRaw(double *t);
  virtual bool getRefCurrentRaw(int j, double *t);
  virtual bool setCurrentPidRaw(int j, const Pid &pid);
  virtual bool setCurrentPidsRaw(const Pid *pids);
  virtual bool getCurrentErrorRaw(int j, double *err);
  virtual bool getCurrentErrorsRaw(double *errs);
  virtual bool getCurrentPidOutputRaw(int j, double *out);
  virtual bool getCurrentPidOutputsRaw(double *outs);
  virtual bool getCurrentPidRaw(int j, Pid *pid);
  virtual bool getCurrentPidsRaw(Pid *pids);
  virtual bool resetCurrentPidRaw(int j);
  virtual bool disableCurrentPidRaw(int j);
  virtual bool enableCurrentPidRaw(int j);

//void run(void);

  /////// Joint steps
  void jointStep();

  int verbosity;

private:
    void compute_mot_pos(double *mot, double *jnt);
    void compute_mot_vel(double *mot, double *jnt);

protected:
    yarp::dev::PolyDriver joints;
    LogicalJoints *manager;

    yarp::os::Semaphore _mutex;
    yarp::os::Semaphore _done;
    
    bool _writerequested;
    bool _noreply;
    bool _opened;

    //current position of the joints
    double *current_jnt_pos;
    double *current_mot_pos;

    //torque of the joints
    double *current_jnt_torques; // at the moment this is fake
    double *current_mot_torques; // at the moment this is fake

    //pwm value
    double *pwm; // at the moment this is fake
    double *pwm_ref; // at the moment this is fake

    //motor current
    double *current_ampere; // at the moment this is fake
    double *current_ampere_ref; // at the moment this is fake

    //current velocity of the joints
    double *current_jnt_vel;
    double *current_mot_vel;
    
    //next position of the joints
    double *next_pos;
    double *ref_command_positions;
    double *ref_positions;

    //next velocity of the joints during velocity control
    double *next_vel;
    double *ref_command_speeds;
    double *ref_speeds;

    //next torques of the joints during torque control
    double *next_torques;
    double *ref_torques;
    
    //semaphore access for synch with run function of webots
    int semaphoreNum;

    int partSelec;
    // number of joints/axes/controlled motors
    int njoints;

    // velocity
    double vel;
    
    //rate at which the position are updated im ms
    //int positionUpdateRate;
    static const int positionUpdateRate = 100;

    //axis remapping lookup-table
    int *axisMap;                              
    // angle to encoder conversion factors
    double *angleToEncoder;                    

    double *zeros;                             /** encoder zeros */
    double *newtonsToSensor;
    double *ampsToSensor;
    double *dutycycleToPwm;

    double *error_tol;

    bool *motor_on;
    int *motor;
    double ErrorPos[100];
    int input;
    int *inputs;  /* in fact we need an "input" flag for every joint */
    double *vels; /* in fact we need a velocity for every joint */

    double *limitsMin;                         // joint limits, min
    double *limitsMax;                         // joint limits, max
    double *velLimitsMin;                      // joint vel limits, min
    double *velLimitsMax;                      // joint vel limits, max
    double *torqueLimits;                      // torque limits
    double *maxCurrent;                        // max motor current (simulated)
    double *rotToEncoder;                      // angle to rotor conversion factors
    double *gearbox;                           // the gearbox ratio
    double *refSpeed;
    double *refAccel;
    double *controlP;
    bool   *hasHallSensor;
    bool   *hasTempSensor;
    bool   *hasRotorEncoder;
    int    *rotorIndexOffset;
    int    *motorPoles;

    int    *controlMode;
    int    *interactionMode;

    Pid    *position_pid;
    Pid    *torque_pid;
    Pid    *current_pid;
    MotorTorqueParameters *motor_torque_params;
    yarp::sig::Matrix kinematic_mj;

    //bool velocityMode;
};

#endif


