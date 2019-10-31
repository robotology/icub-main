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

#include <mutex>
#include <string>

#include <yarp/os/Bottle.h>
//#include <yarp/os/PeriodicThread.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Matrix.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

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
    //public yarp::os::PeriodicThread, 
    public IPositionControlRaw,
    public ImplementPositionControl,
    public IVelocityControlRaw,
    public ImplementVelocityControl,
    public ITorqueControlRaw,
    public ImplementTorqueControl,
    public IAmplifierControlRaw,
    public ImplementAmplifierControl,
    public IControlCalibrationRaw,
    public ImplementControlCalibration,
    public IControlLimitsRaw,
    public ImplementControlLimits,
    public IControlModeRaw,
    public ImplementControlMode,
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
  virtual bool setPidRaw(const PidControlTypeEnum& pidtype, int j, const Pid &pid) override;
  virtual bool setPidsRaw(const PidControlTypeEnum& pidtype, const Pid *pids) override;
  virtual bool setPidReferenceRaw(const PidControlTypeEnum& pidtype, int j, double ref) override;
  virtual bool setPidReferencesRaw(const PidControlTypeEnum& pidtype, const double *refs) override;
  virtual bool setPidErrorLimitRaw(const PidControlTypeEnum& pidtype, int j, double limit) override;
  virtual bool setPidErrorLimitsRaw(const PidControlTypeEnum& pidtype, const double *limits) override;
  virtual bool getPidErrorRaw(const PidControlTypeEnum& pidtype, int j, double *err) override;
  virtual bool getPidErrorsRaw(const PidControlTypeEnum& pidtype, double *errs) override;
  virtual bool getPidOutputRaw(const PidControlTypeEnum& pidtype, int j, double *out) override;
  virtual bool getPidOutputsRaw(const PidControlTypeEnum& pidtype, double *outs) override;
  virtual bool getPidRaw(const PidControlTypeEnum& pidtype, int j, Pid *pid) override;
  virtual bool getPidsRaw(const PidControlTypeEnum& pidtype, Pid *pids) override;
  virtual bool getPidReferenceRaw(const PidControlTypeEnum& pidtype, int j, double *ref) override;
  virtual bool getPidReferencesRaw(const PidControlTypeEnum& pidtype, double *refs) override;
  virtual bool getPidErrorLimitRaw(const PidControlTypeEnum& pidtype, int j, double *limit) override;
  virtual bool getPidErrorLimitsRaw(const PidControlTypeEnum& pidtype, double *limits) override;
  virtual bool resetPidRaw(const PidControlTypeEnum& pidtype, int j) override;
  virtual bool disablePidRaw(const PidControlTypeEnum& pidtype, int j) override;
  virtual bool enablePidRaw(const PidControlTypeEnum& pidtype, int j) override;
  virtual bool setPidOffsetRaw(const PidControlTypeEnum& pidtype, int j, double v) override;
  virtual bool isPidEnabledRaw(const PidControlTypeEnum& pidtype, int j, bool* enabled) override;
  //
  /////////////////////////////// END PID INTERFACE

  /// POSITION CONTROL INTERFACE RAW
  virtual bool getAxes(int *ax) override;
  virtual bool positionMoveRaw(int j, double ref) override;
  virtual bool positionMoveRaw(const double *refs) override;
  virtual bool relativeMoveRaw(int j, double delta) override;
  virtual bool relativeMoveRaw(const double *deltas) override;
  virtual bool checkMotionDoneRaw(bool *flag) override;
  virtual bool checkMotionDoneRaw(int j, bool *flag) override;
  virtual bool setRefSpeedRaw(int j, double sp) override;
  virtual bool setRefSpeedsRaw(const double *spds) override;
  virtual bool setRefAccelerationRaw(int j, double acc) override;
  virtual bool setRefAccelerationsRaw(const double *accs) override;
  virtual bool getRefSpeedRaw(int j, double *ref) override;
  virtual bool getRefSpeedsRaw(double *spds) override;
  virtual bool getRefAccelerationRaw(int j, double *acc) override;
  virtual bool getRefAccelerationsRaw(double *accs) override;
  virtual bool stopRaw(int j) override;
  virtual bool stopRaw() override;

  // Position Control2 Interface
  virtual bool positionMoveRaw(const int n_joint, const int *joints, const double *refs) override;
  virtual bool relativeMoveRaw(const int n_joint, const int *joints, const double *deltas) override;
  virtual bool checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags) override;
  virtual bool setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds) override;
  virtual bool setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs) override;
  virtual bool getRefSpeedsRaw(const int n_joint, const int *joints, double *spds) override;
  virtual bool getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs) override;
  virtual bool stopRaw(const int n_joint, const int *joints) override;
  virtual bool getTargetPositionRaw(const int joint, double *ref) override;
  virtual bool getTargetPositionsRaw(double *refs) override;
  virtual bool getTargetPositionsRaw(const int n_joint, const int *joints, double *refs) override;

  // Velocity Control
  virtual bool velocityMoveRaw(int j, double sp) override;
  virtual bool velocityMoveRaw(const double *sp) override;
  
  // Velocity Control2 Interface
  virtual bool velocityMoveRaw(const int n_joint, const int *joints, const double *spds) override;
  virtual bool getRefVelocityRaw(const int joint, double *ref) override;
  virtual bool getRefVelocitiesRaw(double *refs) override;
  virtual bool getRefVelocitiesRaw(const int n_joint, const int *joints, double *refs) override;


  //////////////////////// BEGIN Encoder Interface
  //
  virtual bool resetEncoderRaw(int j) override;
  virtual bool resetEncodersRaw() override;
  virtual bool setEncoderRaw(int j, double val) override;
  virtual bool setEncodersRaw(const double *vals) override;
  virtual bool getEncoderRaw(int j, double *v) override;
  virtual bool getEncodersRaw(double *encs) override;
  virtual bool getEncoderSpeedRaw(int j, double *sp) override;
  virtual bool getEncoderSpeedsRaw(double *spds) override;
  virtual bool getEncoderAccelerationRaw(int j, double *spds) override;
  virtual bool getEncoderAccelerationsRaw(double *accs) override;

  ///////////////////////// END Encoder Interface

  //////////////////////// BEGIN RemoteVariables Interface
  //
  virtual bool getRemoteVariableRaw(std::string key, yarp::os::Bottle& val) override;
  virtual bool setRemoteVariableRaw(std::string key, const yarp::os::Bottle& val) override;
  virtual bool getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys) override;

  ///////////////////////// END RemoteVariables Interface

  //////////////////////// BEGIN MotorEncoderInterface
  //
  virtual bool getNumberOfMotorEncodersRaw(int* num) override;
  virtual bool resetMotorEncoderRaw(int m) override;
  virtual bool resetMotorEncodersRaw() override;
  virtual bool setMotorEncoderRaw(int m, const double val) override;
  virtual bool setMotorEncodersRaw(const double *vals) override;
  virtual bool getMotorEncoderRaw(int m, double *v) override;
  virtual bool setMotorEncoderCountsPerRevolutionRaw(int m, const double cpr) override;
  virtual bool getMotorEncoderCountsPerRevolutionRaw(int m, double *cpr) override;
  virtual bool getMotorEncodersRaw(double *encs) override;
  virtual bool getMotorEncoderSpeedRaw(int m, double *sp) override;
  virtual bool getMotorEncoderSpeedsRaw(double *spds) override;
  virtual bool getMotorEncoderAccelerationRaw(int m, double *spds) override;
  virtual bool getMotorEncoderAccelerationsRaw(double *accs) override;
  virtual bool getMotorEncodersTimedRaw(double *encs, double *stamps) override;
  virtual bool getMotorEncoderTimedRaw(int m, double *encs, double *stamp) override;

  ///////////////////////// END Encoder Interface
  ////// Amplifier interface
  //
  virtual bool enableAmpRaw(int j) override;
  virtual bool disableAmpRaw(int j) override;
  virtual bool getCurrentsRaw(double *vals) override;
  virtual bool getCurrentRaw(int j, double *val) override;
  virtual bool setMaxCurrentRaw(int j, double val) override;
  virtual bool getMaxCurrentRaw(int j, double* val) override;
  virtual bool getAmpStatusRaw(int *st) override;
  virtual bool getAmpStatusRaw(int k, int *st) override;
  virtual bool getPWMRaw(int j, double* val) override;
  virtual bool getPWMLimitRaw(int j, double* val) override;
  virtual bool setPWMLimitRaw(int j, const double val) override;
  virtual bool getPowerSupplyVoltageRaw(int j, double* val) override;
  //
  /////////////// END AMPLIFIER INTERFACE

  ////// calibration
   virtual bool calibrateAxisWithParamsRaw(int axis, unsigned int type, double p1, double p2, double p3) override;
  virtual bool calibrationDoneRaw(int j) override;

  /////// Limits
  virtual bool setLimitsRaw(int axis, double min, double max) override;
  virtual bool getLimitsRaw(int axis, double *min, double *max) override;
  virtual bool setVelLimitsRaw(int axis, double min, double max) override;
  virtual bool getVelLimitsRaw(int axis, double *min, double *max) override;

  /////// Axis Info
  virtual bool getAxisNameRaw(int axis, std::string& name) override;
  virtual bool getJointTypeRaw(int axis, yarp::dev::JointTypeEnum& type) override;

  /// IMotor
  virtual bool getNumberOfMotorsRaw(int* m) override;
  virtual bool getTemperatureRaw(int m, double* val) override;
  virtual bool getTemperaturesRaw(double *vals) override;
  virtual bool getTemperatureLimitRaw(int m, double *temp) override;
  virtual bool setTemperatureLimitRaw(int m, const double temp) override;
  virtual bool getPeakCurrentRaw(int m, double *val) override;
  virtual bool setPeakCurrentRaw(int m, const double val) override;
  virtual bool getNominalCurrentRaw(int m, double *val) override;
  virtual bool setNominalCurrentRaw(int m, const double val) override;

   /////// Torque Control
  virtual bool getTorqueRaw(int, double *) override;
  virtual bool getTorquesRaw(double *) override;
  virtual bool getTorqueRangeRaw(int,double *,double *) override;
  virtual bool getTorqueRangesRaw(double *,double *) override;
  virtual bool setRefTorquesRaw(const double *) override;
  virtual bool setRefTorqueRaw(int,double) override;
  virtual bool setRefTorquesRaw(const int n_joint, const int *joints, const double *t) override;
  virtual bool getRefTorquesRaw(double *) override;
  virtual bool getRefTorqueRaw(int,double *) override;
  virtual bool getMotorTorqueParamsRaw(int j, MotorTorqueParameters *params) override;
  virtual bool setMotorTorqueParamsRaw(int j, const MotorTorqueParameters params) override;

  /////// Control Mode Interface
  virtual bool getControlModeRaw(int j, int *mode) override;
  virtual bool getControlModesRaw(int* modes) override;

  /////// Control Mode2 Interface
  virtual bool getControlModesRaw(const int n_joint, const int *joints, int *modes) override;
  virtual bool setControlModeRaw(const int j, const int mode) override;
  virtual bool setControlModesRaw(const int n_joint, const int *joints, int *modes) override;
  virtual bool setControlModesRaw(int *modes) override;


  /////// InteractionMode
  virtual bool getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum* mode) override;
  virtual bool getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes) override;
  virtual bool getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes) override;
  virtual bool setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode) override;
  virtual bool setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes) override;
  virtual bool setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes) override;

  /////// PositionDirect
  virtual bool setPositionRaw(int j, double ref) override;
  virtual bool setPositionsRaw(const int n_joint, const int *joints, const double *refs) override;
  virtual bool setPositionsRaw(const double *refs) override;
  virtual bool getRefPositionRaw(const int joint, double *ref) override;
  virtual bool getRefPositionsRaw(double *refs) override;
  virtual bool getRefPositionsRaw(const int n_joint, const int *joints, double *refs) override;

  /////// PWMControl
  virtual bool setRefDutyCycleRaw(int j, double v) override;
  virtual bool setRefDutyCyclesRaw(const double *v) override;
  virtual bool getRefDutyCycleRaw(int j, double *v) override;
  virtual bool getRefDutyCyclesRaw(double *v) override;
  virtual bool getDutyCycleRaw(int j, double *v) override;
  virtual bool getDutyCyclesRaw(double *v) override;

  /////// CurrentControl
  // virtual bool getAxes(int *ax) override;
  //virtual bool getCurrentRaw(int j, double *t) override;
  //virtual bool getCurrentsRaw(double *t) override;
  virtual bool getCurrentRangeRaw(int j, double *min, double *max) override;
  virtual bool getCurrentRangesRaw(double *min, double *max) override;
  virtual bool setRefCurrentsRaw(const double *t) override;
  virtual bool setRefCurrentRaw(int j, double t) override;
  virtual bool setRefCurrentsRaw(const int n_joint, const int *joints, const double *t) override;
  virtual bool getRefCurrentsRaw(double *t) override;
  virtual bool getRefCurrentRaw(int j, double *t) override;

//void run(void);

  /////// Joint steps
  void jointStep();

  int verbosity;

private:
    void compute_mot_pos_from_jnt_pos(double *mot_pos, const double *jnt_pos, int size_joints);
    void compute_mot_vel_and_acc     (double *mot_vel, double *mot_acc, const double *mot_pos, int size_joints);
    void compute_jnt_vel_and_acc     (double *jnt_vel, double *jnt_acc, const double *jnt_pos, int size_joints);

protected:
    yarp::dev::PolyDriver joints;
    LogicalJoints *manager;

    std::mutex _mutex;
    
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
    double *estimated_jnt_vel;
    double *estimated_mot_vel;
    
    //current acceleration of the joints
    double *current_jnt_acc;
    double *current_mot_acc;
    double *estimated_jnt_acc;
    double *estimated_mot_acc;
    iCub::ctrl::AWLinEstimator       *linEstJnt;
    iCub::ctrl::AWQuadEstimator      *quadEstJnt;
    iCub::ctrl::AWLinEstimator       *linEstMot;
    iCub::ctrl::AWQuadEstimator      *quadEstMot;

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
    Pid    *velocity_pid;
    MotorTorqueParameters *motor_torque_params;
    yarp::sig::Matrix kinematic_mj;

    //bool velocityMode;
};

#endif


