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
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>

#include "LogicalJoints.h"

namespace yarp{
    namespace dev{
        class iCubSimulationControl;
    }
}
class yarp::dev::iCubSimulationControl :
    public DeviceDriver,
    //public yarp::os::RateThread, 
    public IPidControlRaw, 
    public IPositionControlRaw, 
    public IVelocityControlRaw, 
    public ITorqueControlRaw,
    public IEncodersRaw, 
    public IAmplifierControlRaw,
    public IControlCalibrationRaw,
    public IControlLimitsRaw,
	public ImplementTorqueControl,
    public ImplementPositionControl<iCubSimulationControl, IPositionControl>,
    public ImplementVelocityControl<iCubSimulationControl, IVelocityControl>,
    public ImplementPidControl<iCubSimulationControl, IPidControl>,
    public ImplementEncoders<iCubSimulationControl, IEncoders>,
    public ImplementControlCalibration<iCubSimulationControl, IControlCalibration>,
    public ImplementAmplifierControl<iCubSimulationControl, IAmplifierControl>,
    public ImplementControlLimits<iCubSimulationControl, IControlLimits>

{
 private:
  iCubSimulationControl(const iCubSimulationControl&);
  void operator=(const iCubSimulationControl&);
 
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

  ///////////// PID INTERFACE
  //
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
  virtual bool setOffsetRaw(int j, double v);/**/
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
  virtual bool stopRaw();/**/

  //
  /////////////////////////////// END Position Control INTERFACE

    ///////////// Velocity control interface raw
  ///
  virtual bool setVelocityModeRaw();
  virtual bool velocityMoveRaw(int j, double sp);
  virtual bool velocityMoveRaw(const double *sp);
  //
  /////////////////////////////// END Velocity Control INTERFACE
  
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

  ////// Amplifier interface
  //
  virtual bool enableAmpRaw(int j);
  virtual bool disableAmpRaw(int j);
  virtual bool getCurrentsRaw(double *vals);
  virtual bool getCurrentRaw(int j, double *val);
  virtual bool setMaxCurrentRaw(int j, double val);
  virtual bool getAmpStatusRaw(int *st);
  virtual bool getAmpStatusRaw(int k, int *st);
  //
  /////////////// END AMPLIFIER INTERFACE

  ////// calibration
  virtual bool calibrateRaw(int j, double p);
  virtual bool doneRaw(int j);

  
  /////// Limits
  virtual bool setLimitsRaw(int axis, double min, double max);
  virtual bool getLimitsRaw(int axis, double *min, double *max);

    /////// Torque Control
    virtual bool setTorqueModeRaw(void);
    virtual bool getTorqueRaw(int, double *);
    virtual bool getTorquesRaw(double *);
    virtual bool getTorqueRangeRaw(int,double *,double *);
    virtual bool getTorqueRangesRaw(double *,double *);
    virtual bool setRefTorquesRaw(const double *);
    virtual bool setRefTorqueRaw(int,double);
    virtual bool getRefTorquesRaw(double *);
    virtual bool getRefTorqueRaw(int,double *);
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

//void run(void);

  /////// Joint steps
  void jointStep();

  bool verbosity;

protected:
    yarp::dev::PolyDriver joints;
    LogicalJoints *manager;

    yarp::os::Semaphore _mutex;
    yarp::os::Semaphore _done;
    
    bool _writerequested;
    bool _noreply;
    bool _opened;

    //current position of the joints
    double *current_pos;

    //torque of the joints
    double *current_torques; // at the moment this is fake

    //current velocity of the joints
    double *current_vel;
    
    //next position of the joints
    double *next_pos;

    //next velocity of the joints during velocity control
    double *next_vel;
    
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

    double *error_tol;

    bool *motor_on;
    int *motor;
    double ErrorPos[100];
    int input;
    int *inputs;  /* in fact we need an "input" flag for every joint */
    double *vels; /* in fact we need a velocity for every joint */

    double *limitsMin;                          /** joint limits, min*/
    double *limitsMax;                         /** joint limits, max*/
    double *torqueLimits;                     /** torque limits */

    double *refSpeed;
    double *refAccel;
    double *controlP;

    bool velocityMode;
};

#endif


