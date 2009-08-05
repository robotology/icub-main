// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Ludovic Righetti
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

///
/// iCubWebotsMotionControl.h
///
///

#ifndef __iCubWebotsMotionControlh__
#define __iCubWebotsMotionControlh__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Thread.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/all.h>

//webots specific
#include <device/servo.h>
#include <device/robot.h>

#include "webots_common.h"

namespace yarp{
    namespace dev{
        class iCubWebotsMotionControl;
    }
}

using namespace yarp::os;
using namespace yarp::dev;


//yarp::os::Semaphore *sem_webots,*sem_control;

/**
 * \file iCubWebotsMotionControl.h 
 * class for interfacing with webots
 */



/**
 * @ingroup dev_impl_motor
 *
 * The ESD motion controller device driver.
 * Contains a thread that takes care of polling the can bus for incoming messages.
 */
class yarp::dev::iCubWebotsMotionControl: 
    public DeviceDriver,
            public os::Thread, 
            public IPidControlRaw, 
            public IPositionControlRaw, 
            public IVelocityControlRaw, 
            public IEncodersRaw, 
            public IAmplifierControlRaw,
            public IControlCalibrationRaw,
            public IControlLimitsRaw,
            public ImplementPositionControl<iCubWebotsMotionControl, IPositionControl>,
            public ImplementVelocityControl<iCubWebotsMotionControl, IVelocityControl>,
            public ImplementPidControl<iCubWebotsMotionControl, IPidControl>,
            public ImplementEncoders<iCubWebotsMotionControl, IEncoders>,
            public ImplementControlCalibration<iCubWebotsMotionControl, IControlCalibration>,
            public ImplementAmplifierControl<iCubWebotsMotionControl, IAmplifierControl>,
            public ImplementControlLimits<iCubWebotsMotionControl, IControlLimits>
{
 private:
  iCubWebotsMotionControl(const iCubWebotsMotionControl&);
  void operator=(const iCubWebotsMotionControl&);
  
 public:
  /**
   * Default constructor. Construction is done in two stages, first build the
   * object and then open the device driver.
   */
  iCubWebotsMotionControl();
  
  /**
   * Destructor.
   */
  virtual ~iCubWebotsMotionControl();
  
  
  /**
   * Open the device driver and start communication with the hardware.
   * @param config is a Searchable object containing the list of parameters.
   * @return true on success/failure.
   */
  virtual bool open(yarp::os::Searchable& config);
  
  /**
   * Closes the device driver.
   * @return true on success.
   */
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
  virtual bool setOffsetRaw(int j, double v);
  //
  /////////////////////////////// END PID INTERFACE
  
  //
  /// POSITION CONTROL INTERFACE RAW
  virtual bool getAxes(int *ax);
  virtual bool setPositionMode();
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
  //
  /////////////////////////////// END Position Control INTERFACE
  
  ///////////// Velocity control interface raw
  ///
  virtual bool setVelocityMode();
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
  //
  ///////////////////////// END Encoder Interface

  ////// Amplifier interface
  //
  virtual bool enableAmpRaw(int j);
  virtual bool disableAmpRaw(int j);
  virtual bool getCurrentsRaw(double *vals);
  virtual bool getCurrentRaw(int j, double *val);
  virtual bool setMaxCurrentRaw(int j, double val);
  virtual bool getAmpStatusRaw(int *st);
  //
  /////////////// END AMPLIFIER INTERFACE

  ////// calibration
  virtual bool calibrateRaw(int j, double p);
  virtual bool doneRaw(int j);

  
  /////// Limits
  virtual bool setLimitsRaw(int axis, double min, double max);
  virtual bool getLimitsRaw(int axis, double *min, double *max);

////// For the webots run
void run(void);

protected:
    yarp::os::Semaphore _mutex;
    yarp::os::Semaphore _done;
    
    bool _writerequested;
    bool _noreply;
    bool _opened;
	
   
    //current position of the joints
    double *current_pos;
    
    //next position of the joints
    double *next_pos;
    
    //semaphore access for synch with run function of webots
    int semaphoreNum;

    //webotscommon instance pointer
    WebotsCommon *webots_instance;

    //access to the joints
    DeviceTag *joint_dev;

    // number of joints/axes/controlled motors
    int njoints;								
    
    //rate at which the position are updated im ms
    int positionUpdateRate;

    //axis remapping lookup-table
    int *axisMap;                              
    // angle to encoder conversion factors
    double *angleToEncoder;                    

    double *zeros;                             /** encoder zeros */

    double *error_tol;

    bool *motor_on;

    double *limitsMin;                          /** joint limits, min*/
    double *limitsMax;                         /** joint limits, max*/
    double *torqueLimits;                     /** torque limits */

    double *refSpeed;
    double *refAccel;
    double *controlP;

    yarp::os::ConstString *jointNames; /*name of the joints to access the webots devices*/
};


#endif
