// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \file iCubSimulationControl.h
 * \brief This is the header file for the yarp interface of the iCubSimulation.
 * \author Vadim Tikhanoff, Paul Fitzpatrick, Giorgio Metta
 * \date 2007
 * \note Release under GNU GPL v2.0
 **/
#ifndef __iCubSimulationControlh__
#define __iCubSimulationControlh__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Thread.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/all.h>

namespace yarp{
    namespace dev{
        class iCubSimulationControl;
    }
}
using namespace yarp::os;
using namespace yarp::dev;

class yarp::dev::iCubSimulationControl :
    public DeviceDriver,
	public os::Thread, 
           public IPidControlRaw, 
            public IPositionControlRaw, 
            public IVelocityControlRaw, 
            public IEncodersRaw, 
            public IAmplifierControlRaw,
            public IControlCalibrationRaw,
            public IControlLimitsRaw,
            public ImplementPositionControl<iCubSimulationControl, IPositionControl>,
            public ImplementVelocityControl<iCubSimulationControl, IVelocityControl>,
            public ImplementPidControl<iCubSimulationControl, IPidControl>,
            public ImplementEncoders<iCubSimulationControl, IEncoders>,
            public ImplementControlCalibration<iCubSimulationControl, IControlCalibration>,
            public ImplementAmplifierControl<iCubSimulationControl, IAmplifierControl>,
            public ImplementControlLimits<iCubSimulationControl, IControlLimits>
			/* */
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
  virtual bool stopRaw();/**/
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

  void run(void);

protected:
    yarp::os::Semaphore _mutex;
    yarp::os::Semaphore _done;
    
    bool _writerequested;
    bool _noreply;
    bool _opened;
	
	//current position of the joints
    double *current_pos;

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
    int positionUpdateRate;

    //axis remapping lookup-table
    int *axisMap;                              
    // angle to encoder conversion factors
    double *angleToEncoder;                    

    double *zeros;                             /** encoder zeros */

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


