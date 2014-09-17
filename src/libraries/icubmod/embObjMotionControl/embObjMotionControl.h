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
 * \defgroup eth2ems eth2ems
 *
 * Implements <a href="http://wiki.icub.org/yarpdoc/d3/d5b/classyarp_1_1dev_1_1ICanBus.html" ICanBus interface <\a> for a ems to can bus device.
 * This is the eth2ems module device.
 *
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 *
 * Author: Alberto Cardellino
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/modules/....h
 *
 */

//
// $Id: embObjMotionControl.h,v 1.5 2008/06/25 22:33:53 nat Exp $
//

#ifndef __embObjMotionControlh__
#define __embObjMotionControlh__

//#undef __cplucplus

using namespace std;

#define EMBMC_SIZE_INFO     128

/////  Yarp stuff
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/dev/ControlBoardInterfacesImpl.inl>

#include <yarp/dev/IVirtualAnalogSensor.h>
///////////////////

#include <iCub/FactoryInterface.h>


// debug interface
#include <iCub/DebugInterfaces.h>

// ACE udp socket
#include <ace/ACE.h>
#include <ace/SOCK_Dgram_Bcast.h>

#include "FeatureInterface.h"

#include "EoMotionControl.h"
// #include <ethManager.h>
#include <ethResource.h>
#include "../embObjLib/hostTransceiver.hpp"
// #include "IRobotInterface.h"

#include "eoRequestsQueue.hpp"
#include "EoMotionControl.h"
//
//   Help structure
//
using namespace yarp::os;

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

    double get_min_stiff()
    {
        return min_stiff;
    }
    double get_max_stiff()
    {
        return max_stiff;
    }
    double get_min_damp()
    {
        return min_damp;
    }
    double get_max_damp()
    {
        return max_damp;
    }
};

struct ImpedanceParameters
{
    double stiffness;
    double damping;
    bool   enabled;
    //ImpedanceLimits limits;
    ImpedanceParameters()
    {
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

#define IMPLEMENT_DEBUG_INTERFACE

#ifdef _SETPOINT_TEST_
typedef struct
{
    yarp::os::Semaphore     mutex;
    uint64_t                send_time;
    eOmeas_position_t       last_pos;
    eOmeas_position_t       pos;
    bool                    gotIt;
    int                     count_old;
    bool                    wtf;
} debug_data_of_joint_t;
#endif

namespace yarp {
    namespace dev  {
    class embObjMotionControl;
    }
}
using namespace yarp::dev;

enum { MAX_SHORT = 32767, MIN_SHORT = -32768, MAX_INT = 0x7fffffff, MIN_INT = 0x80000000,  MAX_U32 = 0xffffffff, MIN_U32 = 0x00, MAX_U16 = 0xffff, MIN_U16 = 0x0000};
enum { CAN_SKIP_ADDR = 0x80 };

class yarp::dev::embObjMotionControl:   public DeviceDriver,
    public IPidControlRaw,
    public IControlCalibration2Raw,
    public IAmplifierControlRaw,
    public IEncodersTimedRaw,
    public ImplementEncodersTimed,
    public IPositionControl2Raw,
    public IVelocityControl2Raw,
    public IControlMode2Raw,
    public ImplementControlMode2,
    public IControlLimits2Raw,
    public IImpedanceControlRaw,
    public ImplementImpedanceControl,
    public ImplementControlLimits2,
    public ImplementAmplifierControl<embObjMotionControl, IAmplifierControl>,
    public ImplementPositionControl2,
    public ImplementControlCalibration2<embObjMotionControl, IControlCalibration2>,
    public ImplementPidControl<embObjMotionControl, IPidControl>,
    public ImplementVelocityControl<embObjMotionControl, IVelocityControl>,
    public ImplementVelocityControl2,
    public ITorqueControlRaw,
    public ImplementTorqueControl,
    public IVirtualAnalogSensor,
    public IPositionDirectRaw,
    public ImplementPositionDirect,
    public IInteractionModeRaw,
    public ImplementInteractionMode,
    public IOpenLoopControlRaw,
    public ImplementOpenLoopControl,
    public IDebugInterfaceRaw,
    public ImplementDebugInterface
{
private:
    int           tot_packet_recv, errors;

    bool initted;
    // embObj stuff
    yarp::os::Semaphore     _mutex;
    FEAT_ID                 _fId;

    int *_axisMap;                              /** axis remapping lookup-table */
    double *_angleToEncoder;                    /** angle to iCubDegrees conversion factors */
    double  *_encodersStamp;                    /** keep information about acquisition time for encoders read */
    float *_encoderconversionfactor;            /** iCubDegrees to encoder conversion factors */
    float *_encoderconversionoffset;            /** iCubDegrees offset */
    double *_rotToEncoder;                      /** angle to rotor conversion factors */
    double *_zeros;                             /** encoder zeros */
    Pid *_pids;                                 /** initial gains */
    Pid *_tpids;                                /** initial torque gains */
    bool _tpidsEnabled;                         /** abilitation for torque gains */
    SpeedEstimationParameters *_estim_params;   /** parameters for speed/acceleration estimation */
    //  DebugParameters *_debug_params;             /** debug parameters */

    ImpedanceLimits     *_impedance_limits;     /** impedancel imits */
    double *_limitsMin;                         /** joint limits, max*/
    double *_limitsMax;                         /** joint limits, min*/
    double *_currentLimits;                     /** current limits */
    int *_velocityShifts;                       /** velocity shifts */
    int *_velocityTimeout;                      /** velocity shifts */
    int *_torqueSensorId;                       /** Id of associated Joint Torque Sensor */
    int *_torqueSensorChan;                     /** Channel of associated Joint Torque Sensor */
    double *_maxTorque;                         /** Max torque of a joint */
    double *_newtonsToSensor;                   /** Newtons to force sensor units conversion factors */
    bool  *checking_motiondone;                 /* flag telling if I'm already waiting for motion done */

    // TODO doubled!!! optimize using just one of the 2!!!
    ImpedanceParameters *_impedance_params;     /** impedance parameters */
    eOmc_impedance_t *_cacheImpedance;			/* cache impedance value to split up the 2 sets */

    bool         useRawEncoderData;
    bool        _pwmIsLimited;                         /** set to true if pwm is limited */

    //debug purpose
    
#undef  VERIFY_ROP_SETIMPEDANCE   //this macro let you send setimpedence rop with signature.								  //if you want use this feature, you should compiled ems firmware with same macro.
#undef VERIFY_ROP_SETPOSITIONRAW  //this macro let you send setposition rop with signature.
								  //if you want use this feature, yuo should compiled ems firmware with same macro.
    
#ifdef VERIFY_ROP_SETIMPEDANCE 
    uint32_t *impedanceSignature;
#endif

#ifdef VERIFY_ROP_SETPOSITIONRAW
    uint32_t *refRawSignature;
    bool        *sendingDirects;
#endif
    
    
    // basic knowledge of my joints
    int   _njoints;                             // Number of joints handled by this EMS; this values will be extracted by the config file

    double 		SAFETY_THRESHOLD;
    // debug
    int     start;
    int     end;

    // internal stuff
    bool    *_enabledAmp;       // Middle step toward a full enabled motor controller. Amp (pwm) plus Pid enable command must be sent in order to get the joint into an active state.
    bool    *_enabledPid;       // Depends on enabledAmp. When both are set, the joint exits the idle mode and goes into position mode. If one of them is disabled, it falls to idle.
    bool    *_calibrated;       // Flag to know if the calibrate function has been called for the joint
    double  *_ref_positions;    // used for position control.
    double  *_ref_speeds;       // used for position control.
    double  *_command_speeds;   // used for velocity control.
    double  *_ref_accs;         // for velocity control, in position min jerk eq is used.
    double  *_ref_torques;      // for torque control.

    uint16_t        NVnumber;       // keep if useful to store, otherwise can be removed. It is used to pass the total number of this EP to the requestqueue
private:
    bool extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size);
    bool configure_mais(yarp::os::Searchable &config);
    bool dealloc();
    bool isEpManagedByBoard();
    bool parsePosPidsGroup_OldFormat(Bottle& pidsGroup, Pid myPid[]);
    bool parseTrqPidsGroup_OldFormat(Bottle& pidsGroup, Pid myPid[]);
    bool parsePidsGroup_NewFormat(Bottle& pidsGroup, Pid myPid[]);
    bool getStatusBasic_withWait(const int n_joint, const int *joints, eOenum08_t *_modes);             // helper function
    bool getInteractionMode_withWait(const int n_joint, const int *joints, eOenum08_t *_modes);     // helper function
    bool interactionModeStatusConvert_embObj2yarp(eOenum08_t embObjMode, int &vocabOut);
    bool interactionModeCommandConvert_yarp2embObj(int vocabMode, eOenum08_t &embOut);

    bool controlModeCommandConvert_yarp2embObj(int vocabMode, eOenum08_t &embOut);
    int  controlModeCommandConvert_embObj2yarp(eOmc_controlmode_command_t embObjMode);

    bool controlModeStatusConvert_yarp2embObj(int vocabMode, eOmc_controlmode_t &embOut);
    int  controlModeStatusConvert_embObj2yarp(eOenum08_t embObjMode);

    void copyPid_iCub2eo(const Pid *in, eOmc_PID_t *out);
    void copyPid_eo2iCub(eOmc_PID_t *in, Pid *out);


    // saturation check and rounding for 16 bit unsigned integer
    int U_16(double x) const
    {
        if (x <= double(MIN_U16) )
            return MIN_U16;
        else
            if (x >= double(MAX_U16))
                return MAX_U16;
        else
            return int(x + .5);
    }

    // saturation check and rounding for 16 bit signed integer
    short S_16(double x)
    {
        if (x <= double(-(MAX_SHORT))-1)
            return MIN_SHORT;
        else
            if (x >= double(MAX_SHORT))
                return MAX_SHORT;
        else
            if  (x>0)
                return short(x + .5);
            else
                return short(x - .5);
    }

    // saturation check and rounding for 32 bit unsigned integer
    int U_32(double x) const
    {
        if (x <= double(MIN_U32) )
            return MIN_U32;
        else
            if (x >= double(MAX_U32))
                return MAX_U32;
        else
            return int(x + .5);
    }

    // saturation check and rounding for 32 bit signed integer
    int S_32(double x) const
    {
        if (x <= double(-(MAX_INT))-1.0)
            return MIN_INT;
        else
            if (x >= double(MAX_INT))
                return MAX_INT;
        else
            if  (x>0)
                return int(x + .5);
            else
                return int(x - .5);
    }


public:
    embObjMotionControl();
    ~embObjMotionControl();

    char                info[EMBMC_SIZE_INFO];
    Semaphore           semaphore;
    eoRequestsQueue     *requestQueue;  // tabella che contiene la lista delle attese

    // embObj stuff -- this is better if private... do a get method!!
    ethResources                *res;
    yarp::dev::TheEthManager    *ethManager;

    /*Device Driver*/
    virtual bool open(yarp::os::Searchable &par);
    virtual bool close();
    bool fromConfig(yarp::os::Searchable &config);

    eoThreadEntry *appendWaitRequest(int j, uint32_t protoid);
    void refreshEncoderTimeStamp(int joint);

    bool alloc(int njoints);
    bool init(void);

    /////////   PID INTERFACE   /////////
    virtual bool setPidRaw(int j, const Pid &pid);
    virtual bool setPidsRaw(const Pid *pids);
    virtual bool setReferenceRaw(int j, double ref);
    virtual bool setReferencesRaw(const double *refs);
    virtual bool setErrorLimitRaw(int j, double limit);
    virtual bool setErrorLimitsRaw(const double *limits);
    virtual bool getErrorRaw(int j, double *err);
    virtual bool getErrorsRaw(double *errs);
//    virtual bool getOutputRaw(int j, double *out);    // uses iOpenLoop interface
//    virtual bool getOutputsRaw(double *outs);         // uses iOpenLoop interface
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

    // POSITION CONTROL INTERFACE RAW
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

    ////// Amplifier interface
    virtual bool enableAmpRaw(int j);
    virtual bool disableAmpRaw(int j);
    virtual bool getCurrentsRaw(double *vals);
    virtual bool getCurrentRaw(int j, double *val);
    virtual bool setMaxCurrentRaw(int j, double val);
    virtual bool getAmpStatusRaw(int *st);
    virtual bool getAmpStatusRaw(int j, int *st);
    /////////////// END AMPLIFIER INTERFACE
    FEAT_ID getFeat_id();

    // virtual analog sensor
    virtual int getState(int ch);
    virtual int getChannels();
    virtual bool updateMeasure(yarp::sig::Vector &fTorques);
    virtual bool updateMeasure(int j, double &fTorque);

#ifdef IMPLEMENT_DEBUG_INTERFACE
    //----------------------------------------------\\
    //  Debug interface
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
    bool getParameterRaw(int j, unsigned int type, double *value);

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
    bool getDebugParameterRaw(int j, unsigned int index, double *value);
    bool getDebugReferencePositionRaw(int j, double *value);
    bool getRotorPositionRaw(int j, double *value);
    bool getRotorPositionsRaw(double *value);
    bool getRotorSpeedRaw(int j, double *value);
    bool getRotorSpeedsRaw(double *value);
    bool getRotorAccelerationRaw(int j, double *value);
    bool getRotorAccelerationsRaw(double *value);
    bool getJointPositionRaw(int j, double *value);
    bool getJointPositionsRaw(double *value);
#endif

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
    int32_t getRefSpeedInTbl(uint8_t boardNum, int j, eOmeas_position_t pos);

    // IVelocityControl2
    bool velocityMoveRaw(const int n_joint, const int *joints, const double *spds);
    bool setVelPidRaw(int j, const Pid &pid);
    bool setVelPidsRaw(const Pid *pids);
    bool getVelPidRaw(int j, Pid *pid);
    bool getVelPidsRaw(Pid *pids);

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
    // helper function for reading/writing impedance parameters
    bool getWholeImpedanceRaw(int j, eOmc_impedance_t &imped);

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

    // OPENLOOP interface
    bool setRefOutputRaw(int j, double v);
    bool setRefOutputsRaw(const double *v);
    bool getRefOutputRaw(int j, double *out);
    bool getRefOutputsRaw(double *outs);
    bool getOutputRaw(int j, double *out);
    bool getOutputsRaw(double *outs);
    bool setOpenLoopModeRaw();
};

#endif // include guard

