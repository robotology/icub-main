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


using namespace std;


//  Yarp stuff
#include <yarp/os/Bottle.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Semaphore.h>


#include <yarp/dev/IVirtualAnalogSensor.h>




#include "IethResource.h"
#include <ethManager.h>
#include <abstractEthResource.h>

#include "serviceParser.h"
#include "eomcParser.h"
#include "measuresConverter.h"

// - public #define  --------------------------------------------------------------------------------------------------

#undef  VERIFY_ROP_SETIMPEDANCE     // this macro let you send setimpedence rop with signature.
                                    // if you want use this feature, you should compiled ems firmware with same macro.
#undef  VERIFY_ROP_SETPOSITIONRAW   // this macro let you send setposition rop with signature.
                                    // if you want use this feature, yuo should compiled ems firmware with same macro.

// marco.accame:    we dont manage mais anymore from the embObjMotionControl class.
//                  the mais is now initted by the ems board with default params (datarate = 10, mode = eoas_maismode_txdatacontinuously)
//                  and never swicthed off.
//                  only embObjAnalog can override its behaviour

#define     EMBOBJMC_DONT_USE_MAIS



//
//   helper structures
//
namespace yarp {
    namespace dev {
        namespace eomc {
            
typedef struct
{
    vector<int>                         joint2set;
    vector <vector <int> >              set2joint;
    int                                 numofjointsets;
    vector<eOmc_jointset_configuration_t> jointset_cfgs;
} jointsetsInfo_t;

typedef struct
{
    eOmc_encoder_t  type;                 /** joint encoder type*/
    double          tolerance;              /** Num of error bits passable for joint encoder */
    int             resolution;
} encoder_t;

typedef struct
{
    bool verbosewhenok;         /** its value depends on environment variable "ETH_VERBOSEWHENOK" */
    bool useRawEncoderData;     /** if true than do not use calibration data */
    bool pwmIsLimited;          /** set to true if pwm is limited */
}behaviour_flags_t;

}}};

namespace yarp {
    namespace dev  {
    class embObjMotionControl;
    }
}



using namespace yarp::dev;



class yarp::dev::embObjMotionControl:   public DeviceDriver,
    public IPidControlRaw,
    public IControlCalibration2Raw,
    public IAmplifierControlRaw,
    public IEncodersTimedRaw,
    public IMotorEncodersRaw,
    public ImplementEncodersTimed,
    public ImplementMotorEncoders,
    public IMotorRaw,
    public ImplementMotor,
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
    public ImplementPidControl,
    public ImplementVelocityControl<embObjMotionControl, IVelocityControl>,
    public ImplementVelocityControl2,
    public ITorqueControlRaw,
    public ImplementTorqueControl,
    public IVirtualAnalogSensor,
    public IPositionDirectRaw,
    public ImplementPositionDirect,
    public IInteractionModeRaw,
    public ImplementInteractionMode,
    public IRemoteVariablesRaw,
    public ImplementRemoteVariables,
    public IAxisInfoRaw,
    public ImplementAxisInfo,
    public IPWMControlRaw,
    public ImplementPWMControl,
    public ICurrentControlRaw,
    public ImplementCurrentControl,
    public eth::IethResource
{


private:

    eth::TheEthManager*        ethManager;
    eth::AbstractEthResource*  res;
    ServiceParser*             parser;
    eomc::Parser *             _mcparser;
    measuresConverter*         _measureConverter;
    yarp::os::Semaphore        _mutex;
    
    bool opened; //internal state


     /////configuartion info (read from xml files)
    int                                     _njoints;       /** Number of joints handled by this EMS */
    eomc::behaviour_flags_t                  behFlags;       /** Contains all flags that define the behaviour of this device */
    servConfigMC_t                          serviceConfig;  /** contains the needed data for configure motion control service, like i.e. board ports where joint are connected */ 
    double *                                _gearbox_M2J;   /** the gearbox ratio motor to joint */
    double *                                _gearbox_E2J;   /** the gearbox ratio encoder to joint */
    double *                                _deadzone;

    eomc::twofocSpecificInfo_t *            _twofocinfo;

    std::vector<eomc::encoder_t>            _jointEncs;
    std::vector<eomc::encoder_t>            _motorEncs;

    std::vector<eomc::rotorLimits_t>        _rotorsLimits; /** contains limit about rotors such as position and pwm */
    std::vector<eomc::jointLimits_t>        _jointsLimits; /** contains limit about joints such as position and velocity */
    std::vector<eomc::motorCurrentLimits_t> _currentLimits;
    eomc::couplingInfo_t                    _couplingInfo; /** contains coupling matrix */
    std::vector<eomc::JointsSet>            _jsets;
    std::vector<int>                        _joint2set;   /** for each joint says the number of  set it belongs to */
    std::vector<eomc::timeouts_t>           _timeouts;

    std::vector<eomc::impedanceParameters_t> _impedance_params;   /** impedance parameters */ // TODO doubled!!! optimize using just one of the 2!!!
    eomc::impedanceLimits_t *               _impedance_limits;  /** impedancel imits */


    eomc::PidInfo    *                      _ppids;
    eomc::PidInfo    *                      _vpids;
    eomc::TrqPidInfo *                      _tpids;
    eomc::PidInfo    *                      _cpids;

    int *                                   _axisMap;   /** axies map*/
    std::vector<eomc::axisInfo_t>           _axesInfo;
    /////// end configuration info


#ifdef VERIFY_ROP_SETIMPEDANCE
    uint32_t *impedanceSignature;
#endif

#ifdef VERIFY_ROP_SETPOSITIONRAW
    uint32_t *refRawSignature;
    bool        *sendingDirects;
#endif



    double  SAFETY_THRESHOLD;


    // internal stuff
    bool    *_enabledAmp;       // Middle step toward a full enabled motor controller. Amp (pwm) plus Pid enable command must be sent in order to get the joint into an active state.
    bool    *_enabledPid;       // Depends on enabledAmp. When both are set, the joint exits the idle mode and goes into position mode. If one of them is disabled, it falls to idle.
    bool    *_calibrated;       // Flag to know if the calibrate function has been called for the joint
    double  *_ref_command_positions;// used for position control.
    double  *_ref_speeds;       // used for position control.
    double  *_ref_command_speeds;   // used for velocity control.
    double  *_ref_positions;    // used for direct position control.
    double  *_ref_accs;         // for velocity control, in position min jerk eq is used.
    double  *_encodersStamp;                    /** keep information about acquisition time for encoders read */
    bool  *checking_motiondone;                 /* flag telling if I'm already waiting for motion done */
    #define MAX_POSITION_MOVE_INTERVAL 0.080
    double *_last_position_move_time;           /** time stamp for last received position move command*/    
    eOmc_impedance_t *_cacheImpedance;    /* cache impedance value to split up the 2 sets */
    


private:

    std::string getBoardInfo(void);
    bool askRemoteValue(eOprotID32_t id32, void* value, uint16_t& size);
    template <class T> 
    bool askRemoteValues(eOprotEndpoint_t ep, eOprotEntity_t entity, eOprotTag_t tag, std::vector<T>& values);
    bool checkRemoteControlModeStatus(int joint, int target_mode);

    bool dealloc();


    bool convertPosPid(eomc::PidInfo myPidInfo[]);
    bool convertTrqPid(eomc::TrqPidInfo myPidInfo[]);

    bool verifyUserControlLawConsistencyInJointSet(eomc::PidInfo *ipdInfo);
    bool verifyUserControlLawConsistencyInJointSet(eomc::TrqPidInfo *pidInfo);
    bool verifyTorquePidshasSameUnitTypes(eomc::GenericControlUnitsType_t &unittype);
    bool verifyUseMotorSpeedFbkInJointSet(int useMotorSpeedFbk []);
    bool updatedJointsetsCfgWithControlInfo(void);
    bool saveCouplingsData(void);
    bool updatedJointsetsCfg(int joint, eOmc_pidoutputtype_t pidoutputtype);
    void debugUtil_printJointsetInfo(void);
    void debugUtil_printControlLaws(void);

    bool isTorqueControlEnabled(int joint);
    bool isVelocityControlEnabled(int joint);

    bool iNeedCouplingsInfo(void); //the device needs coupling info if it manages joints controlled by 2foc and mc4plus.
    bool iMange2focBoards(void);

    bool getJointConfiguration(int joint, eOmc_joint_config_t *jntCfg_ptr);
    bool getMotorConfiguration(int axis, eOmc_motor_config_t *motCfg_ptr);
    bool getGerabox_E2J(int joint, double *gearbox_E2J_ptr);
    bool getJointEncTolerance(int joint, double *jEncTolerance_ptr);
    bool getMotorEncTolerance(int axis, double *mEncTolerance_ptr);
    void updateDeadZoneWithDefaultValues(void);
    bool getJointDeadZoneRaw(int j, double &jntDeadZone);

private:
    
    //functions used in init this object
    bool fromConfig(yarp::os::Searchable &config);
    int fromConfig_NumOfJoints(yarp::os::Searchable &config);
    bool fromConfig_getGeneralInfo(yarp::os::Searchable &config); //get general info: useRawEncoderData, useLiitedPwm, etc....
    bool fromConfig_Step2(yarp::os::Searchable &config);
    bool fromConfig_readServiceCfg(yarp::os::Searchable &config);
    bool initializeInterfaces(measureConvFactors &f);
    bool alloc(int njoints);
    bool init(void);
    
    //function used in the closing this object
    void cleanup(void);
    
    //used in pid interface
    bool helper_setPosPidRaw( int j, const Pid &pid);
    bool helper_getPosPidRaw(int j, Pid *pid);
    bool helper_getPosPidsRaw(Pid *pid);
    
    //used in torque control interface
    bool helper_setTrqPidRaw( int j, const Pid &pid);
    bool helper_getTrqPidRaw(int j, Pid *pid);
    bool helper_getTrqPidsRaw(Pid *pid);
    
    //used in velocity control interface
    bool helper_setVelPidRaw( int j, const Pid &pid);
    bool helper_getVelPidRaw(int j, Pid *pid);
    bool helper_getVelPidsRaw(Pid *pid);
    
    //used in current control interface
    bool helper_setCurPidRaw(int j, const Pid &pid);
    bool helper_getCurPidRaw(int j, Pid *pid);
    bool helper_getCurPidsRaw(Pid *pid);
    
    
public:

    embObjMotionControl();
    ~embObjMotionControl();


    // Device Driver
    virtual bool open(yarp::os::Searchable &par);
    virtual bool close();

    virtual bool initialised();
    virtual eth::iethresType_t type();
    virtual bool update(eOprotID32_t id32, double timestamp, void *rxdata);

    /////////   PID INTERFACE   /////////
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

    // POSITION CONTROL INTERFACE RAW
    virtual bool getAxes(int *ax);
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

    //  Velocity control interface raw
    virtual bool velocityMoveRaw(int j, double sp);
    virtual bool velocityMoveRaw(const double *sp);


    // calibration2raw
    virtual bool setCalibrationParametersRaw(int axis, const CalibrationParameters& params);
    virtual bool calibrate2Raw(int axis, unsigned int type, double p1, double p2, double p3);
    virtual bool doneRaw(int j);


    /////////////////////////////// END Position Control INTERFACE

    // ControlMode
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
    virtual bool getMotorEncoderTimedRaw(int m, double *encs, double *stamp);\
    virtual bool getMotorEncoderCountsPerRevolutionRaw(int m, double *v);
    virtual bool setMotorEncoderCountsPerRevolutionRaw(int m, const double cpr);
    ///////////////////////// END MotorEncoder Interface

    //////////////////////// BEGIN RemoteVariables Interface
    virtual bool getRemoteVariableRaw(yarp::os::ConstString key, yarp::os::Bottle& val);
    virtual bool setRemoteVariableRaw(yarp::os::ConstString key, const yarp::os::Bottle& val);
    virtual bool getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys);
    ///////////////////////// END RemoteVariables Interface

    //////////////////////// BEGIN IAxisInfo Interface
    virtual bool getAxisNameRaw(int axis, yarp::os::ConstString& name);
    virtual bool getJointTypeRaw(int axis, yarp::dev::JointTypeEnum& type);
    ///////////////////////// END IAxisInfo Interface

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
    virtual bool getTorqueControlFilterType(int j, int& type);
    virtual bool getRotorLimitsRaw(int j, double *rotorMin, double *rotorMax);

    ////// Amplifier interface
    virtual bool enableAmpRaw(int j);
    virtual bool disableAmpRaw(int j);
    virtual bool getCurrentsRaw(double *vals);
    virtual bool getCurrentRaw(int j, double *val);
    virtual bool setMaxCurrentRaw(int j, double val);
    virtual bool getMaxCurrentRaw(int j, double *val);
    virtual bool getAmpStatusRaw(int *st);
    virtual bool getAmpStatusRaw(int j, int *st);
    virtual bool getPWMRaw(int j, double* val);
    virtual bool getPWMLimitRaw(int j, double* val);
    virtual bool setPWMLimitRaw(int j, const double val);
    virtual bool getPowerSupplyVoltageRaw(int j, double* val);
    /////////////// END AMPLIFIER INTERFACE

    // virtual analog sensor
    virtual IVirtualAnalogSensor::VAS_status getVirtualAnalogSensorStatus(int ch);
    virtual int getVirtualAnalogSensorChannels();
    virtual bool updateVirtualAnalogSensorMeasure(yarp::sig::Vector &fTorques);
    virtual bool updateVirtualAnalogSensorMeasure(int j, double &fTorque);

#ifdef IMPLEMENT_DEBUG_INTERFACE
    //----------------------------------------------
    //  Debug interface
    //----------------------------------------------
    virtual bool setParameterRaw(int j, unsigned int type, double value);
    virtual bool getParameterRaw(int j, unsigned int type, double *value);
    virtual bool setDebugParameterRaw(int j, unsigned int index, double value);
    virtual bool setDebugReferencePositionRaw(int j, double value);
    virtual bool getDebugParameterRaw(int j, unsigned int index, double *value);
    virtual bool getDebugReferencePositionRaw(int j, double *value);
#endif

    // Limits
    virtual bool setLimitsRaw(int axis, double min, double max);
    virtual bool getLimitsRaw(int axis, double *min, double *max);
    // Limits 2
    virtual bool setVelLimitsRaw(int axis, double min, double max);
    virtual bool getVelLimitsRaw(int axis, double *min, double *max);

    // Torque control
    virtual bool getTorqueRaw(int j, double *t);
    virtual bool getTorquesRaw(double *t);
    virtual bool getBemfParamRaw(int j, double *bemf);
    virtual bool setBemfParamRaw(int j, double bemf);
    virtual bool getTorqueRangeRaw(int j, double *min, double *max);
    virtual bool getTorqueRangesRaw(double *min, double *max);
    virtual bool setRefTorquesRaw(const double *t);
    virtual bool setRefTorqueRaw(int j, double t);
    virtual bool setRefTorquesRaw(const int n_joint, const int *joints, const double *t);
    virtual bool getRefTorquesRaw(double *t);
    virtual bool getRefTorqueRaw(int j, double *t);
    virtual bool getMotorTorqueParamsRaw(int j, MotorTorqueParameters *params);
    virtual bool setMotorTorqueParamsRaw(int j, const MotorTorqueParameters params);


    // IVelocityControl2
    virtual bool velocityMoveRaw(const int n_joint, const int *joints, const double *spds);
    virtual bool getRefVelocityRaw(const int joint, double *ref);
    virtual bool getRefVelocitiesRaw(double *refs);
    virtual bool getRefVelocitiesRaw(const int n_joint, const int *joints, double *refs);


    // Impedance interface
    virtual bool getImpedanceRaw(int j, double *stiffness, double *damping);
    virtual bool setImpedanceRaw(int j, double stiffness, double damping);
    virtual bool setImpedanceOffsetRaw(int j, double offset);
    virtual bool getImpedanceOffsetRaw(int j, double *offset);
    virtual bool getCurrentImpedanceLimitRaw(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp);
    virtual bool getWholeImpedanceRaw(int j, eOmc_impedance_t &imped);

    // PositionDirect Interface
    virtual bool setPositionRaw(int j, double ref);
    virtual bool setPositionsRaw(const int n_joint, const int *joints, double *refs);
    virtual bool setPositionsRaw(const double *refs);
    virtual bool getRefPositionRaw(const int joint, double *ref);
    virtual bool getRefPositionsRaw(double *refs);
    virtual bool getRefPositionsRaw(const int n_joint, const int *joints, double *refs);

    // InteractionMode interface
    virtual bool getInteractionModeRaw(int j, yarp::dev::InteractionModeEnum* _mode);
    virtual bool getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionModeRaw(int j, yarp::dev::InteractionModeEnum _mode);
    virtual bool setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);

    // IMotor interface
    virtual bool getNumberOfMotorsRaw(int * num);
    virtual bool getTemperatureRaw(int m, double* val);
    virtual bool getTemperaturesRaw(double *vals);
    virtual bool getTemperatureLimitRaw(int m, double *temp);
    virtual bool setTemperatureLimitRaw(int m, const double temp);
    virtual bool getPeakCurrentRaw(int m, double *val);
    virtual bool setPeakCurrentRaw(int m, const double val);
    virtual bool getNominalCurrentRaw(int m, double *val);
    virtual bool setNominalCurrentRaw(int m, const double val);

    // PWMControl
    virtual bool setRefDutyCycleRaw(int j, double v);
    virtual bool setRefDutyCyclesRaw(const double *v);
    virtual bool getRefDutyCycleRaw(int j, double *v);
    virtual bool getRefDutyCyclesRaw(double *v);
    virtual bool getDutyCycleRaw(int j, double *v);
    virtual bool getDutyCyclesRaw(double *v);

    // CurrentControl
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

    
};

#endif // include guard

