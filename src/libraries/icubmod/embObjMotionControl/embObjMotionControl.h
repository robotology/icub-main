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




//
// $Id: embObjMotionControl.h,v 1.5 2008/06/25 22:33:53 nat Exp $
//

#ifndef __embObjMotionControlh__
#define __embObjMotionControlh__


using namespace std;

#include <string>
//  Yarp stuff
#include <yarp/os/Bottle.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/ControlBoardHelper.h>

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


/**
 * @ingroup icub_hardware_modules
 * @brief `embObjMotionControl` : driver for iCub motor control boards EMS on a ETH bus.
 *
 * This device contains code which handles communication to
 * the motor control boards (EMS) on the internal ethernet network of the ETH iCub.
 * It converts requests from function calls into ETH bus messages for
 * the motor control boards. A thread monitors the bus for incoming
 * messages and dispatches replies to calling threads.
 *
 * For the description of the parameters supported by this device, please check the
 * template configuration file available in robotology/robots-configuration,
 * i.e.  https://github.com/robotology/robots-configuration/blob/master/iCubTemplates/iCubTemplateV4_0/hardware/motorControl/body_part--ebX-jA_B-mc.xml .
 *
 * | YARP device name |
 * |:-----------------:|
 * | `embObjMotionControl` |
 *
 */
class yarp::dev::embObjMotionControl:   public DeviceDriver,
    public IPidControlRaw,
    public IControlCalibrationRaw,
    public IAmplifierControlRaw,
    public IEncodersTimedRaw,
    public IMotorEncodersRaw,
    public ImplementEncodersTimed,
    public ImplementMotorEncoders,
    public IMotorRaw,
    public ImplementMotor,
    public IPositionControlRaw,
    public IVelocityControlRaw,
    public IControlModeRaw,
    public ImplementControlMode,
    public IControlLimitsRaw,
    public IImpedanceControlRaw,
    public ImplementImpedanceControl,
    public ImplementControlLimits,
    public ImplementAmplifierControl,
    public ImplementPositionControl,
    public ImplementControlCalibration,
    public ImplementPidControl,
    public ImplementVelocityControl,
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
    ControlBoardHelper*        _measureConverter;
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


    eomc::PidInfo    *                      _trj_pids;
    //eomc::PidInfo    *                      _dir_pids;
    eomc::TrqPidInfo *                      _trq_pids;
    eomc::PidInfo    *                      _cur_pids;
    eomc::PidInfo    *                      _spd_pids;

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


    bool verifyUserControlLawConsistencyInJointSet(eomc::PidInfo *ipdInfo);
    bool verifyUserControlLawConsistencyInJointSet(eomc::TrqPidInfo *pidInfo);
    bool verifyTorquePidshasSameUnitTypes(yarp::dev::PidFeedbackUnitsEnum  &fbk_pidunits, yarp::dev::PidOutputUnitsEnum& out_pidunits);
    bool verifyUseMotorSpeedFbkInJointSet(int useMotorSpeedFbk []);
    bool updatedJointsetsCfgWithControlInfo(void);
    bool saveCouplingsData(void);
    void debugUtil_printJointsetInfo(void);

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
    
    //used in low level speed control interface
    bool helper_setSpdPidRaw(int j, const Pid &pid);
    bool helper_getSpdPidRaw(int j, Pid *pid);
    bool helper_getSpdPidsRaw(Pid *pid);
    
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

    // POSITION CONTROL INTERFACE RAW
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

    //  Velocity control interface raw
    virtual bool velocityMoveRaw(int j, double sp) override;
    virtual bool velocityMoveRaw(const double *sp) override;


    // calibration2raw
    virtual bool setCalibrationParametersRaw(int axis, const CalibrationParameters& params) override;
    virtual bool calibrateAxisWithParamsRaw(int axis, unsigned int type, double p1, double p2, double p3) override;
    virtual bool calibrationDoneRaw(int j) override;


    /////////////////////////////// END Position Control INTERFACE

    // ControlMode
    virtual bool getControlModeRaw(int j, int *v) override;
    virtual bool getControlModesRaw(int *v) override;

    // ControlMode 2
    virtual bool getControlModesRaw(const int n_joint, const int *joints, int *modes) override;
    virtual bool setControlModeRaw(const int j, const int mode) override;
    virtual bool setControlModesRaw(const int n_joint, const int *joints, int *modes) override;
    virtual bool setControlModesRaw(int *modes) override;

    //////////////////////// BEGIN EncoderInterface
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

    virtual bool getEncodersTimedRaw(double *encs, double *stamps) override;
    virtual bool getEncoderTimedRaw(int j, double *encs, double *stamp) override;

    //////////////////////// BEGIN MotorEncoderInterface
    virtual bool getNumberOfMotorEncodersRaw(int * num) override;
    virtual bool resetMotorEncoderRaw(int m) override;
    virtual bool resetMotorEncodersRaw() override;
    virtual bool setMotorEncoderRaw(int m, const double val) override;
    virtual bool setMotorEncodersRaw(const double *vals) override;
    virtual bool getMotorEncoderRaw(int m, double *v) override;
    virtual bool getMotorEncodersRaw(double *encs) override;
    virtual bool getMotorEncoderSpeedRaw(int m, double *sp) override;
    virtual bool getMotorEncoderSpeedsRaw(double *spds) override;
    virtual bool getMotorEncoderAccelerationRaw(int m, double *spds) override;
    virtual bool getMotorEncoderAccelerationsRaw(double *accs) override;
    virtual bool getMotorEncodersTimedRaw(double *encs, double *stamps) override;
    virtual bool getMotorEncoderTimedRaw(int m, double *encs, double *stamp) override;
    virtual bool getMotorEncoderCountsPerRevolutionRaw(int m, double *v) override;
    virtual bool setMotorEncoderCountsPerRevolutionRaw(int m, const double cpr) override;
    ///////////////////////// END MotorEncoder Interface

    //////////////////////// BEGIN RemoteVariables Interface
    virtual bool getRemoteVariableRaw(std::string key, yarp::os::Bottle& val) override;
    virtual bool setRemoteVariableRaw(std::string key, const yarp::os::Bottle& val) override;
    virtual bool getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys) override;
    ///////////////////////// END RemoteVariables Interface

    //////////////////////// BEGIN IAxisInfo Interface
    virtual bool getAxisNameRaw(int axis, std::string& name) override;
    virtual bool getJointTypeRaw(int axis, yarp::dev::JointTypeEnum& type) override;
    ///////////////////////// END IAxisInfo Interface

    //Internal use, not exposed by Yarp (yet)
    bool getRotorEncoderResolutionRaw(int m, double &rotres) ;
    bool getJointEncoderResolutionRaw(int m, double &jntres) ;
    bool getJointEncoderTypeRaw(int j, int &type) ;
    bool getRotorEncoderTypeRaw(int j, int &type) ;
    bool getKinematicMJRaw(int j, double &rotres) ;
    bool getHasTempSensorsRaw(int j, int& ret) ;
    bool getHasHallSensorRaw(int j, int& ret) ;
    bool getHasRotorEncoderRaw(int j, int& ret) ;
    bool getHasRotorEncoderIndexRaw(int j, int& ret) ;
    bool getMotorPolesRaw(int j, int& poles) ;
    bool getRotorIndexOffsetRaw(int j, double& rotorOffset) ;
    bool getTorqueControlFilterType(int j, int& type) ;
    bool getRotorLimitsRaw(int j, double *rotorMin, double *rotorMax) ;
    bool getWholeImpedanceRaw(int j, eOmc_impedance_t &imped);

    ////// Amplifier interface
    virtual bool enableAmpRaw(int j) override;
    virtual bool disableAmpRaw(int j) override;
    virtual bool getCurrentsRaw(double *vals) override;
    virtual bool getCurrentRaw(int j, double *val) override;
    virtual bool setMaxCurrentRaw(int j, double val) override;
    virtual bool getMaxCurrentRaw(int j, double *val) override;
    virtual bool getAmpStatusRaw(int *st) override;
    virtual bool getAmpStatusRaw(int j, int *st) override;
    virtual bool getPWMRaw(int j, double* val) override;
    virtual bool getPWMLimitRaw(int j, double* val) override;
    virtual bool setPWMLimitRaw(int j, const double val) override;
    virtual bool getPowerSupplyVoltageRaw(int j, double* val) override;
    /////////////// END AMPLIFIER INTERFACE

    // virtual analog sensor
    virtual yarp::dev::VAS_status getVirtualAnalogSensorStatus(int ch) override;
    virtual int getVirtualAnalogSensorChannels() override;
    virtual bool updateVirtualAnalogSensorMeasure(yarp::sig::Vector &fTorques) override;
    virtual bool updateVirtualAnalogSensorMeasure(int j, double &fTorque) override;

#ifdef IMPLEMENT_DEBUG_INTERFACE
    //----------------------------------------------
    //  Debug interface
    //----------------------------------------------
    virtual bool setParameterRaw(int j, unsigned int type, double value) override;
    virtual bool getParameterRaw(int j, unsigned int type, double *value) override;
    virtual bool setDebugParameterRaw(int j, unsigned int index, double value) override;
    virtual bool setDebugReferencePositionRaw(int j, double value) override;
    virtual bool getDebugParameterRaw(int j, unsigned int index, double *value) override;
    virtual bool getDebugReferencePositionRaw(int j, double *value) override;
#endif

    // Limits
    virtual bool setLimitsRaw(int axis, double min, double max) override;
    virtual bool getLimitsRaw(int axis, double *min, double *max) override;
    // Limits 2
    virtual bool setVelLimitsRaw(int axis, double min, double max) override;
    virtual bool getVelLimitsRaw(int axis, double *min, double *max) override;

    // Torque control
    virtual bool getTorqueRaw(int j, double *t) override;
    virtual bool getTorquesRaw(double *t) override;
    virtual bool getTorqueRangeRaw(int j, double *min, double *max) override;
    virtual bool getTorqueRangesRaw(double *min, double *max) override;
    virtual bool setRefTorquesRaw(const double *t) override;
    virtual bool setRefTorqueRaw(int j, double t) override;
    virtual bool setRefTorquesRaw(const int n_joint, const int *joints, const double *t) override;
    virtual bool getRefTorquesRaw(double *t) override;
    virtual bool getRefTorqueRaw(int j, double *t) override;
    virtual bool getMotorTorqueParamsRaw(int j, MotorTorqueParameters *params) override;
    virtual bool setMotorTorqueParamsRaw(int j, const MotorTorqueParameters params) override;


    // IVelocityControl2
    virtual bool velocityMoveRaw(const int n_joint, const int *joints, const double *spds) override;
    virtual bool getRefVelocityRaw(const int joint, double *ref) override;
    virtual bool getRefVelocitiesRaw(double *refs) override;
    virtual bool getRefVelocitiesRaw(const int n_joint, const int *joints, double *refs) override;


    // Impedance interface
    virtual bool getImpedanceRaw(int j, double *stiffness, double *damping) override;
    virtual bool setImpedanceRaw(int j, double stiffness, double damping) override;
    virtual bool setImpedanceOffsetRaw(int j, double offset) override;
    virtual bool getImpedanceOffsetRaw(int j, double *offset) override;
    virtual bool getCurrentImpedanceLimitRaw(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp) override;

    // PositionDirect Interface
    virtual bool setPositionRaw(int j, double ref) override;
    virtual bool setPositionsRaw(const int n_joint, const int *joints, const double *refs) override;
    virtual bool setPositionsRaw(const double *refs) override;
    virtual bool getRefPositionRaw(const int joint, double *ref) override;
    virtual bool getRefPositionsRaw(double *refs) override;
    virtual bool getRefPositionsRaw(const int n_joint, const int *joints, double *refs) override;

    // InteractionMode interface
    virtual bool getInteractionModeRaw(int j, yarp::dev::InteractionModeEnum* _mode) override;
    virtual bool getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes) override;
    virtual bool getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes) override;
    virtual bool setInteractionModeRaw(int j, yarp::dev::InteractionModeEnum _mode) override;
    virtual bool setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes) override;
    virtual bool setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes) override;

    // IMotor interface
    virtual bool getNumberOfMotorsRaw(int * num) override;
    virtual bool getTemperatureRaw(int m, double* val) override;
    virtual bool getTemperaturesRaw(double *vals) override;
    virtual bool getTemperatureLimitRaw(int m, double *temp) override;
    virtual bool setTemperatureLimitRaw(int m, const double temp) override;
    virtual bool getPeakCurrentRaw(int m, double *val) override;
    virtual bool setPeakCurrentRaw(int m, const double val) override;
    virtual bool getNominalCurrentRaw(int m, double *val) override;
    virtual bool setNominalCurrentRaw(int m, const double val) override;
    virtual bool getGearboxRatioRaw(int m, double *gearbox) override;

    // PWMControl
    virtual bool setRefDutyCycleRaw(int j, double v) override;
    virtual bool setRefDutyCyclesRaw(const double *v) override;
    virtual bool getRefDutyCycleRaw(int j, double *v) override;
    virtual bool getRefDutyCyclesRaw(double *v) override;
    virtual bool getDutyCycleRaw(int j, double *v) override;
    virtual bool getDutyCyclesRaw(double *v) override;

    // CurrentControl
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

    
};

#endif // include guard

