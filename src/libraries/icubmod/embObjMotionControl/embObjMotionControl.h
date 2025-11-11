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

// system std include
#include <string>
#include <mutex>
#include <math.h>
//  Yarp stuff
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Timer.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardHelper.h>

#include <yarp/dev/IVirtualAnalogSensor.h>
#include <yarp/dev/IVelocityDirect.h>

#include<yarp/dev/ImplementJointFault.h>

#include <abstractEthResource.h>

#include <iCub/IRawValuesPublisher.h>

// local include
#include "IethResource.h"
#include"EoError.h"
#include <ethManager.h>

#include "serviceParser.h"
#include "eomcParser.h"
#include "measuresConverter.h"

#include "mcEventDownsampler.h"
#include "ethParser.h"


#ifdef NETWORK_PERFORMANCE_BENCHMARK 
#include <PeriodicEventsVerifier.h>
#endif

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

typedef struct // this struct is used to store the configuration of flags and value for maintenance mode
{
    bool enableSkipRecalibration;   /** if true, the joint will not be recalibrated when the yri is restarted */
} maintenanceModeCfg_t;

class Watchdog
{

private:

bool _isStarted;
uint32_t _count;
double _time;
double _abstime;
uint32_t _threshold; // use 10000 as limit on the watchdog for the error on the temperature sensor receiving of the values - 
                    // since the ETH callback timing is 2ms by default so using 10000 we can set a checking threshould of 5 second
                    // in which we can allow the tdb to not respond. If cannot receive response over 1s we trigger the error
public:

Watchdog(): _count(0), _isStarted(false), _threshold(60000), _time(0), _abstime(yarp::os::Time::now()){;}
Watchdog(uint32_t threshold):_count(0), _isStarted(false), _threshold(threshold), _time(0), _abstime(yarp::os::Time::now()){;}
~Watchdog() = default;
Watchdog(const Watchdog& other) =  default;
Watchdog(Watchdog&& other) noexcept =  default;
Watchdog& operator=(const Watchdog& other) =  default;
Watchdog& operator=(Watchdog&& other) noexcept =  default;


bool isStarted(){return _isStarted;}
void start() {_count = 0; _time = yarp::os::Time::now(); _isStarted = true;}
bool isExpired() {return (_count >= _threshold);}
void increment() {++_count;}
void clear(){_isStarted=false;}
double getStartTime() {return _time;}
uint32_t getCount() {return _count; }
void setThreshold(uint8_t txrateOfRegularROPs){ if(txrateOfRegularROPs != 0) _threshold = _threshold / txrateOfRegularROPs;}
uint32_t getThreshold(){return _threshold;}
double getAbsoluteTime(){return _abstime;}

};

class TemperatureFilter
{
private:
    uint32_t _threshold;   // threshold for the delta between current and previous temperature --> set to 20 Celsius deg by default --> over 20 deg delta spike
    double _motorTempPrev; // motor temperature at previous instant for checking positive temperature spikes 
    bool _isStarted;
    int32_t _initCounter;
    std::vector<double> _initTempBuffer;
public:
    TemperatureFilter(): _threshold(20), _isStarted(false), _initCounter(50), _initTempBuffer(0), _motorTempPrev(0){;}
    TemperatureFilter(uint32_t threshold, int32_t initCounter): _threshold(threshold), _isStarted(false), _initCounter(initCounter), _initTempBuffer(0), _motorTempPrev(0){;}
    ~TemperatureFilter() = default;
    TemperatureFilter(const TemperatureFilter& other) = default;
    TemperatureFilter(TemperatureFilter&& other) noexcept = default;
    TemperatureFilter& operator=(const TemperatureFilter& other) = default;
    TemperatureFilter& operator=(TemperatureFilter&& other) noexcept = default;

    bool isStarted(){return _isStarted;}
    uint32_t getTemperatureThreshold() {return _threshold; }
    double getPrevTemperature(){return _motorTempPrev;}
    void updatePrevTemperature(double temperature){_motorTempPrev = temperature;}
    void start(double temperature)
    {
        if(_initCounter < 0)
        {
            int median_pos = std::ceil(_initTempBuffer.size() / 2) -1;
            _motorTempPrev = _initTempBuffer.at(median_pos);
            _isStarted = true;
        }
        else
        {
            _initTempBuffer.push_back(temperature);
            --_initCounter;
        }
        
    }
};

}}}

namespace yarp {
    namespace dev  {
    class embObjMotionControl;
    }
}


struct MCdiagnostics
{
    eOmn_serv_diagn_cfg_t config;
    std::vector<BufferedPort<Bottle>*> ports;
};

using namespace yarp::dev;
using namespace iCub;


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
    public eth::IethResource,
    public IJointFaultRaw,
    public ImplementJointFault,
    public iCub::debugLibrary::IRawValuesPublisher,
    public IVelocityDirectRaw,
    public ImplementVelocityDirect
    {
private:
    eth::TheEthManager*        ethManager;
    eth::AbstractEthResource*  res;
    ServiceParser*             parser;
    eomc::Parser *             _mcparser;
    ControlBoardHelper*        _measureConverter;
    std::mutex                 _mutex;
    
    bool opened; //internal state

    MCdiagnostics mcdiagnostics;

     /////configuartion info (read from xml files)
    int                                     _njoints;       /** Number of joints handled by this EMS */
    eomc::behaviour_flags_t                  behFlags;       /** Contains all flags that define the behaviour of this device */
    servConfigMC_t                          serviceConfig;  /** contains the needed data for configure motion control service, like i.e. board ports where joint are connected */ 
    double *                                _gearbox_M2J;   /** the gearbox ratio motor to joint */
    double *                                _gearbox_E2J;   /** the gearbox ratio encoder to joint */
    double *                                _deadzone;
    std::vector<eomc::kalmanFilterParams_t> _kalman_params;  /** Kalman filter parameters */
    eomc::maintenanceModeCfg_t              _maintenanceModeCfg; /** contains the configuration for maintenance mode */

    std::vector<std::unique_ptr<eomc::ITemperatureSensor>> _temperatureSensorsVector;  
    
    eomc::focBasedSpecificInfo_t *           _foc_based_info;

    std::vector<eomc::encoder_t>             _jointEncs;
    std::vector<eomc::encoder_t>             _motorEncs;

    std::vector<eomc::rotorLimits_t>         _rotorsLimits; /** contains limit about rotors such as position and pwm */
    std::vector<eomc::jointLimits_t>         _jointsLimits; /** contains limit about joints such as position and velocity */
    std::vector<eomc::motorCurrentLimits_t>  _currentLimits;
    std::vector<eomc::temperatureLimits_t>   _temperatureLimits;
    eomc::couplingInfo_t                     _couplingInfo; /** contains coupling matrix */
    std::vector<eomc::JointsSet>             _jsets;
    std::vector<int>                         _joint2set;   /** for each joint says the number of  set it belongs to */
    std::vector<eomc::timeouts_t>            _timeouts;

    std::vector<eomc::impedanceParameters_t> _impedance_params;   /** impedance parameters */ // TODO doubled!!! optimize using just one of the 2!!!
    std::vector<eomc::lugreParameters_t>     _lugre_params;   /** LuGre friction model parameters */
    eomc::impedanceLimits_t *                _impedance_limits;  /** impedance limits */


    eomc::PidInfo    *                      _trj_pids;
    eomc::PidInfo    *                      _dir_pos_pids;
    eomc::PidInfo    *                      _dir_vel_pids;
    eomc::TrqPidInfo *                      _trq_pids;
    eomc::PidInfo    *                      _cur_pids;
    eomc::PidInfo    *                      _vel_pids;

    int *                                   _axisMap;   /** axies map*/
    std::vector<eomc::axisInfo_t>           _axesInfo;
    /////// end configuration info
    
    // event downsampler
    mced::mcEventDownsampler* event_downsampler;

    eth::parser::boardData bdata;

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
    bool    *checking_motiondone;                 /* flag telling if I'm already waiting for motion done */
    #define MAX_POSITION_MOVE_INTERVAL 0.080
    double *_last_position_move_time;           /** time stamp for last received position move command*/    
    eOmc_impedance_t *_cacheImpedance;    /* cache impedance value to split up the 2 sets */
    std::vector<yarp::dev::eomc::Watchdog>    _temperatureSensorErrorWatchdog;  /* counter used to filter error coming from tdb reading fromm 2FOC board*/
    std::vector<yarp::dev::eomc::Watchdog>    _temperatureExceededLimitWatchdog;  /* counter used to filter the print of the exeded limits*/
    std::vector<yarp::dev::eomc::TemperatureFilter> _temperatureSpikesFilter;

    std::map<std::string, rawValuesKeyMetadata> _rawValuesMetadataMap;
    std::vector<std::int32_t> _rawDataAuxVector;

#ifdef NETWORK_PERFORMANCE_BENCHMARK 
    Tools:Emb_RensponseTimingVerifier m_responseTimingVerifier;
#endif

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
    
    bool checkCalib14RotationParam(int32_t calib_param4);

    // used in rawvaluespublisher interface
    bool getRawData_core(std::string key, std::vector<std::int32_t> &data);
    
public:

    embObjMotionControl();
    ~embObjMotionControl();


    // Device Driver
    virtual bool open(yarp::os::Searchable &par);
    virtual bool close();

    virtual bool initialised();
    virtual eth::iethresType_t type();
    virtual bool update(eOprotID32_t id32, double timestamp, void *rxdata);
    virtual bool getEntityName(uint32_t entityId, std::string &entityName);
    virtual bool getEncoderTypeName(uint32_t jomoId, eOmc_position_t pos, std::string &encoderTypeName) override;

    /////////   PID INTERFACE   /////////
    virtual ReturnValue setPidRaw(const PidControlTypeEnum& pidtype, int j, const Pid &pid) override;
    virtual ReturnValue setPidsRaw(const PidControlTypeEnum& pidtype, const Pid *pids) override;
    virtual ReturnValue setPidReferenceRaw(const PidControlTypeEnum& pidtype, int j, double ref) override;
    virtual ReturnValue setPidReferencesRaw(const PidControlTypeEnum& pidtype, const double *refs) override;
    virtual ReturnValue setPidErrorLimitRaw(const PidControlTypeEnum& pidtype, int j, double limit) override;
    virtual ReturnValue setPidErrorLimitsRaw(const PidControlTypeEnum& pidtype, const double *limits) override;
    virtual ReturnValue getPidErrorRaw(const PidControlTypeEnum& pidtype, int j, double *err) override;
    virtual ReturnValue getPidErrorsRaw(const PidControlTypeEnum& pidtype, double *errs) override;
    virtual ReturnValue getPidOutputRaw(const PidControlTypeEnum& pidtype, int j, double *out) override;
    virtual ReturnValue getPidOutputsRaw(const PidControlTypeEnum& pidtype, double *outs) override;
    virtual ReturnValue getPidRaw(const PidControlTypeEnum& pidtype, int j, Pid *pid) override;
    virtual ReturnValue getPidsRaw(const PidControlTypeEnum& pidtype, Pid *pids) override;
    virtual ReturnValue getPidReferenceRaw(const PidControlTypeEnum& pidtype, int j, double *ref) override;
    virtual ReturnValue getPidReferencesRaw(const PidControlTypeEnum& pidtype, double *refs) override;
    virtual ReturnValue getPidErrorLimitRaw(const PidControlTypeEnum& pidtype, int j, double *limit) override;
    virtual ReturnValue getPidErrorLimitsRaw(const PidControlTypeEnum& pidtype, double *limits) override;
    virtual ReturnValue resetPidRaw(const PidControlTypeEnum& pidtype, int j) override;
    virtual ReturnValue disablePidRaw(const PidControlTypeEnum& pidtype, int j) override;
    virtual ReturnValue enablePidRaw(const PidControlTypeEnum& pidtype, int j) override;
    virtual ReturnValue setPidOffsetRaw(const PidControlTypeEnum& pidtype, int j, double v) override;
    virtual ReturnValue setPidFeedforwardRaw(const PidControlTypeEnum& pidtype,int j, double v) override;
    virtual ReturnValue getPidOffsetRaw(const PidControlTypeEnum& pidtype,int j, double& v) override;
    virtual ReturnValue getPidFeedforwardRaw(const PidControlTypeEnum& pidtype,int j, double& v) override;
    virtual ReturnValue isPidEnabledRaw(const PidControlTypeEnum& pidtype, int j, bool& enabled) override;
    virtual ReturnValue getPidExtraInfoRaw(const PidControlTypeEnum& pidtype, int j, yarp::dev::PidExtraInfo& units) override;
    virtual ReturnValue getPidExtraInfosRaw(const PidControlTypeEnum& pidtype, std::vector<yarp::dev::PidExtraInfo>& units) override;

    // POSITION CONTROL INTERFACE RAW
    virtual ReturnValue getAxes(int *ax) override;
    virtual ReturnValue positionMoveRaw(int j, double ref) override;
    virtual ReturnValue positionMoveRaw(const double *refs) override;
    virtual ReturnValue relativeMoveRaw(int j, double delta) override;
    virtual ReturnValue relativeMoveRaw(const double *deltas) override;
    virtual ReturnValue checkMotionDoneRaw(bool *flag) override;
    virtual ReturnValue checkMotionDoneRaw(int j, bool *flag) override;
    virtual ReturnValue setTrajSpeedRaw(int j, double sp) override;
    virtual ReturnValue setTrajSpeedsRaw(const double *spds) override;
    virtual ReturnValue setTrajAccelerationRaw(int j, double acc) override;
    virtual ReturnValue setTrajAccelerationsRaw(const double *accs) override;
    virtual ReturnValue getTrajSpeedRaw(int j, double *ref) override;
    virtual ReturnValue getTrajSpeedsRaw(double *spds) override;
    virtual ReturnValue getTrajAccelerationRaw(int j, double *acc) override;
    virtual ReturnValue getTrajAccelerationsRaw(double *accs) override;
    virtual ReturnValue stopRaw(int j) override;
    virtual ReturnValue stopRaw() override;


    // Position Control2 Interface
    virtual ReturnValue positionMoveRaw(const int n_joint, const int *joints, const double *refs) override;
    virtual ReturnValue relativeMoveRaw(const int n_joint, const int *joints, const double *deltas) override;
    virtual ReturnValue checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags) override;
    virtual ReturnValue setTrajSpeedsRaw(const int n_joint, const int *joints, const double *spds) override;
    virtual ReturnValue setTrajAccelerationsRaw(const int n_joint, const int *joints, const double *accs) override;
    virtual ReturnValue getTrajSpeedsRaw(const int n_joint, const int *joints, double *spds) override;
    virtual ReturnValue getTrajAccelerationsRaw(const int n_joint, const int *joints, double *accs) override;
    virtual ReturnValue stopRaw(const int n_joint, const int *joints) override;
    virtual ReturnValue getTargetPositionRaw(const int joint, double *ref) override;
    virtual ReturnValue getTargetPositionsRaw(double *refs) override;
    virtual ReturnValue getTargetPositionsRaw(const int n_joint, const int *joints, double *refs) override;

    //  Velocity control interface raw
    virtual ReturnValue velocityMoveRaw(int j, double sp) override;
    virtual ReturnValue velocityMoveRaw(const double *sp) override;


    // calibration2raw
    virtual ReturnValue setCalibrationParametersRaw(int axis, const CalibrationParameters& params) override;
    virtual ReturnValue calibrateAxisWithParamsRaw(int axis, unsigned int type, double p1, double p2, double p3) override;
    virtual ReturnValue calibrationDoneRaw(int j) override;


    /////////////////////////////// END Position Control INTERFACE

    // ControlMode
    virtual ReturnValue getControlModeRaw(int j, int *v) override;
    virtual ReturnValue getControlModesRaw(int *v) override;

    // ControlMode 2
    virtual ReturnValue getControlModesRaw(const int n_joint, const int *joints, int *modes) override;
    virtual ReturnValue setControlModeRaw(const int j, const int mode) override;
    virtual ReturnValue setControlModesRaw(const int n_joint, const int *joints, int *modes) override;
    virtual ReturnValue setControlModesRaw(int *modes) override;

    //////////////////////// BEGIN EncoderInterface
    virtual ReturnValue resetEncoderRaw(int j) override;
    virtual ReturnValue resetEncodersRaw() override;
    virtual ReturnValue setEncoderRaw(int j, double val) override;
    virtual ReturnValue setEncodersRaw(const double *vals) override;
    virtual ReturnValue getEncoderRaw(int j, double *v) override;
    virtual ReturnValue getEncodersRaw(double *encs) override;
    virtual ReturnValue getEncoderSpeedRaw(int j, double *sp) override;
    virtual ReturnValue getEncoderSpeedsRaw(double *spds) override;
    virtual ReturnValue getEncoderAccelerationRaw(int j, double *spds) override;
    virtual ReturnValue getEncoderAccelerationsRaw(double *accs) override;
    ///////////////////////// END Encoder Interface

    virtual ReturnValue getEncodersTimedRaw(double *encs, double *stamps) override;
    virtual ReturnValue getEncoderTimedRaw(int j, double *encs, double *stamp) override;

    //////////////////////// BEGIN MotorEncoderInterface
    virtual ReturnValue getNumberOfMotorEncodersRaw(int * num) override;
    virtual ReturnValue resetMotorEncoderRaw(int m) override;
    virtual ReturnValue resetMotorEncodersRaw() override;
    virtual ReturnValue setMotorEncoderRaw(int m, const double val) override;
    virtual ReturnValue setMotorEncodersRaw(const double *vals) override;
    virtual ReturnValue getMotorEncoderRaw(int m, double *v) override;
    virtual ReturnValue getMotorEncodersRaw(double *encs) override;
    virtual ReturnValue getMotorEncoderSpeedRaw(int m, double *sp) override;
    virtual ReturnValue getMotorEncoderSpeedsRaw(double *spds) override;
    virtual ReturnValue getMotorEncoderAccelerationRaw(int m, double *spds) override;
    virtual ReturnValue getMotorEncoderAccelerationsRaw(double *accs) override;
    virtual ReturnValue getMotorEncodersTimedRaw(double *encs, double *stamps) override;
    virtual ReturnValue getMotorEncoderTimedRaw(int m, double *encs, double *stamp) override;
    virtual ReturnValue getMotorEncoderCountsPerRevolutionRaw(int m, double *v) override;
    virtual ReturnValue setMotorEncoderCountsPerRevolutionRaw(int m, const double cpr) override;
    ///////////////////////// END MotorEncoder Interface

    //////////////////////// BEGIN RemoteVariables Interface
    virtual ReturnValue getRemoteVariableRaw(std::string key, yarp::os::Bottle& val) override;
    virtual ReturnValue setRemoteVariableRaw(std::string key, const yarp::os::Bottle& val) override;
    virtual ReturnValue getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys) override;
    ///////////////////////// END RemoteVariables Interface

    //////////////////////// BEGIN IAxisInfo Interface
    virtual ReturnValue getAxisNameRaw(int axis, std::string& name) override;
    virtual ReturnValue getJointTypeRaw(int axis, yarp::dev::JointTypeEnum& type) override;
    ///////////////////////// END IAxisInfo Interface

    //Internal use, not exposed by Yarp (yet)
    bool getRotorEncoderResolutionRaw(int m, double &rotres) ;
    bool getJointEncoderResolutionRaw(int m, double &jntres) ;
    bool getJointEncoderTypeRaw(int j, int &type) ;
    bool getRotorEncoderTypeRaw(int j, int &type) ;
    bool getKinematicMJRaw(int j, double &rotres) ;
    bool getTemperatureSensorTypeRaw(int j, std::string& ret) ;
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
    virtual ReturnValue enableAmpRaw(int j) override;
    virtual ReturnValue disableAmpRaw(int j) override;
    virtual ReturnValue getCurrentsRaw(double *vals) override;
    virtual ReturnValue getCurrentRaw(int j, double *val) override;
    virtual ReturnValue setMaxCurrentRaw(int j, double val) override;
    virtual ReturnValue getMaxCurrentRaw(int j, double *val) override;
    virtual ReturnValue getAmpStatusRaw(int *st) override;
    virtual ReturnValue getAmpStatusRaw(int j, int *st) override;
    virtual ReturnValue getPWMRaw(int j, double* val) override;
    virtual ReturnValue getPWMLimitRaw(int j, double* val) override;
    virtual ReturnValue setPWMLimitRaw(int j, const double val) override;
    virtual ReturnValue getPowerSupplyVoltageRaw(int j, double* val) override;

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
    virtual ReturnValue setPosLimitsRaw(int axis, double min, double max) override;
    virtual ReturnValue getPosLimitsRaw(int axis, double *min, double *max) override;
    // Limits 2
    virtual ReturnValue setVelLimitsRaw(int axis, double min, double max) override;
    virtual ReturnValue getVelLimitsRaw(int axis, double *min, double *max) override;

    // Torque control
    virtual ReturnValue getTorqueRaw(int j, double *t) override;
    virtual ReturnValue getTorquesRaw(double *t) override;
    virtual ReturnValue getTorqueRangeRaw(int j, double *min, double *max) override;
    virtual ReturnValue getTorqueRangesRaw(double *min, double *max) override;
    virtual ReturnValue setRefTorquesRaw(const double *t) override;
    virtual ReturnValue setRefTorqueRaw(int j, double t) override;
    virtual ReturnValue setRefTorquesRaw(const int n_joint, const int *joints, const double *t) override;
    virtual ReturnValue getRefTorquesRaw(double *t) override;
    virtual ReturnValue getRefTorqueRaw(int j, double *t) override;
    virtual ReturnValue getMotorTorqueParamsRaw(int j, MotorTorqueParameters *params) override;
    virtual ReturnValue setMotorTorqueParamsRaw(int j, const MotorTorqueParameters params) override;


    // IVelocityControl2
    virtual ReturnValue velocityMoveRaw(const int n_joint, const int *joints, const double *spds) override;
    virtual ReturnValue  getTargetVelocityRaw(const int joint, double *ref) override;
    virtual ReturnValue  getTargetVelocitiesRaw(double *refs) override;
    virtual ReturnValue  getTargetVelocitiesRaw(const int n_joint, const int *joints, double *refs) override;


    // Impedance interface
    virtual ReturnValue getImpedanceRaw(int j, double *stiffness, double *damping) override;
    virtual ReturnValue setImpedanceRaw(int j, double stiffness, double damping) override;
    virtual ReturnValue setImpedanceOffsetRaw(int j, double offset) override;
    virtual ReturnValue getImpedanceOffsetRaw(int j, double *offset) override;
    virtual ReturnValue getCurrentImpedanceLimitRaw(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp) override;

    // PositionDirect Interface
    virtual ReturnValue setPositionRaw(int j, double ref) override;
    virtual ReturnValue setPositionsRaw(const int n_joint, const int *joints, const double *refs) override;
    virtual ReturnValue setPositionsRaw(const double *refs) override;
    virtual ReturnValue getRefPositionRaw(const int joint, double *ref) override;
    virtual ReturnValue getRefPositionsRaw(double *refs) override;
    virtual ReturnValue getRefPositionsRaw(const int n_joint, const int *joints, double *refs) override;

    // InteractionMode interface
    virtual ReturnValue getInteractionModeRaw(int j, yarp::dev::InteractionModeEnum* _mode) override;
    virtual ReturnValue getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes) override;
    virtual ReturnValue getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes) override;
    virtual ReturnValue setInteractionModeRaw(int j, yarp::dev::InteractionModeEnum _mode) override;
    virtual ReturnValue setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes) override;
    virtual ReturnValue setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes) override;

    // IMotor interface
    virtual ReturnValue getNumberOfMotorsRaw(int * num) override;
    virtual ReturnValue getTemperatureRaw(int m, double* val) override;
    virtual ReturnValue getTemperaturesRaw(double *vals) override;
    virtual ReturnValue getTemperatureLimitRaw(int m, double *temp) override;
    virtual ReturnValue setTemperatureLimitRaw(int m, const double temp) override;
    virtual ReturnValue getPeakCurrentRaw(int m, double *val) override;
    virtual ReturnValue setPeakCurrentRaw(int m, const double val) override;
    virtual ReturnValue getNominalCurrentRaw(int m, double *val) override;
    virtual ReturnValue setNominalCurrentRaw(int m, const double val) override;
    virtual ReturnValue getGearboxRatioRaw(int m, double *gearbox) override;
    virtual ReturnValue setGearboxRatioRaw(int m, const double val) override;

    // PWMControl
    virtual ReturnValue setRefDutyCycleRaw(int j, double v) override;
    virtual ReturnValue setRefDutyCyclesRaw(const double *v) override;
    virtual ReturnValue getRefDutyCycleRaw(int j, double *v) override;
    virtual ReturnValue getRefDutyCyclesRaw(double *v) override;
    virtual ReturnValue getDutyCycleRaw(int j, double *v) override;
    virtual ReturnValue getDutyCyclesRaw(double *v) override;

    // CurrentControl
    // virtual bool getAxes(int *ax) override;
    //virtual bool getCurrentRaw(int j, double *t) override;
    //virtual bool getCurrentsRaw(double *t) override;
    virtual ReturnValue getCurrentRangeRaw(int j, double *min, double *max) override;
    virtual ReturnValue getCurrentRangesRaw(double *min, double *max) override;
    virtual ReturnValue setRefCurrentsRaw(const double *t) override;
    virtual ReturnValue setRefCurrentRaw(int j, double t) override;
    virtual ReturnValue setRefCurrentsRaw(const int n_joint, const int *joints, const double *t) override;
    virtual ReturnValue getRefCurrentsRaw(double *t) override;
    virtual ReturnValue getRefCurrentRaw(int j, double *t) override;

    // Used in joint faults interface
    // Teturns true if it was successful and writes the fault code in the fault parameter 
    // with the associated string in message. If no fault is detected the fault parameters is set to -1.
    // Returns false and fault is set to -2 if retrieval was unsuccessful.
    virtual ReturnValue getLastJointFaultRaw(int j, int& fault, std::string& message) override;

    // IRawValuesPublisher
    virtual bool getRawDataMap(std::map<std::string, std::vector<std::int32_t>> &map) override;
    virtual bool getRawData(std::string key, std::vector<std::int32_t> &data) override;
    virtual bool getKeys(std::vector<std::string> &keys) override;
    virtual int  getNumberOfKeys() override;
    virtual bool getMetadataMap(rawValuesKeyMetadataMap &metamap) override;
    virtual bool getKeyMetadata(std::string key, rawValuesKeyMetadata &meta) override;
    virtual bool getAxesNames(std::string key, std::vector<std::string> &axesNames) override;

    // VelocityDirect Interface
    virtual ReturnValue getAxes(size_t& axes) override;
    virtual ReturnValue setRefVelocityRaw(int jnt, double vel) override;
    virtual ReturnValue setRefVelocityRaw(const std::vector<double>& vels) override;
    virtual ReturnValue setRefVelocityRaw(const std::vector<int>& jnts, const std::vector<double>& vels) override;
    virtual ReturnValue getRefVelocityRaw(const int jnt, double& vel) override;
    virtual ReturnValue getRefVelocityRaw(std::vector<double>& vels) override;
    virtual ReturnValue getRefVelocityRaw(const std::vector<int>& jnts, std::vector<double>& vels) override;
};

#endif // include guard
