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
#include <yarp/os/Time.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
//#include <yarp/dev/ControlBoardInterfacesImpl.h>
//#include <yarp/dev/ControlBoardInterfacesImpl.inl>

#include <yarp/dev/IVirtualAnalogSensor.h>


#include <iCub/FactoryInterface.h>



// ACE udp socket
#include <ace/ACE.h>
#include <ace/SOCK_Dgram_Bcast.h>

#include "FeatureInterface.h"


#include "EoMotionControl.h"
#include <ethManager.h>
#include <ethResource.h>
#include "../embObjLib/hostTransceiver.hpp"
// #include "IRobotInterface.h"
#include "IethResource.h"
#include "eoRequestsQueue.hpp"
#include "EoMotionControl.h"

#include "serviceParser.h"
#include "mcParser.h"

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

// marco.accame:
// if the macro is disabled: the service parser which reads the runtime config parameters from the xml file is NOT called. everything is as it was before.
// enable it only whan the service parser is full developed.

#define EMBOBJMC_USESERVICEPARSER

//
//   Help structure
//
using namespace yarp::os;
using namespace yarp::dev;

// struct ImpedanceLimits
// {
//     double min_stiff;
//     double max_stiff;
//     double min_damp;
//     double max_damp;
//     double param_a;
//     double param_b;
//     double param_c;
//
// public:
//     ImpedanceLimits()
//     {
//         min_stiff=0;
//         max_stiff=0;
//         min_damp=0;
//         max_damp=0;
//         param_a=0;
//         param_b=0;
//         param_c=0;
//     }
//
//     double get_min_stiff()
//     {
//         return min_stiff;
//     }
//     double get_max_stiff()
//     {
//         return max_stiff;
//     }
//     double get_min_damp()
//     {
//         return min_damp;
//     }
//     double get_max_damp()
//     {
//         return max_damp;
//     }
// };



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




class torqueControlHelper
{
    int  jointsNum;
    double* newtonsToSensor;
    double* angleToEncoders;

    public:
    torqueControlHelper(int njoints, double* angleToEncoders, double* newtons2sens);
    torqueControlHelper(int njoints, float* angleToEncoders, double* newtons2sens);
    inline ~torqueControlHelper()
    {
        if (newtonsToSensor)   delete [] newtonsToSensor;
        if (angleToEncoders)   delete [] angleToEncoders;
        newtonsToSensor=0;
        angleToEncoders=0;
    }
    inline double getNewtonsToSensor (int jnt)
    {
        if (jnt>=0 && jnt<jointsNum) return newtonsToSensor[jnt];
        return 0;
    }
    inline double getAngleToEncoders (int jnt)
    {
        if (jnt>=0 && jnt<jointsNum) return angleToEncoders[jnt];
        return 0;
    }
    inline int getNumberOfJoints ()
    {
        return jointsNum;
    }
    
    inline double convertImpN2S(int j, double nw)
    {
        return nw * newtonsToSensor[j]/angleToEncoders[j];
    }
};



typedef struct
{
    vector<int>                         joint2set;
    vector <vector <int> >              set2joint;
    int                                 numofjointsets;
    vector<eOmc_jointset_configuration_t> jointset_cfgs;
} eomc_jointsetsInfo_t;





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
    public IethResource
{


private:

    char boardIPstring[20];

    TheEthManager* ethManager;
    EthResource* res;
    ServiceParser* parser;
    mcParser *_mcparser;

    bool opened;
    bool verbosewhenok;

    ////////////////////
    // parameters
#if defined(EMBOBJMC_USESERVICEPARSER)
    servConfigMC_t serviceConfig;
#endif

    //int tot_packet_recv;
    //int errors;

    yarp::os::Semaphore _mutex;


    double *_angleToEncoder;                    /** angle to iCubDegrees conversion factors */
    double *_ampsToSensor;
    double *_dutycycleToPWM;
    double  *_encodersStamp;                    /** keep information about acquisition time for encoders read */
    uint8_t *_jointEncoderType;                 /** joint encoder type*/
    uint8_t *_jointNumOfNoiseBits;              /** Num of error bits passable for joint encoder */
    int    *_jointEncoderRes;                   /** joint encoder resolution */
    int    *_rotorEncoderRes;                   /** rotor encoder resolution */
    uint8_t *_rotorNumOfNoiseBits;              /** Num of error bits passable for joint encoder */
    uint8_t *_rotorEncoderType;                  /** rotor encoder type*/
    double *_gearbox;                           /** the gearbox ratio */
    double *_gearboxE2J;                        /** the gearbox ratio */

    eomc_twofocSpecificInfo *_twofocinfo;



    std::vector<eomc_rotorLimits>           _rotorsLimits; /** contains limit about rotors such as position and pwm */
    std::vector<eomc_jointLimits>           _jointsLimits; /** contains limit about joints such as position and velocity */
    std::vector<eomc_motorCurrentLimits>    _currentLimits;
    eomc_couplingInfo_t                     _couplingInfo; /** contains coupling matrix */
    std::vector<eomc_jointsSet>             _jsets;
    std::vector<int>                        _joint2set;
    std::vector<eomc_timeouts_t>            _timeouts;

    std::vector<eomc_impedanceParameters>  _impedance_params;   /** impedance parameters */ // TODO doubled!!! optimize using just one of the 2!!!
    eomc_impedanceLimits                   *_impedance_limits;  /** impedancel imits */
    eOmc_impedance_t                       *_cacheImpedance;    /* cache impedance value to split up the 2 sets */


    eomcParser_pidInfo      *_ppids;
    eomcParser_pidInfo      *_vpids;
    eomcParser_trqPidInfo   *_tpids;
    eomcParser_pidInfo      *_cpids;

    SpeedEstimationParameters *_estim_params;   /** parameters for speed/acceleration estimation */

    int *_axisMap;                              /** axies map*/
    std::vector<eomc_axisInfo_t> _axesInfo;


    double *_newtonsToSensor;                   /** Newtons to force sensor units conversion factors */
    bool  *checking_motiondone;                 /* flag telling if I'm already waiting for motion done */
    #define MAX_POSITION_MOVE_INTERVAL 0.080
    double *_last_position_move_time;           /** time stamp for last received position move command*/





    //behaviour flags
    bool        _useRawEncoderData;              /** if true than do not use calibration data */
    bool        _pwmIsLimited;                  /** set to true if pwm is limited */


     torqueControlHelper    *_torqueControlHelper;


    // debug purpose

#ifdef VERIFY_ROP_SETIMPEDANCE
    uint32_t *impedanceSignature;
#endif

#ifdef VERIFY_ROP_SETPOSITIONRAW
    uint32_t *refRawSignature;
    bool        *sendingDirects;
#endif


    // basic knowledge of my joints
    int   _njoints;                             // Number of joints handled by this EMS; this values will be extracted by the config file

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

    uint16_t        NVnumber;       // keep if useful to store, otherwise can be removed. It is used to pass the total number of this EP to the requestqueue





private:

    bool askRemoteValue(eOprotID32_t id32, void* value, uint16_t& size);
    bool checkRemoteControlModeStatus(int joint, int target_mode);

    bool dealloc();
    bool isEpManagedByBoard();

    bool convertPosPid(eomcParser_pidInfo myPidInfo[]);
    bool convertTrqPid(eomcParser_trqPidInfo myPidInfo[]);

    bool verifyUserControlLawConsistencyInJointSet(eomcParser_pidInfo *ipdInfo);
    bool verifyUserControlLawConsistencyInJointSet(eomcParser_trqPidInfo *pidInfo);
    bool verifyTorquePidshasSameUnitTypes(GenericControlUnitsType_t &unittype);
    bool verifyUseMotorSpeedFbkInJointSet(int useMotorSpeedFbk []);
    bool updatedJointsetsCfgWithControlInfo(void);
    bool saveCouplingsData(void);
    bool updatedJointsetsCfg(int joint, eOmc_pidoutputtype_t pidoutputtype);
    void debugUtil_printJointsetInfo(void);
    void debugUtil_printControlLaws(void);

    bool isTorqueControlEnabled(int joint);
    bool isVelocityControlEnabled(int joint);

    bool interactionModeStatusConvert_embObj2yarp(eOenum08_t embObjMode, int &vocabOut);
    bool interactionModeCommandConvert_yarp2embObj(int vocabMode, eOenum08_t &embOut);

    bool controlModeCommandConvert_yarp2embObj(int vocabMode, eOenum08_t &embOut);
    int  controlModeCommandConvert_embObj2yarp(eOmc_controlmode_command_t embObjMode);

    bool controlModeStatusConvert_yarp2embObj(int vocabMode, eOmc_controlmode_t &embOut);
    int  controlModeStatusConvert_embObj2yarp(eOenum08_t embObjMode);

    eOmc_pidoutputtype_t pidOutputTypeConver_eomc2fw(PidAlgorithmType_t controlLaw); //maybe a day we convert from yarp to fw!

    void copyPid_iCub2eo(const Pid *in, eOmc_PID_t *out);
    void copyPid_eo2iCub(eOmc_PID_t *in, Pid *out);

    //bool pidsAreEquals(Pid &pid1, Pid &pid2);

    bool EncoderType_iCub2eo(const string* in, uint8_t *out);
    bool EncoderType_eo2iCub(const uint8_t *in, string* out);

    // eOmn_serv_type_t getMcServiceType(void);

    bool iNeedCouplingsInfo(void); //the device needs coupling info if it manages joints controlled by 2foc and mc4plus.
    bool iMange2focBoards(void);

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


private:

    int fromConfig_NumOfJoints(yarp::os::Searchable &config);
    bool fromConfig_getGeneralInfo(yarp::os::Searchable &config); //get general info: useRawEncoderData, useLiitedPwm, etc....
    bool fromConfig_Step2(yarp::os::Searchable &config);
    bool fromConfig_readServiceCfg(yarp::os::Searchable &config);


public:

    embObjMotionControl();
    ~embObjMotionControl();

    Semaphore           semaphore;
    eoRequestsQueue     *requestQueue;      // it contains the list of requests done to the remote board


    void cleanup(void);

    // Device Driver
    virtual bool open(yarp::os::Searchable &par);
    virtual bool close();
    bool fromConfig(yarp::os::Searchable &config);

    virtual bool initialised();
    virtual iethresType_t type();
    virtual bool update(eOprotID32_t id32, double timestamp, void *rxdata);
    virtual eoThreadFifo * getFifo(uint32_t variableProgNum);
    virtual eoThreadEntry *getThreadTable(int  threadId);

    eoThreadEntry *appendWaitRequest(int j, uint32_t protoid);

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
    virtual bool setPositionModeRaw(int j);
    virtual bool setVelocityModeRaw(int j);
    virtual bool setTorqueModeRaw(int j);
    virtual bool setImpedancePositionModeRaw(int j);
    virtual bool setImpedanceVelocityModeRaw(int j);
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
    virtual int getState(int ch);
    virtual int getChannels();
    virtual bool updateMeasure(yarp::sig::Vector &fTorques);
    virtual bool updateMeasure(int j, double &fTorque);

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
    virtual bool setTorquePidRaw(int j, const Pid &pid);
    virtual bool setTorquePidsRaw(const Pid *pids);
    virtual bool setTorqueErrorLimitRaw(int j, double limit);
    virtual bool setTorqueErrorLimitsRaw(const double *limits);
    virtual bool getTorqueErrorRaw(int j, double *err);
    virtual bool getTorqueErrorsRaw(double *errs);
    virtual bool getTorquePidOutputRaw(int j, double *out);
    virtual bool getTorquePidOutputsRaw(double *outs);
    virtual bool getTorquePidRaw(int j, Pid *pid);
    virtual bool getTorquePidsRaw(Pid *pids);
    virtual bool getTorqueErrorLimitRaw(int j, double *limit);
    virtual bool getTorqueErrorLimitsRaw(double *limits);
    virtual bool resetTorquePidRaw(int j);
    virtual bool disableTorquePidRaw(int j);
    virtual bool enableTorquePidRaw(int j);
    virtual bool setTorqueOffsetRaw(int j, double v);
    virtual bool getMotorTorqueParamsRaw(int j, MotorTorqueParameters *params);
    virtual bool setMotorTorqueParamsRaw(int j, const MotorTorqueParameters params);
    int32_t getRefSpeedInTbl(uint8_t boardNum, int j, eOmeas_position_t pos);

    // IVelocityControl2
    virtual bool velocityMoveRaw(const int n_joint, const int *joints, const double *spds);
    virtual bool setVelPidRaw(int j, const Pid &pid);
    virtual bool setVelPidsRaw(const Pid *pids);
    virtual bool getVelPidRaw(int j, Pid *pid);
    virtual bool getVelPidsRaw(Pid *pids);
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
};

#endif // include guard

