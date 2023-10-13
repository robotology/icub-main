// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/* Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Valentina Gaggero
 * email: valentina.gaggero@iit.it
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

#ifndef __mcParserh__
#define __mcParserh__



#include <string>
#include <vector>
#include <array>
#include <map>
#include <memory>

//  Yarp stuff
#include <yarp/os/Bottle.h>
//#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/ControlBoardPid.h>
#include <yarp/dev/ControlBoardHelper.h>

#include <yarp/dev/PidEnums.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>

#include "EoMotionControl.h"
#include <yarp/os/LogStream.h>


// - public #define  --------------------------------------------------------------------------------------------------


namespace yarp {
    namespace dev  {
        namespace eomc {


//typedef enum
//{
//    PidAlgo_simple = 0,
//    PIdAlgo_velocityInnerLoop = 1,
//    PidAlgo_currentInnerLoop =2
//} PidAlgorithmType_t;

typedef enum
{
    motor_temperature_sensor_pt100       = 0,
    motor_temperature_sensor_pt1000      = 1,
    motor_temperature_sensor_none        = 255
} motor_temperatureSensorTypeEnum_t;

class ITemperatureSensor
{

public:

    virtual double convertTempCelsiusToRaw(const double temperature) = 0;

    virtual double convertRawToTempCelsius(const double temperature) = 0;
};

class TemperatureSensorPT100 : public ITemperatureSensor
{
private:
    // resistors of the voltage divider bridge
    int _r_1;
    int _r_2;
    int _r_3;
    

    double _ptc_offset;    // offset of the temperature sensor line
    double _ptc_gradient;  // slope/gradient of the temperature sensor line
    double _pga_gain;      // ADC gain set for the tdb (temperature detection board)

    int _vcc;               // vcc that enters the voltage divider bridge
    double _resolution_pga; // resolution of the internal pga of the tdb
    double _resolution_tdb; // resolution used for the raw value for the output of the tdb

    double _half_bridge_resistor_coeff;

    bool isConfigured = false;

public:

    TemperatureSensorPT100()
    {
        _r_1 = 4700;
        _r_2 = 4700;
        _r_3 = 100; 

        _ptc_offset = 100.0;
        _ptc_gradient = 0.3851;
        _pga_gain = 2;
        _vcc = 5;
        _resolution_pga = 2.048;
        _resolution_tdb = 32767; 

        _half_bridge_resistor_coeff = (double)_r_3 / (double)(_r_2 + _r_3);
        isConfigured = true;
    }

    TemperatureSensorPT100(const TemperatureSensorPT100& other) = default; // const copy constructor
    TemperatureSensorPT100& operator=(const TemperatureSensorPT100& other) = default; // const copy assignment operator
    TemperatureSensorPT100(TemperatureSensorPT100&& other) = default; // move constructor
    TemperatureSensorPT100& operator=(TemperatureSensorPT100&& other) = default; // move assignment operator

    ~TemperatureSensorPT100() = default; // destructor

    virtual double convertTempCelsiusToRaw(const double temperature) override
    {
        double res = 0;
        if (!isConfigured)
        {
            yError("Cannot proceed, class paramters not confgured");
            return res;
        }
       	
        double tmp = (( (_ptc_offset + _ptc_gradient * temperature) / ((double)_r_1 + (_ptc_offset + _ptc_gradient * temperature))) - _half_bridge_resistor_coeff) * (double)_vcc;
        res = (_resolution_tdb + 1) * ((_pga_gain * tmp) / _resolution_pga);
        
	    yDebug("Converted temperature limit to raw value:%f", res);
        return res;
    }

    virtual double convertRawToTempCelsius(const double temperature) override
    {
        double res = 0;
        if (!isConfigured)
        {
            yError("Cannot proceed, class paramters not confgured");
            return res;
        }
        
        double tmp = temperature * ((_resolution_pga) / (_pga_gain * _vcc * (_resolution_tdb + 1)));
        double den = _ptc_gradient * (_r_2 - _r_2*tmp - _r_3*tmp);
        res = (tmp * (_r_1*_r_2 + _r_1*_r_3 + _ptc_offset*_r_2 + _ptc_offset*_r_3) / den) + ((_r_3*_r_1 - _r_2*_ptc_offset) / den);
        
        
        //yDebug("Converted temperature  to Celsius degree value:%f", res);
        return res;
    }

};


class TemperatureSensorPT1000 : public ITemperatureSensor
{

private:
    // resistors of the voltage divider bridge
    int _r_1;
    int _r_2;
    int _r_3;
    
     double _ptc_offset;    // offset of the temperature sensor line
    double _ptc_gradient;  // slope/gradient of the temperature sensor line
    double _pga_gain;      // ADC gain set for the tdb (temperature detection board)

    int _vcc;               // vcc that enters the voltage divider bridge
    double _resolution_pga; // resolution of the internal pga of the tdb
    double _resolution_tdb; // resolution used for the raw value for the output of the tdb
    
    double _half_bridge_resistor_coeff;
    
    bool isConfigured = false;

public:

    TemperatureSensorPT1000()
    {
        _r_1 = 4700;
        _r_2 = 4700;
        _r_3 = 1000;

        _ptc_offset = 1000;
        _ptc_gradient = 3.851;
        _pga_gain = 2;
        _vcc = 5;
        _resolution_pga = 2.048;
        _resolution_tdb = 32767;
        
        _half_bridge_resistor_coeff = (double)_r_3 / (double)(_r_2 + _r_3);

        isConfigured = true;
    }

    TemperatureSensorPT1000(const TemperatureSensorPT1000& other) = default; // const copy constructor
    TemperatureSensorPT1000& operator=(const TemperatureSensorPT1000& other) = default; // const copy assignment operator
    TemperatureSensorPT1000(TemperatureSensorPT1000&& other) = default; // move constructor
    TemperatureSensorPT1000& operator=(TemperatureSensorPT1000&& other) = default; // move assignment operator

    ~TemperatureSensorPT1000() = default; // destructor

    virtual double convertTempCelsiusToRaw(const double temperature) override
    {
        double res = 0;
        if (!isConfigured)
        {
            yError("Cannot proceed, class paramters not confgured");
            return res;
        }
        
        double tmp = (( (_ptc_offset + _ptc_gradient * temperature) / ((double)_r_1 + (_ptc_offset + _ptc_gradient * temperature))) - _half_bridge_resistor_coeff) * (double)_vcc;
        res = (_resolution_tdb + 1) * ((_pga_gain * tmp) / _resolution_pga);
        
	    yDebug("Converted temperature limit to raw value:%f", res);
        return res;
    }

    virtual double convertRawToTempCelsius(const double temperature) override
    {
        double res = 0;
        if (!isConfigured)
        {
            yError("Cannot proceed, class paramters not confgured");
            return res;
        }
        
        double tmp = temperature * ((_resolution_pga) / (_pga_gain * _vcc * (_resolution_tdb + 1)));
        double den = _ptc_gradient * (_r_2 - _r_2*tmp - _r_3*tmp);
        res = (tmp * (_r_1*_r_2 + _r_1*_r_3 + _ptc_offset*_r_2 + _ptc_offset*_r_3) / den) + ((_r_3*_r_1 - _r_2*_ptc_offset) / den);
        
	    //yDebug("Converted temperature  to Celsius degree value:%f", res);
        
	    return res;
    }
};


class TemperatureSensorNONE : public ITemperatureSensor
{

public:

    TemperatureSensorNONE()
    {
        yError("Private varibales NOT DEFINED for class TemperatureSensorNONE");
    }

    ~TemperatureSensorNONE() = default;
    
    virtual double convertTempCelsiusToRaw(const double temperature) override
    {
        yError("convertTempCelsiusToRaw METHOD, NOT IMPLEMENTED for class TemperatureSensorNONE");
        return 0;
    }

    virtual double convertRawToTempCelsius(const double temperature) override
    {
        yError("convertRawToTempCelsius METHOD, NOT IMPLEMENTED for class TemperatureSensorNONE");
        return 0;
    }
};


class Pid_Algorithm
{
public:
    //PidAlgorithmType_t type;

    yarp::dev::PidFeedbackUnitsEnum fbk_PidUnits;
    yarp::dev::PidOutputUnitsEnum   out_PidUnits;
    eOmc_ctrl_out_type_t out_type;

    virtual ~Pid_Algorithm(){}

    eOmc_ctrl_out_type_t getOutputType() { return out_type; }

    void setUnits(yarp::dev::PidFeedbackUnitsEnum fu, yarp::dev::PidOutputUnitsEnum ou)
    {
        fbk_PidUnits = fu;
        out_PidUnits = ou;
    }

    virtual yarp::dev::Pid getPID(int joint) = 0;
};


class Pid_Algorithm_simple: public Pid_Algorithm
{
public:
    yarp::dev::Pid *pid;
    
    Pid_Algorithm_simple(int nj, eOmc_ctrl_out_type_t ot)
    {
        out_type = ot;

        pid = new yarp::dev::Pid[nj];
    };

    ~Pid_Algorithm_simple()
    {
        if (pid) delete[] pid;
    };

    yarp::dev::Pid getPID(int joint) override
    {
        return pid[joint];
    }
};

/*
class PidAlgorithm_VelocityInnerLoop: public Pid_Algorithm
{
public:
    yarp::dev::Pid *extPid; //pos, trq, velocity
    //yarp::dev::Pid *innerVelPid;
    PidAlgorithm_VelocityInnerLoop(int nj)
    {
        extPid = new yarp::dev::Pid[nj];
        //innerVelPid = new yarp::dev::Pid[nj];
    };
    ~PidAlgorithm_VelocityInnerLoop()
    {
        if(extPid)
        {
            delete[]extPid;
        }
        //if(innerVelPid)
        //{
        //    delete[]innerVelPid;
        //}
    }

    eOmc_ctrl_out_type_t getOutputType() override
    {
        return eomc_ctrl_out_type_vel;
    }
};


class PidAlgorithm_CurrentInnerLoop: public Pid_Algorithm
{
    public:
    yarp::dev::Pid *extPid;
    yarp::dev::Pid *innerCurrLoop;
    PidAlgorithm_CurrentInnerLoop(int nj)
    {
        extPid = allocAndCheck<yarp::dev::Pid>(nj);
        innerCurrLoop = allocAndCheck<yarp::dev::Pid>(nj);
    };
    ~PidAlgorithm_CurrentInnerLoop()
    {
        checkAndDestroy(extPid);
        checkAndDestroy(innerCurrLoop);
    }

    eOmc_ctrl_out_type_t getOutputType() override
    {
        return eomc_ctrl_out_type_cur;
    }
};
*/

class PidInfo
{
public:

    yarp::dev::Pid pid;
    yarp::dev::PidFeedbackUnitsEnum fbk_PidUnits;
    yarp::dev::PidOutputUnitsEnum   out_PidUnits;
    eOmc_ctrl_out_type_t            out_type;

    //PidAlgorithmType_t controlLaw;

    std::string usernamePidSelected;
    bool enabled;

    PidInfo()//:usernamePidSelected("none")
    {
        enabled = false;
        //controlLaw = PidAlgo_simple;
        
        out_type = eomc_ctrl_out_type_n_a;
        fbk_PidUnits = yarp::dev::PidFeedbackUnitsEnum::RAW_MACHINE_UNITS;
        out_PidUnits = yarp::dev::PidOutputUnitsEnum::RAW_MACHINE_UNITS;
    }
    ~PidInfo()
    {
        //delete (usernamePidSelected);
    }

    //void dumpdata(void);

};

class TrqPidInfo : public PidInfo
{
public:
    double kbemf;                             /** back-emf compensation parameter */
    double ktau;                              /** motor torque constant */
    double viscousPos;
    double viscousNeg;
    double coulombPos;
    double coulombNeg;
    double velocityThres;
    int    filterType;
};

typedef struct
{
    bool hasHallSensor;
    bool hasTempSensor;
    motor_temperatureSensorTypeEnum_t temperatureSensorType;  // uses a typedef enum
    bool hasRotorEncoder;
    bool hasRotorEncoderIndex;
    int  rotorIndexOffset;
    int  motorPoles;
    bool hasSpeedEncoder ; //facoltativo
    bool verbose;
} focBasedSpecificInfo_t;


class JointsSet
{
public:
    int           id; //num of set. it can be between 0 and max number of joint (_njoints)
    std::vector<int>   joints; //list of joints belongig to this set2joint

    eOmc_jointset_configuration_t cfg;

    JointsSet(int num=0)
    {
        id=num;
        joints.resize(0);
        cfg.candotorquecontrol=0;
        cfg.usespeedfeedbackfrommotors=0;
        cfg.pidoutputtype=eomc_pidoutputtype_unknown;
        cfg.dummy=0;
        cfg.constraints.type=eomc_jsetconstraint_unknown;
        cfg.constraints.param1=0;
        cfg.constraints.param2=0;
    }
public:
    int getNumberofJoints(void) {return (joints.size());}
    eOmc_jointset_configuration_t* getConfiguration(void) {return &cfg;}
    void setUseSpeedFeedbackFromMotors(bool flag){cfg.usespeedfeedbackfrommotors = flag;}
    void setPidOutputType(eOmc_pidoutputtype_t type){cfg.pidoutputtype = type;}
    void setCanDoTorqueControl(bool flag) {cfg.candotorquecontrol = flag;}
    void dumpdata();
};

typedef struct
{
    int velocity;
} timeouts_t;

typedef struct
{
    double nominalCurrent;
    double peakCurrent;
    double overloadCurrent;
} motorCurrentLimits_t;


typedef struct
{
    double posMin;                         /** user joint limits, max*/
    double posMax;                         /** user joint limits, min*/
    double posHwMax;                       /** hardaware joint limits, max */
    double posHwMin;                       /** hardaware joint limits, min */
    double velMax;
} jointLimits_t;

typedef struct
{
    double posMin;
    double posMax;
    double pwmMax;
} rotorLimits_t;


typedef struct
{
    std::vector<double>                  matrixJ2M;
    std::vector<double>                  matrixM2J;
    std::vector<double>                  matrixE2J;
} couplingInfo_t;

typedef struct
{
    int             mappedto;
    std::string     name;
    JointTypeEnum   type;
} axisInfo_t;

typedef struct
{
    double min_stiff;
    double max_stiff;
    double min_damp;
    double max_damp;
    double param_a;
    double param_b;
    double param_c;
} impedanceLimits_t;


typedef struct
{
    double stiffness;
    double damping;
    impedanceLimits_t limits;
} impedanceParameters_t;

typedef struct
{
    bool enabled;
    std::array<float32_t, 3> x0;
    std::array<float32_t, 3> Q;
    float32_t R;
    float32_t P0;
} kalmanFilterParams_t;


typedef struct
{
    double         hardwareTemperatureLimit;
    double         warningTemperatureLimit;
} temperatureLimits_t; // limits expressed as raw values after conversion is applied

//template <class T>

class Parser
{

private:
    int _njoints;
    std::string _boardname;
    bool _verbosewhenok;

    std::map<std::string, Pid_Algorithm*> minjerkAlgoMap;
    //std::map<std::string, Pid_Algorithm*> directAlgoMap;
    std::map<std::string, Pid_Algorithm*> torqueAlgoMap;

    //std::map<std::string, Pid_Algorithm*> currentAlgoMap;
    //std::map<std::string, Pid_Algorithm*> speedAlgoMap;

    std::vector<std::string> _positionControlLaw;
    std::vector<std::string> _velocityControlLaw;
    std::vector<std::string> _mixedControlLaw;
    //std::vector<std::string> _posDirectControlLaw;
    //std::vector<std::string> _velDirectControlLaw;
    std::vector<std::string> _torqueControlLaw;
    std::vector<std::string> _currentControlLaw;
    std::vector<std::string> _speedControlLaw;

    double *_kbemf;                             /** back-emf compensation parameter */
    double *_ktau;                              /** motor torque constant */
    int *_filterType;
    double *_viscousPos;
    double *_viscousNeg;
    double *_coulombPos;
    double *_coulombNeg;
    double *_velocityThres;





    //PID parsing functions
    bool parseControlsGroup(yarp::os::Searchable &config);
    
    bool parseSelectedPositionControl(yarp::os::Searchable &config);
    bool parseSelectedVelocityControl(yarp::os::Searchable &config);
    bool parseSelectedMixedControl(yarp::os::Searchable &config);
    //bool parseSelectedPosDirectControl(yarp::os::Searchable &config);
    //bool parseSelectedVelDirectControl(yarp::os::Searchable &config);
    bool parseSelectedTorqueControl(yarp::os::Searchable &config);

    bool parseSelectedCurrentPid(yarp::os::Searchable &config, bool pidisMandatory, PidInfo *pids);
    bool parseSelectedSpeedPid(yarp::os::Searchable &config, bool pidisMandatory, PidInfo *pids);

    bool parsePid_minJerk_outPwm(yarp::os::Bottle &b_pid, std::string controlLaw);
    bool parsePid_minJerk_outCur(yarp::os::Bottle &b_pid, std::string controlLaw);
    bool parsePid_minJerk_outVel(yarp::os::Bottle &b_pid, std::string controlLaw);

    //bool parsePid_direct_outPwm(yarp::os::Bottle &b_pid, std::string controlLaw);
    //bool parsePid_direct_outCur(yarp::os::Bottle &b_pid, std::string controlLaw);
    //bool parsePid_direct_outVel(yarp::os::Bottle &b_pid, std::string controlLaw);

    bool parsePid_torque_outPwm(yarp::os::Bottle &b_pid, std::string controlLaw);
    bool parsePid_torque_outCur(yarp::os::Bottle &b_pid, std::string controlLaw);
    bool parsePid_torque_outVel(yarp::os::Bottle &b_pid, std::string controlLaw);

    bool parsePidsGroup2FOC(yarp::os::Bottle& pidsGroup, Pid myPid[]);
    bool parsePidsGroupSimple(yarp::os::Bottle& pidsGroup, Pid myPid[]);
    bool parsePidsGroupExtended(yarp::os::Bottle& pidsGroup, Pid myPid[]);
    bool parsePidsGroupDeluxe(yarp::os::Bottle& pidsGroup, Pid myPid[]);

    bool parsePidsGroup(yarp::os::Bottle& pidsGroup, yarp::dev::Pid myPid[], std::string prefix);
    bool getCorrectPidForEachJoint(PidInfo *ppids/*, PidInfo *vpids*/, TrqPidInfo *tpids);
    bool parsePidUnitsType(yarp::os::Bottle &bPid, yarp::dev::PidFeedbackUnitsEnum  &fbk_pidunits, yarp::dev::PidOutputUnitsEnum& out_pidunits);

    bool checkJointTypes(PidInfo *pids, const std::string &pid_type);
    bool checkSinglePid(PidInfo &firstPid, PidInfo &currentPid, const int &firstjoint, const int &currentjoint, const std::string &pid_type);

    bool convert(std::string const &fromstring, eOmc_jsetconstraint_t &jsetconstraint, bool& formaterror);
    bool convert(yarp::os::Bottle &bottle, std::vector<double> &matrix, bool &formaterror, int targetsize);

    //general utils functions
    bool extractGroup(yarp::os::Bottle &input, yarp::os::Bottle &out, const std::string &key1, const std::string &txt, int size, bool mandatory=true);
    template <class T>
    bool checkAndSetVectorSize(std::vector<T> &vec, int size, const std::string &funcName)
    {
        if(size > (int)vec.capacity())
        {
            yError() << "embObjMC BOARD " << _boardname << " in " <<  funcName.c_str() << ": try to insert " << size << "element in vector with " << vec.capacity() << " elements";
            return false;
        }

        vec.resize(size);
        return true;
    }

    ///////// DEBUG FUNCTIONS
    void debugUtil_printControlLaws(void);


public:
    Parser(int numofjoints, std::string boardname);
    ~Parser();

    bool parsePids(yarp::os::Searchable &config, PidInfo *ppids/*, PidInfo *vpids*/, TrqPidInfo *tpids, PidInfo *cpids, PidInfo *spids, bool lowLevPidisMandatory);
    bool parseFocGroup(yarp::os::Searchable &config, focBasedSpecificInfo_t *foc_based_info, std::string groupName, std::vector<std::unique_ptr<eomc::ITemperatureSensor>>& temperatureSensorsVector);
    //bool parseCurrentPid(yarp::os::Searchable &config, PidInfo *cpids);//deprecated
    bool parseJointsetCfgGroup(yarp::os::Searchable &config, std::vector<JointsSet> &jsets, std::vector<int> &jointtoset);
    bool parseTimeoutsGroup(yarp::os::Searchable &config, std::vector<timeouts_t> &timeouts, int defaultVelocityTimeout);
    bool parseCurrentLimits(yarp::os::Searchable &config, std::vector<motorCurrentLimits_t> &currLimits);
    bool parseTemperatureLimits(yarp::os::Searchable &config, std::vector<temperatureLimits_t> &temperatureLimits);
    bool parseJointsLimits(yarp::os::Searchable &config, std::vector<jointLimits_t> &jointsLimits);
    bool parseRotorsLimits(yarp::os::Searchable &config, std::vector<rotorLimits_t> &rotorsLimits);
    bool parseCouplingInfo(yarp::os::Searchable &config, couplingInfo_t &couplingInfo);
    bool parseMotioncontrolVersion(yarp::os::Searchable &config, int &version);
    bool parseBehaviourFalgs(yarp::os::Searchable &config, bool &useRawEncoderData, bool  &pwmIsLimited );
    bool isVerboseEnabled(yarp::os::Searchable &config);
    bool parseAxisInfo(yarp::os::Searchable &config, int axisMap[], std::vector<axisInfo_t> &axisInfo);
    bool parseEncoderFactor(yarp::os::Searchable &config, double encoderFactor[]);
    bool parsefullscalePWM(yarp::os::Searchable &config, double dutycycleToPWM[]);
    bool parseAmpsToSensor(yarp::os::Searchable &config, double ampsToSensor[]);
    bool parseGearboxValues(yarp::os::Searchable &config, double gearbox_M2J[], double gearbox_E2J[]);
    bool parseMechanicalsFlags(yarp::os::Searchable &config, int useMotorSpeedFbk[]);
    bool parseImpedanceGroup(yarp::os::Searchable &config,std::vector<impedanceParameters_t> &impedance);
    bool parseDeadzoneValue(yarp::os::Searchable &config, double deadzone[], bool *found);
    bool parseKalmanFilterParams(yarp::os::Searchable &config, std::vector<kalmanFilterParams_t> &kalmanFilterParams);
};

}}}; //close namespaces

#endif // include guard
