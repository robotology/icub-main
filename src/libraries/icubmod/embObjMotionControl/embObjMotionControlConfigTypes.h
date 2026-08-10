// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author: auto-generated
 * email: robotology-superbuild
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

#ifndef __embObjMotionControlConfigTypesh__
#define __embObjMotionControlConfigTypesh__

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <yarp/dev/PidEnums.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/ControlBoardPid.h>
#include <yarp/dev/ControlBoardHelper.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>

#include "EoMotionControl.h"
#include "serviceParser.h"

namespace yarp {
    namespace dev  {
        namespace eomc {
            namespace configType {

typedef struct
{
    int velocity_ref;
    int current_ref;
    int pwm_ref;
    int torque_ref;
    int torque_fbk;
} timeouts_t;


class PidInfo
{
public:

    yarp::dev::Pid pid;
    yarp::dev::PidFeedbackUnitsEnum fbk_PidUnits;
    yarp::dev::PidOutputUnitsEnum   out_PidUnits;
    eOmc_ctrl_out_type_t            out_type;
    std::string                     controlLaw;
    std::string                     usernamePidSelected;
    bool                            enabled;

    PidInfo()
    {
        enabled = false;
        out_type = eomc_ctrl_out_type_n_a;
        fbk_PidUnits = yarp::dev::PidFeedbackUnitsEnum::RAW_MACHINE_UNITS;
        out_PidUnits = yarp::dev::PidOutputUnitsEnum::RAW_MACHINE_UNITS;
    }
    virtual ~PidInfo() = default; //this make the class polymorphic and allows dynamic_cast, which is used in the parser to check if the pid is a TrqPidInfo or a simple PidInfo    

    void print() const
    {
        yInfo("PidInfo:");
        yInfo("  pid: [kp=%f, ki=%f, kd=%f, max_int=%f, max_output=%f, scale=%f, offset=%f, stiction_up_val=%f, stiction_down_val=%f, kff=%f]",
              pid.kp, pid.ki, pid.kd, pid.max_int, pid.max_output, pid.scale, pid.offset, pid.stiction_up_val, pid.stiction_down_val, pid.kff);

        // Convert enums to string
        auto fbkUnitsToString = [](yarp::dev::PidFeedbackUnitsEnum e) -> std::string {
            switch(e) {
                case yarp::dev::PidFeedbackUnitsEnum::RAW_MACHINE_UNITS : return "RAW_MACHINE_UNITS";
                case yarp::dev::PidFeedbackUnitsEnum::METRIC : return "METRIC";
                
                default: return "UNKNOWN";
            }
        };
        auto outUnitsToString = [](yarp::dev::PidOutputUnitsEnum e) -> std::string {
            switch(e) {
                case yarp::dev::PidOutputUnitsEnum::RAW_MACHINE_UNITS: return "RAW_MACHINE_UNITS";
                case yarp::dev::PidOutputUnitsEnum::POSITION_METRIC : return "POSITION_METRIC";
                case yarp::dev::PidOutputUnitsEnum::VELOCITY_METRIC : return "VELOCITY_METRIC";
                case yarp::dev::PidOutputUnitsEnum::DUTYCYCLE_PWM_PERCENT : return "DUTYCYCLE_PWM_PERCENT";
                case yarp::dev::PidOutputUnitsEnum::TORQUE_METRIC : return "TORQUE_METRIC";
                case yarp::dev::PidOutputUnitsEnum::CURRENT_METRIC : return "CURRENT_METRIC";
                default: return "UNKNOWN";
            }
        };
        auto outTypeToString = [](eOmc_ctrl_out_type_t e) -> std::string {
            switch(e) {
                case eomc_ctrl_out_type_n_a: return "n/a";
                case eomc_ctrl_out_type_pwm: return "pwm";
                case eomc_ctrl_out_type_cur: return "cur";
                case eomc_ctrl_out_type_vel: return "vel_pwm";
                case eomc_ctrl_out_type_vel_cur: return "vel_cur";
                default: return "UNKNOWN";
            }
        };

        yInfo("  fbk_PidUnits: %s", fbkUnitsToString(fbk_PidUnits).c_str());
        yInfo("  out_PidUnits: %s", outUnitsToString(out_PidUnits).c_str());
        yInfo("  out_type: %s", outTypeToString(out_type).c_str());
        yInfo("  usernamePidSelected: %s", usernamePidSelected.c_str());
        yInfo("  enabled: %s", enabled ? "true" : "false");
    }

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

// Kalman Filter default values
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
    double         hardware;
    double         warning;
} temperatureLimits_t; // limits expressed as raw values after conversion is applied

} // namespace yarp::dev::eomc::configType

using configType::PidInfo;
using configType::TrqPidInfo;
using configType::timeouts_t;
using configType::temperatureLimits_t;
using configType::kalmanFilterParams_t;

typedef struct
{
    eOmc_encoder_t  type;                 /** joint encoder type*/
    double          tolerance;            /** Num of error bits passable for joint encoder */
    int             resolution;
} encoder_t;

typedef struct
{
    bool verbosewhenok;         /** its value depends on environment variable "ETH_VERBOSEWHENOK" */
    bool useRawEncoderData;     /** if true than do not use calibration data */
    bool pwmIsLimited;          /** set to true if pwm is limited */
} behaviour_flags_t;

typedef struct
{
    bool enableSkipRecalibration;   /** if true, the joint will not be recalibrated when the yri is restarted */
} maintenanceModeCfg_t;

typedef struct
{
    std::vector<PidInfo>    trj;
    std::vector<PidInfo>    mix;
    std::vector<PidInfo>    dir_pos;
    std::vector<PidInfo>    dir_vel;
    std::vector<TrqPidInfo> trq;
    std::vector<PidInfo>    cur;
    std::vector<PidInfo>    vel;
} PidControllers_t;

typedef enum
{
    motor_temperature_sensor_pt100       = 0,
    motor_temperature_sensor_pt1000      = 1,
    motor_temperature_sensor_none        = 255
} motor_temperatureSensorTypeEnum_t;

class ITemperatureSensor
{
public:
    virtual ~ITemperatureSensor() = default;
    virtual double convertTempCelsiusToRaw(const double temperature) = 0;
    virtual double convertRawToTempCelsius(const double temperature) = 0;
    virtual motor_temperatureSensorTypeEnum_t getType() = 0;
};

class TemperatureSensorPT100 : public ITemperatureSensor
{
private:
    int _r_1;
    int _r_2;
    int _r_3;
    double _ptc_offset;
    double _ptc_gradient;
    double _pga_gain;
    int _vcc;
    double _resolution_pga;
    double _resolution_tdb;
    double _half_bridge_resistor_coeff;
    double _first_res_coeff;
    double _second_res_coeff;

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
        _first_res_coeff = _r_1*_r_2 + _r_1*_r_3 + _ptc_offset*_r_2 + _ptc_offset*_r_3;
        _second_res_coeff = _r_3*_r_1 - _r_2*_ptc_offset;
    }

    TemperatureSensorPT100(const TemperatureSensorPT100& other) = default;
    TemperatureSensorPT100& operator=(const TemperatureSensorPT100& other) = default;
    TemperatureSensorPT100(TemperatureSensorPT100&& other) = default;
    TemperatureSensorPT100& operator=(TemperatureSensorPT100&& other) = default;
    ~TemperatureSensorPT100() = default;

    virtual double convertTempCelsiusToRaw(const double temperature) override
    {
        double res = 0;
        double tmp = (((_ptc_offset + _ptc_gradient * temperature) / ((double)_r_1 + (_ptc_offset + _ptc_gradient * temperature))) - _half_bridge_resistor_coeff) * (double)_vcc;
        res = (_resolution_tdb + 1) * ((_pga_gain * tmp) / _resolution_pga);
        yDebug("Converted temperature limit to raw value:%f", res);
        return res;
    }

    virtual double convertRawToTempCelsius(const double temperature) override
    {
        double res = 0;
        double tmp = temperature * ((_resolution_pga) / (_pga_gain * _vcc * (_resolution_tdb + 1)));
        double den = _ptc_gradient * (_r_2 - _r_2*tmp - _r_3*tmp);
        res = (tmp * (_first_res_coeff) / den) + ((_second_res_coeff) / den);
        return res;
    }

    virtual motor_temperatureSensorTypeEnum_t getType() override
    {
        return motor_temperature_sensor_pt100;
    }
};

class TemperatureSensorPT1000 : public ITemperatureSensor
{
private:
    int _r_1;
    int _r_2;
    int _r_3;
    double _ptc_offset;
    double _ptc_gradient;
    double _pga_gain;
    int _vcc;
    double _resolution_pga;
    double _resolution_tdb;
    double _half_bridge_resistor_coeff;
    double _first_res_coeff;
    double _second_res_coeff;

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
        _first_res_coeff = _r_1*_r_2 + _r_1*_r_3 + _ptc_offset*_r_2 + _ptc_offset*_r_3;
        _second_res_coeff = _r_3*_r_1 - _r_2*_ptc_offset;
    }

    TemperatureSensorPT1000(const TemperatureSensorPT1000& other) = default;
    TemperatureSensorPT1000& operator=(const TemperatureSensorPT1000& other) = default;
    TemperatureSensorPT1000(TemperatureSensorPT1000&& other) = default;
    TemperatureSensorPT1000& operator=(TemperatureSensorPT1000&& other) = default;
    ~TemperatureSensorPT1000() = default;

    virtual double convertTempCelsiusToRaw(const double temperature) override
    {
        double res = 0;
        double tmp = (((_ptc_offset + _ptc_gradient * temperature) / ((double)_r_1 + (_ptc_offset + _ptc_gradient * temperature))) - _half_bridge_resistor_coeff) * (double)_vcc;
        res = (_resolution_tdb + 1) * ((_pga_gain * tmp) / _resolution_pga);
        yDebug("Converted temperature limit to raw value:%f", res);
        return res;
    }

    virtual double convertRawToTempCelsius(const double temperature) override
    {
        double res = 0;
        double tmp = temperature * ((_resolution_pga) / (_pga_gain * _vcc * (_resolution_tdb + 1)));
        double den = _ptc_gradient * (_r_2 - _r_2*tmp - _r_3*tmp);
        res = (tmp * (_first_res_coeff) / den) + ((_second_res_coeff) / den);
        return res;
    }

    virtual motor_temperatureSensorTypeEnum_t getType() override
    {
        return motor_temperature_sensor_pt1000;
    }
};

class TemperatureSensorNONE : public ITemperatureSensor
{
public:
    TemperatureSensorNONE() = default;
    ~TemperatureSensorNONE() = default;

    virtual double convertTempCelsiusToRaw(const double temperature) override
    {
        (void)temperature;
        return 0;
    }

    virtual double convertRawToTempCelsius(const double temperature) override
    {
        (void)temperature;
        return 0;
    }

    virtual motor_temperatureSensorTypeEnum_t getType() override
    {
        return motor_temperature_sensor_none;
    }
};

typedef struct
{
    float kbemf;
    bool hasHallSensor;
    bool hasTempSensor;
    bool hasRotorEncoder;
    bool hasRotorEncoderIndex;
    int  rotorIndexOffset;
    int  motorPoles;
    bool hasSpeedEncoder;
    bool verbose;
} focBasedSpecificInfo_t;

class JointsSet
{
public:
    int id;
    std::vector<int> joints;
    eOmc_jointset_configuration_t cfg;

    JointsSet(int num=0)
    {
        id = num;
        joints.resize(0);
        cfg.candotorquecontrol = 0;
        cfg.usespeedfeedbackfrommotors = 0;
        cfg.pidoutputtype = eomc_pidoutputtype_unknown;
        cfg.dummy = 0;
        cfg.constraints.type = eomc_jsetconstraint_unknown;
        cfg.constraints.param1 = 0;
        cfg.constraints.param2 = 0;
    }

    int getNumberofJoints(void) { return joints.size(); }
    eOmc_jointset_configuration_t* getConfiguration(void) { return &cfg; }
    void setUseSpeedFeedbackFromMotors(bool flag) { cfg.usespeedfeedbackfrommotors = flag; }
    void setPidOutputType(eOmc_pidoutputtype_t type) { cfg.pidoutputtype = type; }
    void setCanDoTorqueControl(bool flag) { cfg.candotorquecontrol = flag; }
    void dumpdata();
};

typedef struct
{
    double nominalCurrent;
    double peakCurrent;
    double overloadCurrent;
} motorCurrentLimits_t;

typedef struct
{
    double posMin;
    double posMax;
    double posHwMax;
    double posHwMin;
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
    std::vector<double> matrixJ2M;
    std::vector<double> matrixM2J;
    std::vector<double> matrixE2J;
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
    double Km;
    double Kw;
    double S0;
    double S1;
    double Vth;
    double Fc_pos;
    double Fc_neg;
    double Fs_pos;
    double Fs_neg;
} lugreParameters_t;

struct ParsedConfigData
{
    behaviour_flags_t behFlags{};
    servConfigMC_t serviceConfig{};
    maintenanceModeCfg_t maintenanceModeCfg{};

    std::vector<double> gearbox_M2J;
    std::vector<double> gearbox_E2J;
    std::vector<double> deadzone;

    std::vector<kalmanFilterParams_t> kalman_params;
    std::vector<std::unique_ptr<ITemperatureSensor>> temperatureSensorsVector;
    std::vector<focBasedSpecificInfo_t> foc_based_info;

    std::vector<encoder_t> jointEncs;
    std::vector<encoder_t> motorEncs;

    std::vector<rotorLimits_t> rotorsLimits;
    std::vector<jointLimits_t> jointsLimits;
    std::vector<motorCurrentLimits_t> currentLimits;
    std::vector<temperatureLimits_t> temperatureLimits;

    couplingInfo_t couplingInfo{};
    std::vector<JointsSet> jsets;
    std::vector<int> joint2set;
    std::vector<timeouts_t> timeouts;

    std::vector<impedanceParameters_t> impedance_params;
    std::vector<lugreParameters_t> lugre_params;
    std::vector<impedanceLimits_t> impedance_limits;

    PidControllers_t pidControllers;

    std::vector<int> axisMap;
    std::vector<axisInfo_t> axesInfo;

    void resize(const int njoints)
    {
        gearbox_M2J.resize(njoints);
        gearbox_E2J.resize(njoints);
        deadzone.resize(njoints);

        kalman_params.resize(njoints);
        temperatureSensorsVector.resize(njoints);
        foc_based_info.resize(njoints);

        jointEncs.resize(njoints);
        motorEncs.resize(njoints);

        rotorsLimits.resize(njoints);
        jointsLimits.resize(njoints);
        currentLimits.resize(njoints);
        temperatureLimits.resize(njoints);

        jsets.resize(njoints);
        joint2set.resize(njoints);
        timeouts.resize(njoints);

        impedance_params.resize(njoints);
        lugre_params.resize(njoints);
        impedance_limits.resize(njoints);

        axisMap.resize(njoints);
        axesInfo.resize(njoints);

        pidControllers.trj.resize(njoints);
        pidControllers.dir_pos.resize(njoints);
        pidControllers.dir_vel.resize(njoints);
        pidControllers.trq.resize(njoints);
        pidControllers.cur.resize(njoints);
        pidControllers.vel.resize(njoints);
        pidControllers.mix.resize(njoints);
    }

    void clearPidControllers()
    {
        pidControllers.trj.clear();
        pidControllers.dir_pos.clear();
        pidControllers.dir_vel.clear();
        pidControllers.trq.clear();
        pidControllers.cur.clear();
        pidControllers.vel.clear();
        pidControllers.mix.clear();
    }
};


        } // namespace eomc
    } // namespace dev
} // namespace yarp


#endif