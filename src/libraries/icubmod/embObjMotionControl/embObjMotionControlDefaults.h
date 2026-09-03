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

#ifndef __embObjMotionControlDefaultsh__
#define __embObjMotionControlDefaultsh__

#include <string>
#include <string_view>
#include "embObjMotionControlConfigTypes.h"

namespace yarp::dev::eomc {

/**
 * @brief Allowed parameter values (tokens) used for configuration file validation.
 *
 * This namespace contains the string constants representing the set of valid values
 * that configuration parameters can assume. They are used by the parser to validate
 * user-provided settings (e.g., control law type, feedback units, sensor type, axis type).
 * These are NOT default values; for fallback defaults see the @ref defaults namespace.
 */
namespace ParamValues {

static constexpr std::string_view NONE_STR = "NONE";

namespace controlLaw {
static constexpr std::string_view current    = "low_lev_current";
static constexpr std::string_view direct     = "direct";
static constexpr std::string_view trajectory = "minjerk";
static constexpr std::string_view torque     = "torque";
} // namespace controlLaw

namespace fbkUnits {
static constexpr std::string_view metric  = "metric_units";
static constexpr std::string_view machine = "machine_units";
} // namespace fbkUnits

// Only scale matters; the physical quantity is already encoded in outputType.
namespace outUnits {
static constexpr std::string_view metric  = "metric_units";
static constexpr std::string_view machine = "machine_units";
} // namespace outUnits

namespace temperatureSensor {
static constexpr std::string_view NONE  = NONE_STR;
static constexpr std::string_view PT100  = "PT100";
static constexpr std::string_view PT1000 = "PT1000";
} // namespace temperatureSensor

namespace axisType {
static constexpr std::string_view prismatic = "prismatic";
static constexpr std::string_view revolute  = "revolute";
} // namespace axisType

// Keys used in the CONTROLS config group to select a PID group per control mode.
namespace controlMode {
static constexpr std::string_view position       = "positionControl";
static constexpr std::string_view velocity       = "velocityControl";
static constexpr std::string_view mixed          = "mixedControl";
static constexpr std::string_view torque         = "torqueControl";
static constexpr std::string_view current        = "currentControl";
static constexpr std::string_view positionDirect = "positionDirect";
static constexpr std::string_view velocityDirect = "velocityDirect";
} // namespace controlMode

// Strings used in config "outputType" field to select the physical output quantity.
namespace outputType {
static constexpr std::string_view n_a      = "n_a";
static constexpr std::string_view pwm      = "pwm";
static constexpr std::string_view velocity = "velocity";
static constexpr std::string_view current  = "current";
} // namespace outputType

} // namespace ParamValues

/**
 * @brief Default (fallback) values for embObjMotionControl configuration parameters.
 *
 * This namespace contains the data structures and the singleton factory that hold
 * the default values used when a configuration parameter is missing or invalid.
 * For the set of allowed parameter value tokens, see the @ref paramValues namespace.
 */
namespace defaults {


/**
 * @brief Structure containing all default values used in embObjMotionControl 
 * and eomcParser classes
 */
typedef struct
{


    // FOC (Field Oriented Control) default values
    struct
    {
        bool verbose;               // verbose output - default: false (0)
        bool autoCalibration;       // auto calibration - default: false (0)
        float kbemf_foc;            // back-emf compensation for FOC - default: 0.0f
        std::string temperatureSensorType;  // temperature sensor type - default: "NONE"
    } focDefaults;

    // Timeouts defaults
    configType::timeouts_t timeoutsDefaults;

    // AMC FOC timeouts default values
    /**********************************************
     * Important note on AMC FOC timeouts defaults:
     * These defaults are used when the parser detects that the configuration file is for an AMC board
     * and the TIMEOUTS group is missing or incomplete. The default values are set to 300 ms to account for the additional latency typically observed in AMC boards compared to other types of boards (EMS).
     * now the timeout are mandatory, but this values are used to check if the user provided timeout are correct or not, in case of wrong value the default one will be used and a warning will be printed
     */
    configType::timeouts_t timeoutsDefaults_amcFoc;

    // Temperature limits default values
    configType::temperatureLimits_t temperatureLimitsDefaults;

    // Control parameters default values
    struct
    {
        double deadZone;            // dead zone value - default: not specified
        double pidScale;            // PID scale - default: 0.0
        double pidOffset;           // PID offset - default: 0.0
        double stictionUpVal;       // stiction up value - default: 0.0
        double stictionDownVal;     // stiction down value - default: 0.0
        
        // Deadzone parameters for different encoder types
        struct
        {
            double jointWithAEA;    // AEA encoder deadzone - default: 0.0494
            double jointWithAEA3;   // AEA3 encoder deadzone - default: 0.0 (TODO: fix if needed)
            double jointWithAMO;    // AMO encoder deadzone - default: 0.0055 (360 / 2^16)
            double jointWithAKSIM2; // AKSIM2 encoder deadzone - default: 0.00068 (360 / 2^19)
            double otherEncoder; // AKSIM2 encoder deadzone - default: 0.00068 (360 / 2^19)
        } deadzone;

        configType::kalmanFilterParams_t kalmanFilterParams;

        // Lugre friction model default values
        struct
        {
            double Km;              // Km parameter - default: -1.0
        } lugre;
    } controlParametersDefaults;

    configType::TrqPidInfo torquePidDefaults; // Torque PID default values

    double jointPosLimitsDelta; // default: 2.0. the positionLimits are reduced by this value when returned in the function "GetPositionLimits".
    
    // Minimum time interval (in seconds) between consecutive positionMove commands.
    // If commands arrive faster than this threshold, a performance warning is issued
    // to alert the user that a different control mode may be more appropriate.
    double maxPositionMoveInterval; // default = 0.080 seconds;

} embObjMotionControlDefaults_t;

/**
 * @brief Singleton factory class to create and manage default values with runtime configuration
 * This class ensures a single instance is shared across all objects in the application
 * All modifications to defaults are immediately visible to all objects accessing the singleton
 */
class DefaultsFactory
{
private:
    embObjMotionControlDefaults_t defaults;

    /**
     * @brief Private constructor - initializes all defaults
     * Only called once when the singleton is first instantiated
     */
    DefaultsFactory()
    {
        initializeDefaults();
    }

    // Delete copy constructor and assignment operator to prevent copies
    DefaultsFactory(const DefaultsFactory&) = delete;
    DefaultsFactory& operator=(const DefaultsFactory&) = delete;

public:
    /**
     * @brief Get the singleton instance (thread-safe in C++11+)
     * Uses Meyer's singleton pattern for automatic thread-safety
     * @return reference to the unique DefaultsFactory instance
     */
    static DefaultsFactory& getInstance()
    {
        static DefaultsFactory instance;  // Created only once, destroyed at program exit
        return instance;
    }

    /**
     * @brief Initialize all default values
     */
    void initializeDefaults()
    {
        // Torque PID defaults
        defaults.torquePidDefaults.kbemf = 0.0;
        defaults.torquePidDefaults.viscousPos = 0.0;
        defaults.torquePidDefaults.viscousNeg = 0.0;
        defaults.torquePidDefaults.coulombPos = 0.0;
        defaults.torquePidDefaults.coulombNeg = 0.0;
        defaults.torquePidDefaults.velocityThres = 0.0;
        
        // FOC defaults
        defaults.focDefaults.verbose = false;
        defaults.focDefaults.autoCalibration = false;
        defaults.focDefaults.kbemf_foc = 0.0f;
        
        // Timeouts defaults
        defaults.timeoutsDefaults.velocity_ref = 100;
        defaults.timeoutsDefaults.current_ref = 100;
        defaults.timeoutsDefaults.pwm_ref = 100;
        defaults.timeoutsDefaults.torque_ref = 100;
        defaults.timeoutsDefaults.torque_fbk = 100;
        
        // AMC FOC timeouts defaults
        defaults.timeoutsDefaults_amcFoc.velocity_ref = 300;
        defaults.timeoutsDefaults_amcFoc.current_ref = 300;
        defaults.timeoutsDefaults_amcFoc.pwm_ref = 300;
        defaults.timeoutsDefaults_amcFoc.torque_ref = 300;
        defaults.timeoutsDefaults_amcFoc.torque_fbk = 300;
        
        // Temperature limits defaults
        defaults.temperatureLimitsDefaults.hardware = 0.0;
        defaults.temperatureLimitsDefaults.warning = 0.0;
        
        // Control parameters defaults
        defaults.controlParametersDefaults.deadZone = 0.0;
        defaults.controlParametersDefaults.pidScale = 0.0;
        defaults.controlParametersDefaults.pidOffset = 0.0;
        defaults.controlParametersDefaults.stictionUpVal = 0.0;
        defaults.controlParametersDefaults.stictionDownVal = 0.0;
        defaults.controlParametersDefaults.deadzone.jointWithAEA = 0.0494;
        defaults.controlParametersDefaults.deadzone.jointWithAEA3 = 0.0;
        defaults.controlParametersDefaults.deadzone.jointWithAMO = 0.0055;
        defaults.controlParametersDefaults.deadzone.jointWithAKSIM2 = 0.00068;

        // Kalman Filter defaults
        defaults.controlParametersDefaults.kalmanFilterParams.enabled = false;
        defaults.controlParametersDefaults.kalmanFilterParams.x0.fill(0.0);
        defaults.controlParametersDefaults.kalmanFilterParams.Q.fill(0.0);
        defaults.controlParametersDefaults.kalmanFilterParams.R = 0.0;
        defaults.controlParametersDefaults.kalmanFilterParams.P0 = 0.0;

        // Lugre friction model defaults
        defaults.controlParametersDefaults.lugre.Km = -1.0;

        // Joint position limits delta default value
        defaults.jointPosLimitsDelta = 2.0;

        // Max position move interval default value
        defaults.maxPositionMoveInterval = 0.080;
    }

    /**
     * @brief Set all timeout values to the same value at runtime
     * @param defaultTimeout the timeout value in milliseconds
     */
    void setDefaultTimeouts(int defaultTimeout)
    {
        defaults.timeoutsDefaults.velocity_ref = defaultTimeout;
        defaults.timeoutsDefaults.current_ref = defaultTimeout;
        defaults.timeoutsDefaults.pwm_ref = defaultTimeout;
        defaults.timeoutsDefaults.torque_ref = defaultTimeout;
        defaults.timeoutsDefaults.torque_fbk = defaultTimeout;
    }

    /**
     * @brief Set individual timeout values at runtime
     * @param timeoutType which timeout to set ("velocity_ref", "current_ref", "pwm_ref", "torque_ref", "torque_fbk")
     * @param value the timeout value in milliseconds
     * @return true on success, false if invalid timeout type
     */
    bool setTimeoutValue(const std::string& timeoutType, int value)
    {
        if (timeoutType == "velocity_ref")
            defaults.timeoutsDefaults.velocity_ref = value;
        else if (timeoutType == "current_ref")
            defaults.timeoutsDefaults.current_ref = value;
        else if (timeoutType == "pwm_ref")
            defaults.timeoutsDefaults.pwm_ref = value;
        else if (timeoutType == "torque_ref")
            defaults.timeoutsDefaults.torque_ref = value;
        else if (timeoutType == "torque_fbk")
            defaults.timeoutsDefaults.torque_fbk = value;
        else
            return false;
        return true;
    }

    /**
     * @brief Get the current defaults structure
     * @return const reference to embObjMotionControlDefaults_t
     */
    const embObjMotionControlDefaults_t& getDefaults() const
    {
        return defaults;
    }

    /**
     * @brief Get mutable reference to defaults (advanced usage)
     * Allows direct modification of nested fields
     * @return non-const reference to embObjMotionControlDefaults_t
     */
    embObjMotionControlDefaults_t& getDefaultsMutable()
    {
        return defaults;
    }

    /**
     * @brief Reset all defaults to initial values
     * Useful if you need to restore defaults after modifications
     */
    void resetDefaults()
    {
        initializeDefaults();
    }
};

} // namespace defaults
} // namespace yarp::dev::eomc

#endif // __embObjMotionControlDefaultsh__
