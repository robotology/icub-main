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
#include <string_view>
#include <vector>
#include <memory>

#include <yarp/os/Bottle.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>

#include "embObjMotionControlConfigTypes.h"
#include "embObjMotionControlDefaults.h"

// - public #define  --------------------------------------------------------------------------------------------------

namespace yarp {
    namespace dev  {
        namespace eomc {

// Structure to track optional parameters and their defaults
typedef struct
{
    std::string parameterName;
    std::string defaultValue;
    bool wasUsed;
} OptionalParameterInfo_t;


/**
 * @class e0mcParser
 * @brief Parser for Motion Control device configuration files.
 *
 * This class handles the parsing of all information stored in configuration files 
 * related to the motion control device. It supports files with the following 
 * suffixes: *_mc_xml, *_mc_service.xml, and *_mec.xml.
 *
 * The e0mcParser allows the embObjMotionControl device to read these 
 * configurations and initialize itself correctly. To ensure robustness, the parser 
 * utilizes a dedicated file containing default values and valid strings for 
 * parameters that require specific string inputs.
 * * Internally, the class features private utility functions designed to reduce 
 * code duplication, such as helpers for reading "GROUP" parameters or 
 * extracting one or more lines from a specific group.
 */
class Parser
{

private:
    int _njoints;
    std::string _boardname;
    
    //these are the user selected control modes for each control type 
    std::string _userNameControlPosition;
    std::string _userNameControlVelocity;
    std::string _userNameControlMixed;
    std::string _userNameControlTorque;
    std::string _userNameControlCurrent;
    std::string _userNameControlPositionDirect;
    std::string _userNameControlVelocityDirect;


    // Track optional parameters and their defaults
    std::vector<OptionalParameterInfo_t> _optionalParametersUsed;

    const defaults::embObjMotionControlDefaults_t& _defaultSettings;


    //general utils functions
    bool extractGroup(yarp::os::Bottle &input, yarp::os::Bottle &out, const std::string &key1, const std::string &txt, int size, bool mandatory=true);
    bool GetGroupBottle(yarp::os::Searchable &config, const std::string &groupName, yarp::os::Bottle &outBottle, bool mandatory=true);


    //PID parsing auxiliary functions

    //auxiliary functions for parse the CONTROLS group in xml *_mc.xml
    bool readUserNameControlsGroup(yarp::os::Searchable &config);
    //from the pid group in xml file extracts the control law type
    bool getOutputType(yarp::os::Bottle& pidsGroup, eOmc_ctrl_out_type_t &out_type);
    //get from config file the values of the pids
    bool readControlLaw(yarp::os::Bottle& pidsGroup, std::string &controlLaw_str);
    //from the pid group in xml file extracts the pid units type (feedback and output)
    bool parsePidUnitsType(yarp::os::Bottle& pidsGroup, yarp::dev::PidFeedbackUnitsEnum  &fbk_pidunits, yarp::dev::PidOutputUnitsEnum& out_pidunits);

    bool parseSelectedPositionControl(yarp::os::Searchable &config, std::vector<eomc::PidInfo> &pos_pids);
    bool parseSelectedVelocityControl(yarp::os::Searchable &config, std::vector<eomc::PidInfo> &vel_pids);
    bool parseSelectedMixedControl(yarp::os::Searchable &config, std::vector<eomc::PidInfo> &mix_pids);
    bool parseSelectedTorqueControl(yarp::os::Searchable &config, std::vector<eomc::TrqPidInfo> &trq_pids);
    bool parseSelectedPositionDirectControl(yarp::os::Searchable &config, std::vector<eomc::PidInfo> &posDir_pids);
    bool parseSelectedVelocityDirectControl(yarp::os::Searchable &config, std::vector<eomc::PidInfo> &velDir_pids);
    bool parseSelectedCurrentPid(yarp::os::Searchable &config, bool pidisMandatory, std::vector<eomc::PidInfo> &curr_pids);

    bool areControlPidGroupEqual(const std::vector<std::string> &controlGroup);

    bool parsePidsGroup2FOC(yarp::os::Bottle& pidsGroup, std::vector<eomc::PidInfo> &curr_pids);
    /**
     * @brief Parse a minimal set of PID parameters for all joints.
     *
     * Reads from the given configuration group the basic PID fields needed by the simplest controllers:
     * - kff, kp, kd, maxOutput
     *
     * This parser is intended for control modes where integral action, anti-windup and friction/stiction
     * compensation are not required (or are handled elsewhere, e.g. in lower-level loops).
     *
     * @param pidsGroup Input YARP Bottle containing the PID group.
     * @param myPid     Output array of size _njoints filled with parsed PID parameters.
     * @return true on success, false if mandatory parameters are missing or the group has wrong size.
     */
    bool parsePidsGroupMinimalParams(yarp::os::Bottle& pidsGroup, std::vector<eomc::PidInfo> &pids);

    /**
     * @brief Parse an extended set of PID parameters for all joints (adds integral and stiction compensation).
     *
     * Extends the minimal PID parsing by adding:
     * - ki, maxInt (integral term and anti-windup saturation)
     * - stictionUp, stictionDown (static friction breakaway compensation, possibly direction-dependent)
     *
     * This parser is intended for controllers that need better steady-state accuracy (integral action)
     * and/or require a small additional command to overcome static friction at motion start.
     *
     * @param pidsGroup Input YARP Bottle containing the PID group.
     * @param myPid     Output array of size _njoints filled with parsed PID parameters.
     * @return true on success, false if mandatory parameters are missing or the group has wrong size.
     */
    bool parsePidsGroupRegulationParams(yarp::os::Bottle& pidsGroup, std::vector<eomc::PidInfo> &pids);
    bool parsePidsGroupRegulationParams(yarp::os::Bottle& pidsGroup, std::vector<eomc::TrqPidInfo> &pids);

    /**
     * @brief Parse a torque-oriented PID configuration including motor model, friction and filtering parameters.
     *
     * Extends the "regulation" PID parsing by reading additional parameters typically required by torque control:
     * - ktau (torque constant) and optionally kbemf (back-EMF constant, if available)
     * - filterType (selection of the filtering strategy used by the controller/firmware)
     * - viscousPos/viscousNeg and coulombPos/coulombNeg (direction-dependent friction model terms)
     * - velocityThres (velocity threshold used to apply/switch friction compensation near zero speed)
     *
     * Some fields may be optional in the configuration (depending on firmware/control implementation) and can
     * fall back to 0 if not provided.
     *
     * @param pidsGroup Input YARP Bottle containing the PID group.
     * @param myPid     Output array of size _njoints filled with parsed PID parameters.
     * @return true on success, false if mandatory parameters are missing or the group has wrong size.
     */
    bool parsePidsGroupTorqueCompensationParams(yarp::os::Bottle& pidsGroup, std::vector<eomc::TrqPidInfo> &pids);    

    //bool checkJointTypes(PidInfo *pids, const std::string &pid_type);
    //bool checkSinglePid(PidInfo &firstPid, PidInfo &currentPid, const int &firstjoint, const int &currentjoint, const std::string &pid_type);

    //jointset parsing auxiliary functions
    bool convert(std::string const &fromstring, eOmc_jsetconstraint_t &jsetconstraint, bool& formaterror);
    bool convert(yarp::os::Bottle &bottle, std::vector<double> &matrix, bool &formaterror, int targetsize);



    
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

    // ///////// DEBUG FUNCTIONS
    // void debugUtil_printControlLaws(void);


public:
    Parser(int numofjoints, std::string boardname);
    ~Parser();

    bool parsePids(yarp::os::Searchable &config, PidControllers_t &pids, bool lowLevPidisMandatory);
    //TODO: use reference to std::Vector instaed of a raw pointer.
    bool parseFocGroup(yarp::os::Searchable &config, focBasedSpecificInfo_t *foc_based_info, std::string groupName, std::vector<std::unique_ptr<eomc::ITemperatureSensor>>& temperatureSensorsVector);
    //bool parseCurrentPid(yarp::os::Searchable &config, PidInfo *cpids);//deprecated
    bool parseJointsetCfgGroup(yarp::os::Searchable &config, std::vector<JointsSet> &jsets, std::vector<int> &jointtoset);
    bool parseTimeoutsGroup(yarp::os::Searchable &config, std::vector<timeouts_t> &timeouts);
    bool parseCurrentLimits(yarp::os::Searchable &config, std::vector<motorCurrentLimits_t> &currLimits);

    // temperatureLimits array has to be initialized correctly before calling this function
    bool parseTemperatureLimits(yarp::os::Searchable &config, std::vector<temperatureLimits_t> &temperatureLimits);

    // temperatureLimits array has to be initialized correctly before calling this function
    bool parseJointsLimits(yarp::os::Searchable &config, std::vector<jointLimits_t> &jointsLimits);

    bool parseRotorsLimits(yarp::os::Searchable &config, std::vector<rotorLimits_t> &rotorsLimits);
    bool parseCouplingInfo(yarp::os::Searchable &config, couplingInfo_t &couplingInfo);
    bool parseMotioncontrolVersion(yarp::os::Searchable &config, int &version);
    bool parseBehaviourFalgs(yarp::os::Searchable &config, bool &useRawEncoderData, bool  &pwmIsLimited);
    bool isVerboseEnabled(yarp::os::Searchable &config);
    bool parseAxisInfo(yarp::os::Searchable &config, int axisMap[], std::vector<axisInfo_t> &axisInfo);
    bool parseEncoderFactor(yarp::os::Searchable &config, double encoderFactor[]);
    bool parsefullscalePWM(yarp::os::Searchable &config, double dutycycleToPWM[]);
    bool parseAmpsToSensor(yarp::os::Searchable &config, double ampsToSensor[]);
    bool parseGearboxValues(yarp::os::Searchable &config, double gearbox_M2J[], double gearbox_E2J[]);
    bool parseUseMotorSpeedFbkFlags(yarp::os::Searchable &config, int useMotorSpeedFbk[]);
    bool parseImpedanceGroup(yarp::os::Searchable &config,std::vector<impedanceParameters_t> &impedance);
    bool parseLugreGroup(yarp::os::Searchable &config,std::vector<lugreParameters_t> &lugre);
    bool parseDeadzoneValue(yarp::os::Searchable &config, double deadzone[], bool *found);
    bool parseKalmanFilterParams(yarp::os::Searchable &config, std::vector<kalmanFilterParams_t> &kalmanFilterParams);
    bool parseMaintenanceModeGroup(yarp::os::Searchable &config, bool &skipRecalibrationEnabled);

    // Methods for tracking optional parameters
    void registerOptionalParameter(std::string_view paramName, std::string_view defaultValue, bool wasUsed);
    void printOptionalParametersTable();
    void clearOptionalParametersTrack();

};

}}}; //close namespaces

#endif // include guard
