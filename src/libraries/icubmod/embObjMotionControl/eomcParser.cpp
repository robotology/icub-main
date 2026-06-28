// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
* Authors: Valentina Gaggero
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*
*/


#include <yarp/os/Bottle.h>
#include <algorithm>
#include <string.h>
#include <iostream>



#include <eomcParser.h>
#include <embObjMotionControlDefaults.h> // include the header with default values

#include <yarp/os/LogStream.h>

#include "EoCommon.h"
#include "EOarray.h"
#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolMC.h"
#include "EoProtocolAS.h"
#include <vector>


using namespace std;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::dev::eomc;
using namespace yarp::dev::eomc::configType;

//------------------------------------------------------------------------------
// Utilities functions for parsing the configuration file of embObjMotionControl
//------------------------------------------------------------------------------
inline void loadStrings(std::vector<std::string>& dest, const yarp::os::Bottle& source)
{
    dest.clear();
    for (unsigned int i = 1; i < source.size(); ++i)
    {
        dest.push_back(source.get(i).asString());
    }
}


// Static map for eOmc_ctrl_out_type_t: config string -> (enum, description)
struct CtrlOutTypeInfo
{
    eOmc_ctrl_out_type_t enumValue;
    std::string description;
};

static const std::map<std::string, CtrlOutTypeInfo> ctrlOutTypeConfigMap = 
{
    {"n_a",      {eomc_ctrl_out_type_n_a,  "Not applicable / Not set"}},
    {"pwm",      {eomc_ctrl_out_type_pwm,  "PWM (Pulse Width Modulation)"}},
    {"velocity", {eomc_ctrl_out_type_vel,  "Velocity control output"}},
    {"current",  {eomc_ctrl_out_type_cur,  "Current control output"}}
};

// Helper function to get control output type from configuration string
bool getCtrlOutTypeFromConfig(const std::string &configName, eOmc_ctrl_out_type_t &outType, std::string &description)
{
    auto it = ctrlOutTypeConfigMap.find(configName);
    if (it != ctrlOutTypeConfigMap.end())
    {
        outType = it->second.enumValue;
        description = it->second.description;
        return true;
    }
    
    outType = eomc_ctrl_out_type_n_a;
    description = "Unknown control output type";
    return false;
}



//------------------------------------------------------------------------------
// Parser class implementation
//------------------------------------------------------------------------------
yarp::dev::eomc::Parser::Parser(int numofjoints, string boardname)
    : _defaultSettings(defaults::DefaultsFactory::getInstance().getDefaults())
{
    _njoints = numofjoints;
    _boardname = boardname;
    
    _optionalParametersUsed.clear();
};

Parser::~Parser()
{
    // std::vector manages memory automatically, no need to delete
}

//---------------------------------------------------
// auxiliary functions to parse groups in config file 
// --------------------------------------------------
bool Parser::extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size, bool mandatory)
{
    size++;
    out.clear();
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        std::string message = key1 + " parameter not found for board " + _boardname + " in bottle " + input.toString();
        if(mandatory)
            yError () << message.c_str();
        // else
        //     yWarning() << message.c_str();
        return false;
    }

    if(tmp.size()!=size)
    {
        yError () << key1.c_str() << " incorrect number of entries in BOARD " << _boardname;
        return false;
    }

    out=tmp;
    return true;
}

bool Parser::areControlPidGroupEqualInJointSet(const std::vector<std::string> &controlGroup, const std::vector<int> &joint2set, const std::string &controlName)
{
    if(static_cast<int>(controlGroup.size()) != _njoints)
    {
        yError() << "embObjMC BOARD " << _boardname << "Control group" << controlName.c_str() << "has" << controlGroup.size() << "entries but" << _njoints << "joints are expected. Quitting.";
        return false;
    }

    if(static_cast<int>(joint2set.size()) != _njoints)
    {
        yError() << "embObjMC BOARD " << _boardname << "Invalid joint2set size while checking" << controlName.c_str() << ". Quitting.";
        return false;
    }

    std::vector<bool> setNameAssigned(_njoints, false);
    std::vector<std::string> setName(_njoints);

    for(int j = 0; j < _njoints; ++j)
    {
        const int setId = joint2set[j];
        if((setId < 0) || (setId >= _njoints))
        {
            yError() << "embObjMC BOARD " << _boardname << "Invalid set id" << setId << "for joint" << j << "while checking" << controlName.c_str() << ". Quitting.";
            return false;
        }

        if(!setNameAssigned[setId])
        {
            setNameAssigned[setId] = true;
            setName[setId] = controlGroup[j];
            continue;
        }

        if(setName[setId] != controlGroup[j])
        {
            yError() << "embObjMC BOARD " << _boardname << "Joints belonging to set" << setId
                     << "must have the same" << controlName.c_str() << "name. Joint" << j
                     << "uses" << controlGroup[j].c_str() << "while expected" << setName[setId].c_str() << ". Quitting.";
            return false;
        }
    }

    return true;
}

inline bool Parser::GetGroupBottle(yarp::os::Searchable& config, const std::string& controlName, yarp::os::Bottle& outBottle, bool mandatory)
{
    if(controlName == eomc::paramValues::NONE_STR)
    {
        outBottle.clear();
        return true;
    }
    yarp::os::Bottle& tmp = config.findGroup(controlName.c_str());
    if (tmp.isNull())
    {
        outBottle.clear();
        if(mandatory)
        {
            yError() << "embObjMC BOARD " << _boardname << "Unable to find" << controlName.c_str() << " in config file.";
        }
        return false;
    }

    outBottle = tmp;
    return true;
}


//---------------------------------------------------
// auxiliary functions to parse PIDs in config file 
// --------------------------------------------------
bool Parser::readUserNameControlsGroup(yarp::os::Searchable &config) 
{
    Bottle controlsGroup, xtmp, jointsetcfg;
    std::vector<JointsSet> jsets(_njoints);
    std::vector<int> joint2set(_njoints, -1);

    auto readControlNames = [&](const std::string &paramName,
                                const std::string &description,
                                std::vector<std::string> &dest) -> bool
    {
        if (!extractGroup(controlsGroup, xtmp, paramName, description, _njoints))
        {
            return false;
        }

        loadStrings(dest, xtmp);
        return areControlPidGroupEqualInJointSet(dest, joint2set, paramName);
    };

    if(!GetGroupBottle(config, "CONTROLS", controlsGroup)) return false;

    if(GetGroupBottle(config, "JOINTSET_CFG", jointsetcfg, false))
    {
        if(!parseJointsetCfgGroup(config, jsets, joint2set))
        {
            yError() << "embObjMC BOARD " << _boardname << "Unable to parse JOINTSET_CFG while checking control groups consistency. Quitting.";
            return false;
        }
    }
    else
    {
        // Backward compatibility: if JOINTSET_CFG is not present, keep the old global check behaviour.
        std::fill(joint2set.begin(), joint2set.end(), 0);
    }

    if (!readControlNames("positionControl", "Position Control ", _userNameControlPosition))
        return false;

    if (!readControlNames("velocityControl", "Velocity Control ", _userNameControlVelocity))
        return false;

    for(int j = 0; j < _njoints; ++j)
    {
        if((_userNameControlVelocity[j] != paramValues::NONE_STR) && (_userNameControlVelocity[j] != _userNameControlPosition[j]))
        {
            yError() << "embObjMC BOARD " << _boardname << ": velocityControl parameter for joint" << j << "(" << _userNameControlVelocity[j].c_str()
                     << ") must be equal to positionControl parameter (" << _userNameControlPosition[j].c_str() << ") or set to " << paramValues::NONE_STR
                     << " because embedded control uses the same pid parameters for both position and velocity control. Quitting.";
            return false;
        }
    }

    if (!readControlNames("mixedControl", "Mixed Control ", _userNameControlMixed))
        return false;

    for(int j = 0; j < _njoints; ++j)
    {
        if((_userNameControlMixed[j] != paramValues::NONE_STR) && (_userNameControlMixed[j] != _userNameControlPosition[j]))
        {
            yError() << "embObjMC BOARD " << _boardname << ": mixedControl parameter for joint" << j << "(" << _userNameControlMixed[j].c_str()
                     << ") must be equal to positionControl parameter (" << _userNameControlPosition[j].c_str() << ") or set to " << paramValues::NONE_STR
                     << " because embedded control uses the same pid parameters for both position and velocity control. Quitting.";
            return false;
        }
    }

    if (!readControlNames("torqueControl", "Torque Control ", _userNameControlTorque))
        return false;

    if (!readControlNames("currentPid", "Current Pid ", _userNameControlCurrent))
        return false;

    if (!readControlNames("positionDirect", "Position Direct Control ", _userNameControlPositionDirect))
        return false;
    
    if (!readControlNames("velocityDirect", "Velocity Direct Control ", _userNameControlVelocityDirect))
        return false;

    return true;
}


bool yarp::dev::eomc::Parser::getOutputType(yarp::os::Bottle& pidsGroup, eOmc_ctrl_out_type_t &outType)
{
    Value &valOutputType= pidsGroup.find("outputType");
    if( (valOutputType.isNull()) || (!valOutputType.isString()) )
    {
        yError() << "embObjMC BOARD " << _boardname << "Unable read outputType parameter. Quitting.";
        return false;
    }
    std::string description;
    return getCtrlOutTypeFromConfig(valOutputType.toString(), outType, description);
}



 bool Parser::readControlLaw(yarp::os::Bottle& pidsGroup, std::string &controlLaw_str)
{
    // 2) read control_law
    Value &valControlLaw = pidsGroup.find("controlLaw");
    if ((valControlLaw.isNull()) || (!valControlLaw.isString()))
    {
        yError() << "embObjMC BOARD " << _boardname << "Unable read control law parameter. Quitting.";
        return false;
    }

    controlLaw_str = valControlLaw.toString();
    return true;
}

bool Parser::parsePidUnitsType(yarp::os::Bottle& pidsGroup, yarp::dev::PidFeedbackUnitsEnum  &fbk_pidunits, yarp::dev::PidOutputUnitsEnum& out_pidunits)
{

    Value &fbkControlUnits=pidsGroup.find("fbkControlUnits");
    Value &outControlUnits = pidsGroup.find("outputControlUnits");
    if(fbkControlUnits.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << " missing fbkControlUnits parameter";
        return false;
    }
    if(!fbkControlUnits.isString())
    {
        yError() << "embObjMC BOARD " << _boardname << " fbkControlUnits parameter is not a string";
        return false;
    }
    if (outControlUnits.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << " missing outputControlUnits parameter";
        return false;
    }
    if (!outControlUnits.isString())
    {
        yError() << "embObjMC BOARD " << _boardname << " outputControlUnits parameter is not a string";
        return false;
    }

    if(fbkControlUnits.toString()== eomc::paramValues::FbkControlUnits_metric)
    {
        fbk_pidunits = yarp::dev::PidFeedbackUnitsEnum::METRIC;
    }
    else if(fbkControlUnits.toString()==eomc::paramValues::FbkControlUnits_machine)
    {
        fbk_pidunits = yarp::dev::PidFeedbackUnitsEnum::RAW_MACHINE_UNITS;
    }
    else
    {
        yError() << "embObjMC BOARD " << _boardname << "invalid fbkControlUnits value: " << fbkControlUnits.toString().c_str();
        return false;
    }

    if (outControlUnits.toString() == eomc::paramValues::OutControlUnit_dutycycle)
    {
        out_pidunits = yarp::dev::PidOutputUnitsEnum::DUTYCYCLE_PWM_PERCENT;
    }
    else if (outControlUnits.toString() == string("machine_units"))
    {
        out_pidunits = yarp::dev::PidOutputUnitsEnum::RAW_MACHINE_UNITS;
    }
    else
    {
        yError() << "embObjMC BOARD " << _boardname << "invalid outputControlUnits value: " << outControlUnits.toString().c_str();
        return false;
    }
    return true;
}

// //TODO: fill also vpids
// bool Parser::getCorrectPidForEachJoint(PidInfo *ppids, PidInfo *vpids, PidInfo *pDirpids, PidInfo *vDirpids,TrqPidInfo *tpids)
// {
//     Pid_Algorithm *minjerkAlgo_ptr = NULL;
//     Pid_Algorithm *directPosAlgo_ptr = NULL;
//     Pid_Algorithm *directVelAlgo_ptr = NULL;
//     Pid_Algorithm *torqueAlgo_ptr = NULL;

//     //since some joints could not have all pid configured, reset pid values to 0.
//     memset(ppids, 0, sizeof(PidInfo)*_njoints);
//     memset(vpids, 0, sizeof(PidInfo)*_njoints);
//     memset(pDirpids, 0, sizeof(PidInfo)*_njoints);
//     memset(vDirpids, 0, sizeof(PidInfo)*_njoints);
//     memset(tpids, 0, sizeof(TrqPidInfo)*_njoints);

//     map<string, Pid_Algorithm*>::iterator it;

//     for (int i = 0; i < _njoints; i++)
//     {
//         //get position pid
//         it = minjerkAlgoMap.find(_userNameControlPosition);
//         if (it == minjerkAlgoMap.end())
//         {
//             yError() << "embObjMC BOARD" << _boardname << "Cannot find" << _userNameControlPosition.c_str() << "in parsed pos pid";
//             return false;
//         }

//         minjerkAlgo_ptr = minjerkAlgoMap[_userNameControlPosition];

//         ppids[i].pid = minjerkAlgo_ptr->getPID(i);
//         ppids[i].fbk_PidUnits = minjerkAlgo_ptr->fbk_PidUnits;
//         ppids[i].out_PidUnits = minjerkAlgo_ptr->out_PidUnits;
//         //ppids[i].controlLaw =  minjerkAlgo_ptr->type;
//         ppids[i].out_type = minjerkAlgo_ptr->out_type;
//         ppids[i].usernamePidSelected = _userNameControlPosition;
//         ppids[i].enabled = true;

        
//         //get position direct pid
//         if (_userNameControlPositionDirect != paramValues::NONE_STR)
//         {
//             it = directPosAlgoMap.find(_userNameControlPositionDirect);
//             if (it == directPosAlgoMap.end())
//             {
//                 yError() << "embObjMC BOARD " << _boardname  << "Cannot find " << _userNameControlPositionDirect.c_str() << "in parsed posDirect pid";
//                 return false;
//             }

//             directPosAlgo_ptr = directPosAlgoMap[_userNameControlPositionDirect];
//         }

//         if (directPosAlgo_ptr)
//         {
//             pDirpids[i].pid = directPosAlgo_ptr->getPID(i);
//             pDirpids[i].fbk_PidUnits = directPosAlgo_ptr->fbk_PidUnits;
//             pDirpids[i].out_PidUnits = directPosAlgo_ptr->out_PidUnits;
//             //pDirpids[i].controlLaw = directPosAlgo_ptr->type;
//             pDirpids[i].out_type = directPosAlgo_ptr->out_type;
//             pDirpids[i].usernamePidSelected = _userNameControlPositionDirect;
//             pDirpids[i].enabled = true;
//         }
//         else
//         {
//             pDirpids[i].enabled = false;
//             pDirpids[i].usernamePidSelected = paramValues::NONE_STR;
//         }
        
//         //get velocity direct pid
//         if (_userNameControlVelocityDirect != paramValues::NONE_STR)
//         {
//             it = directVelAlgoMap.find(_userNameControlVelocityDirect);
//             if (it == directVelAlgoMap.end())
//             {
//                 yError() << "embObjMC BOARD " << _boardname  << "Cannot find " << _userNameControlVelocityDirect.c_str() << "in parsed velDirect pid";
//                 return false;
//             }

//             directVelAlgo_ptr = directVelAlgoMap[_userNameControlVelocityDirect];
//         }

//         if (directVelAlgo_ptr)
//         {
//             vDirpids[i].pid = directVelAlgo_ptr->getPID(i);
//             vDirpids[i].fbk_PidUnits = directVelAlgo_ptr->fbk_PidUnits;
//             vDirpids[i].out_PidUnits = directVelAlgo_ptr->out_PidUnits;
//             //vDirpids[i].controlLaw = directVelAlgo_ptr->type;
//             vDirpids[i].out_type = directVelAlgo_ptr->out_type;
//             vDirpids[i].usernamePidSelected = _userNameControlVelocityDirect;
//             vDirpids[i].enabled = true;
//         }
//         else
//         {
//             vDirpids[i].enabled = false;
//             vDirpids[i].usernamePidSelected = paramValues::NONE_STR;
//         }

//         //get torque pid
//         if (_userNameControlTorque == paramValues::NONE_STR)
//         {
//             torqueAlgo_ptr = NULL;
//         }
//         else
//         {
//             it = torqueAlgoMap.find(_userNameControlTorque);
//             if (it == torqueAlgoMap.end())
//             {
//                 yError() << "embObjMC BOARD " << _boardname << "Cannot find " << _userNameControlTorque.c_str() << "in parsed trq pid";
//                 return false;
//             }

//             torqueAlgo_ptr = torqueAlgoMap[_userNameControlTorque];
//         }

//         if (torqueAlgo_ptr)
//         {
//             tpids[i].pid = torqueAlgo_ptr->getPID(i);
//             tpids[i].fbk_PidUnits = torqueAlgo_ptr->fbk_PidUnits;
//             tpids[i].out_PidUnits = torqueAlgo_ptr->out_PidUnits;
//             //tpids[i].controlLaw = torqueAlgo_ptr->type;
//             tpids[i].out_type = torqueAlgo_ptr->out_type;
//             tpids[i].usernamePidSelected = _userNameControlTorque;
//             tpids[i].enabled = true;
//             tpids[i].kbemf = _kbemf[i];
//             tpids[i].ktau = _ktau[i];
//             tpids[i].viscousPos = _viscousPos[i];
//             tpids[i].viscousNeg = _viscousNeg[i];
//             tpids[i].coulombPos = _coulombPos[i];
//             tpids[i].coulombNeg = _coulombNeg[i];
//             tpids[i].velocityThres = _velocityThres[i];
//             tpids[i].filterType = _filterType[i];
//         }
//         else
//         {
//             tpids[i].enabled = false;
//             tpids[i].usernamePidSelected = paramValues::NONE_STR;
//         }
//     }

//         //eomc_ctrl_out_type_n_a = 0,
//         //eomc_ctrl_out_type_pwm = 1,
//         //eomc_ctrl_out_type_vel = 2,
//         //eomc_ctrl_out_type_cur = 3

//     return checkJointTypes(tpids, "TORQUE") && checkJointTypes(ppids, "POSITION");
// }


//---------------------------------------------------
// Parse PIDs functions in config file 
// --------------------------------------------------
bool Parser::parsePids(yarp::os::Searchable &config, PidControllers_t &pids, bool lowLevPidisMandatory)
{
    // Read the CONTROLS group to get the user names of the control pids choosen by user.
    // It checks that all joints in the same set have the same control name for each control type.

    if(!readUserNameControlsGroup(config)) 
        return false;

    if(!parseSelectedCurrentPid(config, lowLevPidisMandatory, pids.cur))
        return false;

    if(!parseSelectedPositionControl(config, pids.trj))
        return false;

    if(!parseSelectedVelocityControl(config, pids.vel))
        return false;

    if(!parseSelectedMixedControl(config, pids.mix)) 
        return false;

    if(!parseSelectedPositionDirectControl(config, pids.dir_pos))
        return false;

    if(!parseSelectedVelocityDirectControl(config, pids.dir_vel))
        return false;

    if(!parseSelectedTorqueControl(config, pids.trq))
        return false;

    return true;
}

bool Parser::parseSelectedCurrentPid(yarp::os::Searchable &config, bool pidisMandatory, std::vector<eomc::PidInfo> &curr_pids)
{
    // first of all verify current pid has been configured if it is mandatory
    int enabledCurrentPids = 0;
    for(int j = 0; j < _njoints; ++j)
    {
        if(_userNameControlCurrent[j] != eomc::paramValues::NONE_STR)
        {
            ++enabledCurrentPids;
        }
    }

    if((enabledCurrentPids == 0) && pidisMandatory)
    {
        for (auto& current_pid : curr_pids)
        {
            current_pid.enabled = false;
        }
        yError() << "embObjMC BOARD " << _boardname << "CurrentPid is mandatory. It should be different from none for at least one joint.";
        return false;
    }

    std::map<std::string, std::vector<eomc::PidInfo>> parsedGroups;
    std::map<std::string, std::string> groupControlLaw;
    std::map<std::string, yarp::dev::PidFeedbackUnitsEnum> groupFbkUnits;
    std::map<std::string, yarp::dev::PidOutputUnitsEnum> groupOutUnits;

    for(int j = 0; j < _njoints; ++j)
    {
        const std::string &groupName = _userNameControlCurrent[j];
        if(groupName == eomc::paramValues::NONE_STR)
        {
            continue;
        }

        if(parsedGroups.find(groupName) != parsedGroups.end())
        {
            continue;
        }

        Bottle bot_ctrl;
        if (!GetGroupBottle(config, groupName, bot_ctrl)) return false;

        std::string strControlLaw;
        if(!readControlLaw(bot_ctrl, strControlLaw)) return false;

        if (strControlLaw != eomc::paramValues::ControlLawCurrent)
        {
            yError() << "embObjMC BOARD" << _boardname << "Unable to use" << strControlLaw << "control law for current pid group" << groupName.c_str() << ". Quitting.";
            return false;
        }

        yarp::dev::PidFeedbackUnitsEnum  fbk_unitstype;
        yarp::dev::PidOutputUnitsEnum    out_unitstype;
        if (!parsePidUnitsType(bot_ctrl, fbk_unitstype, out_unitstype)) return false;

        std::vector<eomc::PidInfo> parsed(_njoints);
        if(!parsePidsGroup2FOC(bot_ctrl, parsed)) return false;

        parsedGroups[groupName] = parsed;
        groupControlLaw[groupName] = strControlLaw;
        groupFbkUnits[groupName] = fbk_unitstype;
        groupOutUnits[groupName] = out_unitstype;
    }

    for(int j = 0; j < _njoints; ++j)
    {
        const std::string &groupName = _userNameControlCurrent[j];
        if(groupName == eomc::paramValues::NONE_STR)
        {
            curr_pids[j].enabled = false;
            curr_pids[j].usernamePidSelected = eomc::paramValues::NONE_STR;
            continue;
        }

        curr_pids[j] = parsedGroups[groupName][j];
        curr_pids[j].enabled = true;
        curr_pids[j].fbk_PidUnits = groupFbkUnits[groupName];
        curr_pids[j].out_PidUnits = groupOutUnits[groupName];
        curr_pids[j].controlLaw = groupControlLaw[groupName];
        curr_pids[j].usernamePidSelected = groupName;
        curr_pids[j].out_type = eomc_ctrl_out_type_cur;
    }

    return true;
}

bool Parser::parseSelectedPositionControl(yarp::os::Searchable &config, std::vector<eomc::PidInfo> &pos_pids) 
{
    int enabledPositionPids = 0;
    for(int j = 0; j < _njoints; ++j)
    {
        if(_userNameControlPosition[j] != eomc::paramValues::NONE_STR)
        {
            ++enabledPositionPids;
        }
    }

    if(enabledPositionPids == 0)
    {
        for (auto& p_pid : pos_pids) p_pid.enabled = false;
        yError() << "embObjMC BOARD " << _boardname << "position Pid is mandatory. It should be different from none for at least one joint.";
        return false;
    }

    std::map<std::string, std::vector<eomc::PidInfo>> parsedGroups;
    std::map<std::string, std::string> groupControlLaw;
    std::map<std::string, yarp::dev::PidFeedbackUnitsEnum> groupFbkUnits;
    std::map<std::string, yarp::dev::PidOutputUnitsEnum> groupOutUnits;
    std::map<std::string, eOmc_ctrl_out_type_t> groupOutType;

    for(int j = 0; j < _njoints; ++j)
    {
        const std::string &groupName = _userNameControlPosition[j];
        if(groupName == eomc::paramValues::NONE_STR)
        {
            continue;
        }

        if(parsedGroups.find(groupName) != parsedGroups.end())
        {
            continue;
        }

        Bottle bot_ctrl = config.findGroup(groupName.c_str());
        if(bot_ctrl.isNull())
        {
            yError() << "embObjMC BOARD " << _boardname << "Unable to find position control group " << groupName.c_str() << " in config file. Quitting.";
            return false;
        }

        std::string strControlLaw;
        if(!readControlLaw(bot_ctrl, strControlLaw)) return false;

        if(strControlLaw != eomc::paramValues::ControlLawTrajectory)
        {
            yError() << "embObjMC BOARD " << _boardname << " Unable to use " << strControlLaw << " control law for position pid group " << groupName.c_str() << ". Quitting.";
            return false;
        }

        yarp::dev::PidFeedbackUnitsEnum  fbk_unitstype;
        yarp::dev::PidOutputUnitsEnum    out_unitstype;
        if (!parsePidUnitsType(bot_ctrl, fbk_unitstype, out_unitstype)) return false;

        eOmc_ctrl_out_type_t out_type;
        if(!getOutputType(bot_ctrl, out_type)) return false;

        std::vector<eomc::PidInfo> parsed(_njoints);
        bool parseOk = false;
        switch (out_type)
        {
            case eomc_ctrl_out_type_pwm:
            case eomc_ctrl_out_type_cur:
                parseOk = parsePidsGroupRegulationParams(bot_ctrl, parsed);
                break;
            case eomc_ctrl_out_type_vel:
                parseOk = parsePidsGroupMinimalParams(bot_ctrl, parsed);
                break;
        }
        if(!parseOk)
        {
            yError() << "embObjMC BOARD " << _boardname << "Unable to parse pid values for " << groupName.c_str() << ". Quitting.";
            return false;
        }

        parsedGroups[groupName] = parsed;
        groupControlLaw[groupName] = strControlLaw;
        groupFbkUnits[groupName] = fbk_unitstype;
        groupOutUnits[groupName] = out_unitstype;
        groupOutType[groupName] = out_type;
    }

    for(int j = 0; j < _njoints; ++j)
    {
        const std::string &groupName = _userNameControlPosition[j];
        if(groupName == eomc::paramValues::NONE_STR)
        {
            pos_pids[j].enabled = false;
            pos_pids[j].usernamePidSelected = eomc::paramValues::NONE_STR;
            continue;
        }

        pos_pids[j] = parsedGroups[groupName][j];
        pos_pids[j].enabled = true;
        pos_pids[j].fbk_PidUnits = groupFbkUnits[groupName];
        pos_pids[j].out_PidUnits = groupOutUnits[groupName];
        pos_pids[j].controlLaw = groupControlLaw[groupName];
        pos_pids[j].usernamePidSelected = groupName;
        pos_pids[j].out_type = groupOutType[groupName];
    }

    return true;
}

bool Parser::parseSelectedVelocityControl(yarp::os::Searchable &config, std::vector<eomc::PidInfo> &vel_pids)
{
    int enabledVelocityPids = 0;
    for(int j = 0; j < _njoints; ++j)
    {
        if(_userNameControlVelocity[j] != eomc::paramValues::NONE_STR)
        {
            ++enabledVelocityPids;
        }
    }

    if(enabledVelocityPids == 0)
    {
        for (auto& v_pid : vel_pids)
        {
            v_pid.enabled = false;
            v_pid.usernamePidSelected = eomc::paramValues::NONE_STR;
        }

        yWarning() << "embObjMC BOARD " << _boardname << "Velocity pid has been desabled";
        return true;
    }

    std::map<std::string, std::vector<eomc::PidInfo>> parsedGroups;
    std::map<std::string, std::string> groupControlLaw;
    std::map<std::string, yarp::dev::PidFeedbackUnitsEnum> groupFbkUnits;
    std::map<std::string, yarp::dev::PidOutputUnitsEnum> groupOutUnits;
    std::map<std::string, eOmc_ctrl_out_type_t> groupOutType;

    for(int j = 0; j < _njoints; ++j)
    {
        const std::string &groupName = _userNameControlVelocity[j];
        if(groupName == eomc::paramValues::NONE_STR)
        {
            continue;
        }

        if(parsedGroups.find(groupName) != parsedGroups.end())
        {
            continue;
        }

        Bottle bot_ctrl;
        if (!GetGroupBottle(config, groupName, bot_ctrl)) return false;

        std::string strControlLaw;
        if(!readControlLaw(bot_ctrl, strControlLaw)) return false;

        if(strControlLaw != eomc::paramValues::ControlLawTrajectory)
        {
            yError() << "embObjMC BOARD " << _boardname << " Unable to use " << strControlLaw << " control law for velocity pid group " << groupName.c_str() << ". Quitting.";
            return false;
        }

        yarp::dev::PidFeedbackUnitsEnum  fbk_unitstype;
        yarp::dev::PidOutputUnitsEnum    out_unitstype;
        if (!parsePidUnitsType(bot_ctrl, fbk_unitstype, out_unitstype)) return false;

        eOmc_ctrl_out_type_t out_type;
        if(!getOutputType(bot_ctrl, out_type)) return false;

        std::vector<eomc::PidInfo> parsed(_njoints);
        bool parseOk = false;
        switch (out_type)
        {
            case eomc_ctrl_out_type_pwm:
            case eomc_ctrl_out_type_cur:
                parseOk = parsePidsGroupRegulationParams(bot_ctrl, parsed);
                break;
            case eomc_ctrl_out_type_vel:
                parseOk = parsePidsGroupMinimalParams(bot_ctrl, parsed);
                break;
        }
        if(!parseOk)
        {
            yError() << "embObjMC BOARD " << _boardname << "Unable to parse pid values for " << groupName.c_str() << ". Quitting.";
            return false;
        }

        parsedGroups[groupName] = parsed;
        groupControlLaw[groupName] = strControlLaw;
        groupFbkUnits[groupName] = fbk_unitstype;
        groupOutUnits[groupName] = out_unitstype;
        groupOutType[groupName] = out_type;
    }

    for(int j = 0; j < _njoints; ++j)
    {
        const std::string &groupName = _userNameControlVelocity[j];
        if(groupName == eomc::paramValues::NONE_STR)
        {
            vel_pids[j].enabled = false;
            vel_pids[j].usernamePidSelected = eomc::paramValues::NONE_STR;
            continue;
        }

        vel_pids[j] = parsedGroups[groupName][j];
        vel_pids[j].enabled = true;
        vel_pids[j].fbk_PidUnits = groupFbkUnits[groupName];
        vel_pids[j].out_PidUnits = groupOutUnits[groupName];
        vel_pids[j].controlLaw = groupControlLaw[groupName];
        vel_pids[j].usernamePidSelected = groupName;
        vel_pids[j].out_type = groupOutType[groupName];
    }

    return true;

}

bool Parser::parseSelectedMixedControl(yarp::os::Searchable &config, std::vector<eomc::PidInfo> &mix_pids)
{
    int enabledMixedPids = 0;
    for(int j = 0; j < _njoints; ++j)
    {
        if(_userNameControlMixed[j] != eomc::paramValues::NONE_STR)
        {
            ++enabledMixedPids;
        }
    }

    if(enabledMixedPids == 0)
    {
        for (auto& m_pid : mix_pids)
        {
            m_pid.enabled = false;
            m_pid.usernamePidSelected = eomc::paramValues::NONE_STR;
        }

        yInfo() << "embObjMC BOARD " << _boardname << "Mixed pid has been desabled";
        return true;
    }

    for(int j = 0; j < _njoints; ++j)
    {
        if((_userNameControlMixed[j] != paramValues::NONE_STR) && (_userNameControlMixed[j] != _userNameControlPosition[j]))
        {
            yError() << "embObjMC BOARD " << _boardname << "Mixed control law has a different name than position control law for joint" << j << ". Currently fw doesn't support different pid values for mixed control from position control, so the control law name should be the same. Quitting.";
            return false;
        }
    }

    std::map<std::string, std::vector<eomc::PidInfo>> parsedGroups;
    std::map<std::string, std::string> groupControlLaw;
    std::map<std::string, yarp::dev::PidFeedbackUnitsEnum> groupFbkUnits;
    std::map<std::string, yarp::dev::PidOutputUnitsEnum> groupOutUnits;
    std::map<std::string, eOmc_ctrl_out_type_t> groupOutType;

    for(int j = 0; j < _njoints; ++j)
    {
        const std::string &groupName = _userNameControlMixed[j];
        if(groupName == eomc::paramValues::NONE_STR)
        {
            continue;
        }

        if(parsedGroups.find(groupName) != parsedGroups.end())
        {
            continue;
        }

        Bottle bot_ctrl;
        if (!GetGroupBottle(config, groupName, bot_ctrl)) return false;

        std::string strControlLaw;
        if(!readControlLaw(bot_ctrl, strControlLaw)) return false;

        if(strControlLaw != paramValues::ControlLawTrajectory)
        {
            yError() << "embObjMC BOARD " << _boardname << "Unable to use " << strControlLaw << " control law for mixed pid group " << groupName.c_str() << ". Quitting.";
            return false;
        }

        yarp::dev::PidFeedbackUnitsEnum  fbk_unitstype;
        yarp::dev::PidOutputUnitsEnum    out_unitstype;
        if (!parsePidUnitsType(bot_ctrl, fbk_unitstype, out_unitstype)) return false;

        eOmc_ctrl_out_type_t out_type;
        if(!getOutputType(bot_ctrl, out_type)) return false;

        std::vector<eomc::PidInfo> parsed(_njoints);
        bool parseOk = false;
        switch (out_type)
        {
            case eomc_ctrl_out_type_pwm:
            case eomc_ctrl_out_type_cur:
                parseOk = parsePidsGroupRegulationParams(bot_ctrl, parsed);
                break;
            case eomc_ctrl_out_type_vel:
                parseOk = parsePidsGroupMinimalParams(bot_ctrl, parsed);
                break;
        }
        if(!parseOk)
        {
            yError() << "embObjMC BOARD " << _boardname << "Unable to parse pid values for " << groupName.c_str() << ". Quitting.";
            return false;
        }

        parsedGroups[groupName] = parsed;
        groupControlLaw[groupName] = strControlLaw;
        groupFbkUnits[groupName] = fbk_unitstype;
        groupOutUnits[groupName] = out_unitstype;
        groupOutType[groupName] = out_type;
    }

    for(int j = 0; j < _njoints; ++j)
    {
        const std::string &groupName = _userNameControlMixed[j];
        if(groupName == eomc::paramValues::NONE_STR)
        {
            mix_pids[j].enabled = false;
            mix_pids[j].usernamePidSelected = eomc::paramValues::NONE_STR;
            continue;
        }

        mix_pids[j] = parsedGroups[groupName][j];
        mix_pids[j].enabled = true;
        mix_pids[j].fbk_PidUnits = groupFbkUnits[groupName];
        mix_pids[j].out_PidUnits = groupOutUnits[groupName];
        mix_pids[j].controlLaw = groupControlLaw[groupName];
        mix_pids[j].usernamePidSelected = groupName;
        mix_pids[j].out_type = groupOutType[groupName];
    }

    return true;

}

bool Parser::parseSelectedPositionDirectControl(yarp::os::Searchable &config, std::vector<eomc::PidInfo> &posdir_pids) 
{
    int enabledPosDirectPids = 0;
    for(int j = 0; j < _njoints; ++j)
    {
        if(_userNameControlPositionDirect[j] != paramValues::NONE_STR)
        {
            ++enabledPosDirectPids;
        }
    }

    if(enabledPosDirectPids == 0)
    {
        for (auto& pd_pid : posdir_pids)
        {
            pd_pid.enabled = false;
            pd_pid.usernamePidSelected = paramValues::NONE_STR;
        }
        
        registerOptionalParameter("position direct pid", "0.0 for all pid values", false);
        //yWarning() << "embObjMC BOARD " << _boardname << "position direct pid has been disabled";
        return true;
    }

    std::map<std::string, std::vector<eomc::PidInfo>> parsedGroups;
    std::map<std::string, std::string> groupControlLaw;
    std::map<std::string, yarp::dev::PidFeedbackUnitsEnum> groupFbkUnits;
    std::map<std::string, yarp::dev::PidOutputUnitsEnum> groupOutUnits;
    std::map<std::string, eOmc_ctrl_out_type_t> groupOutType;

    for(int j = 0; j < _njoints; ++j)
    {
        const std::string &groupName = _userNameControlPositionDirect[j];
        if(groupName == paramValues::NONE_STR)
        {
            continue;
        }

        if(parsedGroups.find(groupName) != parsedGroups.end())
        {
            continue;
        }

        Bottle bot_ctrl;
        if (!GetGroupBottle(config, groupName, bot_ctrl)) return false;

        std::string strControlLaw;
        if(!readControlLaw(bot_ctrl, strControlLaw)) return false;

        if(strControlLaw != paramValues::ControlLawDirect)
        {
            yError() << "embObjMC BOARD " << _boardname  << " Unable to use " << strControlLaw << " control law for position direct pid group " << groupName.c_str() << ". Quitting.";
            return false;
        }

        yarp::dev::PidFeedbackUnitsEnum  fbk_unitstype;
        yarp::dev::PidOutputUnitsEnum    out_unitstype;
        if (!parsePidUnitsType(bot_ctrl, fbk_unitstype, out_unitstype)) return false;

        eOmc_ctrl_out_type_t out_type;
        if(!getOutputType(bot_ctrl, out_type)) return false;

        std::vector<eomc::PidInfo> parsed(_njoints);
        bool parseOk = false;
        switch (out_type)
        {
            case eomc_ctrl_out_type_pwm:
            case eomc_ctrl_out_type_cur:
                parseOk = parsePidsGroupRegulationParams(bot_ctrl, parsed);
                break;
            case eomc_ctrl_out_type_vel:
                parseOk = parsePidsGroupMinimalParams(bot_ctrl, parsed);
                break;
        }
        if(!parseOk)
        {
            yError() << "embObjMC BOARD " << _boardname << "Unable to parse pid values for " << groupName.c_str() << ". Quitting.";
            return false;
        }

        parsedGroups[groupName] = parsed;
        groupControlLaw[groupName] = strControlLaw;
        groupFbkUnits[groupName] = fbk_unitstype;
        groupOutUnits[groupName] = out_unitstype;
        groupOutType[groupName] = out_type;
    }

    for(int j = 0; j < _njoints; ++j)
    {
        const std::string &groupName = _userNameControlPositionDirect[j];
        if(groupName == paramValues::NONE_STR)
        {
            posdir_pids[j].enabled = false;
            posdir_pids[j].usernamePidSelected = paramValues::NONE_STR;
            continue;
        }

        posdir_pids[j] = parsedGroups[groupName][j];
        posdir_pids[j].enabled = true;
        posdir_pids[j].fbk_PidUnits = groupFbkUnits[groupName];
        posdir_pids[j].out_PidUnits = groupOutUnits[groupName];
        posdir_pids[j].controlLaw = groupControlLaw[groupName];
        posdir_pids[j].usernamePidSelected = groupName;
        posdir_pids[j].out_type = groupOutType[groupName];
    }

    return true;
}

bool Parser::parseSelectedVelocityDirectControl(yarp::os::Searchable &config, std::vector<eomc::PidInfo> &veldir_pids) 
{
    int enabledVelDirectPids = 0;
    for(int j = 0; j < _njoints; ++j)
    {
        if(_userNameControlVelocityDirect[j] != paramValues::NONE_STR)
        {
            ++enabledVelDirectPids;
        }
    }

    if(enabledVelDirectPids == 0)
    {
        for (auto& vd_pid : veldir_pids)
        {
            vd_pid.enabled = false;
            vd_pid.usernamePidSelected = paramValues::NONE_STR;
        }
        
        registerOptionalParameter("velocity direct pid", "0.0 for all pid values", false);
        //yWarning() << "embObjMC BOARD " << _boardname << "velocity direct pid has been disabled";
        return true;
    }

    std::map<std::string, std::vector<eomc::PidInfo>> parsedGroups;
    std::map<std::string, std::string> groupControlLaw;
    std::map<std::string, yarp::dev::PidFeedbackUnitsEnum> groupFbkUnits;
    std::map<std::string, yarp::dev::PidOutputUnitsEnum> groupOutUnits;
    std::map<std::string, eOmc_ctrl_out_type_t> groupOutType;

    for(int j = 0; j < _njoints; ++j)
    {
        const std::string &groupName = _userNameControlVelocityDirect[j];
        if(groupName == paramValues::NONE_STR)
        {
            continue;
        }

        if(parsedGroups.find(groupName) != parsedGroups.end())
        {
            continue;
        }

        Bottle bot_ctrl;
        if (!GetGroupBottle(config, groupName, bot_ctrl)) return false;

        std::string strControlLaw;
        if(!readControlLaw(bot_ctrl, strControlLaw)) return false;

        if(strControlLaw != paramValues::ControlLawDirect)
        {
            yError() << "embObjMC BOARD " << _boardname  << " Unable to use " << strControlLaw << " control law for velocity direct pid group " << groupName.c_str() << ". Quitting.";
            return false;
        }

        yarp::dev::PidFeedbackUnitsEnum  fbk_unitstype;
        yarp::dev::PidOutputUnitsEnum    out_unitstype;
        if (!parsePidUnitsType(bot_ctrl, fbk_unitstype, out_unitstype)) return false;

        eOmc_ctrl_out_type_t out_type;
        if(!getOutputType(bot_ctrl, out_type)) return false;

        // Temporary workaround: map PWM to VEL and CUR to VEL+CUR for velocity direct control.
        if (out_type == eomc_ctrl_out_type_pwm)
        {
            out_type = eomc_ctrl_out_type_vel;
        }
        else if (out_type == eomc_ctrl_out_type_cur)
        {
            out_type = eomc_ctrl_out_type_vel_cur;
        }

        std::vector<eomc::PidInfo> parsed(_njoints);
        if(!parsePidsGroup2FOC(bot_ctrl, parsed))
        {
            yError() << "embObjMC BOARD " << _boardname << "Unable to parse pid values for " << groupName.c_str() << ". Quitting.";
            return false;
        }

        parsedGroups[groupName] = parsed;
        groupControlLaw[groupName] = strControlLaw;
        groupFbkUnits[groupName] = fbk_unitstype;
        groupOutUnits[groupName] = out_unitstype;
        groupOutType[groupName] = out_type;
    }

    for(int j = 0; j < _njoints; ++j)
    {
        const std::string &groupName = _userNameControlVelocityDirect[j];
        if(groupName == paramValues::NONE_STR)
        {
            veldir_pids[j].enabled = false;
            veldir_pids[j].usernamePidSelected = paramValues::NONE_STR;
            continue;
        }

        veldir_pids[j] = parsedGroups[groupName][j];
        veldir_pids[j].enabled = true;
        veldir_pids[j].fbk_PidUnits = groupFbkUnits[groupName];
        veldir_pids[j].out_PidUnits = groupOutUnits[groupName];
        veldir_pids[j].controlLaw = groupControlLaw[groupName];
        veldir_pids[j].usernamePidSelected = groupName;
        veldir_pids[j].out_type = groupOutType[groupName];
    }

    return true;
}

bool Parser::parseSelectedTorqueControl(yarp::os::Searchable &config,  std::vector<eomc::TrqPidInfo> &trq_pids)
{
    int enabledTorquePids = 0;
    for(int j = 0; j < _njoints; ++j)
    {
        if(_userNameControlTorque[j] != paramValues::NONE_STR)
        {
            ++enabledTorquePids;
        }
    }

    if(enabledTorquePids == 0)
    {
        for (auto& t_pid : trq_pids)
        {
            t_pid.enabled = false;
            t_pid.usernamePidSelected = paramValues::NONE_STR;
        }
        return true;
    }

    std::map<std::string, std::vector<eomc::TrqPidInfo>> parsedGroups;
    std::map<std::string, std::string> groupControlLaw;
    std::map<std::string, yarp::dev::PidFeedbackUnitsEnum> groupFbkUnits;
    std::map<std::string, yarp::dev::PidOutputUnitsEnum> groupOutUnits;
    std::map<std::string, eOmc_ctrl_out_type_t> groupOutType;

    for(int j = 0; j < _njoints; ++j)
    {
        const std::string &groupName = _userNameControlTorque[j];
        if(groupName == paramValues::NONE_STR)
        {
            continue;
        }

        if(parsedGroups.find(groupName) != parsedGroups.end())
        {
            continue;
        }

        Bottle bot_ctrl;
        if (!GetGroupBottle(config, groupName, bot_ctrl)) return false;

        std::string strControlLaw;
        if(!readControlLaw(bot_ctrl, strControlLaw)) return false;

        if(strControlLaw != paramValues::ControlLawTorque)
        {
            yError() << "embObjMC BOARD " << _boardname << " Unable to use " << strControlLaw << " control law for torque pid group " << groupName.c_str() << ". Quitting.";
            return false;
        }

        yarp::dev::PidFeedbackUnitsEnum  fbk_unitstype;
        yarp::dev::PidOutputUnitsEnum    out_unitstype;
        if (!parsePidUnitsType(bot_ctrl, fbk_unitstype, out_unitstype)) return false;

        eOmc_ctrl_out_type_t out_type;
        if(!getOutputType(bot_ctrl, out_type)) return false;

        std::vector<eomc::TrqPidInfo> parsed(_njoints);
        bool parseOk = false;
        switch (out_type)
        {
            case eomc_ctrl_out_type_pwm:
            case eomc_ctrl_out_type_cur:
                parseOk = parsePidsGroupTorqueCompensationParams(bot_ctrl, parsed);
                break;
            case eomc_ctrl_out_type_vel:
                parseOk = parsePidsGroupRegulationParams(bot_ctrl, parsed);
                break;
        }
        if(!parseOk)
        {
            yError() << "embObjMC BOARD " << _boardname << "Unable to parse pid values for " << groupName.c_str() << ". Quitting.";
            return false;
        }

        parsedGroups[groupName] = parsed;
        groupControlLaw[groupName] = strControlLaw;
        groupFbkUnits[groupName] = fbk_unitstype;
        groupOutUnits[groupName] = out_unitstype;
        groupOutType[groupName] = out_type;
    }

    for(int j = 0; j < _njoints; ++j)
    {
        const std::string &groupName = _userNameControlTorque[j];
        if(groupName == paramValues::NONE_STR)
        {
            trq_pids[j].enabled = false;
            trq_pids[j].usernamePidSelected = paramValues::NONE_STR;
            continue;
        }

        trq_pids[j] = parsedGroups[groupName][j];
        trq_pids[j].enabled = true;
        trq_pids[j].fbk_PidUnits = groupFbkUnits[groupName];
        trq_pids[j].out_PidUnits = groupOutUnits[groupName];
        trq_pids[j].controlLaw = groupControlLaw[groupName];
        trq_pids[j].usernamePidSelected = groupName;
        trq_pids[j].out_type = groupOutType[groupName];
    }

    return true;
}


/*
   <group name="2FOC_CUR_CONTROL">
        <param name="controlLaw">          low_lev_current      </param> 
        <param name="fbkControlUnits">     machine_units        </param> 
        <param name="outputControlUnits">  machine_units        </param>
        <param name="cur_kff">                     0         0      </param>
        <param name="cur_kp">                      8         8      </param>       
        <param name="cur_kd">                      0         0      </param>       
        <param name="cur_ki">                      2         2      </param>
        <param name="cur_shift">                  10        10      </param>
        <param name="cur_maxOutput">           32000     32000      </param>                 
        <param name="cur_maxInt">              32000     32000      </param>         
    </group>
    
    <group name="2FOC_VEL_CONTROL">
        <param name="controlLaw">          low_lev_velocity     </param> 
        <param name="fbkControlUnits">     machine_units        </param> 
        <param name="outputControlUnits">  machine_units        </param>
        <param name="spd_kff">                     0         0      </param>
        <param name="spd_kp">                     12        12      </param>       
        <param name="spd_kd">                      0         0      </param>       
        <param name="spd_ki">                     16        16      </param>
        <param name="spd_shift">                  10        10      </param>
        <param name="spd_maxOutput">           32000     32000      </param>                 
        <param name="spd_maxInt">              32000     32000      </param>        
    </group>
*/

bool Parser::parsePidsGroup2FOC(yarp::os::Bottle& pidsGroup, std::vector<eomc::PidInfo> &curr_pids)
{
    Bottle xtmp;

    if (!extractGroup(pidsGroup, xtmp, "kff", "kff parameter", _njoints)) return false;
    for (int j = 0; j<_njoints; j++) curr_pids[j].pid.kff = xtmp.get(j + 1).asFloat64();

    if (!extractGroup(pidsGroup, xtmp, "kp", "kp parameter", _njoints)) return false;
    for (int j = 0; j<_njoints; j++) curr_pids[j].pid.kp = xtmp.get(j + 1).asFloat64();

    if (!extractGroup(pidsGroup, xtmp, "kd", "kd parameter", _njoints)) return false;
    for (int j = 0; j<_njoints; j++) curr_pids[j].pid.kd = xtmp.get(j + 1).asFloat64();

    if (!extractGroup(pidsGroup, xtmp, "maxOutput", "maxOutput parameter", _njoints)) return false;
    for (int j = 0; j<_njoints; j++) curr_pids[j].pid.max_output = xtmp.get(j + 1).asFloat64();

    if (!extractGroup(pidsGroup, xtmp, "ki", "ki parameter", _njoints)) return false;
    for (int j = 0; j<_njoints; j++) curr_pids[j].pid.ki = xtmp.get(j + 1).asFloat64();

    if (!extractGroup(pidsGroup, xtmp, "maxInt", "maxInt parameter", _njoints)) return false;
    for (int j = 0; j<_njoints; j++) curr_pids[j].pid.max_int = xtmp.get(j + 1).asFloat64();

    if (!extractGroup(pidsGroup, xtmp, "shift", "shift parameter", _njoints)) return false;
    for (int j = 0; j<_njoints; j++) curr_pids[j].pid.scale = xtmp.get(j + 1).asFloat64();

    return true;
}


bool Parser::parsePidsGroupMinimalParams(Bottle& pidsGroup, std::vector<eomc::PidInfo> &pids)
{
    /*
    <param name = "kff">                        1           1         < / param>
    <param name = "kp">                         5           5         < / param>
    <param name = "kd">                         0           0         < / param>
    <param name = "maxOutput">              32000       32000         < / param>
    */

    Bottle xtmp;

    if (!extractGroup(pidsGroup, xtmp, "kff", "kff parameter", _njoints)) return false;
    for (int j = 0; j<_njoints; j++) pids[j].pid.kff = xtmp.get(j + 1).asFloat64();

    if (!extractGroup(pidsGroup, xtmp, "kp", "kp parameter", _njoints)) return false;
    for (int j = 0; j<_njoints; j++) pids[j].pid.kp = xtmp.get(j + 1).asFloat64();

    if (!extractGroup(pidsGroup, xtmp, "kd", "kd parameter", _njoints)) return false;
    for (int j = 0; j<_njoints; j++) pids[j].pid.kd = xtmp.get(j + 1).asFloat64();

    if (!extractGroup(pidsGroup, xtmp, "maxOutput", "maxOutput parameter", _njoints)) return false;
    for (int j = 0; j<_njoints; j++) pids[j].pid.max_output = xtmp.get(j + 1).asFloat64();

    return true;
}

bool Parser::parsePidsGroupRegulationParams(Bottle& pidsGroup, std::vector<eomc::PidInfo> &pids)
{
    /*
    <param name = "kff">                        1           1         < / param>
    <param name = "kp">                         5           5         < / param>
    <param name = "kd">                         0           0         < / param>
    <param name = "maxOutput">              32000       32000         < / param>
    */

    if (!parsePidsGroupMinimalParams(pidsGroup, pids)) return false;

    /*
    <param name = "ki">                     7111.0      1066.0        < / param>
    <param name = "maxInt">                  750        1000          < / param>
    <param name = "stictionUp">                0           0          < / param>
    <param name = "stictionDwn">               0           0          < / param>
    */

    Bottle xtmp;

    if (!extractGroup(pidsGroup, xtmp, "ki", "ki parameter", _njoints)) return false;
    for (int j = 0; j<_njoints; j++) pids[j].pid.ki = xtmp.get(j + 1).asFloat64();

    if (!extractGroup(pidsGroup, xtmp, "maxInt", "maxInt parameter", _njoints)) return false;
    for (int j = 0; j<_njoints; j++) pids[j].pid.max_int = xtmp.get(j + 1).asFloat64();

    if (!extractGroup(pidsGroup, xtmp, "stictionUp", "stictionUp parameter", _njoints)) return false;
    for (int j = 0; j<_njoints; j++) pids[j].pid.stiction_up_val = xtmp.get(j + 1).asFloat64();

    if (!extractGroup(pidsGroup, xtmp, "stictionDown", "stictionDown parameter", _njoints)) return false;
    for (int j = 0; j<_njoints; j++) pids[j].pid.stiction_down_val = xtmp.get(j + 1).asFloat64();

    return true;
}

bool Parser::parsePidsGroupRegulationParams(Bottle& pidsGroup, std::vector<eomc::TrqPidInfo>& pids)
{
    std::vector<eomc::PidInfo> basePids(_njoints);
    if (!parsePidsGroupRegulationParams(pidsGroup, basePids)) return false;
    for (int j = 0; j < _njoints; j++)
    {
        static_cast<eomc::PidInfo&>(pids[j]) = basePids[j];
    }
    return true;
}

bool Parser::parsePidsGroupTorqueCompensationParams(Bottle& pidsGroup, std::vector<eomc::TrqPidInfo> &pids)
{
    /*
    <param name = "kff">                        1           1         < / param>
    <param name = "kp">                         5           5         < / param>
    <param name = "kd">                         0           0         < / param>
    <param name = "maxOutput">              32000       32000         < / param>
    <param name = "ki">                     7111.0      1066.0        < / param>
    <param name = "maxInt">                  750        1000          < / param>
    <param name = "stictionUp">                0           0          < / param>
    <param name = "stictionDwn">               0           0          < / param>
    */

    if (!parsePidsGroupRegulationParams(pidsGroup, pids)) return false;

    Bottle xtmp;

    if (!extractGroup(pidsGroup, xtmp, "kbemf", "kbemf parameter", _njoints, false)) 
    {
        registerOptionalParameter("kbemf", std::to_string(_defaultSettings.torquePidDefaults.kbemf), true);
        for (int j = 0; j<_njoints; j++) pids[j].kbemf = _defaultSettings.torquePidDefaults.kbemf;
    } 
    else
    {
        for (int j = 0; j<_njoints; j++) pids[j].kbemf = xtmp.get(j + 1).asFloat64();
    }
    
    if (!extractGroup(pidsGroup, xtmp, "ktau", "ktau parameter", _njoints)) return false; 
    for (int j = 0; j<_njoints; j++) pids[j].ktau = xtmp.get(j + 1).asFloat64();
    
    if (!extractGroup(pidsGroup, xtmp, "filterType", "filterType param", _njoints)) return false; 
    for (int j = 0; j<_njoints; j++) pids[j].filterType = xtmp.get(j + 1).asInt32();

    // Friction parameters
    if (!extractGroup(pidsGroup, xtmp, "viscousPos", "viscousPos parameter", _njoints, false)) 
    {
        registerOptionalParameter("viscousPos", std::to_string(_defaultSettings.torquePidDefaults.viscousPos), true);
        for (int j = 0; j<_njoints; j++) pids[j].viscousPos = _defaultSettings.torquePidDefaults.viscousPos;
    }
    else
    {
        for (int j = 0; j<_njoints; j++) pids[j].viscousPos = xtmp.get(j + 1).asFloat64();
    }

    if (!extractGroup(pidsGroup, xtmp, "viscousNeg", "viscousNeg parameter", _njoints, false)) 
    {
        registerOptionalParameter("viscousNeg", std::to_string(_defaultSettings.torquePidDefaults.viscousNeg), true);
        for (int j = 0; j<_njoints; j++) pids[j].viscousNeg = _defaultSettings.torquePidDefaults.viscousNeg;
    }
    else
    {
        for (int j = 0; j<_njoints; j++) pids[j].viscousNeg = xtmp.get(j + 1).asFloat64();
    }

    if (!extractGroup(pidsGroup, xtmp, "coulombPos", "coulombPos parameter", _njoints, false))
    {
        registerOptionalParameter("coulombPos", std::to_string(_defaultSettings.torquePidDefaults.coulombPos), true);
        for (int j = 0; j<_njoints; j++) pids[j].coulombPos = _defaultSettings.torquePidDefaults.coulombPos;
    }
    else
    {
        for (int j = 0; j<_njoints; j++) pids[j].coulombPos = xtmp.get(j + 1).asFloat64();
    }

    if (!extractGroup(pidsGroup, xtmp, "coulombNeg", "coulombNeg parameter", _njoints, false))
    {
        registerOptionalParameter("coulombNeg", std::to_string(_defaultSettings.torquePidDefaults.coulombNeg), true);
        for (int j = 0; j<_njoints; j++) pids[j].coulombNeg = _defaultSettings.torquePidDefaults.coulombNeg;
    }
    else
    {
        for (int j = 0; j<_njoints; j++) pids[j].coulombNeg = xtmp.get(j + 1).asFloat64();
    }

    if (!extractGroup(pidsGroup, xtmp, "velocityThres", "velocity threshold parameter for torque control", _njoints, false))
    {
        registerOptionalParameter("velocityThres", std::to_string(_defaultSettings.torquePidDefaults.velocityThres), true);
        for (int j = 0; j<_njoints; j++) pids[j].velocityThres = _defaultSettings.torquePidDefaults.velocityThres;
    }
    else
    {
        for (int j = 0; j<_njoints; j++) pids[j].velocityThres = xtmp.get(j + 1).asFloat64();
    }


    return true;
}


bool Parser::parseFocGroup(yarp::os::Searchable &config, std::vector<focBasedSpecificInfo_t>& foc_based_info, std::string groupName, std::vector<std::unique_ptr<eomc::ITemperatureSensor>>& temperatureSensorsVector)
{
    Bottle &focGroup=config.findGroup(groupName);
    if (focGroup.isNull() )
    {
        yError() << "embObjMC BOARD " << _boardname << " detected that Group " << groupName << " is not found in configuration file";
        return false;
    }

    Bottle xtmp;
    unsigned int i;
    // 1. HasHallSensor
    if (!extractGroup(focGroup, xtmp, "HasHallSensor", "HasHallSensor 0/1 ", _njoints))
        return false;
    
    for (i = 1; i < xtmp.size(); i++)
        foc_based_info[i - 1].hasHallSensor = xtmp.get(i).asInt32() != 0;

    // 2. HasTempSensor is depracated, use TemperatureSensorType
    if (extractGroup(focGroup, xtmp, "HasTempSensor", "HasTempSensor 0/1 ", _njoints, false))
    {
        yError() << _boardname << "ATTENTION: HasTempSensor is deprecated. Use TemperatureSensorType (PT100, PT1000, NONE). If not specified, TemperatureSensorType defaults to NONE and the firmware does not check the motor temperature";
        return false;
    }

    if (!extractGroup(focGroup, xtmp, "TemperatureSensorType", "TemperatureSensorType PT100/PT1000/NONE ", _njoints, false))
    {
        registerOptionalParameter("TemperatureSensorType", eomc::paramValues::TemperatureSensorType_NONE, false);
        for (i = 0; i < (unsigned)_njoints; i++)
        {
            foc_based_info[i].hasTempSensor = 0;
            temperatureSensorsVector.at(i) = std::make_unique<eomc::TemperatureSensorNONE>();
        }
    }    
    else //I have the TemperatureSensorType
    {
        for (i = 1; i < xtmp.size(); i++)
        {
            std::string s = xtmp.get(i).asString();
            if(s == eomc::paramValues::TemperatureSensorType_PT100)
            {
                foc_based_info[i - 1].hasTempSensor = 1;
                temperatureSensorsVector.at(i-1) = std::make_unique<eomc::TemperatureSensorPT100>();
                
            }
            else if (s == eomc::paramValues::TemperatureSensorType_PT1000) 
            {
                
                foc_based_info[i - 1].hasTempSensor = 1;
                temperatureSensorsVector.at(i-1) = std::make_unique<eomc::TemperatureSensorPT1000>();
            }
            else if (s == eomc::paramValues::TemperatureSensorType_NONE)
            {
                foc_based_info[i - 1].hasTempSensor = 0;
                temperatureSensorsVector.at(i-1) = std::make_unique<eomc::TemperatureSensorNONE>();
            }
            else
            {
                yError() <<  _boardname << "Not supported TemperatureSensorType" << s << "Available TemperatureSensorType are: " << eomc::paramValues::TemperatureSensorType_PT100 << ", " << eomc::paramValues::TemperatureSensorType_PT1000 << " and " << eomc::paramValues::TemperatureSensorType_NONE;
                foc_based_info[i - 1].hasTempSensor = 0;
                temperatureSensorsVector.at(i-1) = std::make_unique<eomc::TemperatureSensorNONE>();
                return false;
            }
        }
    }

    // 3. HasRotorEncoder
    if (!extractGroup(focGroup, xtmp, "HasRotorEncoder", "HasRotorEncoder 0/1 ", _njoints))
    {
        return false;
    }
    else
    {

        for (i = 1; i < xtmp.size(); i++)
            foc_based_info[i - 1].hasRotorEncoder = xtmp.get(i).asInt32() != 0;
    }
    if (!extractGroup(focGroup, xtmp, "HasRotorEncoderIndex", "HasRotorEncoderIndex 0/1 ", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            foc_based_info[i - 1].hasRotorEncoderIndex = xtmp.get(i).asInt32() != 0;
    }

    if (!extractGroup(focGroup, xtmp, "Verbose", "Verbose 0/1 ", _njoints, false))
    {
        registerOptionalParameter("Verbose", "0 (disabled)", false);
        for (i = 0; i < (unsigned)_njoints; i++)
            foc_based_info[i].verbose = 0;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            foc_based_info[i - 1].verbose = xtmp.get(i).asInt32() != 0;
    }

	std::vector<int> AutoCalibration (_njoints);
    if (!extractGroup(focGroup, xtmp, "AutoCalibration", "AutoCalibration 0/1 ", _njoints, false))
    {
        registerOptionalParameter("AutoCalibration", "0 (disabled)", false);
        for (i = 0; i < (unsigned)_njoints; i++)
            AutoCalibration[i] = 0;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            AutoCalibration[i - 1] = xtmp.get(i).asInt32();
    }

    std::vector<float> Kbemf (_njoints);
    if (!extractGroup(focGroup, xtmp, "Kbemf", "Kbemf", _njoints, false))
    {
        registerOptionalParameter("Kbemf", "0.0f", false);
        for (i = 0; i < (unsigned)_njoints; i++)
            foc_based_info[i].kbemf = 0.0f;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            foc_based_info[i-1].kbemf = xtmp.get(i).asFloat32();
    }

    if (!extractGroup(focGroup, xtmp, "RotorIndexOffset", "RotorIndexOffset", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
        {
            if(AutoCalibration[i-1] == 0)
            {
                foc_based_info[i - 1].rotorIndexOffset = xtmp.get(i).asInt32();
                if (foc_based_info[i - 1].rotorIndexOffset <0 ||  foc_based_info[i - 1].rotorIndexOffset >359)
                {
                    yError() << "In " << _boardname << "joint " << i-1 << ": rotorIndexOffset should be in [0,359] range." ;
                    return false;
                }
            }
            else
            {
                yWarning() <<  "In " << _boardname << "joint " << i-1 << ": motor autocalibration is enabled!!! ATTENTION!!!" ;
                foc_based_info[i - 1].rotorIndexOffset = -1;
            }
        }
    }


    //Now I verify if rotor encoder hasn't index, then  rotor offset must be zero.
    for (i = 0; i < (unsigned)_njoints; i++)
    {
        if((0 == foc_based_info[i].hasRotorEncoderIndex) && (0 != foc_based_info[i].rotorIndexOffset))
        {
            yError() << "In " << _boardname << "joint " << i << ": inconsistent configuration: if rotor encoder hasn't index then its offset should be 0." ;
            return false;
        }
    }

    // Number of motor poles
    if (!extractGroup(focGroup, xtmp, "MotorPoles", "MotorPoles", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            foc_based_info[i - 1].motorPoles = xtmp.get(i).asInt32();
    }

    if (!extractGroup(focGroup, xtmp, "HasSpeedEncoder", "HasSpeedEncoder 0/1 ", _njoints))
    {
        yWarning () << "missing param HasSpeedEncoder";
        // optional by now
        //return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            foc_based_info[i - 1].hasSpeedEncoder = xtmp.get(i).asInt32() != 0;
    }

    return true;

}



//---------------------------------------------------
// Parse JointSets functions in config file 
// --------------------------------------------------
bool Parser::parseJointsetCfgGroup(yarp::os::Searchable &config, std::vector<JointsSet> &jsets, std::vector<int> &joint2set)
{
    Bottle jointsetcfg, xml;

    if(!GetGroupBottle(config, "JOINTSET_CFG", jointsetcfg)) return false;


    Bottle xtmp;
    int numofsets = 0;

    if(!extractGroup(jointsetcfg, xtmp, "numberofsets", "number of sets ", 1))
        return  false;
 
    numofsets = xtmp.get(1).asInt32();
    if((0 == numofsets) || (numofsets > _njoints))
    {
        yError() << "embObjMC BOARD " << _boardname << "Number of jointsets is not correct. it should belong to (1, " << _njoints << ")";
        return false;
    }

    if(!checkAndSetVectorSize(jsets, numofsets, "parseJointsetCfgGroup"))
        return false;

    if(!checkAndSetVectorSize(joint2set, _njoints, "parseJointsetCfgGroup"))
        return false;

    for(unsigned int s=0;s<(unsigned)numofsets;s++)
    {
        char jointset_string[80];
        sprintf(jointset_string, "JOINTSET_%d", s);
        bool formaterror = false;


        Bottle &js_cfg = jointsetcfg.findGroup(jointset_string);
        if(js_cfg.isNull())
        {
            yError() << "embObjMC BOARD " << _boardname << "cannot find " << jointset_string;
            return false;
        }

        //1) id of set
        jsets.at(s).id=s;


        //2) list of joints
        Bottle &b_listofjoints=js_cfg.findGroup("listofjoints", "list of joints");
        if (b_listofjoints.isNull())
        {
            yError() << "embObjMC BOARD " << _boardname << "listofjoints parameter not found";
            return false;
        }

        int numOfJointsInSet = b_listofjoints.size()-1;
        if((numOfJointsInSet < 1) || (numOfJointsInSet>_njoints))
        {
            yError() << "embObjMC BOARD " << _boardname << "numof joints of set " << s << " is not correct";
            return false;
        }


        for (int j = 0; j <numOfJointsInSet; j++)
        {
            int jointofthisset = b_listofjoints.get(j+1).asInt32();

            if((jointofthisset< 0) || (jointofthisset>_njoints))
            {
                yError() << "embObjMC BOARD " << _boardname << "invalid joint number for set " << s;
                return false;
            }

            jsets.at(s).joints.push_back(jointofthisset);

            //2.1) fill map joint to set
            joint2set.at(jointofthisset) = s;
        }

        // 3) constraints
        if(!extractGroup(js_cfg, xtmp, "constraint", "type of jointset constraint ", 1))
        {
            return  false;
        }

        eOmc_jsetconstraint_t constraint;
        if(!convert(xtmp.get(1).asString(), constraint, formaterror))
        {
            return false;
        }
        jsets.at(s).cfg.constraints.type = constraint;

        //param1
        if(!extractGroup(js_cfg, xtmp, "param1", "param1 of jointset constraint ", 1))
        {
            return  false;
        }
        jsets.at(s).cfg.constraints.param1 = (float)xtmp.get(1).asFloat64();

        //param2
        if(!extractGroup(js_cfg, xtmp, "param2", "param2 of jointset constraint ", 1))
        {
            return  false;
        }
        jsets.at(s).cfg.constraints.param2 = (float)xtmp.get(1).asFloat64();


    }
    return true;
}


//---------------------------------------------------
// Parse timeouts in config file 
// --------------------------------------------------
bool Parser::parseTimeoutsGroup(yarp::os::Searchable &config, std::vector<timeouts_t> &timeouts)
{

    Bottle timeoutsGroup, xtmp; 

    // for(auto i=0; i<_njoints; i++)
    // {
    //     yError() << "embObjMC BOARD " << _boardname << " joint " << i
    //              << " _defaultSettings.timeoutsDefaults:"
    //              << " velocity_ref=" << (double)_defaultSettings.timeoutsDefaults.velocity_ref << " (str: " << std::to_string(_defaultSettings.timeoutsDefaults.velocity_ref) << ")"
    //              << " current_ref="  << (double)_defaultSettings.timeoutsDefaults.current_ref  << " (str: " << std::to_string(_defaultSettings.timeoutsDefaults.current_ref) << ")"
    //              << " pwm_ref="      << (double)_defaultSettings.timeoutsDefaults.pwm_ref      << " (str: " << std::to_string(_defaultSettings.timeoutsDefaults.pwm_ref) << ")"
    //              << " torque_ref="   << (double)_defaultSettings.timeoutsDefaults.torque_ref   << " (str: " << std::to_string(_defaultSettings.timeoutsDefaults.torque_ref) << ")"
    //              << " torque_fbk="   << (double)_defaultSettings.timeoutsDefaults.torque_fbk   << " (str: " << std::to_string(_defaultSettings.timeoutsDefaults.torque_fbk) << ")";
    // }

    if(!GetGroupBottle(config, "TIMEOUTS", timeoutsGroup, false))
    {
        //yWarning() << "embObjMC BOARD " << _boardname << " TIMEOUTS group not found in configuration file. Using default values.";
        for(auto i=0; i<_njoints; i++)
        {
            timeouts[i].velocity_ref = _defaultSettings.timeoutsDefaults.velocity_ref;
            timeouts[i].current_ref  = _defaultSettings.timeoutsDefaults.current_ref;
            timeouts[i].pwm_ref      = _defaultSettings.timeoutsDefaults.pwm_ref;
            timeouts[i].torque_ref   = _defaultSettings.timeoutsDefaults.torque_ref;
            timeouts[i].torque_fbk   = _defaultSettings.timeoutsDefaults.torque_fbk;
        }
        return true;
    }


    if (!extractGroup(timeoutsGroup, xtmp, "velocity", "a list of timeout to be used in the vmo control", _njoints, false))
    {
        registerOptionalParameter("timeout.velocity", std::to_string(_defaultSettings.timeoutsDefaults.velocity_ref), true);
        for(auto i=0; i<_njoints; i++) timeouts[i].velocity_ref = _defaultSettings.timeoutsDefaults.velocity_ref;
    }
    else
    {
        for(auto i=0; i<_njoints; i++) timeouts[i].velocity_ref = xtmp.get(i+1).asInt32();
    }

    if (!extractGroup(timeoutsGroup, xtmp, "current", "a list of timeout to be used in the vmo control", _njoints, false))
    {
        registerOptionalParameter("timeout.current", std::to_string(_defaultSettings.timeoutsDefaults.current_ref), true);
        for(auto i=0; i<_njoints; i++) timeouts[i].current_ref = _defaultSettings.timeoutsDefaults.current_ref;
    }
    else
    {
        for(auto i=0; i<_njoints; i++) timeouts[i].current_ref = xtmp.get(i+1).asInt32();
    }

    if (!extractGroup(timeoutsGroup, xtmp, "pwm", "a list of timeout to be used in the vmo control", _njoints, false))
    {
        registerOptionalParameter("timeout.pwm", std::to_string(_defaultSettings.timeoutsDefaults.pwm_ref), true);
        for(auto i=0; i<_njoints; i++) timeouts[i].pwm_ref = _defaultSettings.timeoutsDefaults.pwm_ref;
    }
    else
    {
        for(auto i=0; i<_njoints; i++) timeouts[i].pwm_ref = xtmp.get(i+1).asInt32();
    }

    if (!extractGroup(timeoutsGroup, xtmp, "torque", "a list of timeout to be used in the vmo control", _njoints, false))
    {
        registerOptionalParameter("timeout.torque", std::to_string(_defaultSettings.timeoutsDefaults.torque_ref), true);
        for(auto i=0; i<_njoints; i++) timeouts[i].torque_ref = _defaultSettings.timeoutsDefaults.torque_ref;
    }
    else
    {
        for(auto i=0; i<_njoints; i++) timeouts[i].torque_ref = xtmp.get(i+1).asInt32();
    }

    if (!extractGroup(timeoutsGroup, xtmp, "torque_measure", "a list of timeout to be used in the vmo control", _njoints, false))
    {
        registerOptionalParameter("timeout.torque_measure", std::to_string(_defaultSettings.timeoutsDefaults.torque_fbk), true);
        for(auto i=0; i<_njoints; i++) timeouts[i].torque_fbk = _defaultSettings.timeoutsDefaults.torque_fbk;
    }
    else
    {
        for(auto i=0; i<_njoints; i++) timeouts[i].torque_fbk = xtmp.get(i+1).asInt32();
    }

    return true;
}

//---------------------------------------------------
// Parse current limits in config file 
// --------------------------------------------------
bool Parser::parseCurrentLimits(yarp::os::Searchable &config, std::vector<motorCurrentLimits_t> &currLimits)
{
    Bottle limits, xtmp; 
    if(!GetGroupBottle(config, "LIMITS", limits)) return false;

    // current limit
    if (!extractGroup(limits, xtmp, "motorOverloadCurrents","a list of current limits", _njoints))
        return false;

    for(auto i=1; i<xtmp.size(); i++) currLimits[i-1].overloadCurrent=xtmp.get(i).asFloat64();

    // nominal current
    if (!extractGroup(limits, xtmp, "motorNominalCurrents","a list of nominal current limits", _njoints))
        return false;
    for(auto i=1; i<xtmp.size(); i++) currLimits[i-1].nominalCurrent =xtmp.get(i).asFloat64();

    // peak current
    if (!extractGroup(limits, xtmp, "motorPeakCurrents","a list of peak current limits", _njoints))
        return false;
    for(auto i=1; i<xtmp.size(); i++) currLimits[i-1].peakCurrent=xtmp.get(i).asFloat64();

    return true;

}

//---------------------------------------------------
// Parse temperature limits in config file 
// --------------------------------------------------
bool Parser::parseTemperatureLimits(yarp::os::Searchable &config, std::vector<temperatureLimits_t> &temperatureLimits)
{
    Bottle limits, xtmp; 
    if(!GetGroupBottle(config, "LIMITS", limits)) return false;


    // hardware limit
    if(!extractGroup(limits, xtmp, "hardwareTemperatureLimits", "a list of temperature limits", _njoints, false))
     {
        registerOptionalParameter("hardwareTemperatureLimits", std::to_string(_defaultSettings.temperatureLimitsDefaults.hardware), false);
        // yWarning("hardwareTemperatureLimits param not found in config file for board %s. Please update robot configuration files or contact https://github.com/robotology/icub-support if needed. Using default values.", _boardname.c_str());
        for (auto i = 0; i < (unsigned)_njoints; i++)
        {
            temperatureLimits[i].hardware  = _defaultSettings.temperatureLimitsDefaults.hardware;
            temperatureLimits[i].warning = _defaultSettings.temperatureLimitsDefaults.warning;
        }
    }
    else
    {
        for (auto i = 1; i < xtmp.size(); i++) temperatureLimits[i - 1].hardware = xtmp.get(i).asFloat64();
        if (!extractGroup(limits, xtmp, "warningTemperatureLimits", "a list of warning temperature limits", _njoints, false))
        {
            // warning limit - parsing it only if hardwareTemperatureLimit available
            registerOptionalParameter("warningTemperatureLimits", std::to_string(_defaultSettings.temperatureLimitsDefaults.warning), false);
            //yWarning("warningTemperatureLimits param not found in config file for board %s. Please update robot configuration files or contact https://github.com/robotology/icub-support if needed. Using default values.", _boardname.c_str());

            for (auto i = 0; i < (unsigned)_njoints; i++) temperatureLimits[i].warning = _defaultSettings.temperatureLimitsDefaults.warning;
        }
        else
        {
            for (auto i = 1; i < xtmp.size(); i++) temperatureLimits[i - 1].warning = xtmp.get(i).asFloat64();
        }
    }
        

    
    //Now I verify that warning temperature limits is < 85% of hardwareTemperatureLimit.
    for (auto i = 0; i < (unsigned)_njoints; i++)
    {
        if(temperatureLimits[i].warning > (0.85 * temperatureLimits[i].hardware))
        {
            yError() << "In " << _boardname << "joint " << i << ": inconsistent temperature limits. warningTemperatureLimit must be smaller than 85% of hardwareTemperatureLimit" ;
            return false;
        }
    }
    return true;
}

//---------------------------------------------------
// Parse joint limits  in config file 
// --------------------------------------------------
bool Parser::parseJointsLimits(yarp::os::Searchable &config, std::vector<jointLimits_t> &jointsLimits)
{
    Bottle limits, xtmp; 
    if(!GetGroupBottle(config, "LIMITS", limits)) return false;

    // max limit
    if (!extractGroup(limits, xtmp, "jntPosMax","a list of user maximum angles (in degrees)", _njoints))
        return false;
    else
        for(auto i=1; i<xtmp.size(); i++) jointsLimits[i-1].posMax = xtmp.get(i).asFloat64();

    // min limit
    if (!extractGroup(limits, xtmp, "jntPosMin","a list of user minimum angles (in degrees)", _njoints))
        return false;
    else
        for(auto i=1; i<xtmp.size(); i++) jointsLimits[i-1].posMin = xtmp.get(i).asFloat64();

    // max hardware limit
    if (!extractGroup(limits, xtmp, "hardwareJntPosMax","a list of hardware maximum angles (in degrees)", _njoints))
        return false;
    else
    {
        for(auto i=1; i<xtmp.size(); i++) jointsLimits[i-1].posHwMax = xtmp.get(i).asFloat64();

        //check hardware limits are bigger then user limits
        for(auto i=0; i<(unsigned)_njoints; i++)
        {
            if(jointsLimits[i].posMax > jointsLimits[i].posHwMax)
            {
                yError() << "embObjMotionControl: user has set a limit  bigger then hardware limit!. Please check jntPosMax.";
                return false;
            }
        }
    }

    // min hardware limit
    if (!extractGroup(limits, xtmp, "hardwareJntPosMin","a list of hardware minimum angles (in degrees)", _njoints))
    {
        return false;
    }
    else
    {
        for(auto i=1; i<xtmp.size(); i++) jointsLimits[i-1].posHwMin = xtmp.get(i).asFloat64();

        //check hardware limits are bigger then user limits
        for(auto i=0; i<(unsigned)_njoints; i++)
        {
            if(jointsLimits[i].posMin < jointsLimits[i].posHwMin)
            {
                yError() << "embObjMotionControl: user has set a limit  bigger then hardware limit!. Please check jntPosMin.";
                return false;
            }
        }

    }

    // joint Velocity command max limit
    if (!extractGroup(limits, xtmp, "jntVelMax", "a list of maximum velocities for the joints (in degrees/s)", _njoints))
        return false;
    else
        for (auto i = 1; i<xtmp.size(); i++)     jointsLimits[i - 1].velMax = xtmp.get(i).asFloat64();

    return true;
}

//---------------------------------------------------
// Parse rotor limits in config file 
// --------------------------------------------------
bool Parser::parseRotorsLimits(yarp::os::Searchable &config, std::vector<rotorLimits_t> &rotorsLimits)
{
    Bottle limits, xtmp; 
    if(!GetGroupBottle(config, "LIMITS", limits)) return false;

    // Rotor max limit
    if (!extractGroup(limits, xtmp, "rotorPosMax","a list of maximum rotor angles (in degrees)", _njoints))
        return false;
    for(auto i=1; i<xtmp.size(); i++) rotorsLimits[i-1].posMax = xtmp.get(i).asFloat64();

    // Rotor min limit
    if (!extractGroup(limits, xtmp, "rotorPosMin","a list of minimum roto angles (in degrees)", _njoints))
        return false;
    for(auto i=1; i<xtmp.size(); i++) rotorsLimits[i-1].posMin = xtmp.get(i).asFloat64();

    // Motor pwm limit
    if (!extractGroup(limits, xtmp, "motorPwmLimit","a list of motor PWM limits", _njoints))
        return false;

    for(auto i=1; i<xtmp.size(); i++)
    {
        rotorsLimits[i-1].pwmMax = xtmp.get(i).asFloat64();
        if(rotorsLimits[i-1].pwmMax<0)
        {
            yError() << "In board " << _boardname << " motorPwmLimit should be a positive value";
            return false;
        }
    }
    return true;
}


//---------------------------------------------------
// Parse behavioral flags in config file 
// --------------------------------------------------
bool Parser::parseMotioncontrolVersion(yarp::os::Searchable &config, int &version)
{
    if (!config.findGroup("GENERAL").find("MotioncontrolVersion").isInt32())
    {
        yError() << "Missing MotioncontrolVersion parameter. RobotInterface cannot start. Please contact icub-support@iit.it";
        return false;
    }

    version = config.findGroup("GENERAL").find("MotioncontrolVersion").asInt32();
    return true;

}


bool Parser::isVerboseEnabled(yarp::os::Searchable &config)
{
    bool ret = false;
    if(!config.findGroup("GENERAL").find("verbose").isBool())
    {
        yError() << "embObjMotionControl::open() detects that general->verbose bool param is different from accepted values (true / false). Assuming false";
        ret = false;
    }
    else
    {
       ret = config.findGroup("GENERAL").find("verbose").asBool();
    }
    return ret;
}

bool Parser::parseBehaviourFalgs(yarp::os::Searchable &config, bool &useRawEncoderData, bool  &pwmIsLimited)
{

    // Check useRawEncoderData = do not use calibration data!
    Value use_raw = config.findGroup("GENERAL").find("useRawEncoderData");

    if(use_raw.isNull())
    {
        useRawEncoderData = false;
    }
    else
    {
        if(!use_raw.isBool())
        {
            yWarning() << "embObjMotionControl::open() detected that useRawEncoderData bool param is different from accepted values (true / false). Assuming false";
            useRawEncoderData = false;
        }
        else
        {
            useRawEncoderData = use_raw.asBool();
            if(useRawEncoderData)
            {
                yWarning() << "embObjMotionControl::open() detected that it is using raw data from encoders! Be careful  See 'useRawEncoderData' param in config file";
                yWarning() << "DO NOT USE OR CALIBRATE THE ROBOT IN THIS CONFIGURATION!";
                yWarning() << "CHECK IF THE FAULT BUTTON IS PRESSED and press ENTER to continue";
                getchar();
            }
        }
    }

    // Check useRawEncoderData = do not use calibration data!
    Value use_limitedPWM = config.findGroup("GENERAL").find("useLimitedPWM");
    if(use_limitedPWM.isNull())
    {
        pwmIsLimited = false;
    }
    else
    {
        if(!use_limitedPWM.isBool())
        {
            pwmIsLimited = false;
        }
        else
        {
            pwmIsLimited = use_limitedPWM.asBool();
        }
    }

    return true;
}

bool Parser::parseMaintenanceModeGroup(yarp::os::Searchable &config, bool &skipRecalibrationEnabled)
{
    // Extract group MaintenanceModeGroup
    Bottle &maintenanceGroup=config.findGroup("MAINTENANCE");
    if (maintenanceGroup.isNull())
    {
        skipRecalibrationEnabled = false;
        return true;
    }
    
    // Check skipRecalibrationEnabled = do not recalibrate joint at yarprobointerface restart! Used by low level motion controller
    Value skip_recalibration = maintenanceGroup.find("skipRecalibration");
    if (skip_recalibration.isNull())
    {
        skipRecalibrationEnabled = false;
    }
    else
    {
        if (!skip_recalibration.isBool())
        {
            yError() << "embObjMotionControl::open() detected that skipRecalibration bool param is different from accepted values (true / false). Assuming false";
            skipRecalibrationEnabled = false;
        }
        else
        {
            skipRecalibrationEnabled = skip_recalibration.asBool();
            if(skipRecalibrationEnabled)
            {
                yWarning() << "embObjMotionControl::open() detected that skipRecalibration is requested! Be careful  See 'skipRecalibration' param in config file";
                yWarning() << "THE ROBOT WILL SKIP THE RECALIBRATION IN THIS CONFIGURATION!";
            }
        }
    }
    return true;
}

//---------------------------------------------------
// Parse mechanical info in config file 
// --------------------------------------------------
bool Parser::parseAxisInfo(yarp::os::Searchable &config, int axisMap[], std::vector<axisInfo_t> &axisInfo)
{
    Bottle general, xtmp; 
    if(!GetGroupBottle(config, "GENERAL", general)) return false;

    if (!extractGroup(general, xtmp, "AxisMap", "a list of reordered indices for the axes", _njoints))
        return false;

    for (auto i = 1; i < xtmp.size(); i++)
    {
        int user_joint =  xtmp.get(i).asInt32();
        if(user_joint>= _njoints)
        {
            yError() << "embObjMC BOARD " << _boardname << "In AxisMap param: joint " << i-1 << "has been mapped to not-existing joint ("<< user_joint <<"). Here there are only "<< _njoints <<"joints";
            return false;
        }
        axisMap[i-1] = user_joint;
    }
    

    if (!extractGroup(general, xtmp, "AxisName", "a list of strings representing the axes names", _njoints))
        return false;

    //beware: axis name has to be remapped here because they are not set using the toHw() helper function
    for (auto i = 1; i < xtmp.size(); i++)
    {
        int mappedto = axisInfo[i-1].mappedto;
        axisInfo[axisMap[i - 1]].name = xtmp.get(i).asString();
    }

    if (!extractGroup(general, xtmp, "AxisType", "a list of strings representing the axes type (revolute/prismatic)", _njoints))
        return false;

    //beware: axis type has to be remapped here because they are not set using the toHw() helper function
    for (auto i = 1; i < xtmp.size(); i++)
    {
        string s = xtmp.get(i).asString();
        int mappedto = axisInfo[i-1].mappedto;
        if (s == eomc::paramValues::AxisType_revolute)  axisInfo[axisMap[i - 1]].type = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE;
        else if (s == eomc::paramValues::AxisType_prismatic)  axisInfo[axisMap[i - 1]].type = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_PRISMATIC;
        else
        {
            yError() << "embObjMC BOARD " << _boardname << "Unknown AxisType value" << s.c_str() << "!";
            axisInfo[axisMap[i - 1]].type = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_UNKNOWN;
            return false;
        }
    }

    return true;
}

bool Parser::parseEncoderFactor(yarp::os::Searchable &config, double encoderFactor[])
{
    Bottle general, xtmp; 
    if(!GetGroupBottle(config, "GENERAL", general)) return false;

    double tmp_A2E;

    // Encoder scales
    if (!extractGroup(general, xtmp, "Encoder", "a list of scales for the encoders", _njoints))
        return false;

    for (auto i = 1; i < xtmp.size(); i++)
    {
        tmp_A2E = xtmp.get(i).asFloat64();
        if (tmp_A2E<0)
        {
            yWarning() << "embObjMC BOARD " << _boardname << "Encoder parameter should be positive!";
        }
        encoderFactor[i - 1] = tmp_A2E;
    }

    return true;
}

bool Parser::parsefullscalePWM(yarp::os::Searchable &config, double dutycycleToPWM[])
{
    Bottle general, xtmp; 
    if(!GetGroupBottle(config, "GENERAL", general)) return false;

    double tmpval;

    // fullscalePWM
    if (!extractGroup(general, xtmp, "fullscalePWM", "a list of scales for the fullscalePWM conversion factor", _njoints))
        return false;

    for (auto i = 1; i < xtmp.size(); i++)
    {
        tmpval = xtmp.get(i).asFloat64();
        if (tmpval<0)
        {
            yError() << "embObjMC BOARD " << _boardname << "fullscalePWM parameter should be positive!";
            return false;
        }
        dutycycleToPWM[i - 1] = tmpval / 100.0;
    }

    return true;
}

bool Parser::parseAmpsToSensor(yarp::os::Searchable &config, double ampsToSensor[])
{
    Bottle general, xtmp; 
    if(!GetGroupBottle(config, "GENERAL", general)) return false;

    // ampsToSensor
    if (!extractGroup(general, xtmp, "ampsToSensor", "a list of scales for the ampsToSensor conversion factor", _njoints))
        return false;

    for (auto i = 1; i < xtmp.size(); i++)
    {
        double tmpval = xtmp.get(i).asFloat64();
        if (tmpval<0)
        {
            yError() << "embObjMC BOARD " << _boardname << "ampsToSensor parameter should be positive!";
            return false;
        }
        ampsToSensor[i - 1] = tmpval;
    }

    return true;
}

bool Parser::parseGearboxValues(yarp::os::Searchable &config, double gearbox_M2J[], double gearbox_E2J[])
{
    Bottle general, xtmp; 
    if(!GetGroupBottle(config, "GENERAL", general)) return false;

    // Gearbox_M2J
    if (!extractGroup(general, xtmp, "Gearbox_M2J", "The gearbox reduction ratio", _njoints))
        return false;

    for (auto i = 1; i < xtmp.size(); i++)
    {
        gearbox_M2J[i-1] = xtmp.get(i).asFloat64();
        if (gearbox_M2J[i-1]==0)
        {
            yError()  << "embObjMC BOARD " << _boardname << "Using a gearbox value = 0 may cause problems! Check your configuration files";
            return false;
        }
    }


    //Gearbox_E2J
    if (!extractGroup(general, xtmp, "Gearbox_E2J", "The gearbox reduction ratio between encoder and joint", _njoints))
        return false;

    int test = xtmp.size();
    for (auto i = 1; i < xtmp.size(); i++)
    {
        gearbox_E2J[i-1] = xtmp.get(i).asFloat64();
        if (gearbox_E2J[i-1]==0)
        {
            yError()  << "embObjMC BOARD " << _boardname << "Using a gearbox value = 0 may cause problems! Check your configuration files";
            return false;
        }
    }


    return true;
}

bool Parser::parseUseMotorSpeedFbkFlags(yarp::os::Searchable &config, int useMotorSpeedFbk[])
{
    Bottle general, xtmp; 
    if(!GetGroupBottle(config, "GENERAL", general)) return false;

    if(!extractGroup(general, xtmp, "useMotorSpeedFbk", "Use motor speed feedback", _njoints))
        return false;
    for (auto i = 1; i < xtmp.size(); i++)
    {
        useMotorSpeedFbk[i-1] = xtmp.get(i).asInt32();
    }

    return true;
}

bool Parser::parseCouplingInfo(yarp::os::Searchable &config, couplingInfo_t &couplingInfo)
{
    Bottle coupling_bottle, xtmp; 
    if(!GetGroupBottle(config, "COUPLINGS", coupling_bottle)) return false;

    int  fixedMatrix4X4Size = 16;
    int  fixedMatrix4X6Size = 24;
    bool formaterror =false;

    // matrix J2M
    if (!extractGroup(coupling_bottle, xtmp, "matrixJ2M", "matrixJ2M ", fixedMatrix4X4Size))
       return false;

    if(false == convert(xtmp, couplingInfo.matrixJ2M, formaterror, fixedMatrix4X4Size))
    {
       yError() << "embObjMC BOARD " << _boardname << " has detected an illegal format for some of the values of CONTROLLER.matrixJ2M";
       return false;
    }

    // matrix E2J
    if (!extractGroup(coupling_bottle, xtmp, "matrixE2J", "matrixE2J ", fixedMatrix4X6Size))
        return false;

    formaterror = false;
    if(false == convert(xtmp, couplingInfo.matrixE2J, formaterror, fixedMatrix4X6Size))
    {
        yError() << "embObjMC BOARD " << _boardname << " has detected an illegal format for some of the values of CONTROLLER.matrixE2J";
        return false;
    }

    // matrix M2J
    if (!extractGroup(coupling_bottle, xtmp, "matrixM2J", "matrixM2J ", fixedMatrix4X4Size))
        return false;

    formaterror = false;
    if( false == convert(xtmp, couplingInfo.matrixM2J, formaterror, fixedMatrix4X4Size))
    {
        yError() << "embObjMC BOARD " << _boardname << " has detected an illegal format for some of the values of CONTROLLER.matrixM2J";
        return false;
    }

    return true;
}

//---------------------------------------------------
// Parse other motion control info in config file 
// --------------------------------------------------
bool Parser::parseDeadzoneValue(yarp::os::Searchable &config, double deadzone[], bool *found)
{
    Bottle xtmp; 
    
    Bottle general = config.findGroup("OTHER_CONTROL_PARAMETERS");
    if (general.isNull())
    {
        // yWarning() << "embObjMC BOARD " << _boardname << "Missing OTHER_CONTROL_PARAMETERS.DeadZone parameter. I'll use default value. (see documentation for more datails)";
        *found = false;
        return true;
    }    

    // DeadZone
    if (!extractGroup(general, xtmp, "deadZone", "The deadzone of joint", _njoints, false))
    {
        *found = false;
        return true;
    }
 
    *found = true;
    for (auto i = 1; i < xtmp.size(); i++) deadzone[i-1] = xtmp.get(i).asFloat64();
    
    return true;
}

bool Parser::parseKalmanFilterParams(yarp::os::Searchable &config, std::vector<kalmanFilterParams_t> &kalmanFilterParams)
{
    
    Bottle general; 
    if(!GetGroupBottle(config, "KALMAN_FILTER", general, false/*Not mandatory*/))
    {
        registerOptionalParameter("kalmanFilter", "disabled", false);
 
        // if you don't specify the Kalman Filter group disable the kalmam filter for all the joints.
        // Get the default values structure from singleton
        
        for(int j=0; j<_njoints; j++)
        {
            kalmanFilterParams[j].enabled = _defaultSettings.controlParametersDefaults.kalmanFilterParams.enabled;
            kalmanFilterParams[j].x0 = _defaultSettings.controlParametersDefaults.kalmanFilterParams.x0;
            kalmanFilterParams[j].Q  = _defaultSettings.controlParametersDefaults.kalmanFilterParams.Q;
            kalmanFilterParams[j].R = _defaultSettings.controlParametersDefaults.kalmanFilterParams.R;
            kalmanFilterParams[j].P0 = _defaultSettings.controlParametersDefaults.kalmanFilterParams.P0;
        }
        return true;
    }
    else
    {    
        Bottle xtmp;
        
        // kalmanFilterEnabled
        if (!extractGroup(general, xtmp, "kalmanFilterEnabled", "kalman filter of joint: ", _njoints, true))
            return false;
        for (int j = 0; j<_njoints; j++) kalmanFilterParams[j].enabled = xtmp.get(j + 1).asBool();

        // x0_0
        if (!extractGroup(general, xtmp, "x0", "Initial state x0 of kalman filter of joint: ", _njoints, true))
            return false;
        for (int j = 0; j<_njoints; j++) kalmanFilterParams[j].x0.at(0) = xtmp.get(j + 1).asFloat32();

        // x0_1
        if (!extractGroup(general, xtmp, "x1", "Initial state x1 of kalman filter of joint: ", _njoints, true))
            return false;
        for (int j = 0; j<_njoints; j++) kalmanFilterParams[j].x0.at(1) = xtmp.get(j + 1).asFloat32();

        // x0_2
        if (!extractGroup(general, xtmp, "x2", "Initial state x2 of kalman filter of joint: ", _njoints, true))
            return false;
        for (int j = 0; j<_njoints; j++) kalmanFilterParams[j].x0.at(2) = xtmp.get(j + 1).asFloat32();

        // Q0
        if (!extractGroup(general, xtmp, "Q0", "Process Q0 noise covariance matrix of kalman filter of joint: ", _njoints, true))
            return false;
        for (int j = 0; j<_njoints; j++) kalmanFilterParams[j].Q.at(0) = xtmp.get(j + 1).asFloat32();

        // Q1
        if (!extractGroup(general, xtmp, "Q1", "Process Q1 noise covariance matrix of kalman filter of joint: ", _njoints, true))
            return false;
        for (int j = 0; j<_njoints; j++) kalmanFilterParams[j].Q.at(1) = xtmp.get(j + 1).asFloat32();

        // Q2
        if (!extractGroup(general, xtmp, "Q2", "Process Q2 noise covariance matrix of kalman filter of joint: ", _njoints, true))
            return false;
        for (int j = 0; j<_njoints; j++) kalmanFilterParams[j].Q.at(2) = xtmp.get(j + 1).asFloat32();

        // R
        if (!extractGroup(general, xtmp, "R", "Measurement noise covariance matrix of kalman filter for joint: ", _njoints, true))
            return false;
        for (int j = 0; j<_njoints; j++) kalmanFilterParams[j].R = xtmp.get(j + 1).asFloat32();

        // P0
        if (!extractGroup(general, xtmp, "P0", "Initial state estimation error covariance matrix of kalman filter of joint: ", _njoints, true))
            return false;
        for (int j = 0; j<_njoints; j++) kalmanFilterParams[j].P0 = xtmp.get(j + 1).asFloat32();
    }
    
    return true;
}

bool Parser::parseImpedanceGroup(yarp::os::Searchable &config,std::vector<impedanceParameters_t> &impedance)
{
    Bottle impedanceGroup, xtmp; 
    if(!GetGroupBottle(config, "IMPEDANCE", impedanceGroup)) return false;

  
    if (!extractGroup(impedanceGroup, xtmp, "stiffness", "stiffness parameter", _njoints))
        return false;

    for (auto j=0; j<_njoints; j++)
        impedance[j].stiffness = xtmp.get(j+1).asFloat64();

    if (!extractGroup(impedanceGroup, xtmp, "damping", "damping parameter", _njoints))
        return false;

    for (auto j=0; j<_njoints; j++)
        impedance[j].damping = xtmp.get(j+1).asFloat64();

     return true;
}

bool Parser::parseLugreGroup(yarp::os::Searchable &config,std::vector<lugreParameters_t> &lugre)
{
    Bottle lugreGroup, xtmp; 
    if(!GetGroupBottle(config, "LUGRE", lugreGroup, false/*Not mandatory*/)) 
    {

        for (int j = 0; j<_njoints; ++j)
        {
            lugre[j].Km = _defaultSettings.controlParametersDefaults.lugre.Km;
        }
        
        return true;
    }

    if (!extractGroup(lugreGroup, xtmp, "Km", "torque constant parameter", _njoints))
        return false;

    for (auto j=0; j<_njoints; j++)
        lugre[j].Km = xtmp.get(j+1).asFloat64();

    if (!extractGroup(lugreGroup, xtmp, "Kw", "viscous friction parameter", _njoints))
        return false;

    for (auto j=0; j<_njoints; j++)
        lugre[j].Kw = xtmp.get(j+1).asFloat64();

    if (!extractGroup(lugreGroup, xtmp, "S0", "hysteresis position parameter", _njoints))
        return false;

    for (auto j=0; j<_njoints; j++)
        lugre[j].S0 = xtmp.get(j+1).asFloat64();

    if (!extractGroup(lugreGroup, xtmp, "S1", "hysteresis velocity parameter", _njoints))
        return false;
        
    for (auto j=0; j<_njoints; j++)
        lugre[j].S1 = xtmp.get(j+1).asFloat64();
        
    if (!extractGroup(lugreGroup, xtmp, "Vth", "velocity threshold parameter", _njoints))
        return false;

    for (auto j=0; j<_njoints; j++)
        lugre[j].Vth = xtmp.get(j+1).asFloat64();

    if (!extractGroup(lugreGroup, xtmp, "Fc_pos", "Coulomb force parameter (forward)", _njoints))
        return false;
                
    for (auto j=0; j<_njoints; j++)
        lugre[j].Fc_pos = xtmp.get(j+1).asFloat64();
        
    if (!extractGroup(lugreGroup, xtmp, "Fc_neg", "Coulomb force parameter (backward)", _njoints))
        return false;
                
    for (auto j=0; j<_njoints; j++)
        lugre[j].Fc_neg = xtmp.get(j+1).asFloat64();

    if (!extractGroup(lugreGroup, xtmp, "Fs_pos", "Stribeck force parameter (forward)", _njoints))
        return false;
                
    for (auto j=0; j<_njoints; j++)
        lugre[j].Fs_pos = xtmp.get(j+1).asFloat64();
        
    if (!extractGroup(lugreGroup, xtmp, "Fs_neg", "Stribeck force parameter (backward)", _njoints))
        return false;
                
    for (auto j=0; j<_njoints; j++)
        lugre[j].Fs_neg = xtmp.get(j+1).asFloat64();

        return true;

}




bool Parser::convert(std::string const &fromstring, eOmc_jsetconstraint_t &jsetconstraint, bool& formaterror)
{
    const char *t = fromstring.c_str();

    eObool_t usecompactstring = eobool_false;
    jsetconstraint = eomc_string2jsetconstraint(t, usecompactstring);

    if(eomc_jsetconstraint_unknown == jsetconstraint)
    {
        usecompactstring = eobool_true;
        jsetconstraint = eomc_string2jsetconstraint(t, usecompactstring);
    }

    if(eomc_jsetconstraint_unknown == jsetconstraint)
    {
        yError() << "embObjMC BOARD " << _boardname << "String" << t << "cannot be converted into a proper eOmc_jsetconstraint_t";
        formaterror = true;
        return false;
    }

    return true;
}

bool Parser::convert(Bottle &bottle, vector<double> &matrix, bool &formaterror, int targetsize)
{
    matrix.resize(0);

    int tmp = bottle.size();
    int sizeofmatrix = tmp - 1;    // first position of bottle contains the tag "matrix"

    // check if there are really the target number of elements in matrix.
    if(targetsize != sizeofmatrix)
    {
        yError() << "embObjMC BOARD " << _boardname << " in converting string do matrix.In the matrix there are not" << targetsize << "elements";
        return false;
    }

    formaterror = false;
    for(int i=0; i<sizeofmatrix; i++)
    {
        double item = 0;

        // ok, i use the standard converter ... but what if it is not a double format? so far we dont check.
        item = bottle.get(i+1).asFloat64();
        matrix.push_back(item);
    }

    // in here we could decide to return false if any previous conversion function has returned error

    return true;
}



//////////////////////////////////////////////////////////////////////////////
/////////////////// DEBUG FUNCTIONS
////////////////////////////////////////////////////////////////////////////////
// void Parser::debugUtil_printControlLaws(void)
// {
//     //////// debug prints
//     yError() << "position control law: ";
//     for(int x=0; x<_njoints; x++)
//     {
//         yError() << " - j " << x << _positionControlLaw[x].c_str();
//     }

//     yError() << "velocity control law: ";
//     for(int x=0; x<_njoints; x++)
//     {
//         yError() << "- j " << x << _velocityControlLaw[x].c_str();
//     }


//     yError() << "torque control law: ";
//     for(int x=0; x<_njoints; x++)
//     {
//         yError() << " - j " << x << _torqueControlLaw[x].c_str();
//     }
//     //////end

// }

/*
void PidInfo::dumpdata(void)
    if(!GetGroupBottle(config, "LUGRE", lugreGroup, false)) 

    cout <<  "Is enabled " << enabled;
    cout <<  ". Username pid selected is " << usernamePidSelected;
    switch(controlLaw)
    {
        case PidAlgo_simple:
            cout <<  ". Control law is " << "PidAlgo_simple";
            break;

        case PIdAlgo_velocityInnerLoop:
            cout <<  ". Control law is " << "PIdAlgo_velocityInnerLoop";
            break;

        case PidAlgo_currentInnerLoop:
            cout <<  ". Control law is " << "PidAlgo_currentInnerLoop";
            break;
        default :
            cout <<  ". Control law is " << "unknown";
    }

    cout << ". PID fbk Unit type is " << (int)fbk_PidUnits;
    cout << ". PID out Unit type is " << (int)out_PidUnits;

    cout << " kp is " << pid.kp;
    cout << endl;

}
*/
void JointsSet::dumpdata(void)
{
    switch(cfg.constraints.type)
    {
        case eomc_jsetconstraint_none:
            cout <<  "constraint is " << "eomc_jsetconstraint_none";
            break;
        case eomc_jsetconstraint_cerhand:
            cout <<  "constraint is " << "eomc_jsetconstraint_cerhand";
             break;
        case eomc_jsetconstraint_trifid:
            cout <<  "constraint is " << "eomc_jsetconstraint_trifid";
             break;
      default :
            cout <<  ". constraint is " << "unknown";
    }

    cout << " param1="<< cfg.constraints.param1 << " param2=" << cfg.constraints.param2 << endl;

}

// bool Parser::checkJointTypes(PidInfo *pids, const std::string &pid_type)
// {
//     //Here we check that all joints have same type units in order to create pid_type helper with correct factor.

//     int firstjoint = -1;

//     // verify if pid type is torque or some other
//     if(pid_type == "TORQUE")
//     {
//         // since we are working with a torque pid, we first cast it as such
//         // this allows the loop to correctly point to the corresponding memory
//         TrqPidInfo* trq_pids = (TrqPidInfo*) pids;

//         for(int i=0; i<_njoints; i++)
//         {
//             // if we already had an enabled PID, compare with current one
//             if(firstjoint != -1 && !checkSinglePid(trq_pids[firstjoint], trq_pids[i], firstjoint, i, pid_type))
//             {
//                 return false;
//             }
//             // if we haven't found an enabled PID yet, and this one is enabled, save it
//             if(firstjoint == -1 && trq_pids[i].enabled)
//             {
//                 firstjoint = i;
//             }
//         }
//     }
//     else
//     {
//         for(int i=0; i<_njoints; i++)
//         {
//             // if we already had an enabled PID, compare with current one
//             if(firstjoint != -1 && !checkSinglePid(pids[firstjoint], pids[i], firstjoint, i, pid_type))
//             {
//                 return false;
//             }
//             // if we haven't found an enabled PID yet, and this one is enabled, save it
//             if(firstjoint == -1 && pids[i].enabled)
//             {
//                 firstjoint = i;
//             }
//         }
//     }

//     return true;
// }

// bool Parser::checkSinglePid(PidInfo &firstPid, PidInfo &currentPid, const int &firstjoint, const int &currentjoint, const std::string &pid_type)
// {
//     // check if the PID we are checking is enabled
//     if(currentPid.enabled)
//     {
//         // if it has different unit types from the previous enabled PIDs
//         if(firstPid.fbk_PidUnits != currentPid.fbk_PidUnits ||
//         firstPid.out_PidUnits != currentPid.out_PidUnits)
//         {
//             yError() << "embObjMC BOARD " << _boardname << "all joints with " << pid_type << " enabled should have same controlunits type. Joint " << firstjoint << " differs from joint " << currentjoint;
//             return false;
//         }
//     }
//     return true;
// }

void Parser::registerOptionalParameter(std::string_view paramName, std::string_view defaultValue, bool wasUsed)
{
    // Create a new entry for this optional parameter
    OptionalParameterInfo_t param_info;
    param_info.parameterName = paramName;
    param_info.defaultValue = defaultValue;
    param_info.wasUsed = wasUsed;
    
    // Add to tracking vector
    _optionalParametersUsed.push_back(param_info);
}

void Parser::printOptionalParametersTable()
{
    if(_optionalParametersUsed.empty())
    {
        return;
    }

    // Build the table header
    std::string line;
    line = "\n";
    line += "╔═══════════════════════════════════════════════════════════════════╗\n";
    size_t pad = (_boardname.length() < 16) ? (16 - _boardname.length()) : 0;
    line += "║     OPTIONAL PARAMETERS SUMMARY FOR BOARD: " + _boardname + std::string(pad, ' ') + "║\n";
    line += "╠═══════════════════════════════════════════════════════════════════╣\n";
    line += "║ Parameter Name                │ Used │ Default Value              │\n";
    line += "╟───────────────────────────────┼──────┼────────────────────────────╢\n";
    
    // Add rows for each parameter
    for(const auto &param : _optionalParametersUsed)
    {
        // Parameter name (max 31 chars)
        std::string param_name = param.parameterName;
        if(param_name.length() > 31) param_name = param_name.substr(0, 28) + "...";
        std::string param_name_padded = param_name + std::string(31 - param_name.length(), ' ');
        
        // Used flag
        std::string used_str = param.wasUsed ? "YES" : "NO";
        std::string used_padded = used_str + std::string(4 - used_str.length(), ' ');
        
        // Default value (max 27 chars)
        std::string default_val = param.defaultValue;
        if(default_val.length() > 27) default_val = default_val.substr(0, 24) + "...";
        std::string default_val_padded = default_val + std::string(27 - default_val.length(), ' ');
        
        line += "║ " + param_name_padded + "│ " + used_padded + " │ " + default_val_padded + "│\n";
    }
    
    // Close the table
    line += "╚═══════════════════════════════════════════════════════════════════╝\n";

    // Print using yCWarning as requested
    yWarning() << line.c_str();

}

void Parser::clearOptionalParametersTrack()
{
    _optionalParametersUsed.clear();
}






