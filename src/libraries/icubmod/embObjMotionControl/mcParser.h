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



#ifndef __mcParserh__
#define __mcParserh__



#include <string>
#include <map>

//  Yarp stuff
#include <yarp/os/Bottle.h>
//#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/ControlBoardPid.h>
#include <yarp/dev/ControlBoardHelper.h>

#include "EoMotionControl.h"



// - public #define  --------------------------------------------------------------------------------------------------



typedef enum
{
    PidAlgo_simple = 0,
    PIdAlgo_velocityInnerLoop = 1,
    PidAlgo_currentInnerLoop =2
} PidAlgorithmType_t;


typedef enum
{
    controlUnits_machine = 0,
    controlUnits_metric = 1,
    controlUnits_unknown = 255
} GenericControlUnitsType_t;

class Pid_Algorithm
{
public:
    PidAlgorithmType_t type;
    GenericControlUnitsType_t ctrlUnitsType;

};


class Pid_Algorithm_simple: public Pid_Algorithm
{
public:
    yarp::dev::Pid *pid;
    Pid_Algorithm_simple(int nj)
    {
        pid = allocAndCheck<yarp::dev::Pid>(nj);
    };
    ~Pid_Algorithm_simple()
    {
        checkAndDestroy(pid);
    };

};

class PidAlgorithm_VelocityInnerLoop: public Pid_Algorithm
{
public:
    yarp::dev::Pid *extPid; //pos, trq, velocity
    yarp::dev::Pid *innerVelPid;
    PidAlgorithm_VelocityInnerLoop(int nj)
    {
        extPid = allocAndCheck<yarp::dev::Pid>(nj);
        innerVelPid = allocAndCheck<yarp::dev::Pid>(nj);
    };
    ~PidAlgorithm_VelocityInnerLoop()
    {
        checkAndDestroy(extPid);
        checkAndDestroy(innerVelPid);
    }
};


// class PidAlgorithm_CurrentInnerLoop: public Pid_Algorithm
// {
// public:
//     yarp::dev::Pid *extPid;
//     yarp::dev::Pid *innerCurrLoop;
//     PidAlgorithm_CurrentInnerLoop(int nj)
//     {
//         extPid = allocAndCheck<yarp::dev::Pid>(nj);
//         innerCurrLoop = allocAndCheck<yarp::dev::Pid>(nj);
//     };
//     ~PidAlgorithm_CurrentInnerLoop()
//     {
//         checkAndDestroy(extPid);
//         checkAndDestroy(innerCurrLoop);
//     }
// };

class eomcParser_pidInfo
{
public:

    yarp::dev::Pid pid;
    GenericControlUnitsType_t ctrlUnitsType;
    PidAlgorithmType_t controlLaw;
    std::string usernamePidSelected;
    bool enabled;

    void dumpdata(void);

};

class eomcParser_trqPidInfo : public eomcParser_pidInfo
{
public:
    double kbemf;                             /** back-emf compensation parameter */
    double ktau;                              /** motor torque constant */
    int    filterType;
};



typedef struct
{
    bool hasHallSensor;
    bool hasTempSensor;
    bool hasRotorEncoder;
    bool hasRotorEncoderIndex;
    int  rotorIndexOffset;
    int  motorPoles;
    bool hasSpeedEncoder ; //facoltativo
} eomc_twofocSpecificInfo;


class mcParser
{

private:
    int _njoints;
    std::string _boardname;

    std::map<std::string, Pid_Algorithm*> posAlgoMap;
    std::map<std::string, Pid_Algorithm*> velAlgoMap;
    std::map<std::string, Pid_Algorithm*> trqAlgoMap;

    std::string  *_posistionControlLaw;
    std::string  *_velocityControlLaw;
    std::string  *_torqueControlLaw;

    double *_kbemf;                             /** back-emf compensation parameter */
    double *_ktau;                              /** motor torque constant */
    int * _filterType;




    //PID parsing functions
    bool parseControlsGroup(yarp::os::Searchable &config);
    bool parseSelectedPositionControl(yarp::os::Searchable &config);
    bool parseSelectedVelocityControl(yarp::os::Searchable &config);
    bool parseSelectedTorqueControl(yarp::os::Searchable &config);
    bool parsePid_inPos_outPwm(yarp::os::Bottle &b_pid, std::string controlLaw);
    bool parsePid_inVel_outPwm(yarp::os::Bottle &b_pid, std::string controlLaw);
    bool parsePid_inTrq_outPwm(yarp::os::Bottle &b_pid, std::string controlLaw);
    bool parsePidPos_withInnerVelPid(yarp::os::Bottle &b_pid, std::string controlLaw);
    bool parsePidTrq_withInnerVelPid(yarp::os::Bottle &b_pid, std::string controlLaw);
    bool parsePidsGroup(yarp::os::Bottle& pidsGroup, yarp::dev::Pid myPid[], std::string prefix);
    bool getCorrectPidForEachJoint(eomcParser_pidInfo *ppids, eomcParser_pidInfo *vpids, eomcParser_trqPidInfo *tpids);
    bool parsePidUnitsType(yarp::os::Bottle &bPid, GenericControlUnitsType_t &unitstype);


    bool pidsAreEquals(yarp::dev::Pid &pid1, yarp::dev::Pid &pid2); //to move in yarp pid (override = operator)

    //general utils functions
    bool extractGroup(yarp::os::Bottle &input, yarp::os::Bottle &out, const std::string &key1, const std::string &txt, int size);

    ///////// DEBUG FUNCTIONS
    void debugUtil_printControlLaws(void);


public:
    mcParser(int numofjoints, std::string boardname);
    ~mcParser();
    bool parsePids(yarp::os::Searchable &config, eomcParser_pidInfo *ppids, eomcParser_pidInfo *vpids, eomcParser_trqPidInfo *tpids);
    bool parse2FocGroup(yarp::os::Searchable &config, eomc_twofocSpecificInfo *twofocinfo);
    bool parseCurrentPid(yarp::os::Searchable &config, eomcParser_pidInfo *cpids);

};

#endif // include guard
