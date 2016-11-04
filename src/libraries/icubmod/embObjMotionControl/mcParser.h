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





//  Yarp stuff
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/dev/ControlBoardPid.h>


#include "EoMotionControl.h"
#include <yarp/dev/ControlBoardHelper.h>


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
    controlUnits_metric = 1
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
};

class eomcParser_pidInfo
{
public:

    yarp::dev::Pid pid;
    GenericControlUnitsType_t ctrlUnitsType;
    PidAlgorithmType_t controlLaw;
    string usernamePidSelected;
    bool enabled;

};

class eomcParser_trqPidInfo : public eomcParser_pidInfo
{

    double kbemf;                             /** back-emf compensation parameter */
    double ktau;                              /** motor torque constant */
    int    filterType;
};

class mcParser
{

private:
    int _njoints;
    string _boardname;

    map<string, Pid_Algorithm*> posAlgoMap;
    map<string, Pid_Algorithm*> velAlgoMap;
    map<string, Pid_Algorithm*> trqAlgoMap;

    string  *_posistionControlLaw;
    string  *_velocityControlLaw;
    string  *_torqueControlLaw;

    double *_kbemf;                             /** back-emf compensation parameter */
    double *_ktau;                              /** motor torque constant */
    int * _filterType;

    bool parseSelectedPositionControl(yarp::os::Searchable &config);
    bool parseSelectedVelocityControl(yarp::os::Searchable &config);
    bool parseSelectedTorqueControl(yarp::os::Searchable &config);
    bool parsePid_inPos_outPwm(Bottle &b_pid, string controlLaw);
    bool parsePid_inVel_outPwm(Bottle &b_pid, string controlLaw);
    bool parsePid_inTrq_outPwm(Bottle &b_pid, string controlLaw);
    bool parsePidPos_withInnerVelPid(Bottle &b_pid, string controlLaw);
    bool parsePidTrq_withInnerVelPid(Bottle &b_pid, string controlLaw);
    bool getCorrectPidForEachJoint(eomcParser_pidInfo *ppids, eomcParser_pidInfo *vpids, eomcParser_trqPidInfo *tpids);
    bool parsePidUnitsType(Bottle &bPid, GenericControlUnitsType_t &unitstype);


    bool pidsAreEquals(Pid &pid1, Pid &pid2); //to move in yarp pid (override = operator)


    ///////// DEBUG FUNCTIONS
    void debugUtil_printControlLaws(void);


public:
    mcParser(int numofjoints, string boardname);
    ~mcParser();
    bool parsePids(yarp::os::Searchable &config, eomcParser_pidInfo *ppids, eomcParser_pidInfo *vpids, eomcParser_trqPidInfo *tpids);

};

#endif // include guard
