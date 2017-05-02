// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
* Authors: Alberto Cardellino
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*
*/

//#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Bottle.h>
#include <string.h>
#include <iostream>



#include <mcParser.h>

#include <yarp/os/LogStream.h>

#include "EoCommon.h"
#include "EOarray.h"
#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolMC.h"
#include "EoProtocolAS.h"


using namespace std;
using namespace yarp::dev;
using namespace yarp::os;


mcParser::mcParser(int numofjoints, string boardname)
{
    _njoints = numofjoints;
    _boardname = boardname;
    _verbosewhenok = 0;
    _posistionControlLaw.resize(0);
    _velocityControlLaw.resize(0);
    _torqueControlLaw.resize(0);
    _currentControlLaw.resize(0);

    _kbemf=allocAndCheck<double>(_njoints);
    _ktau=allocAndCheck<double>(_njoints);
    _filterType=allocAndCheck<int>(_njoints);

    posAlgoMap.clear();
    velAlgoMap.clear();
    trqAlgoMap.clear();


};

mcParser::~mcParser()
{
    checkAndDestroy(_kbemf);
    checkAndDestroy(_ktau);
    checkAndDestroy(_filterType);
}


bool mcParser::parsePids(yarp::os::Searchable &config, eomcParser_pidInfo *ppids, eomcParser_pidInfo *vpids, eomcParser_trqPidInfo *tpids, eomcParser_pidInfo *cpids, bool currentPidisMandatory)
{

    if(!parseControlsGroup(config))
        return false;
    if(!parseSelectedCurrentPid(config, currentPidisMandatory, cpids))
        return false;
    if(!parseSelectedPositionControl(config))
        return false;
    if(!parseSelectedVelocityControl(config))
        return false;
    if(!parseSelectedTorqueControl(config))
        return false;

    if(!getCorrectPidForEachJoint(ppids, vpids, tpids))
        return false;


    return true;
}

bool mcParser::parseControlsGroup(yarp::os::Searchable &config)
{
    Bottle xtmp;
    int i;

    Bottle controlsGroup = config.findGroup("CONTROLS", "Configuration of used control laws ");
    if(controlsGroup.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << " no CONTROLS group found in config file, returning";
        return false;
    }

    if (!extractGroup(controlsGroup, xtmp, "positionControl", "Position Control ", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            //_posistionControlLaw[i - 1] = xtmp.get(i).asString();
            _posistionControlLaw.push_back(xtmp.get(i).asString().c_str());
    }

    if (!extractGroup(controlsGroup, xtmp, "velocityControl", "Velocity Control ", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            _velocityControlLaw.push_back(xtmp.get(i).asString().c_str());
    }

    if (!extractGroup(controlsGroup, xtmp, "torqueControl", "Torque Control ", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            _torqueControlLaw.push_back(xtmp.get(i).asString().c_str());
    }

    if (!extractGroup(controlsGroup, xtmp, "currentPid", "Current Pid ", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            _currentControlLaw.push_back(xtmp.get(i).asString().c_str());
    }


    return true;

}

bool mcParser::parseSelectedCurrentPid(yarp::os::Searchable &config, bool currentPidisMandatory, eomcParser_pidInfo *cpids)
{
    //first of all verify current pid has been configured if it is mandatory
    for(int i=0; i<_njoints; i++)
    {
        if(currentPidisMandatory)
        {
            if(_currentControlLaw[i] == "none")
            {
                yError() << "embObjMC BOARD " << _boardname << "CuuentPid is mandatory. It shlould be different from none ";
                return false;
            }
            if(_currentControlLaw[i] != _currentControlLaw[0])
            {
                yError() << "embObjMC BOARD " << _boardname << "all joints should have same current law ";
                return false;
            }
        }
    }

    if(_currentControlLaw[0]=="none")
    {
       yDebug() << "embObjMC BOARD " << _boardname << "No current control found "; 
       return true;
    }

    // 1) verify that selected control law is defined in file
    Bottle currControlLaw = config.findGroup(_currentControlLaw[0]);
    if (currControlLaw.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << "Missing " << _currentControlLaw[0].c_str();
        return false;
    }

    // 2) read control_law
    Value &controlLaw=currControlLaw.find("controlLaw");
    if( (controlLaw.isNull()) || (! controlLaw.isString()) )
    {
        yError() << "embObjMC BOARD " << _boardname << "Unable read control law parameter for " << _currentControlLaw[0].c_str() <<". Quitting.";
        return false;
    }

    string s_controlaw = controlLaw.toString();
    if (s_controlaw != string("limitscurrent"))
    {
        yError() << "embObjMC BOARD " << _boardname << "Unable to use control law " << s_controlaw << " for current pid. Quitting.";
        return false;
    }

    yarp::dev::Pid *mycpids =  new yarp::dev::Pid[_njoints];

    GenericControlUnitsType_t unitstype;
    if(!parsePidUnitsType(currControlLaw, unitstype))
        return false;

    if(unitstype != controlUnits_metric)
    {
        yError() << "embObjMC BOARD " << _boardname << " current pids can use only metric units";
        return false;
    }

    if(!parsePidsGroup(currControlLaw, mycpids, string("cur_")))
        return false;

    for(int i=0; i<_njoints; i++)
    {
        cpids[i].enabled = true;
        cpids[i].ctrlUnitsType = unitstype;
        cpids[i].controlLaw = PidAlgo_simple;
        cpids[i].pid = mycpids[i];
    }

    delete[] mycpids;

    return true;

}


bool mcParser::parseSelectedPositionControl(yarp::os::Searchable &config)
{
    for(int i=0; i<_njoints; i++)
    {
        // 1) verify that selected control law is defined in file
        Bottle posControlLaw = config.findGroup(_posistionControlLaw[i]);
        if (posControlLaw.isNull())
        {
            yError() << "embObjMC BOARD " << _boardname << "Missing " << _posistionControlLaw[i].c_str();
            return false;
        }

        // 2) read control_law
        Value &controlLaw=posControlLaw.find("controlLaw");
        if( (controlLaw.isNull()) || (! controlLaw.isString()) )
        {
            yError() << "embObjMC BOARD " << _boardname << "Unable read control law parameter for " << _posistionControlLaw[i].c_str() <<". Quitting.";
            return false;
        }

        string s_controlaw = controlLaw.toString();
        if (s_controlaw==string("Pid_inPos_outPwm"))
        {
            if (!parsePid_inPos_outPwm(posControlLaw, _posistionControlLaw[i]))
            {
                yError() << "embObjMC BOARD " << _boardname << "format error in Pid_inPos_outPwm";
                return false;
            }
        }
        else if (s_controlaw==string("PidPos_withInnerVelPid"))
        {
            if (!parsePidPos_withInnerVelPid(posControlLaw, _posistionControlLaw[i]))
            {
                yError() << "embObjMC BOARD " << _boardname << "format error in PidPos_withInnerVelPid";
                return false;
            }
        }
        else
        {
            yError() << "embObjMC BOARD " << _boardname << "Unable to use control law " << s_controlaw << " por position control. Quitting.";
            return false;
        }

    }
    return true;

}


bool mcParser::parseSelectedVelocityControl(yarp::os::Searchable &config)
{
    for(int i=0; i<_njoints; i++)
    {
        if(_velocityControlLaw[i] == "none")
        {
            continue;
        }

        // 1) verify that selected control law is defined in file
        Bottle velControlLaw = config.findGroup(_velocityControlLaw[i]);
        if (velControlLaw.isNull())
        {
           yError() << "embObjMC BOARD " << _boardname << "Missing " << _velocityControlLaw[i].c_str();
            return false;
        }

        // 2) read control_law
        Value &controlLaw=velControlLaw.find("controlLaw");
        if( (controlLaw.isNull()) || (! controlLaw.isString()) )
        {
           yError() << "embObjMC BOARD " << _boardname << "Unable read control law parameter for " << _velocityControlLaw[i].c_str() <<". Quitting.";
            return false;
        }

        string s_controlaw = controlLaw.toString();
        if (s_controlaw==string("Pid_inVel_outPwm"))
        {
            if (!parsePid_inVel_outPwm(velControlLaw, _velocityControlLaw[i]))
            {
                yError() << "embObjMC BOARD " << _boardname << "format error in Pid_inVel_outPwm";
                return false;
            }

        }
        else
        {
           yError() << "embObjMC BOARD " << _boardname << "Unable to use control law " << s_controlaw << " for velocity control. Quitting.";
            return false;
        }

    }
    return true;

}

bool mcParser::parseSelectedTorqueControl(yarp::os::Searchable &config)
{
    for(int i=0; i<_njoints; i++)
    {
        if(_torqueControlLaw[i] == "none")
        {
            continue;
        }
        // 1) verify that selected control law is defined in file
        Bottle trqControlLaw = config.findGroup(_torqueControlLaw[i]);
        if (trqControlLaw.isNull())
        {
            yError() << "embObjMC BOARD " << _boardname << "Missing " << _torqueControlLaw[i].c_str();
            return false;
        }

        // 2) read control_law
        Value &controlLaw=trqControlLaw.find("controlLaw");
        if( (controlLaw.isNull()) || (! controlLaw.isString()) )
        {
            yError() << "embObjMC BOARD " << _boardname << "Unable read control law parameter for " << _torqueControlLaw[i].c_str() <<". Quitting.";
            return false;
        }

        string s_controlaw = controlLaw.toString();
        if (s_controlaw==string("Pid_inTrq_outPwm"))
        {
            if (!parsePid_inTrq_outPwm(trqControlLaw, _torqueControlLaw[i]))
            {
                yError() << "embObjMC BOARD " << _boardname << " format error in Pid_inTrq_outPwm";
                return false;
            }

        }
        else if (s_controlaw==string("PidTrq_withInnerVelPid"))
        {
            if (!parsePidTrq_withInnerVelPid(trqControlLaw,_torqueControlLaw[i] ))
            {
                yError() << "embObjMC BOARD " << _boardname << " format error in PidTrq_withInnerVelPid";
                return false;
            }

        }
        else
        {
            yError() << "embObjMC BOARD " << _boardname << "Unable to use control law " << s_controlaw << " for torque control. Quitting.";
            return false;
        }

    }
    return true;

}


bool mcParser::parsePidsGroup(Bottle& pidsGroup, Pid myPid[], string prefix)
{
    int j=0;
    Bottle xtmp;

    if (!extractGroup(pidsGroup, xtmp,  prefix + string("kp"), "Pid kp parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].kp = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp,  prefix + string("kd"), "Pid kd parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].kd = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp,  prefix + string("ki"), "Pid kp parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].ki = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp,  prefix + string("maxInt"), "Pid maxInt parameter", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].max_int = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp,  prefix + string("maxOutput"), "Pid maxOutput parameter", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].max_output = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp,  prefix + string("shift"), "Pid shift parameter", _njoints))     return false; for (j=0; j<_njoints; j++) myPid[j].scale = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp,  prefix + string("ko"), "Pid ko parameter", _njoints))           return false; for (j=0; j<_njoints; j++) myPid[j].offset = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp,  prefix + string("stictionUp"), "Pid stictionUp", _njoints))     return false; for (j=0; j<_njoints; j++) myPid[j].stiction_up_val = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp,  prefix + string("stictionDwn"), "Pid stictionDwn", _njoints))   return false; for (j=0; j<_njoints; j++) myPid[j].stiction_down_val = xtmp.get(j+1).asDouble();
    if (!extractGroup(pidsGroup, xtmp,  prefix + string("kff"), "Pid kff parameter", _njoints))         return false; for (j=0; j<_njoints; j++) myPid[j].kff = xtmp.get(j+1).asDouble();

    return true;
}

bool mcParser::extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    size++;
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        yError () << key1.c_str() << " parameter not found for board " << _boardname << "in bottle" << input.toString().c_str();
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



bool mcParser::parsePid_inPos_outPwm(Bottle &b_pid, string controlLaw)
{
    bool alreadyParsed = false;
    map<string, Pid_Algorithm*>::iterator it = posAlgoMap.find(controlLaw);
    if(it != posAlgoMap.end())
        alreadyParsed = true;

    if(alreadyParsed)
        return true;

    Pid_Algorithm_simple *pidSimple_ptr = new Pid_Algorithm_simple(_njoints);
    pidSimple_ptr->type = PidAlgo_simple;

    GenericControlUnitsType_t unitstype;
    if(!parsePidUnitsType(b_pid, unitstype))
        return false;
    pidSimple_ptr->ctrlUnitsType = unitstype;

    if(!parsePidsGroup(b_pid, pidSimple_ptr->pid, string("pos_")))
        return false;

    //posAlgoMap[controlLaw] = pidSimple_ptr;
    posAlgoMap.insert ( std::pair<std::string, Pid_Algorithm*>(controlLaw,pidSimple_ptr) );

    return true;
}


bool mcParser::parsePid_inVel_outPwm(Bottle &b_pid, string controlLaw)
{
    bool alreadyParsed = false;
    map<string, Pid_Algorithm*>::iterator it = velAlgoMap.find(controlLaw);
    if(it != velAlgoMap.end())
        alreadyParsed = true;

    if(alreadyParsed)
        return true;

    Pid_Algorithm_simple *pidSimple_ptr = new Pid_Algorithm_simple(_njoints);
    pidSimple_ptr->type = PidAlgo_simple;

    GenericControlUnitsType_t unitstype;
    if(!parsePidUnitsType(b_pid, unitstype))
        return false;
    pidSimple_ptr->ctrlUnitsType = unitstype;

    if(!parsePidsGroup(b_pid, pidSimple_ptr->pid, string("vel_")))
        return false;

    //velAlgoMap[controlLaw] = pidSimple_ptr;
    velAlgoMap.insert ( std::pair<std::string, Pid_Algorithm*>(controlLaw,pidSimple_ptr) );

    return true;
}

bool mcParser::parsePid_inTrq_outPwm(Bottle &b_pid, string controlLaw)
{
    bool alreadyParsed = false;
    map<string, Pid_Algorithm*>::iterator it = trqAlgoMap.find(controlLaw);
    if(it != trqAlgoMap.end())
        alreadyParsed = true;

    if(alreadyParsed)
        return true;

    Pid_Algorithm_simple *pidSimple_ptr = new Pid_Algorithm_simple(_njoints);
    pidSimple_ptr->type = PidAlgo_simple;

    GenericControlUnitsType_t unitstype;
    if(!parsePidUnitsType(b_pid, unitstype))
        return false;

    pidSimple_ptr->ctrlUnitsType = unitstype;

    if(!parsePidsGroup(b_pid, pidSimple_ptr->pid, string("trq_")))
        return false;

    Bottle xtmp;
    //torque specific params
    if (!extractGroup(b_pid, xtmp, "trq_kbemf", "kbemf parameter", _njoints))        return false; for (int j=0; j<_njoints; j++) _kbemf[j]      = xtmp.get(j+1).asDouble();
    if (!extractGroup(b_pid, xtmp, "trq_ktau", "ktau parameter", _njoints))          return false; for (int j=0; j<_njoints; j++) _ktau[j]       = xtmp.get(j+1).asDouble();
    if (!extractGroup(b_pid, xtmp, "trq_filterType", "filterType param", _njoints))  return false; for (int j=0; j<_njoints; j++) _filterType[j] = xtmp.get(j+1).asInt();

    //trqAlgoMap[controlLaw] = pidSimple_ptr;
    trqAlgoMap.insert ( std::pair<std::string, Pid_Algorithm*>(controlLaw,pidSimple_ptr) );

    return true;
}

bool mcParser::parsePidPos_withInnerVelPid(Bottle &b_pid, string controlLaw)
{

    bool alreadyParsed = false;
    map<string, Pid_Algorithm*>::iterator it = posAlgoMap.find(controlLaw);
    if(it != posAlgoMap.end())
        alreadyParsed = true;

    if(alreadyParsed)
        return true;

    PidAlgorithm_VelocityInnerLoop *pidInnerVel_ptr = new PidAlgorithm_VelocityInnerLoop(_njoints);
    pidInnerVel_ptr->type = PIdAlgo_velocityInnerLoop;

    GenericControlUnitsType_t unitstype;
    if(!parsePidUnitsType(b_pid, unitstype))
        return false;
    pidInnerVel_ptr->ctrlUnitsType = unitstype;

    if(!parsePidsGroup(b_pid, pidInnerVel_ptr->extPid, string("pos_")))
        return false;
    if(!parsePidsGroup(b_pid, pidInnerVel_ptr->innerVelPid, string("vel_")))
        return false;

    //posAlgoMap[controlLaw] = pidInnerVel_ptr;
    posAlgoMap.insert ( std::pair<std::string, Pid_Algorithm*>(controlLaw,pidInnerVel_ptr) );

    return true;
}


bool mcParser::parsePidTrq_withInnerVelPid(Bottle &b_pid, string controlLaw)
{

    bool alreadyParsed = false;
    map<string, Pid_Algorithm*>::iterator it = trqAlgoMap.find(controlLaw);
    if(it != trqAlgoMap.end())
        alreadyParsed = true;

    if(alreadyParsed)
        return true;

    PidAlgorithm_VelocityInnerLoop *pidInnerVel_ptr = new PidAlgorithm_VelocityInnerLoop(_njoints);
    pidInnerVel_ptr->type = PIdAlgo_velocityInnerLoop;

    GenericControlUnitsType_t unitstype;
    if(!parsePidUnitsType(b_pid, unitstype))
        return false;
    pidInnerVel_ptr->ctrlUnitsType = unitstype;


    if(!parsePidsGroup(b_pid, pidInnerVel_ptr->extPid, string("trq_")))
        return false;

    Bottle xtmp;
    //torque specific params
    if (!extractGroup(b_pid, xtmp, "trq_kbemf", "kbemf parameter", _njoints))         return false; for (int j=0; j<_njoints; j++) _kbemf[j]      = xtmp.get(j+1).asDouble();
    if (!extractGroup(b_pid, xtmp, "trq_ktau", "ktau parameter", _njoints))           return false; for (int j=0; j<_njoints; j++) _ktau[j]       = xtmp.get(j+1).asDouble();
    if (!extractGroup(b_pid, xtmp, "trq_filterType", "filterType param", _njoints))   return false; for (int j=0; j<_njoints; j++) _filterType[j] = xtmp.get(j+1).asInt();

    if(!parsePidsGroup(b_pid, pidInnerVel_ptr->innerVelPid, string("vel_")))
        return false;

    //trqAlgoMap[controlLaw] = pidInnerVel_ptr;
    trqAlgoMap.insert ( std::pair<std::string, Pid_Algorithm*>(controlLaw,pidInnerVel_ptr) );

    return true;
}




bool mcParser::getCorrectPidForEachJoint(eomcParser_pidInfo *ppids, eomcParser_pidInfo *vpids, eomcParser_trqPidInfo *tpids)
{
    Pid_Algorithm *pidAlgo_ptr = NULL;
    Pid_Algorithm *vpidAlgo_ptr = NULL;
    Pid_Algorithm *tpidAlgo_ptr = NULL;

    map<string, Pid_Algorithm*>::iterator it;

    for(int i=0; i<_njoints; i++)
    {
        //get position pid
        it = posAlgoMap.find(_posistionControlLaw[i]);
        if(it == posAlgoMap.end())
        {
           yError() << "embObjMC BOARD " << _boardname << "Cannot find " << _posistionControlLaw[i].c_str() << "in parsed pos pid";
           return false;
        }

        pidAlgo_ptr = posAlgoMap[_posistionControlLaw[i]];

        //get velocity pid
        if(_velocityControlLaw[i] == "none")
            vpidAlgo_ptr = NULL;
        else
        {
            it = velAlgoMap.find(_velocityControlLaw[i]);
            if(it == velAlgoMap.end())
            {
                yError() << "embObjMC BOARD " << _boardname << "Cannot find " << _velocityControlLaw[i].c_str() << "in parsed vel pid";
                return false;
            }

            vpidAlgo_ptr = velAlgoMap[_velocityControlLaw[i]];
        }

        //get torque pid
        if(_torqueControlLaw[i] == "none")
           tpidAlgo_ptr = NULL;
        else
        {
            it = trqAlgoMap.find(_torqueControlLaw[i]);
            if(it == trqAlgoMap.end())
            {
                yError() << "embObjMC BOARD " << _boardname << "Cannot find " << _torqueControlLaw[i].c_str() << "in parsed trq pid";
                return false;
            }

            tpidAlgo_ptr = trqAlgoMap[_torqueControlLaw[i]];
        }

        //verifico che i giunti abbiamo lo stesso tipo di algoritmo per pid posizione e torque, mentre per il pid di velocita' puo' essere solo Pid_Algorithm_simple
//         if((vpidAlgo_ptr) && (pidAlgo_ptr->type != vpidAlgo_ptr->type))
//         {
//             yError() << "embObjMC BOARD " << _boardname << "Position control law is not equal to velocity control law for joint " << i;
//             return false;
//         }

        if((tpidAlgo_ptr) && (pidAlgo_ptr->type != tpidAlgo_ptr->type))
        {
            yError() << "Torque control law is not equal to velocity control law for joint " << i;
            return false;
        }

        //verify velocity pid is equal for pos and torq in case of PidPos_withInnerVelPid
        if(pidAlgo_ptr->type == PIdAlgo_velocityInnerLoop)
        {
            if(vpidAlgo_ptr)
            {

                //pids are equal if they have same control units type and same values
                if( ( ((PidAlgorithm_VelocityInnerLoop*)pidAlgo_ptr)->ctrlUnitsType != ((Pid_Algorithm_simple*)vpidAlgo_ptr)->ctrlUnitsType) ||
                     ( ! (((PidAlgorithm_VelocityInnerLoop*)pidAlgo_ptr)->innerVelPid[i]==((Pid_Algorithm_simple*)vpidAlgo_ptr)->pid[i]) ) )
                {
                    yError() << "embObjMC BOARD " << _boardname << ":Joint" << i << ": velocity pid values of inner loop of position control are not equal to velocity control pid values";
                    return false;
                }
            }

            if(tpidAlgo_ptr)
            {
                for(int x =0; x<_njoints; x++)
                {
                    if( ( ((PidAlgorithm_VelocityInnerLoop*)pidAlgo_ptr)->ctrlUnitsType != ((Pid_Algorithm_simple*)vpidAlgo_ptr)->ctrlUnitsType ) ||
                        ( ! (((PidAlgorithm_VelocityInnerLoop*)pidAlgo_ptr)->innerVelPid[x]==((PidAlgorithm_VelocityInnerLoop*)tpidAlgo_ptr)->innerVelPid[x]) ) )
                    {
                        yError() << "embObjMC BOARD " <<_boardname << ":Joint" << i << ":velocity pid values of inner loop of torque control are not equal to velocity pid values of inner position pid ";
                        return false;
                    }
                }
            }
        }


        switch(pidAlgo_ptr->type )
        {
            case(PidAlgo_simple):
            {
                Pid_Algorithm_simple *pidAlgo_simple_ptr = dynamic_cast<Pid_Algorithm_simple*>(pidAlgo_ptr);
                if(pidAlgo_simple_ptr == NULL)
                {
                    yError() << "embObjMC BOARD " << _boardname << "dynamic_cast error (ref1)";
                    return false;
                }
                ppids[i].pid = pidAlgo_simple_ptr->pid[i];
                ppids[i].ctrlUnitsType = pidAlgo_simple_ptr->ctrlUnitsType;
                ppids[i].controlLaw =  pidAlgo_simple_ptr->type;
                ppids[i].usernamePidSelected = _posistionControlLaw[i];
                ppids[i].enabled = true;

                if(vpidAlgo_ptr)
                {
                    Pid_Algorithm_simple *vpidAlgo_simple_ptr = dynamic_cast<Pid_Algorithm_simple*>(vpidAlgo_ptr);
                    if(vpidAlgo_simple_ptr == NULL)
                    {
                        yError() << "embObjMC BOARD " << _boardname << "dynamic_cast error (ref2)";
                        return false;
                    }
                    vpids[i].pid = vpidAlgo_simple_ptr->pid[i];
                    vpids[i].ctrlUnitsType = vpidAlgo_simple_ptr->ctrlUnitsType;
                    vpids[i].controlLaw =  vpidAlgo_simple_ptr->type;
                    vpids[i].usernamePidSelected = _velocityControlLaw[i];
                    vpids[i].enabled = true;
                }
                else
                {
                    //_vpids[i] = 0; la allocAndCheck fa gia un memset a zero
                    vpids[i].enabled = false;
                    vpids[i].usernamePidSelected = "none";
                }

                if(tpidAlgo_ptr)
                {
                    Pid_Algorithm_simple *tpidAlgo_simple_ptr = dynamic_cast<Pid_Algorithm_simple*>(tpidAlgo_ptr);
                    if(tpidAlgo_simple_ptr == NULL)
                    {
                        yError() << "embObjMC BOARD " << _boardname << "dynamic_cast error (ref3)";
                        return false;
                    }
                    tpids[i].pid = tpidAlgo_simple_ptr->pid[i];
                    tpids[i].ctrlUnitsType = tpidAlgo_simple_ptr->ctrlUnitsType;
                    tpids[i].controlLaw =  tpidAlgo_simple_ptr->type;
                    tpids[i].usernamePidSelected = _torqueControlLaw[i];
                    tpids[i].enabled = true;
                    tpids[i].kbemf = _kbemf[i];
                    tpids[i].ktau = _ktau[i];
                    tpids[i].filterType = _filterType[i];
                }
                else
                {
                    tpids[i].enabled = false;
                    tpids[i].usernamePidSelected = "none";
                }

            }break;

            case(PIdAlgo_velocityInnerLoop):
            {
                PidAlgorithm_VelocityInnerLoop *pidAlgo_innerVelLoop_ptr = dynamic_cast<PidAlgorithm_VelocityInnerLoop*>(pidAlgo_ptr);
                if(pidAlgo_innerVelLoop_ptr == NULL)
                {
                    yError() << "embObjMC BOARD " << _boardname << "dynamic_cast error (ref4)";
                    return false;
                }
                ppids[i].pid = pidAlgo_innerVelLoop_ptr->extPid[i];
                ppids[i].ctrlUnitsType = pidAlgo_innerVelLoop_ptr->ctrlUnitsType;
                ppids[i].controlLaw =  pidAlgo_innerVelLoop_ptr->type;
                ppids[i].usernamePidSelected = _posistionControlLaw[i];
                ppids[i].enabled = true;
                vpids[i].pid = pidAlgo_innerVelLoop_ptr->innerVelPid[i];
                vpids[i].ctrlUnitsType = pidAlgo_innerVelLoop_ptr->ctrlUnitsType;
                vpids[i].controlLaw =  pidAlgo_innerVelLoop_ptr->type;
                vpids[i].usernamePidSelected = _velocityControlLaw[i];
                vpids[i].enabled = true;

                if(tpidAlgo_ptr)
                {
                    PidAlgorithm_VelocityInnerLoop *tpidAlgo_innerVelLoop_ptr = dynamic_cast<PidAlgorithm_VelocityInnerLoop*>(tpidAlgo_ptr);
                    if(tpidAlgo_innerVelLoop_ptr == NULL)
                    {
                        yError() << "embObjMC BOARD " << _boardname << "dynamic_cast error (ref5)";
                        return false;
                    }
                    tpids[i].pid = tpidAlgo_innerVelLoop_ptr->extPid[i];
                    tpids[i].ctrlUnitsType = tpidAlgo_innerVelLoop_ptr->ctrlUnitsType;
                    tpids[i].controlLaw =  tpidAlgo_innerVelLoop_ptr->type;
                    tpids[i].usernamePidSelected = _torqueControlLaw[i];
                    tpids[i].enabled = true;
                    tpids[i].kbemf = _kbemf[i];
                    tpids[i].ktau = _ktau[i];
                    tpids[i].filterType = _filterType[i];
                    vpids[i].pid = tpidAlgo_innerVelLoop_ptr->innerVelPid[i];
                    vpids[i].ctrlUnitsType = tpidAlgo_innerVelLoop_ptr->ctrlUnitsType;
                    vpids[i].controlLaw =  tpidAlgo_innerVelLoop_ptr->type;
                    vpids[i].usernamePidSelected = _velocityControlLaw[i];
                    vpids[i].enabled = true;
                }
                else
                {
                    //_tpids[i] = 0; la allocAndCheck fa gia un memset a zero
                    tpids[i].enabled = false;
                    tpids[i].usernamePidSelected = "none";
                }
            }break;

            default:
            {
                yError() << "embObjMC BOARD " << _boardname << "Unknown pid algo type. I should naver stay here!";
                return false;
            }
        }
    }


    //Here i would check that all joints have same type units in order to create torquehelper with correct factor.

    //get first joint with enabled torque
    int firstjoint = -1;
    for(int i=0; i<_njoints; i++)
    {
        if(tpids[i].enabled)
            firstjoint = i;
    }

    if(firstjoint==-1)
    {
        // no joint has torque enabed
        return true;
    }

    for(int i=firstjoint+1; i<_njoints; i++)
    {
        if(tpids[i].enabled)
        {
            if(tpids[firstjoint].ctrlUnitsType != tpids[i].ctrlUnitsType)
            {
                yError() << "embObjMC BOARD " << _boardname << "all joints with torque enabled should have same controlunits type. Joint " << firstjoint << " differs from joint " << i;
                return false;
            }
        }
    }
    return true;


}


bool mcParser::parsePidUnitsType(Bottle &bPid, GenericControlUnitsType_t &unitstype)
{

    Value &controlUnits=bPid.find("controlUnits");
    if(controlUnits.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << " missing controlUnits parameter";
        return false;
    }
    if(!controlUnits.isString())
    {
        yError() << "embObjMC BOARD " << _boardname << " controlUnits parameter is not a string";
        return false;
    }


    if(controlUnits.toString()==string("metric_units"))
    {
        unitstype = controlUnits_metric;
        return true;
    }
    else if(controlUnits.toString()==string("machine_units"))
    {
        unitstype = controlUnits_machine;
        return true;
    }
    else
    {
        yError() << "embObjMC BOARD " << _boardname << "invalid controlUnits value: " <<controlUnits.toString().c_str();
        return false;
    }

}


bool mcParser::parse2FocGroup(yarp::os::Searchable &config, eomc_twofocSpecificInfo *twofocinfo)
{
     Bottle &focGroup=config.findGroup("2FOC");
     if (focGroup.isNull() )
     {
        yError() << "embObjMC BOARD " << _boardname << " detected that Group 2FOC is not found in configuration file";
        return false;
     }

    Bottle xtmp;
    int i;

    if (!extractGroup(focGroup, xtmp, "HasHallSensor", "HasHallSensor 0/1 ", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
           twofocinfo[i - 1].hasHallSensor = xtmp.get(i).asInt();
    }
    if (!extractGroup(focGroup, xtmp, "HasTempSensor", "HasTempSensor 0/1 ", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            twofocinfo[i - 1].hasTempSensor = xtmp.get(i).asInt();
    }
    if (!extractGroup(focGroup, xtmp, "HasRotorEncoder", "HasRotorEncoder 0/1 ", _njoints))
    {
        return false;
    }
    else
    {

        for (i = 1; i < xtmp.size(); i++)
            twofocinfo[i - 1].hasRotorEncoder = xtmp.get(i).asInt();
    }
    if (!extractGroup(focGroup, xtmp, "HasRotorEncoderIndex", "HasRotorEncoderIndex 0/1 ", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            twofocinfo[i - 1].hasRotorEncoderIndex = xtmp.get(i).asInt();
    }

    if (!extractGroup(focGroup, xtmp, "RotorIndexOffset", "RotorIndexOffset", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            twofocinfo[i - 1].rotorIndexOffset = xtmp.get(i).asInt();
    }

    // Number of motor poles
    if (!extractGroup(focGroup, xtmp, "MotorPoles", "MotorPoles", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            twofocinfo[i - 1].motorPoles = xtmp.get(i).asInt();
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
            twofocinfo[i - 1].hasSpeedEncoder = xtmp.get(i).asInt();
    }

    return true;

}




bool mcParser::parseJointsetCfgGroup(yarp::os::Searchable &config, std::vector<eomc_jointsSet> &jsets, std::vector<int> &joint2set)
{
    Bottle jointsetcfg = config.findGroup("JOINTSET_CFG");
    if (jointsetcfg.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << "Missing JOINTSET_CFG group";
        return false;
    }


    Bottle xtmp;
    int numofsets = 0;

    if(!extractGroup(jointsetcfg, xtmp, "numberofsets", "number of sets ", 1))
    {
        return  false;
    }

    numofsets = xtmp.get(1).asInt();

    if((0 == numofsets) || (numofsets > _njoints))
    {
        yError() << "embObjMC BOARD " << _boardname << "Number of jointsets is not correct. it should belong to (1, " << _njoints << ")";
        return false;
    }



    if(!checkAndSetVectorSize(jsets, numofsets, "parseJointsetCfgGroup"))
        return false;

    if(!checkAndSetVectorSize(joint2set, _njoints, "parseJointsetCfgGroup"))
        return false;

    for(unsigned int s=0;s<numofsets;s++)
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
            int jointofthisset = b_listofjoints.get(j+1).asInt();

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
        jsets.at(s).cfg.constraints.param1 = xtmp.get(1).asDouble();

        //param2
        if(!extractGroup(js_cfg, xtmp, "param2", "param2 of jointset constraint ", 1))
        {
            return  false;
        }
        jsets.at(s).cfg.constraints.param2 = xtmp.get(1).asDouble();


    }
    return true;
}

bool mcParser::parseTimeoutsGroup(yarp::os::Searchable &config, std::vector<eomc_timeouts_t> &timeouts, int defaultVelocityTimeout)
{
    if(!checkAndSetVectorSize(timeouts, _njoints, "parseTimeoutsGroup"))
        return false;

    bool useDefVal = false;
    int i;

    Bottle timeoutsGroup =config.findGroup("TIMEOUTS");
    if(timeoutsGroup.isNull())
    {
        yWarning() << "embObjMC BOARD " << _boardname << " no TIMEOUTS group found in config file, default values will be used.";
        useDefVal = true;
    }
    else
    {
        Bottle xtmp;
        xtmp.clear();
        if (!extractGroup(timeoutsGroup, xtmp, "velocity", "a list of timeout to be used in the vmo control", _njoints))
        {
            useDefVal = true;
        }
        else
        {
            for(i=1; i<xtmp.size(); i++)
                timeouts[i-1].velocity = xtmp.get(i).asInt();
        }
    }

    if(useDefVal)
    {
        yWarning() << "Using default velocity Timeout="<< defaultVelocityTimeout <<" millisec";
        for(i=0; i<_njoints; i++)
            timeouts[i].velocity = defaultVelocityTimeout;
    }
    return true;

}

bool mcParser::parseCurrentLimits(yarp::os::Searchable &config, std::vector<eomc_motorCurrentLimits> &currLimits)
{
    Bottle &limits=config.findGroup("LIMITS");
    if (limits.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << " detected that Group LIMITS is not found in configuration file";
        return false;
    }

    currLimits.resize(_njoints);
    int i;
    Bottle xtmp;

    // current limit
    if (!extractGroup(limits, xtmp, "motorOverloadCurrents","a list of current limits", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) currLimits[i-1].overloadCurrent=xtmp.get(i).asDouble();

    // nominal current
    if (!extractGroup(limits, xtmp, "motorNominalCurrents","a list of nominal current limits", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) currLimits[i-1].nominalCurrent =xtmp.get(i).asDouble();

    // peak current
    if (!extractGroup(limits, xtmp, "motorPeakCurrents","a list of peak current limits", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) currLimits[i-1].peakCurrent=xtmp.get(i).asDouble();

    return true;

}

bool mcParser::parseJointsLimits(yarp::os::Searchable &config, std::vector<eomc_jointLimits> &jointsLimits)
{
    Bottle &limits=config.findGroup("LIMITS");
    if (limits.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << " detected that Group LIMITS is not found in configuration file";
        return false;
    }

    jointsLimits.resize(_njoints);
    int i;
    Bottle xtmp;

    // max limit
    if (!extractGroup(limits, xtmp, "jntPosMax","a list of user maximum angles (in degrees)", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) jointsLimits[i-1].posMax = xtmp.get(i).asDouble();

    // min limit
    if (!extractGroup(limits, xtmp, "jntPosMin","a list of user minimum angles (in degrees)", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) jointsLimits[i-1].posMin = xtmp.get(i).asDouble();

    // max hardware limit
    if (!extractGroup(limits, xtmp, "hardwareJntPosMax","a list of hardware maximum angles (in degrees)", _njoints))
        return false;
    else
    {
        for(i=1; i<xtmp.size(); i++) jointsLimits[i-1].posHwMax = xtmp.get(i).asDouble();

        //check hardware limits are bigger then user limits
        for(i=0; i<_njoints; i++)
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
        for(i=1; i<xtmp.size(); i++) jointsLimits[i-1].posHwMin = xtmp.get(i).asDouble();

        //check hardware limits are bigger then user limits
        for(i=0; i<_njoints; i++)
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
        for (i = 1; i<xtmp.size(); i++)     jointsLimits[i - 1].velMax = xtmp.get(i).asDouble();

    return true;
}


bool mcParser::parseRotorsLimits(yarp::os::Searchable &config, std::vector<eomc_rotorLimits> &rotorsLimits)
{
    Bottle &limits=config.findGroup("LIMITS");
    if (limits.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << " detected that Group LIMITS is not found in configuration file";
        return false;
    }

    if(!checkAndSetVectorSize(rotorsLimits, _njoints, "parseRotorsLimits"))
        return false;

    Bottle xtmp;
    int i;

    // Rotor max limit
    if (!extractGroup(limits, xtmp, "rotorPosMax","a list of maximum rotor angles (in degrees)", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) rotorsLimits[i-1].posMax = xtmp.get(i).asDouble();



    // Rotor min limit
    if (!extractGroup(limits, xtmp, "rotorPosMin","a list of minimum roto angles (in degrees)", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) rotorsLimits[i-1].posMin = xtmp.get(i).asDouble();

    // Motor pwm limit
    if (!extractGroup(limits, xtmp, "motorPwmLimit","a list of motor PWM limits", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++)
        {
            rotorsLimits[i-1].pwmMax = xtmp.get(i).asDouble();
            if(rotorsLimits[i-1].pwmMax<0)
            {
                yError() << "motorPwmLimit should be a positive value";
                return false;
            }
        }

    return true;

}




bool mcParser::parseCouplingInfo(yarp::os::Searchable &config, eomc_couplingInfo_t &couplingInfo)
{
    Bottle coupling_bottle = config.findGroup("COUPLINGS");
    if (coupling_bottle.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname <<  "Missing Coupling group";
        return false;
    }
    Bottle xtmp;
    int  fixedMatrix4X4Size = 16;
    int  fixedMatrix4X6Size = 24;
    bool formaterror =false;

    // matrix J2M
    if (!extractGroup(coupling_bottle, xtmp, "matrixJ2M", "matrixJ2M ", fixedMatrix4X4Size))
    {
        return false;
    }

    if(false == convert(xtmp, couplingInfo.matrixJ2M, formaterror, fixedMatrix4X4Size))
    {
       yError() << "embObjMC BOARD " << _boardname << " has detected an illegal format for some of the values of CONTROLLER.matrixJ2M";
       return false;
    }


    // matrix E2J
    if (!extractGroup(coupling_bottle, xtmp, "matrixE2J", "matrixE2J ", fixedMatrix4X6Size))
    {
        return false;
    }

    formaterror = false;
    if(false == convert(xtmp, couplingInfo.matrixE2J, formaterror, fixedMatrix4X6Size))
    {
        yError() << "embObjMC BOARD " << _boardname << " has detected an illegal format for some of the values of CONTROLLER.matrixE2J";
        return false;
    }


    // matrix M2J
    if (!extractGroup(coupling_bottle, xtmp, "matrixM2J", "matrixM2J ", fixedMatrix4X4Size))
    {
        return false;
    }

    formaterror = false;
    if( false == convert(xtmp, couplingInfo.matrixM2J, formaterror, fixedMatrix4X4Size))
    {
        yError() << "embObjMC BOARD " << _boardname << " has detected an illegal format for some of the values of CONTROLLER.matrixM2J";
        return false;
    }

    return true;
}


bool mcParser::parseMotioncontrolVersion(yarp::os::Searchable &config, int &version)
{
    if (!config.findGroup("GENERAL").find("MotioncontrolVersion").isInt())
    {
        yError() << "Missing MotioncontrolVersion parameter. RobotInterface cannot start. Please contact icub-support@iit.it";
        return false;
    }

    version = config.findGroup("GENERAL").find("MotioncontrolVersion").asInt();
    return true;

}

bool mcParser::isVerboseEnabled(yarp::os::Searchable &config)
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
    _verbosewhenok = ret;
    return ret;
}

bool mcParser::parseBehaviourFalgs(yarp::os::Searchable &config, bool &useRawEncoderData, bool  &pwmIsLimited )
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



bool mcParser::parseAxisInfo(yarp::os::Searchable &config, int axisMap[], std::vector<eomc_axisInfo_t> &axisInfo)
{

    Bottle xtmp;
    int i;
    axisInfo.resize(_njoints);

    Bottle general = config.findGroup("GENERAL");
    if (general.isNull())
    {
       yError() << "embObjMC BOARD " << _boardname << "Missing General group" ;
       return false;
    }


    if (!extractGroup(general, xtmp, "AxisMap", "a list of reordered indices for the axes", _njoints))
        return false;

    for (i = 1; i < xtmp.size(); i++)
        axisMap[i-1] = xtmp.get(i).asInt();

    if (!extractGroup(general, xtmp, "AxisName", "a list of strings representing the axes names", _njoints))
        return false;

    //beware: axis name has to be remapped here because they are not set using the toHw() helper function
    for (i = 1; i < xtmp.size(); i++)
    {
        int mappedto = axisInfo[i-1].mappedto;
        axisInfo[axisMap[i - 1]].name = xtmp.get(i).asString();
    }

    if (!extractGroup(general, xtmp, "AxisType", "a list of strings representing the axes type (revolute/prismatic)", _njoints))
        return false;

    //beware: axis type has to be remapped here because they are not set using the toHw() helper function
    for (i = 1; i < xtmp.size(); i++)
    {
        string s = xtmp.get(i).asString();
        int mappedto = axisInfo[i-1].mappedto;
        if (s == "revolute")  axisInfo[axisMap[i - 1]].type = VOCAB_JOINTTYPE_REVOLUTE;
        else if (s == "prismatic")  axisInfo[axisMap[i - 1]].type = VOCAB_JOINTTYPE_PRISMATIC;
        else
        {
            yError("Unknown AxisType value %s!", s.c_str());
            axisInfo[axisMap[i - 1]].type = VOCAB_JOINTTYPE_UNKNOWN;
            return false;
        }
    }

    return true;
}




bool mcParser::parseEncoderFactor(yarp::os::Searchable &config, double encoderFactor[])
{
    Bottle general = config.findGroup("GENERAL");
    if (general.isNull())
    {
       yError() << "embObjMC BOARD " << _boardname << "Missing General group" ;
       return false;
    }
    Bottle xtmp;
    int i;
    double tmp_A2E;

    // Encoder scales
    if (!extractGroup(general, xtmp, "Encoder", "a list of scales for the encoders", _njoints))
    {
        return false;
    }

    for (i = 1; i < xtmp.size(); i++)
    {
        tmp_A2E = xtmp.get(i).asDouble();
        if (tmp_A2E<0)
        {
            yWarning() << "embObjMC BOARD " << _boardname << "Encoder parameter should be positive!";
        }
        encoderFactor[i - 1] = tmp_A2E;
    }

    return true;
}

bool mcParser::parsefullscalePWM(yarp::os::Searchable &config, double dutycycleToPWM[])
{
    Bottle general = config.findGroup("GENERAL");
    if (general.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << "Missing General group";
        return false;
    }
    Bottle xtmp;
    int i;
    double tmpval;

    // fullscalePWM
    if (!extractGroup(general, xtmp, "fullscalePWM", "a list of scales for the fullscalePWM conversion factor", _njoints))
    {
        yError("fullscalePWM param not found in config file. Please update robot configuration files or contact https://github.com/robotology/icub-support");
        return false;
    }

    for (i = 1; i < xtmp.size(); i++)
    {
        tmpval = xtmp.get(i).asDouble();
        if (tmpval<0)
        {
            yError() << "embObjMC BOARD " << _boardname << "fullscalePWM parameter should be positive!";
            return false;
        }
        dutycycleToPWM[i - 1] = tmpval / 100.0;
    }

    return true;
}


bool mcParser::parseAmpsToSensor(yarp::os::Searchable &config, double ampsToSensor[])
{
    Bottle general = config.findGroup("GENERAL");
    if (general.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << "Missing General group";
        return false;
    }
    Bottle xtmp;
    int i;
    double tmpval;

    // ampsToSensor
    if (!extractGroup(general, xtmp, "ampsToSensor", "a list of scales for the ampsToSensor conversion factor", _njoints))
    {
        yError("ampsToSensor param not found in config file. Please update robot configuration files or contact https://github.com/robotology/icub-support");
        return false;
    }

    for (i = 1; i < xtmp.size(); i++)
    {
        tmpval = xtmp.get(i).asDouble();
        if (tmpval<0)
        {
            yError() << "embObjMC BOARD " << _boardname << "ampsToSensor parameter should be positive!";
            return false;
        }
        ampsToSensor[i - 1] = tmpval;
    }

    return true;
}

bool mcParser::parseGearboxValues(yarp::os::Searchable &config, double gearbox_M2J[], double gearbox_E2J[])
{
    Bottle general = config.findGroup("GENERAL");
    if (general.isNull())
    {
       yError() << "embObjMC BOARD " << _boardname << "Missing General group" ;
       return false;
    }

    Bottle xtmp;
    int i;

    // Gearbox_M2J
    if (!extractGroup(general, xtmp, "Gearbox_M2J", "The gearbox reduction ratio", _njoints))
    {
        return false;
    }

    for (i = 1; i < xtmp.size(); i++)
    {
        gearbox_M2J[i-1] = xtmp.get(i).asDouble();
        if (gearbox_M2J[i-1]==0)
        {
            yError()  << "embObjMC BOARD " << _boardname << "Using a gearbox value = 0 may cause problems! Check your configuration files";
            return false;
        }
    }


    //Gearbox_E2J
    if (!extractGroup(general, xtmp, "Gearbox_E2J", "The gearbox reduction ratio between encoder and joint", _njoints))
    {
        yWarning()  << "embObjMC BOARD " << _boardname << "Missing Gearbox_E2J param. I use default value (1) " ;
        for(int i=0; i<_njoints; i++)
        {
            gearbox_E2J[i] = 1;
        }
    }
    else
    {
        int test = xtmp.size();
        for (i = 1; i < xtmp.size(); i++)
        {
            gearbox_E2J[i-1] = xtmp.get(i).asDouble();
            if (gearbox_E2J[i-1]==0)
            {
                yError()  << "embObjMC BOARD " << _boardname << "Using a gearbox value = 0 may cause problems! Check your configuration files";
                return false;
            }
        }
    }

    return true;
}



bool mcParser::parseMechanicalsFlags(yarp::os::Searchable &config, int useMotorSpeedFbk[])
{
    Bottle general = config.findGroup("GENERAL");
    if (general.isNull())
    {
       yError() << "embObjMC BOARD " << _boardname << "Missing General group" ;
       return false;
    }
    Bottle xtmp;
    int i;

    if(!extractGroup(general, xtmp, "useMotorSpeedFbk", "Use motor speed feedback", _njoints))
    {
        return false;
    }

    for (int i = 1; i < xtmp.size(); i++)
    {
        useMotorSpeedFbk[i-1] = xtmp.get(i).asInt();
    }

    return true;

}






bool mcParser::parseImpedanceGroup(yarp::os::Searchable &config,std::vector<eomc_impedanceParameters> &impedance)
{
    Bottle impedanceGroup;
    impedanceGroup=config.findGroup("IMPEDANCE","IMPEDANCE parameters");

    if(impedanceGroup.isNull())
    {
        yError() <<"embObjMC BOARD " << _boardname << "fromConfig(): Error: no IMPEDANCE group found in config file, returning";
        return false;
    }



    if(_verbosewhenok)
    {
        yDebug()  << "embObjMC BOARD " << _boardname << ":fromConfig() detected that IMPEDANCE parameters section is found";
    }

    if(!checkAndSetVectorSize(impedance, _njoints, "parseImpedanceGroup"))
        return false;


    int j=0;
    Bottle xtmp;
    if (!extractGroup(impedanceGroup, xtmp, "stiffness", "stiffness parameter", _njoints))
        return false;

    for (j=0; j<_njoints; j++)
        impedance[j].stiffness = xtmp.get(j+1).asDouble();

    if (!extractGroup(impedanceGroup, xtmp, "damping", "damping parameter", _njoints))
        return false;

    for (j=0; j<_njoints; j++)
        impedance[j].damping = xtmp.get(j+1).asDouble();

    if(_verbosewhenok)
    {
        yInfo() << "embObjMC BOARD " << _boardname << "IMPEDANCE section: parameters successfully loaded";
    }
    return true;

}

bool mcParser::convert(ConstString const &fromstring, eOmc_jsetconstraint_t &jsetconstraint, bool& formaterror)
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



bool mcParser::convert(Bottle &bottle, vector<double> &matrix, bool &formaterror, int targetsize)
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
        item = bottle.get(i+1).asDouble();
        matrix.push_back(item);
    }

    // in here we could decide to return false if any previous conversion function has returned error

    return true;
}

//////////////////////////////////////////////////////////////////////////////
/////////////////// DEBUG FUNCTIONS
////////////////////////////////////////////////////////////////////////////////
void mcParser::debugUtil_printControlLaws(void)
{
    //////// debug prints
    yError() << "position control law: ";
    for(int x=0; x<_njoints; x++)
    {
        yError() << " - j " << x << _posistionControlLaw[x].c_str();
    }

    yError() << "velocity control law: ";
    for(int x=0; x<_njoints; x++)
    {
        yError() << "- j " << x << _velocityControlLaw[x].c_str();
    }


    yError() << "torque control law: ";
    for(int x=0; x<_njoints; x++)
    {
        yError() << " - j " << x << _torqueControlLaw[x].c_str();
    }
    //////end

}


void eomcParser_pidInfo::dumpdata(void)
{

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

    if(ctrlUnitsType == controlUnits_machine)
        cout << ". ctr Unit type is " << "controlUnits_machine.";
    else if (ctrlUnitsType == controlUnits_metric)
        cout << ". ctr Unit type is " << "controlUnits_metric.";
    else
        cout << ". ctr Unit type is " << "unknown.";

    cout << " kp is " << pid.kp;
    cout << endl;

}

void eomc_jointsSet::dumpdata(void)
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

