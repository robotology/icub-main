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
    _posistionControlLaw = allocAndCheck<string>(_njoints);
    _velocityControlLaw = allocAndCheck<string>(_njoints);
    _torqueControlLaw = allocAndCheck<string>(_njoints);

    _kbemf=allocAndCheck<double>(_njoints);
    _ktau=allocAndCheck<double>(_njoints);
    _filterType=allocAndCheck<int>(_njoints);

};

mcParser::~mcParser()
{
    checkAndDestroy(_posistionControlLaw);
    checkAndDestroy(_velocityControlLaw);
    checkAndDestroy(_torqueControlLaw);
    checkAndDestroy(_kbemf);
    checkAndDestroy(_ktau);
    checkAndDestroy(_filterType);
}


bool mcParser::parsePids(yarp::os::Searchable &config, eomcParser_pidInfo *ppids, eomcParser_pidInfo *vpids, eomcParser_trqPidInfo *tpids)
{

    if(!parseControlsGroup(config))
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
            _posistionControlLaw[i - 1] = xtmp.get(i).asString();
    }

    if (!extractGroup(controlsGroup, xtmp, "velocityControl", "Velocity Control ", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            _velocityControlLaw[i - 1] = xtmp.get(i).asString();
    }

    if (!extractGroup(controlsGroup, xtmp, "torqueControl", "Torque Control ", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            _torqueControlLaw[i - 1] = xtmp.get(i).asString();
    }


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
            if (!parsePid_inPos_outPwm(velControlLaw, _velocityControlLaw[i]))
            {
                yError() << "embObjMC BOARD " << _boardname << "format error in Pid_inPos_outPwm";
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
        if (s_controlaw==string("Pid_inPos_outPwm"))
        {
            if (!parsePid_inTrq_outPwm(trqControlLaw, _torqueControlLaw[i]))
            {
                yError() << "embObjMC BOARD " << _boardname << " format error in Pid_inPos_outPwm";
                return false;
            }

        }
        else if (s_controlaw==string("PidPos_withInnerVelPid"))
        {
            if (!parsePidPos_withInnerVelPid(trqControlLaw,_torqueControlLaw[i] ))
            {
                yError() << "embObjMC BOARD " << _boardname << " format error in PidPos_withInnerVelPid";
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

    _positionControlUnits = (positionControlUnitsType)unitstype;

    if(!parsePidsGroup(b_pid, pidSimple_ptr->pid, string("pos_")))
        return false;

    posAlgoMap[controlLaw] = pidSimple_ptr;

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

    _positionControlUnits = (positionControlUnitsType)unitstype;
    _velocityControlUnits = (velocityControlUnitsType)unitstype;

    if(!parsePidsGroup(b_pid, pidSimple_ptr->pid, string("vel_")))
        return false;

    velAlgoMap[controlLaw] = pidSimple_ptr;

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

#warning VALE per semplificare ho tolto la possibilita di usare pid con machine units
    if(unitstype == controlUnits_machine)
    {
        yError() << "embObjMC BOARD " << _boardname << "Torque pids cannot be expressed in machine units!! Sorry for the inconvenience!!!";
        return false;
    }

    pidSimple_ptr->ctrlUnitsType = unitstype;

    if(!parsePidsGroup(b_pid, pidSimple_ptr->pid, string("trq_")))
        return false;

    Bottle xtmp;
    //torque specific params
    if (!extractGroup(b_pid, xtmp, "kbemf", "kbemf parameter", _njoints))         return false; for (int j=0; j<_njoints; j++) _kbemf[j]      = xtmp.get(j+1).asDouble();
    if (!extractGroup(b_pid, xtmp, "ktau", "ktau parameter", _njoints))           return false; for (int j=0; j<_njoints; j++) _ktau[j]       = xtmp.get(j+1).asDouble();
    if (!extractGroup(b_pid, xtmp, "filterType", "filterType param", _njoints))   return false; for (int j=0; j<_njoints; j++) _filterType[j] = xtmp.get(j+1).asInt();

    //conversion from metric to machine units (if applicable)
    for (int j=0; j<_njoints; j++)
    {
        pidSimple_ptr->pid[j].kp = pidSimple_ptr->pid[j].kp / _torqueControlHelper->getNewtonsToSensor(j);  //[PWM/Nm]
        pidSimple_ptr->pid[j].ki = pidSimple_ptr->pid[j].ki / _torqueControlHelper->getNewtonsToSensor(j);  //[PWM/Nm]
        pidSimple_ptr->pid[j].kd = pidSimple_ptr->pid[j].kd / _torqueControlHelper->getNewtonsToSensor(j);  //[PWM/Nm]
        pidSimple_ptr->pid[j].stiction_up_val   = pidSimple_ptr->pid[j].stiction_up_val   * _torqueControlHelper->getNewtonsToSensor(j); //[Nm]
        pidSimple_ptr->pid[j].stiction_down_val = pidSimple_ptr->pid[j].stiction_down_val * _torqueControlHelper->getNewtonsToSensor(j); //[Nm]
    }

    trqAlgoMap[controlLaw] = pidSimple_ptr;

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

    _positionControlUnits = (positionControlUnitsType)unitstype;
     _velocityControlUnits = (velocityControlUnitsType)unitstype;

    if(!parsePidsGroup(b_pid, pidInnerVel_ptr->extPid, string("pos_")))
        return false;
    if(!parsePidsGroup(b_pid, pidInnerVel_ptr->innerVelPid, string("vel_")))
        return false;

    posAlgoMap[controlLaw] = pidInnerVel_ptr;

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


    #warning VALE per semplificare ho tolto la possibilita di usare pid con machine units
    if(unitstype == controlUnits_machine)
    {
        yError() << "embObjMC BOARD " << _boardname << "Torque pids cannot be expressed in machine units!! Sorry for the inconvenience!!!";
        return false;
    }

    _torqueControlUnits = (torqueControlUnitsType)unitstype;

    if(!parsePidsGroup(b_pid, pidInnerVel_ptr->extPid, string("trq_")))
        return false;

    Bottle xtmp;
    //torque specific params
    if (!extractGroup(b_pid, xtmp, "kbemf", "kbemf parameter", _njoints))         return false; for (int j=0; j<_njoints; j++) _kbemf[j]      = xtmp.get(j+1).asDouble();
    if (!extractGroup(b_pid, xtmp, "ktau", "ktau parameter", _njoints))           return false; for (int j=0; j<_njoints; j++) _ktau[j]       = xtmp.get(j+1).asDouble();
    if (!extractGroup(b_pid, xtmp, "filterType", "filterType param", _njoints))   return false; for (int j=0; j<_njoints; j++) _filterType[j] = xtmp.get(j+1).asInt();

    if(!parsePidsGroup(b_pid, pidInnerVel_ptr->innerVelPid, string("vel_")))
        return false;

    trqAlgoMap[controlLaw] = pidInnerVel_ptr;

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

        //verifico che i giunti abbiamo lo stesso algoritmo per pid posizione, velocita' e torque
        if((vpidAlgo_ptr) && (pidAlgo_ptr->type != vpidAlgo_ptr->type))
        {
            yError() << "embObjMC BOARD " << _boardname << "Position control law is not equal to velocity control law for joint " << i;
            return false;
        }

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
                for(int x =0; x<_njoints; x++)
                {
                    if( ! pidsAreEquals(((PidAlgorithm_VelocityInnerLoop*)pidAlgo_ptr)->innerVelPid[x], ((Pid_Algorithm_simple*)vpidAlgo_ptr)->pid[x]))
                    {
                        yError() << "embObjMC BOARD " << _boardname << "velocity pid values of inner loop of position control are not equal to velocity control pid values";
                        return false;
                    }
                }
            }

            if(tpidAlgo_ptr)
            {
                for(int x =0; x<_njoints; x++)
                {
                    if(! pidsAreEquals( ((PidAlgorithm_VelocityInnerLoop*)pidAlgo_ptr)->innerVelPid[x], ((PidAlgorithm_VelocityInnerLoop*)tpidAlgo_ptr)->innerVelPid[x]))
                    {
                        yError() << "embObjMC BOARD " << _boardname << "velocity pid values of inner loop of torque control are not equal to velocity control pid values";
                        return false;
                    }
                }
            }

        }


        switch(pidAlgo_ptr->type )
        {
            case(PidAlgo_simple):
            {
                Pid_Algorithm_simple *pidAlgo_simple_ptr = (Pid_Algorithm_simple*)pidAlgo_ptr;
                ppids[i].pid = pidAlgo_simple_ptr->pid[i];
                ppids[i].ctrlUnitsType = pidAlgo_simple_ptr->ctrlUnitsType;
                ppids[i].controlLaw =  pidAlgo_simple_ptr->type;
                ppids[i].usernamePidSelected = _posistionControlLaw[i];
                ppids[i].enabled = true;

                if(vpidAlgo_ptr)
                {
                    Pid_Algorithm_simple *vpidAlgo_simple_ptr = (Pid_Algorithm_simple*)vpidAlgo_ptr;
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
                    Pid_Algorithm_simple *tpidAlgo_simple_ptr = (Pid_Algorithm_simple*)tpidAlgo_ptr;
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
                    //_tpids[i] = 0; la allocAndCheck fa gia un memset a zero
                    tpids[i].enabled = false;
                    tpids[i].usernamePidSelected = "none";
                }

            }break;

            case(PIdAlgo_velocityInnerLoop):
            {
                PidAlgorithm_VelocityInnerLoop *pidAlgo_innerVelLoop_ptr = (PidAlgorithm_VelocityInnerLoop*)pidAlgo_ptr;
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

#warning VALE: il pid esterno e quello interno devono avere lo stessp control units type...oppure cambia struct data!

                 if(tpidAlgo_ptr)
                {
                    PidAlgorithm_VelocityInnerLoop *tpidAlgo_innerVelLoop_ptr = (PidAlgorithm_VelocityInnerLoop*)tpidAlgo_ptr;
                    tpids[i].pid = tpidAlgo_innerVelLoop_ptr->extPid[i];
                    tpids[i].ctrlUnitsType = tpidAlgo_simple_ptr->ctrlUnitsType;
                    tpids[i].controlLaw =  tpidAlgo_simple_ptr->type;
                    tpids[i].usernamePidSelected = _torqueControlLaw[i];
                    tpids[i].enabled = true;
                    tpids[i].kbemf = _kbemf[i];
                    tpids[i].ktau = _ktau[i];
                    tpids[i].filterType = _filterType[i];
                    vpids[i].pid = pidAlgo_innerVelLoop_ptr->innerVelPid[i];
                    vpids[i].ctrlUnitsType = pidAlgo_innerVelLoop_ptr->ctrlUnitsType;
                    vpids[i].controlLaw =  pidAlgo_innerVelLoop_ptr->type;
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

    return true;


}

//to move in yarp pid (override = operator)
bool mcParser::pidsAreEquals(Pid &pid1, Pid &pid2)
{
    if(pid1.kp != pid2.kp)
        return false;

    if(pid1.ki != pid2.ki)
        return false;

    if(pid1.kd != pid2.kd)
        return false;

    if(pid1.max_output != pid2.max_output)
        return false;

    if(pid1.max_int != pid2.max_int)
        return false;

    if(pid1.kff != pid2.kff)
        return false;

    if(pid1.offset != pid2.offset)
        return false;

    if(pid1.scale != pid2.scale)
        return false;

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