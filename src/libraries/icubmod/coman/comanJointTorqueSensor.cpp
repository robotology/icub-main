// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iostream>
#include <cstdlib>

#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <string.h>
#include <ode/odeconfig.h>
#include <yarp/os/Time.h>

#include "comanDevicesHandler.hpp"
#include "comanJointTorqueSensor.h"
#include "Debug.h"

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;

// Utilities
/*
Per comandi TCP usare:      setItem(SET_TORQUE_FACTORS, &torque_factor, sizeof(torque_factor);
Per comandi UDP usare:      UDPCommPacket pkt(SET_DESIRED_POS_VEL);
                            pkt.appendData((char*)des_pos, N'di item nel vettore);   // si puó anche fare un ciclo for di 1 elemento alla volta se serve.
                            pkt.sendToUDPSocket(udp_sock, (sockaddr *)&dest_addr, sizeof(dest_addr));

Prendere il puntatore alla classe corrispondente al giunto
    McBoard *joint_p = getMCpointer(j);   //  -> giusto
Prendere l'indice corretto dell'array per il giunto j
    idx = bId2Idx(j);
*/


inline McBoard * comanJointTorqueSensor::getMCpointer(int j)
{
    return _mcs[jointTobId(j)];
}

inline int comanJointTorqueSensor::bId2Joint(int j)
{
    return _inv_bIdMap[j];
}

inline uint8_t comanJointTorqueSensor::jointTobId(int j)
{
    return (uint8_t) _bIdMap[j];
}

static inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    fprintf(stderr, "%s not yet implemented for comanJointTorqueSensor\n", txt);
    return false;
}

//generic function that check is key1 is present in input bottle and that the result has size elements
// return true/false
bool comanJointTorqueSensor::extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        yError () << key1.c_str() << " not found\n";
        return false;
    }

    if(tmp.size()!=size)
    {
        yError () << key1.c_str() << " incorrect number of entries";
        return false;
    }

    out=tmp;
    return true;
}


bool comanJointTorqueSensor::alloc(int nj)
{
    _axisMap = allocAndCheck<int>(nj);
    _bIdMap = allocAndCheck<int>(nj);
//     _inv_bIdMap = allocAndCheck<int>(nj);      // this will be allocated after!!

    _maxTorque=allocAndCheck<double>(nj);
    _newtonsToSensor=allocAndCheck<double>(nj);
    return true;
}

comanJointTorqueSensor::comanJointTorqueSensor() :  _mutex(1)
{
    yTrace();
    _boards_ctrl = NULL;
    _nChannels 		= 0;
    _axisMap		= NULL;
    _bIdMap     = NULL;
    _inv_bIdMap = NULL;

    _maxTorque = NULL;
    _newtonsToSensor = NULL;
//    bc_policy     = 0;
//    bc_rate     =   0;
}

comanJointTorqueSensor::~comanJointTorqueSensor()
{
    yTrace();

    checkAndDestroy<int>(_axisMap);
    checkAndDestroy<int>(_bIdMap);
    checkAndDestroy<int>(_inv_bIdMap);

    checkAndDestroy<double>(_maxTorque);
    checkAndDestroy<double>(_newtonsToSensor);
}


bool comanJointTorqueSensor::open(yarp::os::Searchable &config)
{
    yTrace();

    if(! fromConfig(config) )
    {
        yError() << " Some vital parameters are missing, check log and correct conf file!";
        return false;
    }

    _comanHandler = comanDevicesHandler::instance();

    if(_comanHandler == NULL)
    {
        yError() << "unable to create a new Coman Handler class!";
        return false;
    }

    if(!_comanHandler->open(config) )
    {
        yError() << "unable to initialize Coman Devices Handler class... probably no boards were found. Check log.";
        return false;
    }
    _boards_ctrl = _comanHandler->getBoard_ctrl_p();

    if(_boards_ctrl == NULL)
    {
        yError() << "unable to create a new Boards_ctrl class!";
        return false;
    }

    // TODO check this!
#warning "<><> TODO: This is a copy of the mcs map. Verify that things will never change after this copy or use a pointer (better) <><>"
    _mcs = _boards_ctrl->get_mcs_map();

    return true;
}

bool comanJointTorqueSensor::fromConfig(yarp::os::Searchable &config)
{
    yError() << config.toString().c_str();
    Bottle xtmp;
    int i;
    Bottle general = config.findGroup("GENERAL");

    Value &tmp= general.find("Joints");
    if(tmp.isNull())
    {
    	yError() << "Missing Joints number!\n";
		return false;
    }
    _nChannels = general.find("Joints").asInt();
    yWarning() << " njoints is " << _nChannels;
    alloc(_nChannels);

    // leggere i valori da file, AxisMap is optional
    // This is a remapping for the user. It is optional because it is actually unuseful and can even add more confusion than other.
    if (extractGroup(general, xtmp, "AxisMap", "a list of reordered indices for the axes", _nChannels+1))
    {
    	for (i = 1; i < xtmp.size(); i++)
    		_axisMap[i-1] = xtmp.get(i).asInt();
    }
    else
    {
    	yWarning() << "No AxisMap map found, using default configuration: continuous from 0 to n";
    	for (i = 1; i < xtmp.size(); i++)
    	{
    		_axisMap[i-1] = i-1;
    	}
    }

    // leggere i valori da file, bIdMap is mandatory!!
    // this value maps the board number to the joint of the comanMotorControl device
    int maxJ = -1;
    if(!extractGroup(general, xtmp, "bIdMap", "a list of ordered bIds", _nChannels+1))
    	return false;

    for (i = 1; i < xtmp.size(); i++)
	{
		_bIdMap[i-1] = xtmp.get(i).asInt();
		if(_bIdMap[i-1] > maxJ)
			maxJ = _bIdMap[i-1];
	}
    maxJ +=1;  // MaxJ has to be an addressable index, so size of vector must be maxJ+1

    _inv_bIdMap = allocAndCheck<int>(maxJ);

    for(int bId_idx=0; bId_idx<maxJ; bId_idx++)
    {
        // reset array to an invaslid value
        _inv_bIdMap[bId_idx] = -1;
    }

    for(int bId_idx=0; bId_idx<_nChannels; bId_idx++)
    {
        _inv_bIdMap[_bIdMap[bId_idx]] = bId_idx;
    }

    // MaxTorque
    if (!extractGroup(general, xtmp, "MaxTorque", "a list of scales for the encoders", _nChannels+1))
        return false;
    else
        for (i = 1; i < xtmp.size(); i++)
            _maxTorque[i-1] = xtmp.get(i).asDouble();

    // Scale Factor
    if (!extractGroup(general, xtmp, "ScaleFactor","a list of offsets for the zero point", _nChannels+1))
        return false;
    else
        for (i = 1; i < xtmp.size(); i++)
            _newtonsToSensor[i-1] = xtmp.get(i).asDouble();

//    bc_policy = 0;
//    extra_policy = 0;
//    Bottle mc_board = config.findGroup("MC_BOARD");
//    bc_policy = mc_board.find("policy").asInt();
//    extra_policy = mc_board.find("extra_policy").asInt();
//    bc_rate = mc_board.find("bc_rate").asInt();

//    printf("bc policy    = %d (0x%0x)", bc_policy, bc_policy);
//    printf("extra_policy = %d (0x%0x)", extra_policy, extra_policy);
    return true;
}


bool comanJointTorqueSensor::close()
{
    yTrace();
    return _comanHandler->deInstance();
}



//
// IAnalogSensor Interface
//

int comanJointTorqueSensor::read(yarp::sig::Vector &out)
{
    out.resize(_nChannels);
    for(int j=0; j<_nChannels; j++)
    {
//        bool ret = true;
        McBoard *joint_p = getMCpointer(j);   //  -> giusto

        if( NULL == joint_p)
        {
            //         yError() << "Trying to get velocity value on a non-existing joint j" << j;
            out[j] = j;   // return the joint number just to debug!!
            return false;
        }

        ts_bc_data_t bc_data;
        mc_bc_data_t &data = bc_data.raw_bc_data.mc_bc_data;

//      Do per scontato che la velocità sia broadcastata.
//        if(bc_policy & BC_POLICY_MOTOR_POSITION)  // se viene broadcastata... usare la ricezione udp
//        {
            joint_p->get_bc_data(bc_data);
//            ret = true;
//        }

//            out[j] = (double) data.Torque?? * _newtonsToSensor[j];
    }
    return true;
}

int comanJointTorqueSensor::getState(int ch)
{
    return AS_OK;
}

int comanJointTorqueSensor::getChannels()
{
    return _nChannels;
}

int comanJointTorqueSensor::calibrateSensor()
{
    printf("Calibrate Sensor not yet implemented\n");
    return 0;
}

int comanJointTorqueSensor::calibrateSensor(const yarp::sig::Vector& value)
{
    printf("Calibrate Sensor not yet implemented\n");
}

int comanJointTorqueSensor::calibrateChannel(int ch)
{
    printf("Calibrate Channel not yet implemented\n");
}

int comanJointTorqueSensor::calibrateChannel(int ch, double value)
{
    printf("Calibrate Channel not yet implemented\n");
}
