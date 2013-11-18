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
#include "comanVelocityObserver.h"
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


inline McBoard * comanVelocityObserver::getMCpointer(int j)
{
    return _mcs[jointTobId(j)];
}

inline int comanVelocityObserver::bId2Joint(int j)
{
    return _inv_bIdMap[j];
}

inline uint8_t comanVelocityObserver::jointTobId(int j)
{
    return (uint8_t) _bIdMap[j];
}

static inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    fprintf(stderr, "%s not yet implemented for comanVelocityObserver\n", txt);
    return false;
}

//generic function that check is key1 is present in input bottle and that the result has size elements
// return true/false
bool comanVelocityObserver::extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
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


bool comanVelocityObserver::alloc(int nj)
{
    _axisMap = allocAndCheck<int>(nj);
    _bIdMap = allocAndCheck<int>(nj);
//     _inv_bIdMap = allocAndCheck<int>(nj);      // this will be allocated after!!

    _angleToEncoder=allocAndCheck<double>(nj);
    _zero = allocAndCheck<double>(nj);

    return true;
}

comanVelocityObserver::comanVelocityObserver() :  _mutex(1)
{
    yTrace();
    _boards_ctrl    = NULL;
    _nChannels 		= 0;
    _axisMap		= NULL;
    _bIdMap         = NULL;
    _inv_bIdMap     = NULL;
    _angleToEncoder = NULL;
    _zero           = NULL;
}

comanVelocityObserver::~comanVelocityObserver()
{
    yTrace();

    checkAndDestroy<int>(_axisMap);
    checkAndDestroy<int>(_bIdMap);
    checkAndDestroy<int>(_inv_bIdMap);
    checkAndDestroy<double>(_angleToEncoder);
    checkAndDestroy<double>(_zero);
}


bool comanVelocityObserver::open(yarp::os::Searchable &config)
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

bool comanVelocityObserver::fromConfig(yarp::os::Searchable &config)
{
    yError() << config.toString().c_str();
    Bottle xtmp;
    int i;
    Bottle general = config.findGroup("GENERAL");

    Value &tmp= general.find("joints");
    if(tmp.isNull())
    {
        yError() << "Missing Joints number!\n";
        return false;
    }
    _nChannels = general.find("joints").asInt();
    yWarning() << " njoints is " << _nChannels;
    alloc(_nChannels);

    // leggere i valori da file, AxisMap is optional
    // This is a remapping for the user. It is optional because it is actually unuseful and can even add more confusion than other.
    if (extractGroup(general, xtmp, "axisMap", "a list of reordered indices for the axes", _nChannels+1))
    {
        for (i = 1; i < xtmp.size(); i++)
            _axisMap[i-1] = xtmp.get(i).asInt();
    }
    else
    {
        yWarning() << "No axisMap map found, using default configuration: continuous from 0 to n";
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

    // Encoder scales
    if (!extractGroup(general, xtmp, "encoder", "a list of scales for the encoders", _nChannels+1))
        return false;
    else
        for (i = 1; i < xtmp.size(); i++)
            _angleToEncoder[i-1] = xtmp.get(i).asDouble();

    // Zero Values
    if (!extractGroup(general, xtmp, "zeros","a list of offsets for the zero point", _nChannels+1))
    {
        printf("'zeros' factor was not found in the config file\n");
        return false;
    }
    else
        for (i = 1; i < xtmp.size(); i++)
            _zero[i-1] = xtmp.get(i).asDouble();

    return true;
}


bool comanVelocityObserver::close()
{
    yTrace();
    return _comanHandler->deInstance();
}



//
// IAnalogSensor Interface
//

int comanVelocityObserver::read(yarp::sig::Vector &out)
{
    out.resize(2*_nChannels);
    for(int j=0; j<_nChannels; j++)
    {
        bool ret = true;
        McBoard *joint_p = getMCpointer(j);   //  -> giusto

        if( NULL == joint_p)
        {
            //         yError() << "Trying to get value from a non-existing joint j" << j;
            out[j] = j;   // return the joint number just to debug!!
            return false;
        }

        ts_bc_data_t bc_data;
        joint_p->get_bc_data(bc_data);

        mc_bc_data_t &data = bc_data.raw_bc_data.mc_bc_data;
        out[j] = (double) (data.Position / _angleToEncoder[j]) - _zero[j];
        out[_nChannels+j] = (double) (data.Velocity / _angleToEncoder[j]);
    }
    return true;
}

int comanVelocityObserver::getState(int ch)
{
    return AS_OK;
}

int comanVelocityObserver::getChannels()
{
    return _nChannels;
}

int comanVelocityObserver::calibrateSensor()
{
    printf("Calibrate Sensor not yet implemented\n");
    return 0;
}

int comanVelocityObserver::calibrateSensor(const yarp::sig::Vector& value)
{
    printf("Calibrate Sensor not yet implemented\n");
}

int comanVelocityObserver::calibrateChannel(int ch)
{
    printf("Calibrate Channel not yet implemented\n");
}

int comanVelocityObserver::calibrateChannel(int ch, double value)
{
    printf("Calibrate Channel not yet implemented\n");
}
