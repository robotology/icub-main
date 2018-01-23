
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2013 Robotcub Consortium
* Author: Alberto Cardellino
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

// general purpose stuff.
#include <string>
#include <iostream>
#include <string.h>

// Yarp Includes
#include <yarp/os/Time.h>
#include <stdarg.h>
#include <stdio.h>
#include <yarp/dev/PolyDriver.h>
#include <ace/config.h>
#include <ace/Log_Msg.h>


// specific to this device driver.
#include <embObjVirtualAnalogSensor.h>
#include <ethManager.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/dev/ControlBoardInterfacesImpl.inl>

#include "EoProtocol.h"
#include "EoMotionControl.h"
#include "EoProtocolMC.h"

#ifdef WIN32
#pragma warning(once:4355)
#endif


using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;


//generic function that check is key1 is present in input bottle and that the result has size elements
// return true/false
static inline bool extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    size++;  // size includes also the name of the parameter
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        yError("%s not found\n", key1.c_str());
        return false;
    }

    if(tmp.size()!=size)
    {
        yError("%s incorrect number of entries\n", key1.c_str());
        return false;
    }

    out=tmp;

    return true;
}

bool embObjVirtualAnalogSensor::fromConfig(yarp::os::Searchable &_config)
{
    Bottle xtmp;

    // Analog Sensor stuff
    Bottle config = _config.findGroup("GENERAL");

    if (!extractGroup(config, xtmp, "Channels","Number of channels of the Analog Sensor", 1))
    {
        yError() << "embObjVirtualAnalogSensor: Missing channel number... aborting";
        _channels = 0;
        return false;
    }
    else
    {
        _channels = xtmp.get(1).asInt();
    }

    // alloc vector for keeping conversion values
    _fullscale = allocAndCheck<double>(_channels);
    _resolution = allocAndCheck<double>(_channels);

    if (!extractGroup(config, xtmp, "UseCalibration","Calibration parameters are needed", 1))
    {
        return false;
    }
    else
    {
        _useCalibration = xtmp.get(1).asInt();
    }

    if (!extractGroup(config, xtmp, "MaxValue","full scale value for this measure", _channels))
    {
        yError() << "Missing conversion factor!! Aborting...";
        return false;
    }
    else
    {
        if(_verbose)
            yDebug() << "embObjVirtualAnalogSensor, fullscales from config file: ";

        for (int ch = 1; ch < xtmp.size(); ch++)
        {
            _fullscale[ch-1] = xtmp.get(ch).asDouble();

            if(_verbose)
                yDebug() << "ch " << ch << ": " << _fullscale[ch-1];
        }
    }

    if (!extractGroup(config, xtmp, "Resolution","Number of bytes used for this measure", _channels))
    {
        yError() << "Missing resolution!! Aborting...";
        return false;
    }
    else
    {
        if(_verbose)
            yDebug() << "embObjVirtualAnalogSensor, resolutions from config file: ";


        for (int ch = 1; ch < xtmp.size(); ch++)
        {
            _resolution[ch-1] = (double) (1 << (xtmp.get(ch).asInt()-1) );
            if(_verbose)
                yDebug() << "ch " << ch << ": " << _resolution[ch-1];
        }
    }
    return true;
}


embObjVirtualAnalogSensor::embObjVirtualAnalogSensor()
{
    _fullscale      = NULL;
    _resolution     = NULL;
    _useCalibration = 0;
    _channels       = 0;
    _verbose        = false;
    _status         = VAS_OK;
    opened          =  false;
}

embObjVirtualAnalogSensor::~embObjVirtualAnalogSensor()
{

}

bool embObjVirtualAnalogSensor::initialised()
{
    return opened;
}

bool embObjVirtualAnalogSensor::update(eOprotID32_t id32, double timestamp, void *rxdata)
{
    return true;
}

eth::iethresType_t embObjVirtualAnalogSensor::type()
{
    return eth::iethres_analogvirtual;
}



bool embObjVirtualAnalogSensor::open(yarp::os::Searchable &config)
{
    // - first thing to do is verify if the eth manager is available. then i parse info about the eth board.

    ethManager = eth::TheEthManager::instance();
    if(NULL == ethManager)
    {
        yFatal() << "embObjVirtualAnalogSensor::open() fails to instantiate ethManager";
        return false;
    }


    if(false == ethManager->verifyEthBoardInfo(config, ipv4addr, boardIPstring, boardName))
    {
        yError() << "embObjVirtualAnalogSensor::open(): object TheEthManager fails in parsing ETH propertiex from xml file";
        return false;
    }
    // add specific info about this device ...



    // - now all other things


    std::string str;
    if(config.findGroup("GENERAL").find("verbose").asBool())
    {
        str=config.toString().c_str();
        _verbose = true;
    }
    else
        str=" ";

    yTrace() << str;

    // Read stuff from config file
    if(!fromConfig(config))
    {
        yError() << "embObjAnalogSensor missing some configuration parameter. Check logs and your config file.";
        return false;
    }


    // -- instantiate EthResource etc.

    res = ethManager->requestResource2(this, config);
    if(NULL == res)
    {
        yError() << "embObjVirtualAnalogSensor::open() fails because could not instantiate the ethResource for BOARD w/ IP = " << boardIPstring << " ... unable to continue";
        return false;
    }

    // i verify the motion-control because .... in here we use messages of this endpoint ... and we need to verfy in order to send messages.
    if(!res->verifyEPprotocol(eoprot_endpoint_motioncontrol))
    {
        cleanup();
        return false;
    }



    yTrace() << "embObjVirtualAnalogSensor::open(): succefully called for BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString << "instantiated correctly";

    opened = true;
    return true;
}

/*
 * IVirtualAnalogSensor Interface
 *
 */

IVirtualAnalogSensor::VAS_status embObjVirtualAnalogSensor::getVirtualAnalogSensorStatus(int ch)
{
    return VAS_OK;
};

int embObjVirtualAnalogSensor::getVirtualAnalogSensorChannels()
{
    return _channels;
};

bool embObjVirtualAnalogSensor::updateVirtualAnalogSensorMeasure(yarp::sig::Vector &measure)
{
    bool ret = true;
    if(measure.size() != _channels)
    {
        yError() << "Vector of measures has a different size from channel number!! Skipping";
        return false;
    }

    for(int ch=0; ch< _channels; ch++)
    {
        ret &= updateVirtualAnalogSensorMeasure(ch, measure[ch]);
    }
    return true;
}

bool embObjVirtualAnalogSensor::updateVirtualAnalogSensorMeasure(int ch, double &measure)
{
    if (measure < ( - _fullscale[ch]) )
        measure =  (-_fullscale[ch]);

    if (measure > _fullscale[ch] )
        measure = _fullscale[ch];

    // Here measure is supposed to be a Torque
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, ch, eoprot_tag_mc_joint_inputs_externallymeasuredtorque);
    //    example measure * 32768.0/12.0;
    // measure should to saturated to resolution -2.0 to avoid casting problem.
    eOmeas_torque_t meas_torque = (eOmeas_torque_t)( measure * ((_resolution[ch]-2.0)/_fullscale[ch]));

    // i write also locally because somebody may read it back later
    res->setLocalValue(protid, &meas_torque);
    // and i want also to send it to the board
    return res->setRemoteValue(protid, &meas_torque);
}

void embObjVirtualAnalogSensor::cleanup(void)
{
    yTrace() << "embObjVirtualAnalogSensor::cleanup(): called for BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString;

    if(_fullscale != NULL)
        delete(_fullscale);

    if(ethManager == NULL) return;

    int ret = ethManager->releaseResource2(res, this);
    res = NULL;
    if(ret == -1)
        ethManager->killYourself();
}

bool embObjVirtualAnalogSensor::close()
{
    cleanup();
    return true;
}




// eof


