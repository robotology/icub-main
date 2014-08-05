
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
#include "FeatureInterface_hid.h"         // Interface with embObj world (callback)
#include <embObjVirtualAnalogSensor.h>
#include <ethManager.h>
#include <Debug.h>

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
        fprintf(stderr, "%s not found\n", key1.c_str());
        return false;
    }

    if(tmp.size()!=size)
    {
        fprintf(stderr, "%s incorrect number of entries\n", key1.c_str());
        return false;
    }

    out=tmp;

    return true;
}

bool embObjVirtualAnalogSensor::fromConfig(yarp::os::Searchable &_config)
{
    Bottle xtmp;

  // embObj parameters, in ETH group
    Value val =_config.findGroup("ETH").check("Ems",Value(1), "Board number");
    if(val.isInt())
    {
        _fId.boardNum =val.asInt();
    }
    else
    {
        yError () << "embObjVirtualAnalogSensor: EMS Board number identifier not found\n";
        return false;
    }

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
        fprintf(stderr, "embObjVirtualAnalogSensor: Using default value = 0 (Don't use calibration)\n");
        _useCalibration = 0;
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
};


embObjVirtualAnalogSensor::embObjVirtualAnalogSensor()
{
    _fullscale      = NULL;
    _resolution     = NULL;
    _useCalibration = 0;
    _channels       = 0;
    _verbose        = false;
    _status         = VAS_OK;
}

embObjVirtualAnalogSensor::~embObjVirtualAnalogSensor()
{

}

bool embObjVirtualAnalogSensor::open(yarp::os::Searchable &config)
{
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

    // Tmp variables
    Bottle          groupEth;
    ACE_TCHAR       address[64];
    ACE_UINT16      port;

    Bottle groupProtocol = Bottle(config.findGroup("PROTOCOL"));
    if(groupProtocol.isNull())
    {
        yWarning() << "embObjVirtualAnalogSensor: Can't find PROTOCOL group in config files ... using max capabilities";
        //return false;
    }


    // Get both PC104 and EMS ip addresses and port from config file
    groupEth  = Bottle(config.findGroup("ETH"));
    Bottle parameter1( groupEth.find("PC104IpAddress").asString() );    // .findGroup("IpAddress");
    port      = groupEth.find("CmdPort").asInt();              // .get(1).asInt();
    snprintf(_fId.PC104ipAddr.string, sizeof(_fId.PC104ipAddr.string), "%s", parameter1.toString().c_str());
    _fId.PC104ipAddr.port = port;

    Bottle parameter2( groupEth.find("IpAddress").asString() );    // .findGroup("IpAddress");
    snprintf(_fId.EMSipAddr.string, sizeof(_fId.EMSipAddr.string), "%s", parameter2.toString().c_str());
    _fId.EMSipAddr.port = port;

    sscanf(_fId.EMSipAddr.string,"\"%d.%d.%d.%d", &_fId.EMSipAddr.ip1, &_fId.EMSipAddr.ip2, &_fId.EMSipAddr.ip3, &_fId.EMSipAddr.ip4);
    sscanf(_fId.PC104ipAddr.string,"\"%d.%d.%d.%d", &_fId.PC104ipAddr.ip1, &_fId.PC104ipAddr.ip2, &_fId.PC104ipAddr.ip3, &_fId.PC104ipAddr.ip4);

    snprintf(_fId.EMSipAddr.string, sizeof(_fId.EMSipAddr.string), "%u.%u.%u.%u:%u", _fId.EMSipAddr.ip1, _fId.EMSipAddr.ip2, _fId.EMSipAddr.ip3, _fId.EMSipAddr.ip4, _fId.EMSipAddr.port);
    snprintf(_fId.PC104ipAddr.string, sizeof(_fId.PC104ipAddr.string), "%u.%u.%u.%u:%u", _fId.PC104ipAddr.ip1, _fId.PC104ipAddr.ip2, _fId.PC104ipAddr.ip3, _fId.PC104ipAddr.ip4, _fId.PC104ipAddr.port);

    //   Debug info
    snprintf(_fId.name, sizeof(_fId.name), "embObjAnalogSensor: referred to EMS: %d at address %s\n", _fId.boardNum, address);       // Saving User Friendly Id

    // Set dummy values
    _fId.boardNum   = FEAT_boardnumber_dummy;
    _fId.ep         = eoprot_endpoint_none;

    Value val =config.findGroup("ETH").check("Ems",Value(1), "Board number");
    if(val.isInt())
        _fId.boardNum =val.asInt();
    else
    {
        yError () << "embObjAnalogSensor: EMS Board number identifier not found for IP" << _fId.PC104ipAddr.string;
        return false;
    }


    ethManager = TheEthManager::instance();
    if(NULL == ethManager)
    {
        yFatal() << "Unable to instantiate ethManager";
        return false;
    }

    //N.B.: use a dynamic_cast to extract correct interface when using this pointer
    _fId.handle = (this);

    /* Once I'm ok, ask for resources, through the _fId struct I'll give the ip addr, port and
    *  and boradNum to the ethManagerin order to create the ethResource requested.
    * I'll Get back the very same sturct filled with other data useful for future handling
    * like the EPvector and EPhash_function */
    res = ethManager->requestResource(config, &_fId);
    if(NULL == res)
    {
        yError() << "EMS device not instantiated... unable to continue";
        return false;
    }

    /*IMPORTANT: implement isEpManagedByBoard like every embObj obj when virtaulAnalogSensor will be exist in eo proto!!!!*/
//    if(!isEpManagedByBoard())
//    {
//        yError() << "EMS "<< _fId.boardNum << "is not connected to virtual analog sensor";
//        return false;
//    }


    yTrace() << "EmbObj Virtual Analog Sensor for board "<< _fId.boardNum << "instantiated correctly";
    return true;
}

/*
 * IVirtualAnalogSensor Interface
 *
 */

int embObjVirtualAnalogSensor::getState(int ch)
{
    return VAS_OK;
};

int embObjVirtualAnalogSensor::getChannels()
{
    return _channels;
};

bool embObjVirtualAnalogSensor::updateMeasure(yarp::sig::Vector &measure)
{
    bool ret = true;
    if(measure.size() != _channels)
    {
        yError() << "Vector of measures has a different size from channel number!! Skipping";
        return false;
    }

    for(int ch=0; ch< _channels; ch++)
    {
        ret &= updateMeasure(ch, measure[ch]);
    }
    return true;
}

bool embObjVirtualAnalogSensor::updateMeasure(int ch, double &measure)
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
    return res->addSetMessageAndCacheLocally(protid, (uint8_t*) &meas_torque);
}

bool embObjVirtualAnalogSensor::close()
{
    yTrace() << _fId.name;
    if(_fullscale != NULL)
        delete(_fullscale);

    int ret = ethManager->releaseResource(_fId);
    if(ret == -1)
    {
        ethManager->killYourself();
    }
    res = NULL;
    return true;
}

// eof


