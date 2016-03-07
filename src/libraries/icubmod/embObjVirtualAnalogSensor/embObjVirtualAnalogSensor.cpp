
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

  // embObj parameters, in ETH group
    Value val =_config.findGroup("ETH").check("Ems",Value(1), "Board number");
    if(val.isInt())
    {
        _fId.boardNumber =val.asInt();
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

iethresType_t embObjVirtualAnalogSensor::type()
{
    return iethres_analogvirtual;
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

    Bottle groupTransceiver = Bottle(config.findGroup("TRANSCEIVER"));
    if(groupTransceiver.isNull())
    {
        yError() << "embObjVirtualAnalogSensor: Can't find TRANSCEIVER group in config files";
        return false;
    }

    Bottle groupProtocol = Bottle(config.findGroup("PROTOCOL"));
    if(groupProtocol.isNull())
    {
        yError() << "embObjVirtualAnalogSensor: Can't find PROTOCOL group in config files";
        return false;
    }


    // Get both PC104 and EMS ip addresses and port from config file
    groupEth  = Bottle(config.findGroup("ETH"));
    Bottle parameter1( groupEth.find("PC104IpAddress").asString() );    // .findGroup("IpAddress");
    port      = groupEth.find("CmdPort").asInt();              // .get(1).asInt();
    snprintf(_fId.pc104IPaddr.string, sizeof(_fId.pc104IPaddr.string), "%s", parameter1.toString().c_str());
    _fId.pc104IPaddr.port = port;

    Bottle parameter2( groupEth.find("IpAddress").asString() );    // .findGroup("IpAddress");
    snprintf(_fId.boardIPaddr.string, sizeof(_fId.boardIPaddr.string), "%s", parameter2.toString().c_str());
    _fId.boardIPaddr.port = port;

    sscanf(_fId.boardIPaddr.string,"\"%d.%d.%d.%d", &_fId.boardIPaddr.ip1, &_fId.boardIPaddr.ip2, &_fId.boardIPaddr.ip3, &_fId.boardIPaddr.ip4);
    sscanf(_fId.pc104IPaddr.string,"\"%d.%d.%d.%d", &_fId.pc104IPaddr.ip1, &_fId.pc104IPaddr.ip2, &_fId.pc104IPaddr.ip3, &_fId.pc104IPaddr.ip4);

    snprintf(_fId.boardIPaddr.string, sizeof(_fId.boardIPaddr.string), "%u.%u.%u.%u:%u", _fId.boardIPaddr.ip1, _fId.boardIPaddr.ip2, _fId.boardIPaddr.ip3, _fId.boardIPaddr.ip4, _fId.boardIPaddr.port);
    snprintf(_fId.pc104IPaddr.string, sizeof(_fId.pc104IPaddr.string), "%u.%u.%u.%u:%u", _fId.pc104IPaddr.ip1, _fId.pc104IPaddr.ip2, _fId.pc104IPaddr.ip3, _fId.pc104IPaddr.ip4, _fId.pc104IPaddr.port);

    //   Debug info
    snprintf(_fId.name, sizeof(_fId.name), "embObjAnalogSensor: referred to EMS: %d at address %s\n", _fId.boardNumber, address);       // Saving User Friendly Id


    Value val =config.findGroup("ETH").check("Ems",Value(1), "Board number");
    if(val.isInt())
        _fId.boardNumber =val.asInt();
    else
    {
        yError () << "embObjAnalogSensor: EMS Board number identifier not found for IP" << _fId.pc104IPaddr.string;
        return false;
    }

    _fId.boardName[0] = '\0';
    Value *valName;
    if(config.findGroup("ETH").check("Name", valName, "Board name"))
    {
        if(valName->isString())
        {
            memset(_fId.boardName, 0, BOARDNAME_MAXSIZE);
            snprintf(_fId.boardName, BOARDNAME_MAXSIZE, "%s", valName->asString().c_str());
        }
        else
        {
            yError () << "embObjAnalogSensor: EMS Board name is not valid";
        }
    }

    ethManager = TheEthManager::instance();
    if(NULL == ethManager)
    {
        yError() << "Unable to instantiate ethManager";
        return false;
    }

    // N.B.: use a dynamic_cast to extract correct interface when using this pointer
    _fId.interface = this;
    _fId.type = ethFeatType_AnalogVirtual;

    res = ethManager->requestResource2(this, config);
    if(NULL == res)
    {
        yError() << "embObjMotionControl::open() fails because could not instantiate the ethResource for board" << _fId.boardNumber << " ... unable to continue";
        return false;
    }

    /*IMPORTANT: implement isEpManagedByBoard like every embObj obj when virtaulAnalogSensor will be exist in eo proto!!!!*/
//    if(!isEpManagedByBoard())
//    {
//        yError() << "EMS "<< _fId.boardNumber << "is not connected to virtual analog sensor";
//        return false;
//    }
//    if(!res->verifyProtocol(groupProtocol, eoprot_endpoint_???))
//    {
//        yError() << "embObjVirtualAnalogSensor and board "<< _fId.boardNumber << "dont not have the same eoprot_endpoint_??? protocol version: DO A FW UPGRADE";
//        return false;
//    }

    yTrace() << "EmbObj Virtual Analog Sensor for board "<< _fId.boardNumber << "instantiated correctly";

    opened = true;
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

void embObjVirtualAnalogSensor::cleanup(void)
{
    yTrace() << _fId.name;
    if(_fullscale != NULL)
        delete(_fullscale);

    if(ethManager == NULL) return;

    int ret = ethManager->releaseResource(_fId);
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


