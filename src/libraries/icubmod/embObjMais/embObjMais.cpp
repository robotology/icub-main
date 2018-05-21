
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2012 Robotcub Consortium
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
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <stdarg.h>
#include <stdio.h>
#include <yarp/dev/PolyDriver.h>
#include <ace/config.h>
#include <ace/Log_Msg.h>


// specific to this device driver.
#include <embObjMais.h>
#include <ethManager.h>
#include <yarp/os/LogStream.h>
#include "EoAnalogSensors.h"
#include "EOnv_hid.h"

#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolAS.h"

#include <yarp/os/NetType.h>

#ifdef WIN32
#pragma warning(once:4355)
#endif



using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;


inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    yWarning() << std::string(txt) << " not yet implemented for embObjMais\n";
    return false;
}

// generic function that checks is key1 is present in input bottle and that the result has size elements
// return true/false
bool embObjMais::extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    size++;  // size includes also the name of the parameter
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        yError ("%s not found\n", key1.c_str());
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


bool embObjMais::fromConfig(yarp::os::Searchable &_config)
{
#if defined(EMBOBJMAIS_USESERVICEPARSER)


    if(false == parser->parseService(_config, serviceConfig))
    {
        return false;
    }

    return true;

#else

    Bottle xtmp;

    int _period = 0;

    // Analog Sensor stuff
    Bottle config = _config.findGroup("GENERAL");
    if (!extractGroup(config, xtmp, "Period","transmitting period of the sensors", 1))
    {
        yError() << "embObjMais Using default value = 0 (disabled)";
        _period = 0;

    }
    else
    {
        _period = xtmp.get(1).asInt();
        yDebug() << "embObjMais::fromConfig() detects embObjMais Using value of" << _period;
    }

    serviceConfig.acquisitionrate = _period;

    return true;
#endif
}


embObjMais::embObjMais()
{
    serviceConfig.acquisitionrate = 0;
    memset(&serviceConfig.ethservice, 0, sizeof(serviceConfig.ethservice));

    timeStamp = 0;

    counterSat=0;
    counterError=0;
    counterTimeout=0;

    status = IAnalogSensor::AS_OK;


    opened = false;

    analogdata.resize(0);


    std::string tmp = NetworkBase::getEnvironment("ETH_VERBOSEWHENOK");
    if (tmp != "")
    {
        verbosewhenok = (bool)NetType::toInt(tmp);
    }
    else
    {
        verbosewhenok = false;
    }

    parser = NULL;
    res = NULL;
}


embObjMais::~embObjMais()
{   
    analogdata.resize(0);

    if(NULL != parser)
    {
        delete parser;
        parser = NULL;
    }

}


bool embObjMais::initialised()
{
    return opened;
}

//#define TEST_MAIS_PLUS_MC


bool embObjMais::open(yarp::os::Searchable &config)
{
    // - first thing to do is verify if the eth manager is available. then i parse info about the eth board.

    ethManager = eth::TheEthManager::instance();
    if(NULL == ethManager)
    {
        yFatal() << "embObjMais::open() fails to instantiate ethManager";
        return false;
    }


    if(false == ethManager->verifyEthBoardInfo(config, ipv4addr, boardIPstring, boardName))
    {
        yError() << "embObjMais::open(): object TheEthManager fails in parsing ETH propertiex from xml file";
        return false;
    }
    // add specific info about this device ...


    // - now all other things


//    std::string str;
//    if(config.findGroup("GENERAL").find("verbose").asBool())
//        str=config.toString().c_str();
//    else
//        str="\n";
//    yTrace() << str;


    if(NULL == parser)
    {
        parser = new ServiceParser;
    }

    // read stuff from config file
    if(!fromConfig(config))
    {
        yError() << "embObjMais missing some configuration parameter. Check logs and your config file.";
        return false;
    }

    // and prepare analogdata
    {
        // must be of size: mais_Channels, and 0.0-initted
        analogdata.resize(mais_Channels, 0.0);
    }



    // -- instantiate EthResource etc.

    res = ethManager->requestResource2(this, config);
    if(NULL == res)
    {
        yError() << "embObjMais::open() fails because could not instantiate the ethResource for BOARD w/ IP = " << boardIPstring << " ... unable to continue";
        return false;
    }

    printServiceConfig();


    if(!res->verifyEPprotocol(eoprot_endpoint_analogsensors))
    {
        cleanup();
        return false;
    }

#if     defined(TEST_MAIS_PLUS_MC)

   // sarebbe bene chiamare requestResource2(ptr2fakemccontroller, config) ma pazienza

    const eOmn_serv_parameter_t* mcservparam = NULL;

    if(false == res->serviceVerifyActivate(eomn_serv_category_mc, mcservparam, 5.0))
    {
        yError() << "embObjMais::open() has an error in call of ethResources::serviceVerifyActivate(MC) for BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString;
        cleanup();
        return false;
    }
    if(false == res->serviceStart(eomn_serv_category_mc))
    {
        yError() << "embObjMais::open() fails to start MC service for BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString << ": cannot continue";
        cleanup();
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjMais::open() correctly starts MC service of BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString;
        }
    }


#endif






#if defined(EMBOBJMAIS_USESERVICEPARSER)
    const eOmn_serv_parameter_t* servparam = &serviceConfig.ethservice;
#else
    const eOmn_serv_parameter_t* servparam = NULL;
#endif

    //servparam = NULL;

    if(false == res->serviceVerifyActivate(eomn_serv_category_mais, servparam, 5.0))
    {
        SystemClock::delaySystem(1);
        yError() << "embObjMais::open() has an error in call of ethResources::serviceVerifyActivate() for BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString;
        //printServiceConfig();
        cleanup();
        return false;
    }

    //printServiceConfig();


    // configure the service: aka, send to the remote board information about the whereabouts of the can boards mais, strain, mtb which offers the service.
    // so far nothing to do


    // configure the sensor(s)


    if(false == sendConfig2Mais())
    {
        cleanup();
        return false;
    }

    // Set variable to be signalled
    if(false == initRegulars())
    {
        cleanup();
        return false;
    }


    if(false == res->serviceStart(eomn_serv_category_mais))
    {
        yError() << "embObjMais::open() fails to start as service for  BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString << ": cannot continue";
        cleanup();
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjMais::open() correctly starts as service of BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString;
        }
    }


    opened = true;
    return true;
}


bool embObjMais::sendConfig2Mais(void)
{
    // version with read-back

    eOprotID32_t id32 = eo_prot_ID32dummy;

    // -- mais datarate

    uint8_t datarate  = serviceConfig.acquisitionrate;
    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_config_datarate);

    if(false == res->setcheckRemoteValue(id32, &datarate, 10, 0.010, 0.050))
    {
        yError() << "FATAL: embObjMais::sendConfig2Mais() had an error while calling setcheckRemoteValue() for mais datarate in BOARD" << res->getProperties().boardnameString << "with IP" << res->getProperties().ipv4addrString;
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjMais::sendConfig2Mais() correctly configured mais datarate at value" << datarate << "in BOARD" << res->getProperties().boardnameString << "with IP" << res->getProperties().ipv4addrString;
        }
    }

    // -- mais tx mode

    eOenum08_t maismode  = eoas_maismode_txdatacontinuously; // use eOas_maismode_t for value BUT USE   for type (their sizes can be different !!)
    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_config_mode);

    if(false == res->setcheckRemoteValue(id32, &maismode, 10, 0.010, 0.050))
    {
        yError() << "FATAL: embObjMais::sendConfig2Mais() had an error while calling setcheckRemoteValue() for mais mode in BOARD" << res->getProperties().boardnameString << "with IP" << res->getProperties().ipv4addrString;
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjMais::sendConfig2Mais() correctly configured mais mode at value" << maismode << "in BOARD" << res->getProperties().boardnameString << "with IP" << res->getProperties().ipv4addrString;
        }
    }

    return true;

}



bool embObjMais::initRegulars()
{
    // configure regular rops

    vector<eOprotID32_t> id32v(0);
    eOprotID32_t id32 = eo_prot_ID32dummy;

    // we need to choose the id32 to put inside the vector

    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_status_the15values);;

    // and put it inside vector

    id32v.push_back(id32);

    // now we send the vector

    if(false == res->serviceSetRegulars(eomn_serv_category_mais, id32v))
    {
        yError() << "embObjMais::initRegulars() fails to add its variables to regulars: cannot proceed any further";
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjMais::initRegulars() added" << id32v.size() << "regular rops to BOARD" << res->getProperties().boardnameString << "with IP" << res->getProperties().ipv4addrString;
            char nvinfo[128];
            for (size_t r = 0; r<id32v.size(); r++)
            {
                uint32_t item = id32v.at(r);
                eoprot_ID2information(item, nvinfo, sizeof(nvinfo));
                yDebug() << "\t it added regular rop for" << nvinfo;
            }
        }
    }
   
    return true;
}


/*! Read a vector from the sensor.
 * @param out a vector containing the sensor's last readings.
 * @return AS_OK or return code. AS_TIMEOUT if the sensor timed-out.
 **/

int embObjMais::read(yarp::sig::Vector &out)
{
    // This method gives analogdata to the analogServer

    if(false == opened)
    {
        return false;
    }

    mutex.wait();


    // errors are not handled for now... it'll always be OK!!
    if (status != IAnalogSensor::AS_OK)
    {
        switch (status)
        {
            case IAnalogSensor::AS_OVF:
            {
                counterSat++;
            }  break;
            case IAnalogSensor::AS_ERROR:
            {
                counterError++;
            } break;
            case IAnalogSensor::AS_TIMEOUT:
            {
                counterTimeout++;
            } break;
            default:
            {
                counterError++;
            } break;
        }
        mutex.post();
        return status;
    }

    out.resize(analogdata.size());
    for (size_t k = 0; k<analogdata.size(); k++)
    {
        out[k] = analogdata[k];
    }


    mutex.post();

    return status;
}


void embObjMais::resetCounters()
{
    counterSat=0;
    counterError=0;
    counterTimeout=0;
}


void embObjMais::getCounters(unsigned int &sat, unsigned int &err, unsigned int &to)
{
    sat=counterSat;
    err=counterError;
    to=counterTimeout;
}


int embObjMais::getState(int ch)
{
    printf("getstate\n");
    return AS_OK;
}


int embObjMais::getChannels()
{
    return analogdata.size();
}


int embObjMais::calibrateSensor()
{
    return AS_OK;
}


int embObjMais::calibrateSensor(const yarp::sig::Vector& value)
{
    return AS_OK;
}


int embObjMais::calibrateChannel(int ch)
{
    return AS_OK;
}


int embObjMais::calibrateChannel(int ch, double v)
{
    return AS_OK;
}


eth::iethresType_t embObjMais::type()
{
    return eth::iethres_analogmais;
}


bool embObjMais::update(eOprotID32_t id32, double timestamp, void* rxdata)
{   
    id32 = id32;
    timestamp = timestamp;

    if(false == opened)
    {
        return false;
    }

    // called by feat_manage_analogsensors_data() which is called by:
    // eoprot_fun_UPDT_as_mais_status_the15values()
    // the void* parameter inside this function is a eOas_arrayofupto36bytes_t*
    // and can be treated as a EOarray


    EOarray *array = (EOarray*)rxdata;
    uint8_t size = eo_array_Size(array);
    uint8_t itemsize = eo_array_ItemSize(array); // marco.accame: must be 1, as the code after uses this convention
    if((0 == size) || (1 != itemsize))
    {
        return false;
    }

    mutex.wait();

    for (size_t k = 0; k<analogdata.size(); k++)
    {
        uint8_t* tmp = (uint8_t*) eo_array_At(array, k);
        if(NULL != tmp)
        {
            uint8_t val = *tmp; // marco.accame: i see that we treat the array as if containing items of 1 byte.
            // Get the kth element of the array
            analogdata[k] = (double)val;
        }
    }

    mutex.post();

    return true;
}




bool embObjMais::close()
{
    opened = false;

    cleanup();
    return true;
}


void embObjMais::printServiceConfig(void)
{
    char loc[20] = {0};
    char fir[20] = {0};
    char pro[20] = {0};

    const char * boardname = (NULL != res) ? (res->getProperties().boardnameString.c_str()) : ("NOT-ASSIGNED-YET");
    const char * ipv4 = (NULL != res) ? (res->getProperties().ipv4addrString.c_str()) : ("NOT-ASSIGNED-YET");

    parser->convert(serviceConfig.ethservice.configuration.data.as.mais.canloc, loc, sizeof(loc));
    parser->convert(serviceConfig.ethservice.configuration.data.as.mais.version.firmware, fir, sizeof(fir));
    parser->convert(serviceConfig.ethservice.configuration.data.as.mais.version.protocol, pro, sizeof(pro));

    yInfo() << "The embObjMais device using BOARD" << boardname << "w/ IP" << ipv4 << "has the following service config:";
    yInfo() << "- acquisitionrate =" << serviceConfig.acquisitionrate;
    yInfo() << "- MAIS named" << serviceConfig.nameOfMais << "@" << loc << "with required protocol version =" << pro << "and required firmware version =" << fir;
}


void embObjMais::cleanup(void)
{
    if(ethManager == NULL) return;

    int ret = ethManager->releaseResource2(res, this);
    res = NULL;
    if(ret == -1)
        ethManager->killYourself();
}

// eof

