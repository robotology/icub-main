/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * LGPL-2.1+ license. See the accompanying LICENSE file for details.
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
#include <embObjPSC.h>
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



bool embObjPSC::fromConfig(yarp::os::Searchable &config, servConfigPSC_t &serviceConfig)
{
    ServiceParser parser;
    if(false == parser.parseService(config, serviceConfig))
    {
        yError() << m_PDdevice.getBoardInfo() << ": missing some configuration parameter. Check logs and your config file.";
        return false;
    }

    return true;
}


embObjPSC::embObjPSC(): m_PDdevice("embObjPSC")
{
;
}


embObjPSC::~embObjPSC()
{
    close();
}


bool embObjPSC::initialised()
{
    return m_PDdevice.isOpen();
}




bool embObjPSC::open(yarp::os::Searchable &config)
{
    // 1) prepare Eth service verifing if the eth manager is available and parsing info about the eth board.

    if(! m_PDdevice.prerareEthService(config, this))
        return false;

    // 2) read stuff from config file
    servConfigPSC_t serviceConfig;
    if(!fromConfig(config, serviceConfig))
    {
        yError() << "embObjPSC missing some configuration parameter. Check logs and your config file.";
        return false;
    }

    // 3) prepare data vector
    {
        m_data.resize(eOas_psc_data_maxnumber, 0.0);
    }


    // 4) verify analog sensor protocol and then verify-Activate the PSC service
    if(!m_PDdevice.res->verifyEPprotocol(eoprot_endpoint_analogsensors))
    {
        cleanup();
        return false;
    }


    const eOmn_serv_parameter_t* servparam = &serviceConfig.ethservice;

    if(!m_PDdevice.res->serviceVerifyActivate(eomn_serv_category_psc, servparam, 5.0))
    {
        yError() << m_PDdevice.getBoardInfo() << "open() has an error in call of ethResources::serviceVerifyActivate() ";
        cleanup();
        return false;
    }


    //printServiceConfig();


    if(false == sendConfig2PSCboards(serviceConfig))
    {
        cleanup();
        return false;
    }

    // Set variable to be signaled
    if(false == initRegulars())
    {
        cleanup();
        return false;
    }


    if(!m_PDdevice.res->serviceStart(eomn_serv_category_psc))
    {
        yError() << m_PDdevice.getBoardInfo() << "open() fails to start as service.... cannot continue";
        cleanup();
        return false;
    }
    else
    {
        if(m_PDdevice.isVerbose())
        {
            yDebug() << m_PDdevice.getBoardInfo() << "open() correctly starts service";
        }
    }

    //send enable command to psc boards
    sendStart2PSCboards();

    m_PDdevice.setOpen(true);
    return true;
}


bool embObjPSC::sendConfig2PSCboards(servConfigPSC_t &serviceConfig)
{
    eOprotID32_t id32 = eo_prot_ID32dummy;

    eOas_psc_config_t cfg;
    cfg.datarate  = serviceConfig.acquisitionrate;
    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_psc, 0, eoprot_tag_as_psc_config);

    if(false == m_PDdevice.res->setcheckRemoteValue(id32, &cfg, 10, 0.010, 0.050))
    {
        yError() << m_PDdevice.getBoardInfo() << "FATAL error in sendConfig2PSCboards() while try to configure datarate=" << cfg.datarate;
        return false;
    }

    if(m_PDdevice.isVerbose())
    {
        yDebug() << m_PDdevice.getBoardInfo() << ": sendConfig2PSCboards() correctly configured boards with datarate=" << cfg.datarate;
    }

    return true;

}

bool embObjPSC::sendStart2PSCboards(void)
{
    eOprotID32_t id32 = eo_prot_ID32dummy;

    uint8_t enable=1;

    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_psc, 0, eoprot_tag_as_psc_cmmnds_enable);

    if(false == m_PDdevice.res->setcheckRemoteValue(id32, &enable, 10, 0.010, 0.050))
    {
        yError() << m_PDdevice.getBoardInfo() << "FATAL error in sendStart2PSCboards() while try to enable the boards transmission";
        return false;
    }

    if(m_PDdevice.isVerbose())
    {
        yDebug() << m_PDdevice.getBoardInfo() << ": sendStart2PSCboards() correctly enabled the boards transmission";
    }

    return true;

}


bool embObjPSC::initRegulars(void)
{
    // configure regular rops

    vector<eOprotID32_t> id32v(0);
    eOprotID32_t id32 = eo_prot_ID32dummy;

    // we need to choose the id32 to put inside the vector

    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_psc, 0, eoprot_tag_as_psc_status);;

    // and put it inside vector

    id32v.push_back(id32);

    // now we send the vector

    if(false == m_PDdevice.res->serviceSetRegulars(eomn_serv_category_psc, id32v))
    {
        yError() << m_PDdevice.getBoardInfo() << "initRegulars() fails to add its variables to regulars: cannot proceed any further";
        return false;
    }

    if(m_PDdevice.isVerbose())
    {
        yDebug() << m_PDdevice.getBoardInfo() << "initRegulars() added" << id32v.size() << "regular rops ";
        char nvinfo[128];
        for (size_t r = 0; r<id32v.size(); r++)
        {
            uint32_t item = id32v.at(r);
            eoprot_ID2information(item, nvinfo, sizeof(nvinfo));
            yDebug() << "\t it added regular rop for" << nvinfo;
        }
    }


    return true;
}


/*! Read a vector from the sensor.
 * @param out a vector containing the sensor's last readings.
 * @return AS_OK or return code. AS_TIMEOUT if the sensor timed-out.
 **/

int embObjPSC::read(yarp::sig::Vector &out)
{
    // This method gives the data, received from embedded boards, to the analogServer

    if(!m_PDdevice.isOpen())
        return AS_ERROR ;

    std::lock_guard<std::mutex> lock(m_mutex);

    // errors are not handled for now... it'll always be OK!!
    out=m_data;

    return AS_OK;
}



int embObjPSC::getState(int ch)
{
    printf("getstate\n");
    return AS_OK;
}


int embObjPSC::getChannels()
{
    return static_cast<int>(eOas_psc_data_maxnumber);
}


int embObjPSC::calibrateSensor()
{
    return AS_OK;
}


int embObjPSC::calibrateSensor(const yarp::sig::Vector& value)
{
    return AS_OK;
}


int embObjPSC::calibrateChannel(int ch)
{
    return AS_OK;
}


int embObjPSC::calibrateChannel(int ch, double v)
{
    return AS_OK;
}


eth::iethresType_t embObjPSC::type()
{
    return eth::iethres_analogpsc;
}


bool embObjPSC::update(eOprotID32_t id32, double timestamp, void* rxdata)
{
    // called by feat_manage_analogsensors_data() which is called by:
    // eoprot_fun_UPDT_as_psc_status
    if(!m_PDdevice.isOpen())
        return false;

    eOas_psc_status_t *psc_st_ptr  = (eOas_psc_status_t*)rxdata;
    uint32_t sizeOfData = psc_st_ptr->arrayofdata.head.size;

    if(sizeOfData>m_data.size())
        yWarning() << m_PDdevice.getBoardInfo() << "In update function I received more data than I had been configured to store.My size is " << m_data.size() << "while I received " << sizeOfData << "values!";

    std::lock_guard<std::mutex> lock(m_mutex);

    for(uint32_t i=0; i<sizeOfData; i++)
    {
        eOas_psc_data_t pscdata = psc_st_ptr->arrayofdata.data[i];
        //psc value is in deci-degree
        m_data[i]= pscdata.value*0.1;
    }

    return true;
}




bool embObjPSC::close()
{
    cleanup();
    return true;
}


// void embObjPSC::printServiceConfig(void)
// {
//     char loc[20] = {0};
//     char fir[20] = {0};
//     char pro[20] = {0};
//
//     const char * boardname = (NULL != res) ? (res->getProperties().boardnameString.c_str()) : ("NOT-ASSIGNED-YET");
//     const char * ipv4 = (NULL != res) ? (res->getProperties().ipv4addrString.c_str()) : ("NOT-ASSIGNED-YET");
//
//     parser->convert(serviceConfig.ethservice.configuration.data.as.mais.canloc, loc, sizeof(loc));
//     parser->convert(serviceConfig.ethservice.configuration.data.as.mais.version.firmware, fir, sizeof(fir));
//     parser->convert(serviceConfig.ethservice.configuration.data.as.mais.version.protocol, pro, sizeof(pro));
//
//     yInfo() << "The embObjPSC device using BOARD" << boardname << "w/ IP" << ipv4 << "has the following service config:";
//     yInfo() << "- acquisitionrate =" << serviceConfig.acquisitionrate;
//     yInfo() << "- MAIS named" << serviceConfig.nameOfMais << "@" << loc << "with required protocol version =" << pro << "and required firmware version =" << fir;
// }


void embObjPSC::cleanup(void)
{
    m_PDdevice.cleanup(static_cast <eth::IethResource*> (this));
}

// eof

