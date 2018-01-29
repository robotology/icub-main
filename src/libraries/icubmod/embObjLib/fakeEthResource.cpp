// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Valentina Gaggero
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

// api

#include <fakeEthResource.h>

// other includes

#include "ethResource.h"
#include <ethManager.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/NetType.h>

// embobj
#include "EOropframe_hid.h"
#include "EOarray.h"
#include "EoProtocol.h"
#include "EoManagement.h"
#include "EoProtocolMN.h"
#include "EoProtocolAS.h"

#include "ethParser.h"

using namespace yarp::os;
using namespace yarp::os::impl;

using namespace eth;

// implemention of the class

FakeEthResource::FakeEthResource()
{
    yTrace();

    ipv4addr = 0;
    ipv4addressing.addr = 0;
    ipv4addressing.port = 0;
    ipv4addrstring = "0.0.0.0";
    ethboardtype = eobrd_ethtype_unknown;
    boardTypeString = "unknown";

    ethManager                  = NULL;
    isInRunningMode             = false;
    objLock                     = new Semaphore(1);

    verifiedBoardPresence       = false;
    verifiedBoardTransceiver    = false;

    memset(verifiedEPprotocol, 0, sizeof(verifiedEPprotocol));

    usedNumberOfRegularROPs     = 0;
    memset(&boardCommStatus, 0, sizeof(boardCommStatus));

}


FakeEthResource::~FakeEthResource()
{
    ethManager = NULL;

    delete objLock;
}

bool FakeEthResource::lock(bool on)
{
    if(true == on)
        objLock->wait();
    else
        objLock->post();

    return true;
}


bool FakeEthResource::open2(eOipv4addr_t remIP, yarp::os::Searchable &cfgtotal)
{
    ethManager = eth::TheEthManager::instance();

    eth::parser::pc104Data pc104data;
    eth::parser::read(cfgtotal, pc104data);
//    eth::parser::print(pc104data);


    eth::parser::boardData brddata;
    eth::parser::read(cfgtotal, brddata);
//    eth::parser::print(brddata);

    properties.ipv4addr = remIP;
    properties.ipv4addressing = brddata.properties.ipv4addressing;
    properties.boardtype = brddata.properties.type;
    properties.ipv4addrString = brddata.properties.ipv4string;
    properties.ipv4addressingString = brddata.properties.ipv4addressingstring;
    properties.boardtypeString = brddata.properties.typestring;
    properties.boardnameString = brddata.settings.name;

    // i fill remote address
    ipv4addr = remIP;
    ipv4addrstring = brddata.properties.ipv4string;
    ipv4addressing = brddata.properties.ipv4addressing;

    ethboardtype = brddata.properties.type;
    boardTypeString = brddata.properties.typestring;

    boardName = brddata.settings.name;


    eth::EthMonitorPresence::Config mpConfig;

    // default values ...
    mpConfig.enabled = brddata.actions.monitorpresence_enabled;
    mpConfig.timeout = brddata.actions.monitorpresence_timeout;
    mpConfig.periodmissingreport = brddata.actions.monitorpresence_periodofmissingreport;
    mpConfig.name = ipv4addrstring + " (" + boardName + ")";



    // now i init objects

    lock(true);

    // 1. init transceiver

    eOipv4addressing_t localIPv4 = ethManager->getLocalIPV4addressing();


    if(false == transceiver.init2(this, cfgtotal, localIPv4, remIP))
    {
        yError() << "EthResource::open2() cannot init transceiver w/ HostTransceiver::init2() for BOARD" << boardName << "IP" << ipv4addrstring;
        lock(false);
        return false;
    }

    // 2. init monitor presence

    //monitorpresence.config(mpConfig);
    //monitorpresence.tick();


    lock(false);

    return true;
}



bool FakeEthResource::close()
{
    yTrace();
    return false;
}


const void * FakeEthResource::getUDPtransmit(eOipv4addressing_t &destination, size_t &sizeofpacket, uint16_t &numofrops)
{
    return nullptr;
}


bool FakeEthResource::Tick()
{
    return true;
}


bool FakeEthResource::Check()
{
    return true;
}




bool FakeEthResource::processRXpacket(const void *data, const size_t size)
{
    return true;
}



const AbstractEthResource::Properties & FakeEthResource::getProperties()
{
    return properties;
}


bool FakeEthResource::isRunning(void)
{
    return(isInRunningMode);
}



bool FakeEthResource::verifyEPprotocol(eOprot_endpoint_t ep)
{
    if((uint8_t)ep >= eoprot_endpoints_numberof)
    {
        yError() << "EthResource::verifyEPprotocol() called with wrong ep = " << ep << ": cannot proceed any further";
        return(false);
    }

    if(true == verifiedEPprotocol[ep])
    {
        return(true);
    }

    if(false == verifyBoard())
    {
        yError() << "FakeEthResource::verifyEPprotocol() cannot verify BOARD" << getProperties().boardnameString << "with IP" << getProperties().ipv4addrString << ": cannot proceed any further";
        return(false);
    }

    verifiedEPprotocol[ep] = true;

    return(true);

}



bool FakeEthResource::verifyBoard(void)
{
    return(true);
}

bool FakeEthResource::getRemoteValue(const eOprotID32_t id32, void *value, const double timeout, const unsigned int retries)
{
    return true;
}

bool FakeEthResource::getRemoteValues(const std::vector<eOprotID32_t> &id32s, const std::vector<void*> &values, const double timeout)
{
    return true;
}



bool FakeEthResource::setRemoteValue(const eOprotID32_t id32, void *value)
{
    return true;
}

bool FakeEthResource::setcheckRemoteValue(const eOprotID32_t id32, void *value, const unsigned int retries, const double waitbeforecheck, const double timeout)
{
    return true;
}



bool FakeEthResource::CANPrintHandler(eOmn_info_basic_t *infobasic)
{
    yError() << "FakeEthResource " << boardName << ": should never be in CANPrintHandler";
    return false;
}


bool FakeEthResource::serviceVerifyActivate(eOmn_serv_category_t category, const eOmn_serv_parameter_t* param, double timeout)
{
    return true;
}


bool FakeEthResource::serviceSetRegulars(eOmn_serv_category_t category, vector<eOprotID32_t> &id32vector, double timeout)
{
    return true;
}

bool FakeEthResource::serviceStart(eOmn_serv_category_t category, double timeout)
{
    isInRunningMode = true;
    return true;
}


bool FakeEthResource::serviceStop(eOmn_serv_category_t category, double timeout)
{
    return true; 
}

bool FakeEthResource::getLocalValue(const eOprotID32_t id32, void *data)
{
    bool ret = transceiver.read(id32, data);
    if(false == ret)
        return ret;
    
    //manage special case:
    if(id32 == eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_status_fullscale))
    {
        //I need to set size to 6, else the device strain can't be opened
        eOas_arrayofupto12bytes_t * fullscale = (eOas_arrayofupto12bytes_t *)data;
        fullscale->head.size = 6;
    }
    
    return ret;
}

bool FakeEthResource::setLocalValue(eOprotID32_t id32, const void *value, bool overrideROprotection)
{
    return transceiver.write(id32, value, overrideROprotection);
    //return true;
}

//bool FakeEthResource::addSetMessage(eOprotID32_t id32, uint8_t* data)
//{
//    return true;
//}

//bool FakeEthResource::addGetMessage(eOprotID32_t id32)
//{
//    return true;
//}

//bool FakeEthResource::addGetMessage(eOprotID32_t id32, std::uint32_t signature)
//{
//    return true;
//}

//bool FakeEthResource::addSetMessageAndCacheLocally(eOprotID32_t id32, uint8_t* data)
//{
//    return true;
//}

//bool FakeEthResource::readSentValue(eOprotID32_t id32, uint8_t *data, uint16_t* size)
//{
//    *size=0;
//    return true;
//}

//EOnv* FakeEthResource::getNVhandler(eOprotID32_t id32, EOnv* nv)
//{
//    return transceiver.getnvhandler(id32, nv);
//}

bool FakeEthResource::isFake()
{
    return true;
}

HostTransceiver * FakeEthResource::getTransceiver()
{
    return nullptr;
}

//bool FakeEthResource::isID32supported(eOprotID32_t id32)
//{
//    return true;
//}



// eof

