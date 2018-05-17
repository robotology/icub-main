/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */


#include <yarp/os/Network.h>
#include <yarp/os/NetType.h>
#include "embObjGeneralDevPrivData.h"

using namespace yarp::dev;

yarp::dev::embObjDevPrivData::embObjDevPrivData(yarp::os::ConstString name):deviceNameType(name)
{
    ethManager = nullptr;
    res = nullptr;
    behFlags.opened = false;

    yarp::os::ConstString tmp = NetworkBase::getEnvironment("ETH_VERBOSEWHENOK");
    if (tmp != "")
    {
        behFlags.verbosewhenok = (bool)NetType::toInt(tmp);
    }
    else
    {
        behFlags.verbosewhenok = false;
    }
}

yarp::dev::embObjDevPrivData::~embObjDevPrivData()
{;}

std::string yarp::dev::embObjDevPrivData::getBoardInfo(void) const
{
    if(nullptr == res)
    {
        return deviceNameType + ": BOARD name_unknown (IP unknown) ";
    }
    else
    {
        return (deviceNameType + ": BOARD " + res->getProperties().boardnameString +  " (IP "  + res->getProperties().ipv4addrString + ") ");
    }
}


bool yarp::dev::embObjDevPrivData::prerareEthService(Searchable& config, eth::IethResource *interface)
{
    // - first thing to do is verify if the eth manager is available. then i parse info about the eth board.

    ethManager = eth::TheEthManager::instance();
    if(NULL == ethManager)
    {
        yFatal() <<  deviceNameType << " fails to instantiate ethManager";
        return false;
    }

    string boardIPstring;
    string boardName;
    eOipv4addr_t ipv4addr;


    if(false == ethManager->verifyEthBoardInfo(config, ipv4addr, boardIPstring, boardName))
    {
        yError() << deviceNameType <<  " object TheEthManager fails in parsing ETH propertiex from xml file";
        return false;
    }

    res = ethManager->requestResource2(interface, config);
    if(NULL == res)
    {
        yError() << deviceNameType <<  ":requestResource fails because could not instantiate the ethResource for BOARD " << getBoardInfo() << " ... unable to continue";
        return false;
    }

    return true;
}

void yarp::dev::embObjDevPrivData::cleanup(eth::IethResource *interface)
{
    if(ethManager == NULL) return;

    int ret = ethManager->releaseResource2(res, interface);
    res = NULL;
    if(ret == -1)
        ethManager->killYourself();
    behFlags.opened = false;
}

