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

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;

using namespace eth;

// implemention of the class

FakeEthResource::FakeEthResource()
{
    yTrace();

    ipv4addr = 0;
    eo_common_ipv4addr_to_string(ipv4addr, ipv4addrstring, sizeof(ipv4addrstring));
    ethboardtype = eobrd_ethtype_unknown;
    snprintf(boardTypeString, sizeof(boardTypeString), "unknown");

    ethManager                  = NULL;
    isInRunningMode             = false;
    objLock                     = new Semaphore(1);

    verifiedBoardPresence       = false;
    verifiedBoardTransceiver    = false;

    memset(verifiedEPprotocol, 0, sizeof(verifiedEPprotocol));

    usedNumberOfRegularROPs     = 0;
    memset(&boardCommStatus, 0, sizeof(boardCommStatus));

    myHostTrans = new HostTransceiver();
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
    ethManager = TheEthManager::instance();

    Bottle groupEthBoard  = Bottle(cfgtotal.findGroup("ETH_BOARD"));
    if(groupEthBoard.isNull())
    {
        yError() << "EthResource::open2() cannot find ETH_BOARD group in config files";
        return NULL;
    }
    Bottle groupEthBoardProps = Bottle(groupEthBoard.findGroup("ETH_BOARD_PROPERTIES"));
    if(groupEthBoardProps.isNull())
    {
        yError() << "EthResource::open2() cannot find ETH_BOARD_PROPERTIES group in config files";
        return NULL;
    }
    Bottle groupEthBoardSettings = Bottle(groupEthBoard.findGroup("ETH_BOARD_SETTINGS"));
    if(groupEthBoardSettings.isNull())
    {
        yError() << "EthResource::open2() cannot find ETH_BOARD_PROPERTIES group in config files";
        return NULL;
    }

    Bottle b_ETH_BOARD_PROPERTIES_Type = groupEthBoardProps.findGroup("Type");
    ConstString Type = b_ETH_BOARD_PROPERTIES_Type.get(1).asString();
    const char *strType = Type.c_str();
    // 1. compare with the exceptions which may be in some old xml files ("EMS4", "MC4PLUS", "MC2PLUS"), and then then call proper functions
    if(0 == strcmp(strType, "EMS4"))
    {
        ethboardtype = eobrd_ethtype_ems4;
    }
    else if(0 == strcmp(strType, "MC4PLUS"))
    {
        ethboardtype = eobrd_ethtype_mc4plus;
    }
    else if(0 == strcmp(strType, "MC2PLUS"))
    {
        ethboardtype = eobrd_ethtype_mc2plus;
    }
    else
    {
        eObrd_type_t brd = eobrd_unknown;
        if(eobrd_unknown == (brd = eoboards_string2type2(strType, eobool_true)))
        {
            brd = eoboards_string2type2(strType, eobool_false);
        }

        // if not found in compact or extended string format, we accept that the board is unknown

        ethboardtype = eoboards_type2ethtype(brd);
    }

    snprintf(boardTypeString, sizeof(boardTypeString), "%s", eoboards_type2string2(eoboards_ethtype2type(ethboardtype), eobool_true));

    Bottle paramNameBoard(groupEthBoardSettings.find("Name").asString());
    char xmlboardname[64] = {0};
    snprintf(xmlboardname, sizeof(xmlboardname), "%s", paramNameBoard.toString().c_str());

    lock(true);


    eOipv4addressing_t localIPv4 = ethManager->getLocalIPV4addressing();

    bool ret;
    uint8_t num = 0;
    eo_common_ipv4addr_to_decimal(remIP, NULL, NULL, NULL, &num);
    if(!myHostTrans->init2(groupEthBoard, localIPv4, remIP))
    {
        ret = false;
        char ipinfo[20] = {0};
        eo_common_ipv4addr_to_string(remIP, ipinfo, sizeof(ipinfo));
        yError() << "EthResource::open2() cannot init transceiver w/ HostTransceiver::init2() for BOARD" << xmlboardname << "IP" << ipinfo;
    }
    else
    {
        ret = true;
    }

    uint8_t ip1, ip2, ip3, ip4;
    eo_common_ipv4addr_to_decimal(remIP, &ip1, &ip2, &ip3, &ip4);
    ACE_UINT32 hostip = (ip1 << 24) | (ip2 << 16) | (ip3 << 8) | (ip4);
    ACE_INET_Addr myIP((u_short)localIPv4.port, hostip);
    remote_dev = myIP;
    ipv4addr = remIP;
    eo_common_ipv4addr_to_string(ipv4addr, ipv4addrstring, sizeof(ipv4addrstring));


    if(0 != strlen(xmlboardname))
    {
        snprintf(boardName, sizeof(boardName), "%s", xmlboardname);
    }
    else
    {
        snprintf(boardName, sizeof(boardName), "NOT-NAMED");
    }



    Bottle groupEthBoardActions = Bottle(groupEthBoard.findGroup("ETH_BOARD_ACTIONS"));
    if(!groupEthBoardActions.isNull())
    {

        Bottle groupEthBoardActions_Monitor = Bottle(groupEthBoardActions.findGroup("MONITOR_ITS_PRESENCE"));
        if(!groupEthBoardActions_Monitor.isNull())
        {

            Bottle groupEthBoardActions_Monitor_enabled = groupEthBoardActions_Monitor.findGroup("enabled");
            ConstString Ena = groupEthBoardActions_Monitor_enabled.get(1).asString();
            const char *strEna = Ena.c_str();

            double presenceTimeout;
            if(true == groupEthBoardActions_Monitor.check("timeout"))
            {
                presenceTimeout = groupEthBoardActions_Monitor.find("timeout").asDouble();

                if(presenceTimeout <= 0)
                {
                    presenceTimeout = 0;
                }

                if(presenceTimeout > 0.100)
                {
                    presenceTimeout = 0.100;
                }

            }


            if(true == groupEthBoardActions_Monitor.check("periodOfMissingReport"))
            {
                double reportMissingPeriod = groupEthBoardActions_Monitor.find("periodOfMissingReport").asDouble();

                if(reportMissingPeriod <= 0)
                {
                    reportMissingPeriod = 0.0;
                }

                if(reportMissingPeriod > 600)
                {
                    reportMissingPeriod = 600;
                }

                yDebug() << "CFG_PRINT" << boardName <<" Enabled monitor parasence = "<< strEna << "with period=" <<  presenceTimeout<< " and report period="<< reportMissingPeriod ; 
            }
        }
    }

    lock(false);

    return ret;
}



bool FakeEthResource::close()
{
    yTrace();
    return false;
}


bool FakeEthResource::getTXpacket(uint8_t **packet, uint16_t *size, uint16_t *numofrops)
{
    return false;
}



bool FakeEthResource::Tick()
{
    return true;
}


bool FakeEthResource::Check()
{
    return true;
}



bool FakeEthResource::canProcessRXpacket(uint64_t *data, uint16_t size)
{
    if(NULL == data)
        return false;

    if(size > myHostTrans->getCapacityOfRXpacket())
        return false;

    return true;
}


void FakeEthResource::processRXpacket(uint64_t *data, uint16_t size, bool collectStatistics)
{;}


ACE_INET_Addr FakeEthResource::getRemoteAddress()
{
    return remote_dev;
}

eOipv4addr_t FakeEthResource::getIPv4remoteAddress(void)
{
    return ipv4addr;
}

const char * FakeEthResource::getName(void)
{
    return boardName;
}

const char * FakeEthResource::getIPv4string(void)
{
    return ipv4addrstring;
}

eObrd_ethtype_t FakeEthResource::getBoardType(void)
{
    return ethboardtype;
}

const char * FakeEthResource::getBoardTypeString(void)
{
    return boardTypeString;
}

void FakeEthResource::getBoardInfo(eOdate_t &date, eOversion_t &version)
{
    date = {0};
    version = {0};
}



bool FakeEthResource::isEPsupported(eOprot_endpoint_t ep)
{
    return myHostTrans->isSupported(ep);
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
        yError() << "FakeEthResource::verifyEPprotocol() cannot verify BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
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


bool FakeEthResource::readBufferedValue(eOprotID32_t id32,  uint8_t *data, uint16_t* size)
{
    *size=0;
    true;
}

bool FakeEthResource::addSetMessage(eOprotID32_t id32, uint8_t* data)
{
    return true;
}

bool FakeEthResource::addGetMessage(eOprotID32_t id32)
{
    return true;
}

bool FakeEthResource::addGetMessage(eOprotID32_t id32, std::uint32_t signature)
{
    return true;
}

bool FakeEthResource::addSetMessageAndCacheLocally(eOprotID32_t id32, uint8_t* data)
{
    return true;
}

bool FakeEthResource::readSentValue(eOprotID32_t id32, uint8_t *data, uint16_t* size)
{
    *size=0;
    return true;
}

EOnv* FakeEthResource::getNVhandler(eOprotID32_t id32, EOnv* nv)
{
    return myHostTrans->getnvhandler(id32, nv);
}

bool FakeEthResource::isFake()
{
    return true;
}

bool FakeEthResource::isID32supported(eOprotID32_t id32)
{
    return true;
}



// eof

