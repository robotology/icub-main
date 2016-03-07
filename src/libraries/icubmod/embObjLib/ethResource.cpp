// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

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
#include "can_string_eth.h"

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;



#define ETHRES_CHECK_MN_APPL_STATUS


// - class EthNetworkQuery

EthNetworkQuery::EthNetworkQuery()
{
    id2wait     = eo_prot_ID32dummy;
    netwait     = new Semaphore(0);
    isbusy      = new Semaphore(1);
    iswaiting   = new Semaphore(0);
}

EthNetworkQuery::~EthNetworkQuery()
{
    delete netwait;
    delete isbusy;
    delete iswaiting;
}

Semaphore* EthNetworkQuery::start(eOprotID32_t id32, uint32_t signature)
{
    // i wait until the busy semaphore is released ... in this way no other thread is able to manage the true semaphore
    isbusy->wait();
    // i also enable someone else (e.g., the network callback functions to retrieve networkQuerySem and increment it
    iswaiting->post();

    id2wait = id32;

    return(netwait);
}

bool EthNetworkQuery::wait(Semaphore *sem, double timeout)
{
    if(NULL == sem)
    {
        return(false);
    }
    return(sem->waitWithTimeout(timeout));
}

bool EthNetworkQuery::arrived(eOprotID32_t id32, uint32_t signature)
{
    Semaphore* sem = NULL;
    signature = signature;

    if((true == iswaiting->check()) && (id2wait == id32))
    {   // i give the semaphore to the function which will unblock only if someone is really waiting that id32
        sem = netwait;
    }

    if(NULL == sem)
    {
        return(false);
    }

    sem->post();

    return(true);
}

bool EthNetworkQuery::stop(Semaphore *sem)
{
    sem = sem;
    // make sure that the control semaphores have zero value. i use check() because if value is already zero it does not harm.
    iswaiting->check();
    netwait->check();

    id2wait = eo_prot_ID32dummy;

    // release the network query semaphore for another call
    isbusy->post();
    return(true);
}



// - class EthResource

EthResource::EthResource()
{
    yTrace();

    ipv4addr = 0;
    eo_common_ipv4addr_to_string(ipv4addr, ipv4addrstring, sizeof(ipv4addrstring));

    ethManager                  = NULL;
    lastRecvMsgTimestamp        = -1.0;
    isInRunningMode             = false;
    infoPkts                    = new InfoOfRecvPkts();
    objLock                     = new Semaphore(1);

    verifiedBoardPresence       = false;

    verifiedBoardTransceiver    = false;
    txrateISset                 = false;
    cleanedBoardBehaviour       = false;

    memset(verifiedEPprotocol, 0, sizeof(verifiedEPprotocol));

    usedNumberOfRegularROPs     = 0;
    memset(&boardCommStatus, 0, sizeof(boardCommStatus));


    ethQuery = new EthNetworkQuery();

    ethQueryServices = new EthNetworkQuery();


    for(int i = 0; i<16; i++)
        c_string_handler[i]     = NULL;

    ConstString tmp = NetworkBase::getEnvironment("ETH_VERBOSEWHENOK");
    if (tmp != "")
    {
        verbosewhenok = (bool)NetType::toInt(tmp);
    }
    else
    {
        verbosewhenok = false;
    }
}

EthResource::~EthResource()
{
    ethManager          = NULL;

    // marco.accame on 11sept14: in here we must surely deinit/delete what we have created/initted in teh constructor and in open() or init()

    delete infoPkts;
    delete objLock;

    delete ethQuery;
    delete ethQueryServices;

    //Delete every can_string_eth object eventually initialized
    for(int i=0; i<16; i++)
    {
        if (c_string_handler[i] != NULL)
        {
            delete c_string_handler[i];
            c_string_handler[i] = NULL;
        }
    }
}

bool EthResource::lock(bool on)
{
    if(true == on)
        objLock->wait();
    else
        objLock->post();

    return true;
}


bool EthResource::open2(eOipv4addr_t remIP, yarp::os::Searchable &cfgtotal)
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

// -- i dont use this code as long as i retrieve the remote ip address from the remIP argument .... however i may remove this argument and use the following code
//    Bottle paramIPboard(groupEth.find("IpAddress").asString());
//    char str[64] = {0};
//    strcpy(str, paramIPboard.toString().c_str());
//    int ip1, ip2, ip3, ip4;
//    sscanf(str, "\"%d.%d.%d.%d", &ip1, &ip2, &ip3, &ip4);
//    eOipv4addr_t ipv4addr = eo_common_ipv4addr(ip1, ip2, ip3, ip4);
    Bottle paramNameBoard(groupEthBoardSettings.find("Name").asString());
    char xmlboardname[64] = {0};
    strcpy(xmlboardname, paramNameBoard.toString().c_str());

    lock(true);

    if(cfgtotal.findGroup("GENERAL").find("verbose").asBool())
    {
        infoPkts->_verbose = true;
    }
    else
    {
        infoPkts->_verbose = false;
    }

    eOipv4addressing_t localIPv4 = ethManager->getLocalIPV4addressing();

    bool ret;
    uint8_t num = 0;
    eo_common_ipv4addr_to_decimal(remIP, NULL, NULL, NULL, &num);
    if(!HostTransceiver::init2(groupEthBoard, localIPv4, remIP))
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


    infoPkts->setBoardIP(remIP);


    if(0 != strlen(xmlboardname))
    {
        snprintf(boardName, sizeof(boardName), "%s", xmlboardname);
    }
    else
    {
        snprintf(boardName, sizeof(boardName), "NOT-NAMED");
    }

    lock(false);

    return ret;
}



//bool EthResource::open(yarp::os::Searchable &cfgtotal, yarp::os::Searchable &cfgtransceiver, yarp::os::Searchable &cfgprotocol, ethFeature_t &request)
//{
//    // Get the pointer to the actual Singleton ethManager
//    ethManager = TheEthManager::instance();

//    lock(true);

//    if(cfgtotal.findGroup("GENERAL").find("verbose").asBool())
//    {
//        infoPkts->_verbose = true;
//    }
//    else
//    {
//        infoPkts->_verbose = false;
//    }


//    bool ret;
//    eOipv4addr_t eo_locIp = eo_common_ipv4addr(request.pc104IPaddr.ip1, request.pc104IPaddr.ip2, request.pc104IPaddr.ip3, request.pc104IPaddr.ip4);
//    eOipv4addr_t eo_remIp = eo_common_ipv4addr(request.boardIPaddr.ip1, request.boardIPaddr.ip2, request.boardIPaddr.ip3, request.boardIPaddr.ip4);
//    const uint16_t packetRXcapacity = EthResource::maxRXpacketsize; // for safety i use the maximum size ... however, i could read the xml file and set this number equal to max tx size of teh ems ...
//    if(!HostTransceiver::init(cfgtransceiver, cfgprotocol, eo_locIp, eo_remIp, request.boardIPaddr.port, packetRXcapacity, request.boardNumber))
//    {
//        ret = false;
//        yError() << "cannot init transceiver... maybe wrong board number... check log and config file.";
//    }
//    else
//    {
//        ret = true;
//    }

//    ACE_UINT32 hostip = (request.boardIPaddr.ip1 << 24) | (request.boardIPaddr.ip2 << 16) | (request.boardIPaddr.ip3 << 8) | (request.boardIPaddr.ip4);
//    ACE_INET_Addr myIP((u_short)request.boardIPaddr.port, hostip);
//    remote_dev = myIP;
//    ipv4addr = eo_common_ipv4addr(request.boardIPaddr.ip1, request.boardIPaddr.ip2, request.boardIPaddr.ip3, request.boardIPaddr.ip4);
//    eo_common_ipv4addr_to_string(ipv4addr, ipv4addrstring, sizeof(ipv4addrstring));


//    infoPkts->setBoardIP(eo_remIp);

//    boardName[0] = '\0';
//    if(0 != strlen(request.boardName))
//    {
//        memset(boardName, 0, sizeof(boardName));
//        snprintf(boardName, sizeof(boardName), "%s", request.boardName);
//    }

//    lock(false);

//    return ret;
//}

bool EthResource::close()
{
    yTrace();
    return false;
}


bool EthResource::getTXpacket(uint8_t **packet, uint16_t *size, uint16_t *numofrops)
{
    return HostTransceiver::getTransmit(packet, size, numofrops);
}


int EthResource::getRXpacketCapacity()
{
    return HostTransceiver::getCapacityOfRXpacket();
}


bool EthResource::printRXstatistics(void)
{
    infoPkts->forceReport();
    return true;
}


void EthResource::checkIsAlive(double curr_time)
{
    if((infoPkts->isInError) || (!infoPkts->initted))
    {
        return;
    }

    if((curr_time - infoPkts->last_recvPktTime) > infoPkts->timeout)
    {

        yError() << "EthResource::checkIsAlive() detected that BOARD " << ipv4addrstring << " @ time" << int(floor(curr_time)) << "secs," << curr_time - floor(curr_time) << "has: more than " << infoPkts->timeout *1000 << "ms without any news. LAST =" << (curr_time - infoPkts->last_recvPktTime) << "sec ago";
        infoPkts->isInError =true;
        infoPkts->printStatistics();
        infoPkts->clearStatistics();
    }

}

bool EthResource::canProcessRXpacket(uint64_t *data, uint16_t size)
{
    if(NULL == data)
        return false;

    if(size > HostTransceiver::getCapacityOfRXpacket())
        return false;

    return true;
}

void EthResource::processRXpacket(uint64_t *data, uint16_t size, bool collectStatistics)
{
    bool collect = collectStatistics && isInRunningMode;

    double curr_timeBeforeParsing = 0;

    if(true == collect)
    {
        curr_timeBeforeParsing = yarp::os::Time::now();
    }

    HostTransceiver::onMsgReception(data, size);

    if(true == collect)
    {
        double curr_timeAfterParsing = yarp::os::Time::now();
        infoPkts->updateAndCheck(data, size, curr_timeBeforeParsing, (curr_timeAfterParsing-curr_timeBeforeParsing), false);
    }
}

ACE_INET_Addr EthResource::getRemoteAddress()
{
    return remote_dev;
}

eOipv4addr_t EthResource::getIPv4remoteAddress(void)
{
    return ipv4addr;
}

const char * EthResource::getName(void)
{
    return boardName;
}

const char * EthResource::getIPv4string(void)
{
    return ipv4addrstring;
}

//int EthResource::getNumberOfAttachedInterfaces(void)
//{
//    ethManager->ethBoards->number_of_interfaces(this);
//}

bool EthResource::isEPsupported(eOprot_endpoint_t ep)
{
    return HostTransceiver::isSupported(ep);
}


//bool EthResource::goToConfig(void)
//{
//    // stop the control loop (if running) and force the board to enter in config mode
//    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_appl, 0, eoprot_tag_mn_appl_cmmnds_go2state);

//    eOenum08_t command_go2state = applstate_config;
//    if(!addSetMessage(protid, (uint8_t*) &command_go2state))
//    {
//        yError() << "for var goToConfig";
//        return false;
//    }


//#if !defined(ETHRES_CHECK_MN_APPL_STATUS)

//    isInRunningMode = false;
//    return true;

//#else

//    // this delay is required because we want to force the two rops (go2state comamnd and ask<state>) to be in different udp packets
//    //Time::delay(0.010);
//    // however, if we do the loop over maxAttempts then we dont need it anymore. if the board can execute the order
//    // then at most there will be two iterations of the for() loop.


//    yDebug("EthResource::goToConfig() is called for BOARD %s (%s)", ipv4addrstring, boardName);


//    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_appl, 0, eoprot_tag_mn_appl_status);
//    eOmn_appl_status_t status = {0};
//    uint16_t size = 0;
//    const char* statestr[] = {"applstate_config", "applstate_running", "applstate_error", "applstate_unknown"};

//    const int maxAttempts = 5;
//    bool verified = false;
//    bool found = false;

//    char foundmessage[512] = "nothing found";
//    const char *foundstatestr = statestr[3];

//    for(int i=0; i<maxAttempts; i++)
//    {
//        // getRemoteValue() sends a request and blocks until the value arrives from the board.
//        // if the value does not arrive within a timeout it asks again and again
//        if(true == getRemoteValue(id32, &status, size))
//        {
//            found = true;

//            int index = (status.currstate > 2) ? (3) : (status.currstate);

//            foundstatestr = statestr[index];

//            snprintf(foundmessage, sizeof(foundmessage), "EthResource::goToConfig() detected BOARD %s (%s). It has a binary with: name = %s, ver = %d.%d, build date = %d.%d.%d at %d:%d. Its application is now in state %s",
//                                            ipv4addrstring,
//                                            boardName,
//                                            status.name,
//                                            status.version.major,
//                                            status.version.minor,
//                                            status.buildate.day,
//                                            status.buildate.month,
//                                            status.buildate.year,
//                                            status.buildate.hour,
//                                            status.buildate.min,
//                                            foundstatestr);


//            if(applstate_config == status.currstate)
//            {
//                yInfo("%s", foundmessage);
//                yDebug("EthResource::goToConfig() successfully sent BOARD %s (%s) in cfg mode", ipv4addrstring, boardName);

//                verified = true;
//                isInRunningMode = false;
//                break;
//            }
//            else
//            {
//                // we have received a reply from the board but: if i is equal to 0, the board may not have processed the order yet (it takes 1 ms = 1 cycle of its control-loop).
//                // or ... the remote board have processed the order but it cannot execute it.
//                // we keep on asking some more times because if we dont verify then we must quit robotInterface
//            }
//        }
//        else
//        {
//            // in here if we havent received any reply from the remote board. it is unlikely but possible. maybe the board crashed.
//            // ok, there is also teh possibility that id32 is wrong (unlikely) or &status is NULL (unlikely).
//            yError("EthResource::goToConfig() called getRemoteValue() for BOARD %s (%s) but there was no reply", ipv4addrstring, boardName);

//        }
//    }

//    if(false == found)
//    {
//        yError("EthResource::goToConfig() could not verify the status of BOARD %s (%s) because it could not find it after %d attempts", ipv4addrstring, boardName, maxAttempts);
//    }
//    else if(false == verified)
//    {
//        yInfo("%s", foundmessage);
//        yError("EthResource::goToConfig() could not send BOARD %s (%s) in cfg state. the board is instead in state %s", ipv4addrstring, boardName, foundstatestr);

//        isInRunningMode = (applstate_running == status.currstate) ? true : false; // we quit robotInterface ... it means nothing to set this value
//    }

//    return verified;

//#endif

//}


//bool EthResource::goToRun(eOprotEndpoint_t endpoint, eOprotEntity_t entity)
//{

//#if defined(ETHRES_DEBUG_DONTREADBACK)
//    yWarning() << "EthResource::goToRun() is in ETHRES_DEBUG_DONTREADBACK mode";
//    // execute but force verify to false
//    return true;
//#endif

//    // start the control loop by sending a proper message to the board
//    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_appl, 0, eoprot_tag_mn_appl_cmmnds_go2state);


//    eOenum08_t command_go2state = applstate_running;
//    if(!addSetMessage(protid, (uint8_t*) &command_go2state))
//    {
//        yError("EthResource::goToRun() in BOARD %s (%s) fails to add a command go2state running to transceiver", ipv4addrstring, boardName);
//        return false;
//    }

//#if !defined(ETHRES_CHECK_MN_APPL_STATUS)

//    isInRunningMode = true;
//    return true;

//#else

//    // this delay is required because we want to force the two rops (go2state comamnd and ask<state>) to be in different udp packets
//    //Time::delay(0.010);
//    // however, if we do the loop over maxAttempts then we dont need it anymore. if the board can execute the order
//    // then at most there will be two iterations of the for() loop.



//    yDebug("EthResource::goToRun() is called for BOARD %s (%s) for the entity %s", ipv4addrstring, boardName, eoprot_EN2string(endpoint, entity));

//    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_appl, 0, eoprot_tag_mn_appl_status);
//    eOmn_appl_status_t status = {0};
//    uint16_t size = 0;
//    const char* statestr[] = {"applstate_config", "applstate_running", "applstate_error", "applstate_unknown"};

//    const int maxAttempts = 5;
//    bool verified = false;
//    bool found = false;

//    char foundmessage[512] = "nothing found";
//    const char *foundstatestr = statestr[3];

//    for(int i=0; i<maxAttempts; i++)
//    {
//        // getRemoteValue() sends a request and blocks until the value arrives from the board.
//        // if the value does not arrive within a timeout it asks again and again
//        if(true == getRemoteValue(id32, &status, size))
//        {
//            found = true;

//            int index = (status.currstate > 2) ? (3) : (status.currstate);

//            foundstatestr = statestr[index];

//            snprintf(foundmessage, sizeof(foundmessage), "EthResource::goToRun() detected BOARD %s (%s). It has a binary with: name = %s, ver = %d.%d, build date = %d.%d.%d at %d:%d. Its application is now in state %s",
//                                            ipv4addrstring,
//                                            boardName,
//                                            status.name,
//                                            status.version.major,
//                                            status.version.minor,
//                                            status.buildate.day,
//                                            status.buildate.month,
//                                            status.buildate.year,
//                                            status.buildate.hour,
//                                            status.buildate.min,
//                                            foundstatestr);


//            if(applstate_running == status.currstate)
//            {
//                yInfo("%s", foundmessage);
//                yDebug("EthResource::goToRun() successfully sent BOARD %s (%s) in run mode", ipv4addrstring, boardName);

//                verified = true;
//                isInRunningMode = true;
//                break;
//            }
//            else
//            {
//                // we have received a reply from the board but: if i is equal to 0, the board may not have processed the order yet (it must decode the whole udp packet containing the order before it can change state).
//                // or ... the remote board have processed the order but it cannot execute it. this second situation may happen if the board cannot find the required can boards attached to it.
//                // in any case, we ask some more times because if we dont set verified equal to true, then we must quit robotInterface
//            }
//        }
//        else
//        {
//            // in here if we havent received any reply from the remote board. it is unlikely but possible. maybe the board crashed.
//            // ok, there is also teh possibility that id32 is wrong (unlikely) or &status is NULL (unlikely).
//            yError("EthResource::goToRun() called getRemoteValue() for BOARD %s (%s) but there was no reply", ipv4addrstring, boardName);
//        }
//    }


//    if(false == found)
//    {
//        yError("EthResource::goToRun() could not verify the status of BOARD %s (%s) because it could not find it after %d attempts", ipv4addrstring, boardName, maxAttempts);
//    }
//    else if(false == verified)
//    {
//        yInfo("%s", foundmessage);
//        yError("EthResource::goToRun() could not send BOARD %s (%s) in run state. the board is instead in state %s", ipv4addrstring, boardName, foundstatestr);

//        if(applstate_config == status.currstate)
//        {
//            yError() << "It may be that the BOARD is not ready yet to enter in run mode: PLEASE WAIT A FEW SECONDS AND RELAUNCH robotInterface";
//        }
//        isInRunningMode = (applstate_running == status.currstate) ? true : false; // we quit robotInterface ... it means nothing to set this value
//    }

//    return verified;

//#endif

//}


//double  EthResource::getLastRecvMsgTimestamp(void)
//{
//    return(infoPkts->last_recvPktTime);
//}


//bool EthResource::clearRegulars(bool verify)
//{
//#if defined(ETHRES_DEBUG_DONTREADBACK)
//    yWarning() << "EthResource::clearRegulars() is in ETHRES_DEBUG_DONTREADBACK mode";
//    // execute but force verify to false
//    verify = false;
//#endif

//    uint16_t numberofregulars = 0;
//    if(true == verify)
//    {
//        if(false == numberofRegulars(numberofregulars))
//        {
//            yError() << "ethResource::clearRegulars() fails at asking the number of regulars";
//            return false;
//        }
//        else
//        {
//            if(verbosewhenok)
//            {
//                yDebug() << "EthResource::clearRegulars() has detected" << numberofregulars << "regulars in BOARD" << getName() << "with IP" << getIPv4string() << "now is attempting clearing them";
//            }
//        }
//    }


//    // we send a command which clears the regular rops

//    eOmn_cmd_config_t cmdconfig     = {0};
//    eOprotID32_t IDcmdconfig        = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_command_config);

//    cmdconfig.opcpar.opc            = eomn_opc_config_REGROPs_clear;
//    cmdconfig.opcpar.plustime       = 0;
//    cmdconfig.opcpar.plussign       = 0;
//    cmdconfig.opcpar.dummy01        = 0;
//    cmdconfig.opcpar.signature      = eo_rop_SIGNATUREdummy;


//    // send set command

//    if(!addSetMessage(IDcmdconfig, (uint8_t*) &cmdconfig))
//    {
//        yError() << "EthResource::clearRegulars(): call of addSetMessage() has failed";
//        return false;
//    }


//    // set number of used rops and size to zero.
//    usedNumberOfRegularROPs     = 0;

//    if(true == verify)
//    {
//        Time::delay(0.010); // waiting some time before command is surely executed

//        // must read back the remote board to verify if the regulars have been successfully loaded

//        if(false == numberofRegulars(numberofregulars))
//        {
//            yError() << "ethResource::clearRegulars() fails at asking the number of regulars";
//            return false;
//        }

//        if(usedNumberOfRegularROPs != numberofregulars)
//        {
//            yError() << "ethResource::clearRegulars() detects something wrong in regulars: (expected, number in board) =" << usedNumberOfRegularROPs << numberofregulars;
//            return false;
//        }

//        if(verbosewhenok)
//        {
//            yDebug() << "EthResource::clearRegulars() has correctly checked regulars: expected = " << usedNumberOfRegularROPs << " and number in board = " << numberofregulars;
//        }
//    }


//    return true;
//}


bool EthResource::isRunning(void)
{
    return(isInRunningMode);
}

#if 0

Semaphore* EthResource::startNetworkQuerySession(eOprotID32_t id32, uint32_t signature)
{
#if 0
    return(networkQuerySem);
#else
    // i wait until the busy semaphore is released ... in this way no other thread is able to manage the networkQuerySem
    isbusyNQsem->wait();
    // i also enable someone else (e.g., the network callback functions to retrieve networkQuerySem and increment it
    iswaitingNQsem->post();

    return(networkQuerySem);
#endif
}


bool EthResource::waitForNetworkQueryReply(Semaphore* sem, double timeout)
{
    return(sem->waitWithTimeout(timeout));
}

bool EthResource::stopNetworkQuerySession(Semaphore* sem)
{
#if 0
    return(true);
#else
    // make sure that the control semaphores have zero value. i use check() because if value is already zero it does not harm.
    iswaitingNQsem->check();
    networkQuerySem->check();

    // release the network query semaphore for another call
    isbusyNQsem->post();
    return(true);
#endif
}

#endif

bool EthResource::aNetQueryReplyHasArrived(eOprotID32_t id32, uint32_t signature)
{
    if(id32 == eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_service, 0, eoprot_tag_mn_service_status_commandresult))
    {
        return(ethQueryServices->arrived(id32, signature));
    }
    else
    {
        return(ethQuery->arrived(id32, signature));
    }
}



bool EthResource::verifyBoardTransceiver()
{

#if defined(ETHRES_DEBUG_DONTREADBACK)
    yWarning() << "EthResource::verifyBoardTransceiver() is in ETHRES_DEBUG_DONTREADBACK mode";
    verifiedBoardTransceiver = true;
    return true;
#endif

    if(verifiedBoardTransceiver)
    {
        return(true);
    }

    // step 1: we ask the remote board the eoprot_tag_mn_comm_status variable and then we verify vs transceiver properties and .. mn protocol version

    uint32_t signature = 0xaa000000;
    const eoprot_version_t * pc104versionMN = eoprot_version_of_endpoint_get(eoprot_endpoint_management);
    const double timeout = 0.100;   // now the timeout can be reduced because the board is already connected.

    eOprotID32_t id2send = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_status);
    eOprotID32_t id2wait = id2send;
    eOmn_comm_status_t brdstatus = {0};
    uint16_t size = 0;
    // the semaphore used for waiting for replies from the board
    yarp::os::Semaphore* sem = ethQuery->start(id2wait, signature);


    // send ask message
    if(false == addGetMessageWithSignature(id2send, signature))
    {
        yError() << "EthResource::verifyBoardTransceiver() cannot transmit a request about the communication status to BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
        return(false);
    }

    // wait for a say message arriving from the board. the eoprot_fun_UPDT_mn_comm_status() function shall release the waiting semaphore
    if(false == ethQuery->wait(sem, timeout))
    {
        // must release the semaphore
        ethQuery->stop(sem);
        yError() << "  FATAL: EthResource::verifyEPprotocol() had a timeout of" << timeout << "secs when asking the comm status to BOARD" << getName() << "with IP" << getIPv4string() <<  ": cannot proceed any further";
        yError() << "         EthResource::verifyEPprotocol() asks: can you ping the board? if so, is the MN protocol version of BOARD equal to (" << pc104versionMN->major << pc104versionMN->minor << ")? if not, perform FW upgrade. if so, was the ropframe transmitted in time?";
        return(false);
    }

    // must release the semaphore
    ethQuery->stop(sem);

    // get the reply
    if(false == readBufferedValue(id2wait, (uint8_t*)&brdstatus, &size))
    {
        yError() << "EthResource::verifyBoardTransceiver() cannot read the comm status of BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
        return(false);
    }

    // save it in a variable of the class for future use
    memcpy(&boardCommStatus, &brdstatus, sizeof(boardCommStatus));


    //#warning --> marco.accame: inside EthResource::verifyBoardTransceiver() in the future you shall use variable mnprotocolversion
    // now i must verify that there is the same mn protocol version
    //const eoprot_version_t * brdversionMN = pc104versionMN; // at the moment we cannot get it from remote board
    eoprot_version_t * brdversionMN = (eoprot_version_t*)&brdstatus.managementprotocolversion;

    if(pc104versionMN->major != brdversionMN->major)
    {
        yError() << "EthResource::verifyBoardTransceiver() detected different mn protocol major versions: local =" << pc104versionMN->major << ", remote =" << brdversionMN->major << ": cannot proceed any further. FW upgrade is required";
        return(false);
    }


    if(pc104versionMN->minor != brdversionMN->minor)
    {
        yError() << "EthResource::verifyBoardTransceiver() detected different mn protocol minor versions: local =" << pc104versionMN->minor << ", remote =" << brdversionMN->minor << ": cannot proceed any further. FW upgrade is required";
        return(false);
        //yWarning() << "EthResource::verifyBoardTransceiver() detected different mn protocol minor versions: local =" << pc104versionMN->minor << ", remote =" << brdversionMN->minor << ": FW upgrade is advised";
    }

#if 0

    // now i must check brdstatus.transceiver vs HostTransceiver::localTransceiverProperties

    if(localTransceiverProperties.listeningPort != brdstatus.transceiver.destinationPort)
    {
        // marco.accame: if i am in here, it means that i received the message back ... it means that the ports are correct. however i keep this control.
        yError() << "EthResource::verifyBoardTransceiver() detected different ports: local listening =" << localTransceiverProperties.listeningPort << ", remote destination=" << brdstatus.transceiver.destinationPort << ": cannot proceed any further";
        return(false);
    }

    if(localTransceiverProperties.destinationPort != brdstatus.transceiver.listeningPort)
    {
        // marco.accame: if i am in here, it means that i received the message back ... it means that the ports are correct. however i keep this control.
        yError() << "EthResource::verifyBoardTransceiver() detected different ports: local destination =" << localTransceiverProperties.destinationPort << ", remote listening=" << brdstatus.transceiver.listeningPort << ": cannot proceed any further";
        return(false);
    }

    if(localTransceiverProperties.maxsizeRXpacket < brdstatus.transceiver.maxsizeTXpacket)
    {
        yError() << "EthResource::verifyBoardTransceiver() detected that max size of rx packet is too small: max size local rx =" << localTransceiverProperties.maxsizeRXpacket << ", max size remote tx=" << brdstatus.transceiver.maxsizeTXpacket << ": cannot proceed any further";
        return(false);
    }
    else
 //   {
 //       yDebug() << "EthResource::verifyBoardTransceiver() detected that local max size of rx packet = " << localTransceiverProperties.maxsizeRXpacket << "can accept board tx packet of max size = " << brdstatus.transceiver.maxsizeTXpacket;
 //   }

    if(localTransceiverProperties.maxsizeTXpacket > brdstatus.transceiver.maxsizeRXpacket)
    {
        yError() << "EthResource::verifyBoardTransceiver() detected that max size of tx packet is too big for the board: max size local tx =" << localTransceiverProperties.maxsizeTXpacket << ", max size remote rx=" << brdstatus.transceiver.maxsizeRXpacket << ": verify";
        return(false);
    }
    else
 //   {
 //       yDebug() << "EthResource::verifyBoardTransceiver() detected that local max size of tx packet = " << localTransceiverProperties.maxsizeTXpacket << "can be accepted by remote board with max rx size = " << brdstatus.transceiver.maxsizeRXpacket;
 //   }

    if(remoteTransceiverProperties.maxsizeROPframeRegulars != brdstatus.transceiver.maxsizeROPframeRegulars)
    {
        yWarning() << "EthResource::verifyBoardTransceiver() detected different maxsizeROPframeRegulars: from xml =" << remoteTransceiverProperties.maxsizeROPframeRegulars << ", board=" << brdstatus.transceiver.maxsizeROPframeRegulars << ": correct xml file";
        //return(false); // it is not a fatal error because for this value we use what we have received from the board
    }

    if(remoteTransceiverProperties.maxsizeROPframeReplies != brdstatus.transceiver.maxsizeROPframeReplies)
    {
        yWarning() << "EthResource::verifyBoardTransceiver() detected different maxsizeROPframeReplies: from xml =" << remoteTransceiverProperties.maxsizeROPframeReplies << ", board=" << brdstatus.transceiver.maxsizeROPframeReplies << ": correct xml file";
        //return(false); // it is not a fatal error because we dont use this value
    }


    if(remoteTransceiverProperties.maxsizeROPframeOccasionals != brdstatus.transceiver.maxsizeROPframeOccasionals)
    {
        yWarning() << "EthResource::verifyBoardTransceiver() detected different maxsizeROPframeOccasionals: from xml =" << remoteTransceiverProperties.maxsizeROPframeOccasionals << ", board=" << brdstatus.transceiver.maxsizeROPframeOccasionals << ": correct xml file";
        //return(false); // it is not a fatal error because we dont use this value
    }


    if(localTransceiverProperties.maxsizeROP != brdstatus.transceiver.maxsizeROP )
    {
        yError() << "EthResource::verifyBoardTransceiver() detected different maxsizeROP: local =" << localTransceiverProperties.maxsizeROP << ", remote=" << brdstatus.transceiver.maxsizeROP << ": cannot proceed any further";
        return(false);
    }


    if(remoteTransceiverProperties.maxnumberRegularROPs != brdstatus.transceiver.maxnumberRegularROPs)
    {
        yWarning() << "EthResource::verifyBoardTransceiver() detected different maxnumberRegularROPs: from xml =" << remoteTransceiverProperties.maxnumberRegularROPs << ", board=" << brdstatus.transceiver.maxnumberRegularROPs << ": correct xml file";
        //return(false); // it is not a fatal error because for this value we use what we have received from the board
    }



    // step 2: we ask the number of endpoints in the board. it is useful to verify that their number is not too high
    eOmn_command_t command = {0};
    // prepare message to send. we send a set<> with request of number of endpoints
    id2send = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_command_querynumof);
    memset(&command, 0, sizeof(command));
    command.cmd.opc                             = eomn_opc_query_numof_EPs;
    command.cmd.querynumof.opcpar.opc           = eomn_opc_query_numof_EPs;
    command.cmd.querynumof.opcpar.endpoint      = eoprot_endpoint_all;

    id2wait = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_command_replynumof);;
    sem = ethQuery->start(id2wait, 0);
    // send set message
    if(false == addSetMessage(id2send, (uint8_t*)&command))
    {
        yError() << "EthResource::verifyBoardTransceiver() cannot transmit a request about the number of owned endpoints to BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
        return(false);
    }

    // wait for a sig message arriving from the board. the eoprot_fun_UPDT_mn_comm_cmmnds_command_replynumof() function shall release the waiting semaphore
    if(false == ethQuery->wait(sem, timeout))
    {
        // must release the semaphore
        ethQuery->stop(sem);
        yError() << "  FATAL: EthResource::verifyBoardTransceiver() had a timeout of" << timeout << "secs when asking the number of endpoints to BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
        return(false);
    }

    // must release the semaphore
    ethQuery->stop(sem);

    // get the data of variable containing the reply about the number of endpoints
    memset(&command, 0, sizeof(command));

    if(false == readBufferedValue(id2wait, (uint8_t*)&command, &size))
    {
        yError() << "  FATAL: EthResource::verifyBoardTransceiver() cannot read the number of endpoints of BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
        return(false);
    }

    boardEPsNumber = command.cmd.replynumof.opcpar.numberof;

//    yDebug() << "EthResource::verifyBoardTransceiver() detected" << boardEPsNumber << "endpoints in BOARD" << getName() << "with IP" << getIPv4string();


#endif

    if(verbosewhenok)
    {
        yDebug() << "EthResource::verifyBoardTransceiver() has validated the transceiver of BOARD" << getName() << "with IP" << getIPv4string();
    }

    verifiedBoardTransceiver = true;


    return(true);
}


bool EthResource::setTXrate()
{

#if defined(ETHRES_DEBUG_DONTREADBACK)
    yWarning() << "EthResource::setTXrate() is in ETHRES_DEBUG_DONTREADBACK mode";
    txrateISset = true;
    return true;
#endif

    if(txrateISset)
    {
        return(true);
    }

    // step 1: we send teh remote board a message of type eoprot_tag_mn_appl_config_txratedivider with the value read from the proper section
    //         if doen find the section we set it with value 1

    // call a set until verified

    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_appl, 0, eoprot_tag_mn_appl_config_txratedivider);
    uint8_t txratediv = HostTransceiver::TXrateOfRegulars;
    if(false == setRemoteValueUntilVerified(id32, &txratediv, sizeof(txratediv), 5, 0.010, 0.050, 2))
    {
        yWarning() << "EthResource::setTXrate() could not configure txrate divider at" << txratediv << "in BOARD" << getName() << "with IP" << getIPv4string();
        return true;
    }
    else
    {
        yDebug() << "EthResource::setTXrate() has succesfully set the TX rate of the transceiver of BOARD" << getName() << "with IP" << getIPv4string() << "at" << HostTransceiver::TXrateOfRegulars << "ms";
    }


    if(verbosewhenok)
    {
        yDebug() << "EthResource::setTXrate() has succesfully set the TX rate of the transceiver of BOARD" << getName() << "with IP" << getIPv4string() << "at" << HostTransceiver::TXrateOfRegulars << "ms";
    }

    txrateISset = true;


    return(true);
}


bool EthResource::cleanBoardBehaviour(void)
{
    if(cleanedBoardBehaviour)
    {
        return(true);
    }

    // send a ...
    if(false == serviceStop(eomn_serv_category_all))
    {
        yError() << "EthResource::cleanBoardBehaviour() cannot stop services for BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
        return(false);
    }


    if(verbosewhenok)
    {
        yDebug() << "EthResource::cleanBoardBehaviour() has cleaned the application in BOARD" << getName() << "with IP" << getIPv4string() << ": config mode + cleared all its regulars";
    }

    cleanedBoardBehaviour = true;

    return(true);

}


bool EthResource::verifyEPprotocol(eOprot_endpoint_t ep)
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
        yError() << "EthResource::verifyEPprotocol() cannot verify BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
        return(false);
    }


#if defined(ETHRES_DEBUG_DONTREADBACK)
    verifiedEPprotocol[ep] =  true;
    yWarning() << "EthResource::verifyEPprotocol() is in ETHRES_DEBUG_DONTREADBACK mode";
    return true;
#endif

    // 1. send a set<eoprot_tag_mn_comm_cmmnds_command_queryarray> and wait for the arrival of a sig<eoprot_tag_mn_comm_cmmnds_command_replyarray>
    //    the opc to send is eomn_opc_query_array_EPdes which will trigger a opc in reception eomn_opc_reply_array_EPdes
    // 2. the resulting array will contains a eoprot_endpoint_descriptor_t item for the specifeid ep with the protocol version of the ems.



    const int capacityOfArrayOfEPDES = (EOMANAGEMENT_COMMAND_DATA_SIZE - sizeof(eOarray_head_t)) / sizeof(eoprot_endpoint_descriptor_t);
    const double timeout = 0.100;

    eOprotID32_t id2send = eo_prot_ID32dummy;
    eOprotID32_t id2wait = eo_prot_ID32dummy;
    eOmn_command_t command = {0};
    uint16_t size = 0;


    // the semaphore used for waiting for replies from the board
    yarp::os::Semaphore* sem = NULL;


//    if(boardEPsNumber > capacityOfArrayOfEPDES)
//    {   // to support more than capacityOfArrayOfEPDES (= 16 on date of jul 22 2014) endpoints: just send two (or more) eoprot_tag_mn_comm_cmmnds_command_queryarray messages with setnumbers 0 and 1 (or more)
//        yError() << "EthResource::verifyEPprotocol() detected that BOARD" << getName() << "with IP" << getIPv4string() << "has" << boardEPsNumber << "endpoints and at most" << capacityOfArrayOfEPDES << "are supported: cannot proceed any further (review the code to support them all)";
//        return(false);
//    }

    // step 1: ask all the EP descriptors. from them we can extract protocol version of MN and of the target ep
    id2send = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_command_queryarray);
    memset(&command, 0, sizeof(command));
    command.cmd.opc                             = eomn_opc_query_array_EPdes;
    command.cmd.queryarray.opcpar.opc           = eomn_opc_query_array_EPdes;
    command.cmd.queryarray.opcpar.endpoint      = eoprot_endpoint_all;
    command.cmd.queryarray.opcpar.setnumber     = 0;
    command.cmd.queryarray.opcpar.setsize       = 0;

    // the semaphore must be retrieved using the id of the variable which is waited. in this case, it is the array of descriptors
    id2wait = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_command_replyarray);
    sem = ethQuery->start(id2wait, 0);

    if(false == addSetMessage(id2send, (uint8_t*)&command))
    {
        yError() << "EthResource::verifyEPprotocol() cannot transmit a request about the endpoint descriptors to BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
        return(false);
    }


    if(false == ethQuery->wait(sem, timeout))
    {
        // must release the semaphore
        ethQuery->stop(sem);
        yError() << "  FATAL: EthResource::verifyEPprotocol() had a timeout of" << timeout << "secs when asking the endpoint descriptors to BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
        return(false);
    }
    // must release the semaphore
    ethQuery->stop(sem);

    // now i get the array of descriptors
    memset(&command, 0, sizeof(command));
    if(false == readBufferedValue(id2wait, (uint8_t*)&command, &size))
    {
        yError() << "  FATAL: EthResource::verifyEPprotocol() cannot retrieve the endpoint descriptors of BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
        return(false);
    }

    // the array is ...
    eOmn_cmd_replyarray_t* cmdreplyarray = (eOmn_cmd_replyarray_t*)&command.cmd.replyarray;
    EOarray* array = (EOarray*)cmdreplyarray->array;

    //yDebug() << "EthResource::verifyEPprotocol() -> xxx-debug TO BE REMOVED AFTER DEBUG: head.capacity, itemsize, size" << array->head.capacity << array->head.itemsize << array->head.size;

    uint8_t sizeofarray = eo_array_Size(array);

//    if(sizeofarray != boardEPsNumber)
//    {
//        yWarning() << "EthResource::verifyEPprotocol() retrieved from BOARD" << ethManager->getName(ipv4addr) << ":" << sizeofarray << "endpoint descriptors, and there are" << boardEPsNumber << "endpoints";
//    }


    for(int i=0; i<sizeofarray; i++)
    {
        eoprot_endpoint_descriptor_t *epd = (eoprot_endpoint_descriptor_t*)eo_array_At(array, i);

        if(epd->endpoint == eoprot_endpoint_management)
        {
            const eoprot_version_t * pc104versionMN = eoprot_version_of_endpoint_get(eoprot_endpoint_management);
            if(pc104versionMN->major != epd->version.major)
            {
                yError() << "EthResource::verifyEPprotocol() for ep =" << eoprot_EP2string(epd->endpoint) << "detected: pc104.version.major =" << pc104versionMN->major << "and board.version.major =" << epd->version.major;
                yError() << "EthResource::verifyEPprotocol() detected mismatching protocol version.major in BOARD" << getName() << "with IP" << getIPv4string() << "for eoprot_endpoint_management: cannot proceed any further. FW upgrade is required";
                return(false);
            }
            if(pc104versionMN->minor != epd->version.minor)
            {
                yWarning() << "EthResource::verifyEPprotocol() for ep =" << eoprot_EP2string(epd->endpoint) << "detected: pc104.version.minor =" << pc104versionMN->minor << "and board.version.minor =" << epd->version.minor;
                yWarning() << "EthResource::verifyEPprotocol() detected mismatching protocol version.minor BOARD" << getName() << "with IP" << getIPv4string() << "for eoprot_endpoint_management: FW upgrade is advised";
            }
        }
        if(epd->endpoint == ep)
        {
            const eoprot_version_t * pc104versionEP = eoprot_version_of_endpoint_get(ep);
            if(pc104versionEP->major != epd->version.major)
            {
                yError() << "EthResource::verifyEPprotocol() for ep =" << eoprot_EP2string(epd->endpoint) << "detected: pc104.version.major =" << pc104versionEP->major << "and board.version.major =" << epd->version.major;
                yError() << "EthResource::verifyEPprotocol() detected mismatching protocol version.major in BOARD" << getName() << "with IP" << getIPv4string() << " for" << eoprot_EP2string(ep) << ": cannot proceed any further. FW upgrade is required";
                return(false);
            }
            if(pc104versionEP->minor != epd->version.minor)
            {
                yError() << "EthResource::verifyEPprotocol() for ep =" << eoprot_EP2string(epd->endpoint) << "detected: pc104.version.minor =" << pc104versionEP->minor << "and board.version.minor =" << epd->version.minor;
                yError() << "EthResource::verifyEPprotocol() detected mismatching protocol version.minor in BOARD" << getName() << "with IP" << getIPv4string() << " for" << eoprot_EP2string(ep) << ": FW upgrade is required";
                return(false);
            }
        }
    }

    verifiedEPprotocol[ep] = true;

    return(true);

}



bool EthResource::verifyBoard(void)
{
    if((true == verifyBoardPresence()) &&
       (true == verifyBoardTransceiver()) &&
//       (true == setRemoteBoardNumber()) &&
       (true == setTXrate()) &&
       (true == cleanBoardBehaviour()) )
    {
        return(true);
    }

    return(false);
}

//bool EthResource::setRemoteBoardNumber(void)
//{
//    remoteBoardNumberIsSet = true;
//    return(remoteBoardNumberIsSet);
//}

bool EthResource::verifyBoardPresence(void)
{

#if defined(ETHRES_DEBUG_DONTREADBACK)
    yWarning() << "EthResource::verifyBoardPresence() is in ETHRES_DEBUG_DONTREADBACK mode";
    verifiedBoardPresence =  true;
    return true;
#endif

    if(verifiedBoardPresence)
    {
        return(true);
    }

    // we ask the remote board a variable which is surely supported. best thing to do is asking the mn-protocol-version.
    // however, at 03 sept 2014 there is not a single variable to contain this, thus ... ask the eoprot_tag_mn_comm_status variable.

    //#warning --> marco.accame: inside EthResource::verifyBoardPresence() in the future you shall ask eoprot_tag_mn_comm_status_mnprotocolversion instead of eoprot_tag_mn_comm_status

    const double timeout = 0.500;   // 500 ms is more than enough if board is present. if link is not on it is a good time to wait
    const int retries = 20;         // the number of retries depends on the above timeout and on link-up time of the EMS.

    uint32_t signature = 0xaa000000;
    eOprotID32_t id2send = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_status);
    eOprotID32_t id2wait = id2send;
    eOmn_comm_status_t brdstatus = {0};
    uint16_t size = 0;
    // the semaphore used for waiting for replies from the board
    yarp::os::Semaphore* sem = ethQuery->start(id2wait, signature);

    bool pinged = false;
    int i; // kept in here because i want to see it also outside of the loop

    double start_time = yarp::os::Time::now();

    for(i=0; i<retries; i++)
    {
        // attempt the request until either a reply arrives or the max retries are reached

        // send ask message
        if(false == addGetMessageWithSignature(id2send, signature))
        {
            yWarning() << "EthResource::verifyBoardPresence() cannot transmit a request about the communication status to BOARD" << getName() << "with IP" << getIPv4string();
        }

        // wait for a say message arriving from the board. the eoprot_fun_UPDT_mn_xxx() function shall release the waiting semaphore
        if(false == ethQuery->wait(sem, timeout))
        {
            //yWarning() << "EthResource::verifyBoardPresence() had a timeout of" << timeout << "secs when asking a variable to BOARD" << getName() << "with IP" << getIPv4string()1;
            //yError() << "EthResource::verifyBoardPresence() asks: can you ping the board?";
        }
        else
        {
            // get the reply
            if(false == readBufferedValue(id2wait, (uint8_t*)&brdstatus, &size))
            {
                yWarning() << "EthResource::verifyBoardPresence() received a reply from BOARD" << getName() << "with IP" << getIPv4string() << "but cannot read it";
            }
            else
            {
                // ok: i have a reply: i just done read it ...
                pinged = true;
                // stop attempts
                break;
            }
        }

        if(!pinged)
        {
            yWarning() << "EthResource::verifyBoardPresence() cannot reach BOARD" << getName() << "with IP" << getIPv4string() << "at attempt #" << i+1 << "w/ timeout of" << timeout << "seconds";
        }

    }

    // must release the semaphore
    ethQuery->stop(sem);

    double end_time = yarp::os::Time::now();
    if(pinged)
    {
        verifiedBoardPresence = true;
        if(verbosewhenok)
        {
            yDebug() << "EthResource::verifyBoardPresence() found BOARD" << getName() << "with IP" << getIPv4string() << " at attempt #" << i+1 << "after" << end_time-start_time << "seconds";
        }
    }
    else
    {
        yError() << "EthResource::verifyBoardPresence() DID NOT have replies from BOARD" << getName() << "with IP" << getIPv4string() << " even after " << i << " attempts and" << end_time-start_time << "seconds: CANNOT PROCEED ANY FURTHER";
    }


    return(verifiedBoardPresence);

}


//bool EthResource::configureENDPOINT(yarp::os::Searchable &protconfig, eOprot_endpoint_t ep)
//{

//    configuredEP[ep] = true;
//    return(true);
//}


//bool EthResource::verifyENTITYnumber(yarp::os::Searchable &protconfig, eOprot_endpoint_t ep, eOprotEntity_t en, int expectednumber)
//{
//    // so that, it can work with branch runtime_ems_config and its future merge into master
//    return(true);

//#if defined(ETHRES_DEBUG_DONTREADBACK)
//    yWarning() << "EthResource::verifyENTITYnumber() is in ETHRES_DEBUG_DONTREADBACK mode";
//    return true;
//#endif

//    if(false == verifyEPprotocol(protconfig, ep))
//    {
//        yError() << "EthResource::verifyENTITYnumber() cannot even verify protocol in BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
//        return(false);
//    }



//    eOprotBRD_t protbrd = get_protBRDnumber();
//    uint8_t numofentities_prot = eoprot_entity_numberof_get(protbrd, ep, en);
//    uint8_t entities_in_endpoint = eoprot_entities_in_endpoint_numberof_get(protbrd, ep);


//    // at first we compare the two local numbers

//    if(-1 != expectednumber)
//    {
//        if(expectednumber != numofentities_prot)
//        {
//            yError() << "EthResource::verifyENTITYnumber() has detected a mismatching number of " << eoprot_EN2string(ep, en) << " between the XML files: cannot proceed any further";
//            yError() << " protocol uses =" << numofentities_prot << "but calling device uses" << expectednumber;
//        }
//    }

//    // and now we ask the number to the remote board


//    // we ask the array of eoprot_entity_descriptor_t inside that given endpoint


//    // 1. send a set<eoprot_tag_mn_comm_cmmnds_command_queryarray> and wait for the arrival of a sig<eoprot_tag_mn_comm_cmmnds_command_replyarray>
//    //    the opc to send is eomn_opc_query_array_EPdes which will trigger a opc in reception eomn_opc_reply_array_EPdes
//    // 2. the resulting array will contains a eoprot_endpoint_descriptor_t item for the specifeid ep with the protocol version of the ems.



//    const int capacityOfArrayOfENDES = (EOMANAGEMENT_COMMAND_DATA_SIZE - sizeof(eOarray_head_t)) / sizeof(eoprot_entity_descriptor_t);
//    const double timeout = 0.100;

//    eOprotID32_t id2send = eo_prot_ID32dummy;
//    eOprotID32_t id2wait = eo_prot_ID32dummy;
//    eOmn_command_t command = {0};
//    uint16_t size = 0;



//    // the semaphore used for waiting for replies from the board
//    yarp::os::Semaphore* sem = NULL;


//    if(entities_in_endpoint > capacityOfArrayOfENDES)
//    {   // to support more than capacityOfArrayOfENDES (= 16 on date of jul 22 2014) endpoints: just send two (or more) eoprot_tag_mn_comm_cmmnds_command_queryarray messages with setnumbers 0 and 1 (or more)
//        yError() << "EthResource::verifyENTITYnumber() detected that BOARD" << getName() << "with IP" << getIPv4string() << "has" << numofentities_prot << "entities in" <<  eoprot_EP2string(ep) << "and at most" << capacityOfArrayOfENDES << "are supported: cannot proceed any further (review the code to support them all)";
//        return(false);
//    }


//    // step 1: ask all the EN descriptors. from them we can extract the number of target en
//    id2send = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_command_queryarray);
//    memset(&command, 0, sizeof(command));
//    command.cmd.opc                             = eomn_opc_query_array_ENdes;
//    command.cmd.queryarray.opcpar.opc           = eomn_opc_query_array_ENdes;
//    command.cmd.queryarray.opcpar.endpoint      = ep;
//    command.cmd.queryarray.opcpar.setnumber     = 0;
//    command.cmd.queryarray.opcpar.setsize       = 0;

//    id2wait = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_command_replyarray);
//    sem = ethQuery->start(id2wait, 0);

//    if(false == addSetMessage(id2send, (uint8_t*)&command))
//    {
//        yError() << "EthResource::verifyENTITYnumber() cannot transmit a request about the entity descriptors to BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
//        return(false);
//    }


//    if(false == ethQuery->wait(sem, timeout))
//    {
//        // must release the semaphore
//        ethQuery->stop(sem);
//        yError() << "EthResource::verifyENTITYnumber() had a timeout of" << timeout << "secs when asking the entity descriptors to BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
//        return(false);
//    }
//    // must release the semaphore
//    ethQuery->stop(sem);

//    // now i get the array of descriptors
//    memset(&command, 0, sizeof(command));
//    if(false == readBufferedValue(id2wait, (uint8_t*)&command, &size))
//    {
//        yError() << "EthResource::verifyENTITYnumber() cannot retrieve the entity descriptors of BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
//        return(false);
//    }

//    // the array is ...
//    eOmn_cmd_replyarray_t* cmdreplyarray = (eOmn_cmd_replyarray_t*)&command.cmd.replyarray;
//    EOarray* array = (EOarray*)cmdreplyarray->array;

//    //yDebug() << "EthResource::verifyENTITYnumber() -> xxx-debug TO BE REMOVED AFTER DEBUG: head.capacity, itemsize, size" << array->head.capacity << array->head.itemsize << array->head.size;

//    uint8_t sizeofarray = eo_array_Size(array);

//    if(sizeofarray != entities_in_endpoint)
//    {
//        yWarning() << "EthResource::verifyEPprotocol() retrieved from BOARD" << getName() << "with IP" << getIPv4string() << ":" << sizeofarray << "entity descriptors, BUT there are" << entities_in_endpoint << "in local protocol";
//    }


//    uint8_t numofentities_brd = 255;

//    for(int i=0; i<sizeofarray; i++)
//    {
//        eoprot_entity_descriptor_t *end = (eoprot_entity_descriptor_t*)eo_array_At(array, i);

//        if((end->endpoint == ep) && (end->entity == en))
//        {
//            numofentities_brd = end->multiplicity;
//            break;
//        }

//    }

//    if(255 == numofentities_brd)
//    {
//        yError() << "EthResource::verifyEPprotocol() could not retrieve from BOARD" << getName() << "with IP" << getIPv4string() << "the multiplicity of entity" << eoprot_EN2string(ep, en) << ": cannot proceed anymore";
//        return false;
//    }

//    if(numofentities_brd != numofentities_prot)
//    {
//        yError() << "  FATAL: EthResource::verifyENTITYnumber() has detected a mismatching number of " << eoprot_EN2string(ep, en) << " between the PC104 and remote BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
//        yError() << "         pc014 uses =" << numofentities_prot << "but remote board has" << numofentities_brd;
//        return false;
//    }

//    // yDebug() << "EthResource::verifyENTITYnumber(): PC104 uses =" << numofentities_prot << " entities and remote board has" << numofentities_brd;

//    return(true);
//}



//bool EthResource::addRegulars(vector<eOprotID32_t> &id32vector, bool verify)
//{

//#if defined(ETHRES_DEBUG_DONTREADBACK)
//    yWarning() << "EthResource::addRegulars() is in ETHRES_DEBUG_DONTREADBACK mode";
//    verify = false;
//    return true; // uncomment to avoid sending command
//#endif

//    const double delaybetweentransmissions = 0.010; // 10 ms

//    eOmn_cmd_config_t cmdconfig     = {0};
//    eOropSIGcfg_t sigcfg            = {0};
//    eOprotID32_t IDcmdconfig        = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_command_config);
//    uint16_t targetcapacity         = (sizeof(cmdconfig.array)-sizeof(eOarray_head_t)) / sizeof(eOropSIGcfg_t);
//    EOarray *array                  = eo_array_New((uint8_t)targetcapacity, sizeof(eOropSIGcfg_t), cmdconfig.array); // reduction to uint8_t is ok

//    cmdconfig.opcpar.opc            = eomn_opc_config_REGROPs_append;
//    cmdconfig.opcpar.plustime       = 0;
//    cmdconfig.opcpar.plussign       = 0;
//    cmdconfig.opcpar.dummy01        = 0;
//    cmdconfig.opcpar.signature      = eo_rop_SIGNATUREdummy;

//    eOropctrl_t ropctrl = {0};
//    memcpy(&ropctrl, &eok_ropctrl_basic, sizeof(ropctrl));
//    ropctrl.plustime    = cmdconfig.opcpar.plustime;
//    ropctrl.plussign    = cmdconfig.opcpar.plussign;


//    // now i must load each id32item extracted by id32vector<> into the array. then, when the array is full we send the rop
//    // it is worth verifying, however ... how many regulars there are available and how many we can still add.
//    // moreover, it is worth verifying if the ropframe of regulars inside the board can host the new rops.

//    // at date of 04 sept 14 the value of targetcapacity is 16, thus we also must pay attention because for some boards we can
//    // pass an id32vector with bigger size. we solve this by sending the set<cmdconfig> message also inside the for() loop
//    // as soon as the array is full.

//    int id32vectorsize = id32vector.size();

//    // first check: verify if the number of extra rops is compatible with remote board
//    if((usedNumberOfRegularROPs+id32vectorsize) > boardCommStatus.transceiver.maxnumberRegularROPs)
//    {
//        yError() << "ethResource::addRegulars() cannot load the requested regular ROPs because they would be too many: (max, sofar, rqst) = " << boardCommStatus.transceiver.maxnumberRegularROPs << usedNumberOfRegularROPs << id32vectorsize;
//        return false;
//    }
//    /*
//    **  we dont do it anymore because it is enough to read back the number of regular rops from the board
//    int extrasize = 0;
//    // second check: about the size:
//    for(int i=0; i<id32vectorsize; i++)
//    {
//        eOprotID32_t id32 = id32vector.at(i);
//        uint16_t ss = eoprot_variable_sizeof_get(get_protBRDnumber(), id32); // sizeof data associated to id32
//        uint16_t ropsize = eo_rop_compute_size(ropctrl ,eo_ropcode_sig, ss);
//        //yDebug() << "size = " << ss << "ropsize = " << ropsize;
//        extrasize += ropsize;
//    }
//    //yDebug() << "(ropframemax, ropframenow, extrabytes) = " << boardCommStatus.transceiver.maxsizeROPframeRegulars << usedSizeOfRegularROPframe << extrasize;

//    // second check: verify if the data occupied by extra rops is compatible with remote board
//    if((usedSizeOfRegularROPframe+extrasize) > boardCommStatus.transceiver.maxsizeROPframeRegulars)
//    {
//        yError() << "ethResource::addRegulars() cannot load the requested regular ROPs because they need too much space: (max, sofar, rqst) = " << boardCommStatus.transceiver.maxsizeROPframeRegulars << usedSizeOfRegularROPframe << extrasize;
//        return false;
//    }
//    **
//    */
//    // ok we can go on.

//    for(int i=0; i<id32vectorsize; i++)
//    {
//        sigcfg.id32 = id32vector.at(i);
//        if(eores_OK != eo_array_PushBack(array, &sigcfg))
//        {
//            // it may be that there is an error
//            yError() << "ethResource::addRegulars() fails at loading a sigcfg item into array";
//            return false;
//        }
//        else
//        {
//            if(targetcapacity == eo_array_Size(array)) // pity that we dont have a eo_array_Full() method
//            {
//                // the array is full: send it to board
//                // A ropsigcfg vector can hold at max NUMOFROPSIGCFG (21) value. If more are needed, send another packet,
//                // so wait some time to let ethManager send this packet and then start again.
//                if(false == addSetMessage(IDcmdconfig, (uint8_t *) &cmdconfig))
//                {
//                    yError() << "ethResource::addRegulars() fails at adding a set message";
//                    return false;
//                }
//                Time::delay(delaybetweentransmissions);         // wait here, the ethManager thread will take care of sending the loaded message
//                eo_array_Reset(array);      // reset so that the array is able to contain new items at the following iteration
//            }
//        }

//    }

//    // if there are items in the array we must send them now
//    if(0 != eo_array_Size(array))
//    {   // there are still ropsigcfg to send
//        if(false == addSetMessage(IDcmdconfig, (uint8_t *) &cmdconfig) )
//        {
//            yError() << "ethResource::addRegulars() fails at adding a set message";
//            return false;
//        }
//    }


//    // if everything is ok we increment values
//    usedNumberOfRegularROPs     += id32vectorsize;
//    //usedSizeOfRegularROPframe   += extrasize;


//    if(true == verify)
//    {
//        // must read back the remote board to verify if the regulars have been successfully loaded
//        uint16_t numberofregulars = 0;
//        if(false == numberofRegulars(numberofregulars))
//        {
//            yError() << "ethResource::addRegulars() fails at asking the number of regulars";
//            return false;
//        }

//        if(usedNumberOfRegularROPs != numberofregulars)
//        {
//            yError() << "ethResource::addRegulars() detects something wrong in regulars: (expected, number in board) =" << usedNumberOfRegularROPs << numberofregulars;
//            return false;
//        }

//        if(verbosewhenok)
//        {
//            yDebug() << "EthResource::addRegulars() has correctly checked regulars: expected = " << usedNumberOfRegularROPs << " and number in board = " << numberofregulars;
//        }
//    }

//    return(true);
//}


//bool EthResource::numberofRegulars(uint16_t &numberofregulars)
//{

//#if defined(ETHRES_DEBUG_DONTREADBACK)
//    yWarning() << "EthResource::numberofRegulars() is in ETHRES_DEBUG_DONTREADBACK mode and always gives back 0";
//    return true;
//#endif

//    const double timeout = 0.100; // 100 ms

//    uint16_t size = 0;
//    eOmn_command_t command = {0};
//    // prepare message to send. we send a set<> with request of number of endpoints
//    eOprotID32_t id2send = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_command_querynumof);
//    memset(&command, 0, sizeof(command));
//    command.cmd.opc                             = eomn_opc_query_numof_REGROPs;
//    command.cmd.querynumof.opcpar.opc           = eomn_opc_query_numof_REGROPs;
//    command.cmd.querynumof.opcpar.endpoint      = eoprot_endpoint_all;


//#if 0

//    // the semaphore used for waiting for replies from the board
//    eOprotID32_t id2wait = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_command_replynumof);
//    yarp::os::Semaphore* sem = ethQuery->start(id2wait, 0);

//    // send set message
//    if(false == addSetMessage(id2send, (uint8_t*)&command))
//    {
//        yError() << "EthResource::numberofRegulars() cannot transmit a request about the number of regulars to BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
//        return(false);
//    }

//    // wait for a sig message arriving from the board. the eoprot_fun_UPDT_mn_comm_cmmnds_command_replynumof() function shall release the waiting semaphore
//    if(false == ethQuery->wait(sem, timeout))
//    {
//        // must release the semaphore
//        ethQuery->stop(sem);
//        yError() << "EthResource::numberofRegulars() had a timeout of" << timeout << "secs when asking the number of regulars to BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
//        return(false);
//    }
//    ethQuery->stop(sem);

//    // get the data of variable containing the reply about the number of endpoints
//    memset(&command, 0, sizeof(command));

//    if(false == readBufferedValue(id2wait, (uint8_t*)&command, &size))
//    {
//        yError() << "EthResource::numberofRegulars() cannot read the number of regulars of BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
//        return(false);
//    }

//    numberofregulars = command.cmd.replynumof.opcpar.numberof;

//    return true;

//#else

//    const int retries = 10;
//    bool replied = false;
//    bool numberisreceived = false;
//    int i = 0; // must be in here because it counts the number of attempts
//    // the semaphore used for waiting for replies from the board
//    eOprotID32_t id2wait = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_command_replynumof);
//    yarp::os::Semaphore* sem = ethQuery->start(id2wait, 0);


//    double start_time = yarp::os::Time::now();

//    for(i=0; i<retries; i++)
//    {
//        // attempt the request until either a reply arrives or the max retries are reached

//        // send set message
//        if(false == addSetMessage(id2send, (uint8_t*)&command))
//        {
//            yWarning() << "EthResource::numberofRegulars() cannot transmit a request about the number of regulars to BOARD" << getName() << "with IP" << getIPv4string();
//            //yError() << "EthResource::numberofRegulars() cannot transmit a request about the number of regulars to BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
//            //return(false);
//        }

//        // wait for a sig message arriving from the board. the eoprot_fun_UPDT_mn_comm_cmmnds_command_replynumof() function shall release the waiting semaphore
//        if(false == ethQuery->wait(sem, timeout))
//        {
//            // must release the semaphore
//            //ethQuery->stop(sem);
//            //yError() << "EthResource::numberofRegulars() had a timeout of" << timeout << "secs when asking the number of regulars to BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
//            //return(false);
//        }
//        else
//        {
//            // get the data of variable containing the reply about the number of endpoints
//            memset(&command, 0, sizeof(command));

//            if(false == readBufferedValue(id2wait, (uint8_t*)&command, &size))
//            {
//                yWarning() << "EthResource::numberofRegulars() cannot read the number of regulars of BOARD" << getName() << "with IP" << getIPv4string();
//            }
//            else
//            {
//                // ok: i have a reply: i have just read it ...
//                replied = true;
//                // stop attempts
//                break;
//            }

//        }

//        if(!replied)
//        {
//            yWarning() << "EthResource::numberofRegulars() cannot have a reply from BOARD" << getName() << "with IP" << getIPv4string() << "at attempt #" << i+1 << "w/ timeout of" << timeout << "seconds";
//        }

//    }


//    // must release the semaphore
//    ethQuery->stop(sem);

//    double end_time = yarp::os::Time::now();

//    if(replied)
//    {
//        numberofregulars = command.cmd.replynumof.opcpar.numberof;
//        numberisreceived = true;
//        if(0 == i)
//        {
//            if(verbosewhenok)
//            {
//                yDebug() << "EthResource::numberofRegulars() retrieved value from BOARD" << getName() << "with IP" << getIPv4string() << " at attempt #" << i+1 << "after" << end_time-start_time << "seconds";;
//            }
//        }
//        else
//        {
//            yWarning() << "EthResource::numberofRegulars() retrieved value from BOARD" << getName() << "with IP" << getIPv4string() << " at attempt #" << i+1 << "after" << end_time-start_time << "seconds";;
//        }

//    }
//    else
//    {
//        yError() << "  FATAL: EthResource::numberofRegulars() DID NOT have replies from BOARD" << getName() << "with IP" << getIPv4string() << " even after " << i << " attempts and" << end_time-start_time << "seconds: CANNOT PROCEED ANY FURTHER";
//        numberisreceived = false;
//    }



//    return numberisreceived;


//#endif
//}

bool EthResource::setRemoteValueUntilVerified(eOprotID32_t id32, void *value, uint16_t size, int retries, double waitbeforeverification, double verificationtimeout, int verificationretries)
{
    int attempt = 0;
    bool done = false;

    int maxattempts = retries + 1;

    for(attempt=0; (attempt<maxattempts) && (false == done); attempt++)
    {

        if(!addSetMessage(id32, (uint8_t *) value))
        {
            yWarning() << "EthResource::setRemoteValueUntilVerified() had an error while calling addSetMessage() in BOARD" << getName() << "with IP" << getIPv4string() << "at attempt #" << attempt+1;
            continue;
        }

#if defined(ETHRES_DEBUG_DONTREADBACK)
        yWarning() << "EthResource::setRemoteValueUntilVerified() is in ETHRES_DEBUG_DONTREADBACK";
        return true;
#endif

        // ok, now i wait some time before asking the value back for verification
        Time::delay(waitbeforeverification);

        if(false == verifyRemoteValue(id32, (uint8_t *) value, size, verificationtimeout, verificationretries))
        {
            yWarning() << "EthResource::setRemoteValueUntilVerified() had an error while calling verifyRemoteValue() in BOARD" << getName() << "with IP" << getIPv4string() << "at attempt #" << attempt+1;
        }
        else
        {
            done = true;
        }

    }

    char nvinfo[128];
    eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));

    if(done)
    {
        if(attempt > 1)
        {
            yWarning() << "EthResource::setRemoteValueUntilVerified has set and verified ID" << nvinfo << "in BOARD" << getName() << "with IP" << getIPv4string() << "at attempt #" << attempt;
        }
        else
        {
            if(verbosewhenok)
            {
                yDebug() << "EthResource::setRemoteValueUntilVerified has set and verified ID" << nvinfo << "in BOARD" << getName() << "with IP" << getIPv4string() << "at attempt #" << attempt;
            }
        }
    }
    else
    {
        yError() << "FATAL: EthResource::setRemoteValueUntilVerified could not set and verify ID" << nvinfo << "in BOARD" << getName() << "with IP" << getIPv4string() << " even after " << attempt << "attempts";
    }


    return(done);
}



// must: 1. be sure we have the callback written and configured, 2. that the callback releases the sem (maybe if a suitable signature is found).
bool EthResource::verifyRemoteValue(eOprotID32_t id32, void *value, uint16_t size, double timeout, int retries)
{

#if defined(ETHRES_DEBUG_DONTREADBACK)
        yWarning() << "EthResource::verifyRemoteValue() is in ETHRES_DEBUG_DONTREADBACK mode, thus it does not verify";
        return true;
#endif


    char nvinfo[128];
    eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
    uint32_t signature = 0xaa000000;

//    this check is done inside the methods of hostTransceiver class
//    eOprotBRD_t brd = HostTransceiver::get_protBRDnumber();//    if(eobool_false == eoprot_id_isvalid(brd, id32))
//    {
//        yError() << "EthResource::verifyRemoteValue() detected an invalid" << nvinfo << "for BOARD" << getName() << "with IP" << getIPv4string();
//        return false;
//    }

    if((NULL == value) || (0 == size))
    {
        yError() << "EthResource::verifyRemoteValue() detected NULL value or zero size for " << nvinfo << "for BOARD" << getName() << "with IP" << getIPv4string();
        return false;
    }

    uint8_t *datainside = (uint8_t*)calloc(size, 1);
    uint16_t sizeinside = 0;



#if 0

    eOprotID32_t id2send = id32;
    eOprotID32_t id2wait = id32;
    const double timeout = 0.100;

    // the semaphore used for waiting for replies from the board
    yarp::os::Semaphore* sem = ethQuery->start(id2wait, signature);

    // send ask message
    if(false == addGetMessageWithSignature(id2send, signature))
    {
        free(datainside);
        ethQuery->stop(sem);
        yError() << "EthResource::verifyRemoteValue() cannot transmit a request about" << nvinfo << "to BOARD" << getName() << "with IP" << getIPv4string();
        return false;
    }

    // wait for a say message arriving from the board. the proper function shall release the waiting semaphore
    if(false == ethQuery->wait(sem, timeout))
    {
        free(datainside);
        ethQuery->stop(sem);
        yError() << "  FATAL: EthResource::verifyRemoteValue() had a timeout of" << timeout << "secs when asking value of" << nvinfo << "to BOARD" << getName() << "with IP" << getIPv4string();
        return false;
    }
    else
    {
        // get the reply
        if(false == readBufferedValue(id2wait, datainside, &sizeinside))
        {
            free(datainside);
            ethQuery->stop(sem);
            yError() << "EthResource::verifyRemoteValue() received a reply about" << nvinfo << "from BOARD" << getName() << "with IP" << getIPv4string() << "but cannot read it";
            return false;
        }
        else
        {
            ethQuery->stop(sem);
            // ok: i have a reply: compare it with a memcmp

            if(size != sizeinside)
            {
                free(datainside);
                yError() << "EthResource::verifyRemoteValue() has found different sizes for" << nvinfo <<"arg, inside =" << size << sizeinside;
                return false;
            }
            else if(0 != memcmp(datainside, value, size))
            {
                free(datainside);
                yError() << "EthResource::verifyRemoteValue() has found different values for" << nvinfo << "from BOARD" << getName() << "with IP" << getIPv4string();
                return false;
            }
            else
            {
                yDebug() << "EthResource::verifyRemoteValue() verified value inside" << nvinfo << "from BOARD" << getName() << "with IP" << getIPv4string();
            }
        }
    }


    free(datainside);

    return true;

#else


        eOprotID32_t id2send = id32;
        eOprotID32_t id2wait = id32;
//        const double timeout = 0.100;
//        const int retries = 10;
        bool replied = false;
        bool valueisverified = false;
        int i = 0; // must be in here because it count the number of attempts
        // the semaphore used for waiting for replies from the board
        yarp::os::Semaphore* sem = ethQuery->start(id2wait, signature);


        double start_time = yarp::os::Time::now();

        for(i=0; i<retries; i++)
        {
            // attempt the request until either a reply arrives or the max retries are reached


            // send ask message
            if(false == addGetMessageWithSignature(id2send, signature))
            {
                yWarning() << "EthResource::verifyRemoteValue() cannot transmit a request to BOARD" << getName() << "with IP" << getIPv4string();
            //    free(datainside);
            //    ethQuery->stop(sem);
            //    yError() << "EthResource::verifyRemoteValue() cannot transmit a request about" << nvinfo << "to BOARD" << getName() << "with IP" << getIPv4string();
            //    return false;
            }

            // wait for a say message arriving from the board. the proper function shall release the waiting semaphore
            if(false == ethQuery->wait(sem, timeout))
            {
            //    free(datainside);
            //    ethQuery->stop(sem);
            //    yError() << "  FATAL: EthResource::verifyRemoteValue() had a timeout of" << timeout << "secs when asking value of" << nvinfo << "to BOARD" << getName() << "with IP" << getIPv4string();
            //    return false;
            }
            else
            {
                // get the reply
                if(false == readBufferedValue(id2wait, datainside, &sizeinside))
                {
                //    free(datainside);
                //    ethQuery->stop(sem);
                //    yError() << "EthResource::verifyRemoteValue() received a reply about" << nvinfo << "from BOARD" << getName() << "with IP" << getIPv4string() << "but cannot read it";
                //    return false;
                    yWarning() << "EthResource::verifyRemoteValue() received a reply from BOARD" << getName() << "with IP" << getIPv4string() << "but cannot read it";
                }
                else
                {
                    // ok: i have a reply: i just done read it ...
                    replied = true;
                    // stop attempts
                    break;
                }

            }

            if(!replied)
            {
                yWarning() << "EthResource::verifyRemoteValue() cannot have a reply from BOARD" << getName() << "with IP" << getIPv4string() << "at attempt #" << i+1 << "w/ timeout of" << timeout << "seconds";
            }

        }


        // must release the semaphore
        ethQuery->stop(sem);

        double end_time = yarp::os::Time::now();

        if(replied)
        {
            // ok: i have a reply: compare it with a memcmp

            if(size != sizeinside)
            {
                yError() << "  FATAL: EthResource::verifyRemoteValue() has found different sizes for" << nvinfo <<"arg, inside =" << size << sizeinside;
                valueisverified = false;
            }
            else if(0 != memcmp(datainside, value, size))
            {
                yError() << "  FATAL: EthResource::verifyRemoteValue() has found different values for" << nvinfo << "from BOARD" << getName() << "with IP" << getIPv4string();
                valueisverified = false;
            }
            else
            {
                if(0 == i)
                {
                    if(verbosewhenok)
                    {
                        yDebug() << "EthResource::verifyRemoteValue() verified value inside" << nvinfo << "from BOARD" << getName() << "with IP" << getIPv4string() << " at attempt #" << i+1 << "after" << end_time-start_time << "seconds";;
                    }
                }
                else
                {
                    yWarning() << "EthResource::verifyRemoteValue() verified value inside" << nvinfo << "from BOARD" << getName() << "with IP" << getIPv4string() << " at attempt #" << i+1 << "after" << end_time-start_time << "seconds";;
                }
                valueisverified = true;
            }


        }
        else
        {
            yError() << "  FATAL: EthResource::verifyRemoteValue() DID NOT have replies from BOARD" << getName() << "with IP" << getIPv4string() << " even after " << i << " attempts and" << end_time-start_time << "seconds: CANNOT PROCEED ANY FURTHER";
            valueisverified = false;
        }


        // must release allocated buffer
        free(datainside);

        // return result
        return valueisverified;

#endif

}



bool EthResource::getRemoteValue(eOprotID32_t id32, void *value, uint16_t &size, double timeout, int retries)
{

#if defined(ETHRES_DEBUG_DONTREADBACK)
        yWarning() << "EthResource::getRemoteValue() is in ETHRES_DEBUG_DONTREADBACK mode, thus it does not verify";
        return true;
#endif

    bool myverbosewhenok = verbosewhenok;

    //myverbosewhenok = false;


    char nvinfo[128];
    eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
    uint32_t signature = 0xaa000000;

//    this check is done inside the methods of hostTransceiver class
//    eOprotBRD_t brd = HostTransceiver::get_protBRDnumber();
//    if(eobool_false == eoprot_id_isvalid(brd, id32))
//    {
//        yError() << "EthResource::getRemoteValue() detected an invalid" << nvinfo << "for BOARD" << getName() << "with IP" << getIPv4string();
//        return false;
//    }

    if((NULL == value))
    {
        yError() << "EthResource::getRemoteValue() detected NULL value or zero size for" << nvinfo << "for BOARD" << getName() << "with IP" << getIPv4string();
        return false;
    }



    eOprotID32_t id2send = id32;
    eOprotID32_t id2wait = id32;
    bool replied = false;
    int i = 0; // must be in here because it count the number of attempts
    // the semaphore used for waiting for replies from the board
    yarp::os::Semaphore* sem = ethQuery->start(id2wait, signature);


    double start_time = yarp::os::Time::now();

    for(i=0; i<retries; i++)
    {
        // attempt the request until either a reply arrives or the max retries are reached


        // send ask message
        if(false == addGetMessageWithSignature(id2send, signature))
        {
            yWarning() << "EthResource::getRemoteValue() cannot transmit a request to BOARD" << getName() << "with IP" << getIPv4string();
        }

        // wait for a say message arriving from the board. the proper function shall release the waiting semaphore
        if(false == ethQuery->wait(sem, timeout))
        {

        }
        else
        {
            uint16_t ss = 0;
            // get the reply
            if(false == readBufferedValue(id2wait, (uint8_t*)value, &ss))
            {
                yWarning() << "EthResource::getRemoteValue() received a reply from BOARD" << getName() << "with IP" << getIPv4string() << "but cannot read it";
            }
            else
            {
                // ok: i have a reply: i just done read it ...
                size = ss;
                replied = true;
                // stop attempts
                break;
            }

        }

        if(!replied)
        {
            yWarning() << "EthResource::getRemoteValue() cannot have a reply from BOARD" << getName() << "with IP" << getIPv4string() << "at attempt #" << i+1 << "w/ timeout of" << timeout << "seconds";
        }

    }


    // must release the semaphore
    ethQuery->stop(sem);

    double end_time = yarp::os::Time::now();


    if(replied)
    {
        // ok: i have a reply: compare it with a memcmp

        if(0 == i)
        {
            if(myverbosewhenok)
            {
                yDebug() << "EthResource::getRemoteValue() obtained value inside" << nvinfo << "from BOARD" << getName() << "with IP" << getIPv4string() << " at attempt #" << i+1 << "after" << end_time-start_time << "seconds";;
            }
        }
        else
        {
            yWarning() << "EthResource::getRemoteValue() obtained value inside" << nvinfo << "from BOARD" << getName() << "with IP" << getIPv4string() << " at attempt #" << i+1 << "after" << end_time-start_time << "seconds";;
        }

    }
    else
    {
        yError() << "  FATAL: EthResource::getRemoteValue() DID NOT have replies from BOARD" << getName() << "with IP" << getIPv4string() << " even after " << i << " attempts and" << end_time-start_time << "seconds: CANNOT PROCEED ANY FURTHER";
    }


    // return result
    return replied;

}

bool EthResource::CANPrintHandler(eOmn_info_basic_t *infobasic)
{
    char str[256];
    char canfullmessage[64];

    static const char * sourcestrings[] =
    {
        "LOCAL",
        "CAN1",
        "CAN2",
        "UNKNOWN"
    };
    int source                      = EOMN_INFO_PROPERTIES_FLAGS_get_source(infobasic->properties.flags);
    const char * str_source         = (source > eomn_info_source_can2) ? (sourcestrings[3]) : (sourcestrings[source]);
    uint16_t address                = EOMN_INFO_PROPERTIES_FLAGS_get_address(infobasic->properties.flags);
    uint8_t *p64 = (uint8_t*)&(infobasic->properties.par64);

    int msg_id = (p64[1]&0xF0) >> 4;

    uint32_t sec = infobasic->timestamp / 1000000;
    uint32_t msec = (infobasic->timestamp % 1000000) / 1000;
    uint32_t usec = infobasic->timestamp % 1000;

    const char *boardstr = boardName;


    // Validity check
    if(address > 15)
    {
        snprintf(canfullmessage,sizeof(canfullmessage),"Error while parsing the message: CAN address detected is out of allowed range");
        snprintf(str,sizeof(str), "from BOARD %s (%s), src %s, adr %d, time %ds %dm %du: CAN PRINT MESSAGE[id %d] -> %s",
                                    ipv4addrstring,
                                    boardstr,
                                    str_source,
                                    address,
                                    sec,
                                    msec,
                                    usec,
                                    msg_id,
                                    canfullmessage
                                    );
        feat_PrintError(str);
    }
    else
    {
        // Initialization needed?
        if (c_string_handler[address] == NULL)
            c_string_handler[address] = new can_string_eth();

        CanFrame can_msg;
        can_msg.setCanData(infobasic->properties.par64);
        can_msg.setId(msg_id);
        can_msg.setSize(infobasic->properties.par16);
        int ret = c_string_handler[address]->add_string(&can_msg);

        // String finished?
        if (ret != -1)
        {
            char* themsg = c_string_handler[address]->get_string(ret);
            memcpy(canfullmessage, themsg, sizeof(canfullmessage));
            canfullmessage[63] = 0;
            c_string_handler[address]->clear_string(ret);

            snprintf(str,sizeof(str), "from BOARD %s (%s), src %s, adr %d, time %ds %dm %du: CAN PRINT MESSAGE[id %d] -> %s",
                                        ipv4addrstring,
                                        boardstr,
                                        str_source,
                                        address,
                                        sec,
                                        msec,
                                        usec,
                                        msg_id,
                                        canfullmessage
                                        );
            feat_PrintInfo(str);
        }
    }
    return true;
}


bool EthResource::serviceCommand(eOmn_service_operation_t operation, eOmn_serv_category_t category, const eOmn_serv_parameter_t* param, double timeout, int times)
{
    eOprotID32_t id2send = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_service, 0, eoprot_tag_mn_service_cmmnds_command);
    eOprotID32_t id2wait = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_service, 0, eoprot_tag_mn_service_status_commandresult);;

    // get a sem, transmit a set<>, wait for a reply (add code in callback of status_commandresult), retrieve the result. return true or false

    // the semaphore used for waiting for replies from the board
    yarp::os::Semaphore* sem = NULL;


    eOmn_service_cmmnds_command_t command = {0};
    command.operation = operation;
    command.category = category;
    if(NULL != param)
    {
        memcpy(&command.parameter, param, sizeof(eOmn_serv_parameter_t));
    }
    else
    {
       memset(&command.parameter, 0, sizeof(eOmn_serv_parameter_t));
       if((eomn_serv_operation_regsig_load == operation) || ((eomn_serv_operation_regsig_clear == operation)))
       {    // we send an empty array
            eo_array_New(eOmn_serv_capacity_arrayof_id32, 4, &command.parameter.arrayofid32);
       }
       else
       {
            command.parameter.configuration.type = eomn_serv_NONE;
       }
    }


    sem = ethQueryServices->start(id2wait, 0);
    bool replied = false;

    for(int i=0; i<times; i++)
    {

        if(false == addSetMessage(id2send, (uint8_t*)&command))
        {
            ethQueryServices->stop(sem);
            yError() << "EthResource::serviceCommand() cannot transmit an activation request to BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
            return(false);
        }


        if(false == ethQueryServices->wait(sem, timeout))
        {
            yWarning() << "EthResource::serviceCommand() had a timeout of" << timeout << "secs when sending an activation request to BOARD" << getName() << "with IP" << getIPv4string();
            // must release the semaphore
            //ethQueryServices->stop(sem);
            //return(false);
        }
        else
        {
            replied = true;
            break;
        }
    }
    // must release the semaphore
    ethQueryServices->stop(sem);

    // now i get the answer
    eOmn_service_command_result_t result = {0};
    uint16_t size = 0;
    if(false == readBufferedValue(id2wait, (uint8_t*)&result, &size))
    {
        yError() << "EthResource::serviceCommand() cannot retrieve the result for BOARD" << getName() << "with IP" << getIPv4string() << ": cannot proceed any further";
        return(false);
    }


    return(result.latestcommandisok);
}


bool EthResource::serviceVerifyActivate(eOmn_serv_category_t category, const eOmn_serv_parameter_t* param, double timeout)
{
    return(serviceCommand(eomn_serv_operation_verifyactivate, category, param, timeout, 1));
}


bool EthResource::serviceSetRegulars(eOmn_serv_category_t category, vector<eOprotID32_t> &id32vector, double timeout)
{
    eOmn_serv_parameter_t param = {0};
    EOarray *array = eo_array_New(eOmn_serv_capacity_arrayof_id32, 4, &param.arrayofid32);
    for(int i=0; i<id32vector.size(); i++)
    {
        eOprotID32_t id32 = id32vector.at(i);
        eo_array_PushBack(array, &id32);
    }

    return(serviceCommand(eomn_serv_operation_regsig_load, category, &param, timeout, 1));
}



bool EthResource::serviceStart(eOmn_serv_category_t category, double timeout)
{
    bool ret = serviceCommand(eomn_serv_operation_start, category, NULL, timeout, 1);

    if(ret)
    {
        isInRunningMode = true;
    }

    return ret;
}


bool EthResource::serviceStop(eOmn_serv_category_t category, double timeout)
{
    bool ret = serviceCommand(eomn_serv_operation_stop, category, NULL, timeout, 3);

    #warning TODO: the result for command stop shall also tell if the the board is in running mode or not.
    return ret;
}



// - class InfoOfRecvPkts


InfoOfRecvPkts::InfoOfRecvPkts() : DEFAULT_TIMEOUT_STAT(0.010) // 100 ms expessed in sec
{
    ipv4 = 0;
    eo_common_ipv4addr_to_string(ipv4, ipv4string, sizeof(ipv4string));

    initted = false;
    isInError = false;
    count = 0;
    max_count = DEFAULT_MAX_COUNT_STAT;
    timeout = DEFAULT_TIMEOUT_STAT;
    _verbose = false;
    timeoflastreport = yarp::os::Time::now();
    reportperiod = 30.0;

    last_seqNum = 0;
    last_ageOfFrame = 0;
    last_recvPktTime = 0.0;

    receivedPackets = 0;
    totPktLost = 0;
    currPeriodPktLost = 0;
    stat_ageOfFrame = new StatExt();
    stat_periodPkt = new StatExt();
    stat_lostPktgap = new StatExt();
    stat_printstatPktgap = new StatExt();
    stat_precessPktTime = new StatExt();
    stat_pktSize = new StatExt();


    ConstString statistcs_count_max = NetworkBase::getEnvironment("ETHREC_STATISTICS_COUNT_RECPKT_MAX");
    if (statistcs_count_max!="")
    {
        max_count = NetType::toInt(statistcs_count_max);
    }

    ConstString statistcs_timeout = NetworkBase::getEnvironment("ETHREC_STATISTICS_TIMEOUT_MSEC");
    if (statistcs_timeout!="")
    {
        timeout = (double)(NetType::toInt(statistcs_timeout))/1000; // because timeout is in sec
    }

    justprinted = true;


    //yTrace() << "Initialized with timeout " << timeout << "sec and count_rec_pkt " << max_count;

}


InfoOfRecvPkts::~InfoOfRecvPkts()
{
    delete stat_ageOfFrame;
    delete stat_periodPkt;
    delete stat_lostPktgap;
    delete stat_printstatPktgap;
    delete stat_precessPktTime;
    delete stat_pktSize;
}

void InfoOfRecvPkts::setBoardIP(eOipv4addr_t ip)
{
    ipv4 = ip;
    eo_common_ipv4addr_to_string(ipv4, ipv4string, sizeof(ipv4string));
}

void InfoOfRecvPkts::printStatistics(void)
{
    yDebug() << "  (STATS-RX)-> BOARD " << ipv4string << ":" << receivedPackets << "ropframes have been received by EthReceiver() in this period";

    if(0 == receivedPackets)
    {
        yDebug() << "  (STATS-RX)-> BOARD " << ipv4string << "DID NOT SEND ROPFRAMES in this period\n";
    }
    else if(0 != currPeriodPktLost)
    {
        yDebug() << "  (STATS-RX)-> BOARD " << ipv4string << "has ropframe losses in this period:" << currPeriodPktLost << "---------------------";
    }
    else
    {
        yDebug() << "  (STATS-RX)-> BOARD " << ipv4string << " does not have ropframe losses in this period";
    }

    if(0 != receivedPackets)
    {
        yDebug() << "  (STATS-RX)-> BOARD " << ipv4string << " curr ropframe losses = " << currPeriodPktLost<< "   tot ropframe lost = " << totPktLost;
        yDebug() << "  (STATS-RX)-> BOARD " << ipv4string << " inter-ropframe gap (as written by remote board)[holes are discarded]: avg=" << stat_ageOfFrame->mean()<< "ms std=" << stat_ageOfFrame->deviation()<< "ms min=" << stat_ageOfFrame->getMin() << "ms max=" << stat_ageOfFrame->getMax()<< "ms on " << stat_ageOfFrame->count() << "values";
        yDebug() << "  (STATS-RX)-> BOARD " << ipv4string << " gap between processed ropframes [holes are discarded]: avg=" << stat_periodPkt->mean()*1000 << "ms std=" << stat_periodPkt->deviation()*1000 << "ms min=" << stat_periodPkt->getMin()*1000 << "ms max=" << stat_periodPkt->getMax()*1000 << "ms on " << stat_periodPkt->count() << "values";
        yDebug() << "  (STATS-RX)-> BOARD " << ipv4string << " duration of holes in rx ropframes: avg=" << stat_lostPktgap->mean()*1000 << "ms std=" << stat_lostPktgap->deviation()*1000 << "ms min=" << stat_lostPktgap->getMin()*1000 << "ms max=" << stat_lostPktgap->getMax()*1000 << "ms on " << stat_lostPktgap->count() << "values";
        yDebug() << "  (STATS-RX)-> BOARD " << ipv4string << " gap between two ropframe w/ a print stat in between: avg=" << stat_printstatPktgap->mean()*1000 << "ms std=" << stat_printstatPktgap->deviation()*1000 << "ms min=" << stat_printstatPktgap->getMin()*1000 << "ms max=" << stat_printstatPktgap->getMax()*1000 << "ms on " << stat_printstatPktgap->count() << "values";
        yDebug() << "  (STATS-RX)-> BOARD " << ipv4string << " ropframe process time: avg=" << stat_precessPktTime->mean()*1000 << "ms std=" << stat_precessPktTime->deviation()*1000 << "ms min=" << stat_precessPktTime->getMin()*1000 << "ms max=" << stat_precessPktTime->getMax()*1000 << "ms on " << stat_precessPktTime->count() << "values";
        yDebug() << "  (STATS-RX)-> BOARD " << ipv4string << " ropframe size time: avg=" << stat_pktSize->mean() << "bytes std=" << stat_pktSize->deviation() << "min=" << stat_pktSize->getMin() << "max=" << stat_pktSize->getMax() << " on " << stat_pktSize->count() << "values\n";
    }

}

void InfoOfRecvPkts::clearStatistics(void)
{
    stat_ageOfFrame->clear();
    stat_periodPkt->clear();
    stat_lostPktgap->clear();
    stat_printstatPktgap->clear();
    stat_precessPktTime->clear();
    stat_pktSize->clear();
    currPeriodPktLost = 0;
    // initted = false;
    count = 0;
    timeoflastreport = yarp::os::Time::now();
    receivedPackets = 0;

    justprinted = true;
}


uint64_t InfoOfRecvPkts::getSeqNum(uint64_t *packet, uint16_t size)
{
    return(eo_ropframedata_seqnum_Get((EOropframeData*)packet));
}


uint64_t InfoOfRecvPkts::getAgeOfFrame(uint64_t *packet, uint16_t size)
{
    return(eo_ropframedata_age_Get((EOropframeData*)packet));
}

void InfoOfRecvPkts::updateAndCheck(uint64_t *packet, uint16_t size, double reckPktTime, double processPktTime, bool evalreport)
{
    uint64_t curr_seqNum = getSeqNum(packet, size);;
    uint64_t curr_ageOfFrame = getAgeOfFrame(packet, size); // in usec
    double curr_periodPkt; // in usec: it is the delta time between two consecutive processed packets
    double diff_ageofframe_ms; // in ms
    long long diff;

    int num_lost_pkts = 0; // number of lost packets detected in this run of updateAndCheck

    double timenow = yarp::os::Time::now();
    bool local_verbose = _verbose;


    if(initted)
    {
        // (1) check sequence number

        if(curr_seqNum != last_seqNum+1)
        {

            if(curr_seqNum < (last_seqNum+1))
            {
                if(local_verbose)
                    yError()<< "REC PKTS not in order!!!!" << ipv4string << " seq num rec=" << curr_seqNum << " expected=" << last_seqNum+1 << "!!!!!!!" ;
            }
            else
            {
                //i lost some pkts
                num_lost_pkts = curr_seqNum - last_seqNum -1;
            }
            currPeriodPktLost += num_lost_pkts;
            totPktLost += num_lost_pkts;

            if(local_verbose)
                yError()<< "LOST "<< num_lost_pkts <<"  PKTS on board=" << ipv4string << " seq num rec="<< curr_seqNum << " expected=" << last_seqNum+1 << "!! curr pkt lost=" << currPeriodPktLost << "  Tot lost pkt=" << totPktLost;
        }


        // (2) check age of ropframe

        diff = (curr_ageOfFrame - last_ageOfFrame);
        diff_ageofframe_ms = (double)(diff) / 1000.0; // age of frame is expressed in msec but in floating point
        if( diff_ageofframe_ms > (timeout*1000))
        {
            if(local_verbose)
                yError() << "Board " << ipv4string << ": EMS time (ageOfFrame) between 2 pkts bigger then " << timeout * 1000 << "ms;\t Actual delay is" << diff_ageofframe_ms << "ms diff = "<< double(diff)/1000.0;
        }

        if(0 == num_lost_pkts)
        {   // i add the delta only if we had no packet loss. in this way i measure the exact statistics of emission time from the remote board.
            stat_ageOfFrame->add(diff_ageofframe_ms);
        }


        // (3) check receive time inside calling thread

        curr_periodPkt = reckPktTime - last_recvPktTime;
        if(curr_periodPkt > timeout)
        {
            if(local_verbose)
                yError() << "Board " << ipv4string << ": Gap of " << curr_periodPkt*1000 << "ms between two consecutive messages !!!";
        }


        if(false == justprinted)
        {   // dont count this in statistics because in previous interval we have printed and i dont want it.
            if(0 == num_lost_pkts)
            {
                stat_periodPkt->add(curr_periodPkt);
            }
            else
            {
                stat_lostPktgap->add(curr_periodPkt);
            }
        }
        else
        {
            justprinted = false;
            stat_printstatPktgap->add(curr_periodPkt);
        }

        // (4) add process time as passed in argument

        stat_precessPktTime->add(processPktTime);

        // (5) add packet size as apassed in argument

        stat_pktSize->add(size);
    }
    else
    {
        initted = true; // i have received first ever packet
        isInError = false;
    }

    // i update some static data

    last_seqNum = curr_seqNum;
    last_ageOfFrame = curr_ageOfFrame;
    last_recvPktTime = reckPktTime;
    count++;
    receivedPackets++;

    // i evaluate a possible report

    if(true == evalreport)
    {
        evalReport();
    }
}


void InfoOfRecvPkts::evalReport(void)
{
    double timenow = yarp::os::Time::now();

    if((timenow - timeoflastreport) > reportperiod)
    {
        printStatistics();
        clearStatistics();
    }

}

void InfoOfRecvPkts::forceReport(void)
{
    double timenow = yarp::os::Time::now();

//    if((timenow - timeoflastreport) > reportperiod)
    {
        printStatistics();
        clearStatistics();
    }

}



// eof







