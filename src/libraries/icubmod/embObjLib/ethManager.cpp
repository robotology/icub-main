// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Alberto cardellino, Marco Accame
 * email:   alberto.cardellino@iit.it, marco.accame@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/


// --------------------------------------------------------------------------------------------------------------------
// - public interface
// --------------------------------------------------------------------------------------------------------------------

#include <ethManager.h>



// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include <string>
#include <ethResource.h>
#include <errno.h>

#include "EOYtheSystem.h"

#include "EOropframe.h"

#include <stdexcept>      // std::out_of_range
#include <yarp/os/Network.h>
#include <yarp/os/NetType.h>
#include <ace/Time_Value.h>

#if defined(__unix__)
#include <pthread.h>
#include <unistd.h>
#endif

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;



#include <fakeEthResource.h>
#include <ethResource.h>
#include <ethParser.h>

using namespace eth;



// --------------------------------------------------------------------------------------------------------------------
// - pimpl: private implementation (see scott meyers: item 22 of effective modern c++, item 31 of effective c++
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - the class
// --------------------------------------------------------------------------------------------------------------------


// - class eth::TheEthManager


TheEthManager* TheEthManager::handle = NULL;
yarp::os::Semaphore TheEthManager::managerSem = 1;

yarp::os::Semaphore TheEthManager::rxSem = 1;
yarp::os::Semaphore TheEthManager::txSem = 1;



TheEthManager::TheEthManager()
{
    // it is a singleton. the constructor is private.
    communicationIsInitted = false;
    UDP_socket  = NULL;

    // the container of ethernet boards: resources and attached interfaces
    ethBoards = new(eth::EthBoards);

    // required by embobj system
    TheEthManager::initEOYsystem();

    // default address
    ipv4local.addr = eo_common_ipv4addr(10, 0, 1, 104);
    ipv4local.port = 12345;

    // the time of creation according to yarp
    startUpTime = yarp::os::Time::now();
}



void delete_resources(eth::AbstractEthResource *p, void* par)
{
    delete p;
}


TheEthManager::~TheEthManager()
{
    yTrace();

    // Deinitialize feature interface
    feat_DeInitialise();

    // close communication (tx and rx threads, socket) BUT only if they were created and initted
    if(isCommunicationInitted())
    {
        // for sure we should stop threads tx / rx.
        // before stopping threads, flush all pkts not yet sent. we wait 250 ms.
        yarp::os::Time::delay(0.250);
        stopCommunicationThreads();

        // then we close and delete the socket
        lock(true);
        UDP_socket->close();
        delete UDP_socket;
        communicationIsInitted = false;


        delete sender;
        delete receiver;

        lock(false);
    }


    lock(true);

    // remove all ethresource ... we dont need to call lockTXRX() because we are not transmitting now
    ethBoards->execute(delete_resources, NULL);
    delete ethBoards;

    lock(false);

    handle = NULL;
}



TheEthManager *TheEthManager::instance()
{
    yTrace();
    // marco.accame: in here we dont use this->lock() because if object does not already exists, the function does (?) not exist.
    // much better using the static semaphore instead
    managerSem.wait();
    if (NULL == handle)
    {
        yTrace() << "Calling EthManager Constructor";
        handle = new TheEthManager();
        if (NULL == handle)
            yError() << "While calling EthManager constructor";
        else
            feat_Initialise(static_cast<void*>(handle)); // we give the pointer to the feature-interface c module
    }
    managerSem.post();

    return handle;
}


bool TheEthManager::killYourself()
{
    yTrace();
    delete handle;

    return true;
}


// this function is called by the embobj error manager
void embOBJerror(eOerrmanErrorType_t errtype, const char *info, eOerrmanCaller_t *caller, const eOerrmanDescriptor_t *des)
{
    const char defobjstr[] = "EO?";
    const char *eobjstr = (NULL == caller) ? (defobjstr) : (caller->eobjstr);

    yError() << "embOBJerror(): errtype = " << eo_errman_ErrorStringGet(eo_errman_GetHandle(), errtype) << "from EOobject = " << eobjstr << " w/ message = " << info;

    if(errtype == eo_errortype_fatal)
    {
        yError() << "embOBJerror(): FATAL ERROR: the calling thread shall now be stopped in a forever loop here inside";
        for(;;);
    }

}

void TheEthManager::initEOYsystem(void)
{
    // marco.accame: in here we init the embOBJ system for YARP.
    eOerrman_cfg_t errmanconfig = {0};
    errmanconfig.extfn.usr_on_error     = embOBJerror;
    const eOysystem_cfg_t *syscfg       = NULL;
    const eOmempool_cfg_t *mpoolcfg     = NULL;     // uses standard mode
    //const eOerrman_cfg_t *errmancf      = NULL;     // uses default mode
    eoy_sys_Initialise(syscfg, mpoolcfg, &errmanconfig);
}




void ethEvalTXropframe(eth::AbstractEthResource *r, void* p)
{
    if((NULL == r) || (NULL == p) || (r->isFake()))
    {
        return;
    }

    TheEthManager *ethman = (TheEthManager*)p;

    uint16_t numofbytes = 0;
    uint16_t numofrops = 0;
    uint8_t* data2send = NULL;
    bool transmitthepacket = r->getTXpacket(&data2send, &numofbytes, &numofrops);

    if(true == transmitthepacket)
    {
        eOipv4addressing_t ipv4addressing;
        r->getIPv4remoteAddressing(ipv4addressing);
        ethman->sendPacket(data2send, (size_t)numofbytes, ipv4addressing);
    }
}


bool TheEthManager::Transmission(void)
{
    lockTX(true);

    ethBoards->execute(ethEvalTXropframe, this);

    lockTX(false);

    return true;
}


void ethEvalPresence(eth::AbstractEthResource *r, void* p)
{
    if((NULL == r) || (NULL == p))
    {
        return;
    }
    r->Check();
}


bool TheEthManager::CheckPresence(void)
{
    ethBoards->execute(ethEvalPresence, this);
    return true;
}

#if 1

bool TheEthManager::verifyEthBoardInfo(yarp::os::Searchable &cfgtotal, eOipv4addr_t &boardipv4, string boardipv4string, string boardname)
{
    eth::parser::boardData bdata;
    if(false == eth::parser::read(cfgtotal, bdata))
    {
        yError() << "TheEthManager::verifyEthBoardInfo() fails";
        return false;
    }


    boardipv4 = bdata.properties.ipv4addressing.addr;
    boardipv4string = bdata.properties.ipv4string;
    boardname = bdata.settings.name;

    return true;
}



#else

bool TheEthManager::verifyEthBoardInfo(yarp::os::Searchable &cfgtotal, eOipv4addr_t* boardipv4, char *boardipv4string, int stringsize, char *boardNameStr, int sizeofBoardNameStr)
{
    // Get PC104 address and port from config file
    Bottle groupPC104  = Bottle(cfgtotal.findGroup("PC104"));
    if (groupPC104.isNull())
    {
        yError() << "TheEthManager::verifyEthBoardInfo() cannot find PC104 group in config files";
        return false;
    }
    Value *PC104IpAddress_p;
    if (!groupPC104.check("PC104IpAddress", PC104IpAddress_p))
    {
        yError() << "TheEthManager::verifyEthBoardInfo() cannot find PC104/PC104IpAddress";
        return false;
    }
    Value *PC104IpPort_p;
    if (!groupPC104.check("PC104IpPort", PC104IpPort_p))
    {
        yError() << "TheEthManager::verifyEthBoardInfo() cannot find PC104/PC104IpPort";
        return false;
    }


    // get ip adress and port for board from config file
    Bottle groupETH_BOARD  = Bottle(cfgtotal.findGroup("ETH_BOARD"));
    if (groupETH_BOARD.isNull())
    {
        yError() << "TheEthManager::verifyEthBoardInfo() cannot find ETH_BOARD group in config files";
        return false;
    }
    Bottle groupETH_BOARD_PROPERTIES  = Bottle(groupETH_BOARD.findGroup("ETH_BOARD_PROPERTIES"));
    if (groupETH_BOARD_PROPERTIES.isNull())
    {
        yError() << "TheEthManager::verifyEthBoardInfo() cannot find ETH_BOARD_PROPERTIES group in config files";
        return false;
    }

    Value *BOARDIpAddress_p;
    if (!groupETH_BOARD_PROPERTIES.check("IpAddress", BOARDIpAddress_p))
    {
        yError() << "TheEthManager::verifyEthBoardInfo() cannot find ETH_BOARD_PROPERTIES/IpAddress";
        return false;
    }

    char strIPaddress[32];
    Bottle parameter2( groupETH_BOARD_PROPERTIES.find("IpAddress").asString() );
    strcpy(strIPaddress, parameter2.toString().c_str());

    int ip1, ip2, ip3, ip4;
    sscanf(strIPaddress,"\"%d.%d.%d.%d", &ip1, &ip2, &ip3, &ip4);

    eOipv4addr_t ipv4 = eo_common_ipv4addr(ip1, ip2, ip3, ip4);

    if(NULL != boardipv4)
    {
        *boardipv4 = ipv4;
    }

    if((NULL != boardipv4string) && (stringsize > 0))
    {
        eo_common_ipv4addr_to_string(ipv4, boardipv4string, stringsize);
    }



    if((NULL != boardNameStr) && (0 != sizeofBoardNameStr))
    {
        Bottle groupEthBoardSettings = Bottle(groupETH_BOARD.findGroup("ETH_BOARD_SETTINGS"));
        if(groupEthBoardSettings.isNull())
        {
            yError() << "TheEthManager::verifyEthBoardInfo() cannot find ETH_BOARD_SETTINGS group in config files";
            return NULL;
        }

        Bottle paramNameBoard(groupEthBoardSettings.find("Name").asString());
        snprintf(boardNameStr, sizeofBoardNameStr, "%s", paramNameBoard.toString().c_str());
    }
    return true;
}

#endif

#if 1
bool TheEthManager::initCommunication(yarp::os::Searchable &cfgtotal)
{
    eth::parser::pc104Data pcdata;
    if(false == eth::parser::read(cfgtotal, pcdata))
    {
        yError() << "TheEthManager::initCommunication() fails";
        return false;
    }

    int txrate = pcdata.txrate;
    int rxrate = pcdata.rxrate;
    eOipv4addressing_t tmpaddress = pcdata.localaddressing;
    embBoardsConnected = pcdata.embBoardsConnected;

    // localaddress
    if(false == createCommunicationObjects(tmpaddress, txrate, rxrate) )
    {
        yError () << "TheEthManager::initCommunication() cannot create communication objects";
        return false;
    }

    // save local address
    ipv4local.addr = tmpaddress.addr;
    ipv4local.port = tmpaddress.port;

    return true;
}

#else
bool TheEthManager::initCommunication(yarp::os::Searchable &cfgtotal)
{
    int txrate = -1;                    // it uses default
    int rxrate = -1;                    // it uses default

    embBoardsConnected = true;



    Bottle groupDEBUG  = cfgtotal.findGroup("DEBUG");
    if ((! groupDEBUG.isNull()) && (groupDEBUG.check("embBoardsConnected")))
        embBoardsConnected = groupDEBUG.find("embBoardsConnected").asBool();

    if(!embBoardsConnected)
    {
        yError() << "ATTENTION: NO EMBEDDED BOARDS CONNECTED. YOU ARE IN DEBUG MODE";
    }

    // localaddress

    Bottle groupPC104  = Bottle(cfgtotal.findGroup("PC104"));
    if (groupPC104.isNull())
    {
        yError() << "TheEthManager::initCommunication cannot find PC104 group in config files";
        return false;
    }

    Value *value;

    if (!groupPC104.check("PC104IpAddress", value))
    {
        yError() << "missing PC104/PC104IpAddress in config files";
        return false;
    }
    if (!groupPC104.check("PC104IpPort", value))
    {
        yError() << "missing PC104/PC104IpPort in config files";
        return false;
    }

    Bottle paramIPaddress(groupPC104.find("PC104IpAddress").asString());
    ACE_UINT16 port = groupPC104.find("PC104IpPort").asInt();              // .get(1).asInt();
    char strIP[64] = {0};


    snprintf(strIP, sizeof(strIP), "%s", paramIPaddress.toString().c_str());
    // strIP is now "10.0.1.104" ... i want to remove the "".... VERY IMPORTANT: use \" in sscanf
    int ip1, ip2, ip3, ip4;
    sscanf(strIP, "\"%d.%d.%d.%d", &ip1, &ip2, &ip3, &ip4);

    eOipv4addressing_t tmpaddress;
    tmpaddress.addr = eo_common_ipv4addr(ip1, ip2, ip3, ip4);
    tmpaddress.port = port;

    yDebug() << "TheEthManager::initCommunication() has found IP for PC104 = " << strIP;

    // txrate
    if(cfgtotal.findGroup("PC104").check("PC104TXrate"))
    {
        int value = cfgtotal.findGroup("PC104").find("PC104TXrate").asInt();
        if(value > 0)
            txrate = value;
    }
    else
    {
        yWarning () << "TheEthManager::initCommunication() cannot find ETH/PC104TXrate. thus using default value" << EthSender::EthSenderDefaultRate;
    }

    // rxrate
    if(cfgtotal.findGroup("PC104").check("PC104RXrate"))
    {
        int value = cfgtotal.findGroup("PC104").find("PC104RXrate").asInt();
        if(value > 0)
            rxrate = value;
    }
    else
    {
        yWarning () << "TheEthManager::initCommunication() cannot find ETH/PC104RXrate. thus using default value" << EthReceiver::EthReceiverDefaultRate;
    }

    // localaddress
    if(false == createCommunicationObjects(tmpaddress, txrate, rxrate) )
    {
        yError () << "TheEthManager::initCommunication() cannot create communication objects";
        return false;
    }

    // save local address
    ipv4local.addr = tmpaddress.addr;
    ipv4local.port = tmpaddress.port;

    return true;
}
#endif

#if 1

eth::AbstractEthResource *TheEthManager::requestResource2(IethResource *interface, yarp::os::Searchable &cfgtotal)
{
    if(false == communicationIsInitted)
    {
        yTrace() << "TheEthManager::requestResource2(): we need to init the communication";

        if(false == initCommunication(cfgtotal))
        {
            yError() << "TheEthManager::requestResource2(): cannot init the communication";
            return NULL;
        }
    }

    eth::parser::boardData bdata;
    if(false == eth::parser::read(cfgtotal, bdata))
    {
        yError() << "TheEthManager::requestResource2() fails";
        return NULL;
    }

    eOipv4addr_t ipv4addr = bdata.properties.ipv4addressing.addr;

    // i want to lock the use of resources managed by ethBoards to avoid that we attempt to use for TX a ethres not completely initted

    lockTXRX(true);

    // i do an attempt to get the resource.
    eth::AbstractEthResource *rr = ethBoards->get_resource(ipv4addr);

    if(NULL == rr)
    {
        // i dont have the resource yet ...

        if(true == embBoardsConnected)
        {
            rr = new EthResource();
        }
        else
        {
            rr = new eth::FakeEthResource();
        }

        if(true == rr->open2(ipv4addr, cfgtotal))
        {
            ethBoards->add(rr);
        }
        else
        {
            yError() << "TheEthManager::requestResource2(): error creating a new ethResource for IP = " << bdata.properties.ipv4string;

            if(NULL != rr)
            {
                delete rr;
            }

            rr = NULL;
            return NULL;
        }

        yDebug() << "TheEthManager::requestResource2(): has just succesfully created a new EthResource for board of type" << rr->getBoardTypeString()<< "with IP = " << bdata.properties.ipv4string;
    }


    ethBoards->add(rr, interface);


    lockTXRX(false);

    return(rr);
}

#else

eth::AbstractEthResource *TheEthManager::requestResource2(IethResource *interface, yarp::os::Searchable &cfgtotal)
{
    if(communicationIsInitted == false)
    {
        yTrace() << "TheEthManager::requestResource2(): we need to init the communication";

        if(false == initCommunication(cfgtotal))
        {
            yError() << "TheEthManager::requestResource2(): cannot init the communication";
            return NULL;
        }
    }

    // now we extract the ip address of the board

    Bottle groupEthBoard  = Bottle(cfgtotal.findGroup("ETH_BOARD"));
    if(groupEthBoard.isNull())
    {
        yError() << "TheEthManager::requestResource2() cannot find ETH_BOARD group in config files";
        return NULL;
    }
    Bottle groupEthBoardProps = Bottle(groupEthBoard.findGroup("ETH_BOARD_PROPERTIES"));
    if(groupEthBoardProps.isNull())
    {
        yError() << "TheEthManager::requestResource2() cannot find ETH_BOARD_PROPERTIES group in config files";
        return NULL;
    }

    Bottle paramIPboard(groupEthBoardProps.find("IpAddress").asString());
    char str[64] = {0};
    strcpy(str, paramIPboard.toString().c_str());
    int ip1, ip2, ip3, ip4;
    sscanf(str, "\"%d.%d.%d.%d", &ip1, &ip2, &ip3, &ip4);
    eOipv4addr_t ipv4addr = eo_common_ipv4addr(ip1, ip2, ip3, ip4);

    // i want to lock the use of resources managed by ethBoards to avoid that we attempt to use for TX a ethres not completely initted

    lockTXRX(true);

    // i do an attempt to get the resource.
    eth::AbstractEthResource *rr = ethBoards->get_resource(ipv4addr);

    if(NULL == rr)
    {
        // i dont have the resource yet ...

        char ipinfo[20] = {0};
        eo_common_ipv4addr_to_string(ipv4addr, ipinfo, sizeof(ipinfo));

        if(embBoardsConnected)
            rr = new EthResource();
        else
            rr = new eth::FakeEthResource();

        if(true == rr->open2(ipv4addr, cfgtotal))
        {
            ethBoards->add(rr);
        }
        else
        {

            yError() << "TheEthManager::requestResource2(): error creating a new ethResource for IP = " << ipinfo;
            if(NULL != rr)
            {
                delete rr;
            }

            rr = NULL;
            return NULL;
        }

        yDebug() << "TheEthManager::requestResource2(): has just succesfully created a new EthResource for board of type" << rr->getBoardTypeString()<< "with IP = " << ipinfo;
    }


    ethBoards->add(rr, interface);


    lockTXRX(false);

    return(rr);
}
#endif


int TheEthManager::releaseResource2(eth::AbstractEthResource* ethresource, IethResource* interface)
{
    int ret = 1; // -1 means that the singleton is not needed anymore. 0 means error
    if((NULL == ethresource) || (NULL == interface))
    {
        yError() << "TheEthManager::releaseResource2(): there is an attempt to release a NULL EthResource or IethResource";
        return 0;
    }


    eth::AbstractEthResource* rr = ethresource;

    iethresType_t type = interface->type();

    // but you must change later on with eomn_serv_category_mc or ...
    eOmn_serv_category_t category = eomn_serv_category_all;
    switch(type)
    {
        case iethres_analogmais:
        {
            category = eomn_serv_category_mais;
        } break;

        case iethres_analogstrain:
        {
            category = eomn_serv_category_strain;
        } break;

        case iethres_motioncontrol:
        {
            category = eomn_serv_category_mc;
        } break;

        case iethres_skin:
        {
            category = eomn_serv_category_skin;
        } break;

        case iethres_analogvirtual:
        {
            category = eomn_serv_category_none;
        } break;

        case iethres_analoginertial:
        {
            category = eomn_serv_category_inertials;
        } break;

        default:
        {
            category = eomn_serv_category_none;
        } break;
    }

    // marco.accame on 8 mar 2016: better to stop all services so that all regulars are removed and board immediately
    // exits the control loop. later on i will remove this line .... when robotInterface stops crashing in exit.
    // marco.accame on 10 mar 2016: now robotInterface does not crash in exit anymore. see today's commit.
    // however, for now i keep on stopping all services in the board. i will change it later on if needed.

    category = eomn_serv_category_all;

    if(eomn_serv_category_none != category)
    {
        bool success = rr->serviceStop(category);
        success = success;
    }

    // at this time, if the serviceStop() is successful [hey, change its interfce to retrieve num of rops in category, in total and run mode and print them]
    // the ropframe sent now do not contain any regular for the interface anymore, thus we can just removing the interface in list of those assciated
    // to the resource, without any harm. only thing is: protect ethBoards with a mutex.

    // now we change internal data structure of ethBoards, thus .. must disable tx and rx
    lockTXRX(true);

    // remove the interface
    ethBoards->rem(rr, type);

    int remaining = ethBoards->number_of_interfaces(rr);
    if(0 == remaining)
    {   // remove also the resource
        rr->close();
        ethBoards->rem(rr);
        delete rr;
    }

    if(0 == ethBoards->number_of_resources())
    {   // we dont have any more resources
        ret = -1;
    }

    lockTXRX(false);


    return(ret);
}




const eOipv4addressing_t& TheEthManager::getLocalIPV4addressing(void)
{
    return(ipv4local);
}


IethResource* TheEthManager::getInterface(eOipv4addr_t ipv4, eOprotID32_t id32)
{
    IethResource *interfacePointer = ethBoards->get_interface(ipv4, id32);

    return interfacePointer;
}


double TheEthManager::getLifeTime(void)
{
    return(yarp::os::Time::now() - startUpTime);
}



bool TheEthManager::createCommunicationObjects(const eOipv4addressing_t &localaddress, int txrate, int rxrate)
{
    lock(true);

    ACE_INET_Addr inetaddr = toaceinet(localaddress);

    if(!communicationIsInitted)
    {
        UDP_socket = new ACE_SOCK_Dgram();
        if((embBoardsConnected)  && (-1 == UDP_socket->open(inetaddr)))
        {
            char tmp[64] = {0};
            inetaddr.addr_to_string(tmp, 64);
            yError() <<   "\n/--------------------------------------------------------------------------------------------------------------\\"
                     <<   "\n| TheEthManager::createCommunicationObjects() is unable to bind to local IP address " << tmp
                     <<   "\n\\--------------------------------------------------------------------------------------------------------------/";
            delete UDP_socket;
            UDP_socket = NULL;
            communicationIsInitted = false;
        }
        else
        {
            communicationIsInitted = true;
            ipv4local = localaddress;

            if((txrate <= 0) || (txrate > EthSender::EthSenderMaxRate))
            {
                txrate = eth::EthSender::EthSenderDefaultRate;
            }
            if((rxrate <= 0) | (rxrate > EthReceiver::EthReceiverMaxRate))
            {
                rxrate = EthReceiver::EthReceiverDefaultRate;
            }
            sender = new eth::EthSender(txrate);
            receiver = new eth::EthReceiver(rxrate);

            sender->config(UDP_socket, this);
            receiver->config(UDP_socket, this);

            /* Start the threads sending to and receiving messages from the boards.
             * It will execute the threadInit and pass its return value to the following calls
             * afterStart to check if they started correctly.
             */
            bool ret1, ret2;
            ret1 = sender->start();
            ret2 = receiver->start();

            if(!ret1 || !ret2)
            {
                yError() << "TheEthManager::createCommunicationObjects() fails in starting UDP communication threads ethSender / ethReceiver";

                // stop threads
                stopCommunicationThreads();

                delete UDP_socket;
                communicationIsInitted = false;
                return false;
            }
            else
            {
                yTrace() << "TheEthManager::createCommunicationObjects(): both UDP communication threads ethSender / ethReceiver start correctly!";

            }
        }
    }

    lock(false);
    return communicationIsInitted;
}


bool TheEthManager::isCommunicationInitted(void)
{
    yTrace();
    bool ret;
    lock(true);
    ret = communicationIsInitted;
    lock(false);
    return ret;
}



bool TheEthManager::stopCommunicationThreads()
{
    bool ret = true;
    // Stop method also make a join waiting the thread to exit
    if(sender->isRunning())
    {
        sender->stop();
    }
    if(receiver->isRunning())
    {
        receiver->stop();
    }
    return ret;
}



int TheEthManager::sendPacket(void *udpframe, size_t len, const eOipv4addressing_t &toaddressing)
{
    ACE_INET_Addr inetaddr = toaceinet(toaddressing);
    ssize_t ret = UDP_socket->send(udpframe, len, inetaddr);
    return ret;
}



eOipv4addr_t TheEthManager::toipv4addr(const ACE_INET_Addr &aceinetaddr)
{
    ACE_UINT32 a32 = aceinetaddr.get_ip_address();
    uint8_t ip4 = a32 & 0xff;
    uint8_t ip3 = (a32 >> 8) & 0xff;
    uint8_t ip2 = (a32 >> 16) & 0xff;
    uint8_t ip1 = (a32 >> 24) & 0xff;
    return eo_common_ipv4addr(ip1, ip2, ip3, ip4);
}

ACE_INET_Addr TheEthManager::toaceinet(const eOipv4addressing_t &ipv4addressing)
{
    uint8_t ip1, ip2, ip3, ip4;
    eo_common_ipv4addr_to_decimal(ipv4addressing.addr, &ip1, &ip2, &ip3, &ip4);
    ACE_UINT32 hostip = (ip1 << 24) | (ip2 << 16) | (ip3 << 8) | (ip4);
    ACE_INET_Addr myIP((u_short)ipv4addressing.port, hostip);

    return myIP;
}


bool TheEthManager::Reception(eOipv4addr_t from, uint64_t* data, ssize_t size)
{
    lockRX(true);

    eth::AbstractEthResource* r = ethBoards->get_resource(from);

    if((NULL != r) && (!r->isFake()))
    {
        r->Tick();

        if(false == r->canProcessRXpacket(data, size))
        {   // cannot give packet to ethresource
            yError() << "TheEthManager::Reception() cannot give a received packet of size" << size << "to EthResource because EthResource::canProcessRXpacket() returns false.";
        }
        else
        {
            r->processRXpacket(data, size);
        }

    }
    else
    {
    //    adr.addr_to_string(address, sizeof(address));
    //    yError() << "TheEthManager::Reception cannot get a ethres associated to address" << address;
    }

    lockRX(false);


    return(true);
}



int TheEthManager::getNumberOfResources(void)
{
    return(ethBoards->number_of_resources());
}


const string & TheEthManager::getName(eOipv4addr_t ipv4)
{
    return ethBoards->name(ipv4);
}


eth::AbstractEthResource* TheEthManager::getEthResource(eOipv4addr_t ipv4)
{
    return(ethBoards->get_resource(ipv4));
}


bool TheEthManager::lock(bool on)
{
    if(on)
    {
        managerSem.wait();
    }
    else
    {
        managerSem.post();
    }

    return true;
}


bool TheEthManager::lockTX(bool on)
{
    if(on)
    {
        txSem.wait();
    }
    else
    {
        txSem.post();
    }

    return true;
}


bool TheEthManager::lockRX(bool on)
{
    if(on)
    {
        rxSem.wait();
    }
    else
    {
        rxSem.post();
    }

    return true;
}

bool TheEthManager::lockTXRX(bool on)
{
    if(on)
    {
        txSem.wait();
        rxSem.wait();
    }
    else
    {
        rxSem.post();
        txSem.post();
    }

    return true;
}


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



