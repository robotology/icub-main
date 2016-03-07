// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <string>
#include <ethManager.h>
#include <ethResource.h>
#include <errno.h>

#include "EOYtheSystem.h"

#include "EOropframe.h"

#include <stdexcept>      // std::out_of_range
#include <yarp/os/Network.h>
#include <yarp/os/NetType.h>
#include <ace/Time_Value.h>

#ifdef ICUB_USE_REALTIME_LINUX
#include <pthread.h>
#include <unistd.h>
#endif //ICUB_USE_REALTIME_LINUX

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;

TheEthManager* TheEthManager::handle = NULL;
yarp::os::Semaphore TheEthManager::managerSem = 1;

yarp::os::Semaphore TheEthManager::rxSem = 1;
yarp::os::Semaphore TheEthManager::txSem = 1;



// - class EthBoards

const char * EthBoards::defaultnames[EthBoards::maxEthBoards] =
{
    "noname-in-xml-board.1", "noname-in-xml-board.2", "noname-in-xml-board.3", "noname-in-xml-board.4",
    "noname-in-xml-board.5", "noname-in-xml-board.6", "noname-in-xml-board.7", "noname-in-xml-board.8",
    "noname-in-xml-board.9", "noname-in-xml-board.10", "noname-in-xml-board.11", "noname-in-xml-board.12",
    "noname-in-xml-board.13", "noname-in-xml-board.14", "noname-in-xml-board.15", "noname-in-xml-board.16",
    "noname-in-xml-board.17", "noname-in-xml-board.18", "noname-in-xml-board.19", "noname-in-xml-board.20",
    "noname-in-xml-board.21", "noname-in-xml-board.22", "noname-in-xml-board.23", "noname-in-xml-board.24",
    "noname-in-xml-board.25", "noname-in-xml-board.26", "noname-in-xml-board.27", "noname-in-xml-board.28",
    "noname-in-xml-board.29", "noname-in-xml-board.30", "noname-in-xml-board.31", "noname-in-xml-board.32"
};

const char * EthBoards::errorname[1] =
{
    "wrong-unknown-board"
};

EthBoards::EthBoards()
{
    memset(LUT, 0, sizeof(LUT));
    sizeofLUT = 0;
}


EthBoards::~EthBoards()
{
    memset(LUT, 0, sizeof(LUT));
    sizeofLUT = 0;
}


size_t EthBoards::number_of_resources(void)
{
    return(sizeofLUT);
}

size_t EthBoards::number_of_interfaces(EthResource * res)
{
    eOipv4addr_t ipv4 = res->getIPv4remoteAddress();
    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;


    if(index>=maxEthBoards)
    {
        return 0;
    }

    if(NULL != LUT[index].resource)
    {
        return 0;
    }

    return(LUT[index].numberofinterfaces);
}


bool EthBoards::add(EthResource* res)
{
    if(NULL == res)
    {
        return false;
    }

    eOipv4addr_t ipv4 = res->getIPv4remoteAddress();
    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;

    if(index>=maxEthBoards)
    {
        return false;
    }

    if(NULL != LUT[index].resource)
    {
        return false;
    }

    LUT[index].resource = res;
    LUT[index].ipv4 = ipv4;
    LUT[index].boardnumber = index;
    snprintf(LUT[index].name, sizeof(LUT[index].name), "%s", res->getName());
    if(0 == strlen(LUT[index].name))
    {   // use default name
        snprintf(LUT[index].name, sizeof(LUT[index].name), "%s", defaultnames[index]);
    }
    LUT[index].numberofinterfaces = 0;
    for(int i=0; i<ethFeatType_numberof; i++)
    {
        LUT[index].interfaces[i] = NULL;
    }

    sizeofLUT++;

    return true;
}


bool EthBoards::add(EthResource* res, IethResource* interface)
{
    if((NULL == res) || (NULL == interface))
    {
        return false;
    }

    iethresType_t type = interface->type();

    if(iethres_none == type)
    {
        return false;
    }

    // now i compute the index
    eOipv4addr_t ipv4 = res->getIPv4remoteAddress();
    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;
    if(index>=maxEthBoards)
    {
        return false;
    }

    if(res != LUT[index].resource)
    {
        return false;
    }

    if(NULL != LUT[index].interfaces[type])
    {
        return false;
    }

    // ok, i add it
    LUT[index].interfaces[type] = interface;
    LUT[index].numberofinterfaces ++;


    return true;
}


bool EthBoards::rem(EthResource* res)
{
    if(NULL == res)
    {
        return false;
    }

    // now i compute the index
    eOipv4addr_t ipv4 = res->getIPv4remoteAddress();
    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;
    if(index>=maxEthBoards)
    {
        return false;
    }

    if(res != LUT[index].resource)
    {
        return false;
    }

    LUT[index].resource = NULL;
    LUT[index].ipv4 = 0;
    LUT[index].boardnumber = 0;
    memset(LUT[index].name, 0, sizeof(LUT[index].name));
    LUT[index].numberofinterfaces = 0;
    for(int i=0; i<iethresType_numberof; i++)
    {
        LUT[index].interfaces[i] = NULL;
    }

    sizeofLUT--;

    return true;
}


bool EthBoards::rem(EthResource* res, iethresType_t type)
{
    if((NULL == res) || (iethres_none == type))
    {
        return false;
    }

    // now i compute the index
    eOipv4addr_t ipv4 = res->getIPv4remoteAddress();
    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;
    if(index>=maxEthBoards)
    {
        return false;
    }

    if(res != LUT[index].resource)
    {
        return false;
    }

    if(NULL != LUT[index].interfaces[type])
    {
        LUT[index].interfaces[type] = NULL;
        LUT[index].numberofinterfaces --;
    }

    return true;
}


EthResource* EthBoards::get_resource(eOipv4addr_t ipv4)
{
    EthResource * ret = NULL;

    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;
    if(index<maxEthBoards)
    {
        ret = LUT[index].resource;
    }

    return(ret);
}


IethResource* EthBoards::get_interface(eOipv4addr_t ipv4, eOprotID32_t id32)
{
    IethResource *dev = NULL;

    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;
    if(index>=maxEthBoards)
    {
        return NULL;
    }

    if(NULL == LUT[index].resource)
    {
        return dev;
    }

    iethresType_t type = iethres_none;
    eOprotEndpoint_t ep = eoprot_ID2endpoint(id32);
    switch(ep)
    {
        case eoprot_endpoint_management:
        {
            type = iethres_management;
        } break;

        case eoprot_endpoint_motioncontrol:
        {
            type = iethres_motioncontrol;
        } break;

        case eoprot_endpoint_skin:
        {
            type = iethres_skin;
        } break;

        case eoprot_endpoint_analogsensors:
        {
            eOprotEntity_t en = eoprot_ID2entity(id32);
            if(eoprot_entity_as_strain == en)
                type = iethres_analogstrain;
            else if(eoprot_entity_as_mais == en)
                type = iethres_analogmais;
            else if(eoprot_entity_as_inertial == en)
                type = iethres_analoginertial;
            else
                type = iethres_none;
        } break;

        default:
        {
            type = iethres_none;
        } break;
    }

    if(iethres_none != type)
    {
        dev = LUT[index].interfaces[type];
    }



    return dev;
}


const char * EthBoards::name(eOipv4addr_t ipv4)
{
    const char * ret = NULL;

    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;
    if(index<maxEthBoards)
    {
        ret = LUT[index].name;
    }
    else
    {
        ret = errorname[0]; // the last one contains an error string
    }

    return(ret);
}



bool EthBoards::execute(void (*action)(EthResource* res, void* p), void* par)
{
    if(NULL == action)
    {
        return(false);
    }

    for(int i=0; i<maxEthBoards; i++)
    {
        EthResource* res = LUT[i].resource;
        if(NULL != res)
        {
            action(res, par);
        }

    }

    return(true);
}


bool EthBoards::execute(eOipv4addr_t ipv4, void (*action)(EthResource* res, void* p), void* par)
{
    if(NULL == action)
    {
        return(false);
    }

    EthResource* res = get_resource(ipv4);

    if(NULL == res)
    {
        return(false);
    }

    action(res, par);

    return(true);
}



// -------------------------------------------------------------------\\
//            TheEthManager   Singleton
// -------------------------------------------------------------------\\



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

void ethEvalTXropframe(EthResource *r, void* p)
{
    if((NULL == r) || (NULL == p))
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
        ACE_INET_Addr ipaddress = r->getRemoteAddress();
        ethman->send(data2send, (size_t)numofbytes, ipaddress);
    }
}


bool TheEthManager::Transmission(void)
{
    lockTX(true);

    ethBoards->execute(ethEvalTXropframe, this);

    lockTX(false);

    return true;
}


bool TheEthManager::startCommunication(yarp::os::Searchable &cfgtotal)
{
    // we need: ip address of pc104, port used by socket, tx rate, rx rate.

    //ACE_INET_Addr ipaddress((u_short)12345, ((10 << 24) | (0 << 16) | (1 << 8) | (104)));            // it must be found ...
    int txrate = -1;                    // it uses default
    int rxrate = -1;                    // it uses default


    // localaddress

    Bottle groupPC104  = Bottle(cfgtotal.findGroup("PC104"));
    if (groupPC104.isNull())
    {
        yError() << "TheEthManager::startCommunication cannot find PC104 group in config files";
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

    ACE_UINT32 hostip = (ip1 << 24) | (ip2 << 16) | (ip3 << 8) | (ip4);
    ACE_INET_Addr myIP((u_short)port, hostip);

    myIP.addr_to_string(strIP, 64);

    yDebug() << "TheEthManager::startCommunication() has found IP for PC104 = " << strIP;

    // txrate
    if(cfgtotal.findGroup("PC104").check("PC104TXrate"))
    {
        int value = cfgtotal.findGroup("PC104").find("PC104TXrate").asInt();
        if(value > 0)
            txrate = value;
    }
    else
    {
        yWarning () << "TheEthManager::requestResource() cannot find ETH/PC104TXrate. thus using default value" << EthSender::EthSenderDefaultRate;
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
        yWarning () << "TheEthManager::requestResource() cannot find ETH/PC104RXrate. thus using default value" << EthReceiver::EthReceiverDefaultRate;
    }

    // localaddress
    if(false == createSocket(myIP, txrate, rxrate) )
    {
        yError () << "TheEthManager::requestResource() cannot create socket";
        return false;
    }

    // save local address

    localIPaddress = myIP;
    ipv4local.addr = eo_common_ipv4addr(ip1, ip2, ip3, ip4);
    ipv4local.port = port;

    return true;
}


EthResource *TheEthManager::requestResource2(IethResource *interface, yarp::os::Searchable &cfgtotal, yarp::os::Searchable &cfgtransceiver)
{

    // 1. must create communication objects: sender, receiver, socket

    if(communicationIsInitted == false)
    {
        yTrace() << "TheEthManager::requestResource2(): we need to init the communication";

        if(false == startCommunication(cfgtotal))
        {
            yError() << "TheEthManager::requestResource2(): cannot init the communication";
            return NULL;
        }
    }

    // now we extract the ip address of the board & its name

    Bottle groupEth  = Bottle(cfgtotal.findGroup("ETH"));
    if(groupEth.isNull())
    {
        yError() << "TheEthManager::requestResource2() cannot find ETH group in config files";
        return NULL;
    }
    Bottle paramIPboard(groupEth.find("IpAddress").asString());
    char str[64] = {0};
    strcpy(str, paramIPboard.toString().c_str());
    int ip1, ip2, ip3, ip4;
    sscanf(str, "\"%d.%d.%d.%d", &ip1, &ip2, &ip3, &ip4);
    eOipv4addr_t ipv4addr = eo_common_ipv4addr(ip1, ip2, ip3, ip4);
//    Bottle paramNameBoard(groupEth.find("Name").asString());
//    char boardname[64] = {0};
//    strcpy(boardname, paramNameBoard.toString().c_str());


    // i want to lock the use of resources managed by ethBoards to avoid that we attempt to use for TX a ethres not completely initted

    lockTXRX(true);

    // i do an attempt to get the resource.
    EthResource *rr = ethBoards->get_resource(ipv4addr);

    if(NULL == rr)
    {
        // i dont have the resource yet ...

        char ipinfo[20] = {0};
        eo_common_ipv4addr_to_string(ipv4addr, ipinfo, sizeof(ipinfo));

        rr = new EthResource;

        if(true == rr->open2(ipv4addr, cfgtotal, cfgtransceiver))
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

        yDebug() << "TheEthManager::requestResource2(): has just succesfully created a new EthResource for IP = " << ipinfo;
    }


    ethBoards->add(rr, interface);


    lockTXRX(false);

    return(rr);
}




//EthResource *TheEthManager::requestResource(yarp::os::Searchable &cfgtotal, yarp::os::Searchable &cfgtransceiver, yarp::os::Searchable &cfgprotocol, ethFeature_t &request)
//{
//    // Check if local socket is initted, if not do it.
//    ACE_TCHAR address[64] = {0};
//    snprintf(address, sizeof(address), "%s:%d", request.pc104IPaddr.string, request.pc104IPaddr.port);

//    yTrace() << "TheEthManager::requestResource(): called for board #" << request.boardNumber;

//    if(request.boardNumber > TheEthManager::maxBoards)
//    {
//        yError() << "FATAL ERROR: TheEthManager::requestResource() detected a board number beyond the maximum allowed (max, rqst) =" << maxBoards << request.boardNumber << ")";
//        return NULL;
//    }

//    if(request.boardIPaddr.ip4 != request.boardNumber)
//    {
//        yError() << "FATAL ERROR: TheEthManager::requestResource() detected a board number different from its ip4 address (boardNumber, ip4) =" << request.boardNumber << request.boardIPaddr.ip4 << ")";
//        return NULL;
//    }

//    if(0 == strlen(request.boardName))
//    {
//        // set default name: unset_xml_boardName__10.0.1.x
//        snprintf(request.boardName, sizeof(request.boardName), "unsetXMLboardName__10.0.1.%d", request.boardNumber);
//    }

//    eOipv4addr_t ipv4addr = eo_common_ipv4addr(request.pc104IPaddr.ip1, request.pc104IPaddr.ip2, request.pc104IPaddr.ip3, request.pc104IPaddr.ip4);
//    ACE_UINT32 hostip = (request.pc104IPaddr.ip1 << 24) | (request.pc104IPaddr.ip2 << 16) | (request.pc104IPaddr.ip3 << 8) | (request.pc104IPaddr.ip4);
//    ACE_INET_Addr myIP((u_short)request.pc104IPaddr.port, hostip);
//    myIP.dump();

//    int txrate = -1; // uses default
//    int rxrate = -1; // uses default

//    // if i find it in section ... of cfgtotal then i change it
//    if(cfgtotal.findGroup("PC104").check("PC104TXrate"))
//    {
//        int value = cfgtotal.findGroup("PC104").find("PC104TXrate").asInt();
//        if(value > 0)
//            txrate = value;
//    }
//    else
//    {
//        yWarning () << "TheEthManager::requestResource() cannot find ETH/PC104TXrate. thus using default value" << EthSender::EthSenderDefaultRate;;
//    }

//    if(cfgtotal.findGroup("PC104").check("PC104RXrate"))
//    {
//        int value = cfgtotal.findGroup("PC104").find("PC104RXrate").asInt();
//        if(value > 0)
//            rxrate = value;
//    }
//    else
//    {
//        yWarning () << "TheEthManager::requestResource() cannot find ETH/PC104RXrate. thus using default value" << EthReceiver::EthReceiverDefaultRate;
//    }

//    if(!createSocket(myIP, txrate, rxrate) )
//    {
//        return NULL;
//    }


//    // i want to lock the use of resources managed by ethBoards to avoid that we attempt to use for TX a ethres not completely initted

//    lockTXRX(true);

//    EthResource *rr = ethBoards->get_resource(ipv4addr);

//    if(NULL == rr)
//    {
//        // device doesn't exist yet: create it
//        yTrace() << "Creating ethResource for IP " << request.boardIPaddr.string;
//        rr = new EthResource;
//        if(false == rr->open(cfgtotal, cfgtransceiver, cfgprotocol, request))
//        {
//            yError() << "Error creating new ethReasource for IP " << request.boardIPaddr.string;
//            if(NULL != rr)
//            {
//                delete rr;
//            }

//            rr = NULL;
//            return NULL;
//        }

//        ethBoards->add(rr);
//    }

//    ethBoards->add(rr, request.interface);


//    lockTXRX(false);

//    return(rr);
//}



int TheEthManager::releaseResource(ethFeature_t &resource)
{

    int ret = 1; // -1 means that the singleton is not needed anymore

    eOipv4addr_t ipv4addr = eo_common_ipv4addr(resource.boardIPaddr.ip1, resource.boardIPaddr.ip2, resource.boardIPaddr.ip3, resource.boardIPaddr.ip4);

    EthResource* rr = ethBoards->get_resource(ipv4addr);
    if(NULL != rr)
    {
        bool success = rr->serviceStop(eomn_serv_category_all); // but you must change later on with eomn_serv_category_mc or ...
        success = success;


        // now we change internal data structure of ethBoards, thus .. must disable tx and rx
        lockTXRX(true);

        // remove the interface
        ethBoards->rem(rr, static_cast<iethresType_t>(resource.type));

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

    }

    return(ret);
}


const ACE_INET_Addr& TheEthManager::getLocalIPaddress(void)
{
    return(localIPaddress);
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


TheEthManager::TheEthManager()
{
    yTrace();

    communicationIsInitted = false;
    UDP_socket  = NULL;
    localIPaddress = ACE_INET_Addr("10.0.1.104:12345");

    ethBoards = new(EthBoards);

    TheEthManager::initEOYsystem();

    ipv4local.addr = eo_common_ipv4addr(10, 0, 1, 104);
    ipv4local.port = 12345;


    startUpTime = yarp::os::Time::now();
}


//EthSender* TheEthManager::getEthSender(void)
//{
//    return sender;
//}


//EthReceiver* TheEthManager::getEthReceiver(void)
//{
//    return receiver;
//}


double TheEthManager::getTimeOfStartUp(void)
{
    return startUpTime;
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


bool TheEthManager::createSocket(ACE_INET_Addr localaddress, int txrate, int rxrate)
{
    lock(true);


    if(!communicationIsInitted)
    {
        UDP_socket = new ACE_SOCK_Dgram();
        if(-1 == UDP_socket->open(localaddress))
        {
            char tmp[64] = {0};
            localaddress.addr_to_string(tmp, 64);
            yError() <<   "\n/--------------------------------------------------------------------------------------------------------------\\"
                     <<   "\n| TheEthManager::createSocket() is unable to bind to local IP address " << tmp
                     <<   "\n\\--------------------------------------------------------------------------------------------------------------/";
            delete UDP_socket;
            UDP_socket = NULL;
            communicationIsInitted = false;
        }
        else
        {
            communicationIsInitted = true;
            localIPaddress = localaddress;

            if((txrate <= 0) || (txrate > EthSender::EthSenderMaxRate))
            {
                txrate = EthSender::EthSenderDefaultRate;
            }
            if((rxrate <= 0) | (rxrate > EthReceiver::EthReceiverMaxRate))
            {
                rxrate = EthReceiver::EthReceiverDefaultRate;
            }
            sender = new EthSender(txrate);
            receiver = new EthReceiver(rxrate);

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
                yError() << "TheEthManager::createSocket() fails in starting UDP communication threads ethSender / ethReceiver";

                // stop threads
                stopCommunicationThreads();

                delete UDP_socket;
                communicationIsInitted = false;
                return false;
            }
            else
            {
                yTrace() << "TheEthManager::createSocket(): both UDP communication threads ethSender / ethReceiver start correctly!";

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
    //TODO harakiri
    delete handle;

    return true;
}

bool TheEthManager::stopCommunicationThreads()
{
    bool ret = true;
    // Stop method also make a join waiting the thread to exit
    if(sender->isRunning() )
        sender->stop();
    if(receiver->isRunning() )
    {
        receiver->stop();
    }
    return ret;
}


void delete_resources(EthResource *p, void* par)
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
        // for sure we should stop threads tx / rx. before stopping threads, flush all pkts not yet sent.
        flush();
        stopCommunicationThreads();

        // then we close and delete the socket
        lock(true);
        UDP_socket->close();
        delete UDP_socket;
        communicationIsInitted = false;

        // dont we delete sender and receiver ???
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

int TheEthManager::send(void *udpframe, size_t len, ACE_INET_Addr toaddress)
{
    ssize_t ret = UDP_socket->send(udpframe, len, toaddress);
    return ret;
}

bool TheEthManager::Reception(ACE_INET_Addr adr, uint64_t* data, ssize_t size, bool collectStatistics)
{
    ACE_UINT32 a32 = adr.get_ip_address();
    uint8_t ip4 = a32 & 0xff;
    uint8_t ip3 = (a32 >> 8) & 0xff;
    uint8_t ip2 = (a32 >> 16) & 0xff;
    uint8_t ip1 = (a32 >> 24) & 0xff;

    eOipv4addr_t ipv4addr = eo_common_ipv4addr(ip1, ip2, ip3, ip4);

    lockRX(true);

    EthResource* r = ethBoards->get_resource(ipv4addr);

    if(NULL != r)
    {
        if(false == r->canProcessRXpacket(data, size))
        {   // cannot give packet to ethresource
            yError() << "TheEthManager::Reception() cannot give a received packet of size" << size << "to EthResource because EthResource::canProcessRXpacket() returns false.";
        }
        else
        {
            r->processRXpacket(data, size, collectStatistics);
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


EthResource* TheEthManager::IPtoResource(ACE_INET_Addr adr)
{
    ACE_UINT32 a32 = adr.get_ip_address();
    uint8_t ip4 = a32 & 0xff;
    uint8_t ip3 = (a32 >> 8) & 0xff;
    uint8_t ip2 = (a32 >> 16) & 0xff;
    uint8_t ip1 = (a32 >> 24) & 0xff;

    eOipv4addr_t ipv4addr = eo_common_ipv4addr(ip1, ip2, ip3, ip4);
    return(ethBoards->get_resource(ipv4addr));
}

int TheEthManager::IPtoBoardNumber(ACE_INET_Addr adr)
{
    ACE_UINT32 a32 = adr.get_ip_address();
    uint8_t ip4 = a32 & 0xff;
    return(ip4);
}

int TheEthManager::GetNumberOfUsedBoards(void)
{
    return(ethBoards->number_of_resources());
}

const char * TheEthManager::getName(eOipv4addr_t ipv4)
{
    const char * ret = ethBoards->name(ipv4);

    return ret;
}

EthResource* TheEthManager::GetEthResource(eOipv4addr_t ipv4)
{
    return(ethBoards->get_resource(ipv4));
}

// Probably useless
bool TheEthManager::open()
{
    yTrace();

    return true;
}

bool TheEthManager::close()
{
    yTrace();

    return true;
}


void TheEthManager::flush()
{
    //#warning --> marco.accame: TODO think about how removing delay of 1 second
    // here sleep is essential in order to let sender thread send gotoconfig command.
    yarp::os::Time::delay(1);
}


// -- class EthSender
// -- here is it code

EthSender::EthSender(int txrate) : RateThread(txrate)
{
    rateofthread = txrate;
    yDebug() << "EthSender is a RateThread with txrate =" << rateofthread << "ms";
    yTrace();
}

EthSender::~EthSender()
{

}

bool EthSender::config(ACE_SOCK_Dgram *pSocket, TheEthManager* _ethManager)
{
    yTrace();
    send_socket = pSocket;
    ethManager  = _ethManager;

    return true;
}

bool EthSender::threadInit()
{
    yTrace() << "Do some initialization here if needed";

#ifdef ICUB_USE_REALTIME_LINUX
    /**
     * Make it realtime (works on both RT and Standard linux kernels)
     * - increase the priority upto the system IRQ's priorities (50) and less than the receiver thread
     * - set the scheduler to FIFO
     */
    struct sched_param thread_param;
    thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO)/2 - 1; // = 48
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);
#endif //ICUB_USE_REALTIME_LINUX

    return true;
}

//void EthSender::evalPrintTXstatistics(void)
//{
//#ifdef ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_
//    // For statistic purpose

//    unsigned int it = getIterations();
//    if(it == ethmanager_stats_frequency_numberofcycles)
//    {
//        printTXstatistics();
//    }
//#endif
//}


//void EthSender::printTXstatistics(void)
//{
//#ifdef ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_
//    // For statistic purpose

//    unsigned int it=getIterations();

//    double avPeriod, stdPeriod;
//    double avThTime, stdTime;

//    getEstUsed(avThTime, stdTime);
//    getEstPeriod(avPeriod, stdPeriod);

//    char string[128] = {0};
//    snprintf(string, sizeof(string), "  (STATS-TX)-> EthSender::run() thread run %d times, est period: %.3lf, +-%.4lf[ms], est used: %.3lf, +-%.4lf[ms]\n",
//                            it,
//                            avPeriod, stdPeriod,
//                            avThTime, stdTime);
//    yDebug() << string;

//    resetStat();

//#endif
//}



#if defined(ETHMANAGER_TEST_NEW_SENDER_RUN)


void EthSender::run()
{    
    // by calling this metod of ethManager, we put protection vs concurrency internal to the class.
    // for tx we must protect the EthResource not being changed. they can be changed by a device such as
    // embObjMotionControl etc which adds or releases its resources.
    ethManager->Transmission();
}


#else

#error dont use that

void EthSender::run()
{
    EthResource  *ethRes;
    uint16_t      bytes_to_send = 0;
    uint16_t      numofrops = 0;
    ethResRIt     riterator, _rBegin, _rEnd;


    /*
        Usare un reverse iterator per scorrere la lista dalla fine verso l'inizio. Questo aiuta a poter scorrere
        la lista con un iteratore anche durante la fase iniziale in cui vengono ancora aggiunti degli elementi,
        in teoria senza crashare. Al più salvarsi il puntatore alla rbegin sotto mutex prima di iniziare il ciclo,
        giusto per evitare che venga aggiunto un elemento in concomitanza con la lettura dell rbegin stesso.
        Siccome gli elementi vengono aggiunti solamente in coda alla lista, questa iterazione a ritroso non
        dovrebbe avere altri problemi e quindi safe anche senza il mutex che prende TUTTO il ciclo.
    */

    ethManager->getEMSlistRiterators(_rBegin, _rEnd);

    for(riterator = _rBegin; riterator != _rEnd && (isRunning()); riterator++)
    {
        p_sendData = NULL;
        bytes_to_send = 0;
        numofrops = 0;
        if(NULL == *riterator)
        {
            yError() << "EthManager::run, iterator==NULL";
            continue;
        }

        ethRes = (*riterator);

        // This uses directly the pointer of the transceiver
        bool transmitthepacket = ethRes->getTXpacket(&p_sendData, &bytes_to_send, &numofrops);

        if(true == transmitthepacket)
        {
            ACE_INET_Addr addr = ethRes->getRemoteAddress();
            int ret = ethManager->send(p_sendData, (size_t)bytes_to_send, addr);
        }

    }

}

#endif // defined(ETHMANAGER_TEST_NEW_SENDER_RUN)


EthReceiver::EthReceiver(int raterx): RateThread(raterx)
{
    rateofthread = raterx;
    yDebug() << "EthReceiver is a RateThread with rxrate =" << rateofthread << "ms";
    // ok, and now i get it from xml file ... if i find it.

    ConstString tmp = NetworkBase::getEnvironment("ETHSTAT_PRINT_INTERVAL");
    if (tmp != "")
    {
        statPrintInterval = (double)NetType::toInt(tmp);
    }
    else
    {
        statPrintInterval = 0.0;
    }
}

void EthReceiver::onStop()
{
    // in here i send a small packet to ... myself ?
    uint8_t tmp = 0;
    ethManager->send( &tmp, 1, ethManager->getLocalIPaddress());
}

EthReceiver::~EthReceiver()
{

}

bool EthReceiver::config(ACE_SOCK_Dgram *pSocket, TheEthManager* _ethManager)
{
    yTrace();
    recv_socket = pSocket;
    ethManager  = _ethManager;

    ACE_HANDLE sockfd = pSocket->get_handle();
    int retval;
    int32_t mysize = 1024*1024;     // 1Mb note:actually kernel uses memory with size doblem of mysize
                                    // with this size i'm sure ems pkts are not lost
    int len = sizeof(mysize);

    // the user can change buffer size by environment variable ETHRECEIVER_BUFFER_SIZE
    ConstString _dgram_buffer_size = NetworkBase::getEnvironment("ETHRECEIVER_BUFFER_SIZE");
    if (_dgram_buffer_size!="")
        mysize = NetType::toInt(_dgram_buffer_size);

    retval = ACE_OS::setsockopt  (sockfd, SOL_SOCKET, SO_RCVBUF, (char *)&mysize, sizeof(mysize));
    if (retval != 0)
    {
        int myerr = errno;
        yError()<< "ERROR in SetSockOpt SO_RCVBUF";
    }

    int32_t sock_input_buf_size = 0;
    retval = ACE_OS::getsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, (char *)&sock_input_buf_size, &len);
    if (retval != 0)
    {
        int myerr = errno;
        yError() << "ERROR inGetSockOpt SO_RCVBUF";
    }

    yWarning() << "in EthReceiver::config() the config socket has queue size = "<< sock_input_buf_size<< "; you request ETHRECEIVER_BUFFER_SIZE=" << _dgram_buffer_size;

#if defined(ETHMANAGER_RECEIVER_CHECK_SEQUENCE_NUMBER)
    for(int i=0; i<TheEthManager::maxBoards; i++)
    {
        recFirstPkt[i] = false;
        seqnumList[i] = 0;
    }
#endif

    return true;
}


bool EthReceiver::threadInit()
{
    yTrace() << "Do some initialization here if needed";

#ifdef ICUB_USE_REALTIME_LINUX
    /**
     * Make it realtime (works on both RT and Standard linux kernels)
     * - increase the priority upto the system IRQ's priorities (< 50)
     * - set the scheduler to FIFO
     */
    struct sched_param thread_param;
    thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO)/2; // = 49
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);
#endif //ICUB_USE_REALTIME_LINUX

    return true;
}


uint64_t getRopFrameAge(char *pck)
{
    return(eo_ropframedata_age_Get((EOropframeData*)pck));
}


#if defined(ETHMANAGER_RECEIVER_CHECK_SEQUENCE_NUMBER)

void EthReceiver::checkPktSeqNum(char* pktpayload, ACE_INET_Addr addr)
{
    int board = ethManager->IPtoBoardNumber(addr);
    uint64_t seqnum = eo_ropframedata_seqnum_Get((EOropframeData*)pktpayload);

    if(board > TheEthManager::maxBoards)
    {
        yError() << "EthReceiver::checkPktSeqNum() detected a board number beyond maximum allowed: (detected, maximum) =" << board << "," << TheEthManager::maxBoards << ")";
        return;
    }

    if(0 == board)
    {
        yError() << "EthReceiver::checkPktSeqNum() detected a zero board number";
        return;
    }

    int boardindex = board - 1;

    if(recFirstPkt[boardindex]==false)
    {
        seqnumList[boardindex] = seqnum;
        recFirstPkt[boardindex] = true;
        yError() << "EthREceiver: FIRST SEQ NUM for board=" << board << " is " << seqnum;
    }
    else
    {
        if(seqnum != (seqnumList[boardindex]+1))
        {
            yError() << "EthREceiver: ---LOST PKTS---board =" << board << " seq num rec=" << seqnum << " expected=" << seqnumList[boardindex]+1;
        }
        seqnumList[boardindex] = seqnum;
    }
}

#endif // #if defined(ETHMANAGER_RECEIVER_CHECK_SEQUENCE_NUMBER)


#if defined(ETHMANAGER_TEST_NEW_RECEIVER_RUN)

void EthReceiver::run()
{
    ssize_t       incoming_msg_size = 0;
    ACE_INET_Addr sender_addr;
    uint64_t      incoming_msg_data[EthResource::maxRXpacketsize/8];   // 8-byte aligned local buffer for incoming packet: it must be able to accomodate max size of packet
    const ssize_t incoming_msg_capacity = EthResource::maxRXpacketsize;

    int flags = 0;
#ifndef WIN32
    flags |= MSG_DONTWAIT;
#endif

    static int earlyexit = 0;

    int maxUDPpackets = EthReceiver::rateofthread*ethManager->GetNumberOfUsedBoards() + 2 + earlyexit*5; // marco.accame: set it as the number of boards plus a ... margin of 30% plus a +/- offset which depends on activity

    earlyexit = 0;

    for(int i=0; i<maxUDPpackets; i++)
    {
        incoming_msg_size = recv_socket->recv((void *) incoming_msg_data, incoming_msg_capacity, sender_addr, flags);
        if(incoming_msg_size <= 0)
        {
            // marco.accame: i prefer using <= 0.
            earlyexit = 1;
            return;
        }

#if defined(ETHMANAGER_RECEIVER_CHECK_SEQUENCE_NUMBER)
        // redundant check. this check is already done inside hostTransceiver wherein case of errors it is called:
        // cpp_protocol_callback_incaseoferror_in_sequencenumberReceived(EOreceiver *r)
        checkPktSeqNum((char *)incoming_msg_data, sender_addr);
#endif

        // we have a packet ... we give it to the ethmanager for it parsing
        bool collectStatistics = (statPrintInterval > 0) ? true : false;
        ethManager->Reception(sender_addr, incoming_msg_data, incoming_msg_size, collectStatistics);
    }

}

#else

#error dont use it

#ifdef ETHRECEIVER_ISPERIODICTHREAD

void EthReceiver::run()
{

    //yTrace();

    ACE_TCHAR     address[64];
    EthResource  *ethRes;
    ssize_t       incoming_msg_size = 0;
    ACE_INET_Addr sender_addr;
    uint64_t      incoming_msg_data[EthResource::maxRXpacketsize/8]; // 8-byte aligned local buffer for incoming packet: it must be able to accomodate max size of packet
    const int     incoming_msg_capacity = EthResource::maxRXpacketsize;


    //ACE_Time_Value recvTimeOut;
    //recvTimeOut.set(0.010f); // timeout of socket reception is 10 milliseconds

    //double myTestTimeout = recvTimeOut.sec() + (double)recvTimeOut.msec()/1000.0f;

    int flags = 0;
#ifndef WIN32
    flags |= MSG_DONTWAIT;
#endif

    static int earlyexit = 0;

    int maxUDPpackets = EthReceiver::rateofthread*ethManager->GetNumberOfUsedBoards() + 2 + earlyexit*5; // marco.accame: set it as the number of boards plus a ... margin of 30% plus a +/- offset which depends on activity

    earlyexit = 0;

    for(int i=0; i<maxUDPpackets; i++)
    {
        incoming_msg_size = recv_socket->recv((void *) incoming_msg_data, incoming_msg_capacity, sender_addr, flags);
        if(incoming_msg_size <= 0)
        {
            // marco.accame: i prefer using <= 0.
            earlyexit = 1;
            return;
        }

        ethRes = ethManager->IPtoResource(sender_addr);
        if(NULL != ethRes)
        {
            if(false == ethRes->canProcessRXpacket(incoming_msg_data, incoming_msg_size))
            {   // cannot give packet to ethresource
                yError() << "EthReceiver::run() cannot give a received packet of size" << incoming_msg_size << "to EthResource because EthResource::canProcessRXpacket() returns false.";
            }
            else
            {
                // we collect statistics only if we ever print them, thus statPrintInterval > 0
                bool collectStatistics = (statPrintInterval > 0) ? true : false;
                ethRes->processRXpacket(incoming_msg_data, incoming_msg_size, collectStatistics);
            }

        }
        else
        {
        //    sender_addr.addr_to_string(address, sizeof(address));
        //    yError() << "EthReceiver::run() cannot get a ethres associated to address" << address;
        }


    }

}

#else
    #error -> ETHRECEIVER_ISPERIODICTHREAD must be defined
#endif

#endif // 1


// eof




