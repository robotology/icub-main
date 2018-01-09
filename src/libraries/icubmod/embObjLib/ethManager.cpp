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

#if defined(__unix__)
#include <pthread.h>
#include <unistd.h>
#endif

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;

TheEthManager* TheEthManager::handle = NULL;
yarp::os::Semaphore TheEthManager::managerSem = 1;

yarp::os::Semaphore TheEthManager::rxSem = 1;
yarp::os::Semaphore TheEthManager::txSem = 1;


// - class IethResource

const char * IethResource::names[iethresType_numberof+1] =
{
    "resManagement", "resAnalogMais", "resAnalogStrain", "resMotionControl",
    "resSkin", "resAnalogVirtual", "resAnalogInertial", "resAnalogmultienc", "resNONE"
};

const char * IethResource::stringOfType()
{
    iethresType_t t = this->type();

    if(iethres_none == t)
    {
        return names[iethresType_numberof];
    }

    return names[t];
}



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

size_t EthBoards::number_of_interfaces(AbstractEthResource * res)
{
    if(NULL == res)
    {
        return(0);
    }

    eOipv4addr_t ipv4 = res->getIPv4remoteAddress();
    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;


    if(index>=maxEthBoards)
    {
        return 0;
    }

    if(NULL == LUT[index].resource)
    {
        return 0;
    }

    return(LUT[index].numberofinterfaces);
}


bool EthBoards::add(AbstractEthResource* res)
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
    LUT[index].type = res->getBoardType();
    LUT[index].nameoftype = res->getBoardTypeString();
    LUT[index].boardnumber = index;
    snprintf(LUT[index].name, sizeof(LUT[index].name), "%s", res->getName());
    if(0 == strlen(LUT[index].name))
    {   // use default name
        snprintf(LUT[index].name, sizeof(LUT[index].name), "%s", defaultnames[index]);
    }
    LUT[index].numberofinterfaces = 0;
    for(int i=0; i<iethresType_numberof; i++)
    {
        LUT[index].interfaces[i] = NULL;
    }

    sizeofLUT++;

    return true;
}


bool EthBoards::add(AbstractEthResource* res, IethResource* interface)
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


bool EthBoards::rem(AbstractEthResource* res)
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


bool EthBoards::rem(AbstractEthResource* res, iethresType_t type)
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


AbstractEthResource* EthBoards::get_resource(eOipv4addr_t ipv4)
{
    AbstractEthResource * ret = NULL;

    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;
    if(index<maxEthBoards)
    {
        ret = LUT[index].resource;
    }

    return(ret);
}

bool EthBoards::get_LUTindex(eOipv4addr_t ipv4, uint8_t &index)
{
    index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;

    if(index>=maxEthBoards)
    {
        return false;
    }

    if(NULL == LUT[index].resource)
    {
        return false;
    }
    return true;
}

IethResource* EthBoards::get_interface(eOipv4addr_t ipv4, iethresType_t type)
{
    IethResource *dev = NULL;
    uint8_t index;

    if(!get_LUTindex(ipv4, index))
    {
        return NULL;
    }

    if(iethres_none == type)
    {
        return NULL;
    }

    dev = LUT[index].interfaces[type];

    return dev;

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



bool EthBoards::execute(void (*action)(AbstractEthResource* res, void* p), void* par)
{
    if(NULL == action)
    {
        return(false);
    }

    for(int i=0; i<maxEthBoards; i++)
    {
        AbstractEthResource* res = LUT[i].resource;
        if(NULL != res)
        {
            action(res, par);
        }

    }

    return(true);
}


bool EthBoards::execute(eOipv4addr_t ipv4, void (*action)(AbstractEthResource* res, void* p), void* par)
{
    if(NULL == action)
    {
        return(false);
    }

    AbstractEthResource* res = get_resource(ipv4);

    if(NULL == res)
    {
        return(false);
    }

    action(res, par);

    return(true);
}



// - class TheEthManager


TheEthManager::TheEthManager()
{
    // it is a singleton. the constructor is private.
    communicationIsInitted = false;
    UDP_socket  = NULL;

    // the container of ethernet boards: resources and attached interfaces
    ethBoards = new(EthBoards);

    // required by embobj system
    TheEthManager::initEOYsystem();

    // default address
    localIPaddress = ACE_INET_Addr("10.0.1.104:12345");
    ipv4local.addr = eo_common_ipv4addr(10, 0, 1, 104);
    ipv4local.port = 12345;

    // the time of creation according to yarp
    startUpTime = yarp::os::Time::now();
}



void delete_resources(AbstractEthResource *p, void* par)
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




void ethEvalTXropframe(AbstractEthResource *r, void* p)
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
        ACE_INET_Addr ipaddress = r->getRemoteAddress();
        ethman->sendPacket(data2send, (size_t)numofbytes, ipaddress);
    }
}


bool TheEthManager::Transmission(void)
{
    lockTX(true);

    ethBoards->execute(ethEvalTXropframe, this);

    lockTX(false);

    return true;
}


void ethEvalPresence(AbstractEthResource *r, void* p)
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

//bool TheEthManager::parseEthBoardInfo(yarp::os::Searchable &cfgtotal, ethFeature_t& ethbrdinfo)
//{
//    // Get PC104 address and port from config file
//    Bottle groupPC104  = Bottle(cfgtotal.findGroup("PC104"));
//    if (groupPC104.isNull())
//    {
//        yError() << "TheEthManager::parseEthBoardInfo() cannot find PC104 group in config files";
//        return false;
//    }
//    Value *PC104IpAddress_p;
//    if (!groupPC104.check("PC104IpAddress", PC104IpAddress_p))
//    {
//        yError() << "missing PC104IpAddress";
//        return false;
//    }
//    Bottle parameter1(groupPC104.find("PC104IpAddress").asString());
//    ACE_UINT16 port = groupPC104.find("PC104IpPort").asInt();

//    strcpy(ethbrdinfo.pc104IPaddr.string, parameter1.toString().c_str());
//    ethbrdinfo.pc104IPaddr.port = port;

//    // get ip adress and port for board from config file
//    Bottle groupETH_BOARD  = Bottle(cfgtotal.findGroup("ETH_BOARD"));
//    if (groupETH_BOARD.isNull())
//    {
//        yError() << "TheEthManager::parseEthBoardInfo() cannot find ETH_BOARD group in config files";
//        return false;
//    }
//    Bottle groupETH_BOARD_PROPERTIES  = Bottle(groupETH_BOARD.findGroup("ETH_BOARD_PROPERTIES"));
//    if (groupETH_BOARD_PROPERTIES.isNull())
//    {
//        yError() << "TheEthManager::parseEthBoardInfo() cannot find ETH_BOARD_PROPERTIES group in config files";
//        return false;
//    }
//    Bottle parameter2( groupETH_BOARD_PROPERTIES.find("IpAddress").asString() );
//    strcpy(ethbrdinfo.boardIPaddr.string, parameter2.toString().c_str());
//    ethbrdinfo.boardIPaddr.port = port;

//    sscanf(ethbrdinfo.boardIPaddr.string,"\"%d.%d.%d.%d", &ethbrdinfo.boardIPaddr.ip1, &ethbrdinfo.boardIPaddr.ip2, &ethbrdinfo.boardIPaddr.ip3, &ethbrdinfo.boardIPaddr.ip4);
//    sscanf(ethbrdinfo.pc104IPaddr.string,"\"%d.%d.%d.%d", &ethbrdinfo.pc104IPaddr.ip1, &ethbrdinfo.pc104IPaddr.ip2, &ethbrdinfo.pc104IPaddr.ip3, &ethbrdinfo.pc104IPaddr.ip4);

//    snprintf(ethbrdinfo.boardIPaddr.string, sizeof(ethbrdinfo.boardIPaddr.string), "%u.%u.%u.%u:%u", ethbrdinfo.boardIPaddr.ip1, ethbrdinfo.boardIPaddr.ip2, ethbrdinfo.boardIPaddr.ip3, ethbrdinfo.boardIPaddr.ip4, ethbrdinfo.boardIPaddr.port);
//    snprintf(ethbrdinfo.pc104IPaddr.string, sizeof(ethbrdinfo.pc104IPaddr.string), "%u.%u.%u.%u:%u", ethbrdinfo.pc104IPaddr.ip1, ethbrdinfo.pc104IPaddr.ip2, ethbrdinfo.pc104IPaddr.ip3, ethbrdinfo.pc104IPaddr.ip4, ethbrdinfo.pc104IPaddr.port);


////    snprintf(ethbrdinfo.boardName, sizeof(ethbrdinfo.boardName), "BRD-IP-%s", ethbrdinfo.boardIPaddr.string);
////    ethbrdinfo.boardNumber  = ethbrdinfo.boardIPaddr.ip4;

//    return true;
//}


bool TheEthManager::initCommunication(yarp::os::Searchable &cfgtotal)
{
    // we need: ip address of pc104, port used by socket, tx rate, rx rate.

    //ACE_INET_Addr ipaddress((u_short)12345, ((10 << 24) | (0 << 16) | (1 << 8) | (104)));            // it must be found ...
    int txrate = -1;                    // it uses default
    int rxrate = -1;                    // it uses default

    embBoardsConnected = true;

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

    ACE_UINT32 hostip = (ip1 << 24) | (ip2 << 16) | (ip3 << 8) | (ip4);
    ACE_INET_Addr myIP((u_short)port, hostip);

    myIP.addr_to_string(strIP, 64);

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
    if(false == createCommunicationObjects(myIP, txrate, rxrate) )
    {
        yError () << "TheEthManager::initCommunication() cannot create communication objects";
        return false;
    }

    // save local address

    localIPaddress = myIP;
    ipv4local.addr = eo_common_ipv4addr(ip1, ip2, ip3, ip4);
    ipv4local.port = port;

    return true;
}


AbstractEthResource *TheEthManager::requestResource2(IethResource *interface, yarp::os::Searchable &cfgtotal)
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
//    Bottle paramNameBoard(groupEthBoardParams.find("Name").asString());
//    char boardname[64] = {0};
//    strcpy(boardname, paramNameBoard.toString().c_str());


    // i want to lock the use of resources managed by ethBoards to avoid that we attempt to use for TX a ethres not completely initted

    lockTXRX(true);

    // i do an attempt to get the resource.
    AbstractEthResource *rr = ethBoards->get_resource(ipv4addr);

    if(NULL == rr)
    {
        // i dont have the resource yet ...

        char ipinfo[20] = {0};
        eo_common_ipv4addr_to_string(ipv4addr, ipinfo, sizeof(ipinfo));

        if(embBoardsConnected)
            rr = new EthResource();
        else
            rr = new FakeEthResource();

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



int TheEthManager::releaseResource2(AbstractEthResource* ethresource, IethResource* interface)
{
    int ret = 1; // -1 means that the singleton is not needed anymore. 0 means error
    if((NULL == ethresource) || (NULL == interface))
    {
        yError() << "TheEthManager::releaseResource2(): there is an attempt to release a NULL EthResource or IethResource";
        return 0;
    }


    AbstractEthResource* rr = ethresource;

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


//EthSender* TheEthManager::getEthSender(void)
//{
//    return sender;
//}


//EthReceiver* TheEthManager::getEthReceiver(void)
//{
//    return receiver;
//}


double TheEthManager::getLifeTime(void)
{
    return(yarp::os::Time::now() - startUpTime);
}




bool TheEthManager::createCommunicationObjects(ACE_INET_Addr localaddress, int txrate, int rxrate)
{
    lock(true);


    if(!communicationIsInitted)
    {
        UDP_socket = new ACE_SOCK_Dgram();
        if((embBoardsConnected)  && (-1 == UDP_socket->open(localaddress)))
        {
            char tmp[64] = {0};
            localaddress.addr_to_string(tmp, 64);
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



int TheEthManager::sendPacket(void *udpframe, size_t len, ACE_INET_Addr toaddress)
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

    AbstractEthResource* r = ethBoards->get_resource(ipv4addr);

    if((NULL != r) && (!r->isFake()))
    {
        r->Tick();

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


//EthResource* TheEthManager::IPtoResource(ACE_INET_Addr adr)
//{
//    ACE_UINT32 a32 = adr.get_ip_address();
//    uint8_t ip4 = a32 & 0xff;
//    uint8_t ip3 = (a32 >> 8) & 0xff;
//    uint8_t ip2 = (a32 >> 16) & 0xff;
//    uint8_t ip1 = (a32 >> 24) & 0xff;

//    eOipv4addr_t ipv4addr = eo_common_ipv4addr(ip1, ip2, ip3, ip4);
//    return(ethBoards->get_resource(ipv4addr));
//}


//int TheEthManager::IPtoBoardNumber(ACE_INET_Addr adr)
//{
//    ACE_UINT32 a32 = adr.get_ip_address();
//    uint8_t ip4 = a32 & 0xff;
//    return(ip4);
//}


int TheEthManager::getNumberOfResources(void)
{
    return(ethBoards->number_of_resources());
}


const char * TheEthManager::getName(eOipv4addr_t ipv4)
{
    const char * ret = ethBoards->name(ipv4);
    return ret;
}


AbstractEthResource* TheEthManager::getEthResource(eOipv4addr_t ipv4)
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

#if defined(__unix__)
    /**
     * Make it realtime (works on both RT and Standard linux kernels)
     * - increase the priority upto the system IRQ's priorities (50) and less than the receiver thread
     * - set the scheduler to FIFO
     */
    struct sched_param thread_param;
    thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO)/2 - 1; // = 48
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);
#endif

    return true;
}



void EthSender::run()
{
    // by calling this metod of ethManager, we put protection vs concurrency internal to the class.
    // for tx we must protect the EthResource not being changed. they can be changed by a device such as
    // embObjMotionControl etc which adds or releases its resources.
    ethManager->Transmission();
}




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
    ethManager->sendPacket( &tmp, 1, ethManager->getLocalIPaddress());
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

    return true;
}


bool EthReceiver::threadInit()
{
    yTrace() << "Do some initialization here if needed";

#if defined(__unix__)
    /**
     * Make it realtime (works on both RT and Standard linux kernels)
     * - increase the priority upto the system IRQ's priorities (< 50)
     * - set the scheduler to FIFO
     */
    struct sched_param thread_param;
    thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO)/2; // = 49
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);
#endif

    return true;
}


uint64_t getRopFrameAge(char *pck)
{
    return(eo_ropframedata_age_Get((EOropframeData*)pck));
}



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

    static uint8_t earlyexit_prev = 0;
    static uint8_t earlyexit_prevprev = 0;

    // marco.accame: set maxUDPpackets as a fixed minimum number (2) + the number of boards. all is multipled by rateofthread and by a gain which depends on past activity
    // the gain on past activity is usually 1. if maxUDPpackets is not enough the gain becomes (1+f1). if again it is not enough, the gain becomes 1+f1+f2+f3 and stays as
    // such until it is enough. at this time it becomes 1+f2 and then 1 again
    const double f1 = 0.5;
    const double f2 = 0.5;
    const double f3 = 8.0;
    double gain = 1.0 + f1*(1-earlyexit_prev) + f2*(1-earlyexit_prevprev) + f3*(1-earlyexit_prev)*(1-earlyexit_prevprev);
    int maxUDPpackets = (2 + ethManager->getNumberOfResources()) * EthReceiver::rateofthread *  gain;


    earlyexit_prevprev = earlyexit_prev;    // save previous early exit
    earlyexit_prev = 0;                     // consider no early exit this time

    for(int i=0; i<maxUDPpackets; i++)
    {
        incoming_msg_size = recv_socket->recv((void *) incoming_msg_data, incoming_msg_capacity, sender_addr, flags);
        if(incoming_msg_size <= 0)
        { // marco.accame: i prefer using <= 0.
            earlyexit_prev = 1; // yes, we have an early exit
            break; // we break and do not return because we want to be sure to execute what is after the for() loop
        }

        // we have a packet ... we give it to the ethmanager for it parsing
        bool collectStatistics = (statPrintInterval > 0) ? true : false;
        ethManager->Reception(sender_addr, incoming_msg_data, incoming_msg_size, collectStatistics);
    }

    // execute the check on presence of all eth boards.
    ethManager->CheckPresence();
}



// eof
