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
#include <yarp/os/impl/PlatformTime.h>
#include <errno.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;

TheEthManager* TheEthManager::handle = NULL;
yarp::os::Semaphore TheEthManager::managerMutex = 1;


#if 0
ethResources* TheEthManager::getResource(yarp::os::Searchable &config)
{
    std::string   str;
    Bottle        xtmp2;
    ACE_TCHAR     remote_address[64];
    ACE_TCHAR     address_tmp_string[64];
    ACE_UINT16    rem_port;
    ACE_UINT32    rem_ip1,rem_ip2,rem_ip3,rem_ip4;

    if(config.findGroup("GENERAL").find("Verbose").asInt())
        {str=config.toString().c_str();}
    else
        {str=" ";}
    yTrace() << str;


    /* Get a Socket for the PC104 This needs a reading from config file, at least for the port to use...
    * If the socket is already initialized the createSocket function does nothing, it is also thread safe  */
    ACE_UINT32 loc_ip1,loc_ip2,loc_ip3,loc_ip4;
    xtmp2 = config.findGroup("PC104IpAddress");
    strcpy(address_tmp_string, xtmp2.get(1).asString().c_str());

    // ACE format
    sscanf(address_tmp_string,"%d.%d.%d.%d",&loc_ip1, &loc_ip2, &loc_ip3, &loc_ip4);
    ACE_INET_Addr loc_dev(rem_port, (loc_ip1<<24)|(loc_ip2<<16)|(loc_ip3<<8)|loc_ip4);

    if(!createSocket(loc_dev))
        {return false;}


    //
    // Get EMS ip addresses from config file, to see if we need to instantiate a new Resources or simply return
    // a pointer to an already existing object
    //

    Bottle xtmp = Bottle(config.findGroup("ETH"));
    xtmp2 = xtmp.findGroup("IpAddress");
    strcpy(remote_address, xtmp2.get(1).asString().c_str());
    sscanf(remote_address,"%d.%d.%d.%d",&rem_ip1, &rem_ip2, &rem_ip3, &rem_ip4);

    // Get EMS CmdPort from config file
    xtmp2 = xtmp.findGroup("CmdPort");
    rem_port = xtmp2.get(1).asInt();

    // ACE format
    ACE_INET_Addr remote_addr_tmp;
    remote_addr_tmp.set(rem_port, (rem_ip1<<24)|(rem_ip2<<16)|(rem_ip3<<8)|rem_ip4);

    ethResources *newRes = NULL;

    managerMutex.wait();
    ethResIt iterator = EMS_list.begin();

    while(iterator != EMS_list.end())
    {
        if((*iterator)->getRemoteAddress() == remote_addr_tmp)
        {
            // device already exist.
            newRes = (*iterator);
            break;
        }
        iterator++;
    }

    if(NULL == newRes)
    {
        // device doesn't exist yet, create it
        yDebug() << "Creating EMS device with IP " << remote_address;
        newRes = new ethResources;
        if(!newRes->open(config))
        {
            printf("Error creating new EMS!!");
            if(NULL != newRes)
                delete newRes;

            newRes = NULL;
        }
        else
        {
            nBoards++;
            EMS_list.push_back(newRes);
        }
    }
    managerMutex.post();
    return newRes;
}

bool TheEthManager::removeResource(ethResources* to_be_removed)
{
    managerMutex.wait();
    to_be_removed->close();
    if(EMS_list.size() != 0)
    {
        ethResIt iterator = EMS_list.begin();
        while(iterator != EMS_list.end())
        {
            if((*iterator)->getRemoteAddress() == to_be_removed->getRemoteAddress())
            {
                delete (*iterator);
                EMS_list.remove(*iterator);
                managerMutex.post();
                return true;
            }
            iterator++;
        }
        yError() << "EthManager: Asked to remove an entry but it was not found!\n";
    }
    else
        yError() << "EthManager: Asked to remove an entry in an empty list!\n";

    managerMutex.post();
    return false;
}
#endif


// -------------------------------------------------------------------\\
//            TheEthManager   Singleton
// -------------------------------------------------------------------\\

ethResources *TheEthManager::requestResource(FEAT_ID *request)
{
    yTrace() << request->boardNum;
    // Check if local socket is initted, if not do it.
    ACE_TCHAR       address[64];
    sprintf(address, "%s:%d", request->PC104ipAddr.string, request->PC104ipAddr.port);

    //ACE_INET_Addr myIP(address, AF_INET);

    ACE_UINT32 hostip = (request->PC104ipAddr.ip1 << 24) | (request->PC104ipAddr.ip2 << 16) | (request->PC104ipAddr.ip3 << 8) | (request->PC104ipAddr.ip4);
    ACE_INET_Addr myIP((u_short)request->PC104ipAddr.port, hostip);
    myIP.dump();
    char tmp_addr[64];

    if(!createSocket(myIP) )
    {  return NULL;  }

      ethResources *newRes = NULL;

    // Grab the mutex
    managerMutex.wait();

    int justCreated = false;
    ACE_INET_Addr tmpIp;
    ethResIt iterator = EMS_list.begin();
    while(iterator != EMS_list.end())
    {
        (*iterator)->getRemoteAddress().addr_to_string(tmp_addr, 64);
       // tmpIp->addr_to_string(tmp_addr, 64);
        //(*iterator)->getRemoteAddress().addr_to_string(tmp_addr, 64);
        if( strcmp(tmp_addr, request->EMSipAddr.string) == 0)
        {
            // device already exists, return the pointer.
            newRes = (*iterator);
            break;
        }
        iterator++;
    }

    managerMutex.post();

    if(NULL == newRes)
    {
        // device doesn't exist yet, create it
        yDebug() << "Creating EMS device with IP " << request->EMSipAddr.string;
        newRes = new ethResources;
        if(!newRes->open(*request))
        {
            yError() << "Error creating new EMS!!";
            if(NULL != newRes)
                delete newRes;

            newRes = NULL;
//            managerMutex.post(); //?
            return NULL;
        }
        justCreated = true;
    }

    // If this point is reached everything must be ok... i.e. newRes != NULL.
    managerMutex.wait();

    // the push_back has to be done only once for EMS
    if(justCreated)
        EMS_list.push_back(newRes);

    // The registerFeature has to be done always
    newRes->registerFeature(request);
    addLUTelement(request);

    managerMutex.post();
    return newRes;
}



int TheEthManager::releaseResource(FEAT_ID resource)
{
    yTrace() << resource.boardNum;
    int           ret           = 0;
    int           stillUsed     = 0;
    ethResources  *res2release  = NULL;
    char tmp_addr[64];

    ethResources *tmpEthRes;
    ACE_INET_Addr  tmp_ace_addr;

    managerMutex.wait();

    removeLUTelement(resource);

    ethResIt iterator = EMS_list.begin();
    while(iterator != EMS_list.end())
    {
        tmpEthRes = (*iterator);
        tmp_ace_addr = tmpEthRes->getRemoteAddress();
        tmp_ace_addr.addr_to_string(tmp_addr, 64);
        if( strcmp(tmp_addr, resource.EMSipAddr.string) == 0)
        {
            // device exists
            res2release = (*iterator);
            stillUsed = res2release->deregisterFeature(resource);
            if( !! stillUsed)
            {
                ret = 1;
                break;
            }
            else
            {
                res2release->close();
                EMS_list.remove(res2release);
                delete res2release;
                ret = 1;
                break;
            }
        }
        iterator++;
    }

    if(EMS_list.size() == 0 )
        yError() << "EMS_list.size() = 0";

    if(boards_map.size() == 0 )
        yError() << "boards_map.size = 0";

    if(     (EMS_list.size() == 0 ) && (boards_map.size() != 0 ) 
        ||  (EMS_list.size() != 0 ) && (boards_map.size() == 0 ) )
    {
        yError() << "Something strange happened... EMS_list.size is" << EMS_list.size() << "while boards_map.size is "<< boards_map.size();
    }

    if(!ret)
        yError() << "EthManager: Trying to release a non existing resource" << resource.name << " for boardId " << resource.boardNum << "maybe already deleted?";

    if( (EMS_list.size() == 0 ) && (boards_map.size() == 0 ) )
        ret = -1;

    managerMutex.post();
    return ret;
}


void TheEthManager::addLUTelement(FEAT_ID *id)
{
    yTrace() << id->boardNum;
    /* NO MUTEX HERE because it's a PRIVATE method, so called only inside other already mutexed methods */
    // Defining the var addLUT_result to have the true/false result of the insert operation...
    // it is unlikely to fail, so keep it or not?
    //std::pair<_Rb_tree_iterator <std::pair <const eOnvEP_t, FEAT_ID > >, bool > addLUT_result;
    //addLUT_result =
    boards_map.insert(std::pair<uint8_t, FEAT_ID>(id->ep, *id)).second;
    // Check result of insertion
    //addLUT_result.second ? yWarning() << "ok add lut element" : yError() << "NON ok add lut element";
}

bool TheEthManager::removeLUTelement(FEAT_ID element)
{
    yTrace() << element.boardNum;
    /* NO MUTEX HERE because it's a PRIVATE method, so called only inside other already mutexed methods */
    bool ret;
    int n = (int) boards_map .erase(element.ep);

    switch(n)
    {
        case 0:
        {
            yError() << "Trying to remove a non-existing element";
            ret = false;
            break;
        }

        case 1:
        {
            yDebug() << "FEAT_ID element removed succesfully from the map";
            ret = true;
            break;
        }
        default:
        {
            yError() << "More than one element were removed with key board num " << element.boardNum << "ep " << element.ep;
            ret = true;
            break;
        }
    }
}

void *TheEthManager::getHandleFromEP(eOnvEP_t ep)
{
    yTrace();
//     managerMutex.wait();
    void * ret = boards_map[ep].handle;
//     managerMutex.post();
    return ret;
}

FEAT_ID TheEthManager::getFeatInfoFromEP(uint8_t ep)
{
    yTrace();
    FEAT_ID ret_val;
//     managerMutex.wait();  // il thread che chiama questa funz ha già preso questo mutex in ethReceiver::run
    ret_val = boards_map[ep];
//     managerMutex.post();
    return ret_val;
}


TheEthManager::TheEthManager()
{
    yTrace();
    memset(info, 0x00, SIZE_INFO);
    sprintf(info, "TheEthManager");

    UDP_initted = false;
    UDP_socket  = NULL;
}

bool TheEthManager::createSocket(ACE_INET_Addr local_addr)
{
    yTrace();
    managerMutex.wait();

//     if(NULL == handle)
//     {
//         yError() << "Called createSocket while EthManager is not instantiated";
//         handle = TheEthManager::instance();
//     }

    if(!UDP_initted)
    {
        UDP_socket = new ACE_SOCK_Dgram();
        char tmp[64];
        if(-1 == UDP_socket->open(local_addr))
        {
            local_addr.addr_to_string(tmp, 64);
            yError() <<   "\n/---------------------------------------------------\\"
                     <<   "\n| Unable to bind to local IP address " << tmp
                     <<   "\n\\---------------------------------------------------/";
            delete UDP_socket;
            UDP_socket = NULL;
            UDP_initted = false;
        }
        else
        {
            UDP_initted = true;

            sender = new EthSender();
            receiver = new EthReceiver();

            //managerMutex.post();

            sender->config(UDP_socket, this);
            receiver->config(UDP_socket, this);
            /* Start the threads sending to and receiving messages from the boards.
             * It will execute the threadInit and pass its return value to the following calls
             * afterStart to check if they started correctly.
             */
            bool ret1, ret2;
            ret1 = sender->start();
            ret2 = receiver->start();

            //managerMutex.wait();
            if(!ret1 || !ret2)
            {
                yError() << "EthManager: issue while starting threads for UDP communication with EMSs";

                // stop kills and waits.
                if(sender->isRunning() )
                    sender->stop();

                if(receiver->isRunning() )
                    receiver->stop();

                delete UDP_socket;
                UDP_initted = false;
                return false;
            }
            else
            {
                yTrace() << "Both sending and Receiving thread started correctly!";

            }
        }
    }

    managerMutex.post();
    return UDP_initted;
}

bool TheEthManager::isInitted(void)
{
    yTrace();
    bool ret;
    managerMutex.wait();
    ret = UDP_initted;
    managerMutex.post();
    return ret;
}

TheEthManager *TheEthManager::instance()
{
    yTrace();
    managerMutex.wait();
    if (NULL == handle)
    {
        yTrace() << "Calling EthManager Constructor";
        handle = new TheEthManager();
        if (NULL == handle)
            yError() << "While calling EthManager constructor";
        else
            initCallback((void*)handle);
    }
    managerMutex.post();

    return handle;
}

bool TheEthManager::killYourself()
{
    yTrace();
    //TODO harakiri
    delete handle;
}

TheEthManager::~TheEthManager()
{
    yTrace();
    int timeout = 5;

    // Stop method also make a join waiting the thread to exit
    sender->stop();
    receiver->stop();

        // Close UDP socket
    if(isInitted())
    {
        UDP_socket->close();
        delete UDP_socket;
        UDP_initted = false;
    }

    managerMutex.wait();
    // Destroy all EMS boards
    if(EMS_list.size() != 0)
    {
        ethResIt iterator = EMS_list.begin();

        while(iterator != EMS_list.end())
        {
            delete(*iterator);
            iterator++;
        }
    }
    managerMutex.post();
}

int TheEthManager::send(void *data, size_t len, ACE_INET_Addr remote_addr)
{
    ssize_t ret = UDP_socket->send(data,len,remote_addr);
    return ret;
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
//     nBoards--;
//     yTrace();
//     if(0 == nBoards)
//     {
//         keepGoingOn = false;
//         Time::delay(2);
//         delete handle;
//     }

    return true;
}

EthSender::EthSender() : RateThread(1)
{
    yTrace();
}

bool EthSender::config(ACE_SOCK_Dgram *pSocket, TheEthManager* _ethManager)
{
    yTrace();
    send_socket = pSocket;
    ethManager  = _ethManager;
    ethResList  = &(_ethManager->EMS_list);
}

bool EthSender::threadInit()
{
    yTrace() << "Do some initialization here if needed";
    return true;
}

void EthSender::run()
{
    ethResources  *ethRes;
    ACE_TCHAR     address_tmp[128];
    uint16_t      bytes_to_send = 0;
    ethResIt      iterator;

    ethManager->managerMutex.wait();
    for(iterator = ethResList->begin(); iterator != ethResList->end() && (isRunning()); iterator++)
    {
        p_sendData = NULL;
        bytes_to_send = 0;
        if(NULL == *iterator)
        {
            yError() << "EthManager::run, iterator==NULL";
            continue;
        }

        ethRes = (*iterator);

        // This uses directly the pointer of the transceiver
        ethRes->getPointer2TxPack(&p_sendData, &bytes_to_send);
        if((bytes_to_send > EMPTY_PACKET_SIZE) && (NULL != p_sendData))
        {
            ACE_INET_Addr addr = ethRes->getRemoteAddress();
            int ret = ethManager->send(p_sendData, (size_t)bytes_to_send, addr);
        }

        // This will copy the data from the transceiver into private memory
        /* ethRes->getTxPack(&sendBuffer, &bytes_to_send);
        if(bytes_to_send > EMPTY_PACKET_SIZE)
        {
            ACE_INET_Addr addr = ethRes->getRemoteAddress();
            int ret = TheEthManager::instance()->send(sendBuffer, (size_t)bytes_to_send, addr);
        }
        */
    }
    ethManager->managerMutex.post();
}

EthReceiver::EthReceiver()
{
    yTrace();
}

void EthReceiver::onStop()
{
    uint8_t tmp;
    ethManager->send( &tmp, 1, ethManager->local_addr);
};

EthReceiver::~EthReceiver()
{
    yTrace();
}

bool EthReceiver::config(ACE_SOCK_Dgram *pSocket, TheEthManager* _ethManager)
{
    yTrace();
    recv_socket = pSocket;
    ethManager  = _ethManager;
    ethResList  = &(_ethManager->EMS_list);
}


bool EthReceiver::threadInit()
{
    yTrace() << "Do some initialization here if needed";
    return true;
}

void EthReceiver::run()
{
    yTrace();
    ACE_TCHAR     address[64];
    ethResources  *ethRes;
    ssize_t       recv_size;
    ACE_INET_Addr sender_addr;
    char          incoming_msg[RECV_BUFFER_SIZE];

    ethResIt iteratoreLista, fineLista;

//     while( NULL == recv_socket )
//     {
//         yError() << "socket non inizializzato... come è possibile???";
//         Time::delay(1);
//     }

    yTrace() << "Starting udp RECV thread\n";

    ACE_Time_Value recvTimeOut;
    fromDouble(recvTimeOut, 1.0);

    //while(isRunning())
    while(!isStopping())
    {
#warning
        // per ogni msg ricevuto  -1 visto come 65535!!
        recv_size = recv_socket->recv((void *) incoming_msg, RECV_BUFFER_SIZE, sender_addr, 0, &recvTimeOut);

        sender_addr.addr_to_string(address, 64);
//     printf("Received new packet from address %s, size = %d\n", address, recv_size);

        if( (recv_size > 0) && (isRunning()) )
        {
            ethManager->managerMutex.wait();
            iteratoreLista = ethManager->EMS_list.begin();
            while(iteratoreLista != ethManager->EMS_list.end())
            {
                if( (*iteratoreLista)->getRemoteAddress() == sender_addr)
                {
                    ethRes = (*iteratoreLista);
                    if(ethRes == NULL)      // possibile questo caso?? TODO verificare
                    {
                        yError() << "trying to access a non initted ethres";
                        iteratoreLista++;
                        break;  //?
                    }

                    if(recv_size > ethRes->getBufferSize() || (recv_size <= 0) )
                    {
                        yError() << "EthReceiver got a message of wrong size ( received" << recv_size << " bytes while buffer is" << ethRes->getBufferSize() << " bytes long)";
                    }
                    else
                    {
                        memcpy(ethRes->recv_msg, incoming_msg, recv_size);
                        ethRes->onMsgReception(ethRes->recv_msg, recv_size);
                    }
                    break;  //? devo uscire da questo while e rimanere in quello più esterno.
                }

                iteratoreLista++;
            }
            ethManager->managerMutex.post();
        }
        else if(errno == ETIME)
        {
            yWarning() << "No message received from the EMS boards for " << recvTimeOut.sec() << "sec";
        }
        else
        {
            yWarning() << "Received weird msg of size" << recv_size;
        }
    }
    yTrace() << "Exiting recv thread";
    return;
}
