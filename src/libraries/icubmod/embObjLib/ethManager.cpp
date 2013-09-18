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

#include <eODeb_eoProtoParser.h>
#include <eODeb_eoProtoParser_hid.h>
#include <eOtheEthLowLevelParser.h>

#include <stdexcept>      // std::out_of_range

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

    if(false == emsAlreadyClosed)
    {
		ethResIt it = EMS_list.begin();
		while(it != EMS_list.end())
		{
			tmpEthRes = (*it);
			tmpEthRes->goToConfig();
			tmpEthRes->clearPerSigMsg();
			it++;
		}
		//before stopping threads, flush all pkts not yet sent.
		flush();
		emsAlreadyClosed = true;
    }
    stopThreads();
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

    if( (EMS_list.size() == 0 ) || (boards_map.size() == 0 ) )
        ret = -1;

    managerMutex.post();
    return ret;
}


void TheEthManager::addLUTelement(FEAT_ID *id)
{
    yTrace() << id->boardNum;
    /* NO MUTEX HERE because it's a PRIVATE method, so called only inside other already mutexed methods */
    /* Defined the var addLUT_result to have the true/false result of the insert operation...It helped catching a bug.
     * It fails if the element is already present. This can happen if someone tries to read an element with
     * the '[]' operator before the insert, because the std::map will create it automatically (hopefully initted
     * with zeros.
     */
     bool addLUT_result =  boards_map.insert(std::pair<eOnvEP_t, FEAT_ID>(id->ep, *id)).second;

    // Check result of insertion
    addLUT_result ? yDebug() << "ok add lut element for board " << id->boardNum << " and ep " << id->ep :
                    yError() << "NON ok add lut element for board " << id->boardNum << " and ep " << id->ep;
}

bool TheEthManager::removeLUTelement(FEAT_ID element)
{
    yTrace() << element.boardNum;
    /* NO MUTEX HERE because it's a PRIVATE method, so called only inside other already mutexed methods */
    bool ret = false;
    int n = (int) boards_map.erase(element.ep);

    switch(n)
    {
        case 0:
        {
            yError() << "Trying to remove a non-existing element" << element.name << "maybe already removed?";
            ret = false;
            break;
        }

        case 1:
        {
            yDebug() << "FEAT_ID element removed succesfully from the map" << element.name;
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

    return ret;
}

void *TheEthManager::getHandleFromEP(eOnvEP_t ep)
{
//     managerMutex.wait();
    void * ret = NULL;
    static int error = 0;

    try
    {
        // USE .at AND NOT the '[ ]' alternative!!! It will create a bug!!!
        /* The bug is caused by the fact that the [] version will create an unitialized element inside the map,
         * causing the return of a wrong pointer.
         * Furthermore the insert method used to correctly initialze the element will fail because a (wrong)
         * element is already present preventing the map to be corrected.
         */
        ret = boards_map.at(ep).handle;
    }
    catch (const std::out_of_range& error)
    {
//        if(0 == (error%1000) )
            yError() << "Got a message from EP " << ep << "but boards_map does not contains any related class pointer yet";

//        error++;
    }
//     managerMutex.post();
    return ret;
}

FEAT_ID TheEthManager::getFeatInfoFromEP(eOnvEP_t ep)
{
//    yTrace();
    FEAT_ID ret_val;
//     managerMutex.wait();  // il thread che chiama questa funz ha già preso questo mutex in ethReceiver::run ... // nn più vero dopo le ultime ottimizzazioni
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
    emsAlreadyClosed = false;
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

                // stop threads
                stopThreads();

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

    return true;
}

bool TheEthManager::stopThreads()
{
    bool ret = true;
    // Stop method also make a join waiting the thread to exit
    if(sender->isRunning() )
        sender->stop();
    if(receiver->isRunning() )
        ret = receiver->stop();

    return ret;
}

TheEthManager::~TheEthManager()
{
    yTrace();
    int timeout = 5;

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

    if(boards_map.size() != 0)
    {
        std::map<eOnvEP_t, FEAT_ID>::iterator mIt;
        for(mIt = boards_map.begin(); mIt!=boards_map.end(); mIt++)
        {
            yError() << "Feature " << mIt->second.name << "was not correctly removed from map.., removing it now in the EthManager destructor.";
            removeLUTelement(mIt->second);
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


void TheEthManager::flush()
{
    // #warning remove sleep asap!!!!!!
        //here sleep is essential in order to let sender thread send gotocongig command.
        yarp::os::Time::delay(1); // EO_WARNING()

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

    return true;
}

bool EthSender::threadInit()
{
    yTrace() << "Do some initialization here if needed";
    return true;
}

void EthSender::run()
{
    ethResources  *ethRes;
//    ACE_TCHAR     address_tmp[128];
    uint16_t      bytes_to_send = 0;
    ethResIt      iterator;
    ethResRIt    riterator, _rBegin, _rEnd;

    /*
        Usare un revers iterator per scorrere la lista dalla fine verso l'inizio. Questo aiuta a poter scorrere
        la lista con un iteratore anche durante la fase iniziale in cui vengono ancora aggiunti degli elementi,
        in teoria senza crashare. Al più salvarsi il puntatore alla rbegin sotto mutex prima di iniziare il ciclo,
        giusto per evitare che venga aggiunto un elemento in concomitanza con la lettura dell rbegin stesso.
        Siccome gli elementi vengono aggiunti solamente in coda alla lista, questa iterazione a ritroso non
        dovrebbe avere altri problemi e quindi safe anche senza ilmutex che prende TUTTO il ciclo.

      std::list<int> mylist;
    for (int i=1; i<=5; ++i) mylist.push_back(i);

    std::cout << "mylist backwards:";
    for (std::list<int>::reverse_iterator rit=mylist.rbegin(); rit!=mylist.rend(); ++rit)
    std::cout << ' ' << *rit;
  */

    ethManager->managerMutex.wait();
    //for(iterator = ethResList->begin(); iterator != ethResList->end() && (isRunning()); iterator++)
    _rBegin = ethResList->rbegin();
    _rEnd = ethResList->rend();
    ethManager->managerMutex.post();

    for(riterator = _rBegin; riterator != _rEnd && (isRunning()); riterator++)
    {
        p_sendData = NULL;
        bytes_to_send = 0;
        if(NULL == *riterator)
        {
            yError() << "EthManager::run, iterator==NULL";
            continue;
        }

        ethRes = (*riterator);

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
    //ethManager->managerMutex.post();
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

    return true;
}


bool EthReceiver::threadInit()
{
#define MSG020979 "WARNING-> could not find extern eODeb_eoProtoParser_cfg_t *deb_eoParserCfg_ptr thus i removed it. thus FIX IT !!"
#if defined(_MSC_VER)
    #pragma message(MSG020979)
#else
    #warning MSG020979
#endif
    //extern eODeb_eoProtoParser_cfg_t *deb_eoParserCfg_ptr;
    eODeb_eoProtoParser_cfg_t *deb_eoParserCfg_ptr = NULL;
    eODeb_eoProtoParser_Initialise(deb_eoParserCfg_ptr);
    yTrace() << "Do some initialization here if needed";
    return true;
}



typedef struct  // 24 bytes
{
    uint32_t            startofframe;
    uint16_t            ropssizeof;
    uint16_t            ropsnumberof;
    uint64_t            ageofframe;
    uint64_t            sequencenumber;
} EOropframeHeader_TEST_t;

uint64_t getRopFrameAge(char *pck)
{
    EOropframeHeader_TEST_t *test = (EOropframeHeader_TEST_t *) pck;
    return test->ageofframe;
}

class times_test_delay
{
public:
    times_test_delay();

    bool          initted;
    double        prevRecvMsg_PC104;
    double        newRecvMsg_PC104;
    double        gap_EMS;
    uint64_t      ageofframe_EMS;
    double        gap_PC104;
    bool          error_PC104;
    bool          error_EMS;
};

times_test_delay::times_test_delay()
{
    initted = false;
    prevRecvMsg_PC104 = 0;
    newRecvMsg_PC104 = 0;
    gap_EMS = 0;
    ageofframe_EMS =0;
    gap_PC104 = 0;
    error_PC104 = false;
    error_EMS = false;
}

void EthReceiver::run()
{
    yTrace();
    ACE_TCHAR     address[64];
    ethResources  *ethRes;
    ssize_t       recv_size;
    ACE_INET_Addr sender_addr;
    char          incoming_msg[RECV_BUFFER_SIZE];

    // TODO aggiunto per debugging. tenere tempo trascorso dall'ultima ricezione di un msg da parte della ems i-esima.
    std::map<int, times_test_delay> lastHeard;

    yError() << "Starting udp RECV thread\n";

    ACE_Time_Value recvTimeOut;
    fromDouble(recvTimeOut, 0.01);
    double myTestTimeout = recvTimeOut.sec() + (double)recvTimeOut.msec()/1000.0f;

    eODeb_eoProtoParser *PP =  eODeb_eoProtoParser_GetHandle();
    eOethLowLevParser_packetInfo_t pckInfo;

    while(!isStopping())
    {
        ethManager->managerMutex.wait();
        // new, reverse iterator
        ethResRIt    riterator, _rBegin, _rEnd;
        _rBegin = ethResList->rbegin();
        _rEnd = ethResList->rend();
        ethManager->managerMutex.post();

        // per ogni msg ricevuto  -1 visto come 65535!!
        recv_size = recv_socket->recv((void *) incoming_msg, RECV_BUFFER_SIZE, sender_addr, 0, &recvTimeOut);

        riterator = _rBegin;
        while(riterator != _rEnd)
        {
            ethRes = (*riterator);
            if(ethRes->isRunning() /*&& (ethRes->getLastRecvMsgTimestamp()>0)*/ && (lastHeard[ethRes->boardNum].initted))
            {
				if(yarp::os::Time::now() - ethRes->getLastRecvMsgTimestamp() > myTestTimeout)
				{
					if(!lastHeard[ethRes->boardNum].error_PC104)
					{
						yError() << "Board " << ethRes->boardNum << ": more than " << myTestTimeout *1000 << "ms are passed without any news LAST=" << ethRes->getLastRecvMsgTimestamp() ;
						lastHeard[ethRes->boardNum].error_PC104 = true;
					}
				}
            }
            else
            {
            	lastHeard[ethRes->boardNum].initted = false;
            }
            riterator++;
        }

        if( recv_size > 60000)
        {
            yWarning() << "Huge message received " << recv_size;
        }

        sender_addr.addr_to_string(address, 64);

        if( (recv_size > 0) && (isRunning()) )
        {
            pckInfo.payload_ptr = (uint8_t *) incoming_msg;
            pckInfo.prototype = protoType_udp;
            pckInfo.dst_port = sender_addr.get_port_number();
            pckInfo.src_port = sender_addr.get_port_number();
            pckInfo.src_addr = 1;
            pckInfo.dst_addr = 2;

            ethManager->managerMutex.wait();
            eODeb_eoProtoParser_RopFrameDissect(PP, &pckInfo);
            ethManager->managerMutex.post();

            riterator = _rBegin;

            while(riterator != _rEnd)
            {
                ethRes = (*riterator);
                if(ethRes->getRemoteAddress() == sender_addr)
                {
                    if(recv_size > ethRes->getBufferSize())
                    {
                        yError() << "EthReceiver got a message of wrong size ( received" << recv_size << " bytes while buffer is" << ethRes->getBufferSize() << " bytes long)";
                    }
                    else
                    {
//                         lastHeard[ethRes->boardNum].prevRecvMsg_PC104 = ethRes->getLastRecvMsgTimestamp();
                        memcpy(ethRes->recv_msg, incoming_msg, recv_size);
                        ethRes->onMsgReception(ethRes->recv_msg, recv_size);

                        if(lastHeard[ethRes->boardNum].initted)
                        {
                            //check recvTime on the PC104 side
                            lastHeard[ethRes->boardNum].gap_PC104 = ethRes->getLastRecvMsgTimestamp() - lastHeard[ethRes->boardNum].prevRecvMsg_PC104;

                            if(lastHeard[ethRes->boardNum].gap_PC104 > myTestTimeout)
                            {
                                yError() << "Board " << ethRes->boardNum << ": Gap of " << lastHeard[ethRes->boardNum].gap_PC104*1000 << "ms between two consecutive messages !!!";
                            }

                            // check time written into packet
                            int diff = (int)(getRopFrameAge(incoming_msg)/1000 - lastHeard[ethRes->boardNum].ageofframe_EMS/1000);
                            if( diff > (int)(myTestTimeout * 1000))
                            {
                                yError() << "Board " << ethRes->boardNum << ": EMS time between 2 ropframes bigger then " << myTestTimeout * 1000 << "ms;\t Actual delay is" << diff << "ms.";
                            }

                            //reset errors
                            lastHeard[ethRes->boardNum].error_PC104 = false;

                        }
                        else
                        {
                        	if(ethRes->isRunning())
                        	{
                        		lastHeard[ethRes->boardNum].initted = true;
                        	}
                        	else
                        	{
                        		lastHeard[ethRes->boardNum].initted = false;
                        	}
                        }

                        lastHeard[ethRes->boardNum].ageofframe_EMS = getRopFrameAge(incoming_msg);
                        lastHeard[ethRes->boardNum].prevRecvMsg_PC104 = ethRes->getLastRecvMsgTimestamp();
                    }

                    //break;  // devo uscire da questo while e rimanere in quello più esterno.
                }
                riterator++;
            }

            //ethManager->managerMutex.post();
        }
    }
    yError() << "Exiting recv thread";
    return;
}


// eof


