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

#include "EOYtheSystem.h"


#include <stdexcept>      // std::out_of_range
#include <yarp/os/Network.h>
#include <yarp/os/NetType.h>

#ifdef ICUB_USE_REALTIME_LINUX
#include <pthread.h>
#include <unistd.h>
#endif //ICUB_USE_REALTIME_LINUX

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;

TheEthManager* TheEthManager::handle = NULL;
yarp::os::Semaphore TheEthManager::managerMutex = 1;

//#ifdef _STATS_DEBUG_FOR_CYCLE_TIME_
ACE_INET_Addr		eb1(eb1_ip":12345");
ACE_INET_Addr		eb2(eb2_ip":12345");
ACE_INET_Addr		eb3(eb3_ip":12345");
ACE_INET_Addr		eb4(eb4_ip":12345");
ACE_INET_Addr		eb5(eb5_ip":12345");
ACE_INET_Addr		eb6(eb6_ip":12345");
ACE_INET_Addr		eb7(eb7_ip":12345");
ACE_INET_Addr		eb8(eb8_ip":12345");
ACE_INET_Addr		eb9(eb9_ip":12345");
ACE_INET_Addr		eb10(eb10_ip":12345");
ACE_INET_Addr		eb11(eb11_ip":12345");
//#endif


#undef _ENABLE_TRASMISSION_OF_EMPTY_ROPFRAME_ // if this macro is defined then ethManager sends pkts to ems even if they are empty
											  // ATTENTION: it is important to define/undefine the same macro also in hostTransceiver.cpp


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
        yTrace() << "Creating EMS device with IP " << remote_address;
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

ethResources *TheEthManager::requestResource(yarp::os::Searchable &config, FEAT_ID *request)
{
    yTrace() << request->boardNum;
    // Check if local socket is initted, if not do it.
    ACE_TCHAR       address[64];
    snprintf(address, sizeof(address), "%s:%d", request->PC104ipAddr.string, request->PC104ipAddr.port);

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
        if( strcmp(tmp_addr, request->EMSipAddr.string) == 0 )
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
        yTrace() << "Creating EMS device with IP " << request->EMSipAddr.string;
        newRes = new ethResources;
        if(!newRes->open(config, *request))
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

    if(     (EMS_list.size() == 0 ) && (boards_map.size() != 0 )
        ||  (EMS_list.size() != 0 ) && (boards_map.size() == 0 ) )
    {
        yError() << "Something strange happened... EMS_list.size is" << EMS_list.size() << "while boards_map.size is "<< boards_map.size();
    }

    if(!ret)
        yError() << "EthManager: Trying to release a non existing resource" << resource.name << " for boardId " << resource.boardNum << "maybe already deleted?";

    // ret = -1 means that the singleton is not needed anymore
    if( (EMS_list.size() == 0 ) || (boards_map.size() == 0 ) )
        ret = -1;

    managerMutex.post();
    return ret;
}


void TheEthManager::addLUTelement(FEAT_ID *id)
{
    yTrace() << id->boardNum;
    //in maps use board num starts from 0 so
    FEAT_boardnumber_t brdnum = id->boardNum; 
    /* NO MUTEX HERE because it's a PRIVATE method, so called only inside other already mutexed methods */
    /* Defined the var addLUT_result to have the true/false result of the insert operation...It helped catching a bug.
     * It fails if the element is already present. This can happen if someone tries to read an element with
     * the '[]' operator before the insert, because the std::map will create it automatically (hopefully initted
     * with zeros.
     */
     bool addLUT_result =  boards_map.insert(std::pair< std::pair<FEAT_boardnumber_t, eOprotEndpoint_t>, FEAT_ID>(std::make_pair<FEAT_boardnumber_t, eOprotEndpoint_t>(brdnum, id->ep), *id)).second;

    // Check result of insertion
    addLUT_result ? yTrace() << "ok add lut element for board " << id->boardNum << " and ep " << id->ep :
                    yError() << "NON ok add lut element for board " << id->boardNum << " and ep " << id->ep;


    std::pair<FEAT_boardnumber_t, eOprotEndpoint_t > key (brdnum, id->ep);
    try
        {
            // USE .at AND NOT the '[ ]' alternative!!! It will create a bug!!!
            /* The bug is caused by the fact that the [] version will create an unitialized element inside the map,
             * causing the return of a wrong pointer.
             * Furthermore the insert method used to correctly initialze the element will fail because a (wrong)
             * element is already present preventing the map to be corrected.
             */
        void * ret = boards_map.at(key).handle;
        }
        catch (const std::out_of_range& errMsg)
        {
                yError() << "Error after  LUT insertion!!!";
        }
}

bool TheEthManager::removeLUTelement(FEAT_ID element)
{
    yTrace() << element.boardNum;
    /* NO MUTEX HERE because it's a PRIVATE method, so called only inside other already mutexed methods */
    bool ret = false;
    int n = (int) boards_map.erase(std::make_pair<FEAT_boardnumber_t, eOprotEndpoint_t>(element.boardNum, element.ep));

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
            yTrace() << "FEAT_ID element removed succesfully from the map" << element.name;
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

void *TheEthManager::getHandle(FEAT_boardnumber_t boardnum, eOprotEndpoint_t ep)
{
//     managerMutex.wait();
    void * ret = NULL;
    static int _error = 0;
    std::pair<FEAT_boardnumber_t, eOprotEndpoint_t > key (boardnum, ep);

    try
    {
        // USE .at AND NOT the '[ ]' alternative!!! It will create a bug!!!
        /* The bug is caused by the fact that the [] version will create an unitialized element inside the map,
         * causing the return of a wrong pointer.
         * Furthermore the insert method used to correctly initialze the element will fail because a (wrong)
         * element is already present preventing the map to be corrected.
         */
        ret = boards_map.at(key).handle;
    }
    catch (const std::out_of_range& errMsg)
    {
        if(0 == (_error%1000) )
            yError() << "Got a message from boardNum "<< boardnum << " and EP " << ep << "but boards_map does not contains any related class pointer yet";

        _error++;
    }
//     managerMutex.post();
    return ret;
}

FEAT_ID TheEthManager::getFeatInfo(FEAT_boardnumber_t boardnum, eOprotEndpoint_t ep)
{
//    yTrace();
    FEAT_ID ret_val;
    std::pair<FEAT_boardnumber_t, eOprotEndpoint_t > key (boardnum, ep);

//     managerMutex.wait();  // il thread che chiama questa funz ha già preso questo mutex in ethReceiver::run ... // nn più vero dopo le ultime ottimizzazioni
    ret_val = boards_map.at(key);
//     managerMutex.post();
    return ret_val;
}

void embOBJerror(eOerrmanErrorType_t errtype, eOid08_t taskid, const char *eobjstr, const char *info)
{
    static const char* theerrors[] = { "eo_errortype_info", "eo_errortype_warning", "eo_errortype_weak", "eo_errortype_fatal" }; 

    yError() << "embOBJerror(): errtype = " << theerrors[errtype] << "from EOobject = " << eobjstr << " w/ message = " << info;

    if(errtype == eo_errortype_fatal)
    {
        yError() << "embOBJerror(): FATAL ERROR: the calling thread shall now be stopped in a forever loop here inside";
        for(;;);
    }

}


TheEthManager::TheEthManager()
{
    yTrace();
    memset(info, 0x00, sizeof(info));
    snprintf(info, sizeof(info), "TheEthManager");

    UDP_initted = false;
    UDP_socket  = NULL;
    emsAlreadyClosed = false;

    // marco.accame: in here we init the embOBJ system for YARP.
    eOerrman_cfg_t errmanconfig = {0};
    errmanconfig.extfn.usr_on_error        = embOBJerror;
    const eOysystem_cfg_t *syscfg       = NULL;
    const eOmempool_cfg_t *mpoolcfg     = NULL;     // uses standard mode 
    //const eOerrman_cfg_t *errmancf      = NULL;     // uses default mode
    eoy_sys_Initialise(syscfg, mpoolcfg, &errmanconfig);
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
    {
#ifdef ETHRECEIVER_ISPERIODICTHREAD
        receiver->stop();
#else
        ret = receiver->stop();
#endif
    }
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
        std::map<std::pair<FEAT_boardnumber_t, eOprotEndpoint_t>, FEAT_ID>::iterator mIt;
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
        //here sleep is essential in order to let sender thread send gotoconfig command.
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

void EthSender::run()
{
    ethResources  *ethRes;
    uint16_t      bytes_to_send = 0;
    ethResIt      iterator;
    ethResRIt    riterator, _rBegin, _rEnd;

#ifdef _STATS_DEBUG_FOR_CYCLE_TIME_
    // For statistic purpose

    double avThTime=getEstUsed();
    unsigned int it=getIterations();
    if(it == 2000)
    {
        double avPeriod, stdPeriod;
        double avThTime, stdTime;

        getEstUsed(avThTime, stdTime);
        getEstPeriod(avPeriod, stdPeriod);

        printf("EthSender Thread run %d times, est period: %.3lf, +-%.4lf[ms], est used: %.3lf, +-%.4lf[ms]\n",
                it,
                avPeriod, stdPeriod,
                avThTime, stdTime);
        resetStat();
    }
#endif

    /*
        Usare un reverse iterator per scorrere la lista dalla fine verso l'inizio. Questo aiuta a poter scorrere
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

#ifdef _ENABLE_TRASMISSION_OF_EMPTY_ROPFRAME_
        if((NULL != p_sendData))
#else
        if((bytes_to_send > EMPTY_PACKET_SIZE) && (NULL != p_sendData))
#endif
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
#ifdef ETHRECEIVER_ISPERIODICTHREAD
EthReceiver::EthReceiver(): RateThread(1)
#else
EthReceiver::EthReceiver()
#endif
{
    yTrace();
#ifdef ETHRECEIVER_STATISTICS_ON
    stat = new StatExt();
    stat_onRecFunc  = new StatExt();
    stat_onMutex = new StatExt();
#endif

#ifdef ETHRECEIVER_ISPERIODICTHREAD
    count=0;
    isFirst=true;
#endif
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

    ACE_HANDLE sockfd = pSocket->get_handle();
    int retval;
    int32_t mysize = 102400; //100kb note:actually kernel uses memory with size doblem of mysize
                            //with this size i'm sure ems pkts are not lost
    int len = sizeof(mysize);

    //the user can change buffer size by environment variable ETHRECEIVER_BUFFER_SIZE
    ConstString _dgram_buffer_size = NetworkBase::getEnvironment("ETHRECEIVER_BUFFER_SIZE");
    if (_dgram_buffer_size!="")
        mysize = NetType::toInt(_dgram_buffer_size);

    retval = ACE_OS::setsockopt  (sockfd, SOL_SOCKET, SO_RCVBUF, (char *)&mysize, sizeof(mysize));
    if (retval != 0)
    {
        int myerr = errno;
        yError()<< "ERROR in SetSockOpt SO_RCVBUF";
    }

    int32_t sock_input_buf_size =0;
    retval = ACE_OS::getsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, (char *)&sock_input_buf_size, &len);
    if (retval != 0)
    {
        int myerr = errno;
        yError() << "ERROR inGetSockOpt SO_RCVBUF";
    }

    yDebug() << "ethReceiver config socket with queue size = "<< sock_input_buf_size<< "; you request ETHRECEIVER_BUFFER_SIZE=" << _dgram_buffer_size;


    for(int i=0; i<10; i++)
    {
        recFirstPkt[i] = false;
        seqnumList[i] = 0;
    }
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


// marco.accame on 10 apr 2014: better not using the exact copy of a struct defined elsewhere (in EOropframe_hid.h) 
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


// marco.accame on 11 apr 2014:
// if we want to detect the board number by address, ... allow some more or think about a more general rule.
int EthReceiver::getBoardNum(ACE_INET_Addr addr)
{
    int board;

    // detect board
    if( addr == eb1)
    {
        board = 1;
    }
    else if( addr == eb2)
    {
        board = 2;
    }
    else if( addr == eb3)
    {
        board = 3;
    }
    else if( addr == eb4)
    {
        board = 4;
    }
    else if( addr == eb5)
    {
        board = 5;
    }
    else if( addr == eb6)
    {
        board = 6;
    }
    else if( addr == eb7)
    {
        board = 7;
    }
    else if( addr == eb8)
    {
        board = 8;
    }
    else if( addr == eb9)
    {
        board = 9;
    }
    else if( addr == eb10)
    {
        board = 10;
    }
    else if( addr == eb11)
    {
        board = 11;
    }
    else
    {
        board = 0;
    }

    return(board);
}



void EthReceiver::checkPktSeqNum(char* pktpayload, ACE_INET_Addr addr)
{
    int board = getBoardNum(addr);
    //check seq num
    uint64_t seqnum = *((uint64_t*)(&pktpayload[16]));
    if(recFirstPkt[board]==false)
    {
        seqnumList[board] = seqnum;
        recFirstPkt[board] = true;
        yError()<< "EthREceiver: FIRST SEQ NUM for board=" <<board<< " is "<<seqnum;
    }
    else
    {
        if(seqnum != seqnumList[board]+1)
        {
            yError()<< "EthREceiver: ---LOST PKTS---board=" <<board<< " seq num rec="<<seqnum << " expected=" << seqnumList[board]+1;
        }
        seqnumList[board] = seqnum;
    }
}

//#ifndef ETHRECEIVER_ISPERIODICTHREAD
//void EthReceiver::run()
//{
//    yTrace();
//    ACE_TCHAR     address[64];
//    ethResources  *ethRes;
//    ssize_t       recv_size;
//    ACE_INET_Addr sender_addr;
//    char          incoming_msg[RECV_BUFFER_SIZE];
//    int board;
//
//    // TODO aggiunto per debugging. tenere tempo trascorso dall'ultima ricezione di un msg da parte della ems i-esima.
//    std::map<int, times_test_delay> lastHeard;
//
//    yError() << "Starting udp RECV thread with prio "<< getPriority() << "\n";
//
//    ACE_Time_Value recvTimeOut;
//    fromDouble(recvTimeOut, 0.01);
//    double myTestTimeout = recvTimeOut.sec() + (double)recvTimeOut.msec()/1000.0f;
//
//
//
//#ifdef _STATS_DEBUG_FOR_CYCLE_TIME_
//    for(int i=1; i<=9; i++)
//    {
//        stats[i].resetStat();
//    }
//#endif
//
//    while(!isStopping())
//    {
//        ethManager->managerMutex.wait();
//        // new, reverse iterator
//        ethResRIt    riterator, _rBegin, _rEnd;
//        _rBegin = ethResList->rbegin();
//        _rEnd = ethResList->rend();
//        ethManager->managerMutex.post();
//
//        // ricevo un messaggio dal socket: chiamata bloccante con timeout
//        recv_size = recv_socket->recv((void *) incoming_msg, RECV_BUFFER_SIZE, sender_addr, 0, &recvTimeOut);
//
//        if(recv_size < 1)
//        {
//            //if i'm here, i exited from recv because of timeout
//            yError() << "EthReceiver: passed " <<myTestTimeout *1000<< " ms without receive a pkt!!";
//        }
//
//        if( recv_size > 60000)
//        {
//            yWarning() << "Huge message received " << recv_size;
//        }
//
//
//        //verifico che non siano passati più di 10 milli da l'ultima volta che ho ricevuta un pkt da una scheda
//        riterator = _rBegin;
//        while(riterator != _rEnd)
//        {
//            ethRes = (*riterator);
//            if(ethRes->isRunning() /*&& (ethRes->getLastRecvMsgTimestamp()>0)*/ && (lastHeard[ethRes->boardNum].initted))
//            {
//                if(yarp::os::Time::now() - ethRes->getLastRecvMsgTimestamp() > myTestTimeout)
//                {
//                    if(!lastHeard[ethRes->boardNum].error_PC104)
//                    {
//                        yError() << "Board " << ethRes->boardNum << ": more than " << myTestTimeout *1000 << "ms are passed without any news LAST=" << ethRes->getLastRecvMsgTimestamp() ;
//                        lastHeard[ethRes->boardNum].error_PC104 = true;
//                    }
//                }
//            }
//            else
//            {
//                lastHeard[ethRes->boardNum].initted = false;
//            }
//            riterator++;
//        }
//
//
//#ifdef _STATS_DEBUG_FOR_CYCLE_TIME_
//        int board = getBoardNum(sender_addr);
//        // For statistic purpose
//        stats[board].tickStart();
//#endif
//
//
//
//        sender_addr.addr_to_string(address, 64);
//
//        if( (recv_size > 0) && (isRunning()) )
//        {
//            checkPktSeqNum(incoming_msg, sender_addr);
//
//
//            //se ho ricevuto un pkt, allora cerco la scheda nella lista e controllo le tempistiche
//            riterator = _rBegin;
//
//            while(riterator != _rEnd)
//            {
//                ethRes = (*riterator);
//                if(ethRes->getRemoteAddress() == sender_addr)
//                {
//                    if(recv_size > ethRes->getBufferSize())
//                    {
//                        yError() << "EthReceiver got a message of wrong size ( received" << recv_size << " bytes while buffer is" << ethRes->getBufferSize() << " bytes long)";
//                    }
//                    else
//                    {
//                        memcpy(ethRes->recv_msg, incoming_msg, recv_size);
//                        ethRes->onMsgReception(ethRes->recv_msg, recv_size);
//
//                        if(lastHeard[ethRes->boardNum].initted)
//                        {
//                            bool gap_ems_limits_changed =false;
//                            //check recvTime on the PC104 side
//                            lastHeard[ethRes->boardNum].gap_PC104 = ethRes->getLastRecvMsgTimestamp() - lastHeard[ethRes->boardNum].prevRecvMsg_PC104;
//
//                            if(lastHeard[ethRes->boardNum].gap_PC104 > myTestTimeout)
//                            {
//                                yError() << "Board " << ethRes->boardNum << ": Gap of " << lastHeard[ethRes->boardNum].gap_PC104*1000 << "ms between two consecutive messages !!!";
//                            }
//
//                            if(lastHeard[ethRes->boardNum].gap_PC104 < lastHeard[ethRes->boardNum].gap_PC104_min)
//                            {
//                                lastHeard[ethRes->boardNum].gap_PC104_min = lastHeard[ethRes->boardNum].gap_PC104;
//                                gap_ems_limits_changed = true;
//                            }
//
//
//                            if(lastHeard[ethRes->boardNum].gap_PC104 > lastHeard[ethRes->boardNum].gap_PC104_max)
//                            {
//                                lastHeard[ethRes->boardNum].gap_PC104_max = lastHeard[ethRes->boardNum].gap_PC104;
//                                gap_ems_limits_changed = true;
//                            }
//
//                            if(gap_ems_limits_changed)
//                            {
//                                yError() << "EthReceiver: ems gap changed for board " << ethRes->boardNum << ": min=" << lastHeard[ethRes->boardNum].gap_PC104_min*1000 << " max=" << lastHeard[ethRes->boardNum].gap_PC104_max*1000;
//                            }
//
//
//                            // check time written into packet
//                            int diff = (int)(getRopFrameAge(incoming_msg)/1000 - lastHeard[ethRes->boardNum].ageofframe_EMS/1000);
//                            if( diff > (int)(myTestTimeout * 1000))
//                            {
//                                yError() << "Board " << ethRes->boardNum << ": EMS time between 2 ropframes bigger then " << myTestTimeout * 1000 << "ms;\t Actual delay is" << diff << "ms.";
//                            }
//
//                            //se sono qui significa che ho ricevuto un pkt dalla scheda, allora resetto gli errori
//                            lastHeard[ethRes->boardNum].error_PC104 = false;
//
//                        }
//                        else
//                        {
//                            if(ethRes->isRunning())
//                            {
//                                lastHeard[ethRes->boardNum].initted = true;
//                            }
//                            else
//                            {
//                                lastHeard[ethRes->boardNum].initted = false;
//                            }
//                        }
//                        //aggiorno i valori
//                        lastHeard[ethRes->boardNum].ageofframe_EMS = getRopFrameAge(incoming_msg);
//                        lastHeard[ethRes->boardNum].prevRecvMsg_PC104 = ethRes->getLastRecvMsgTimestamp();
//                    }
//
//                    //break;  // devo uscire da questo while e rimanere in quello più esterno.
//                }
//                riterator++;
//            }
//
//            //ethManager->managerMutex.post();
//        }
//#ifdef _STATS_DEBUG_FOR_CYCLE_TIME_
//        stats[board].tickEnd();
//
//        //compute statistics
//        if (stats[board].getIterations() == 2000)
//        {
//            double avEst=0;
//            double stdEst=0;
//            double avUsed=0;
//            double stdUsed=0;
//            stats[board].getEstPeriod(avEst, stdEst);
//            stats[board].getEstUsed(avUsed, stdUsed);
//            printf("EthReceiver Thread [%d] run %d times, est period: %.3lf, +-%.4lf[ms], est used: %.3lf, +-%.4lf[ms]\n", board, stats[board].getIterations(), avEst, stdEst, avUsed, stdUsed);
//            stats[board].resetStat();
//        }
//
//        double totUsed = 0;
//        if(board == 9)
//        {
//            for(int i=1; i<=9; i++)
//            {
//                totUsed += stats[i].getElapsed();
//            }
//            if(totUsed >= 0.95)
//                printf("**EthReceiver Thread: total used time to precess 9 ropframe is %f**\n", totUsed);
//        }
//#endif
//    }
//    yError() << "Exiting recv thread";
//    return;
//}
//#endif











#ifndef ETHRECEIVER_ISPERIODICTHREAD
void EthReceiver::run()
{
    yTrace();

    ACE_TCHAR     address[64];
    ethResources  *ethRes;
    ssize_t       recv_size;
    ACE_INET_Addr sender_addr;
    char          incoming_msg[RECV_BUFFER_SIZE];
    bool recError = false;
    bool allEmsInConfigstate = true; //if true means all ems are in config state


    //yDebug() << "Starting udp RECV thread with prio "<< getPriority() << "\n";
    ACE_Time_Value recvTimeOut;
    fromDouble(recvTimeOut, 0.01);

#ifdef ETHRECEIVER_STATISTICS_ON
    bool isFirst =true;
    double last_time, curr_time, diff;
    double before_rec, after_rec, diff_onRec;
    double before_mutex, after_mutex, diff_onMutex;
    int count;
    #define count_max 5000
#endif
    while(!isStopping())
    {

#ifdef ETHRECEIVER_STATISTICS_ON
        before_rec = yarp::os::Time::now();
#endif
        //get pkt from socket: blocking call with timeout
        recv_size = recv_socket->recv((void *) incoming_msg, RECV_BUFFER_SIZE, sender_addr, 0, &recvTimeOut);
#ifdef ETHRECEIVER_STATISTICS_ON
        after_rec =  yarp::os::Time::now();
        diff_onRec = after_rec - before_rec;
        stat_onRecFunc->add((diff_onRec*1000));
#endif
        if(!isRunning())
        {
            continue; //i go to recv a new pkt and wait someone to stop me
        }

#ifdef ETHRECEIVER_STATISTICS_ON
        before_mutex = yarp::os::Time::now();
#endif
        //take pointers to ems board list
         ethManager->managerMutex.wait();
         // new, reverse iterator
         ethResRIt    riterator, _rBegin, _rEnd;
         _rBegin = ethResList->rbegin();
         _rEnd = ethResList->rend();
         ethManager->managerMutex.post();


#ifdef ETHRECEIVER_STATISTICS_ON
         after_mutex = yarp::os::Time::now();
         diff_onMutex = after_mutex - before_mutex;
         stat_onMutex->add((diff_onMutex*1000));
#endif

         //i get here because of timeout ot i received a pkt;in both cases i check if all boards are alive.In the meanwhile i check if all ems are in config state.
         riterator = _rBegin;
         bool allEmsInConfigstate = true;
         double curr_time = yarp::os::Time::now();
         while(riterator != _rEnd)
         {
             ethRes = (*riterator);
             if(ethRes->isRunning())
             {
                 ethRes->checkIsAlive(curr_time);
                 allEmsInConfigstate = true;
             }
             riterator++;
         }



        if(recv_size < 1)
        {
            //print error if have not already done  and if one or more ems boards are in running state , so thy should sent pkt every 1 msec
            if((!recError) &&(!allEmsInConfigstate))
            {
                //if i'm here, i exited from recv because of timeout
                yError() << "EthReceiver: passed " <<recvTimeOut.msec() << " ms without receive a pkt!!";
                recError = true;
            }continue; //try to receive again
        }
        else
        {
            recError=false;
        }


#ifdef ETHRECEIVER_STATISTICS_ON
        if(isFirst)
        {
            last_time = yarp::os::Time::now();
            isFirst = false;
        }
        else
        {
            curr_time = yarp::os::Time::now();
            diff = curr_time - last_time;
            stat->add((diff*1000));
            if (diff> 0.006)
                yError() <<"ethReceiver pass more 6 mill without activity!!! diff= "<< diff;

            count++;
            last_time = curr_time;
        }

        if(count == count_max)
        {
            yDebug()<< "ETHRECEIVER stat: avg=" << stat->mean()<< "ms std=" << stat->deviation()<< "ms min=" << stat->getMin() << "ms max=" << stat->getMax()<< "ms  " ;
            yDebug()<< "ETHRECEIVER stat_onRecFunc: avg=" << stat_onRecFunc->mean()<< "ms std=" << stat_onRecFunc->deviation()<< "ms min=" << stat_onRecFunc->getMin() << "ms max=" << stat_onRecFunc->getMax()<< "ms  " ;
            yDebug()<< "ETHRECEIVER stat_onMutex: avg=" << stat_onMutex->mean()<< "ms std=" << stat_onMutex->deviation()<< "ms min=" << stat_onMutex->getMin() << "ms max=" << stat_onMutex->getMax()<< "ms  " ;
            count = 0;
            stat->clear();
            stat_onRecFunc->clear();
            stat_onMutex->clear();
        }
#endif





        //if i rec a pkt, then looking for the sender ems and parse the pkt
        sender_addr.addr_to_string(address, 64);
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
                    memcpy(ethRes->recv_msg, incoming_msg, recv_size);
                    ethRes->onMsgReception(ethRes->recv_msg, recv_size);
                }
                break;
            }
            riterator++;
        }

    }//while(!isStopping)

//    yError() << "Exiting recv thread";
    return;
}
#endif



#ifdef ETHRECEIVER_ISPERIODICTHREAD
#define MAX_COUNT_STAT  120000
#define MAX_NUM_PKT     15
void EthReceiver::run()
{
//attention: don't insert too prints because this function is called every 1 millisec

    ACE_TCHAR     address[64];
    ethResources  *ethRes;
    ssize_t       recv_size;
    ACE_INET_Addr sender_addr;
    char          incoming_msg[RECV_BUFFER_SIZE];
    int flags = 0;
    static int countstat =0;
    int num_pkt;
    int myerr;




    ACE_Time_Value recvTimeOut;
    fromDouble(recvTimeOut, 0.01);
    double myTestTimeout = recvTimeOut.sec() + (double)recvTimeOut.msec()/1000.0f;


    countstat++;
    if(countstat== MAX_COUNT_STAT)
    {
        double av, std, avUsed, stdUsed;
        getEstPeriod (av, std);
        getEstUsed (avUsed, stdUsed);
        yDebug()<< "=== estPeriod: av=" <<av<<" std=" <<std;
        yDebug()<< "===UsedPeriod: av=" <<avUsed<<" std=" <<stdUsed;
        countstat = 0;
    }



#ifdef _STATS_DEBUG_FOR_CYCLE_TIME_
    for(int i=1; i<=9; i++)
    {
        stats[i].resetStat();
    }
#endif


    flags |= MSG_DONTWAIT;

    while(num_pkt<MAX_NUM_PKT)
    {

        // per ogni msg ricevuto  -1 visto come 65535!!
        recv_size = recv_socket->recv((void *) incoming_msg, RECV_BUFFER_SIZE, sender_addr, flags);
        if(recv_size < 1)
        {
            //if i'm here socket input queue is empty
            return;
        }

        ethManager->managerMutex.wait();
        // new, reverse iterator
        ethResRIt    riterator, _rBegin, _rEnd;
        _rBegin = ethResList->rbegin();
        _rEnd = ethResList->rend();
        ethManager->managerMutex.post();

        if( recv_size > 60000)
        {
            yWarning() << "Huge message received " << recv_size;
        }


#ifdef _STATS_DEBUG_FOR_CYCLE_TIME_
        int board = getBoardNum(sender_addr);
        // For statistic purpose
        stats[board].tickStart();
#endif




        //if i rec a pkt, then looking for the sender ems and parse the pkt
        sender_addr.addr_to_string(address, 64);
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
                    memcpy(ethRes->recv_msg, incoming_msg, recv_size);
                    ethRes->onMsgReception(ethRes->recv_msg, recv_size);
                }
                break;
            }
            riterator++;
        }



#ifdef _STATS_DEBUG_FOR_CYCLE_TIME_
        stats[board].tickEnd();

        //compute statistics
        if (stats[board].getIterations() == 2000)
        {
            double avEst=0;
            double stdEst=0;
            double avUsed=0;
            double stdUsed=0;
            stats[board].getEstPeriod(avEst, stdEst);
            stats[board].getEstUsed(avUsed, stdUsed);
            printf("EthReceiver Thread [%d] run %d times, est period: %.3lf, +-%.4lf[ms], est used: %.3lf, +-%.4lf[ms]\n", board, stats[board].getIterations(), avEst, stdEst, avUsed, stdUsed);
            stats[board].resetStat();
        }

        double totUsed = 0;
        if(board == 9)
        {
            for(int i=1; i<=9; i++)
            {
                totUsed += stats[i].getElapsed();
            }
            if(totUsed >= 0.95)
                printf("**EthReceiver Thread: total used time to precess 9 ropframe is %f**\n", totUsed);
        }
#endif
        num_pkt++;
    }
//    yError() << "Exiting recv thread";
    return;
}

#endif


// eof



