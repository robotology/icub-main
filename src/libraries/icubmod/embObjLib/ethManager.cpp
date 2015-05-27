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
yarp::os::Semaphore TheEthManager::managerMutex = 1;



// -------------------------------------------------------------------\\
//            TheEthManager   Singleton
// -------------------------------------------------------------------\\

bool TheEthManager::lock()
{
    managerMutex.wait();
    return true;
}

bool TheEthManager::unlock()
{
    managerMutex.post();
    return true;
}

ethResources *TheEthManager::requestResource(yarp::os::Searchable &cfgtotal, yarp::os::Searchable &cfgtransceiver, yarp::os::Searchable &cfgprotocol, ethFeature_t &request)
{
    yTrace() << " Requested BRD" << request.boardNumber;
    // Check if local socket is initted, if not do it.
    ACE_TCHAR       address[64];
    snprintf(address, sizeof(address), "%s:%d", request.pc104IPaddr.string, request.pc104IPaddr.port);

    if(request.boardNumber > TheEthManager::maxBoards)
    {
        yError() << "FATAL ERROR: TheEthManager::requestResource() detected a board number beyond the maximum allowed (max, rqst) =" << maxBoards << request.boardNumber << ")";
        return NULL;
    }

    if(request.boardIPaddr.ip4 != request.boardNumber)
    {
        yError() << "FATAL ERROR: TheEthManager::requestResource() detected a board number different from its ip4 address (boardNumber, ip4) =" << request.boardNumber << request.boardIPaddr.ip4 << ")";
        return NULL;
    }

    ACE_UINT32 hostip = (request.pc104IPaddr.ip1 << 24) | (request.pc104IPaddr.ip2 << 16) | (request.pc104IPaddr.ip3 << 8) | (request.pc104IPaddr.ip4);
    ACE_INET_Addr myIP((u_short)request.pc104IPaddr.port, hostip);
    myIP.dump();
    char tmp_addr[64];

    int txrate = -1; // uses default
    int rxrate = -1; // uses default

    // if i find it in section ... of cfgtotal then i change it


    if(cfgtotal.findGroup("ETH").check("PC104TXrate"))
    {
        int value = cfgtotal.findGroup("ETH").find("PC104TXrate").asInt();
        if(value > 0)
            txrate = value;
    }
    else
    {
        yWarning () << "TheEthManager::requestResource() cannot find ETH/PC104TXrate. thus using default value";
    }

    if(cfgtotal.findGroup("ETH").check("PC104RXrate"))
    {
        int value = cfgtotal.findGroup("ETH").find("PC104RXrate").asInt();
        if(value > 0)
            rxrate = value;
    }
    else
    {
        yWarning () << "TheEthManager::requestResource() cannot find ETH/PC104RXrate. thus using default value";
    }

    if(!createSocket(myIP, txrate, rxrate) )
    {  return NULL;  }


    int justCreated = false;
    ethResources *newRes = NULL;


    lock();          // lock so that we can use EMS_list in exclusive way
    ethResIt it = EMS_list.begin();
    ethResIt itend = EMS_list.end();

    while(it != itend)
    {
        (*it)->getRemoteAddress().addr_to_string(tmp_addr, 64);
        if( strcmp(tmp_addr, request.boardIPaddr.string) == 0 )
        {
            // device already exists, return the pointer.
            newRes = (*it);
            break;
        }
        it++;
    }
    unlock(); // must unlock now. because ethResources::open() need to use the receiver which uses the lock() to get the EMS_list

    if(NULL == newRes)
    {
        // device doesn't exist yet: create it
        yTrace() << "Creating BRD device with IP " << request.boardIPaddr.string;
        newRes = new ethResources;
        if(!newRes->open(cfgtotal, cfgtransceiver, cfgprotocol, request))
        {
            yError() << "Error creating new BRD!!";
            if(NULL != newRes)
                delete newRes;

            newRes = NULL;
            return NULL;
        }
        justCreated = true;
    }


    lock(); // lock because i use the EMS_list and i call addLUTelement

    // the push_back has to be done only once for EMS
    if(justCreated)
    {
        EMS_list.push_back(newRes);
        int index = request.boardNumber-1;
        if(index < ethresLUTsize)
        {
            ethresLUT[index] = newRes;
            numberOfUsedBoards++;
        }
    }

    // The registerFeature has to be done always
    newRes->registerFeature(request);
    addLUTelement(request);

    unlock();

    return newRes;
}



int TheEthManager::releaseResource(ethFeature_t &resource)
{
    yTrace() << resource.boardNumber;
    int           ret           = 0;
    int           stillUsed     = 0;
    ethResources  *res2release  = NULL;
    char tmp_addr[64];


    ethResources *tmpEthRes;
    ACE_INET_Addr  tmp_ace_addr;

    lock();
    ethResIt it = EMS_list.begin();
    ethResIt itend = EMS_list.end();
    unlock();

    if(false == emsAlreadyClosed)
    {
        while(it != itend)
        {
            tmpEthRes = (*it);
            tmpEthRes->goToConfig();
            tmpEthRes->clearRegulars();
            it++;
        }
        // before stopping threads, flush all pkts not yet sent.
        flush();
        emsAlreadyClosed = true;
    }
    stopThreads();

    removeLUTelement(resource);

    lock();
    it = EMS_list.begin();
    itend = EMS_list.end();
    while(it != itend)
    {
        tmpEthRes = (*it);
        tmp_ace_addr = tmpEthRes->getRemoteAddress();
        tmp_ace_addr.addr_to_string(tmp_addr, 64);
        if( strcmp(tmp_addr, resource.boardIPaddr.string) == 0)
        {
            // device exists
            res2release = (*it);
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
        it++;
    }

    if(     (EMS_list.size() == 0 ) && (boards_map.size() != 0 )
        ||  (EMS_list.size() != 0 ) && (boards_map.size() == 0 ) )
    {
        yError() << "Something strange happened... EMS_list.size is" << EMS_list.size() << "while boards_map.size is "<< boards_map.size();
    }

    if(!ret)
        //yError() << "EthManager: Trying to release a non existing resource" << resource.name << " for boardId " << resource.boardNumber << "maybe already deleted?";
        yError() << "EthManager: Trying to release a non existing resource for boardId " << resource.boardNumber << "maybe already deleted?";

    // ret = -1 means that the singleton is not needed anymore
    if( (EMS_list.size() == 0 ) || (boards_map.size() == 0 ) )
        ret = -1;

    unlock();

    return ret;
}


void TheEthManager::addLUTelement(ethFeature_t &id)
{
    yTrace() << id.boardNumber;
    //in maps use board num starts from 0 so
    FEAT_boardnumber_t brdnum = id.boardNumber;
    /* NO MUTEX HERE because it's a PRIVATE method, so called only inside other already mutexed methods */
    /* Defined the var addLUT_result to have the true/false result of the insert operation...It helped catching a bug.
     * It fails if the element is already present. This can happen if someone tries to read an element with
     * the '[]' operator before the insert, because the std::map will create it automatically (hopefully initted
     * with zeros.
     */
     bool addLUT_result =  boards_map.insert(std::pair< std::pair<FEAT_boardnumber_t, eOprotEndpoint_t>, ethFeature_t>(std::make_pair(brdnum, id.endpoint), id)).second;

    // Check result of insertion
    addLUT_result ? yTrace() << "ok add lut element for board " << id.boardNumber << " and ep " << id.endpoint :
                    yError() << "NON ok add lut element for board " << id.boardNumber << " and ep " << id.endpoint;


    std::pair<FEAT_boardnumber_t, eOprotEndpoint_t > key (brdnum, id.endpoint);
    try
    {
        // USE .at AND NOT the '[ ]' alternative!!! It will create a bug!!!
        /* The bug is caused by the fact that the [] version will create an unitialized element inside the map,
            * causing the return of a wrong pointer.
            * Furthermore the insert method used to correctly initialze the element will fail because a (wrong)
            * element is already present preventing the map to be corrected.
            */
        IethResource * ret = boards_map.at(key).interface;
    }
    catch (const std::out_of_range& errMsg)
    {
        yError() << "Error after  LUT insertion!!! (" << errMsg.what() << ")";
    }
}

bool TheEthManager::removeLUTelement(ethFeature_t &element)
{
    yTrace() << element.boardNumber;
    /* NO MUTEX HERE because it's a PRIVATE method, so called only inside other already mutexed methods */
    bool ret = false;
    int n = (int) boards_map.erase(std::make_pair(element.boardNumber, element.endpoint));

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
            yTrace() << "ethFeature_t element removed succesfully from the map" << element.name;
            ret = true;
            break;
        }
        default:
        {
            yError() << "More than one element were removed with key board num " << element.boardNumber << "ep " << element.endpoint;
            ret = true;
            break;
        }
    }

    return ret;
}

bool TheEthManager::getHandle(FEAT_boardnumber_t boardnum, eOprotID32_t id32, IethResource **interfacePointer, ethFeatType_t *type)
{
    eOprotEndpoint_t ep = eoprot_ID2endpoint(id32);

//     lock(); // marco.accame: found already commented. see why
    bool ret = false;
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
        *interfacePointer = boards_map.at(key).interface;
        *type = boards_map.at(key).type;
        ret = true;
    }
    catch (const std::out_of_range& errMsg)
    {
        if(0 == (_error%1000))
        {
            char nvinfo[128];
            eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
            interfacePointer = NULL;
            *type = ethFeatType_NULL;

            yError() << "TheEthManager::getHandle() cannot find a handle for boardNum "<< boardnum << " and nv" << nvinfo << " ... maybe the board was in running mode before robotInterface was started (" << errMsg.what() <<")";
        }
        ret = false;
        _error++;
    }
//     unlock(); // marco.accame: found already commented. see why
    return ret;
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
    memset(info, 0, sizeof(info));
    snprintf(info, sizeof(info), "TheEthManager");

    memset(ethresLUT, 0, sizeof(ethresLUT));
    numberOfUsedBoards = 0;

    EMS_list.clear();
    UDP_initted = false;
    UDP_socket  = NULL;
    emsAlreadyClosed = false;

    TheEthManager::initEOYsystem();

    starttime = yarp::os::Time::now();
}


EthSender* TheEthManager::getEthSender(void)
{
    return sender;
}


EthReceiver* TheEthManager::getEthReceiver(void)
{
    return receiver;
}


double TheEthManager::getStartTime(void)
{
    return starttime;
}

bool TheEthManager::getEMSlistRiterators(ethResRIt& rbegin, ethResRIt& rend)
{   // marco.accame: added this function so that we can keep lock()/unlock() private
    lock();
    rbegin = EMS_list.rbegin();
    rend = EMS_list.rend();
    unlock();
    return true;
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


bool TheEthManager::createSocket(ACE_INET_Addr local_addr, int txrate, int rxrate)
{
    yTrace();
    lock();


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

    unlock();
    return UDP_initted;
}

bool TheEthManager::isInitted(void)
{
    yTrace();
    bool ret;
    lock();
    ret = UDP_initted;
    unlock();
    return ret;
}

TheEthManager *TheEthManager::instance()
{
    yTrace();
    managerMutex.wait();    // marco.accame: in here we dont use this->lock() because if object does not already exists, the function does not exists. i can use the static semaphore instead
    if (NULL == handle)
    {
        yTrace() << "Calling EthManager Constructor";
        handle = new TheEthManager();
        if (NULL == handle)
            yError() << "While calling EthManager constructor";
        else
            feat_Initialise(static_cast<void*>(handle));
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

    // Close UDP socket
    if(isInitted())
    {
        managerMutex.wait();
        UDP_socket->close();
        delete UDP_socket;
        UDP_initted = false;
        managerMutex.post();
    }


    lock();
    // Destroy all EMS boards
    if(EMS_list.size() != 0)
    {
        memset(ethresLUT, 0, sizeof(ethresLUT));
        numberOfUsedBoards = 0;

        ethResIt iterator = EMS_list.begin();

        while(iterator != EMS_list.end())
        {
            delete(*iterator);
            iterator++;
        }
    }

    if(boards_map.size() != 0)
    {
        std::map<std::pair<FEAT_boardnumber_t, eOprotEndpoint_t>, ethFeature_t>::iterator mIt;
        for(mIt = boards_map.begin(); mIt!=boards_map.end(); mIt++)
        {
            yError() << "Feature " << mIt->second.name << "was not correctly removed from map.., removing it now in the EthManager destructor.";
            removeLUTelement(mIt->second);
        }
    }
    unlock();
    handle = NULL;
    managerMutex.post();
}

int TheEthManager::send(void *data, size_t len, ACE_INET_Addr remote_addr)
{
    ssize_t ret = UDP_socket->send(data, len, remote_addr);
    return ret;
}

ethResources* TheEthManager::IPtoResource(ACE_INET_Addr adr)
{
    ACE_UINT32 a32 = adr.get_ip_address();
    uint32_t index = a32 & 0xff;
    index --;
    if(index>=ethresLUTsize)
    {
        return(NULL);
    }

    return(TheEthManager::ethresLUT[index]);
}

int TheEthManager::GetNumberOfUsedBoards(void)
{
    return(numberOfUsedBoards);
}

ethResources* TheEthManager::GetEthResource(FEAT_boardnumber_t boardnum)
{
    ethResources *res = NULL;

    lock();

    ethResIt iterator = EMS_list.begin();

    eOprotBRD_t targetbrd = featIdBoardNum2nvBoardNum(boardnum);
    while(iterator != EMS_list.end())
    {
        eOprotBRD_t brdn = (*iterator)->get_protBRDnumber();

        if(brdn == targetbrd)
        {
            res = *iterator;
            break;
        }
        iterator++;
    }


    unlock();

    return(res);
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


EthSender::EthSender(int txrate) : RateThread(txrate)
{

    rateofthread = txrate;
    yDebug() << "EthSender is a RateThread with rate =" << rateofthread << "ms";
    yTrace();
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

void EthSender::evalPrintTXstatistics(void)
{
#ifdef ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_
    // For statistic purpose

    unsigned int it=getIterations();
    if(it == ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_NUMBEROF_CYCLES)
    {
        printTXstatistics();
    }
#endif
}

void EthSender::printTXstatistics(void)
{
#ifdef ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_
    // For statistic purpose

    unsigned int it=getIterations();

    double avPeriod, stdPeriod;
    double avThTime, stdTime;

    getEstUsed(avThTime, stdTime);
    getEstPeriod(avPeriod, stdPeriod);

    char string[128] = {0};
    snprintf(string, sizeof(string), "  (STATS-TX)-> EthSender::run() thread run %d times, est period: %.3lf, +-%.4lf[ms], est used: %.3lf, +-%.4lf[ms]\n",
                            it,
                            avPeriod, stdPeriod,
                            avThTime, stdTime);
    yDebug() << string;

//        printf("EthSender::run() thread run %d times, est period: %.3lf, +-%.4lf[ms], est used: %.3lf, +-%.4lf[ms]\n",
//                it,
//                avPeriod, stdPeriod,
//                avThTime, stdTime);
    resetStat();

#endif
}

void EthSender::run()
{
    ethResources  *ethRes;
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
        bool transmitthepacket = ethRes->getPointer2TxPack(&p_sendData, &bytes_to_send, &numofrops);

        if(true == transmitthepacket)
        {
            ACE_INET_Addr addr = ethRes->getRemoteAddress();
            int ret = ethManager->send(p_sendData, (size_t)bytes_to_send, addr);
        }

    }

}


#ifdef ETHRECEIVER_ISPERIODICTHREAD
EthReceiver::EthReceiver(int raterx): RateThread(raterx)
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
    rateofthread = raterx;
    yDebug() << "EthReceiver is a RateThread with rate =" << rateofthread << "ms";
    // ok, and now i get it from xml file ... if i find it.
#endif

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
    uint8_t tmp;
    ethManager->send( &tmp, 1, ethManager->local_addr);
}

EthReceiver::~EthReceiver()
{
    yTrace();
#ifdef ETHRECEIVER_STATISTICS_ON
    delete stat;
    delete stat_onRecFunc;
    delete stat_onMutex;
#endif
}

bool EthReceiver::config(ACE_SOCK_Dgram *pSocket, TheEthManager* _ethManager)
{
    yTrace();
    recv_socket = pSocket;
    ethManager  = _ethManager;

    ACE_HANDLE sockfd = pSocket->get_handle();
    int retval;
    int32_t mysize = 1024*1024; //1Mb note:actually kernel uses memory with size doblem of mysize
                            //with this size i'm sure ems pkts are not lost
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


    for(int i=0; i<TheEthManager::maxBoards; i++)
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


uint64_t getRopFrameAge(char *pck)
{
    return(eo_ropframedata_age_Get((EOropframeData*)pck));
}



int EthReceiver::getBoardNum(ACE_INET_Addr addr)
{
    int board;
    ACE_UINT32 ip = addr.get_ip_address();

    // board is last number in ip address
    board = ip & 0xff;
    return(board);
}



void EthReceiver::checkPktSeqNum(char* pktpayload, ACE_INET_Addr addr)
{
    int board = getBoardNum(addr);
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



#ifndef ETHRECEIVER_ISPERIODICTHREAD

#if 1
// version simplified by marco.accame on 02 oct 2014

void EthReceiver::run()
{
    yTrace();

    ACE_TCHAR     address[64];
    ethResources  *ethRes;
    ssize_t       incoming_msg_size;
    ACE_INET_Addr sender_addr;
    uint64_t      incoming_msg_data[ethResources::maxRXpacketsize/8]; // 8-byte aligned local buffer for incoming packet: it must be able to accomodate max size of packet
    const int     incoming_msg_capacity = ethResources::maxRXpacketsize;

    ethResRIt riterator;
    ethResRIt _rBegin, _rEnd;
    double statLastTime = yarp::os::Time::now();


    ACE_Time_Value recvTimeOut;
    recvTimeOut.set(0.010f);

    while(!isStopping())
    {   // forever loop... almost

        // get pkt from socket: blocking call with timeout
        incoming_msg_size = recv_socket->recv((void *) incoming_msg_data, incoming_msg_capacity, sender_addr, 0, &recvTimeOut);

        if(!isRunning())
        {
            continue;  // i go to recv a new pkt and wait someone to stop me
        }


#if defined(ETHRECEIVER_TEST_QUICKER_ONEVENT_RX_MODE)
        // marco.accame: testing a lut approach: from ip address directly to ethres pointer.

        if(incoming_msg_size > 0)
        {
            ethRes = ethManager->IPtoResource(sender_addr);
            if(NULL != ethRes)
            {
                if(false == ethRes->canProcessRXpacket(incoming_msg_data, incoming_msg_size))
                {   // cannot give packet to ethresource
                    yError() << "EthReceiver::run() cannot give a received packet of size" << incoming_msg_size << "to ethResources because ethResources::canProcessRXpacket() returns false.";
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
                sender_addr.addr_to_string(address, sizeof(address));
                yError() << "EthReceiver::run() cannot get a ethres associated to address" << address;
            }

        }
        else
        {   // a timeout. do nothing. because we must however execute what is after

        }

        // now i evaluate if we have to print the statistics
        if(statPrintInterval > 0)
        {
            double currTime = yarp::os::Time::now();
            double delta = currTime - statLastTime;

            if( (delta) > statPrintInterval )
            {
                statLastTime = currTime;

                yDebug() << "  (STATS-XX): new report for the past" << delta << "seconds";

                EthSender* ethSender = ethManager->getEthSender();
                ethSender->printTXstatistics();

                // now for every ethresource i print the stats ... but only if it is running.
                // by running stats for all boards at a given time, i understand if a board does not tx anymore

                ethManager->getEMSlistRiterators(_rBegin, _rEnd);

                riterator = _rBegin;
                while(riterator != _rEnd)
                {
                    ethRes = (*riterator);
                    if(ethRes->isRunning())
                    {
                        ethRes->printRXstatistics();
                    }
                    riterator++;
                }
            }
        }

#else // (ETHRECEIVER_TEST_QUICKER_ONEVENT_RX_MODE)

        // at every loop we get pointers of the ethresources list. we do so because it may change in time as a new device is added
        // we use reverse iterators.

        ethManager->getEMSlistRiterators(_rBegin, _rEnd);


        if(incoming_msg_size > 0)
        {   // process a valid packet

            sender_addr.addr_to_string(address, 64);
            riterator = _rBegin;

            // look for the ethresource associated to the ip address
            while(riterator != _rEnd)
            {
                ethRes = (*riterator);

                if(ethRes->getRemoteAddress() == sender_addr)
                {   // ok: i have found the relevant ethresource

                    if(false == ethRes->canProcessRXpacket(incoming_msg_data, incoming_msg_size))
                    {   // cannot give packet to ethresource
                        yError() << "EthReceiver::run() cannot give a received packet of size" << incoming_msg_size << "to ethResources because ethResources::canProcessRXpacket() returns false.";
                    }
                    else
                    {
                        ethRes->processRXpacket(incoming_msg_data, incoming_msg_size, true);
                    }

                    break;  // ok ... exit loop as i have found the correct ethresource
                }
                riterator++;
            }
        }
        else
        {   // a timeout. do nothing. because we must execute what is after

        }

        // now before repeating the loop we evaluate if a print of stats is required.


        // now i evaluate if we have to print the statistics
        if(statPrintInterval > 0)
        {
            double currTime = yarp::os::Time::now();
            double delta = currTime - statLastTime;

            if( (delta) > statPrintInterval )
            {
                statLastTime = currTime;

                yDebug() << "  (STATS-XX): new report for the past" << delta << "seconds";

                EthSender* ethSender = ethManager->getEthSender();
                ethSender->printTXstatistics();

                // now for every ethresource i print the stats ... but only if it is running.
                // by running stats for all boards at a given time, i understand if a board does not tx anymore
                riterator = _rBegin;
                while(riterator != _rEnd)
                {
                    ethRes = (*riterator);
                    if(ethRes->isRunning())
                    {
                        ethRes->printRXstatistics();
                    }
                    riterator++;
                }
            }
        }
#endif // (ETHRECEIVER_TEST_QUICKER_ONEVENT_RX_MODE)

    }   // end of while(!isStopping())


    return;
}

#else


void EthReceiver::run()
{
    yTrace();

    ACE_TCHAR     address[64];
    ethResources  *ethRes;
    ssize_t       incoming_msg_size;
    ACE_INET_Addr sender_addr;
    uint64_t      incoming_msg_data[ethResources::maxRXpacketsize/8]; // 8-byte aligned local buffer for incoming packet: it must be able to accomodate max size of packet
    const int     incoming_msg_capacity = ethResources::maxRXpacketsize;
    bool recError = false;



    ACE_Time_Value recvTimeOut;
    recvTimeOut.set(0.010f); // timeout of socket reception is 10 milliseconds

#ifdef ETHRECEIVER_STATISTICS_ON
    bool isFirst =true;
    double last_time, curr_time, diff;
    double before_rec, after_rec, diff_onRec;
    double before_mutex, after_mutex, diff_onMutex;
    int count;
    #define count_max 5000
#endif

    while(!isStopping())
    {   // forever loop... almost

#ifdef ETHRECEIVER_STATISTICS_ON
        before_rec = yarp::os::Time::now();
#endif
        // get pkt from socket: blocking call with timeout
        incoming_msg_size = recv_socket->recv((void *) incoming_msg_data, incoming_msg_capacity, sender_addr, 0, &recvTimeOut);

#ifdef ETHRECEIVER_STATISTICS_ON
        after_rec =  yarp::os::Time::now();
        diff_onRec = after_rec - before_rec;
        stat_onRecFunc->add((diff_onRec*1000));
#endif
        if(!isRunning())
        {
            continue;  // i go to recv a new pkt and wait someone to stop me
        }

#ifdef ETHRECEIVER_STATISTICS_ON
        before_mutex = yarp::os::Time::now();
#endif
        // take pointers to ems board list
        // new, reverse iterator
        ethResRIt    riterator, _rBegin, _rEnd;
        ethManager->getEMSlistRiterators(_rBegin, _rEnd);


#ifdef ETHRECEIVER_STATISTICS_ON
        after_mutex = yarp::os::Time::now();
        diff_onMutex = after_mutex - before_mutex;
        stat_onMutex->add((diff_onMutex*1000));
#endif

        // i get here because of timeout or because i have just received a packet
        // in both cases i check if all boards are alive. In the meanwhile i check if all ems are in config state.
         riterator = _rBegin;
// xxx-rem         bool allEmsInConfigstate = true;
         double curr_time = yarp::os::Time::now();
         while(riterator != _rEnd)
         {
             ethRes = (*riterator);
             if(ethRes->isRunning())
             {
                 ethRes->checkIsAlive(curr_time);
// xxx-rem                allEmsInConfigstate = true;
             }
             riterator++;
         }

        #warning --> marco.accame says: allEmsInConfigstate is always true ... thus i removed some code tagged with xxx-rem

        if(incoming_msg_size < 1)
        {

#if 0
// xxx-rem
            // print error if have not already done and if one or more ems boards are in running state , so thy should sent pkt every 1 msec
            if((!recError) && (!allEmsInConfigstate))
            {
                //if i'm here, i exited from recv because of timeout
                yError() << "EthReceiver: passed " << recvTimeOut.msec() << " ms without receive a pkt!!";
                recError = true;
            }
#endif

            continue; // try to receive again
        }
        else
        {
            recError = false;
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
                if(false == ethRes->canProcessRXpacket(incoming_msg_data, incoming_msg_size))
                {   // cannot give packet to ethresource
                    yError() << "EthReceiver::run() cannot give a received packet of size" << incoming_msg_size << "to ethResources because ethResources::canProcessRXpacket() returns false.";
                }
                else
                {
                    ethRes->processRXpacket(incoming_msg_data, incoming_msg_size);
                }
                break;  // ok ... exit loop as i have found the correct ethresource
            }
            riterator++;
        }

    }   // end of while(!isStopping())


    return;
}

#endif

#endif



#ifdef ETHRECEIVER_ISPERIODICTHREAD


void EthReceiver::run()
{

    //yTrace();

    ACE_TCHAR     address[64];
    ethResources  *ethRes;
    ssize_t       incoming_msg_size = 0;
    ACE_INET_Addr sender_addr;
    uint64_t      incoming_msg_data[ethResources::maxRXpacketsize/8]; // 8-byte aligned local buffer for incoming packet: it must be able to accomodate max size of packet
    const int     incoming_msg_capacity = ethResources::maxRXpacketsize;

//    static bool firstrun = true;
//    if(firstrun)
//    {
//        EthReceiver::setRate(5);
//        firstrun = false;
//    }



    //ACE_Time_Value recvTimeOut;
    //recvTimeOut.set(0.010f); // timeout of socket reception is 10 milliseconds

    //double myTestTimeout = recvTimeOut.sec() + (double)recvTimeOut.msec()/1000.0f;

    int flags = MSG_DONTWAIT;

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
                yError() << "EthReceiver::run() cannot give a received packet of size" << incoming_msg_size << "to ethResources because ethResources::canProcessRXpacket() returns false.";
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
            sender_addr.addr_to_string(address, sizeof(address));
            yError() << "EthReceiver::run() cannot get a ethres associated to address" << address;
        }


    }

}

#endif


#if 0
// marco.accame: it is the old ETHRECEIVER_ISPERIODICTHREAD
#define MAX_COUNT_STAT  120000
#define MAX_NUM_PKT     15
void EthReceiver::run()
{
//attention: don't insert too prints because this function is called every 1 millisec

    ACE_TCHAR     address[64];
    ethResources  *ethRes;
    ssize_t       incoming_msg_size;
    ACE_INET_Addr sender_addr;
    uint64_t      incoming_msg_data[ethResources::maxRXpacketsize/8]; // local buffer for incoming packet: it must be able to accomodate max size of packet
    const int     incoming_msg_capacity = ethResources::maxRXpacketsize;
    int flags = 0;
    static int countstat =0;
    int num_pkt;
    int myerr;




    ACE_Time_Value recvTimeOut;
    recvTimeOut.set(0.010f); // timeout of socket reception is 10 milliseconds

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



#ifdef ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_
    for(int i=0; i<TheEthManager::maxBoards; i++)
    {
        stats[i].resetStat();
    }
#endif


    flags |= MSG_DONTWAIT;

    while(num_pkt<MAX_NUM_PKT)
    {

        // per ogni msg ricevuto  -1 visto come 65535!!
        incoming_msg_size = recv_socket->recv((void *) incoming_msg_data, incoming_msg_capacity, sender_addr, flags);
        if(incoming_msg_size < 1)
        {
            //if i'm here socket input queue is empty
            return;
        }

        // new, reverse iterator
        ethResRIt    riterator, _rBegin, _rEnd;
        ethManager->getEMSlistRiterators(_rBegin, _rEnd);

        if( incoming_msg_size > 60000)
        {
            yWarning() << "Huge message received " << incoming_msg_size;
        }


#ifdef ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_
        int board = getBoardNum(sender_addr);
        int boardindex = board - 1;

        if(board > TheEthManager::maxBoards)
        {
            yError() << "detected board number is too big";
            boardindex = TheEthManager::maxBoards;
        }
        else if(0 == board)
        {
            yError() << "detected board number is ZERO";
            boardindex = TheEthManager::maxBoards
        }


        if(boardindex != TheEthManager::maxBoards)
            stats[boardindex].tickStart();
#endif




        //if i rec a pkt, then looking for the sender ems and parse the pkt
        sender_addr.addr_to_string(address, 64);
        riterator = _rBegin;

        while(riterator != _rEnd)
        {
            ethRes = (*riterator);
            if(ethRes->getRemoteAddress() == sender_addr)
            {
                //if(incoming_msg_size > ethRes->getRXpacketCapacity())
                if(false == ethRes->canProcessRXpacket(incoming_msg_data, incoming_msg_size))
                {
                    yError() << "EthReceiver got a message of wrong size ( received" << incoming_msg_size << " bytes while buffer is" << ethRes->getRXpacketCapacity() << " bytes long)";
                }
                else
                {
                    ethRes->processRXpacket(incoming_msg_data, incoming_msg_size);
                }
                break;
            }
            riterator++;
        }



#ifdef ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_

        if(boardindex != TheEthManager::maxBoards)
        {
            stats[boardindex].tickEnd();

            //compute statistics
            if (stats[boardindex].getIterations() == ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_NUMBEROF_CYCLES)
            {
                double avEst=0;
                double stdEst=0;
                double avUsed=0;
                double stdUsed=0;
                stats[boardindex].getEstPeriod(avEst, stdEst);
                stats[boardindex].getEstUsed(avUsed, stdUsed);
                printf("EthReceiver Thread [%d] run %d times, est period: %.3lf, +-%.4lf[ms], est used: %.3lf, +-%.4lf[ms]\n", board, stats[boardindex].getIterations(), avEst, stdEst, avUsed, stdUsed);
                stats[boardindex].resetStat();
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
        }
#endif
        num_pkt++;
    }
//    yError() << "Exiting recv thread";
    return;
}

// marco.accame: it is the endif of old ETHRECEIVER_ISPERIODICTHREAD
#endif


// eof




