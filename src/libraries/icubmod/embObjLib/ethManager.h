
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules
 * \defgroup TheEthManager TheEthManager
 *
*/
/* Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Alberto Cardellino
 * email: alberto.cardellino@iit.it
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

//
// $Id: TheEthManager.h,v 1.5 2008/06/25 22:33:53 nat Exp $
//
//

#ifndef __ethManager__
#define __ethManager__

// Standard includes
#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <stdio.h>
#include <map>


// ACE includes
#include <ace/ACE.h>
#include <ace/config.h>
#include <ace/Thread.h>
#include <ace/SOCK_Dgram_Bcast.h>
#include "ace/OS_main.h"
#include "ace/OS_NS_string.h"
#include "ace/OS_NS_sys_socket.h"

// YARP includes
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>

// embobjlib includes
#include "hostTransceiver.hpp"
#include "FeatureInterface.h"
#include "IethResource.h"
#include "ethResource.h"

// embobj includes
#include "EoProtocol.h"

// others
#include <yarp/os/LogStream.h>              // iCub debug class include




// -- defines

// the check of sequence number is done inside the hostTransceiver. by defining this macro there is an additional redundant check.
// i recommend to keep it undefined.
#undef ETHMANAGER_RECEIVER_CHECK_SEQUENCE_NUMBER

// marco.accame: the following two are for new tx and rx modes with different protection vs concurrency
#define ETHMANAGER_TEST_NEW_SENDER_RUN
#define ETHMANAGER_TEST_NEW_RECEIVER_RUN


//#define ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_
//#ifdef ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_
//enum { ethmanager_stats_frequency_numberofcycles = 2000 };
//#include "testStats.h"
//#endif


using namespace yarp::os;
using namespace yarp::dev;
using namespace std;


// -- class EthBoards
// -- it collects all the ETH boards managed by ethManager.
// -- each board surely has an EthResource object associated to it. and it may have one or more interfaces which use the
// -- services of EthResource to transmit or receive.
// -- it is responsibility of the object which owns EthBoards (it is ethManager) to protect the class EthBoards vs concurrent use.
// -- examples of concurrent use are: transmit or receive using an ethresource and ... attempting to create or destroy a resource.


class EthBoards
{

public:

    enum { maxEthBoards = 32 };

public:

    EthBoards();
    ~EthBoards();

    size_t number_of_resources(void);
    bool add(EthResource* res);
    EthResource* get_resource(eOipv4addr_t ipv4);
    bool rem(EthResource* res);

    size_t number_of_interfaces(EthResource* res);
    bool add(EthResource* res, IethResource* interface);
    IethResource* get_interface(eOipv4addr_t ipv4, eOprotID32_t id32);
    bool rem(EthResource* res, iethresType_t type);

    // the name of the board
    const char * name(eOipv4addr_t ipv4);

    // executes an action on all EthResource which have been added in the class.
    bool execute(void (*action)(EthResource* res, void* p), void* par);

    // executes an action on the ethResource having a specific ipv4.
    bool execute(eOipv4addr_t ipv4, void (*action)(EthResource* res, void* p), void* par);


private:

    // private types

    enum { ethboardNameMaxSize = EthResource::boardNameSize };

    typedef struct
    {
        eOipv4addr_t        ipv4;
        char                name[EthBoards::ethboardNameMaxSize];
        uint8_t             numberofinterfaces;
        uint8_t             boardnumber;
        EthResource*        resource;
        IethResource*       interfaces[iethresType_numberof];
    } ethboardProperties_t;


private:

    // private variables

    static const char * defaultnames[EthBoards::maxEthBoards];
    static const char * errorname[1];

    int sizeofLUT;
    ethboardProperties_t LUT[EthBoards::maxEthBoards];
};



// -- class TheEthManager
// -- it is the main singleton which delas with eth communication.
// -- it holds the two tx and rx threads (classes EthSender and EthReceiver)
// -- it holds class EthBoards which stores references to EthResource (what is used to form / parse UDP packets for the eth board)
// -- and to the interfaces which use the EthResource of a given board.

// forward declaration because they are used inside TheEthManager
class EthSender;
class EthReceiver;

class yarp::dev::TheEthManager: public DeviceDriver
{

public:
    // this is the maximum number of boards that the singleton can manage.
    enum { maxBoards = EthBoards::maxEthBoards };

    // these are the boards, their use is protected by txSem or rxSem or both of them.
    EthBoards* ethBoards;

private:

    // singletons have private constructor / destructor
    TheEthManager();
    ~TheEthManager();

public:

    double getTimeOfStartUp(void);

    void initEOYsystem(void);

    static  TheEthManager* instance();

    static bool killYourself();


    bool isCommunicationInitted(void);
    bool open(void);
    bool stopCommunicationThreads(void);
    bool close(void);


    bool startCommunication(yarp::os::Searchable &cfgtotal);

    EthResource* requestResource2(IethResource *interface, yarp::os::Searchable &cfgtotal);


    int releaseResource(ethFeature_t &resource);

    const ACE_INET_Addr& getLocalIPaddress(void);

    const eOipv4addressing_t & getLocalIPV4addressing(void);

    bool Transmission(void);
    bool Reception(ACE_INET_Addr adr, uint64_t* data, ssize_t size, bool collectStatistics);

    EthResource* GetEthResource(eOipv4addr_t ipv4);


    IethResource* getInterface(eOipv4addr_t ipv4, eOprotID32_t id32);


    //EthSender* getEthSender(void);
    //EthReceiver* getEthReceiver(void);

    EthResource* IPtoResource(ACE_INET_Addr adr);
    int IPtoBoardNumber(ACE_INET_Addr adr); // non-zero value: 1, 2, ......

    int GetNumberOfUsedBoards(void);

    const char * getName(eOipv4addr_t ipv4);

    int send(void *udpframe, size_t len, ACE_INET_Addr toaddress);

private:

    // this semaphore is used to ....
    static yarp::os::Semaphore    managerSem;
    // the following two semaphore are used separately or together to stop tx and rx if a change is done on ethboards (in startup and shutdown phases)
    static yarp::os::Semaphore    txSem;
    static yarp::os::Semaphore    rxSem;

    static TheEthManager*         handle;

    // contains the start-up time of teh system so that time measures / prints can be done relative.
    double                        startUpTime;

    ACE_INET_Addr                   localIPaddress;
    eOipv4addressing_t              ipv4local;

    bool                            communicationIsInitted;

    // periodic threads which use methods of class TheEthManager to transmit / receive + the udp socket
    EthSender                       *sender;
    EthReceiver                     *receiver;
    ACE_SOCK_Dgram                  *UDP_socket;


private:

    void flush();

    bool createSocket(ACE_INET_Addr local_addr, int txrate, int rxrate);

    bool lock(bool on);

    bool lockTX(bool on);
    bool lockRX(bool on);
    bool lockTXRX(bool on);

};


// -- class EthSender
// -- it is a rate thread created by singleton TheEthManager. it regularly transmits packets (if any available) to the eth boards.
// -- it uses methods made available by TheEthManager.

class EthSender : public yarp::os::RateThread
{
private:
    int rateofthread;

    uint8_t                       *p_sendData;
    TheEthManager                 *ethManager;
    ACE_SOCK_Dgram                *send_socket;
    void run();


//#ifdef ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_
//    // for statistic debug purpose
//    Stats stats;
//    Port statsPort;
//#endif

public:

    enum { EthSenderDefaultRate = 1, EthSenderMaxRate = 20 };

    EthSender(int txrate);
    ~EthSender();
    bool config(ACE_SOCK_Dgram *pSocket, TheEthManager* _ethManager);
    bool threadInit();

//    void evalPrintTXstatistics(void);
//    void printTXstatistics(void);
};


// -- class EthReceiver
// -- it is a rate thread created by singleton TheEthManager.
// -- it regularly wakes up to see if a packet is in its listening socket and it parses that with methods made available by TheEthManager.

class EthReceiver : public yarp::os::RateThread
{
private:
    int rateofthread;

    ACE_SOCK_Dgram                  *recv_socket;
    TheEthManager                   *ethManager;
    double                          statPrintInterval;


//#ifdef ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_
//    // for statistic debug purpose
//    Stats stats[TheEthManager::maxBoards];
//    Port statsPort;
//#endif

#if defined(ETHMANAGER_RECEIVER_CHECK_SEQUENCE_NUMBER)
    void checkPktSeqNum(char* pktpayload, ACE_INET_Addr addr);
    uint64_t seqnumList[TheEthManager::maxBoards];
    bool recFirstPkt[TheEthManager::maxBoards];
#endif

public:

    enum { EthReceiverDefaultRate = 5, EthReceiverMaxRate = 20 };

    EthReceiver(int rxrate);
    ~EthReceiver();
    bool config(ACE_SOCK_Dgram *pSocket, TheEthManager* _ethManager);
    bool threadInit();
    void run();
    void onStop();
};



#endif

// eof






