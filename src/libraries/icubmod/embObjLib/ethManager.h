
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
#include "ethResource.h"

// embobj includes
#include "EoProtocol.h"

// others
#include "Debug.h"              // iCub debug class include
//#include "debugFunctions.h"   // marco.accame: removed as it seems not used.



#ifdef _STATS_DEBUG_FOR_CYCLE_TIME_
// Statistic debug
#warning --> marco.accame: now ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_ is used instead of _STATS_DEBUG_FOR_CYCLE_TIME_ and is locally defined
//#include "testStats.h"
#endif

// -- defines

#define ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_

#ifdef ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_
#define ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_NUMBEROF_CYCLES   2000
#include "testStats.h"
#endif


#if 0
// Actually there should be no need to include this class into yarp::dev namespace.
namespace yarp {
    namespace dev {
        class TheEthManager;
    }
}
#endif


using namespace yarp::os;
using namespace yarp::dev;
using namespace std;


class EthSender;
class EthReceiver;
class ethResource;

typedef std::list<ethResources *>::iterator ethResIt;
typedef std::list<ethResources *>::reverse_iterator ethResRIt;

// -------------------------------------------------------------------\\
//            TheEthManager   Singleton
// -------------------------------------------------------------------\\


class yarp::dev::TheEthManager: public DeviceDriver
{

public:
    // this is the maximum number of boards that the singleton can manage.
    // so far, this number depends also on the maximum capabilities of the protocol library.
    enum { maxBoards = eoprot_boards_maxnumberof };

    // this keeps the size of info buffer
    enum { ETHMAN_SIZE_INFO = 128 };


public:
    static yarp::os::Semaphore    managerMutex;

private:
    // Data for Singleton handling

    static TheEthManager          *handle;
    bool                          keepGoingOn;
    bool                          emsAlreadyClosed;
    double                        starttime;

    // Data for EMS handling
public:
    map<std::pair<FEAT_boardnumber_t, eOprotEndpoint_t>, FEAT_ID>  boards_map;         //!< Map of high level classes (referred to as Feature) using EMS, es eoMotionControl, eoSkin, eoAnalogSensor etc... Can be more the one for each EMS
    std::list<ethResources *>     EMS_list;           //!< List of pointer to classes that represent EMS boards
    ACE_INET_Addr                 local_addr;         

private:
    // Data for UDP socket handling
    bool                          UDP_initted;
    ACE_SOCK_Dgram                *UDP_socket;


    EthSender                     *sender;            //!< class used to send messages to the EMSs every 1ms. Derived from RathThread
    EthReceiver                   *receiver;          //!< class handling data coming from EMSs with a blocking recv mechanism. Derived rom Thread

    // Data for Debug or support
    char                          info[ETHMAN_SIZE_INFO];

    // Methods for Singleton handling#include <yarp/os/Bottle.h>
private:

    /*! @fn     flush();
     *  @brief  flush all packets not yet sent
     *  @warning currently is developed like a wait of 1 second. TODO. make it better!!
     */
    void flush();
    TheEthManager();                      // Singletons have private constructor
    ~TheEthManager();
public:

    double getStartTime(void);

    void initEOYsystem(void);

    /*! @fn     static  TheEthManager* instance();
     *  @brief  Create the Singleton if it doesn't exists yet and return the pointer.
     *  @return Pointer to TheEthManager singleton
     */
    static  TheEthManager* instance();

    /*! @fn     static  bool killYourself();
     *  @brief  Destroy the Singleton
     *  @return True if ok, false if errors or cannot kill yet.
     */
    static bool killYourself();

    /*! @fn     bool isInitted(void);
     *  @brief  Tells if the Signleton is correctly initialized
     *  @return True if initted, false otherwise
     */
    bool isInitted(void);
    bool open(void);
    bool stopThreads(void);
    bool close(void);


    /*! @fn     ethResources* requestResource(FEAT_ID request);
     *  @brief  Get the pointer to a specific EMS board. If it doesn't exists yet it'll be created.
     *  @param  config  Description and parameter for the identifying the requested class
     *  @return Pointer to the requested EMS, NULL if errors arise in the creation.
     */
    ethResources* requestResource(yarp::os::Searchable &cfgtotal, yarp::os::Searchable &cfgtransceiver, yarp::os::Searchable &cfgprotocol, FEAT_ID *request);

    /*! @fn     bool releaseResource(FEAT_ID resouce);
     *  @brief  Tells the manager the specified resource is not used anymore by the caller,
     *          therefore the manager could decide to delete it if no used by any other one.
     *  @param  config  Description and parameter identifying the releasing class
     *  @return True if ok, false in case of errors.
     */
    int releaseResource(FEAT_ID resource);

private:
    /*! @fn     void addLUTelement(FEAT_ID id);
     *  @brief  Insert a ethResource class descriptor of FEAT_ID type in a map to easy the access from the embObj callbacks
     *  @param  id  A struct of FEAT_ID type with useful information about the class requesting an ethResource, they can be eoMotionControl, eoSkin, eoAnalogSensor...
     */
    void addLUTelement(FEAT_ID *id);

    /*! @fn     bool removeLUTelement(FEAT_ID element);
     *  @brief  Remove a ethResource class descriptor of FEAT_ID type from the map used by the callbacks.
     *  @param  id  A struct of FEAT_ID type with information about the class to be removed.
     *  @return True if everything went as expected, false if class not found or more than one were removed.
     */
    bool removeLUTelement(FEAT_ID element);

public:
    /*! @fn     void *getHandleFromEP(eOnvEP_t ep);
     *  @brief  Get a pointer to the class handling the specified EndPoint
     *  @param  boardnum  in range [1, max]
     *  @param  ep  The desired EndPoint
     *  @return Pointer to the class, casted to a portable void type. The user must cast it to the correct, expected type like eoMotionControl ecc..
     */
    void *getHandle(FEAT_boardnumber_t boardnum, eOprotEndpoint_t ep);

    /*! @fn     FEAT_ID getFeatInfoFromEP(eOnvEP_t ep);
     *  @brief  Get the struct of FEAT_ID type with useful information about the class handling the desired EndPoint.
     *  @param  boardnum the board number in range [1, max]
     *  @param  ep  The desired EndPoint
     *  @return std::list<ethResources *>Struct with info
     */
    FEAT_ID getFeatInfo(FEAT_boardnumber_t boardnum, eOprotEndpoint_t ep);

    // Methods for UDP socket handling
private:
    /*! @fn     bool createSocket(ACE_INET_Addr local_addr);
     *  @brief  Create the UDP socket if it doesn't already exists.
     *  @param  local_addr  The IP address and port to be used for the local machine (pc104) in ACE_INET_Addr form
     *  @return True if creation went well, or socket already created, false if errors
     */
    bool createSocket(ACE_INET_Addr local_addr);

public:
    /*! @fn     int send(void *data, size_t len, ACE_INET_Addr remote_addr);
     *  @brief  Send a message to the EMSs
     *  @param  data  pointer to the data to be sent
     *  @param  len   number of bytes to be sent
     *  @param  remote_addr  destination address in ACE format
     *  @return Number of bytes actually sent
     */
    int send(void *data, size_t len, ACE_INET_Addr remote_addr);

    ethResources* GetEthResource(FEAT_boardnumber_t boardnum);

    // Methods for Debug or support

};


// -------------------------------------------------------------------\\
//            EthSender
// -------------------------------------------------------------------\\

class EthSender : public yarp::os::RateThread
{
private:
    uint8_t                       *p_sendData;
    TheEthManager                 *ethManager;
    ACE_SOCK_Dgram                *send_socket;
    std::list<ethResources *>     *ethResList;
    void run();

#ifdef ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_
    // for statistic debug purpose
    Stats stats;
    Port statsPort;
#endif

public:
    EthSender();
    bool config(ACE_SOCK_Dgram *pSocket, TheEthManager* _ethManager);
    bool threadInit();
};

// -------------------------------------------------------------------\\
//            EthReceiver
// -------------------------------------------------------------------\\

#ifdef ETHRECEIVER_ISPERIODICTHREAD
class EthReceiver : public yarp::os::RateThread
#else
class EthReceiver : public yarp::os::Thread
#endif
{
private:
    // the 8-byte aligned buffer containing a received packet. it must be able to hold maximum size of packet managed by ethResource
    uint64_t                        recvBuffer[ethResources::maxRXpacketsize/8];
    ACE_SOCK_Dgram                  *recv_socket;
    TheEthManager                   *ethManager;
    std::list<ethResources *>       *ethResList;
    uint64_t                        seqnumList[TheEthManager::maxBoards];
    bool                            recFirstPkt[TheEthManager::maxBoards];


#ifdef ETHRECEIVER_STATISTICS_ON
    StatExt                         *stat;
    StatExt                         *stat_onRecFunc;
    StatExt                         *stat_onMutex;
#endif

#ifdef ETHRECEIVER_ISPERIODICTHREAD
    int count;
    bool isFirst;
#endif

#ifdef ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_
    // for statistic debug purpose
    Stats stats[TheEthManager::maxBoards];
    Port statsPort;
#endif

    int getBoardNum(ACE_INET_Addr addr); //return board number from address (returns 0 in case of error)
    void checkPktSeqNum(char* pktpayload, ACE_INET_Addr addr);
public:
    EthReceiver();
    ~EthReceiver();
    bool config(ACE_SOCK_Dgram *pSocket, TheEthManager* _ethManager);
    bool threadInit();
    void run();
    void onStop();
};

#endif

// eof






