
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

// do we need it ???
//#define WIN32_LEAN_AND_MEAN 


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

// Emb Obj includes
#include "hostTransceiver.hpp"
#include "debugFunctions.h"
#include "FeatureInterface.h"
#include "EoProtocol.h"

// marco.accame on 11 apr 2014:
// really needed?
#include "FeatureInterface_hid.h"


// iCub debug class include
#include "Debug.h"

#ifdef _STATS_DEBUG_FOR_CYCLE_TIME_
// Statistic debug
#include "testStats.h"
#endif


#define ETHMAN_SIZE_INFO            128



// sizes of rx and tx buffers. 
// the rx buffer must be able to accepts udp packets of max size (1500), so that we are safe against changes 
// of tx size done inside the ems boards. 
// the tx buffer must be able to contain the maximum payload size managed in reception inside the ems board.
// this value is EOK_HOSTTRANSCEIVER_capacityofpacket. however, 1500 is good enough, as

#if defined(_WIP_CHECK_PROTOCOL_VERSION_)
// new one will be: 
enum {rxBUFFERsize = 1500, txBUFFERsize = 1500};
#else
// current one is:
enum {rxBUFFERsize = EOK_HOSTTRANSCEIVER_capacityofrxpacket, txBUFFERsize = EOK_HOSTTRANSCEIVER_capacityoftxpacket};
#endif

// Actually there sould be no need to include this class into yarp::dev namespace.
namespace yarp {
    namespace dev {
        class TheEthManager;
    }
}

#include "ethResource.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;


class EthSender;
class EthReceiver;

typedef std::list<ethResources *>::iterator ethResIt;
typedef std::list<ethResources *>::reverse_iterator ethResRIt;

// -------------------------------------------------------------------\\
//            TheEthManager   Singleton
// -------------------------------------------------------------------\\


class yarp::dev::TheEthManager: public DeviceDriver
{
public:
    static yarp::os::Semaphore    managerMutex;

private:
    // Data for Singleton handling

    static TheEthManager          *handle;
    bool                          keepGoingOn;
    bool                          emsAlreadyClosed;

    // Data for EMS handling
public:
//    int                           nBoards;            //!< Number of EMS instantiated
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

    // Methods for EMS handling
//     /*! @fn     ethResources* getResource(yarp::os::Searchable &config);
//      *  @brief  Get the pointer to a specific EMS board. If it doesn't exists yet it'll be created.
//      *  @param  config  Description and parameter for the class in yarp Searchable format
//      *  @return Pointer to the requested EMS, NULL if ;errors arise in the creation.
//      */
//     ethResources* getResource(yarp::os::Searchable &config);
// 
//     /*! @fn     bool removeResource(ethResources* to_be_removed);
//      *  @brief  Remove an ethResources class from the list.
//      *  @param  to_be_removed  Pointer to the class to be removed. The actual check is done looking at its IP address
//      *  @return True if everything was ok, false if asked to remove a non existing class
//      */
//     bool removeResource(ethResources* to_be_removed);

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
    // buffer storing data to be transmetted
    uint8_t                       sendBuffer[txBUFFERsize];
    uint8_t                       *p_sendData;
    TheEthManager                 *ethManager;
    ACE_SOCK_Dgram                *send_socket;
    std::list<ethResources *>     *ethResList;
    void run();

#ifdef _STATS_DEBUG_FOR_CYCLE_TIME_
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
    uint8_t                         recvBuffer[rxBUFFERsize];
    ACE_SOCK_Dgram                  *recv_socket;
    TheEthManager                   *ethManager;
    std::list<ethResources *>       *ethResList;
    uint64_t                        seqnumList[10];
    bool                            recFirstPkt[10];


#ifdef ETHRECEIVER_STATISTICS_ON
    StatExt                         *stat;
    StatExt                         *stat_onRecFunc;
    StatExt                         *stat_onMutex;
#endif

#ifdef ETHRECEIVER_ISPERIODICTHREAD
    int count;
    bool isFirst;
#endif

#ifdef _STATS_DEBUG_FOR_CYCLE_TIME_
    // for statistic debug purpose
    Stats stats[10];
    Port statsPort;
#endif

    int getBoardNum(ACE_INET_Addr addr);//return board num from address (returns 0 in case of error)
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






