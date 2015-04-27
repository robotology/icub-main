
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


#undef  ETHRECEIVER_ISPERIODICTHREAD
#undef  ETHRECEIVER_TEST_QUICKER_ONEVENT_RX_MODE


#define ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_

#ifdef ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_
#define ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_NUMBEROF_CYCLES   2000
#include "testStats.h"
#endif


#if 0
// marco.accame: actually there should be no need to include this class into yarp::dev namespace.
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
//class ethResource;


typedef std::list<ethResources *>::iterator ethResIt;
typedef std::list<ethResources *>::reverse_iterator ethResRIt;

// -------------------------------------------------------------------\\
//            TheEthManager   Singleton
// -------------------------------------------------------------------\\


class yarp::dev::TheEthManager: public DeviceDriver
{

public:
    // this is the maximum number of boards that the singleton can manage.
    enum { maxBoards = 32 };

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

    enum { ethresLUTsize = TheEthManager::maxBoards };

    ethResources*                 ethresLUT[ethresLUTsize];
    int                           numberOfUsedBoards;


    // Data for EMS handling
public:
    map<std::pair<FEAT_boardnumber_t, eOprotEndpoint_t>, ethFeature_t>  boards_map;         //!< Map of high level classes (referred to as Feature) using EMS, es eoMotionControl, eoSkin, eoAnalogSensor etc... Can be more the one for each EMS
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

    bool getEMSlistRiterators(ethResRIt& begin, ethResRIt& end);

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


    /*! @fn     ethResources* requestResource(ethFeature_t request);
     *  @brief  Get the pointer to a specific EMS board. If it doesn't exists yet it'll be created.
     *  @param  config  Description and parameter for the identifying the requested class
     *  @return Pointer to the requested EMS, NULL if errors arise in the creation.
     */
    ethResources* requestResource(yarp::os::Searchable &cfgtotal, yarp::os::Searchable &cfgtransceiver, yarp::os::Searchable &cfgprotocol, ethFeature_t &request);

    /*! @fn     bool releaseResource(ethFeature_t resouce);
     *  @brief  Tells the manager the specified resource is not used anymore by the caller,
     *          therefore the manager could decide to delete it if no used by any other one.
     *  @param  config  Description and parameter identifying the releasing class
     *  @return True if ok, false in case of errors.
     */
    int releaseResource(ethFeature_t &resource);

private:
    /*! @fn     void addLUTelement(ethFeature_t id);
     *  @brief  Insert a ethResource class descriptor of ethFeature_t type in a map to easy the access from the embObj callbacks
     *  @param  id  A struct of ethFeature_t type with useful information about the class requesting an ethResource, they can be eoMotionControl, eoSkin, eoAnalogSensor...
     */
    void addLUTelement(ethFeature_t &id);

    /*! @fn     bool removeLUTelement(ethFeature_t element);
     *  @brief  Remove a ethResource class descriptor of ethFeature_t type from the map used by the callbacks.
     *  @param  id  A struct of ethFeature_t type with information about the class to be removed.
     *  @return True if everything went as expected, false if class not found or more than one were removed.
     */
    bool removeLUTelement(ethFeature_t &element);

public:
    /*! @fn     getHandle(FEAT_boardnumber_t boardnum, eOprotID32_t id32, IethResource * interfacePointer, ethFeatType_t *type);
     *  @brief  Get a infos about the class handling the specified network variable
     *  @param  boardnum  in range [1, max]
     *  @param  id32  The desired network variable id
     *  @param  interfacePointer return the pointer to the IethResource of that device class
     *  @param  type  return the type of the class (skin, analog, motionControl ... ) using the enum type ethFeatType_t
     *  @return Pointer to the class, casted to a portable void type. The user must cast it to the correct, expected type like eoMotionControl ecc..
     */
    bool getHandle(FEAT_boardnumber_t boardnum, eOprotID32_t id32, IethResource **interfacePointer, ethFeatType_t *type);

#if 0
    /*! @fn     ethFeature_t getFeatInfoFromEP(eOnvEP_t ep);
     *  @brief  Get the struct of ethFeature_t type with useful information about the class handling the desired EndPoint.
     *  @param  boardnum the board number in range [1, max]
     *  @param  ep  The desired EndPoint
     *  @return std::list<ethResources *>Struct with info
     */
    ethFeature_t getFeatInfo(FEAT_boardnumber_t boardnum, eOprotEndpoint_t ep);
#endif


    // Methods for UDP socket handling
private:
    /*! @fn     bool createSocket(ACE_INET_Addr local_addr);
     *  @brief  Create the UDP socket if it doesn't already exists.
     *  @param  local_addr  The IP address and port to be used for the local machine (pc104) in ACE_INET_Addr form
     *  @return True if creation went well, or socket already created, false if errors
     */
    bool createSocket(ACE_INET_Addr local_addr);

    bool lock();
    bool unlock();

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


    EthSender* getEthSender(void);
    EthReceiver* getEthReceiver(void);

    ethResources* IPtoResource(ACE_INET_Addr adr);
    int GetNumberOfUsedBoards(void);

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
    void run();

    enum { EthSenderRate = 1 };

#ifdef ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_
    // for statistic debug purpose
    Stats stats;
    Port statsPort;
#endif

public:
    EthSender();
    bool config(ACE_SOCK_Dgram *pSocket, TheEthManager* _ethManager);
    bool threadInit();
    void evalPrintTXstatistics(void);
    void printTXstatistics(void);
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
    uint64_t                        seqnumList[TheEthManager::maxBoards];
    bool                            recFirstPkt[TheEthManager::maxBoards];

    double                          statPrintInterval;

    enum { EthReceiverRate = 1 };


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






