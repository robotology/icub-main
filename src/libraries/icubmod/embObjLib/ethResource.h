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

#ifndef __ethResource__
#define __ethResource__

// System includes
#include <stdio.h>
#include <iostream>
#include <vector>
#include <list>
#include <string>

// Yarp includes
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include "statExt.h"

// embobjlib includes
#include "hostTransceiver.hpp"
#include "FeatureInterface.h"
#include "IethResource.h"

// embobj includes
#include "EoProtocol.h"


// ACE stuff
#include <ace/ACE.h>
#include <ace/config.h>
#include <ace/SOCK_Dgram_Bcast.h>


typedef struct
{
    uint16_t    port;
    int         ip1;
    int         ip2;
    int         ip3;
    int         ip4;
    char        string[64];
} ethFeatIPaddress_t;

typedef enum
{
    ethFeatType_NULL            = 0xFF,
    ethFeatType_Management      = 0x00,
    ethFeatType_AnalogMais      = 0x01,
    ethFeatType_AnalogStrain    = 0x02,
    ethFeatType_MotionControl   = 0x03,
    ethFeatType_Skin            = 0x04,
    ethFeatType_AnalogVirtual   = 0x05,
    ethFeatType_AnalogInertial  = 0x06
} ethFeatType_t;

#define BOARDNAME_MAXSIZE 50
typedef struct
{
    ethFeatIPaddress_t  pc104IPaddr;
    ethFeatIPaddress_t  boardIPaddr;
    FEAT_boardnumber_t  boardNumber;
    eOprotEndpoint_t    endpoint;
    eOprotEntity_t      entity;
    ethFeatType_t       type;
    IethResource*       interface;
    char                name[128];
    char                boardName[BOARDNAME_MAXSIZE]; //this is the name read in xml file
} ethFeature_t;

class can_string_eth;

namespace yarp{
    namespace dev{
        class ethResources;
        class TheEthManager;
    }
}

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev;
using namespace std;


//this class contains all info about pkts received from a specific ems (its id is stored in "board" field)
//and let you calculate statistic about receive timing and check if one of following errors occur:
// 1) error in sequence number
// 2) between two consecutive pkts there are more the "timeout" sec (default is 0.01 sec and this is can modified by environment var ETHREC_STATISTICS_TIMEOUT_MSEC):
      //this elapsed time is calculated in two way:
      //1) using sys time: when arrived a pkt number n its received time(get by sys call) is compared with received time of pkt number n-1
      //2) using ageOfFrame filed in packets: this time is insert by transmitter board. when arrive a pkt number n its is compared with ageOfFrme of pkt n-1.
      //   if there are more then timeout sec it mean that board did not transmit for this period.
class infoOfRecvPkts
{

private:
    enum { DEFAULT_MAX_COUNT_STAT = 60000 }; // 1 minute expressed in ms
    const double DEFAULT_TIMEOUT_STAT;

private:
    int             board;              // the number of ems board with range [1, TheEthManager::maxBoards]
public:
    bool            initted;          //if true is pc104 has already received a pkt from the ems when it is in running mode
    bool            isInError;        //is true if an error occur. it is used to avoid to print error every cicle
    int             count;
    int             max_count;        //every max_count received pkts statistics are printed. this number can be modified by environment variable ETHREC_STATISTICS_COUNT_RECPKT_MAX
    double          timeout;          //is expressed in sec. its default val is 0.01 sec, but is it can modified by environment var ETHREC_STATISTICS_TIMEOUT_MSEC

    double          reportperiod;
    double          timeoflastreport;

    long            receivedPackets;
    uint64_t        last_seqNum;      //last seqNum rec
    uint64_t        last_ageOfFrame;  //value of ageOfFrame filed of last rec pkt
    double          last_recvPktTime; //time of last recv pkt

    int             totPktLost;       //total pkt lost from start up
    int             currPeriodPktLost; // total pkt lost during thsi period
    StatExt         *stat_ageOfFrame; //period between two packets calculating using ageOfFrame filed of ropframe. values are stored in millisec
    StatExt         *stat_periodPkt;  //delta time between two consegutivs pkt; time is calculating using pc104 systime. Values are stored in sec
    StatExt         *stat_printstatPktgap;
    StatExt         *stat_lostPktgap;
    StatExt         *stat_precessPktTime; //values are stored in sec
    StatExt         *stat_pktSize; // rx pkt size
    bool            _verbose;
    bool            justprinted;

    infoOfRecvPkts();
    ~infoOfRecvPkts();
    void printStatistics(void);
    void clearStatistics(void);
    uint64_t getSeqNum(uint64_t *packet, uint16_t size);
    uint64_t getAgeOfFrame(uint64_t *packet, uint16_t size);

    /*!   @fn       void updateAndCheck(uint8_t *packet, double reckPktTime, double processPktTime);
     *    @brief    updates communication info statistics and checks if there is a transmission error with this ems board.In case of error print it.
     *    @param    packet           pointer to received packet
     *    @param    reckPktTime      sys time on rec pkt (expressed in seconds)
     *    @param    processPktTime   time needed to process this packet (expressed in seconds)
     */
    void updateAndCheck(uint64_t *packet, uint16_t size, double reckPktTime, double processPktTime, bool evalreport = false);

    void evalReport(void);

    void forceReport(void);

    void setBoardNum(int board);
};

class ethNetworkQuery
{

private:

    eOprotID32_t id2wait;           // the id32 which we wait
    yarp::os::Semaphore* netwait;   // the semaphore used by the class to wait for a reply from network. it must have exclusive access
    yarp::os::Semaphore* isbusy;    // the semaphore used to guarantee exclusive access of ::netwait to calling threads
    yarp::os::Semaphore* iswaiting; // the semaphore used to guarantee that the wait of the class is unblocked only once and when it is required

public:

    ethNetworkQuery();
    ~ethNetworkQuery();

    Semaphore* start(eOprotID32_t id32, uint32_t signature);     // associates a semaphore to a (id2wait, signature) pair
    bool wait(Semaphore* sem, double timeout);                      // waits the semaphore until either a reply arrives or timeout expires (returns false)
    bool arrived(eOprotID32_t id32, uint32_t signature);            // a reply has arrived. the rx handler must call it. true if it releases, false if not
    bool stop(Semaphore* sem);                                      // we deassociate the semaphore to the waiting pair
};


class yarp::dev::ethResources:  public DeviceDriver,
                                public hostTransceiver
{
public:

    // this is the maximum size of rx and tx packets managed by the ethresource.
    enum { maxRXpacketsize = 1496, maxTXpacketsize = 1496 };

private:

    enum { ETHRES_SIZE_INFO = 128 };

    char              info[ETHRES_SIZE_INFO];
    int               how_many_features;      //!< Keep track of how many high level class registered. onto this EMS
    ACE_INET_Addr     remote_dev;             //!< IP address of the EMS this class is talking to.
    double            lastRecvMsgTimestamp;   //! stores the system time of the last received message, gettable with getLastRecvMsgTimestamp()
    bool			  isInRunningMode;        //!< say if goToRun cmd has been sent to EMS
    infoOfRecvPkts    *infoPkts;

    yarp::os::Semaphore*  objLock;

    ethNetworkQuery*    ethQuery;

    ethNetworkQuery*    ethQueryServices;

//    yarp::os::Semaphore*  networkQuerySem;      // the semaphore used by the class to wait for a reply from network. it must have exclusive access
//    yarp::os::Semaphore*  isbusyNQsem;          // the semaphore used to guarantee exclusive access of networkQuerySem to calling threads
//    yarp::os::Semaphore*  iswaitingNQsem;       // the semaphore used to guarantee that the wait of the class is unblocked only once and when it is required

    bool lock();
    bool unlock();

    bool                verifiedEPprotocol[eoprot_endpoints_numberof];
    bool                configuredEP[eoprot_endpoints_numberof];
    bool                verifiedBoardPresence;
    bool                remoteBoardNumberIsSet;
    bool                verifiedBoardTransceiver; // transceiver capabilities (size of rop, ropframe, etc.) + MN protocol version
    bool                txrateISset;
    bool                cleanedBoardBehaviour;    // the board is in config mode and does not have any regulars
    uint8_t             boardEPsNumber;
    eOmn_comm_status_t  boardCommStatus;
    uint16_t            usedNumberOfRegularROPs;
    //uint16_t            usedSizeOfRegularROPframe;

    can_string_eth*     c_string_handler[16];

public:
    TheEthManager     *ethManager;          //!< Pointer to the Singleton handling the UDP socket
    int               boardNum;             // the number of ems board with range [1, TheEthManager::maxBoards]
    char              boardName[BOARDNAME_MAXSIZE];        //this is the name read in xml file
    ethResources();
    ~ethResources();


    bool            open(yarp::os::Searchable &cfgtotal, yarp::os::Searchable &cfgtransceiver, yarp::os::Searchable &cfgprotocol, ethFeature_t &request);
    bool            close();

    /*!   @fn       registerFeature(void);
     *    @grief    tells the EMS a new device requests its services.
     *    @return   true.
     */
    bool            registerFeature(ethFeature_t &request);

    /*!   @fn       unregisterFeature();
     *    @brief    tell the EMS a user has been closed. If was the last one, close the EMS
     */
    int             deregisterFeature(ethFeature_t &request);

    ACE_INET_Addr   getRemoteAddress(void);

    // the function returns true if the packet can be transmitted. 
    // it returns false if it cannot be transmitted: either it is with no rops inside in mode donttrxemptypackets, or there is an error somewhere
    bool            getPointer2TxPack(uint8_t **pack, uint16_t *size, uint16_t *numofrops);


    // returns the capacity of the receiving buffer.
    int             getRXpacketCapacity();


    bool            canProcessRXpacket(uint64_t *data, uint16_t size);

    void            processRXpacket(uint64_t *data, uint16_t size, bool collectStatistics = true);


    /*!   @fn       goToRun(eOprotEndpoint_t endpoint, eOprotEntity_t entity);
     *    @brief    Tells the EMS to start the control loop to activate a given service described by (endpoint, entity).
     */
    bool            goToRun(eOprotEndpoint_t endpoint, eOprotEntity_t entity);


    /*!   @fn       goToConfig(void);
     *    @brief    Tells the EMS to stop the control loop and go into configuration mode.
     */
    bool            goToConfig(void);

    /*!   @fn       getLastRecvMsgTimestamp(void);
     *    @brief    return the system time of the last received message from the corresponding EMS board.
     *    @return   seconds passed from the very first message received.
     */
    double          getLastRecvMsgTimestamp(void);

    /*!   @fn       clearRegulars(void);
     *    @brief    clears periodic signal message of EMS
     *    @return   true on success else false
     */
    bool            clearRegulars(bool verify = false);

    bool addRegulars(vector<eOprotID32_t> &id32vector, bool verify = false);

    bool numberofRegulars(uint16_t &numberofregulars);

    bool verifyRemoteValue(eOprotID32_t id32, void *value, uint16_t size, double timeout = 0.100, int retries = 10);

    bool getRemoteValue(eOprotID32_t id32, void *value, uint16_t &size, double timeout = 0.100, int retries = 10);


    // very important note: it works only if there is an handler for the id32 and it manages the unlock of the mutex
    bool setRemoteValueUntilVerified(eOprotID32_t id32, void *value, uint16_t size, int retries = 10, double waitbeforeverification = 0.001, double verificationtimeout = 0.050, int verificationretries = 2);


    /*!   @fn       isRunning(void);
     *    @brief    says if goToRun command has been sent to EMS
     *    @return   true if goToRun command has been sent to EMS else false
     */
    bool            isRunning(void);

    /*!   @fn       checkIsAlive(double curr_time);
     *    @brief    check if ems is ok by verifing time elapsed from last received packet. In case of error print yError.
     *    @return
     */
    void checkIsAlive(double curr_time);

    bool isEPmanaged(eOprot_endpoint_t ep);

    bool verifyBoard(yarp::os::Searchable &protconfig);
    bool verifyBoardPresence(yarp::os::Searchable &protconfig);
    bool verifyBoardTransceiver(yarp::os::Searchable &protconfig);   
    bool setTXrate(yarp::os::Searchable &protconfig);
    bool cleanBoardBehaviour(void);
    bool setRemoteBoardNumber(void);

    bool verifyEPprotocol(yarp::os::Searchable &protconfig, eOprot_endpoint_t ep);
    bool configureENDPOINT(yarp::os::Searchable &protconfig, eOprot_endpoint_t ep);
    bool verifyENTITYnumber(yarp::os::Searchable &protconfig, eOprot_endpoint_t ep, eOprotEntity_t en, int expectednumber = -1);


//    Semaphore* startNetworkQuerySession(eOprotID32_t id32, uint32_t signature); // to the associated board
//    bool waitForNetworkQueryReply(Semaphore* sem, double timeout);
//    bool stopNetworkQuerySession(Semaphore* sem);

    bool aNetQueryReplyHasArrived(eOprotID32_t id32, uint32_t signature);


    bool printRXstatistics(void);
    bool CANPrintHandler(eOmn_info_basic_t* infobasic);


    bool serviceVerifyActivate(eOmn_serv_category_t category, const eOmn_serv_parameter_t* param, double timeout = 0.500);

    bool serviceStart(eOmn_serv_category_t category, double timeout = 0.500);

    bool serviceStop(eOmn_serv_category_t category, double timeout = 0.500);


private:

    bool serviceCommand(eOmn_service_operation_t operation, eOmn_serv_category_t category, const eOmn_serv_parameter_t* param, double timeout);

    bool verbosewhenok;
    //uint64_t        RXpacket[maxRXpacketsize/8];    // buffer holding the received messages after EthReceiver::run() has read it from socket
    //uint16_t        RXpacketSize;
    //    ACE_UINT16      recv_size;                      // size of the received message
};


#endif

// eof




