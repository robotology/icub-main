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


//typedef struct
//{
//    uint16_t    port;
//    int         ip1;
//    int         ip2;
//    int         ip3;
//    int         ip4;
//    char        string[64];
//} ethFeatIPaddress_t;


//typedef struct
//{
//    ethFeatIPaddress_t  pc104IPaddr;
//    ethFeatIPaddress_t  boardIPaddr;
//    iethresType_t       type;
//    IethResource*       interface;
//} ethFeature_t;



class can_string_eth;

namespace yarp{
    namespace dev{
        class EthResource;
        class TheEthManager;
    }
}

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev;
using namespace std;





// -- class InfoOfRecvPkts
// -- it is used to compute (and print) statistics about time or reception of UDP packets from a given IP address, and hence for a given EthResource.
// -- further comments (maybe to be revised and verified) are in the following.
// --
// -- this class contains all info about pkts received from a specific ems (its id is stored in "board" field)
// -- and let you calculate statistic about receive timing and check if one of following errors occur:
// -- 1) error in sequence number
// -- 2) between two consecutive pkts there are more the "timeout" sec (default is 0.01 sec and this is can modified by environment var ETHREC_STATISTICS_TIMEOUT_MSEC):
// --    this elapsed time is calculated in two way:
// --    1) using sys time: when arrived a pkt number n its received time(get by sys call) is compared with received time of pkt number n-1
// --    2) using ageOfFrame filed in packets: this time is insert by transmitter board. when arrive a pkt number n its is compared with ageOfFrme of pkt n-1.
// --       if there are more then timeout sec it mean that board did not transmit for this period.

class InfoOfRecvPkts
{

private:
    enum { DEFAULT_MAX_COUNT_STAT = 60000 }; // 1 minute expressed in ms
    const double DEFAULT_TIMEOUT_STAT;

private:
    eOipv4addr_t    ipv4;
    char            ipv4string[20];
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

    InfoOfRecvPkts();
    ~InfoOfRecvPkts();
    void printStatistics(void);
    void clearStatistics(void);
    uint64_t getSeqNum(uint64_t *packet, uint16_t size);
    uint64_t getAgeOfFrame(uint64_t *packet, uint16_t size);


    void updateAndCheck(uint64_t *packet, uint16_t size, double reckPktTime, double processPktTime, bool evalreport = false);

    void evalReport(void);

    void forceReport(void);

    void setBoardIP(eOipv4addr_t ip);
};


// -- class EthNetworkQuery
// -- it is used to wait for a reply from a board.

class EthNetworkQuery
{

public:

    EthNetworkQuery();
    ~EthNetworkQuery();

    Semaphore* start(eOprotID32_t id32, uint32_t signature);    // associates a semaphore to a (id2wait, signature) pair
    bool wait(Semaphore* sem, double timeout);                  // waits the semaphore until either a reply arrives or timeout expires (returns false)
    bool arrived(eOprotID32_t id32, uint32_t signature);        // a reply has arrived. the rx handler must call it. true if it releases, false if not
    bool stop(Semaphore* sem);                                  // we deassociate the semaphore to the waiting pair

private:

    eOprotID32_t id2wait;           // the id32 which we wait
    yarp::os::Semaphore* netwait;   // the semaphore used by the class to wait for a reply from network. it must have exclusive access
    yarp::os::Semaphore* isbusy;    // the semaphore used to guarantee exclusive access of ::netwait to calling threads
    yarp::os::Semaphore* iswaiting; // the semaphore used to guarantee that the wait of the class is unblocked only once and when it is required

};


// -- class EthResource
// -- it is used to manage udp communication towards a given board ... there is no need to derive it from DeviceDriver, is there?

class yarp::dev::EthResource:  public DeviceDriver,
                               public HostTransceiver
//class yarp::dev::EthResource:    public HostTransceiver
{
public:

    // the size of the boardname must belong to EthResources because it is this class which extracts it from the xml file.
    // all other places where this name is stored have copied it from here using EthResource::getName(void).
    // an example is TheEthManager::ethBoards which makes it available with TheEthManager::getName(eOipv4addr_t ipv4)

    enum { boardNameSize = 32 };

    // this is the maximum size of rx and tx packets managed by the ethresource. however, the HostTranceveiver can reduce these values.
    enum { maxRXpacketsize = 1496, maxTXpacketsize = 1496 };

public:

    EthResource();
    ~EthResource();


    bool            open2(eOipv4addr_t remIP, yarp::os::Searchable &cfgtotal);
    bool            close();
    bool            isEPsupported(eOprot_endpoint_t ep);

    ACE_INET_Addr   getRemoteAddress(void);

    eOipv4addr_t    getIPv4remoteAddress(void);

    const char *    getName(void);
    const char *    getIPv4string(void);

    // the function returns true if the packet can be transmitted. 
    // it returns false if it cannot be transmitted: either it is with no rops inside in mode donttrxemptypackets, or there is an error somewhere
    bool            getTXpacket(uint8_t **packet, uint16_t *size, uint16_t *numofrops);

    bool            canProcessRXpacket(uint64_t *data, uint16_t size);

    void            processRXpacket(uint64_t *data, uint16_t size, bool collectStatistics = true);


    bool verifyRemoteValue(eOprotID32_t id32, void *value, uint16_t size, double timeout = 0.100, int retries = 10);

    bool getRemoteValue(eOprotID32_t id32, void *value, uint16_t &size, double timeout = 0.100, int retries = 10);


    // very important note: it works only if there is an handler for the id32 and it manages the unlock of the mutex
    bool setRemoteValueUntilVerified(eOprotID32_t id32, void *value, uint16_t size, int retries = 10, double waitbeforeverification = 0.001, double verificationtimeout = 0.050, int verificationretries = 2);


    bool verifyEPprotocol(eOprot_endpoint_t ep);

    bool aNetQueryReplyHasArrived(eOprotID32_t id32, uint32_t signature);


    bool printRXstatistics(void);
    bool CANPrintHandler(eOmn_info_basic_t* infobasic);


    bool serviceVerifyActivate(eOmn_serv_category_t category, const eOmn_serv_parameter_t* param, double timeout = 0.500);

    bool serviceSetRegulars(eOmn_serv_category_t category, vector<eOprotID32_t> &id32vector, double timeout = 0.500);

    bool serviceStart(eOmn_serv_category_t category, double timeout = 0.500);

    bool serviceStop(eOmn_serv_category_t category, double timeout = 0.500);


    // -- not used at the moment
    void checkIsAlive(double curr_time);
    // double          getLastRecvMsgTimestamp(void);
    // int             getNumberOfAttachedInterfaces(void);
    // returns the capacity of the receiving buffer.
    // int getRXpacketCapacity();

private:

    eOipv4addr_t      ipv4addr;
    char              ipv4addrstring[20];
    char              boardName[32];
    ACE_INET_Addr     remote_dev;             //!< IP address of the EMS this class is talking to.
    double            lastRecvMsgTimestamp;   //! stores the system time of the last received message, gettable with getLastRecvMsgTimestamp()
    bool			  isInRunningMode;        //!< say if goToRun cmd has been sent to EMS
    InfoOfRecvPkts    *infoPkts;

    yarp::os::Semaphore*  objLock;

    EthNetworkQuery*    ethQuery;

    EthNetworkQuery*    ethQueryServices;

    bool                verifiedEPprotocol[eoprot_endpoints_numberof];
    bool                verifiedBoardPresence;

    bool                verifiedBoardTransceiver; // transceiver capabilities (size of rop, ropframe, etc.) + MN protocol version
    bool                txrateISset;
    bool                cleanedBoardBehaviour;    // the board is in config mode and does not have any regulars
    eOmn_comm_status_t  boardCommStatus;
    uint16_t            usedNumberOfRegularROPs;

    can_string_eth*     c_string_handler[16];

    TheEthManager       *ethManager;

private:


    bool verifyBoard();
    bool setTXrate();
    bool verifyBoardPresence();
    bool verifyBoardTransceiver();
    bool cleanBoardBehaviour(void);
    // we keep isRunning() and we add a field in the reply of serviceStart()/Stop() which tells if the board is in run mode or not.
    bool isRunning(void);

    // lock of the object: on / off
    bool lock(bool on);

    bool serviceCommand(eOmn_service_operation_t operation, eOmn_serv_category_t category, const eOmn_serv_parameter_t* param, double timeout, int times);

    bool verbosewhenok;
};


#endif

// eof




