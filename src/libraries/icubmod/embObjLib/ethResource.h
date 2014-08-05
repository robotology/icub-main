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
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include "statExt.h"

// embObj includes

#include "hostTransceiver.hpp"
#include "debugFunctions.h"
#include "FeatureInterface.h"


// ACE stuff
#include <ace/ACE.h>
#include <ace/config.h>
#include <ace/SOCK_Dgram_Bcast.h>


// we see EOK_HOSTTRANSCEIVER_capacityofrxpacket by including FeatureInterface.hpp which includes EOhostTransceiver.h
#define	RECV_BUFFER_SIZE        EOK_HOSTTRANSCEIVER_capacityofrxpacket
#define	ETHRES_SIZE_INFO        128
#define MAX_ICUB_EP             32

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
    int board; //num of ems board
public:
    bool          initted;          //if true is pc104 has already received a pkt from the ems when it is in running mode
    bool          isInError;        //is true if an error occur. it is used to avoid to print error every cicle
    int           count;
    int           max_count;        //every max_count received pkts statistics are printed. this number can be modified by environment variable ETHREC_STATISTICS_COUNT_RECPKT_MAX
    double        timeout;          //is expressed in sec. its default val is 0.01 sec, but is it can modified by environment var ETHREC_STATISTICS_TIMEOUT_MSEC

    uint64_t      last_seqNum;      //last seqNum rec
    uint64_t      last_ageOfFrame;  //value of ageOfFrame filed of last rec pkt
    double        last_recvPktTime; //time of last recv pkt

    int           totPktLost;       //total pkt lost from start up
    int           currPeriodPktLost; // total pkt lost during thsi period
    StatExt       *stat_ageOfFrame; //period between two packets calculating using ageOfFrame filed of ropframe. values are stored in millisec
    StatExt       *stat_periodPkt;  //delta time between two consegutivs pkt; time is calculating using pc104 systime. Values are stored in sec
    StatExt       *stat_precessPktTime; //values are stored in sec
    bool          _verbose;

    infoOfRecvPkts();
    void printStatistics(void);
    void clearStatistics(void);
    uint64_t getSeqNum(uint8_t *packet);
    uint64_t getAgeOfFrame(uint8_t *packet);

    /*!   @fn       void updateAndCheck(uint8_t *packet, double reckPktTime, double processPktTime);
     *    @brief    updates communication info statistics and checks if there is a transmission error with this ems board.In case of error print it.
     *    @param    packet           pointer to received packet
     *    @param    reckPktTime      sys time on rec pkt (expressed in seconds)
     *    @param    processPktTime   time needed to process this packet (expressed in seconds)
     */
    void updateAndCheck(uint8_t *packet, double reckPktTime, double processPktTime);

    void setBoardNum(int board);
};


class yarp::dev::ethResources:  public DeviceDriver,
                                public hostTransceiver
{
private:
//     static yarp::os::Semaphore  ethResMutex;

    char              info[ETHRES_SIZE_INFO];
    int               how_many_features;      //!< Keep track of how many high level class registered. onto this EMS
    ACE_INET_Addr     remote_dev;             //!< IP address of the EMS this class is talking to.
    double            lastRecvMsgTimestamp;   //! stores the system time of the last received message, gettable with getLastRecvMsgTimestamp()
    bool			  isInRunningMode;        //!< say if goToRun cmd has been sent to EMS
    infoOfRecvPkts    *infoPkts;



public:
//     hostTransceiver   *transceiver;         //!< Pointer to the protocol handler   non più usato. ora c'è derivazione diretta.
    yarp::dev::TheEthManager     *ethManager;          //!< Pointer to the Singleton handling the UDP socket
    int               boardNum;

    ethResources();
    ~ethResources();

    uint8_t         recv_msg[RECV_BUFFER_SIZE];   // buffer for the reveived messages
    ACE_UINT16      recv_size;                    // size of the incoming message

    ethResources *  already_exists(yarp::os::Searchable &config);
    bool            open(yarp::os::Searchable &config, FEAT_ID request);
    bool            close();

    /*!   @fn       registerFeature(void);
     *    @grief    tells the EMS a new device requests its services.
     *    @return   true.
     */
    bool            registerFeature(FEAT_ID *request);

    /*!   @fn       unregisterFeature();
     *    @brief    tell the EMS a user has been closed. If was the last one, close the EMS
     */
    int             deregisterFeature(FEAT_ID request);

    ACE_INET_Addr   getRemoteAddress(void);

    /*!   @fn       void getPointer2txPack(uint8_t **pack, uint16_t *size);
     *    @brief    Create the new packet to be sent with all requests collected so far, and return the pointer to
     *              this fresh new packet.
     *    @param    pack  this pointer will be modified to make it point to the new packet. No memory needs to be associated
     *                    with it, just a pointer.
     *    @param    size  retutn the dimension of the packet.
     */
    void            getPointer2TxPack(uint8_t **pack, uint16_t *size);

    /*!   @fn       void getPack(uint8_t *pack, uint16_t *size);
     *    @brief    Create the new packet to be sent with all requests collected so far, and copy it in data. External memory
     *              needs to be provided.
     *    @param    size  retutn the dimension of the packet.
     */
    void            getTxPack(uint8_t *pack, uint16_t *size);

    //! Return the dimension of the receiving buffer.
    int             getBufferSize();

    void            onMsgReception(uint8_t *data, uint16_t size);


    /*!   @fn       goToRun(void);
     *    @brief    Tells the EMS to start the 1ms loop.
     */
    bool            goToRun(void);


    /*!   @fn       goToConfig(void);
     *    @brief    Tells the EMS to stop the 1ms loop and go into configuration mode.
     */
    bool            goToConfig(void);

    /*!   @fn       getLastRecvMsgTimestamp(void);
     *    @brief    return the system time of the last received message from the corresponding EMS board.
     *    @return   seconds passed from the very first message received.
     */
    double          getLastRecvMsgTimestamp(void);

    /*!   @fn       clearPerSigMsg(void);
     *    @brief    clears periodic signal message of EMS
     *    @return   true on success else false
     */
    bool            clearPerSigMsg(void);


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





//     //! Send a sporadic message. Do not use now.
//     int             send(void *data, size_t len);
};


#endif

// eof




