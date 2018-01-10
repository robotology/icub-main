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

// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _HOSTTRANSCEIVER_H_
#define _HOSTTRANSCEIVER_H_


/** @file       hostTransceiver.h
    @brief      This header file implements public interface to ...
    @author     marco.accame@iit.it
    @date       04/20/2011
 **/

/** @defgroup  Library hostTransceiver
    It is an example of how the embOBJ can operate as host trasceiver.

    @{        
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "FeatureInterface.h"

#include "EoCommon.h"
#include "EOhostTransceiver.h"
#include "EOtransceiver.h"
#include "EOnvSet.h"
#include "EOnv.h"
#include "EOpacket.h"
#include "EoProtocol.h"


#include <yarp/os/Semaphore.h>
#include <yarp/dev/DeviceDriver.h>

using namespace yarp::dev;



// -- class HostTransceiver
// -- it contains methods for communication with the ETH boards.

class HostTransceiver
{
public:
     enum { maxSizeOfRXpacket = 1496 };
     enum { defTXrateOfRegularROPs = 3, defMaxSizeOfROP = 256, defMaxSizeOfTXpacket = 768, defcycletime = 1000, defmaxtimeRX = 400, defmaxtimeDO = 300, defmaxtimeTX = 300};

public:

    HostTransceiver();
    ~HostTransceiver();


    bool init2(yarp::os::Searchable &cfgEthBoard, eOipv4addressing_t& localIPaddressing, eOipv4addr_t remoteIP, uint16_t rxpktsize = maxSizeOfRXpacket);

    // methods which put a set<> ROP inside the UDP packet which will be transmitted by the EthSender
    bool addSetROP(eOprotID32_t id32, uint8_t* data);
    bool addSetROPwithSignature(eOprotID32_t id32, uint8_t* data, uint32_t sig);
    bool addSetROPandCacheLocally(eOprotID32_t id32, uint8_t* data);

    bool isID32supported(eOprotID32_t id32);

    uint16_t getMaxSizeofROP();

    // methods which put a ask<> ROP inside the UDP packet which will be transmitted by the EthSender
    bool addGetROP(eOprotID32_t id32);
    bool addGetROPwithSignature(eOprotID32_t id32, uint32_t signature);

    // called inside the thread ethReceiver (by a call to TheEthManager::Reception() which calls ... etc.) to process incoming UDP packet.
    // this function processes sig<> ROPs and say<> ROPs and it: 1. writes the received values into internal buffered memory, and
    // 2. calls the relevant callback functions.
    void onMsgReception(uint64_t *data, uint16_t size);

    // reads the value of a network variable buffered by the object at reception of a UDP packet by method onMsgReception().
    bool readBufferedValue(eOprotID32_t id32,  uint8_t *data, uint16_t* size);

    // This method echoes back a value that has just been sent from the HostTransceiver to someone else, if called addSetROPAndCacheLocally()
    bool readSentValue(eOprotID32_t id32, uint8_t *data, uint16_t* size);


    // gives maximum size of UDP packet which can be managed in reception
    int getCapacityOfRXpacket(void);


    // much better to remove this function from here, because direct access to EOnv data is not recommended.
    EOnv* getNVhandler(eOprotID32_t id32, EOnv* nv);

    // total number of variables inside the endpoint of the given board. on wrong params or is the ep is not present the function returns 0
    uint16_t getNVnumber(eOnvEP8_t ep);

    // progressive index on the endpoint of the variable described by protid. EOK_uint32dummy if the id does not exist on that board.
    uint32_t translate_NVid2index(eOprotID32_t id32);

protected:

    eOprotBRD_t             protboardnumber;    // the number of board ranging from 0 upwards, in the format that the functions in EoProtocol.h expects.
    EOhostTransceiver       *hosttxrx;
    EOtransceiver           *pc104txrx;
    eOhosttransceiver_cfg_t hosttxrxcfg;
    EOnvSet                 *nvset;
    eOipv4addr_t            localipaddr;
    eOipv4addr_t            remoteipaddr;
    char                    remoteipstring[20];
    eOipv4port_t            ipport;
    EOpacket                *p_RxPkt;
    uint16_t                pktsizerx;

    uint8_t TXrateOfRegularROPs;
    eOreltime_t cycletime;
    uint16_t    maxtimeRX;
    uint16_t    maxtimeDO;
    uint16_t    maxtimeTX;
    uint16_t capacityofTXpacket;
    uint16_t maxSizeOfROP;
    eOnvset_BRDcfg_t nvsetbrdconfig;


protected:

    /* Ask the transceiver to get the ropframe to be sent
     * This pointer will be modified by the getPack function to point to the TX buffer.
     * No need to allocate memory here */
    // the function returns true if the packet can be transmitted, false if not.
    // if HOSTTRANSCEIVER_EmptyROPframesAreTransmitted is defined, the function returns true also if there are no rops
    // inside the ropframe.
    bool getTransmit(uint8_t **data, uint16_t *size, uint16_t* numofrops);

    bool isSupported(eOprot_endpoint_t ep);


private:

    enum { maxNumberOfROPloadingAttempts = 5 };
    double delayAfterROPloadingFailure;


private:

    eOprotBRD_t get_protBRDnumber(void);    // the number in range [0, max-1]

    bool lock_transceiver(bool on);
    yarp::os::Semaphore *htmtx;


    bool lock_nvs(bool on);
    yarp::os::Semaphore *nvmtx;


    bool addSetROP__(eOprotID32_t id32, uint8_t* data, uint32_t signature, bool writelocalrxcache = false);
    bool addGetROP__(eOprotID32_t id32, uint32_t signature);

    bool initProtocol();

    void eoprot_override_mn(void);
    void eoprot_override_mc(void);
    void eoprot_override_as(void);
    void eoprot_override_sk(void); 

    bool prepareTransceiverConfig2(yarp::os::Searchable &cfgEthBoard);

    // set user data in the local memory, ready to be loaded by the load_occasional_rop method
//    bool nvSetData(const EOnv *nv, const void *dat, eObool_t forceset, eOnvUpdate_t upd);
    bool getNVvalue(EOnv *nv, uint8_t* data, uint16_t* size);

    // the ip address
    eOipv4addr_t get_remoteIPaddress(void);


};

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



