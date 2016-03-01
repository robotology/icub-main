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



#undef _ENABLE_TRASMISSION_OF_EMPTY_ROPFRAME_ // if this macro is defined then ethManager sends pkts to ems even if they are empty

#undef  TEST_TX_HOSTTRANSCEIVER_OPTIMISATION

// - external dependencies --------------------------------------------------------------------------------------------

#include "FeatureInterface.h"

#include "EoCommon.h"
#include "EOhostTransceiver.h"
#include "EOtransceiver.h"
#include "EOnvSet.h"
#include "EOnv.h"
#include "EOpacket.h"
#include "EoProtocol.h"
#include "EOprotocolConfigurator.h"


#include <yarp/os/Semaphore.h>
#include <yarp/dev/DeviceDriver.h>

using namespace yarp::dev;


// debug
//void checkDataForDebug(uint8_t *data, uint16_t size);

class hostTransceiver
{
private:
    enum { maxNumberOfROPloadingAttempts = 5 };
    double delayAfterROPloadingFailure;

protected:

    eOprotBRD_t             protboardnumber;    // the number of board ranging from 0 upwards, in the format that the functions in EoProtocol.h expects.
    EOhostTransceiver       *hosttxrx;
    EOtransceiver           *pc104txrx;
    eOhosttransceiver_cfg_t hosttxrxcfg;
    EOnvSet                 *nvset;
    eOipv4addr_t            localipaddr;
    eOipv4addr_t            remoteipaddr;
    eOipv4port_t            ipport;
    EOpacket                *p_RxPkt;
    uint16_t                pktsizerx;
    EOprotocolConfigurator* protconfigurator;

    eOmn_transceiver_properties_t remoteTransceiverProperties;  // contains properties of the transceiver of the remote board as read from xml file
    eOmn_transceiver_properties_t localTransceiverProperties;   // contains properties of the transceiver here instantiated.
                                                                // properties derive from remoteTransceiverProperties, from hosttxrxcfg and from elsewhere
    uint8_t TXrate;
public:

    hostTransceiver();
    ~hostTransceiver();



    bool init(yarp::os::Searchable &cfgtransceiver, yarp::os::Searchable &cfgprotocol, eOipv4addr_t localipaddr, eOipv4addr_t remoteipaddr, eOipv4port_t ipport, uint16_t pktsize, FEAT_boardnumber_t board_n);

    bool addSetMessage(eOprotID32_t protid, uint8_t* data);
    bool addSetMessageWithSignature(eOprotID32_t protid, uint8_t* data, uint32_t sig);   //as above, but with signature
    bool addSetMessageAndCacheLocally(eOprotID32_t protid, uint8_t* data);

    /*! This method add a Get type rop in the next ropframe. 
        Parameters are:
        protid: unique network variable identifier given the board
        */
    bool addGetMessage(eOprotID32_t protid);

    bool addGetMessageWithSignature(eOprotID32_t protid, uint32_t signature);


    /*! Read data from the transceiver internal memory.
        Parameters are:
        protid: unique network variable identifier given the board
        data: pointer to 
     */
    bool readBufferedValue(eOprotID32_t protid,  uint8_t *data, uint16_t* size);

    /* ! This method echoes back a value that has just been sent from the hostTransceiver to someone else, if called addSetMessageAndCacheLocally() */
    bool readSentValue(eOprotID32_t protid, uint8_t *data, uint16_t* size);

    // Set user data in the local memory, ready to be loaded by the load_occasional_rop method
    bool nvSetData(const EOnv *nv, const void *dat, eObool_t forceset, eOnvUpdate_t upd);

    int getCapacityOfRXpacket(void);

    // pass and process the received packet
    void onMsgReception(uint64_t *data, uint16_t size);

protected:

    /* Ask the transceiver to get the ropframe to be sent
     * This pointer will be modified by the getPack function to point to the TX buffer.
     * No need to allocate memory here */
    // the function returns true if the packet can be transmitted, false if not.
    // if _ENABLE_TRASMISSION_OF_EMPTY_ROPFRAME_ is undefined, the function returns false also if there are no rops
    // inside the ropframe. in such a way the macro _ENABLE_TRASMISSION_OF_EMPTY_ROPFRAME_ can have
    // a scope that is local only to hostTransceiver.cpp
    bool getTransmit(uint8_t **data, uint16_t *size, uint16_t* numofrops);

private:


    bool lock_transceiver();
    bool unlock_transceiver();
    yarp::os::Semaphore *htmtx;


    bool lock_nvs();
    bool unlock_nvs();
    yarp::os::Semaphore *nvmtx;


    bool addSetMessage__(eOprotID32_t protid, uint8_t* data, uint32_t signature, bool writelocalrxcache = false);
    bool addGetMessage__(eOprotID32_t protid, uint32_t signature);

    bool initProtocol(yarp::os::Searchable &cfgprotocol);

    void eoprot_override_mn(void);
    void eoprot_override_mc(void);
    void eoprot_override_as(void);
    void eoprot_override_sk(void); 

    bool prepareTransceiverConfig(yarp::os::Searchable &cfgtransceiver, yarp::os::Searchable &cfgprotocol);

    bool fillRemoteProperties(yarp::os::Searchable &cfgprotocol);

    const eOnvset_BRDcfg_t * getNVset_BRDcfg(yarp::os::Searchable &cfgprotocol);

public:

    bool getNVvalue(EOnv *nv, uint8_t* data, uint16_t* size);
    EOnv* getNVhandler(eOprotID32_t protid, EOnv* nv);

    // total number of variables inside the endpoint of the given board. on wrong params or is the ep is not present the function returns 0
    uint16_t getNVnumber(eOnvEP8_t ep);
    
    // progressive index on the endpoint of the variable described by protid. EOK_uint32dummy if the id does not exist on that board.
    uint32_t translate_NVid2index(eOprotID32_t protid);
    
    eOprotBRD_t get_protBRDnumber(void);    // the number in range [0, max-1]

    eOipv4addr_t get_remoteIPaddress(void);
};

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



