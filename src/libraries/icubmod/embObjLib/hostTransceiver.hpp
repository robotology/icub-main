/* Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author:  Alberto Cardellino, Marco Accame
 * email:   alberto.cardellino@iit.it, marco.accame@iit.it
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

//#include "FeatureInterface.h"

#include "EoCommon.h"
#include "EOhostTransceiver.h"
//#include "EOtransceiver.h"
#include "EOnvSet.h"
#include "EOnv.h"
//#include "EOpacket.h"
#include "EoProtocol.h"


#include <yarp/os/Semaphore.h>

#include <yarp/os/Searchable.h>


using namespace std;


// -- class HostTransceiver
// -- it contains methods for communication with the ETH boards.

namespace eth {

    class AbstractEthResource;

    class HostTransceiver
    {
    public:
         enum { maxSizeOfRXpacket = 1496 };
         enum { defMaxSizeOfROP = 256, defMaxSizeOfTXpacket = 768};

    public:

        HostTransceiver();
        ~HostTransceiver();


        bool init2(AbstractEthResource *owner, yarp::os::Searchable &cfgtotal, eOipv4addressing_t& localIPaddressing, eOipv4addr_t remoteIP, uint16_t rxpktsize = maxSizeOfRXpacket);

        AbstractEthResource * getOwner();

        bool isEPsupported(const eOprot_endpoint_t ep);
        bool isID32supported(const eOprotID32_t id32);

        // reads locally.
        bool read(const eOprotID32_t id32, void *data);

        // writes locally
        bool write(const eOprotID32_t id32, const void* data, bool forcewriteOfReadOnly);

        // adds a set<> ROP to the UDP packet
        bool addROPset(const eOprotID32_t id32, const void* data, const uint32_t signature = eo_rop_SIGNATUREdummy);

        // adds a ask<> ROP to the UDP packet
        bool addROPask(const eOprotID32_t id32, const uint32_t signature = eo_rop_SIGNATUREdummy);

        // called inside the thread ethReceiver (by a call to TheEthManager::Reception() which calls ... etc.) to process incoming UDP packet.
        // this function processes sig<> ROPs and say<> ROPs and:
        // 1. writes the received values into internal buffered memory,
        // 2. calls the relevant callback functions.
        bool parseUDP(const void *data, const uint16_t size);


        // returns the pointer of the udp packet formed inside the transceiver. if nullptr then no data to transmit.
        const void * getUDP(size_t &size, uint16_t &numofrops);


        bool getapplconfig(eOmn_appl_config_t &txcfg);

        AbstractEthResource * getResource();

        eOipv4addr_t getIPv4();

    private:

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
        uint16_t capacityofTXpacket;
        uint16_t maxSizeOfROP;
        eOnvset_BRDcfg_t nvsetbrdconfig;


    private:

        enum { maxNumberOfROPloadingAttempts = 5 };
        double delayAfterROPloadingFailure;


    private:

        AbstractEthResource *_owner;

        EOnv* getnvhandler(eOprotID32_t id32, EOnv* nv);

        eOprotBRD_t get_protBRDnumber(void);    // the number in range [0, max-1]

        bool lock_transceiver(bool on);
        yarp::os::Semaphore *htmtx;


        bool lock_nvs(bool on);
        yarp::os::Semaphore *nvmtx;


        bool addSetROP__(const eOprotID32_t id32, const void* data, const uint32_t signature, bool writelocalrxcache = false);
//        bool addGetROP__(eOprotID32_t id32, uint32_t signature);

        bool initProtocol();

        void eoprot_override_mn(void);
        void eoprot_override_mc(void);
        void eoprot_override_as(void);
        void eoprot_override_sk(void);

        bool prepareTransceiverConfig2(yarp::os::Searchable &cfgtotal);

        // set user data in the local memory, ready to be loaded by the load_occasional_rop method
    //    bool nvSetData(const EOnv *nv, const void *dat, eObool_t forceset, eOnvUpdate_t upd);
        bool getNVvalue(EOnv *nv, uint8_t* data, uint16_t* size);

        // the ip address
        eOipv4addr_t get_remoteIPaddress(void);
    };

} // namespace eth

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



