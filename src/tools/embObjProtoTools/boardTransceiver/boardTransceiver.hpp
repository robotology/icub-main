/* Copyright (C) 2014  iCub Facility, Istituto Italiano di Tecnologia
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


/** @file       BoardTransceiver.h
    @brief      This header file implements public interface to ...
    @author     marco.accame@iit.it
    @date       04/20/2011
 **/

/** @defgroup  Library BoardTransceiver
    It is an example of how the embOBJ can operate as host trasceiver.

    @{        
 **/


// - external dependencies --------------------------------------

#include "FeatureInterface.h"

#include "EoCommon.h"
#include "EOtheBOARDtransceiver.h"
#include "EOtransceiver.h"
#include "EOnvSet.h"
#include "EOnv.h"
#include "EOpacket.h"
#include "EoProtocol.h"

// Boards configurations

// ACE includes
#include <ace/ACE.h>
#include <ace/config.h>
#include <ace/Thread.h>
#include <ace/SOCK_Dgram_Bcast.h>

#include <yarp/os/RFModule.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/DeviceDriver.h>

using namespace yarp::dev;

//static yarp::os::Semaphore _all_transceivers_mutex = 1;
#define	RECV_BUFFER_SIZE        4000


// debug
void checkDataForDebug(uint8_t *data, uint16_t size);
void fromDouble(ACE_Time_Value &v, double x,int unit=1);

class BoardTransceiver : public yarp::os::RFModule
{
protected:
    eOprotBRD_t             protboardnumber;    // the number of board ranging from 0 upwards, in the format that the functions in EoProtocol.h expects. it is the same type as eOnvBRD_t 
    EOtheBOARDtransceiver   *hosttxrx;              /// CHECK??
    EOtransceiver           *pc104txrx;
    eOboardtransceiver_cfg_t hosttxrxcfg;           /// CHECK??
    EOnvSet                 *nvset;
    uint32_t                localipaddr;
    uint32_t                remoteipaddr;
    uint16_t                ipport;
    EOpacket                *p_RxPkt;

    FEAT_ID                  _fId;
    ACE_SOCK_Dgram          *UDP_socket;
    ACE_INET_Addr           pc104Addr;

public:
    BoardTransceiver();
    ~BoardTransceiver();

    yarp::os::Semaphore   transMutex;

    // yarp module methods
    bool createSocket(ACE_INET_Addr local_addr);
    bool configure(yarp::os::ResourceFinder &rf);
    bool updateModule();

    // Transceiver class
    bool init(yarp::os::Searchable &config, uint32_t localipaddr, uint32_t remoteipaddr, uint16_t ipport, uint16_t pktsize, FEAT_boardnumber_t board_n);


    /*! This method add a Set type rop in the next ropframe.
        Parameters are:
        protid: unique network variable identifier given the board
        data:  pointer to the data to be copied
        ----
        Note: size is calculated internally by getting Network Variable associated metadata
        */
    bool addSetMessage(eOprotID32_t protid, uint8_t* data);
    bool addSetMessageWithSignature(eOprotID32_t protid, uint8_t* data, uint32_t sig);   //as above, but with signature
    bool addSetMessageAndCacheLocally(eOprotID32_t protid, uint8_t* data);

    /*! This method add a Get type rop in the next ropframe. 
        Parameters are:
        protid: unique network variable identifier given the board
        */
    bool addGetMessage(eOprotID32_t protid);

    /*! Read data from the transceiver internal memory.
        Parameters are:
        protid: unique network variable identifier given the board
        data: pointer to 
     */
    bool readBufferedValue(eOprotID32_t protid,  uint8_t *data, uint16_t* size);

    /* ! This method echoes back a value that has just been sent from the BoardTransceiver to someone else, if called addSetMessageAndCacheLocally() */
    bool readSentValue(eOprotID32_t protid, uint8_t *data, uint16_t* size);

    // Set user data in the local memory, ready to be loaded by the load_occasional_rop method
    bool nvSetData(const EOnv *nv, const void *dat, eObool_t forceset, eOnvUpdate_t upd);

    // somebody passes the received packet
    void SetReceived(uint8_t *data, uint16_t size);
    // and Processes it
    virtual void onMsgReception(uint8_t *data, uint16_t size);

protected:
    /* Ask the transceiver to get the ropframe to be sent
     * This pointer will be modified by the getPack function to point to the TX buffer.
     * No need to allocate memory here */
    void getTransmit(uint8_t **data, uint16_t *size);
private:
    bool addSetMessage__(eOprotID32_t protid, uint8_t* data, uint32_t signature, bool writelocalrxcache = false);

    bool initProtocol(yarp::os::Searchable &config);

    void eoprot_override_mn(void);
    void eoprot_override_mc(void);
    void eoprot_override_as(void);
    void eoprot_override_sk(void); 

    bool prepareTransceiverConfig(yarp::os::Searchable &config);


    const eOnvset_DEVcfg_t * getNVset_DEVcfg(yarp::os::Searchable &config);  

public:
    bool getNVvalue(EOnv *nv, uint8_t* data, uint16_t* size);
    EOnv* getNVhandler(eOprotID32_t protid, EOnv* nv);

    // total number of variables inside the endpoint of the given board. on wrong params or is the ep is not present the function returns 0
    uint16_t getNVnumber(eOnvEP8_t ep);
    
    // progressive index on the endpoint of the variable described by protid. EOK_uint32dummy if the id does not exist on that board.
    uint32_t translate_NVid2index(eOprotID32_t protid);
    
    eOprotBRD_t get_protBRDnumber(void);    // the number in range [0, max-1]
};

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



