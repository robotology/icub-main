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

#include "EoCommon.h"
#include "EOhostTransceiver.h"
//#include "transceiverInterface.h"

	// Boards configurations

//#include "eOcfg_nvsEP_mn.h"
//#include "eOcfg_nvsEP_mc.h"
//#include "eOcfg_nvsEP_sk.h"
#include "EOnvSet.h"

//#include "eOcfg_EPs_board.h"
#include "eOprot_b01.h"
#include "eOprot_b02.h"
#include "eOprot_b03.h"
#include "eOprot_b04.h"
#include "eOprot_b05.h"
#include "eOprot_b06.h"
#include "eOprot_b07.h"
#include "eOprot_b08.h"
#include "eOprot_b09.h"
#include "eOprot_b10.h"
#include "eOprot_b11.h"

#include <yarp/os/Semaphore.h>
#include <yarp/dev/DeviceDriver.h>

using namespace yarp::dev;

//static yarp::os::Semaphore _all_transceivers_mutex = 1;

// debug
void checkDataForDebug(uint8_t *data, uint16_t size);

class hostTransceiver
{
protected:
    EOhostTransceiver       *hosttxrx;
    EOtransceiver           *pc104txrx;
    eOhosttransceiver_cfg_t hosttxrxcfg;
    EOnvSet                 *nvset;
    uint32_t                localipaddr;
    uint32_t                remoteipaddr;
    uint16_t                ipport;
    EOpacket                *p_RxPkt;

public:
    hostTransceiver();
    ~hostTransceiver();

//    const EOconstvector*  EPvector;
//    eOuint16_fp_uint16_t  EPhash_function_ep2index;
    yarp::os::Semaphore   transMutex;


    bool init( uint32_t localipaddr, uint32_t remoteipaddr, uint16_t ipport, uint16_t pktsize, uint8_t board_n);


    /*! This method add a Set type rop in the next ropframe.
        Parameters are:
        nvid: unique network variable identifier got with the appropriate eo_cfg_nvsEP_XXX_NVID_Get
        endpoint: enum of the enpoint it has to operate into
        data:  pointer to the data to be copied
        ----
        Note: size is calculated internally by getting Network Variable associated metadata
        */
    bool addSetMessage( eOprotID32_t protid, uint8_t* data);
    bool addSetMessageWithSignature(eOprotID32_t protid, uint8_t* data, uint32_t sig);   //as above, but with signature

    /*! This method add a Get type rop in the next ropframe.
        Parameters are:
        nvid: unique network variable identifier got with the appropriate eo_cfg_nvsEP_XXX_NVID_Get
        endpoint: enum of the enpoint it has to operate into
        */
    bool addGetMessage( eOprotID32_t protid);

    /*! Read data from the transceiver internal memory.
        Parameters are:
        nvid: unique network variable identifier got with the appropriate eo_cfg_nvsEP_XXX_NVID_Get
        endpoint: enum of the enpoint it has to operate into
        data: pointer to 
     */
    bool readBufferedValue(eOprotID32_t protid,  uint8_t *data, uint16_t* size);

    /* ! This method echoes back a value that has just been sent from the hostTransceiver to someone else */
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
    bool addSetMessage__( eOprotID32_t protid, uint8_t* data, uint32_t signature);

public:
    bool getNVvalue(EOnv *nv, uint8_t* data, uint16_t* size);
    EOnv* getNVhandler(eOprotID32_t protid, EOnv* nv);


    //void getHostData(const EOconstvector **pEPvector, eOuint16_fp_uint16_t *pEPhash_function);

    uint16_t getNVnumber(int boardNum, eOnvEP8_t ep);
    uint32_t translate_NVid2index(int boardNum, eOprotID32_t protoid);
};

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

