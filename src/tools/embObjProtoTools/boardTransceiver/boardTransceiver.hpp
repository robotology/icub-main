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
#include "EOdeviceTransceiver.h"
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
#include <yarp/dev/DeviceDriver.h>

using namespace yarp::dev;

#define	RECV_BUFFER_SIZE        4000


// debug
void checkDataForDebug(uint8_t *data, uint16_t size);
void fromDouble(ACE_Time_Value &v, double x,int unit=1);

class BoardTransceiver : public yarp::os::RFModule
{
protected:
    eOprotBRD_t                 protboardnumber;
    EOdeviceTransceiver*        devtxrx;
    EOtransceiver*              transceiver;
    eOdevicetransceiver_cfg_t   devtxrxcfg;
    EOnvSet*                    nvset;
    uint32_t                    localipaddr;
    uint32_t                    remoteipaddr;
    uint16_t                    remoteipport;
    EOpacket                    *p_RxPkt;

    FEAT_ID                     _fId;
    ACE_SOCK_Dgram*             UDP_socket;
    ACE_INET_Addr               pc104Addr;

    eOmn_appl_status_t*         pApplStatus;
    EOnv*                       oneNV;

public:
    BoardTransceiver();
    ~BoardTransceiver();

    // yarp module methods
    bool createSocket(ACE_INET_Addr local_addr);
    bool configure(yarp::os::ResourceFinder &rf);
    bool updateModule();

    // Transceiver class
    bool init(yarp::os::Searchable &config, uint32_t localipaddr, uint32_t remoteipaddr, uint16_t ipport, uint16_t pktsize, FEAT_boardnumber_t board_n);


    // and Processes it
    virtual void onMsgReception(uint8_t *data, uint16_t size);

protected:

    /* Ask the transceiver to get the ropframe to be sent
     * This pointer will be modified by the getPack function to point to the TX buffer.
     * No need to allocate memory here */
    void getTransmit(uint8_t **data, uint16_t *size);


private:

    bool initProtocol(yarp::os::Searchable &config);

    void eoprot_override_mn(void);
    void eoprot_override_mc(void);
    void eoprot_override_as(void);
    void eoprot_override_sk(void);

    bool prepareTransceiverConfig(yarp::os::Searchable &config);


    const eOnvset_DEVcfg_t * getNVset_DEVcfg(yarp::os::Searchable &config);

public:

    eOprotBRD_t get_protBRDnumber(void);    // the number in range [0, max-1]
};

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



