/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author:  Valentina Gaggero
 * email:   valentina.gaggero@iit.it
 * website: www.robotcub.org
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

/* @file       main.cpp
    @brief
    @author     valentina.gaggero@iit.it
    @date       03/19/2013
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include <unistd.h>
#include <stdio.h>


// ACE stuff
#include <ace/ACE.h>
#include "ace/SOCK_Dgram.h"
#include "ace/Addr.h"


//embody stuff
#include "EoCommon.h"
#include "EoMotionControl.h"
#include "eODeb_eoProtoParser.h"
#include "eOtheEthLowLevelParser.h"
//pcap stuff
#include "pcap_wrapper_linux.h"




// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface
// --------------------------------------------------------------------------------------------------------------------
// empty-section







// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section





// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set()
// --------------------------------------------------------------------------------------------------------------------
static void my_cbk_onErrorSeqNum(eOethLowLevParser_packetInfo_t *pktInfo_ptr, uint32_t rec_seqNum, uint32_t expected_seqNum);
static void my_cbk_onNVfound(eOethLowLevParser_packetInfo_t *pktInfo_ptr, eODeb_eoProtoParser_ropAdditionalInfo_t *ropAddInfo_ptr);
static void my_cbk_onInvalidRopFrame(eOethLowLevParser_packetInfo_t *pktInfo_ptr);



const eODeb_eoProtoParser_cfg_t  deb_eoParserCfg =
{
	EO_INIT(.checks)
	{
		EO_INIT(.seqNum)
		{
			EO_INIT(.cbk_onErrSeqNum)           my_cbk_onErrorSeqNum,
		},

		EO_INIT(.nv)
		{
			EO_INIT(.NVs2searchArray)
			{
				EO_INIT(.head)
				{
					EO_INIT(.capacity)       eODeb_eoProtoParser_maxNV2find,
					EO_INIT(.itemsize)       sizeof(eODeb_eoProtoParser_nv_identify_t),
					EO_INIT(.size)           0,
				},
				EO_INIT(.data)
				{
					0,			//ep EB8 mc , nvid setpoint for joint 0
				}

			},
			EO_INIT(.cbk_onNVfound)            NULL
		},

		EO_INIT(.invalidRopFrame)
		{
			EO_INIT(.cbk)					   my_cbk_onInvalidRopFrame

		}
	}
};


const eODeb_eoProtoParser_cfg_t *deb_eoParserCfg_ptr = &deb_eoParserCfg;

// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions
// --------------------------------------------------------------------------------------------------------------------

static void my_cbk_onErrorSeqNum(eOethLowLevParser_packetInfo_t *pktInfo_ptr, uint32_t rec_seqNum, uint32_t expected_seqNum)
{

	printf("ERR in SEQNUM from 0x%x; rec=%d expected=%d\n", pktInfo_ptr->src_addr, rec_seqNum, expected_seqNum );
return;
}

static void my_cbk_onNVfound(eOethLowLevParser_packetInfo_t *pktInfo_ptr, eODeb_eoProtoParser_ropAdditionalInfo_t *ropAddInfo_ptr)
{

	printf("NV found!!: id=%x\n", ropAddInfo_ptr->desc.id32);
	return;
}

static void my_cbk_onInvalidRopFrame(eOethLowLevParser_packetInfo_t *pktInfo_ptr)
{
	printf("Invalid ropframe rec from 0x%x\n", pktInfo_ptr->src_addr);
	return;

}
static void my_cbk_onNVsetpointFound(eOethLowLevParser_packetInfo_t *pktInfo_ptr, eODeb_eoProtoParser_ropAdditionalInfo_t *ropAddInfo_ptr)
{
    eOmc_setpoint_t * setpoint_ptr = (eOmc_setpoint_t *)ropAddInfo_ptr->desc.data;
    uint8_t board = 0, j;
    float enc_factor, zero, enc_factor_6=182.044 , enc_factor_8=182.044, zero_6=180, zero_8=-180;

	//printf("NV found!!: ep=%x id=%x\n", ropAddInfo_ptr->desc.ep, ropAddInfo_ptr->desc.id);

    //add here code to get info of j and board
    float vel, pos;

    pos = (setpoint_ptr->to.position.value/enc_factor)-zero;
    vel = setpoint_ptr->to.position.withvelocity/fabs(enc_factor);

    printf("board %d  j %d   pos %f (%d)  vel %f (%d)  enc_factor=%f  zero=%f\n", board, j, pos, setpoint_ptr->to.position.value, vel, setpoint_ptr->to.position.withvelocity,enc_factor,zero);

    return;
}


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




