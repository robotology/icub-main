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
    @date       07/21/2014
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include <unistd.h>
#include <stdio.h>
#include <ctime>
#include <time.h>
#include <string>

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



const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%d-%m-%Y %X", &tstruct);

    return buf;
}

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set()
// --------------------------------------------------------------------------------------------------------------------
static void my_cbk_onErrorSeqNum(eOethLowLevParser_packetInfo_t *pktInfo_ptr, uint32_t rec_seqNum, uint32_t expected_seqNum);
static void my_cbk_fullScaleFound(eOethLowLevParser_packetInfo_t *pktInfo_ptr, eODeb_eoProtoParser_ropAdditionalInfo_t *ropAddInfo_ptr);
static void my_cbk_onInvalidRopFrame(eOethLowLevParser_packetInfo_t *pktInfo_ptr);



const eODeb_eoProtoParser_cfg_t  deb_eoParserCfg =
{
	EO_INIT(.checks)
	{
		EO_INIT(.seqNum)
		{
			EO_INIT(.cbk_onErrSeqNum)           NULL, //my_cbk_onErrorSeqNum,
		},

		EO_INIT(.nv)
		{
			EO_INIT(.NVs2searchArray)
			{
				EO_INIT(.head)
				{
					EO_INIT(.capacity)       eODeb_eoProtoParser_maxNV2find,
					EO_INIT(.itemsize)       sizeof(eODeb_eoProtoParser_nv_identify_t),
                    EO_INIT(.size)           12,
				},
				EO_INIT(.data)
				{
                    0x100000F,           // calibration message
                    0x100010F,
                    0x100020F,
                    0x100030F,
                    0x100040F,
                    0x100050F,
                    0x100060F,
                    0x100070F,
                    0x100080F,
                    0x100090F,
                    0x1000A0F,
                    0x1000B0F
                }

			},
			EO_INIT(.cbk_onNVfound)            my_cbk_fullScaleFound
		},

		EO_INIT(.invalidRopFrame)
		{
			EO_INIT(.cbk)			my_cbk_onInvalidRopFrame

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


static void my_cbk_fullScaleFound(eOethLowLevParser_packetInfo_t *pktInfo_ptr, eODeb_eoProtoParser_ropAdditionalInfo_t *ropAddInfo_ptr)
{
        eOas_strain_config_t *straincfg_ptr = (eOas_strain_config_t *)ropAddInfo_ptr->desc.data;

//    time_t t = time(0);   // get time now
//    struct tm * now = localtime( & t );
//    printf(" %d - %d - %d - ", (now->tm_year + 1900), (now->tm_mon + 1), now->tm_mday);
    printf("date %s \n\t", currentDateTime().c_str());

    printf("Calibration MSG found: from 0x%x to 0x%x protoId=0x%x (%d) ropcode=0x%x plussig=%d sig=%d seqnum=%ld\n",
        pktInfo_ptr->src_addr,
        pktInfo_ptr->dst_addr,
		ropAddInfo_ptr->desc.id32,
		ropAddInfo_ptr->desc.id32,
		ropAddInfo_ptr->desc.ropcode,
		ropAddInfo_ptr->desc.control.plussign,
		ropAddInfo_ptr->desc.signature,
        ropAddInfo_ptr->seqnum);

	return;
}


static void my_cbk_onInvalidRopFrame(eOethLowLevParser_packetInfo_t *pktInfo_ptr)
{
	printf("Invalid ropframe rec from 0x%x\n", pktInfo_ptr->src_addr);
	return;
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




