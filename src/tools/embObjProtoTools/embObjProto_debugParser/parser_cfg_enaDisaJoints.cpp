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
//#include <ace/ACE.h>
//#include "ace/SOCK_Dgram.h"
//#include "ace/Addr.h"


//embody stuff
#include "EoCommon.h"
#include "EoMotionControl.h"
#include "eODeb_eoProtoParser.h"
//#include "eOtheEthLowLevelParser.h"
//pcap stuff
//#include "pcap_wrapper_linux.h"




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
static void my_cbk_onControlModeFound(eOethLowLevParser_packetInfo_t *pktInfo_ptr, eODeb_eoProtoParser_ropAdditionalInfo_t *ropAddInfo_ptr);
static void my_cbk_onInvalidRopFrame(eOethLowLevParser_packetInfo_t *pktInfo_ptr);



const eODeb_eoProtoParser_cfg_t  deb_eoParserCfg =
{
	EO_INIT(.checks)
	{
		EO_INIT(.seqNum)
		{
			EO_INIT(.cbk_onErrSeqNum)           NULL,
		},

		EO_INIT(.nv)
		{
			EO_INIT(.NVs2searchArray)
			{
				EO_INIT(.head)
				{
					EO_INIT(.capacity)       eODeb_eoProtoParser_maxNV2find,
					EO_INIT(.itemsize)       sizeof(eODeb_eoProtoParser_nvidEp_couple_t),
					EO_INIT(.size)           12,
				},

				EO_INIT(.data)
				{

					{0xffff, 0xa01c}, //set control mode
					{0xffff, 0xa03c}, //set control mode
					{0xffff, 0xa05c}, //set control mode
					{0xffff, 0xa07c}, //set control mode
					{0xffff, 0xa09c}, //set control mode
 					{0xffff, 0xa0bc}, //set control mode
					{0xffff, 0xa0dc}, //set control mode
					{0xffff, 0xa0fc}, //set control mode
					{0xffff, 0xa11c}, //set control mode
					{0xffff, 0xa13c}, //set control mode
					{0xffff, 0xa15c}, //set control mode
					{0xffff, 0xa17c}, //set control mode
				}

			},
			EO_INIT(.cbk_onNVfound)            my_cbk_onControlModeFound
		},

		EO_INIT(.invalidRopFrame)
		{
			EO_INIT(.cbk)					   NULL //my_cbk_onInvalidRopFrame

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
// empty-section



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

	printf("ERR in SEQNUM; rec=%d expected=%d\n",rec_seqNum, expected_seqNum );
return;
}



static void my_cbk_onInvalidRopFrame(eOethLowLevParser_packetInfo_t *pktInfo_ptr)
{
	printf("Invalid ropframe\n");
	return;

}


static void my_cbk_onControlModeFound(eOethLowLevParser_packetInfo_t *pktInfo_ptr, eODeb_eoProtoParser_ropAdditionalInfo_t *ropAddInfo_ptr)
{
	eOmc_controlmode_command_t *cmd_ptr =(eOmc_controlmode_command_t *)ropAddInfo_ptr->desc.data;
    char modestr[50];

//    switch(*cmd_ptr)
//    {
//		case eomc_controlmode_cmd_position:
//		{
//			sprintf(modestr, "_controlmode_cmd_position");
//			break;
//		}
//
//		case eomc_controlmode_cmd_velocity:
//		{
//			sprintf(modestr, "_controlmode_cmd_velocity");
//			break;
//		}
//
//		case eomc_controlmode_cmd_torque:
//		{
//			sprintf(modestr, "_controlmode_cmd_torque");
//			break;
//		}
//
//		case eomc_controlmode_cmd_current:
//		{
//			sprintf(modestr, "_controlmode_cmd_current");
//			break;
//		}
//
//		case eomc_controlmode_cmd_openloop:
//		{
//			sprintf(modestr, "_controlmode_cmd_openloop");
//			break;
//		}
//
//		case eomc_controlmode_cmd_switch_everything_off:
//		{
//			sprintf(modestr, "_controlmode_cmd_switch_everything_off");
//			break;
//		}
//
//		default:
//		{
//			sprintf(modestr, "no match!! %hhd", (*cmd_ptr));
//			break;
//
//		}
//    }


	printf("set control mode: op:0x%x ep=0x%x id=0x%x cmd_val=%hhd\n", ropAddInfo_ptr->desc.ropcode, ropAddInfo_ptr->desc.ep, ropAddInfo_ptr->desc.id, (*cmd_ptr));

	fflush(stdout);
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




