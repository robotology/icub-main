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

// acemor: ... dont need posix
//#include <unistd.h>
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
static void my_cbk_onPidFound(eOethLowLevParser_packetInfo_t *pktInfo_ptr, eODeb_eoProtoParser_ropAdditionalInfo_t *ropAddInfo_ptr);
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
                            EO_INIT(.size)           36,
                },

				EO_INIT(.data)
				{
                    //j 0
					{0xffff, 0x9c01}, //NVID_jxx_jconfig__pidposition
					{0xffff, 0x9c03}, //NVID_jxx_jconfig__pidtorque
					{0xffff, 0x9c04}, //NVID_jxx_jconfig__impedance
                    //j 1
                                        {0xffff, 0x9c21}, //NVID_jxx_jconfig__pidposition
					{0xffff, 0x9c23}, //NVID_jxx_jconfig__pidtorque
					{0xffff, 0x9c24}, //NVID_jxx_jconfig__impedance
                    //j 2
                    {0xffff, 0x9c41}, //NVID_jxx_jconfig__pidposition
                    {0xffff, 0x9c43}, //NVID_jxx_jconfig__pidtorque
                    {0xffff, 0x9c44}, //NVID_jxx_jconfig__impedance
                    //j 3
                    {0xffff, 0x9c61}, //NVID_jxx_jconfig__pidposition
                    {0xffff, 0x9c63}, //NVID_jxx_jconfig__pidtorque
                    {0xffff, 0x9c64}, //NVID_jxx_jconfig__impedance
                    //j 4
                    {0xffff, 0x9c81}, //NVID_jxx_jconfig__pidposition
                    {0xffff, 0x9c83}, //NVID_jxx_jconfig__pidtorque
                    {0xffff, 0x9c84}, //NVID_jxx_jconfig__impedance
                    //j 5
                    {0xffff, 0x9ca1}, //NVID_jxx_jconfig__pidposition
                    {0xffff, 0x9ca3}, //NVID_jxx_jconfig__pidtorque
                    {0xffff, 0x9ca4}, //NVID_jxx_jconfig__impedance
                    //j 6
                    {0xffff, 0x9cc1}, //NVID_jxx_jconfig__pidposition
                    {0xffff, 0x9cc3}, //NVID_jxx_jconfig__pidtorque
                    {0xffff, 0x9cc4}, //NVID_jxx_jconfig__impedance
                    //j 7
                    {0xffff, 0x9ce1}, //NVID_jxx_jconfig__pidposition
                    {0xffff, 0x9ce3}, //NVID_jxx_jconfig__pidtorque
                    {0xffff, 0x9ce4}, //NVID_jxx_jconfig__impedance
                    //j 8
                    {0xffff, 0x9d01}, //NVID_jxx_jconfig__pidposition
                    {0xffff, 0x9d03}, //NVID_jxx_jconfig__pidtorque
                    {0xffff, 0x9d04}, //NVID_jxx_jconfig__impedance
                    //j 9
                    {0xffff, 0x9d21}, //NVID_jxx_jconfig__pidposition
                    {0xffff, 0x9d23}, //NVID_jxx_jconfig__pidtorque
                    {0xffff, 0x9d24}, //NVID_jxx_jconfig__impedance
                    //j 10
                    {0xffff, 0x9d41}, //NVID_jxx_jconfig__pidposition
                    {0xffff, 0x9d43}, //NVID_jxx_jconfig__pidtorque
                    {0xffff, 0x9d44}, //NVID_jxx_jconfig__impedance
                    //j 11
                    {0xffff, 0x9d61}, //NVID_jxx_jconfig__pidposition
                    {0xffff, 0x9d63}, //NVID_jxx_jconfig__pidtorque
                    {0xffff, 0x9d64}, //NVID_jxx_jconfig__impedance

                }

            },
            EO_INIT(.cbk_onNVfound)            my_cbk_onPidFound
        },

        EO_INIT(.invalidRopFrame)
        {
            EO_INIT(.cbk)                      NULL //my_cbk_onInvalidRopFrame

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


static void my_cbk_onPidFound(eOethLowLevParser_packetInfo_t *pktInfo_ptr, eODeb_eoProtoParser_ropAdditionalInfo_t *ropAddInfo_ptr)
{

	printf("Pid found: op:0x%x ep=0x%x id=0x%x from %d to %d\n", ropAddInfo_ptr->desc.ropcode, ropAddInfo_ptr->desc.ep, ropAddInfo_ptr->desc.id, pktInfo_ptr->src_addr, pktInfo_ptr->dst_addr);

}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




