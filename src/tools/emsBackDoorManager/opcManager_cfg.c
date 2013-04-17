/*
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Marco Accame
 * email:   marco.accame@iit.it
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

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------


#include "stdio.h" 
#include "stdlib.h" 
#include "string.h" 
#include "EoCommon.h"
#include "OPCprotocolManager.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------


extern void on_rec_ipnet_debug(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata);

extern void on_rec_runner_debug(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata);

extern void on_rec_transceiver_debug(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata);

extern void on_rec_emscontroller_debug(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata);

extern void on_rec_canFaultLog_debug(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata);

extern void on_rec_encoderError_debug(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata);


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
#define eo_emsrunner_task_numberof 3


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------


typedef struct
{
    uint32_t    datagrams_failed_to_go_in_rxfifo;
    uint32_t    datagrams_failed_to_go_in_txosalqueue;
    uint32_t    datagrams_failed_to_be_retrieved_from_txfifo;
    uint32_t    datagrams_failed_to_be_sent_by_ipal;    
} EOMtheIPnetDEBUG_t;


typedef struct
{
    uint64_t    numberofperiods;
    uint64_t    cumulativeabsoluteerrorinperiod;  
    uint32_t    meanofabsoluteerrorinperiod;
    uint32_t    movingmeanofabsoluteerrorinperiod;
    uint32_t    maxabsoluteerrorinperiod;
    uint32_t    minabsoluteerrorinperiod;  
    uint32_t    executionoverflows[eo_emsrunner_task_numberof];
    uint32_t    datagrams_failed_to_go_in_txsocket;    
} EOMtheEMSrunnerDEBUG_t;


typedef struct
{
    uint32_t    rxinvalidropframes;
    uint32_t    errorsinsequencenumber;
    uint32_t    lostreplies;
    uint32_t    failuresinloadofreplyropframe;
    uint32_t    txropframeistoobigforthepacket;
    uint32_t    cannotloadropinregulars;
    uint32_t    cannotloadropinoccasionals;
} EOMtheEMStransceiverDEBUG_t;


typedef struct
{   // 4 is MAX_JOINTS but i use 4 to avoid ... problems with the host
	uint8_t					    boardid;
    uint8_t                     count[4];
    int32_t                     position[4];
    int32_t                     velocity[4];
} EOemsControllerDEBUG_t;


typedef struct
{
    int16_t currSetPointList[4];
    eOcanframe_t overCurrentMsg;
    eOcanframe_t nextCanMsgs[6];
}EOcanFaultLogDEBUG_t; //size 2*4 + 16 +16*6 = 120

typedef struct
{
    uint16_t parityCheck[6];
    uint16_t status[6];
} EOencoderErrorDEBUG_t;


EOMtheIPnetDEBUG_t eom_ipnet_hid_DEBUG =
{
    .datagrams_failed_to_go_in_rxfifo               = 0,
    .datagrams_failed_to_go_in_txosalqueue          = 0,
    .datagrams_failed_to_be_retrieved_from_txfifo   = 0,
    .datagrams_failed_to_be_sent_by_ipal            = 0    
};

EOMtheEMSrunnerDEBUG_t eom_emsrunner_hid_DEBUG =
{
    .numberofperiods                            = 0,
    .cumulativeabsoluteerrorinperiod            = 0,
    .meanofabsoluteerrorinperiod                = 0,
    .movingmeanofabsoluteerrorinperiod          = 0,
    .maxabsoluteerrorinperiod                   = 0,
    .minabsoluteerrorinperiod                   = 1000000000,
    .executionoverflows                         = {0, 0, 0},
    .datagrams_failed_to_go_in_txsocket         = 0  
};

EOMtheEMStransceiverDEBUG_t eom_emstransceiver_hid_DEBUG =
{
    .rxinvalidropframes                     = 0,
    .errorsinsequencenumber                 = 0,
    .lostreplies                            = 0,
    .failuresinloadofreplyropframe          = 0,
    .txropframeistoobigforthepacket         = 0,
    .cannotloadropinregulars                = 0,
    .cannotloadropinoccasionals             = 0    
};


EOemsControllerDEBUG_t eo_emsController_hid_DEBUG =
{
		.boardid 							= 0,
		.count								= {0,0,0,0},
		.position							= {0,0,0,0},
		.velocity							= {0,0,0,0}
};

EOcanFaultLogDEBUG_t EOcanFaultLogDEBUG = {0};

EOencoderErrorDEBUG_t EOencoderErrorDEBUG = {0};

#define eom_ipnet_hid_DEBUG_id 				1
#define eom_emsrunner_hid_DEBUG_id		 	2
#define eom_emstransceiver_hid_DEBUG_id 	3
#define eo_emsController_hid_DEBUG_id 		4
#define eo_canFaultLogDEBUG_id 				5
#define eo_EncoderErrorDEBUG_id             6


static opcprotman_var_map_t s_myarray[] = 
{
    {
        .var        = eom_ipnet_hid_DEBUG_id,
        .size       = sizeof(eom_ipnet_hid_DEBUG),
        .ptr        = &eom_ipnet_hid_DEBUG,
        .onrec      = on_rec_ipnet_debug
    },
    {
        .var        = eom_emsrunner_hid_DEBUG_id,
        .size       = sizeof(eom_emsrunner_hid_DEBUG),
        .ptr        = &eom_emsrunner_hid_DEBUG,
        .onrec      = on_rec_runner_debug
    },
    {
        .var        = eom_emstransceiver_hid_DEBUG_id,
        .size       = sizeof(eom_emstransceiver_hid_DEBUG),
        .ptr        = &eom_emstransceiver_hid_DEBUG,
        .onrec      = on_rec_transceiver_debug
    },
    {
        .var        = eo_emsController_hid_DEBUG_id,
        .size       = sizeof(eo_emsController_hid_DEBUG),
        .ptr        = &eo_emsController_hid_DEBUG,
        .onrec      = on_rec_emscontroller_debug
    },    
    {
        .var        = eo_canFaultLogDEBUG_id,
        .size       = sizeof(EOcanFaultLogDEBUG_t),
        .ptr        = &EOcanFaultLogDEBUG,
        .onrec      = on_rec_canFaultLog_debug
    },

     {
        .var        = eo_EncoderErrorDEBUG_id,
        .size       = sizeof(EOencoderErrorDEBUG_t),
        .ptr        = &EOencoderErrorDEBUG,
        .onrec      = on_rec_encoderError_debug
    } 
    
};

extern opcprotman_cfg_t opcprotmanCFGv0x1234 =
{
    .databaseversion        = 0x1234,
    .numberofvariables      = 6,
    .arrayofvariablemap     = s_myarray
};

// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
static void s_print_canmsg(eOcanframe_t *frame);

extern void on_rec_ipnet_debug(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata)
{   // for the host

    EOMtheIPnetDEBUG_t* data = (EOMtheIPnetDEBUG_t*)recdata;
    
    switch(opc)
    {
        
        default:
        case opcprotman_opc_set:
        {   // nobody can order that to us           
            // we just dont do it ...         
        } break;
        
        case opcprotman_opc_say:    // someboby has replied to a ask we sent
        case opcprotman_opc_sig:    // someboby has spontaneously sent some data
        {   
        
            printf("received data of ipnet");
        
        } break;
    }       
    
}

extern void on_rec_runner_debug(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata)
{   // for the host

    EOMtheEMSrunnerDEBUG_t* data = (EOMtheEMSrunnerDEBUG_t*)recdata;
    
    switch(opc)
    {
        
        default:
        case opcprotman_opc_set:
        {   // nobody can order that to us           
            // we just dont do it ...         
        } break;
        
        case opcprotman_opc_say:    // someboby has replied to a ask we sent
        case opcprotman_opc_sig:    // someboby has spontaneously sent some data
        {   
        
            printf("\n\n-----received data of runner---\n");

	    printf("numberofperiods=%lld\n ", data->numberofperiods);
	    printf("cumulativeabsoluteerrorinperiod=%lld\n ", data->cumulativeabsoluteerrorinperiod);
	    printf("meanofabsoluteerrorinperiod=%d\n ", data->meanofabsoluteerrorinperiod);

	    printf("movingmeanofabsoluteerrorinperiod=%d\n ", data->movingmeanofabsoluteerrorinperiod);
	    printf("maxabsoluteerrorinperiod=%d\n ", data->maxabsoluteerrorinperiod);
	    printf("minabsoluteerrorinperiod=%d\n ", data->minabsoluteerrorinperiod);

	    printf("executionoverflows[0]=%d\n ", data->executionoverflows[0]);
	    printf("executionoverflows[1]=%d\n ", data->executionoverflows[1]);
	    printf("executionoverflows[2]=%d\n ", data->executionoverflows[2]);
	    printf("datagrams_failed_to_go_in_txsocket=%d\n ", data->datagrams_failed_to_go_in_txsocket);
        
        } break;
    }       
    
}

extern void on_rec_transceiver_debug(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata)
{   // for the host
    
    switch(opc)
    {
        
        default:
        case opcprotman_opc_set:
        {   // nobody can order that to us           
            // we just dont do it ...         
        } break;
        
        case opcprotman_opc_say:    // someboby has replied to a ask we sent
        case opcprotman_opc_sig:    // someboby has spontaneously sent some data
        {   
        
            printf("received data of transceiver");
        
        } break;
    }       
    
}


extern void on_rec_emscontroller_debug(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata)
{   // for the host

	EOemsControllerDEBUG_t* data = (EOemsControllerDEBUG_t*)recdata;
	uint8_t i;

    switch(opc)
    {

        default:
        case opcprotman_opc_set:
        {   // nobody can order that to us
            // we just dont do it ...
        } break;

        case opcprotman_opc_say:    // someboby has replied to a ask we sent
        case opcprotman_opc_sig:    // someboby has spontaneously sent some data
        {

            float enc_factor , zero, enc_factor_6=182.044 , enc_factor_8=182.044, zero_6=180, zero_8=-180;
            //note: encoder factor is equal on both boards
            float vel, pos;

            if(data->boardid == 8)
            {
                enc_factor = enc_factor_8;
                zero = zero_8;
            }
            else if(data->boardid == 6)
            {
                enc_factor = enc_factor_6;
                zero = zero_6;
            }
            else
            {
                printf("\n\n ERROR: un expected board!!! %d \n ", data->boardid);
                return;
            }

            printf("\n\n-----received data emsController---\n");
            printf("BOARD-ID %d\n", data->boardid);
            for( i=0;i<4;i++)
            {
                pos = (data->position[i]/enc_factor)-zero;
                vel = data->velocity[i]/enc_factor;//rimosso fabs percheè se no non compilava, tanto non serviva piuù percheè enco_factor è sempre positivo

            	printf("\t\t j %d: count=%u  pos=%f (%d)  vel=%f (%d)\n", i, data->count[i], pos, data->position[i], vel, data->velocity[i]);
            }

        } break;
    }

}


extern void on_rec_canFaultLog_debug(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata)
{   // for the host

	EOcanFaultLogDEBUG_t* data = (EOcanFaultLogDEBUG_t*)recdata;
	uint8_t i;

    switch(opc)
    {

        default:
        case opcprotman_opc_set:
        {   // nobody can order that to us
            // we just dont do it ...
        } break;

        case opcprotman_opc_say:    // someboby has replied to a ask we sent
        case opcprotman_opc_sig:    // someboby has spontaneously sent some data
        {

			printf("\n\n OVER CURR FAULT INFO RECEIVED!!! \n");

			printf("Last curr setpoint:\n");
			for(i= 0; i<4; i++)
			{
				printf("\t j %d curr %d\n", i, data->currSetPointList[i]);
			}
			printf("Can msg with over current:\n");
			s_print_canmsg(&data->overCurrentMsg);

			printf("Next can msg:\n");
			for(i=0; i<6; i++)
			{
				s_print_canmsg(&data->nextCanMsgs[i]);
			}
        } break;
    }

}




extern void on_rec_encoderError_debug(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata)
{   // for the host

	EOencoderErrorDEBUG_t* data = (EOencoderErrorDEBUG_t*)recdata;
	uint8_t i;

    switch(opc)
    {

        default:
        case opcprotman_opc_set:
        {   // nobody can order that to us
            // we just dont do it ...
        } break;

        case opcprotman_opc_say:    // someboby has replied to a ask we sent
        case opcprotman_opc_sig:    // someboby has spontaneously sent some data
        {

		printf("\n\n encoder statistics!!! \n");


		for(i= 0; i<6; i++)
		{
			printf("\t enc %d parity check err = %d    status err = %d \n", i, data->parityCheck[i], data->status[i]);
		}
			
        } break;
    }

}
// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern opcprotman_cfg_t* get_OPCprotocolManager_cfg(void)
{
    return(&opcprotmanCFGv0x1234);
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void s_print_canmsg(eOcanframe_t *frame)
{
	uint8_t i;

	printf("id = 0x%x\t", frame->id);
	for(i=0; i<frame->size; i++)
	{
		printf(" %x", frame->data[i]);
	}
	printf("\n");
}

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------





