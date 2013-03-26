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


extern EOMtheIPnetDEBUG_t eom_ipnet_hid_DEBUG_of_ems =
{
    .datagrams_failed_to_go_in_rxfifo               = 0,
    .datagrams_failed_to_go_in_txosalqueue          = 0,
    .datagrams_failed_to_be_retrieved_from_txfifo   = 0,
    .datagrams_failed_to_be_sent_by_ipal            = 0    
};

extern EOMtheEMSrunnerDEBUG_t eom_emsrunner_hid_DEBUG_of_ems =
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

extern EOMtheEMStransceiverDEBUG_t eom_emstransceiver_hid_DEBUG_of_ems =
{
    .rxinvalidropframes                     = 0,
    .errorsinsequencenumber                 = 0,
    .lostreplies                            = 0,
    .failuresinloadofreplyropframe          = 0,
    .txropframeistoobigforthepacket         = 0,
    .cannotloadropinregulars                = 0,
    .cannotloadropinoccasionals             = 0    
};

static opcprotman_var_map_t s_myarray[] = 
{
    {
        .var        = 1,
        .size       = sizeof(EOMtheIPnetDEBUG_t),
        .ptr        = &eom_ipnet_hid_DEBUG_of_ems,
        .onrec      = on_rec_ipnet_debug
    },
    {
        .var        = 2,
        .size       = sizeof(EOMtheEMSrunnerDEBUG_t),
        .ptr        = &eom_emsrunner_hid_DEBUG_of_ems,
        .onrec      = on_rec_runner_debug
    },
    {
        .var        = 3,
        .size       = sizeof(EOMtheEMStransceiverDEBUG_t),
        .ptr        = &eom_emstransceiver_hid_DEBUG_of_ems,
        .onrec      = on_rec_transceiver_debug
    },
    
};

extern opcprotman_cfg_t opcprotmanCFGv0x1234 =
{
    .databaseversion        = 0x1234,
    .numberofvariables      = 3,
    .arrayofvariablemap     = s_myarray
};

// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------


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


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------





