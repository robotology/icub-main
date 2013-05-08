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
#include "OPCprotocolManager_Cfg.h"


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



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------


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

extern opcprotman_res_t opcprotman_personalize_database(OPCprotocolManager *p)
{
    opcprotman_res_t res = opcprotman_OK;

//commented because not used!!!
///* personalize eom_ipnet_hid_DEBUG_id var*/
//	res = opcprotman_personalize_var(   p,
//                                        eom_ipnet_hid_DEBUG_id,
//                                        (uint8_t*) &eom_ipnet_hid_DEBUG,
//                                        on_rec_ipnet_debug);
//
//    if(opcprotman_OK != res)
//    {
//        return(res);
//    }

/* personalize eom_emsrunner_hid_DEBUG_id var*/
	res = opcprotman_personalize_var(   p,
                                        eom_emsrunner_hid_DEBUG_id,
                                        NULL,  //use NULL because i'd like print received data and not store them!!
                                        	   //pay attention: see NOTE 1 at the end of this function!!!
                                        on_rec_runner_debug);

    if(opcprotman_OK != res)
    {
        return(res);
    }

//commented because not used!!!
///* personalize eom_emstransceiver_hid_DEBUG_id var*/
//	res = opcprotman_personalize_var(   p,
//                                        eom_emstransceiver_hid_DEBUG_id,
//                                        (uint8_t*)&eom_emstransceiver_hid_DEBUG,
//                                        on_rec_transceiver_debug);
//
//    if(opcprotman_OK != res)
//    {
//        return(res);
//    }




    return(res);

    /* NOTE 1:
     * if ems doesn't send ask, that needs a reply, it is possible use NULL like data pointer.
     * in case of replay the  opcprotman object replay by coping data pointed by var_ram param to the message response automatically,
     * so if var_ram is null the application will not send the response.
     * Moreover you can use NULL pointer even you are not interested in storing  received data, but only check or print them.
     * */
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





