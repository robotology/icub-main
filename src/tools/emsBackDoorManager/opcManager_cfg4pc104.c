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

extern void on_rec_emscontroller_debug(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata);

extern void on_rec_canFaultLog_debug(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata);

extern void on_rec_encoderError_debug(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata);


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


/* personalize eo_emsController_hid_DEBUG_id var*/
	res = opcprotman_personalize_var(   p,
                                        eo_emsController_hid_DEBUG_id,
                                        NULL,  //use NULL because i'd like print received data and not store them!!
                                        	   //pay attention: see NOTE 1 at the end of this function!!!
                                        on_rec_emscontroller_debug);

    if(opcprotman_OK != res)
    {
        return(res);
    }


/* personalize eo_emsController_hid_DEBUG_id var*/
	res = opcprotman_personalize_var(   p,
                                        eo_canFaultLogDEBUG_id,
                                        NULL,  //use NULL because i'd like print received data and not store them!!
                                        	   //pay attention: see NOTE 1 at the end of this function!!!
                                        on_rec_canFaultLog_debug);

    if(opcprotman_OK != res)
    {
        return(res);
    }

    /* personalize eo_emsController_hid_DEBUG_id var*/
	res = opcprotman_personalize_var(   p,
                                         eo_EncoderErrorDEBUG_id ,
                                         NULL,  //use NULL because i'd like print received data and not store them!!
                                         	   //pay attention: see NOTE 1 at the end of this function!!!
                                        on_rec_encoderError_debug);

    if(opcprotman_OK != res)
    {
        return(res);
    }

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





