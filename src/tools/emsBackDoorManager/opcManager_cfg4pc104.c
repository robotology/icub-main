/*
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
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

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------


#include "stdio.h" 
#include "stdlib.h" 
#include "string.h" 
#include "EoCommon.h"
#include "EoDiagnostics.h"
#include "OPCprotocolManager.h"
#include "OPCprotocolManager_Cfg.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------
extern eOdgn_commands_t dgnCommands;

// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
static void s_print_emsperiph_candata(eOdgn_canstatus_t *canst_ptr);
static void on_rec_emsperiph(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata);

static void on_rec_emsapplcommon(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata);
static void s_print_emsapplcomm_core(eOdgn_coreapplication_t *appcore_ptr);
static void s_print_emsapplcomm_ipnet(eOdgn_ipnet_t  *appipnet_ptr);
static void s_print_emsapplcomm_transceiver(eOdgn_embObjtransceiver_t  *apptrans_ptr);

static void on_rec_emsapplmc(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata);
static void s_print_emsapplmc_encoderserror(eOdgn_encoderreads_t *encreads);

static void on_rec_motorstflags(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata);
static void on_rec_errorLog(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata);

/*
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
*/




// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern opcprotman_res_t opcprotman_personalize_database(OPCprotocolManager *p)
{
    opcprotman_res_t res = opcprotman_OK;

/* personalize eodgn_nvidbdoor_cmd */
	res = opcprotman_personalize_var(   p,
                                        eodgn_nvidbdoor_cmds,
                                        (void*)&dgnCommands,
                                        NULL); //here I'm not interested in callback func on received, becose ems never will send me it

    if(opcprotman_OK != res)
    {
        return(res);
    }
/* personalize eodgn_nvidbdoor_errorlog */
	res = opcprotman_personalize_var(   p,
										eodgn_nvidbdoor_errorlog,
										NULL,//use NULL because i'd like print received data and not store them!!
                                 	   //pay attention: see NOTE 1 at the end of this function!!!
										on_rec_errorLog);

	if(opcprotman_OK != res)
	{
		return(res);
	}
/*personalize eodgn_nvidbdoor_emsperiph var*/
	res = opcprotman_personalize_var(   p,
                                        eodgn_nvidbdoor_emsperiph,
                                        NULL,  //use NULL because i'd like print received data and not store them!!
                                        	   //pay attention: see NOTE 1 at the end of this function!!!
                                        on_rec_emsperiph);

    if(opcprotman_OK != res)
    {
        return(res);
    }


/*personalize eodgn_nvidbdoor_emsapplcommon var*/
	res = opcprotman_personalize_var(   p,
                                        eodgn_nvidbdoor_emsapplcommon,
                                        NULL,  //use NULL because i'd like print received data and not store them!!
                                        	   //pay attention: see NOTE 1 at the end of this function!!!
                                        on_rec_emsapplcommon);

    if(opcprotman_OK != res)
    {
        return(res);
    }


/*personalize eodgn_nvidbdoor_emsapplmc var*/
	res = opcprotman_personalize_var(   p,
                                        eodgn_nvidbdoor_emsapplmc,
                                        NULL,  //use NULL because i'd like print received data and not store them!!
                                        	   //pay attention: see NOTE 1 at the end of this function!!!
                                        //NULL);
                                        on_rec_emsapplmc);

    if(opcprotman_OK != res)
    {
        return(res);
    }


/*personalize eodgn_nvidbdoor_motorstatus var*/
	res = opcprotman_personalize_var(   p,
                                        eodgn_nvidbdoor_motorstatus,
                                        NULL,  //use NULL because i'd like print received data and not store them!!
                                        	   //pay attention: see NOTE 1 at the end of this function!!!
                                        on_rec_motorstflags);

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
static void s_print_emsperiph_candata(eOdgn_canstatus_t *canst_ptr)
{
    printf("\t\twarning=%d passive=%d busoff=%d\n", canst_ptr->hw.warning, canst_ptr->hw.passive, canst_ptr->hw.busoff);  
    printf("\t\thw_rx_queue_is_full=%d\n", canst_ptr->hw.rxqueueisfull);
    printf("\t\tsw_rx_queue_is_full=%d sw_tx_queue_is_full=%d\n", canst_ptr->sw.rxqueueisfull, canst_ptr->sw.txqueueisfull);
}

static void on_rec_emsperiph(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata)
{
    eOdgn_emsperipheralstatus_t* data = (eOdgn_emsperipheralstatus_t*)recdata;
    
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
        
            printf("-----EMS periph data---\n");
            printf("\t CAN 1:\n");
            s_print_emsperiph_candata(&data->can_dev[0]);
            printf("\t CAN 2:\n");
            s_print_emsperiph_candata(&data->can_dev[1]);
            printf("\t ETH MASK: 0x%x\n", data->eth_dev.linksmask);
                    
        } break;
    }       


}

static void on_rec_emsapplcommon(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata)
{

    eOdgn_emsapplication_common_t* data = (eOdgn_emsapplication_common_t*)recdata;
    
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
        
            printf("-----EMS appl common---\n");

            s_print_emsapplcomm_core(&data->core);
            s_print_emsapplcomm_ipnet(&data->ipnet);
            s_print_emsapplcomm_transceiver(&data->transceiver);

            fflush(stdout);    
        } break;
    }       
}

static void s_print_emsapplcomm_core(eOdgn_coreapplication_t *appcore_ptr)
{
    printf("\tRUNNING ST\n");
	    printf("\t\tnumberofperiods=%lld\n ", appcore_ptr->runst.numberofperiods);
	    printf("\t\tcumulativeabsoluteerrorinperiod=%lld\n ", appcore_ptr->runst.cumulativeabsoluteerrorinperiod);
	    printf("\t\tmeanofabsoluteerrorinperiod=%d\n ", appcore_ptr->runst.meanofabsoluteerrorinperiod);

	    printf("\t\tmovingmeanofabsoluteerrorinperiod=%d\n ", appcore_ptr->runst.movingmeanofabsoluteerrorinperiod);
	    printf("\t\tmaxabsoluteerrorinperiod=%d\n ", appcore_ptr->runst.maxabsoluteerrorinperiod);
	    printf("\t\tminabsoluteerrorinperiod=%d\n ", appcore_ptr->runst.minabsoluteerrorinperiod);

	    printf("\t\texecutionoverflows[0]=%d\n ", appcore_ptr->runst.executionoverflows[0]);
	    printf("\t\texecutionoverflows[1]=%d\n ", appcore_ptr->runst.executionoverflows[1]);
	    printf("\t\texecutionoverflows[2]=%d\n ", appcore_ptr->runst.executionoverflows[2]);
	    printf("\t\tdatagrams_failed_to_go_in_txsocket=%d\n ", appcore_ptr->runst.datagrams_failed_to_go_in_txsocket);
        printf("\t\tcantxfailuretimeoutsemaphore=%d\n ", appcore_ptr->runst.cantxfailuretimeoutsemaphore);

    printf("\tCONFIG ST\n");
        printf("\t\tcantxfailuretimeoutsemaphore=%d\n ", appcore_ptr->cfgst.cantxfailuretimeoutsemaphore);
}
static void s_print_emsapplcomm_ipnet(eOdgn_ipnet_t  *appipnet_ptr)
{
    printf("\tIPNET INFO\n");
	    printf("\t\tdatagrams_failed_to_go_in_rxfifo=%d\n ", appipnet_ptr->datagrams_failed_to_go_in_rxfifo);
	    printf("\t\tdatagrams_failed_to_go_in_txosalqueue=%d\n ", appipnet_ptr->datagrams_failed_to_go_in_txosalqueue);
	    printf("\t\tdatagrams_failed_to_be_retrieved_from_txfifo=%d\n ", appipnet_ptr->datagrams_failed_to_be_retrieved_from_txfifo);
	    printf("\t\tdatagrams_failed_to_be_sent_by_ipal=%d\n ", appipnet_ptr->datagrams_failed_to_be_sent_by_ipal);
}
static void s_print_emsapplcomm_transceiver(eOdgn_embObjtransceiver_t  *apptrans_ptr)
{

    printf("\tTRANSCEIVER INFO\n");
	    printf("\t\trxinvalidropframes=%d\n ", apptrans_ptr->rxinvalidropframes);
	    printf("\t\trxseqnumwrong=%d\n ", apptrans_ptr->rxseqnumwrong);
	    printf("\t\tlostreplies=%d\n ", apptrans_ptr->lostreplies);
	    printf("\t\tfailuresinloadofreplyropframe=%d\n ", apptrans_ptr->failuresinloadofreplyropframe);
	    printf("\t\ttxropframeistoobigforthepacket=%d\n ", apptrans_ptr->txropframeistoobigforthepacket);
	    printf("\t\tcannotloadropinregulars=%d\n ", apptrans_ptr->cannotloadropinregulars);
	    printf("\t\tcannotloadropinoccasionals=%d\n ", apptrans_ptr->cannotloadropinoccasionals);


}


static void on_rec_emsapplmc(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata)
{

    eOdgn_emsapplication_emswithmc_t* data = (eOdgn_emsapplication_emswithmc_t*)recdata;
    
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
        
            printf("-----EMS appl mc---\n");

            s_print_emsapplmc_encoderserror(&(data->encreads));

            fflush(stdout);    
        } break;
    }      

}

static void s_print_emsapplmc_encoderserror(eOdgn_encoderreads_t *encreads)
{
    uint32_t i;
    for(i=0; i<6; i++)
    {
        printf("Encoder num %d\n", i);
        printf("\t err_onReadFromSpi=%d   ", encreads->encList[i].err_onReadFromSpi);  
        printf("err_onParityError=%d  ", encreads->encList[i].err_onParityError);  
        printf("err_onInvalidValue=%d\n", encreads->encList[i].err_onInvalidValue);  
    }
}

static void on_rec_motorstflags(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata)
{
    eOdgn_motorstatusflags_t* data = (eOdgn_motorstatusflags_t*)recdata;
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
        
            printf("-----motor status flag---\n");

            for(i=0; i<12; i++)
            {
                if(data->motorlist[i] != 0)
                {
                    printf("\tmotor %d with err flags=0x%x\n", i, data->motorlist[i]);
                }
            }


            fflush(stdout);    
        } break;
    }      

}


static void on_rec_errorLog(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata)
{
	eOdgn_errorlog_t *data = (eOdgn_errorlog_t *)recdata;

	printf("----- switch to error state because... \n");
	printf("\t%s\n", data->errorstate_str);
}
// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------





