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
extern uint32_t cmdena_rxsetPointCheck;
extern uint8_t board;

// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
typedef struct
{
    uint32_t sum;
} ethCounters_total;



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

static void on_rec_canQueueStatistics(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata);
static void on_rec_rxcheckSetpoints(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata);


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
static eOdgn_emsapplication_emswithmc_t s_emswithmc_data;
static ethCounters_total ethCounterBoards[9][3] = {{0}}; //9= num of boards; 3 is num of eth links

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
                                        (void*)&s_emswithmc_data,
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

    /*personalize eodgn_nvidbdoor_jointsStateCmd var*/
        res = opcprotman_personalize_var(   p,
                                            eodgn_nvidbdoor_canQueueStatistics,
                                            NULL,
                                            on_rec_canQueueStatistics);

        if(opcprotman_OK != res)
        {
            return(res);
        }
   /*personalize eodgn_nvidbdoor_rxcheckSetpoints var */
        res = opcprotman_personalize_var(   p,
                                            eodgn_nvidbdoor_rxcheckSetpoints,
                                            NULL,
                                            on_rec_rxcheckSetpoints);

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
    printf("\t\tsw_rx_queue_is_full=%d sw_tx_queue_is_full=%d dummy=0x%x\n", canst_ptr->sw.rxqueueisfull, canst_ptr->sw.txqueueisfull, canst_ptr->sw.dummy);
}

static void on_rec_emsperiph(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata)
{
    eOdgn_emsperipheralstatus_t* data = (eOdgn_emsperipheralstatus_t*)recdata;
    uint8_t i;
    uint8_t myboard;

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
            for(i=0; i<3; i++)
            {
                if(((data->eth_dev.crcErrorCnt_validVal & (1<<i)) == (1<<i)))
                {
                    int overflow=0;
                    if(((data->eth_dev.crcErrorCnt_overflow & (1<<i)) == (1<<i)))
                    {
                        overflow =1;
                    }
                    printf("\t CRC_error phy-port %d: %d overflow=%d", i, data->eth_dev.crcErrorCnt[i], overflow);
                    if(board>9)
                    {
                        //in case i use board on desk with addr 99
                        myboard = 0;
                    }
                    else
                    {
                        myboard = board-1;
                    }
                    ethCounterBoards[myboard][i].sum +=  data->eth_dev.crcErrorCnt[i];
                    printf(" Sum=%d", ethCounterBoards[myboard][i].sum);
                }
                else
                {
                    printf("\t CRC_error phy-port %d: INVALID VALUE", i);
                }

            }
            printf("\n");
            for(i=0; i<3; i++)
            {
                printf("\t i2cError %d on phy-port %d", data->eth_dev.i2c_error[i], i);
            }
            printf("\n");
            fflush(stdout);

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
        printf("\t\tnumberofperiods=%ld\n ", appcore_ptr->runst.numberofperiods);
        printf("\t\tcumulativeabsoluteerrorinperiod=%ld\n ", appcore_ptr->runst.cumulativeabsoluteerrorinperiod);
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


//            s_print_emsapplmc_encoderserror(&(data->encreads));
//
//
//            printf("Limited current mask: 0x%x\n", data->encreads.dummy);
//
//            fflush(stdout);
//            memcpy(&s_emswithmc_data, data, sizeof(eOdgn_emsapplication_emswithmc_t));



            if(memcmp(&data->encreads.encList[0], &s_emswithmc_data.encreads.encList[0], eOdgn_emsmaxnumofencoders*sizeof(eOappEncReader_error_t)) != 0)
            {
                //if i received new data about encoders' error
                s_print_emsapplmc_encoderserror(&(data->encreads));
            }
            fflush(stdout);
            memcpy(&s_emswithmc_data, data, sizeof(eOdgn_emsapplication_emswithmc_t));

        } break;
    }

}

static void s_print_emsapplmc_encoderserror(eOdgn_encoderreads_t *encreads)
{
    uint32_t i;
    for(i=0; i<6; i++)
    {
        printf("Encoder num %d\t", i);
        printf("\t err_onReadFromSpi=%d   ", encreads->encList[i].err_onReadFromSpi);
        printf("err_onParityError=%d  ", encreads->encList[i].err_onParityError);
        printf("err_onInvalidValue=%d  \n", encreads->encList[i].err_onInvalidValue);
    }
    printf("\tcount=%d\n", encreads->count);
}

static void on_rec_motorstflags(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata)
{
    eOdgn_motorstatusflags_t* data = (eOdgn_motorstatusflags_t*)recdata;
    uint8_t i;
    char stroutput[300];

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
                    sprintf(stroutput, "motor %d with err flags=0x%x: ", i, data->motorlist[i]);
                    if((data->motorlist[i] & DGN_MOTOR_FAULT_UNDERVOLTAGE) == DGN_MOTOR_FAULT_UNDERVOLTAGE)
                        strcat(stroutput, "UNDERVOLTAGE, ");
                    if((data->motorlist[i] & DGN_MOTOR_FAULT_OVERVOLTAGE) == DGN_MOTOR_FAULT_OVERVOLTAGE)
                        strcat(stroutput, "OVERVOLTAGE, ");
                    if((data->motorlist[i] & DGN_MOTOR_FAULT_EXTERNAL) == DGN_MOTOR_FAULT_EXTERNAL)
                        strcat(stroutput, "EXTERNAL, ");
                    if((data->motorlist[i] & DGN_MOTOR_FAULT_OVERCURRENT) == DGN_MOTOR_FAULT_OVERCURRENT)
                        strcat(stroutput, "OVERCURRENT, ");
                    if((data->motorlist[i] & DGN_MOTOR_FAULT_I2TFAILURE) == DGN_MOTOR_FAULT_I2TFAILURE)
                        strcat(stroutput, "I2TFAILURE, ");
                    if((data->motorlist[i] & DGN_MOTOR_FAULT_CANRECWARNING) == DGN_MOTOR_FAULT_CANRECWARNING)
                        strcat(stroutput, "CANRECWARNING, ");
                    if((data->motorlist[i] & DGN_MOTOR_FAULT_CANRECERROR) == DGN_MOTOR_FAULT_CANRECERROR)
                        strcat(stroutput, "CANRECERROR, ");
                    if((data->motorlist[i] & DGN_MOTOR_FAULT_CANRECHWOVERRUN) == DGN_MOTOR_FAULT_CANRECHWOVERRUN)
                        strcat(stroutput, "CANRECHWOVERRUN ");
                    printf("%s\n", stroutput);
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

static void on_rec_canQueueStatistics(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata)
{
    eOdgn_can_statistics_t* data = (eOdgn_can_statistics_t*)recdata;
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

            printf("-----can queue statistics ---\n");

            printf("Config_mode: ");
            for(i=0; i<2; i++)
            {
                printf("\t port %d ==> rx.min=%d, rx.max=%d, tx.min=%d, tx.max=%d\n\t",i, data->config_mode.stat[i].info_rx.min, data->config_mode.stat[i].info_rx.max,
                                                                                        data->config_mode.stat[i].info_tx.min, data->config_mode.stat[i].info_tx.max);
            }
            printf("\nRun_mode: ");
            for(i=0; i<2; i++)
            {
                printf("\t port %d ==> rx.min=%d, rx.max=%d, tx.min=%d, tx.max=%d\n\t",i, data->run_mode.stat[i].info_rx.min, data->run_mode.stat[i].info_rx.max,
                                                                                        data->run_mode.stat[i].info_tx.min, data->run_mode.stat[i].info_tx.max);
            }
            fflush(stdout);
        } break;
    }


}
//static void on_rec_jointsStateCmd(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata)
//{
//    eOdgn_jointsStateCmd_t* data = (eOdgn_jointsStateCmd_t*)recdata;
//    uint8_t i;
//
//    switch(opc)
//    {
//
//        default:
//        case opcprotman_opc_set:
//        {   // nobody can order that to us
//            // we just dont do it ...
//        } break;
//
//        case opcprotman_opc_say:    // someboby has replied to a ask we sent
//        case opcprotman_opc_sig:    // someboby has spontaneously sent some data
//        {
//
//            printf("-----joints last received cmd---\n");
//
//            for(i=0; i<12; i++)
//            {
//
//              printf("j %d cmd=0x%x last_two_are_equal=%d\n", i, data->jLastRecCmd[i], data->lastTwoEqual[i]);
//            }
//
//
//            fflush(stdout);
//        } break;
//    }
//
//}




static void on_rec_rxcheckSetpoints(opcprotman_opc_t opc, opcprotman_var_map_t* map, void* recdata)
{

    eOdgn_rxCheckSetpoints_t* data = (eOdgn_rxCheckSetpoints_t*)recdata;
    uint8_t i;

    if(!cmdena_rxsetPointCheck)
    {
        return;
    }


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

            printf("\n\n-----rx check setpoints---\n");

            for(i=0; i<4; i++)
            {
                //printf("\tj %d: pos-deltaprognum 0x%x   pos-deltarxtime0x%x    imp-deltaprognum 0x%x   imp-deltarxtime0x%x\n", i,  data->position[i].deltaprognumber,  data->position[i].deltarxtime,  data->impedence[i].deltaprognumber, data->impedence[i].deltarxtime);
                printf("\tj %d: pos-deltaprognum =", i );
                if(data->position[i].deltaprognumber == INT32_MAX)
                {
                    printf(" __ ");
                }
                else
                {
                    printf(" %d ", data->position[i].deltaprognumber);
                }

                printf("    pos-deltarxtime =");
                if(data->position[i].deltarxtime == UINT32_MAX)
                {
                    printf(" __ ");
                }
                else
                {
                    printf(" %d ", data->position[i].deltarxtime);
                }

                    printf("    imp-deltaprognum =");

                if(data->impedence[i].deltaprognumber == INT32_MAX)
                {
                    printf(" __ ");
                }
                else
                {
                    printf(" %d ", data->impedence[i].deltaprognumber);
                }


                printf("    imp-deltarxtime=");

                if(data->impedence[i].deltarxtime == UINT32_MAX)
                {
                    printf(" __ \n");
                }
                else
                {
                    printf(" %d \n", data->impedence[i].deltarxtime);
                }

            }


            fflush(stdout);
        } break;
    }


}
// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------





