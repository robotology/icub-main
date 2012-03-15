// --------------------------------------------------------------------------------------------------------------------
// - doxy
// --------------------------------------------------------------------------------------------------------------------
/* @file       iCubRunningMsg_manager.c  CHE NOME BRUTTO!!!!! :(
	@brief      This file implements the task that wait eth datagram sent by pc104 in running mode.
                this datagram contains message for engine control and/or can-based baord.
	@author     valentina.gaggero@iit.it
    @date       12/13/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------
#include "stdlib.h"
#include "string.h"


//embObj
#include "eEcommon.h"
#include "EOMprodConsSharedData.h"
#include "EOMtask.h"
#include "EOSocketDatagram.h"
#include "EOMtheEntitiesEnv.h"
#include "EOMmutex.h"
#include "EOtheErrorManager.h"
#include "EOtimer.h"

// application
#include "appl_common.h"




// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------
#include "iCubRunningMsg_manager.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
//#include "iCubRunningMsg_manager_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
#define EVT_ICUBRUNMSG_MNG_STOP                 EVT_TASK_STOP
#define EVT_ICUBRUNMSG_MNG_START                EVT_TASK_START
#define EVT_ICUBRUNMSG_MNG_DGRAM_REC            (1 << 2)
#define EVT_ICUBRUNMSG_MNG_TIMEOUT              (1 << 3)



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables. deprecated: better using _get(), _set() on static variables 
// --------------------------------------------------------------------------------------------------------------------
extern EOMprodConsSharedData          *ethCan1_shData;
extern EOMprodConsSharedData          *ethCan2_shData;


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
/* questo tipo e' usato qui solo per test!!!*/
typedef struct
{
    uint16_t size;
    uint8_t *data_ptr;
} iCubRunMsg_Mng_canData_ptr_t;


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
static void s_iCubRunningMsg_manager_recDgram_mng(void);
static void s_iCubRunningMsg_manager_send_periodicDatagram(void);

//le seguenti funzioni sono usate solo per test finche' non ho la appl completa!!
static void s_iCubRunningMsg_manager_get_dataToSend_TEST(uint8_t **data_ptr, uint16_t *size_ptr);
static void s_iCubRunMsg_Mng_parser_TEST(uint8_t *eth_payload, uint16_t size);
static void s_iCubRunMsg_Mng_parser_getCANdata_TEST(uint8_t *eth_payload, iCubRunMsg_Mng_canData_ptr_t *CAN1data_ptr, iCubRunMsg_Mng_canData_ptr_t *CAN2data_ptr);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
static const char s_task_name[] = "iCubRunMsg_Mng";

/*--- state variables ---*/
/* NOTA: da sostituire con state_machine???*/
static eObool_t s_conected_st   = eobool_false;  //indicates if this task is connected with PC104 by socket.
static eObool_t s_running_st    = eobool_false;  //indicates if i have received start command already.

static eOipv4addr_t             s_remaddr;    //Remote address: initilized on connect and used to send datagram
static eOipv4port_t             s_remport;    //Remote port: initilized on connect and used to send datagram
static eOreltime_t              s_sendingDgramperiod; //if running, this task sends eth-dgram each <sendingDgramperiod> microsec


static EOaction                 *s_iCubRunMsg_Mng_action_onrx_dgram = NULL;
static EOsocketDatagram         *s_iCubRunMsg_Mng_dgramskt = NULL;
static EOpacket                 *s_iCubRunMsg_Mng_rxpkt = NULL;
static EOpacket                 *s_iCubRunMsg_Mng_txpkt = NULL;
static EOtimer                  *s_iCubRunMsg_Mng_timer_SendDtgram;
static EOaction                 *s_iCubRunMsg_Mng_action_timeout = NULL;



static uint8_t my_data_TEST[30];  //la uso per prendere i dati da spedire...poi e' da togliere!!!



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------
extern void iCubRunningMsg_manager_startup(EOMtask *p, uint32_t t)
{
    eOresult_t res;
    EOMtheEntitiesEnv_task_info_t my_task_info;


    //prepare action that will be execute on datagram received
    s_iCubRunMsg_Mng_action_onrx_dgram = eo_action_New();

    eo_action_SetEvent(s_iCubRunMsg_Mng_action_onrx_dgram, (eOevent_t)EVT_ICUBRUNMSG_MNG_DGRAM_REC , p);

    //create socket where pc104 send me messages
    s_iCubRunMsg_Mng_dgramskt = eo_socketdtg_New(4, ETH_RUN_DGRAM_PAYLOAD_SIZE, eom_mutex_New(), 
                                                 4, ETH_RUN_DGRAM_PAYLOAD_SIZE, eom_mutex_New());

    //prepare TX and RX packet
    s_iCubRunMsg_Mng_rxpkt = eo_packet_New(ETH_RUN_DGRAM_PAYLOAD_SIZE); 
    s_iCubRunMsg_Mng_txpkt = eo_packet_New(ETH_RUN_DGRAM_PAYLOAD_SIZE);

    //prepare timer for send periodically datagram
    s_iCubRunMsg_Mng_timer_SendDtgram =  eo_timer_New();
    s_iCubRunMsg_Mng_action_timeout =  eo_action_New();
    eo_action_SetEvent(s_iCubRunMsg_Mng_action_timeout, (eOevent_t)EVT_ICUBRUNMSG_MNG_TIMEOUT , p);

    //read value of configuration from ROM
#ifndef _WITHOUT_SHALIB_
     s_sendingDgramperiod = read_from_rom_period();    
#else
     s_sendingDgramperiod = 1000;
#endif

    //TODO: se voglio essere avvisata quando consumer (can1 or can2) ha prelevato un item
    //devo configurare qui la callbbkp.
    //EOMprodConsSharedData_ProducerCallbackSet(ethCan1_shData, my_func_signalToProducer1,my_arg1);
    //EOMprodConsSharedData_ProducerCallbackSet(ethCan2_shData, my_func_signalToProducer2,my_arg2);

    //Regitser me to EOMtheEntitiesEnv
    my_task_info.task_ptr = p;
    my_task_info.type = eom_mtask_EventDriven;

    res = EOMtheEntitiesEnv_register_task(EOMtheEntitiesEnv_GetHandle(), &my_task_info);
    if(eores_OK != res)
    {
        eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal , s_task_name, "I can NOT register");
    }

    
}

extern void iCubRunningMsg_manager_run(EOMtask *tsk, uint32_t evtmsgper)
{
    eOresult_t res;
    eOevent_t evt = (eOevent_t)evtmsgper;


    if(EVT_CHECK(evt, EVT_ICUBRUNMSG_MNG_STOP))//(EVT_ICUBRUNMSG_MNG_STOP == (evt & EVT_ICUBRUNMSG_MNG_STOP))
    {
       s_running_st = eobool_false;
       res = eo_socketdtg_Close(s_iCubRunMsg_Mng_dgramskt);
       /* Note: TODO  if I can't close the socket, is it batter i send me the same event..to tray another time??*/ 
       APPL_CHECKandPRINT_ERROR(res, eo_errortype_warning, s_task_name, "I can't close socket"); 

       res = eo_timer_Stop(s_iCubRunMsg_Mng_timer_SendDtgram);
       /* Note: TODO if I can't stop the timer, is it batter i send me the same event..to tray another time??*/  
       APPL_CHECKandPRINT_ERROR(res, eo_errortype_warning, s_task_name, "I can't stop timer"); 
    }


    if(EVT_CHECK(evt, EVT_ICUBRUNMSG_MNG_START))
    {
        s_running_st = eobool_true;
        /* Open the socket:
           - on port s_sysCntrl_port
           - in tx-rx 
           - in no-blocking mode 
           - tx-datagram will be sent immediately 
           - execute s_action upon rx of a datagram
           - the task will be not advise on trasmission of a datagram.
        */
        res = eo_socketdtg_Open(s_iCubRunMsg_Mng_dgramskt, RUNNING_SOCKET_PORT, eo_sktdir_TXRX, eobool_false, 
                                NULL, s_iCubRunMsg_Mng_action_onrx_dgram, NULL);
        APPL_CHECKandPRINT_ERROR(res, eo_errortype_warning, s_task_name, "I can't open socket");

        res = eo_timer_Start(s_iCubRunMsg_Mng_timer_SendDtgram, eok_abstimeNOW, s_sendingDgramperiod,
                             eo_tmrmode_FOREVER, s_iCubRunMsg_Mng_action_timeout);
        APPL_CHECKandPRINT_ERROR(res, eo_errortype_warning, s_task_name, "I can't start timer");

        /* Note: TODO is it better I send to myself the same event if i can't open sock or start timer???? */
    }


    if(EVT_CHECK(evt, EVT_ICUBRUNMSG_MNG_DGRAM_REC))
    {
        if(eobool_false == s_running_st)
        {
            APPL_ERRMAN_ERROR(eo_errortype_warning, s_task_name, "I'm in idle state, but rec evt_rec"); 
            return;
        }
        s_iCubRunningMsg_manager_recDgram_mng();
    }

    if(EVT_CHECK(evt, EVT_ICUBRUNMSG_MNG_TIMEOUT))
    {
        if(eobool_false == s_running_st)
        {
            APPL_ERRMAN_ERROR(eo_errortype_warning, s_task_name, "I'm in idle state, but rec evt_timeout");
            return;
        }
        if(eobool_false == s_conected_st)
        {
            return;    
        }
        s_iCubRunningMsg_manager_send_periodicDatagram();
    }



}

extern void iCubRunningMsg_manager_task(void *p)
{
    eom_task_Start(p);
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

/* this function manages received datagrams */
static void s_iCubRunningMsg_manager_recDgram_mng(void)
{
    eOresult_t res;
    uint8_t *data_ptr;
    uint16_t size;

    /*the third param is ignored, because the socket is not blocking*/
    res = eo_socketdtg_Get( s_iCubRunMsg_Mng_dgramskt, s_iCubRunMsg_Mng_rxpkt, eok_reltimeZERO );
    //NOTE: the only reason I get error is caused by socket's fifo is emmpty
    APPL_CHECKandPRINT_ERROR(res,eo_errortype_warning, s_task_name, "EVT: rec dgram, but socket empty");
    if(eores_OK != res)
    {
        return;
    }


    if(!s_conected_st)
    {
        eo_packet_Destination_Get(s_iCubRunMsg_Mng_rxpkt, &s_remaddr, &s_remport);

        res = eo_socketdtg_Connect(s_iCubRunMsg_Mng_dgramskt, s_remaddr, 500); //TODO: verifica il tempo
        APPL_CHECKandPRINT_ERROR(res, eo_errortype_info, s_task_name, "I can NOT connect");
        if(eores_OK == res)
        {
            s_conected_st = eobool_true;
            /* METTI QUI LA FUNZIONE CHE PREVEDE DI MANDARE UNA RISPOSTA AL PC104 SU CONNESSIONE */
            //s_iCubRunningMsg_manager_resp_onConnect(); TODO. e' necessaria??da vedere col proto...
        }
        return; 
    }


    //if i'm here i'm already connected.
    eo_packet_Payload_Get(s_iCubRunMsg_Mng_rxpkt, &data_ptr, &size);

    /*ATTENZIONE: ora chiamo un parser semplice, ma poi e' da sostituire con la chiamata al netvar-parser!!!!*/
    s_iCubRunMsg_Mng_parser_TEST(data_ptr, size);

}


/*questa funzione fa le veci del parser di alto livello, ovvero smista le info che devono rimanere sulla ems, 
quelle che devono essere inviate sul can 1 e quelle sul can 2*/
static void s_iCubRunMsg_Mng_parser_TEST(uint8_t *eth_payload, uint16_t size)  
{
    eOresult_t res;
    iCubRunMsg_Mng_canData_ptr_t CAN1data_ptr, CAN2data_ptr;

//1) prendo i dati per il can1 e 2 
    s_iCubRunMsg_Mng_parser_getCANdata_TEST(eth_payload, &CAN1data_ptr, &CAN2data_ptr);
    
//2) se ho qc per can 1 glielo do
    if((CAN1data_ptr.size > 0) && (CAN1data_ptr.size < ETHCAN1_SHDATA_ITEMSIZE))
    {
        res = EOMprodConsSharedData_Put(ethCan1_shData, (void*)CAN1data_ptr.data_ptr, 100); //TODO: stai attenta al valore di timeout!!!
        if(eores_OK != res)
        {
            ;//errore; boh!!!
        }
    }

// 3) se ho qc per can 2 glielo do
    if((CAN2data_ptr.size > 0) && (CAN2data_ptr.size < ETHCAN2_SHDATA_ITEMSIZE))
    {
        res = EOMprodConsSharedData_Put(ethCan2_shData, (void*)CAN2data_ptr.data_ptr, 100); //stai attenta al valore di timeout!!!
        if(eores_OK != res)
        {
            ;//errore; boh!!!
        }
    }

// 4)chiamo il parser rop per fargli fare le sue operazioni!!
    //    res = s_iCubRunMsg_Mng_parser_getRops(eth_payload, &ethData_ptr);
    //    if( (eores_OK != res) && (NULL != ethData_ptr) )
    //    {
    //        ;//chiama parser rops!!!!
    //    }
    
}


static void s_iCubRunMsg_Mng_parser_getCANdata_TEST(uint8_t *eth_payload, iCubRunMsg_Mng_canData_ptr_t *CAN1data_ptr, iCubRunMsg_Mng_canData_ptr_t *CAN2data_ptr)
{
    CAN1data_ptr->size = 16;
    CAN1data_ptr->data_ptr = eth_payload;
    
    CAN2data_ptr->size = 16;
    CAN2data_ptr->data_ptr = eth_payload;    
    
}


static void s_iCubRunningMsg_manager_send_periodicDatagram(void)
{
    uint16_t size;
    uint8_t *data_ptr;

//1) recupero i dati che devo spedire periodicamente
    s_iCubRunningMsg_manager_get_dataToSend_TEST(&data_ptr, &size);

//2) li metto nel pkt
    eo_packet_Full_Set(s_iCubRunMsg_Mng_txpkt, s_remaddr, s_remport, size, data_ptr);
    /* TODO: ha piu' senso settare remaddr e remport una volta per tutte e poi settare solo i dati.
            i remaddr e remport si possono settare su conect.!!!
    */

//3) spedisco il pkt
    eo_socketdtg_Put(s_iCubRunMsg_Mng_dgramskt, s_iCubRunMsg_Mng_txpkt);


//4) preparo il pkt per il prox invio
    eo_packet_Full_Clear(s_iCubRunMsg_Mng_txpkt, 0);
}


//Questa funzione simula il prendere i valori da spedire periodoicamente verso pc104
static void s_iCubRunningMsg_manager_get_dataToSend_TEST(uint8_t **data_ptr, uint16_t *size_ptr)
{
    static int8_t count = 0;

    memset(my_data_TEST, count, 30);
    count++; 
    *data_ptr = my_data_TEST;
    *size_ptr = 30;
}
// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



