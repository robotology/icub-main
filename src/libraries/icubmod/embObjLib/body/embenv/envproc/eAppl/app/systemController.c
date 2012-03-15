// --------------------------------------------------------------------------------------------------------------------
// - doxy
// --------------------------------------------------------------------------------------------------------------------
/* @file       systemController.c
	@brief      This file implements system controller task
	@author     valentina.gaggero@iit.it
    @date       12/05/2011
**/



// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "shalPART.h" 

//embObj
#include "eEcommon.h"
#include "EOMtheIPnet.h"
#include "EOMtheEntitiesEnv.h"
#include "EOSocketDatagram.h"
#include "EOtheErrorManager.h"
#include "EOMmutex.h"
#include "EOMprodConsSharedData.h"
#include "EOMdataContainer.h"


// application
#include "appl_common.h"
#include "can1mng.h"
#include "can2mng.h"
#include "iCubRunningMsg_manager.h"
#include "encodersReader.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------
#include "systemController.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
//#include "entity_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

//************************ system controller task events **************************************************
#define         EVT_SYSCNTRL_DGRAM_REC              (1 << 0) //da rivedere
#define         EVT_SYSCNTRL_START_SYS              (1 << 1) //da rivedere
#define         EVT_SYSCNTRL_STOP_SYS               (1 << 2) //da rivedere
#define         EVT_SYSCNTRL_ALLENTITIES_REGISTERD  (1 << 3) //da rivedere


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables. deprecated: better using _get(), _set() on static variables 
// --------------------------------------------------------------------------------------------------------------------
extern EOMtask                 *sysCntr_task; //pointer to myself (my task)
EOMprodConsSharedData          *ethCan1_shData; //pointer to shared data between iCubRunningMsg_manager and can1 manager
EOMprodConsSharedData          *ethCan2_shData; //pointer to shared data between iCubRunningMsg_manager and can2 manager
EOMdataContainer               *commonData; 

// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
static void s_sysCntrl_entitiesEnv_allRegistred(void);
static void s_sysController_recDgram_mng(void); //this function manages the received datagram

//Le seguenti funzioni sono usate solo per test, finche' non ho l'applicaz completa.
static void s_sysCntrl_parser_TEST(uint8_t *data, uint16_t size);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
static const char s_task_name[] = "sysController";

static EOsocketDatagram         *s_sysCntrl_dgramskt = NULL;
static EOaction                 *s_sysCntrl_action_onrx_dgram = NULL;
static EOpacket                 *s_sysCntrl_rxpkt = NULL;

static eOipv4addr_t             remaddr;    //Remote address: initilized on connect
static eOipv4port_t             remport;    //Remote port: initilized on connect
static EOMtask                  *s_CAN1_task = NULL; //TODO: serve???
static EOMtask                  *s_CAN2_task = NULL; //TODO: serve???
static EOMtask                  *s_iCubRunMsgMng_task = NULL; //TODO: serve??? 
static EOMtask                  *s_encReader_task = NULL; //TODO: serve??? 

/*--- state variables ---*/
/* NOTA: da sostituire con state_machine???*/
static eObool_t s_conected_st               = eobool_false;  //indicates if this task is connected with PC104 by socket.
static eObool_t s_allEntitiesAvailable_st   = eobool_false;  //indicates that all entities are availble, 
                                                             //that is are ready to reaceive start command
static eObool_t s_startSys_req              = eobool_false;  //if true indicates that pc104 has requested to start the system


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------
extern void sysController_startup(EOMtask *p, uint32_t t)
{
    eOresult_t res;
    EOMtheEntitiesEnv_cfg_t entitiesEnv_cfg;

/* NOTA: puo' essere che l'applicazione che va su una scheda ha dei manager in piu'/in meno di un'altra? 
 Se si allora bisogna prevedere una configurazione!!!! */

#warning VALE: entitiesEnv_cfg modificata solo per test!!!

//    entitiesEnv_cfg.task_num = 4;
//    entitiesEnv_cfg.eotimer_num = 1;
//    entitiesEnv_cfg.haltimer_num = 0;
//ATTENZIONE MODIFICATI SOLO PER TEST!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    entitiesEnv_cfg.task_num = 4;
    entitiesEnv_cfg.eotimer_num = 0;
    entitiesEnv_cfg.haltimer_num = 0;

    entitiesEnv_cfg.signals.task.startMsg = MSG_TASK_START;
    entitiesEnv_cfg.signals.task.stopMsg = MSG_TASK_STOP;
    entitiesEnv_cfg.signals.task.startEvt = EVT_TASK_START;
    entitiesEnv_cfg.signals.task.stopEvt = EVT_TASK_STOP;
    entitiesEnv_cfg.callback_allEntities_registered = s_sysCntrl_entitiesEnv_allRegistred; //request to notify me when all entities are registered.
    entitiesEnv_cfg.callback_etity_regitered = NULL;

    EOMtheEntitiesEnv_Initialise(&entitiesEnv_cfg, p);
 

    //prepare RX packet
    s_sysCntrl_rxpkt = eo_packet_New(ETH_CONTROL_DGRAM_PAYLOAD_SIZE);


    //prepare action that will be execute on datagram received
    s_sysCntrl_action_onrx_dgram = eo_action_New();
    eo_action_SetEvent(s_sysCntrl_action_onrx_dgram, (eOevent_t)EVT_SYSCNTRL_DGRAM_REC , p);

    //create the socket where i will receive system-control messages
    s_sysCntrl_dgramskt = eo_socketdtg_New(4, ETH_CONTROL_DGRAM_PAYLOAD_SIZE, eom_mutex_New(), // input queue: 4 datagram with mwax size = ETH_CONTROL_DGRAM_PAYLOAD_SIZE
                                           4, ETH_CONTROL_DGRAM_PAYLOAD_SIZE, eom_mutex_New()  // input queue: 4 datagram with mwax size = ETH_CONTROL_DGRAM_PAYLOAD_SIZE
                                           );

    /* Open the socket:
       - on port s_sysCntrl_port
       - in tx-rx 
       - in no-blocking mode 
       - tx-datagram will be sent immediately 
       - execute s_action upon rx of a datagram
       - the task will be not advise on trasmission of a datagram.
     */
    eo_socketdtg_Open(s_sysCntrl_dgramskt, (eOipv4port_t)SYSCONTROL_SOCKET_PORT, eo_sktdir_TXRX, eobool_false, 
                      NULL, s_sysCntrl_action_onrx_dgram, NULL);


    //Create objects for inter-task communication between iCubRunningMsg_manager and can1 manager 
    // and beetwen iCubRunningMsg_manager and can2 manager.
    ethCan1_shData = EOMprodConsSharedData_New((eOsizeitem_t) ETHCAN1_SHDATA_ITEMSIZE, 
                                               (eOsizecntnr_t) ETHCAN1_SHDATA_CAPACITY);
    ethCan2_shData = EOMprodConsSharedData_New((eOsizeitem_t) ETHCAN2_SHDATA_ITEMSIZE, 
                                               (eOsizecntnr_t) ETHCAN2_SHDATA_CAPACITY);

    commonData = eom_dataContainer_New(24);

    //qui creo tutti i task del sistema
    s_CAN1_task = eom_task_New(eom_mtask_EventDriven,
                                  69,//Priorita' del task: da rivedere ora ho messo rpio a caso.
                                  2*1024, //stacksize: da rivedere
                                  can1mng_startup, 
                                  can1mng_run,  
                                  0, //message queue size. the task is evt based 
                                  eok_reltimeINFINITE, 
                                  can1mng_task, 
                                  "can1mng");

    s_CAN2_task = eom_task_New(eom_mtask_EventDriven,
                                  69,//Priorita' del task: da rivedere ora ho messo rpio a caso.
                                  3*1024, //stacksize: da rivedere
                                  can2mng_startup, 
                                  can2mng_run,  
                                  0, //message queue size. the task is evt based 
                                  eok_reltimeINFINITE, 
                                  can2mng_task, 
                                  "can2mng");

    s_iCubRunMsgMng_task = eom_task_New(eom_mtask_EventDriven,
                                  80,//Priorita' del task: da rivedere ora ho messo rpio a caso.
                                  2*1024, //stacksize: da rivedere
                                  iCubRunningMsg_manager_startup, 
                                  iCubRunningMsg_manager_run,  
                                  0, //message queue size. the task is evt based 
                                  eok_reltimeINFINITE, 
                                  iCubRunningMsg_manager_task, 
                                  "iCubRunningMsg_manager");

    s_encReader_task = eom_task_New(eom_mtask_EventDriven,
                                  80,//Priorita' del task: da rivedere ora ho messo rpio a caso.
                                  3*1024, //stacksize: da rivedere
                                  encodersReader_startup, 
                                  encodersReader_run,  
                                  0, //message queue size. the task is evt based 
                                  eok_reltimeINFINITE, 
                                  encodersReader_task, 
                                  "iCubRunningMsg_manager");

}

extern void sysController_task(void *p)
{
    // do here whatever you like before startup() is executed and then forever()
    eom_task_Start(p);
} 



extern void sysController_run(EOMtask *tsk, uint32_t evtmsgper)
{
    eOevent_t evt;

    evt = (eOevent_t)evtmsgper;


    if(EVT_CHECK(evt, EVT_SYSCNTRL_DGRAM_REC))
    {
        s_sysController_recDgram_mng();
    }

    if(EVT_CHECK(evt, EVT_SYSCNTRL_START_SYS))
    {
        s_startSys_req = eobool_true;
        if(eobool_true == s_allEntitiesAvailable_st)
        {
            EOMtheEntitiesEnv_start_all(EOMtheEntitiesEnv_GetHandle());
        }
    }

    if(EVT_CHECK(evt, EVT_SYSCNTRL_STOP_SYS))
    {
        EOMtheEntitiesEnv_stop_all(EOMtheEntitiesEnv_GetHandle());
    }

    if(EVT_CHECK(evt, EVT_SYSCNTRL_ALLENTITIES_REGISTERD))
    {
        s_allEntitiesAvailable_st = eobool_true;
        if(eobool_true == s_startSys_req)
        {
            EOMtheEntitiesEnv_start_all(EOMtheEntitiesEnv_GetHandle());
        }
    }

}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
static void s_sysController_recDgram_mng(void)
{
    eOresult_t res;
    uint8_t *data_ptr;
    uint16_t size;

    /*the third param is ignored, because the socket is not blocking*/
    res = eo_socketdtg_Get( s_sysCntrl_dgramskt, s_sysCntrl_rxpkt, eok_reltimeZERO );
    //NOTE: the only reason I get error is caused by socket's fifo is emmpty
    APPL_CHECKandPRINT_ERROR(res,eo_errortype_warning, s_task_name, "EVT: rec dgram, but socket empty");
    if(eores_OK != res)
    {
        return;
    }

    if(!s_conected_st)
    {
        eo_packet_Destination_Get(s_sysCntrl_rxpkt, &remaddr, &remport);

        res = eo_socketdtg_Connect(s_sysCntrl_dgramskt, remaddr, 500); //TODO: verifica il tempo
        APPL_CHECKandPRINT_ERROR(res, eo_errortype_info, s_task_name, "I can NOT connect");
        if(eores_OK == res)
        {
            s_conected_st = eobool_true;
            /* METTI QUI LA FUNZIONE CHE PREVEDE DI MANDARE UNA RISPOSTA AL PC104 SU CONNESSIONE */
            //s_sysCntrl_resp_onConnect(); 
        }

        return; 
    }


    //if i'm here i'm already connected.
    eo_packet_Payload_Get(s_sysCntrl_rxpkt, &data_ptr, &size);

    /*ATTENZIONE: ora chiamo un parser semplice, ma poi e' da sostituire con la chiamata al netvar-parser!!!!*/
    s_sysCntrl_parser_TEST(data_ptr, size);


}

/*ATTENZIONE: questa funzione e' da sostituire col parser vero e proprio*/
static void s_sysCntrl_parser_TEST(uint8_t *data, uint16_t size)
{
    if(1 == data[0]) //Corrisponde a PKT QUERY in WINNODE utils appl
    {
        eom_task_SetEvent(sysCntr_task, EVT_SYSCNTRL_START_SYS);
    }
    else if(2 == data[0]) //Corrisponde a PKT REPLY in WINNODE utils appl
    {
        eom_task_SetEvent(sysCntr_task, EVT_SYSCNTRL_STOP_SYS);
    }
    

}


static void s_sysCntrl_entitiesEnv_allRegistred(void)
{
    eom_task_SetEvent(sysCntr_task, EVT_SYSCNTRL_ALLENTITIES_REGISTERD);    
}
// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



