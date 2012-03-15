
/* @file       eom_applprotoc_specialise.c
	@brief      This file keeps the vcececew
	@author     marco.accame@iit.it
    @date       01/11/2012
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "eEcommon.h"
#include "eEmemorymap.h"
#include "EOVtheSystem.h"
#include "EOMtask.h"
#include "EOpacket.h"
#include "EOsocketDatagram.h"
#include "EOMmutex.h"
#include "EOaction_hid.h"
#include "EOtheErrorManager.h"

#include "osal.h"
#include "hal.h"
#include "ipal.h"

#include "stdlib.h"


#include "eom_applprotoc_extra.h"

extern const ipal_cfg_t    s_ipal_cfg;



// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eom_applprotoc_specialise.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

extern void task_udpserver(void *p);



// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_OnError(eOerrmanErrorType_t errtype, eOid08_t taskid, const char *eobjstr, const char *info);

static void s_udpserver_startup(EOMtask *p, uint32_t t);

static void s_udpserver_run(EOMtask *p, uint32_t t);



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


static osal_semaphore_t*        s_sem_tx_pkt        = NULL;

static EOsocketDatagram*        s_skt_rops          = NULL;
static EOpacket*                s_rxpkt             = NULL;
static EOpacket*                s_txpkt             = NULL;
static EOMtask*                 s_task_udpserver    = NULL;


static void (*s_eom_applprotoc_specialise_on_ropframe_received)(EOpacket*) = NULL;



static const eOevent_t s_event_tx_into_skt_rops         = 0x00000001;
static const eOevent_t s_event_connect_to_host          = 0x00000002;
static const eOevent_t s_event_from_skt_rops            = 0x00000004;

static const eOipv4port_t s_server_port                 = 33333; 

static volatile eObool_t s_host_connected               = eobool_false;

static eOipv4addr_t s_host_ipaddress                    = 0;


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------

extern const ipal_cfg_t* eom_applprotoc_specialise_ipal_cfg = &s_ipal_cfg;


extern const eOerrman_cfg_t  eom_applprotoc_specialise_errcfg = 
{
    .extfn.usr_on_error = s_OnError
};

extern const eOmipnet_cfg_dtgskt_t eom_applprotoc_specialise_dtgskt_cfg = 
{   
    .numberofsockets            = 2, 
    .maxdatagramenqueuedintx    = 2
};

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern void eom_applprotoc_specialise_updserver_start(void)
{
    s_sem_tx_pkt =  osal_semaphore_new(255, 0);

    s_host_connected = eobool_false;

    eo_errman_Assert(eo_errman_GetHandle(), (NULL != s_sem_tx_pkt), "applprotoc_specialise", "semaphore is NULL");

    s_task_udpserver = eom_task_New(eom_mtask_EventDriven, 100, 2*1024, s_udpserver_startup, s_udpserver_run,  6, 
                                    eok_reltimeINFINITE, NULL, 
                                    task_udpserver, "udpserver");

    s_task_udpserver = s_task_udpserver;

}  

extern void eom_applprotoc_specialise_onpktreceived_set( void (*cbk)(EOpacket*)  )
{
    s_eom_applprotoc_specialise_on_ropframe_received = cbk;
}

extern void eom_applprotoc_specialise_otherthings(void)
{

    eom_applprotoc_extra_transceiver_init();

}


extern void eom_applprotoc_specialise_transmit(EOpacket *txpkt)
{

   // copy teh pkt pointer
    s_txpkt = txpkt;

    eo_packet_Addressing_Set(s_txpkt, s_host_ipaddress, s_server_port);

    eo_socketdtg_Put(s_skt_rops, s_txpkt);

//    // alert the task upd server
//    eom_task_SetEvent(s_task_udpserver, s_event_tx_into_skt_rops);
//
//    // decrement the semaphore to wait for end of operation
//    osal_semaphore_decrement(s_sem_tx_pkt, OSAL_reltimeINFINITE);
   
}


extern eObool_t eom_applprotoc_specialise_connect(eOipv4addr_t remotehostaddr)
{

//    if((eobool_false == s_host_connected) || (remotehostaddr != s_host_ipaddress))
//    {
//        s_host_ipaddress = remotehostaddr;
//        s_host_connected = eobool_false;
//
//        // alert the task upd server
//        eom_task_SetEvent(s_task_udpserver, s_event_connect_to_host);
//
//        // decrement the semaphore to wait for end of operation
//        osal_semaphore_decrement(s_sem_tx_pkt, OSAL_reltimeINFINITE); 
//    }

    if((eobool_false == s_host_connected) || (remotehostaddr != s_host_ipaddress))
    {
        // attempt connection for 1 second. no more. 
        if(eores_OK == eo_socketdtg_Connect(s_skt_rops, remotehostaddr, eok_reltime1sec))
        {
            s_host_ipaddress = remotehostaddr;
            s_host_connected = eobool_true;
        }
        else
        {
            s_host_connected = eobool_false;
            //printf("not connecetd after %d ms. i shall try again at next reception\n\r", eok_reltime1sec/1000);
        }
    }

    return(s_host_connected);
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


extern void task_udpserver(void *p)
{
    // do here whatever you like before startup() is executed and then forever()
    eom_task_Start(p);
} 


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void s_OnError(eOerrmanErrorType_t errtype, eOid08_t taskid, const char *eobjstr, const char *info)
{
    const char err[4][16] = {"info", "warning", "weak error", "fatal error"};
    char str[128];

    snprintf(str, sizeof(str)-1, "[eobj: %s, tsk: %d] %s: %s", eobjstr, taskid, err[(uint8_t)errtype], info);
    hal_trace_puts(str);

    if(errtype <= eo_errortype_warning)
    {
        return;
    }

    for(;;);
}




static void s_udpserver_startup(EOMtask *p, uint32_t t)
{
    EOaction s_fullaction;

    // init the rx packet 
    s_rxpkt = eo_packet_New(512);  
 

    // initialise the socket 
#if 0
    s_skt_rops = eo_socketdtg_New(  2, 512, eom_mutex_New(), // input queue
                                    2, 512, eom_mutex_New()  // output queue
                                 );
#else
    s_skt_rops = eo_socketdtg_New(  2, 512, NULL, // input queue
                                    2, 512, NULL  // output queue
                                 );
#endif

    // set the rx action on socket to be a message s_message_from_skt_rops to this task object
    eo_action_SetEvent(&s_fullaction, s_event_from_skt_rops, p);
    eo_socketdtg_Open(s_skt_rops, s_server_port, eo_sktdir_TXRX, eobool_false, NULL, &s_fullaction, NULL);
        
}

static void s_udpserver_run(EOMtask *p, uint32_t t)
{
    // read the packet.
    eOresult_t res;
    uint16_t numberof = 0;

    // the event that we have received
    eOevent_t evt = (eOevent_t)t;

    if(s_event_from_skt_rops == (evt & s_event_from_skt_rops))
    {
        // retrieve the pkt.
        res = eo_socketdtg_Get(s_skt_rops, s_rxpkt, eok_reltimeINFINITE);

        // call a function for it
        if((eores_OK == res) && (NULL != s_eom_applprotoc_specialise_on_ropframe_received))
        {
            s_eom_applprotoc_specialise_on_ropframe_received(s_rxpkt);
        }

        // if any other pkt in socket then send event again
        res = eo_socketdtg_Received_NumberOf(s_skt_rops, &numberof);
        if((0 != numberof) && (eores_OK == res))
        {
            eom_task_SetEvent(p, s_event_from_skt_rops);
        }
    }
    
//    if(s_event_tx_into_skt_rops == (evt & s_event_tx_into_skt_rops))
//    {
//        // transmit the pkt.
//        res = eo_socketdtg_Put(s_skt_rops, s_txpkt);
//
//        // increment the semaphore so taht teh caller can return
//        osal_semaphore_increment(s_sem_tx_pkt);
//    }
//
//    if(s_event_connect_to_host == (evt & s_event_connect_to_host))
//    {
//
//        // attempt connection for 1 second. no more. 
//        if(eores_OK == eo_socketdtg_Connect(s_skt_rops, s_host_ipaddress, eok_reltime1sec))
//        {
//            s_host_connected = eobool_true;
//        }
//        else
//        {
//            s_host_connected = eobool_false;
//            //printf("not connecetd after %d ms. i shall try again at next reception\n\r", eok_reltime1sec/1000);
//        }
// 
//        // increment the semaphore so taht teh caller can return
//        osal_semaphore_increment(s_sem_tx_pkt);
//    }

}








// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



