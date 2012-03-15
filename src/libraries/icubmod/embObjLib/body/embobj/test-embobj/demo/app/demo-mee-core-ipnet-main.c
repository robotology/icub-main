
/* @file       demo-mee-core-ipnet-main.c
	@brief      This file implements a simple udp node which acts as a simply reply
	@author     marco.accame@iit.it
    @date       06/21/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdint.h"
#include "stdlib.h"

// abslayer 
#include "hal.h"
#include "osal.h"
#include "ipal.h"
#include "fsal.h"

// embobj  
#include "EoCommon.h"
#include "EOMtheSystem.h"
#include "EOVtheSystem.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrormanager.h"
#include "EOMtheIPnet.h"

#include "EOaction.h"
#include "EOpacket.h"
#include "EOMmutex.h"
#include "EOsocketDatagram.h"







// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------
extern const hal_cfg_t *hal_cfgMINE;
extern const fsal_params_cfg_t *fsal_params_cfgMINE;
extern const osal_cfg_t *osal_cfgMINE;
 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------


extern void task_udpserver(void *p);

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------

typedef struct
{
    uint32_t        client_id;
    eOnanotime_t    client_tx_time;
    eOnanotime_t    server_tx_time;
} pkt_payload_t;

// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------


static void s_udpnode_errman_OnError(eOerrmanErrorType_t errtype, eOid08_t taskid, const char *eobjstr, const char *info);

static void s_udpnode_init(void);

static void s_udpserver_startup(EOMtask *p, uint32_t t);
static void s_udpserver_run(EOMtask *p, uint32_t t);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static EOsocketDatagram*        s_dgramskt          = NULL;
static EOsocketDatagram*        s_synskt           = NULL;
static EOpacket*                s_rxpkt             = NULL;
static EOpacket*                s_txpkt             = NULL;
static EOMtask*                 s_task_udpserver    = NULL;
static EOaction*                s_action            = NULL;


static const eOmessage_t s_message_from_skt_dtg = 0x00000001;
static const eOmessage_t s_message_from_skt_syn = 0x00000002;

static const eOipv4port_t s_server_port = 3333; 


static const eOerrman_cfg_t  errmancfg = 
{
    .extfn.usr_on_error = s_udpnode_errman_OnError
};


//static eOsktdtgTXmode_t txonce =
//{
//    .startat = EOK_abstimeNOW,
//    .after = 3*1000*1000,
//    .periodic    = eobool_false
//};


static eOsktdtgTXmode_t txperiodic =
{
    .startat = 0,
    .after = 5*1000*1000,
    .periodic    = eobool_true
};

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


int main(void)
{

    eOmsystem_cfg_t syscfg =
    {
        .halcfg     = hal_cfgMINE,
        .osalcfg    = osal_cfgMINE,
        .fsalcfg    = fsal_params_cfgMINE
    };
    
    eom_sys_Initialise( &syscfg,
                        NULL,                           // mempool
                        &errmancfg,                     // errman
                        &eom_timerman_DefaultCfg,
                        &eom_callbackman_DefaultCfg
                      );  
    
    eom_sys_Start(eom_sys_GetHandle(), s_udpnode_init);


}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


void osal_on_idle(void)
{
    static uint32_t cnt = 0;

    for(;;)
    {
        cnt++;
    }
}




extern void task_udpserver(void *p)
{
    // do here whatever you like before startup() is executed and then forever()
    eom_task_Start(p);
}  


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static void s_udpnode_errman_OnError(eOerrmanErrorType_t errtype, eOid08_t taskid, const char *eobjstr, const char *info)
{
    const char err[4][16] = {"info", "warning", "weak error", "fatal error"};
    char str[128];

    sprintf(str, "[eobj: %s, tsk: %d] %s: %s\n\r", eobjstr, taskid, err[(uint8_t)errtype], info);
    hal_trace_puts(str);

    if(errtype <= eo_errortype_warning)
    {
        return;
    }

    for(;;);
}

static void s_udpnode_init(void)
{
    extern const ipal_cfg_t *ipal_cfgMINE;
    const uint8_t *ipaddr = (const uint8_t*)&(ipal_cfgMINE->eth_ip);
    char str[128];
    
    // init the action used for various tasks
    s_action = eo_action_New();    

    // start the ipnet
    eom_ipnet_Initialise(&eom_ipnet_DefaultCfg,
                         (ipal_cfg_t*)ipal_cfgMINE, 
                         NULL,
                         // &s_differentaddrcfg, 
                         &eom_ipnet_dtgskt_DefaultCfg
                         );

    sprintf(str, "STARTING a demo for ipnet. IP addr is: %d.%d.%d.%d. Use program winNode.exe to communicate\n\r",
           ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]);
    
    hal_trace_puts(str);

    s_task_udpserver = eom_task_New(eom_mtask_MessageDriven, 69, 3*1024, s_udpserver_startup, s_udpserver_run,  6, 
                                    eok_reltimeINFINITE, NULL,
                                    task_udpserver, "udpserver");

    s_task_udpserver = s_task_udpserver;

}


static void s_udpserver_startup(EOMtask *p, uint32_t t)
{
    char str[128];

    // init the rx packet 
    s_rxpkt = eo_packet_New(64);  
    s_txpkt = eo_packet_New(64);

    // initialise the socket 
    s_dgramskt = eo_socketdtg_New(  4, 64, eom_mutex_New(), // input queue
                                    4, 64, eom_mutex_New()  // output queue
                                 );

    // initialise the syn socket 
    s_synskt = eo_socketdtg_New(  2, 64, eom_mutex_New(), // input queue
                                    2, 64, eom_mutex_New()  // output queue
                                );    
    
    // set the rx action on socket to be a message s_message_from_skt_dtg to this task object
    eo_action_SetMessage(s_action, s_message_from_skt_dtg, p);

#if _DELAYED_TX_
    sprintf(str, "opening a txrx socket on port %d w/ delayed tx of %d milli\n\r", s_server_port, txonce.after/1000);
    hal_trace_puts(str);

    sprintf(str, "the socket shall send back any packet it receives\n\r");
    hal_trace_puts(str);
    // open the socket on port s_server_port to be tx-rx and and execute s_action upon rx of a datagram
    eo_socketdtg_Open(s_dgramskt, s_server_port, eo_sktdir_TXRX, eobool_false, &txonce, s_action, NULL);//NULL or &txonce
#else
    sprintf(str, "opening a txrx socket on port %d w/ immediate tx\n\r", s_server_port);
    hal_trace_puts(str);

    sprintf(str, "the socket shall send back any packet it receives\n\r");
    hal_trace_puts(str);
    // open the socket on port s_server_port to be tx-rx and and execute s_action upon rx of a datagram
    eo_socketdtg_Open(s_dgramskt, s_server_port, eo_sktdir_TXRX, eobool_false, NULL, s_action, NULL);//NULL or &txonce
#endif
    
    
    // set the rx action on socket to be a message s_message_from_skt_syn to this task object
    eo_action_SetMessage(s_action, s_message_from_skt_syn, p);     

    sprintf(str, "opening a txrx socket on port %d w/ slotted tx of period = %d milli\n\r", s_server_port+1, txperiodic.after/1000);
    hal_trace_puts(str);
    sprintf(str, "the socket shall send back any packet it receives\n\r");
    hal_trace_puts(str);
    // open the socket on port s_server_port to be tx-rx and and execute s_action upon rx of a datagram
    eo_socketdtg_Open(s_synskt, s_server_port+1, eo_sktdir_TXRX, eobool_false, &txperiodic, s_action, NULL);
    
}

static void s_udpserver_run(EOMtask *p, uint32_t t)
{
    // read the packet.
    static eOresult_t res;
    eOipv4addr_t remaddr;
    eOipv4port_t remport;
    uint8_t *ipaddr;
    uint8_t *data;
    uint16_t size;
    pkt_payload_t *pktdata = NULL;
    static eObool_t connected = eobool_false;

    // the message that we have received
    eOmessage_t msg = (eOmessage_t)t;


    if(s_message_from_skt_dtg == msg)
    {   // ok, message from the socket

        res = eo_socketdtg_Get(s_dgramskt, 
                               s_rxpkt, 
                               eok_reltimeZERO //eok_reltimeINFINITE
                               );
    
        if(eores_OK == res)
        {
            // print stats of rx packet
            eo_packet_Destination_Get(s_rxpkt, &remaddr, &remport);
            eo_packet_Payload_Get(s_rxpkt, &data, &size);
            ipaddr = ((uint8_t*)&remaddr);
            ipaddr = ipaddr;
            pktdata = (pkt_payload_t*)data;
            //printf("received %d bytes from %d.%d.%d.%d-%d\n\r", size, ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3], remport);
            //printf("w/ id: %x\n\r", pktdata->client_id);
            
            if(eobool_false == connected)
            {
                //printf("not yet connected \n\r");
                res = eo_socketdtg_Connect(s_dgramskt, remaddr, eok_reltime1sec);

                if(eores_OK == res)
                {
                    connected = eobool_true;
                    //printf("connecetd now\n\r");
                }
                else
                {
                    //printf("not connecetd after %d ms. i shall try again at next reception\n\r", eok_reltime1sec/1000);
                }
            }

            if(eobool_true == connected)
            {
                // prepare tx packet
                eov_sys_NanoTimeGet(eov_sys_GetHandle(), &pktdata->server_tx_time); 
    
                //eo_packet_Full_Set(s_txpkt, remaddr, remport, size, data);
                eo_packet_Full_Set(s_txpkt, remaddr, s_server_port, size, data);
                
                // and put it back into the socket
                eo_socketdtg_Put(s_dgramskt, s_txpkt);
                
                //printf("transmitted a pkt back\n\r");
            }
        }

    }
    else if(s_message_from_skt_syn == msg)
    {   // ok, message from the socketsyn

        res = eo_socketdtg_Get(s_synskt, 
                               s_rxpkt, 
                               eok_reltimeZERO //eok_reltimeINFINITE
                               );
    
        if(eores_OK == res)
        {
            // print stats of rx packet
            eo_packet_Destination_Get(s_rxpkt, &remaddr, &remport);
            eo_packet_Payload_Get(s_rxpkt, &data, &size);
            ipaddr = ((uint8_t*)&remaddr);
            pktdata = (pkt_payload_t*)data;
            //printf("received %d bytes from %d.%d.%d.%d-%d on syn-socket\n\r", size, ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3], remport);
            //printf("w/ id: %x\n\r", pktdata->client_id);
            
            if(eobool_false == connected)
            {
                //printf("not yet connected to syn\n\r");
                res = eo_socketdtg_Connect(s_synskt, remaddr, eok_reltime1sec);

                if(eores_OK == res)
                {
                    connected = eobool_true;
                    //printf("connecetd now to syn\n\r");
                }
                else
                {                         
                   // printf("not connecetd to syn after %d ms. i shall try again at next reception\n\r", eok_reltime1sec/1000);
                }
            }

            if(eobool_true == connected)
            {
                // prepare tx packet
                eov_sys_NanoTimeGet(eov_sys_GetHandle(), &pktdata->server_tx_time); 
    
                //eo_packet_Full_Set(s_txpkt, remaddr, remport, size, data);
                eo_packet_Full_Set(s_txpkt, remaddr, s_server_port+1, size, data);
                
                // and put it back into the socket
                res = eo_socketdtg_Put(s_synskt, s_txpkt);
                
                if(eores_OK == res)
                {
                    //printf("transmitted a synsocket pkt back\n\r");
                }
                else
                {
                    //printf("failed transmitting a synsocket pkt back\n\r");
                }
            }
        }

    }

}







// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



