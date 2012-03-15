
/* @file       eom-eupdater-main.c
	@brief      This file implements the e-updater using the embobj
	@author     marco.accame@iit.it, alessandro.scalzo@iit.it
    @date       01/11/2012
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdint.h"
#include "stdlib.h"

// abslayer 
#include "hal.h"
#include "hal_trace.h"
#include "osal.h"
#include "ipal.h"


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


#include "eupdater-info.h"

#include "EOtheARMenvironment.h"
#include "EOVtheEnvironment.h"


#include "eupdater_parser.h"

#include "EOtimer.h"





// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------
// empty-section
 
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


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------


static void s_udpnode_errman_OnError(eOerrmanErrorType_t errtype, eOid08_t taskid, const char *eobjstr, const char *info);

static void s_eom_eupdater_main_init(void);

static void s_udpserver_startup(EOMtask *p, uint32_t t);
static void s_udpserver_run(EOMtask *p, uint32_t t);


static eObool_t s_eom_eupdater_main_connected2host(EOpacket *rxpkt, EOsocketDatagram *skt);

static void s_toggle_led(void *p);



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static EOsocketDatagram*        s_skt_rops          = NULL;
static EOsocketDatagram*        s_skt_data          = NULL;
static EOpacket*                s_rxpkt             = NULL;
static EOpacket*                s_txpkt             = NULL;
static EOMtask*                 s_task_udpserver    = NULL;
static EOaction*                s_action            = NULL;



static const eOmessage_t s_message_from_skt_rops        = 0x00000001;
static const eOmessage_t s_message_from_skt_data        = 0x00000002;

static const eOipv4port_t s_server_port = 3333; 


static const eOerrman_cfg_t  errmancfg = 
{
    .extfn.usr_on_error = s_udpnode_errman_OnError
};


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


int main(void)
{
    
    eom_sys_Initialise( &eupdater_syscfg,
                        NULL,                           // mempool
                        &errmancfg,                     // errman
                        &eom_timerman_DefaultCfg,
                        &eom_callbackman_DefaultCfg
                      );  
    
    eom_sys_Start(eom_sys_GetHandle(), s_eom_eupdater_main_init);


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
    char str[96];

    sprintf(str, "[eobj: %s, tsk: %d] %s: %s", eobjstr, taskid, err[(uint8_t)errtype], info);
    hal_trace_puts(str);

    if(errtype <= eo_errortype_warning)
    {
        return;
    }

    for(;;);
}


static void s_eom_eupdater_main_init(void)
{
    uint8_t *ipaddr = (uint8_t*)&(eupdater_ipal_cfg->eth_ip);
    eOmipnet_cfg_addr_t* eomipnet_addr;
    const eEipnetwork_t *ipnet = NULL;
    char str[96];

    const eOmipnet_cfg_dtgskt_t eom_ipnet_dtgskt_MyCfg = 
    {   
        .numberofsockets            = 3, 
        .maxdatagramenqueuedintx    = 2
    };

#warning --> e se la gestione della eeprom richiedesse il disable irq in certi momenti?
    eo_armenv_Initialise(&eupdater_modinfo, NULL);
    eov_env_SharedData_Synchronise(eo_armenv_GetHandle());

    if(eores_OK == eov_env_IPnetwork_Get(eo_armenv_GetHandle(), &ipnet))
    {
        eomipnet_addr = (eOmipnet_cfg_addr_t*)ipnet;   //they have the same memory layout

        ipaddr = (uint8_t*)&(eomipnet_addr->ipaddr);
    }
//    else
    {
        eomipnet_addr = NULL;
        ipaddr = (uint8_t*)&(eupdater_ipal_cfg->eth_ip);
    }


    
    sprintf(str, "starting EOMeUpdater::ipnet with IP addr: %d.%d.%d.%d\n\r", ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]);
    hal_trace_puts(str);    

    // start the ipnet
    eom_ipnet_Initialise(&eom_ipnet_DefaultCfg,
                         eupdater_ipal_cfg, 
                         eomipnet_addr,
                         &eom_ipnet_dtgskt_MyCfg
                         );
                         

    sprintf(str, "starting EOMeUpdater::taskudpserver");
    hal_trace_puts(str);                         

    s_task_udpserver = eom_task_New(eom_mtask_MessageDriven, 100, 2*1024, s_udpserver_startup, s_udpserver_run,  6, 
                                    eok_reltimeINFINITE, NULL, 
                                    task_udpserver, "udpserver");

    s_task_udpserver = s_task_udpserver;


    hal_led_init(hal_led0, NULL);
    hal_led_init(hal_led1, NULL);



}


static void s_udpserver_startup(EOMtask *p, uint32_t t)
{
    char str[96];
    EOtimer* timer = NULL;

    // init the rx packet 
    s_rxpkt = eo_packet_New(1500);  
    s_txpkt = eo_packet_New(128);

    // init the action used for various tasks
    s_action = eo_action_New();  

    // initialise the socket 
    s_skt_rops = eo_socketdtg_New(  2, 1500, eom_mutex_New(), // input queue
                                    2, 128, eom_mutex_New()  // output queue
                                 );


    s_skt_data = eo_socketdtg_New(  1, 1500, eom_mutex_New(), // input queue
                                    1, 128, eom_mutex_New()  // output queue
                                 );
    

    sprintf(str, "opening a txrx socket on port %d w/ immediate tx\n\r", s_server_port);
    hal_trace_puts(str);


    // set the rx action on socket to be a message s_message_from_skt_rops to this task object
    eo_action_SetMessage(s_action, s_message_from_skt_rops, p);
    eo_socketdtg_Open(s_skt_rops, s_server_port, eo_sktdir_TXRX, eobool_false, NULL, s_action, NULL);

    // set the rx action on socket to be a message s_message_from_skt_data to this task object
    eo_action_SetMessage(s_action, s_message_from_skt_data, p);
    eo_socketdtg_Open(s_skt_data, s_server_port+1, eo_sktdir_TXRX, eobool_false, NULL, s_action, NULL);


    eupdater_parser_init();

    timer = eo_timer_New();
    eo_action_SetCallback(s_action, s_toggle_led, NULL, eom_callbackman_GetTask(eom_callbackman_GetHandle()));
    eo_timer_Start(timer, 0, 500*1000, eo_tmrmode_FOREVER, s_action);

        
}

static void s_udpserver_run(EOMtask *p, uint32_t t)
{
    // read the packet.
    eOresult_t res;
    EOsocketDatagram *socket = NULL;
    eObool_t (*parser)(EOpacket*, EOpacket *) = NULL;

    // the message that we have received
    eOmessage_t msg = (eOmessage_t)t;

    switch(msg)
    {
        case s_message_from_skt_rops:
        {
            socket = s_skt_rops;
            parser = eupdater_parser_process_rop;
        } break;

        case s_message_from_skt_data:
        {
            socket = s_skt_data;
            parser = eupdater_parser_process_data;
        } break;

        default:
        {
            socket = NULL;
            parser = NULL;
        } break;

    }

    if((NULL == socket) || (NULL == parser))
    {
        return;
    }


    hal_led_toggle(hal_led1);


    res = eo_socketdtg_Get(socket, s_rxpkt, eok_reltimeINFINITE);

    if((eores_OK == res) && (eobool_true == s_eom_eupdater_main_connected2host(s_rxpkt, socket)))
    {

        if(eobool_true == parser(s_rxpkt, s_txpkt))
        {
                // transmit a pkt back to the host
                eo_socketdtg_Put(socket, s_txpkt);
        }
 
    }


}


static void s_udpserver_run_safe(EOMtask *p, uint32_t t)
{
    // read the packet.
    eOresult_t res;
    //eOipv4addr_t remaddr;
    //eOipv4port_t remport;
    //uint8_t *ipaddr;

    // the message that we have received
    eOmessage_t msg = (eOmessage_t)t;


    if(s_message_from_skt_rops == msg)
    {   // ok, message from the socket

        res = eo_socketdtg_Get(s_skt_rops, 
                               s_rxpkt, 
                               eok_reltimeZERO //eok_reltimeINFINITE
                               );
    
        if((eores_OK == res) && (eobool_true == s_eom_eupdater_main_connected2host(s_rxpkt, s_skt_rops)))
        {

            if(eobool_true == eupdater_parser_process_rop(s_rxpkt, s_txpkt))
            {
                // transmit a pkt back to the host
                eo_socketdtg_Put(s_skt_rops, s_txpkt);
            }
 
        }

    }
    else if(s_message_from_skt_data == msg)
    {   // ok, message from the socket transfer



    }

}

static eObool_t s_eom_eupdater_main_connected2host(EOpacket *rxpkt, EOsocketDatagram *skt)
{
    static eObool_t     host_connected = eobool_false;
    static eOipv4addr_t host_ipaddress = 0;

    eOipv4addr_t remaddr = 0;
    eOipv4port_t remport = 0;

    // print stats of rx packet
    eo_packet_Destination_Get(rxpkt, &remaddr, &remport);
    
    if((eobool_false == host_connected) || (remaddr != host_ipaddress))
    {
        host_ipaddress = remaddr;
        host_connected = eobool_false;
        
        // attempt connection for 1 second. no more. 
        if(eores_OK == eo_socketdtg_Connect(skt, remaddr, eok_reltime1sec))
        {
            host_connected = eobool_true;
        }
        else
        {
            host_connected = eobool_false;
            //printf("not connecetd after %d ms. i shall try again at next reception\n\r", eok_reltime1sec/1000);
        }
    }

    return(host_connected);
}


static void s_toggle_led(void *p)
{
    hal_led_toggle(hal_led0);
}

// --- another file -------------------------------------------------------------------------------------------------


//typedef struct
//{
//    eOnanotime_t    client_tx_time;
//    eOnanotime_t    server_tx_time;
//    uint32_t        client_id;
//} pkt_payload_t;
//
//// return true if there is a pkt to transmit back
//extern eObool_t eom_eupdater_parse_rop(EOpacket *rxpkt, EOpacket *txpkt)
//{
//    pkt_payload_t *pktdata = NULL;
//    uint8_t *data;
//    uint16_t size;
//    eOnanotime_t nano = 0;
//
//    eOipv4addr_t remaddr = 0;
//    eOipv4port_t remport = 0;
//
//    eo_packet_Destination_Get(rxpkt, &remaddr, &remport);
//
//
//    eo_packet_Payload_Get(s_rxpkt, &data, &size);
//    //ipaddr = ((uint8_t*)&remaddr);
//    pktdata = (pkt_payload_t*)data;
//    //printf("received %d bytes from %d.%d.%d.%d-%d\n\r", size, ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3], remport);
//    //printf("w/ id: %x\n\r", pktdata->client_id);
//
//    // prepare tx packet
//    pktdata->client_id = 0x10101010;
//    eov_sys_NanoTimeGet(eov_sys_GetHandle(), &nano); 
//
//    pktdata->client_tx_time = nano;
//
//    eo_packet_Payload_Set(txpkt, (uint8_t*)pktdata, sizeof(pkt_payload_t));
//
//    eo_packet_Destination_Set(s_txpkt, remaddr, s_server_port);
//
//    return(eobool_true);
//}




// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



