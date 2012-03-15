                
/* @file       udpnode-main.c
	@brief      This file implements a simple udp node which acts as a simply reply
	@author     marco.accame@iit.it
    @date       06/21/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdint.h"
#include "string.h"
#include "stdlib.h"

// abslayer 
#include "hal.h"
#include "osal.h"
#include "ipal.h"
#include "fsal.h"

// embobj  
#include "EoCommon.h"
#include "EOMtheSystem.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrormanager.h"
#include "EOMtheIPnet.h"

#include "EOVtheSystem.h"

#include "EOaction.h"
#include "EOpacket.h"
#include "EOpacket_hid.h"
#include "EOMmutex.h"
#include "EOdatagramSocket.h"
#include "EOsynchroSocket.h"
#include "EOtimer.h"



#include "application_protocol.h"




// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------
extern const hal_params_cfg_t *hal_params_cfgMINE;
extern const fsal_params_cfg_t *fsal_params_cfgMINE;
extern const osal_params_cfg_t *osal_params_cfgMINE;
 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------


extern void task_node(void *p);

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
#define ST_NOT_CONNECTED    0
#define ST_READY            1
#define ST_LISTEN           2
#define ST_REPLY            3


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_udpnode_init(void);

static void s_node_startup(EOMtask *p, uint32_t t);
static void s_node_run(EOMtask *p, uint32_t t);

static void s_evt_pkt_received(void);
static void s_evt_timer_expired(void);
static void s_command_pkt_manage(packet_t *cmd_pkt, uint8_t *status);
static void s_collectedInfo_reset(void);

// send packet functions
static eObool_t s_connect_and_sendACK(void);
static void s_send_dataPkt(packet_t *received_pkt, eOnanotime_t *receive_time);
static void s_send_statPkt(void);
static void s_send_ackPkt(void);
static eOresult_t s_send_pkt(packet_t *pkt_to_send);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static EOdatagramSocket*        s_rx_dgramSocket    = NULL;
#ifdef SOCK_SYNCHRO
static EOsynchroSocket*         s_tx_synchroSocket  = NULL;
#endif
static EOdatagramSocket*        s_tx_dgramSocket    = NULL;
static EOpacket*                s_rxpkt             = NULL;
static EOpacket*                s_txpkt             = NULL;
static EOMtask*                 s_task_node         = NULL;
static EOaction*                s_action            = NULL;
static EOtimer*                 s_timer             = NULL;
static EOaction*                s_tmr_action        = NULL;


static const eOmessage_t s_message_from_skt = 0x00000001;
static const eOmessage_t s_message_from_tmr = 0x00000002;

static const eOipv4port_t s_node_rx_port_local = 3333;//3333;//3335;
static const eOipv4port_t s_node_tx_port_local = 3332;//3334;//3336;
static const eOipv4port_t s_node_tx_port_rem = 3334;

static eOmipnet_cfg_addr_t addrcfg;

static eOipv4addr_t s_remaddr;   //pc104 address.




static statistic_datastruct_t s_stat_data;    //here there are statistic info corrected during running time
static uint32_t seq_num_rx;  //received packet sequence number.it use to check if pkt are in order 
static uint32_t seq_num_tx;  //sent packet sequence number.it is use to know the next pkt seq num.



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


int main(void)
{

    eOmsystem_cfg_t syscfg =
    {
        .halcfg     = hal_params_cfgMINE,
        .osalcfg    = osal_params_cfgMINE,
        .fsalcfg    = fsal_params_cfgMINE
    };
    
    eom_sys_Initialise( &syscfg,
                        NULL,                   // mempool
                        NULL,                   // errman
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


//extern void eo_errman_OnError(eOerrmanErrorType_t errtype, uint32_t taskid, const char *eobjstr, const char *info)
//{
//    const char err[4][16] = {"info", "warning", "weak error", "fatal error"};
//#ifdef PRINT_DEBUG
//    printf("[eobj: %s, tsk: %d] %s: %s\n\r", eobjstr, taskid, err[(uint8_t)errtype], info);
//#endif
//    if(errtype <= eo_errortype_warning)
//    {
//        return;
//    }
//
//    for(;;);
//}


extern void task_node(void *p)
{
    // do here whatever you like before startup() is executed and then forever()
    eom_task_Start(p);
}  


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void s_udpnode_init(void)
{
    extern const ipal_params_cfg_t *ipal_params_cfgMINE;


    //init GPIO for test pourpose
    //speed parameter is not considered for input GPIO
    hal_gpio_init(hal_gpio_portC, hal_gpio_pin6, hal_gpio_dirINP, hal_gpio_speed_50MHz); 
    hal_gpio_init(hal_gpio_portE, hal_gpio_pin7, hal_gpio_dirOUT, hal_gpio_speed_50MHz); 
   
    hal_gpio_setval(hal_gpio_portE, hal_gpio_pin7, hal_gpio_valLOW);
   
   
    // init the action used for various tasks
    s_action = eo_action_New();  
    

    addrcfg.macaddr = ipal_params_cfgMINE->eth_mac;

    addrcfg.ipaddr = ipal_params_cfgMINE->eth_ip;

    addrcfg.ipmask = ipal_params_cfgMINE->eth_mask;

    // start the ipnet
    eom_ipnet_Initialise(&eom_ipnet_DefaultCfg,
                         (ipal_params_cfg_t*)ipal_params_cfgMINE, 
                         &addrcfg,
                         //NULL,
                         // &s_differentaddrcfg, 
                         &eom_ipnet_dtgskt_DefaultCfg,
                         1 // maxsynchrosocks
                         );


    s_task_node = eom_task_New(eom_mtask_MessageDriven/*eom_mtask_EventDriven*/, 69, 6*1024, s_node_startup, s_node_run,  6, 
                                    eok_reltimeINFINITE, 
                                   task_node, "node_appl");
    s_task_node = s_task_node; // just to remove warning
#ifdef PRINT_DEBUG
    printf("init done!! \n\r");
#endif
}


static void s_node_startup(EOMtask *p, uint32_t t)
{

    //initialise seq_num
    seq_num_rx = 0;
    seq_num_tx = 0; 
    
    s_stat_data.rec_pkt = 0;
    s_stat_data.lost_pkt = 0;
    s_stat_data.sent_pkt = 0;

    //initialise packet
    s_rxpkt = eo_packet_New(PACKET_SIZE);  
    s_txpkt = eo_packet_New(PACKET_SIZE);

    //init the timer and its action
    s_timer = eo_timer_New();  
    s_tmr_action = eo_action_New();
    eo_action_SetMessage(s_tmr_action, s_message_from_tmr, p);

//----------------- R X -----------------------------
    // initialise the receive datagram socket 
    s_rx_dgramSocket = eo_dtgsocket_New(  4, PACKET_SIZE, eom_mutex_New(), // input queue
                                    0, 0, NULL  // output queue
                                  );

    // set the rx action on socket to be a message s_message_from_skt to this task object
    eo_action_SetMessage(s_action, s_message_from_skt, p);


    // open the socket on port s_server_port to be tx-rx and and execute s_action upon rx of a datagram
    eo_dtgsocket_Open(s_rx_dgramSocket, s_node_rx_port_local, eo_sktdir_RXonly, eo_sktgetmode_bySignalling, s_action);  


#ifdef SOCK_SYNCHRO
// --------------- T X   S Y N C H R O ---------------------
    // initialise the transmit syncro socket     
    s_tx_synchroSocket = eo_synsocket_New(PACKET_SIZE); 
    
    //open the synchro socket on s_node_tx_port in tx, it starts at sys time = 0, and sends pkt each 1 ms 
    eo_synsocket_Open(s_tx_synchroSocket, s_node_tx_port_local, 
                                    eo_sktdir_TXonly, 0, 1000,
                                    eo_sktgetmode_bySignalling, NULL, NULL);

#else
// --------------- T X   N O R M A L ---------------------

    s_tx_dgramSocket = eo_dtgsocket_New(  0, 0, NULL, // input queue
                                          4, PACKET_SIZE, eom_mutex_New() // output queue
                                       );

    eo_dtgsocket_Open(s_tx_dgramSocket, s_node_tx_port_local, eo_sktdir_TXonly, eo_sktgetmode_bySignalling, NULL); 

#endif    
}

static void s_node_run(EOMtask *p, uint32_t t)
{

    // the message that we have received
    eOmessage_t msg = (eOmessage_t)t;

    if(s_message_from_skt == msg)//message from the socket
    {   
        s_evt_pkt_received();
    }

    if(s_message_from_tmr == msg)//message from the timer
    {
        s_evt_timer_expired();
    }
}


static void s_evt_pkt_received(void)
{
    static eOresult_t res;
    static uint8_t status = ST_NOT_CONNECTED;
    

    eOipv4port_t remport;
    uint8_t *data;
    uint16_t size;
    packet_t *my_pkt = NULL;

    eOnanotime_t receiv_time;
       
       
       
    res = eo_dtgsocket_Get(s_rx_dgramSocket, s_rxpkt, eok_reltimeZERO );
    
    if(eores_OK != res)
    {
#ifdef PRINT_DEBUG
        printf("Error in receiving packet\n\r");
#endif                  
        return;
    }

    hal_gpio_setval(hal_gpio_portE, hal_gpio_pin7, hal_gpio_valHIGH);
    hal_gpio_setval(hal_gpio_portE, hal_gpio_pin7, hal_gpio_valLOW);

    eov_sys_NanoTimeGet(eov_sys_GetHandle(), &receiv_time); 
    eo_packet_Destination_Get(s_rxpkt, &s_remaddr, &remport);
    eo_packet_Payload_Get(s_rxpkt, &data, &size);
    eo_packet_Payload_Get(s_rxpkt, &data, &size);
    my_pkt = (packet_t*)data;


    if(PACKET_SIZE != size)
    {
        if(PACKET_SIZE > size)
        {
        #ifdef PRINT_DEBUG
            printf("Warining: rec pkt size = %d. Expect %d\n\r", size, PACKET_SIZE );
        #endif                  
        }
        else
        {
        #ifdef PRINT_DEBUG
            printf("Error: rec pkt size = %d. Expect %d\n\r", size, PACKET_SIZE );
        #endif
            return;
        }

    }

    if( (NODE_ID_BROADCAST != my_pkt->hdr.node_id_dest) && (NODE_ID != my_pkt->hdr.node_id_dest) )
    {
        return; //node id not correct
    }

    //If i'm here, the pkt received is ok

    if(ST_NOT_CONNECTED == status)
    {
            
        if(s_connect_and_sendACK())
        {
            status = ST_READY;
        }
        return;
    }


    switch(my_pkt->hdr.type)
    {
        case PKT_TYPE_CMD:
            s_command_pkt_manage(my_pkt, &status);    
        break;
        
        case PKT_TYPE_DATA:
        {

            if( (ST_REPLY != status ) && (ST_LISTEN != status) )
            {
                return;
            }

            if(my_pkt->data.data_pload.pc104_info.seq_num == ((seq_num_rx +1)%UINT32_MAX))//no pkt lost
            { 
               s_stat_data.rec_pkt++;
        
               if(ST_REPLY == status)
               {
                   s_send_dataPkt(my_pkt, &receiv_time);
               }
            }
            else
            {
                if(my_pkt->data.data_pload.pc104_info.seq_num < seq_num_rx)
                {
                    s_stat_data.lost_pkt += (my_pkt->data.data_pload.pc104_info.seq_num + UINT32_MAX + 1) - seq_num_rx;
                }
                else
                {
                    s_stat_data.lost_pkt += my_pkt->data.data_pload.pc104_info.seq_num - seq_num_rx;
                }
            }
    
            seq_num_rx = my_pkt->data.data_pload.pc104_info.seq_num; //update sequence number
 
        }
        break;

        case PKT_TYPE_STATISTIC:
            //non mi dovrebbe arrivare
        break;
    
        default:
            return;
         
    
    }
    return;

}

static void s_evt_timer_expired(void)
{

    s_send_dataPkt(NULL, NULL);

}


static void s_collectedInfo_reset(void)
{
    //reset statistic
    memset(&s_stat_data, 0x0, sizeof(statistic_datastruct_t));
    //reset seq num  tx and rx
    seq_num_rx = 0;
    seq_num_tx = 0;
}


static void s_command_pkt_manage(packet_t *cmd_pkt, uint8_t *status)
{
    eOabstime_t time_in_microsec;
    uint32_t period_in_microsec;

    switch(cmd_pkt->data.cmd_pload.cmd)
    {
        case CMD_SET_TIME:
        {
            time_in_microsec = (cmd_pkt->data.cmd_pload.param[1]/1000)+ cmd_pkt->data.cmd_pload.param[0]*1000000;
            eov_sys_TimelifeSet(eov_sys_GetHandle(), time_in_microsec);
        }
        break; 

        case CMD_SEND_START:
        {
            s_collectedInfo_reset();
            period_in_microsec = cmd_pkt->data.cmd_pload.param[0];
            eo_timer_Start(s_timer, eok_abstimeNOW, period_in_microsec, eo_tmrmode_FOREVER, s_tmr_action);
        }
        break;
        
        case CMD_SEND_STOP:
        {
          eo_timer_Stop(s_timer);
        }
        break;
        
        case CMD_GET_STATISTICS:
        {
            s_send_statPkt();
        }
        break;

        case CMD_LISTEN_START:
        {
            s_collectedInfo_reset();
            *status = ST_LISTEN;
        }
        break;

        case CMD_LISTEN_STOP:
        {
            *status = ST_READY;
        }
        break;

        case CMD_SEND_and_LISTEN_START:
        {
            s_collectedInfo_reset();
            period_in_microsec = cmd_pkt->data.cmd_pload.param[0];
            eo_timer_Start(s_timer, eok_abstimeNOW, period_in_microsec, eo_tmrmode_FOREVER, s_tmr_action);
            *status = ST_LISTEN;
        }
        break;

        case CMD_SEND_and_LISTEN_STOP:
        {
            eo_timer_Stop(s_timer);
            *status = ST_READY;
        }
        break;

        case CMD_REPLAY_START:
        {
            s_collectedInfo_reset();
            *status = ST_REPLY;
        }
        break;

        case CMD_REPLAY_STOP:
        {
            *status = ST_READY;
        }
        break;

        default:
            return;

    }    
}


static void s_send_statPkt(void)
{
    packet_t statPkt_to_send;
    eOresult_t res;
    
    statPkt_to_send.hdr.node_id_dest = NODE_ID_PC104;
    statPkt_to_send.hdr.node_id_src = NODE_ID;
    statPkt_to_send.hdr.type = PKT_TYPE_STATISTIC;
    statPkt_to_send.data.stat_pload.received_pkt =  s_stat_data.rec_pkt;
    statPkt_to_send.data.stat_pload.sent_pkt =  s_stat_data.sent_pkt; 
    statPkt_to_send.data.stat_pload.lost_pkt =  s_stat_data.lost_pkt;

    res = s_send_pkt(&statPkt_to_send);

    if(eores_OK == res)
    {
        seq_num_tx++;
        s_stat_data.sent_pkt++;
    }

}



static void s_send_ackPkt(void)
{
     packet_t ackPkt_to_send;
     eOresult_t res;

     memset(&ackPkt_to_send, 0x0, PACKET_SIZE);
     
    //fill header
    ackPkt_to_send.hdr.node_id_dest = NODE_ID_PC104;
    ackPkt_to_send.hdr.node_id_src = NODE_ID;
    ackPkt_to_send.hdr.type = PKT_TYPE_ACK;
    
    res = s_send_pkt(&ackPkt_to_send);

    if(eores_OK ==res)
    {
        seq_num_tx++;
        s_stat_data.sent_pkt++;
    }
}

static void s_send_dataPkt(packet_t *received_pkt, eOnanotime_t *receive_time)
{
    packet_t dataPkt_to_send;
    eOresult_t res;
    eOnanotime_t nt;
    
    
    //NOTE: when this function is invoked, in rec_pkt->receiv_time must be set already.
    
    memset(&dataPkt_to_send, 0x0, PACKET_SIZE);
    
    //fill header
    dataPkt_to_send.hdr.node_id_dest = NODE_ID_PC104;
    dataPkt_to_send.hdr.node_id_src = NODE_ID;
    dataPkt_to_send.hdr.type = PKT_TYPE_DATA;
    
    //fill payload
    dataPkt_to_send.data.data_pload.ems_info.seq_num = seq_num_tx;
 

    if(NULL != receive_time)
    {
       dataPkt_to_send.data.data_pload.ems_info.receive_time.secs = receive_time->secs;
       dataPkt_to_send.data.data_pload.ems_info.receive_time.nano = receive_time->secs;
    }

    dataPkt_to_send.data.data_pload.ems_info.idle_time = osal_system_idletime_get();
    

    if(NULL != received_pkt)
    {
        dataPkt_to_send.data.data_pload.pc104_info.send_time =  received_pkt->data.data_pload.pc104_info.send_time;
    }

    eov_sys_NanoTimeGet(eov_sys_GetHandle(), &nt);
    dataPkt_to_send.data.data_pload.ems_info.send_time.secs = nt.secs;
    dataPkt_to_send.data.data_pload.ems_info.send_time.nano = nt.nano;

    res = s_send_pkt(&dataPkt_to_send);

    if(eores_OK ==res)
    {
        seq_num_tx++;
        s_stat_data.sent_pkt++;
    }

}
/*
received_pkt and receive_time can be null.
*/
static eOresult_t s_send_pkt(packet_t *pkt_to_send)
{
    eOresult_t res;


    eo_packet_Full_Clear(s_txpkt, 0xAA);
   
    eo_packet_Full_Set(s_txpkt, s_remaddr, s_node_tx_port_rem, PACKET_SIZE, (uint8_t*)pkt_to_send);
    
    hal_gpio_setval(hal_gpio_portE, hal_gpio_pin7, hal_gpio_valHIGH);
    hal_gpio_setval(hal_gpio_portE, hal_gpio_pin7, hal_gpio_valLOW);
       
#ifdef SOCK_SYNCHRO    
    res = eo_synsocket_Put(s_tx_synchroSocket, s_txpkt);
#else    
    res = eo_dtgsocket_Put(s_tx_dgramSocket, s_txpkt);
#endif    


#ifdef PRINT_DEBUG                
    if(eores_NOK_generic == res)
    {
        printf("Error in trasmitting pkt!!!!\n\r");
    }
#endif

    return(res);

}

static eObool_t s_connect_and_sendACK(void)
{
    eOresult_t res;
#ifdef PRINT_DEBUG
    printf("not yet connected \n\r");
#endif


#ifdef SOCK_SYNCHRO
    res = eo_synsocket_Connect(s_tx_synchroSocket, s_remaddr, eok_reltime1sec);
#else    
    res = eo_dtgsocket_Connect(s_tx_dgramSocket, s_remaddr, eok_reltime1sec);
#endif

    
    if(eores_OK == res)
    {
#ifdef PRINT_DEBUG
        printf("connecetd now \n\r");
#endif
        s_send_ackPkt();
        return(eobool_true);
    
    }
    else
    {
#ifdef PRINT_DEBUG
        printf("not connected after %d ms. i shall try again at next reception\n\r", eok_reltime1sec/1000);
#endif
        return(eobool_false);
    }
}


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



