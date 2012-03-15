
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
#include "EOMmutex.h"
#include "EOdatagramSocket.h"
#include "EOsynchroSocket.h"








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
#define PACKET_SIZE     256
#define NODE_ID         0
#define NODE_BROADCAST  255


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------

typedef struct
{
    uint32_t        node_id;    //4 byte
    uint32_t        seq_num;
    eOnanotime_t    pc104_time; //pc104 save in this field its sending time.
    uint32_t        cmd; //if the pkt is received form pc104, this filed can values 0 or 1.
                         //     if 1 the node must set pc104_time as its system life time, else ignore.
                         //if the pkt is trasmitted from node, then cmd field takes mean of ack.
                         //Actually it is alway set to one.
    uint32_t        pkt_received;
    uint32_t        pkt_lost;
    uint32_t        pad_for_idle_time; //idle time is set at addres multiply of 4
    osal_lifetime_t idle_time;
    eOnanotime_t    receiv_time; // This fiels is used by ems only. it contains receiving time.
    eOnanotime_t    send_time;

    uint8_t         pad[PACKET_SIZE-52];
} pkt_payload_t;

// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_udpnode_init(void);

static void s_node_startup(EOMtask *p, uint32_t t);
static void s_node_run(EOMtask *p, uint32_t t);
static void s_send_pkt(eOipv4addr_t remote_addr, pkt_payload_t *receive_pkt);
static eObool_t s_connect_and_send(eOipv4addr_t remote_addr);
static void s_send_ack(eOipv4addr_t remote_addr);
static eObool_t s_connect_and_sendACK(eOipv4addr_t remote_addr);
static eOresult_t s_sock_put(EOpacket* pkt);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static EOdatagramSocket*        s_rx_dgramSocket    = NULL;
static EOsynchroSocket*         s_tx_synchroSocket  = NULL;
static EOdatagramSocket*        s_tx_dgramSocket    = NULL;
static EOpacket*                s_rxpkt             = NULL;
static EOpacket*                s_txpkt             = NULL;
static EOMtask*                 s_task_node         = NULL;
static EOaction*                s_action            = NULL;


static const eOmessage_t s_message_from_skt = 0x00000001;

static const eOipv4port_t s_node_rx_port_local = 3333;//3333;//3335;
static const eOipv4port_t s_node_tx_port_local = 3332;//3334;//3336;
static const eOipv4port_t s_node_tx_port_rem = 3334;

static uint32_t seq_num_rx;
static uint32_t seq_num_tx;
static uint32_t rec_pkt;
static uint32_t lost_pkt;

static eOmipnet_cfg_addr_t addrcfg;

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


int main(void)
{

    
    eom_sys_Initialise( hal_params_cfgMINE,
                        osal_params_cfgMINE,
                        fsal_params_cfgMINE,
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


extern void eo_errman_OnError(eOerrmanErrorType_t errtype, uint32_t taskid, const char *eobjstr, const char *info)
{
    const char err[4][16] = {"info", "warning", "weak error", "fatal error"};
#ifdef PRINT_DEBUG
    printf("[eobj: %s, tsk: %d] %s: %s\n\r", eobjstr, taskid, err[(uint8_t)errtype], info);
#endif
    if(errtype <= eo_errortype_warning)
    {
        return;
    }

    for(;;);
}


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
    uint8_t *my_p;


    //init GPIO for test pourpose
    //speed parameter is not considered for input GPIO
    hal_gpio_init(hal_gpio_portC, hal_gpio_pin6, hal_gpio_dirINP, hal_gpio_speed_50MHz); 
    hal_gpio_init(hal_gpio_portE, hal_gpio_pin7, hal_gpio_dirOUT, hal_gpio_speed_50MHz); 
   
    hal_gpio_setval(hal_gpio_portE, hal_gpio_pin7, hal_gpio_valLOW);
   
   
    // init the action used for various tasks
    s_action = eo_action_New();    

    my_p = (uint8_t*)ipal_params_cfgMINE->eth_mac;
    addrcfg.macaddr = EO_COMMON_MACADDR(my_p[0],my_p[1],my_p[2],my_p[3], my_p[4], my_p[5]);
    
    my_p = (uint8_t*)ipal_params_cfgMINE->eth_ip;
    addrcfg.ipaddr = EO_COMMON_IPV4ADDR(my_p[0],my_p[1],my_p[2],my_p[3]);
    
    my_p = (uint8_t*)ipal_params_cfgMINE->eth_mask;
    addrcfg.ipmask = EO_COMMON_IPV4ADDR(my_p[0],my_p[1],my_p[2],my_p[3]);

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
#ifdef PRINT_DEBUG
    printf("init done!! \n\r");
#endif
}


static void s_node_startup(EOMtask *p, uint32_t t)
{

    //initialise seq_num
    seq_num_rx = 0;
    seq_num_tx = 0; 
    
    rec_pkt = 0;
    lost_pkt = 0;

    //initialise packet
    s_rxpkt = eo_packet_New(PACKET_SIZE);  
    s_txpkt = eo_packet_New(PACKET_SIZE);


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
    // read the packet.
    static eOresult_t res;
    eOipv4addr_t remaddr;
    eOipv4port_t remport;
    uint8_t *ipaddr;
    uint8_t *data;
    uint16_t size;
    pkt_payload_t *pktdata = NULL;
    static eObool_t connected = eobool_false;
    eOabstime_t time_in_microsec;
    eOnanotime_t receiv_time;
    

    // the message that we have received
    eOmessage_t msg = (eOmessage_t)t;

    if(s_message_from_skt == msg)
    {   // ok, message from the socket

        res = eo_dtgsocket_Get(s_rx_dgramSocket, 
                               s_rxpkt, 
                               eok_reltimeZERO //eok_reltimeINFINITE
                               );
    
        if(eores_OK == res)
        {
            hal_gpio_setval(hal_gpio_portE, hal_gpio_pin7, hal_gpio_valHIGH);

            eov_sys_NanoTimeGet(eov_sys_GetHandle(), &receiv_time); 

            eo_packet_Destination_Get(s_rxpkt, &remaddr, &remport);
            eo_packet_Payload_Get(s_rxpkt, &data, &size);
            ipaddr = ((uint8_t*)&remaddr);
            pktdata = (pkt_payload_t*)data;

            //check packet size
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

            //check node id
            if((NODE_BROADCAST != pktdata->node_id) && (NODE_ID != pktdata->node_id))
            {
                // printf("Error: Node_id in rec pkt mismatch!!!!\n\r");
                 return;
            }
            
            if(pktdata->cmd == 1)
            {
                time_in_microsec = (pktdata->pc104_time.nano/1000)+ pktdata->pc104_time.secs*1000000;
                eov_sys_TimelifeSet(eov_sys_GetHandle(), time_in_microsec);
            }


            if(eobool_false == connected)
            {
                
                connected = s_connect_and_sendACK(remaddr);
//                if(pktdata->cmd == 1)
//                {
//                    time_in_microsec = (pktdata->host_time.nano/1000)+ pktdata->host_time.secs*1000000;
//                    eov_sys_TimelifeSet(eov_sys_GetHandle(), time_in_microsec);
//                }
                return;
            }

            //check sequence number
            if(pktdata->seq_num == ((seq_num_rx +1)%UINT32_MAX))//no pkt lost
            { 
                rec_pkt++;

                if(pktdata->cmd == 1)
                {
                   time_in_microsec = (pktdata->pc104_time.nano/1000)+ pktdata->pc104_time.secs*1000000;
                   
                   eov_sys_TimelifeSet(eov_sys_GetHandle(), time_in_microsec);
                   //printf("Time life set!!\n\r");

//                   if(eobool_false == connected)
//                   {
//                        connected = s_connect_and_send(remaddr);
//                        not_send = eobool_false;
//                   }
                }
                 pktdata->receiv_time = receiv_time;
                 s_send_pkt(remaddr, pktdata);
            }
            else
            {
                if(pktdata->seq_num < seq_num_rx)
                {
                    lost_pkt += (pktdata->seq_num + UINT32_MAX + 1) - seq_num_rx;
                }
                else
                {
                    lost_pkt += pktdata->seq_num - seq_num_rx;
                }
            }

            seq_num_rx = pktdata->seq_num; //update sequence number

            
//            wait_to_print--;
//            //print statistics
//            if(0 == wait_to_print)
//            {
//                wait_to_print = 100;
//                idle_time = osal_system_idletime_get();
//                
//                printf("pkt received = %d     pkt lost = %d  idle time = %llu \n\r", rec_pkt, lost_pkt, idle_time);
 //           }

//            if(eobool_false == connected) // node could not connect on firts pkt with cmd ==1. re-try here!!!
//            {
//                connected = s_connect_and_send(remaddr);
//                not_send = eobool_false;
//            }
//
//            if((eobool_true == connected) && (eobool_true == not_send))
//            {
//                s_send_pkt(remaddr);
//            }
        }

    }

}
static void s_send_ack(eOipv4addr_t remote_addr)
{
     pkt_payload_t pktdata_to_send;
     eOresult_t res;

     
     pktdata_to_send.node_id = NODE_ID;
     pktdata_to_send.seq_num = 0; 
     pktdata_to_send.cmd = 1;    //ACK
 
    
     eo_packet_Full_Set(s_txpkt, remote_addr, s_node_tx_port_rem, PACKET_SIZE, (uint8_t*)&pktdata_to_send);
     
     hal_gpio_setval(hal_gpio_portE, hal_gpio_pin7, hal_gpio_valLOW);
                
//     eo_synsocket_Put(s_tx_synchroSocket, s_txpkt);

     res = s_sock_put(s_txpkt);
                
     if(eores_NOK_generic == res)
     {
#ifdef PRINT_DEBUG
        printf("Error in trasmitting ack!!!!\n\r");
#endif
     }
}
static void s_send_pkt(eOipv4addr_t remote_addr, pkt_payload_t *receive_pkt)
{
     eOresult_t res;
     pkt_payload_t pktdata_to_send;


//NOTE: when this function is invoked, in rec_pkt->receiv_time must be set already.
     
     memset(&pktdata_to_send, 0xA, PACKET_SIZE);

     pktdata_to_send.receiv_time = receive_pkt->receiv_time;
     pktdata_to_send.pc104_time = receive_pkt->pc104_time;  

     pktdata_to_send.node_id = NODE_ID;
     pktdata_to_send.seq_num = seq_num_tx;
     seq_num_tx++;
      
     pktdata_to_send.cmd = 0;
     pktdata_to_send.pkt_received = rec_pkt;
     pktdata_to_send.pkt_lost = lost_pkt;
     pktdata_to_send.idle_time = osal_system_idletime_get();

     eov_sys_NanoTimeGet(eov_sys_GetHandle(), &pktdata_to_send.send_time);
    
     eo_packet_Full_Set(s_txpkt, remote_addr, s_node_tx_port_rem, PACKET_SIZE, (uint8_t*)&pktdata_to_send);
     
     hal_gpio_setval(hal_gpio_portE, hal_gpio_pin7, hal_gpio_valLOW);
               
     //res = eo_synsocket_Put(s_tx_synchroSocket, s_txpkt);
     res = s_sock_put(s_txpkt);
                
     if(eores_NOK_generic == res)
     {
#ifdef PRINT_DEBUG
        printf("Error in trasmitting pkt!!!!\n\r");
#endif
     }
}

static eObool_t s_connect_and_sendACK(eOipv4addr_t remote_addr)
{
    eOresult_t res;
#ifdef PRINT_DEBUG
    printf("not yet connected \n\r");
#endif


#ifdef SOCK_SYNCHRO
    res = eo_synsocket_Connect(s_tx_synchroSocket, remote_addr, eok_reltime1sec);
#else    
    res = eo_dtgsocket_Connect(s_tx_dgramSocket, remote_addr, eok_reltime1sec);
#endif

    
    if(eores_OK == res)
    {
#ifdef PRINT_DEBUG
        printf("connecetd now \n\r");
#endif
        s_send_ack(remote_addr);
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

static eOresult_t s_sock_put(EOpacket* pkt)
{
    eOresult_t res;
    
#ifdef SOCK_SYNCHRO    
    res = eo_synsocket_Put(s_tx_synchroSocket, pkt);
#else    
    res = eo_dtgsocket_Put(s_tx_dgramSocket, pkt);
#endif    
    return res;
}

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



