
/* @file       app-ipnet.c
	@brief      This file implements a test for abslayer with a telnet server, an ftp server (hal + ipal + fsal), a 
                blinking led facility (hal) done w/ a sw timer (osal). wow.
	@author     marco.accame@iit.it
    @date       06/16/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdint.h"
#include "stdlib.h"
#include "string.h"

#include "hal.h"
//#include "fsal.h"

//#include "app-telnet.h"

#include "shalINFO.h" 

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "app-ipnet.h"
#include "updater-core.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------

extern ipal_cfg_t *ipal_cfgMINE;
 

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

#define HTONS(a)    ((((a)&0x00ff)<<8) | (((a)&0xff00)>>8))


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------

typedef struct
{
    uint8_t             opc;
    uint8_t             forfutureuse;     // size of data ????
    uint16_t            adr;
} alpacket_header_t;

typedef struct
{
    alpacket_header_t   head;  
    uint8_t             data[1];
} alpacket_t;


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_task_ipal_periodic_tick(void *p);
static void s_ipal_nomemory_anymore(void);
//static uint16_t s_tnet_onexec(const char *cmd, char *rep, uint8_t *quitflag);


static void onrec_dgram(void *arg, ipal_udpsocket_t *skt, ipal_packet_t *pkt, ipal_ipv4addr_t adr, ipal_port_t por);
static void s_task_udp_owner(void *p);

//static void s_msg_do_action(datagram_t *rxdgram);
//static datagram_t * s_msg_form_reply_if_needed(datagram_t *rxdgram);

//static int s_rename_not(const char *n, const char *m);

//static void s_appipnet_task_wakeup(void);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

//static const ipal_ftp_cfg_t s_ipal_ftp =
//{
//    fopen, 
//    fclose,
//    fread,
//    fwrite,
//    (int (*)(const char *))fsal_delete,  
//    s_rename_not, // disable rename.    //    (int (*)(const char *, const char *))fsal_rename,
//    (int (*)(const char *, void *))fsal_find
//};

//static const char s_tel_msg_login[] = 
//{
//    "                                           \r\n"
//    "       eUpdater                            \r\n"
//    "                                           \r\n"
//};
//
//static const char s_tel_msg_welcome[] = 
//{
//    "       Welcome to eUpdater                  \r\n"
//    "       Type help for supported commands     \r\n"
//    "                                            \r\n"
//};
//
//static const char s_tel_prompt[] = "\r\nupdt> ";

//static const ipal_telnet_cfg_t s_ipal_tel =
//{
//    s_tel_msg_login,
//    s_tel_msg_welcome,
//    s_tel_prompt,
//    apptelnet_onexecupdater //apptelnet_onexecsafe //s_tnet_onexec
//};


static osal_task_t *s_tskid_ip      = NULL;

//static osal_task_t *s_task_udp      = NULL;

static ipal_udpsocket_t *s_thesocket = NULL;

static ipal_ipv4addr_t mcastgroup = IPAL_ipv4addr(239, 0, 0, 1);
static osal_messagequeue_t *s_rxpktqueue = NULL;
static osal_messagequeue_t *s_txpktqueue = NULL;

static ipal_cfg_t s_ipal_ram_cfg;

static const shalinfo_deviceinfo_t* s_deviceinfo = NULL;

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern void appinet_start_ipal(void)
{
   // create task for ip. very high priority
    s_tskid_ip = osal_task_new(appipnet_task_launcher, NULL, 50, 2048);
}

extern void appipnet_task_launcher(void *p)
{
    uint32_t ram32sizeip = 0;
    uint32_t *ram32dataip = NULL;
    const osal_time_t ipal_maxidle = 1*1000; // 1 ms.
    osal_result_t res;
    osal_eventflag_t evt = 0;
    datagram_t *txdgram = NULL;
    ipal_packet_t pkt;

    const ipal_tos_t tos = 
    { 
        .precedence = ipal_prec_immediate, 
        .lowdelay = 1, 
        .highthroughput = 0, 
        .highreliability = 1, 
        .unused = 0 
    };

    // build the cfg w/ mac + ip + msk from the shalINFO.
    memcpy(&s_ipal_ram_cfg, ipal_cfgMINE, sizeof(ipal_cfg_t));

    shalinfo_deviceinfo_get(&s_deviceinfo);
    
    if (s_deviceinfo->ipnetwork.ipaddress != 0)
    {
        s_ipal_ram_cfg.eth_ip = s_deviceinfo->ipnetwork.ipaddress;
    }
    if (s_deviceinfo->ipnetwork.ipnetmask != 0)
    {
        s_ipal_ram_cfg.eth_mask = s_deviceinfo->ipnetwork.ipnetmask;    
    }
    /*
    if (s_deviceinfo->ipnetwork.macaddress != 0xFFFFFFFFFFFFFFFF)
    {
        s_ipal_ram_cfg.eth_mac = s_deviceinfo->ipnetwork.macaddress;
    }
    */

    // get memory
    ipal_base_memory_getsize(&s_ipal_ram_cfg, &ram32sizeip);
    ram32dataip = (uint32_t*)calloc(ram32sizeip/4, sizeof(uint32_t));
    
    if(NULL == ram32dataip)
    {
        s_ipal_nomemory_anymore();
    }

    // initialise
    ipal_base_initialise(&s_ipal_ram_cfg, ram32dataip);

    // create the rx-message-queue
    s_rxpktqueue = osal_messagequeue_new(3);

    // create the tx-message-queue
    s_txpktqueue = osal_messagequeue_new(3);

    // create task user of udp services
    /*s_task_udp =*/ osal_task_new(s_task_udp_owner, NULL, 31, 512);
    
    
    // start ipal
    ipal_sys_start();
      

    // create task periodic tick, low priority
    osal_task_new(s_task_ipal_periodic_tick, (void*)s_ipal_ram_cfg.sys_timetick, 5, 128);


    // start a udp socket
    s_thesocket = ipal_udpsocket_new(tos);
    ipal_udpsocket_bind(s_thesocket, IPAL_ipv4addr_INADDR_ANY, 1001);
    ipal_udpsocket_recv(s_thesocket, onrec_dgram, NULL);
    // register that on multicasting 239.0.0.1
    ipal_igmp_join(mcastgroup);


    upd_core_init(s_txpktqueue, s_tskid_ip);
    // use a timeout-ed wait for eventflags from the eth isr or a sending task
    for(;;)
    {
        res = osal_eventflag_get(0xffffffff, osal_waitANYflag, &evt, ipal_maxidle); 

        if(osal_res_OK == res)
        {
            if(0x0002 & evt)
            {
                // if there is need to tx, then .... get the msg from the the tx queue,
                // retrieve: ipaddr (4b), port (2b), size (2b), data. copy into ipbuffer
                // copy ipaddr etc. remember to free the msg pointer.

                // send pkt.

                osal_messagequeue_get(s_txpktqueue, (osal_message_t*)&txdgram, 100*1000, osal_callerTSK);

                pkt.data = &txdgram->data[0]; 
                pkt.size = txdgram->head.size;
                ipal_udpsocket_sendto(s_thesocket, &pkt, txdgram->head.ipaddr, txdgram->head.port); 

                free(txdgram);                                         
            }
        }
        
        ipal_sys_process_communication();
    }    
}


extern void appipnet_task_wakeup(void)
{
    osal_eventflag_set(0x0001, s_tskid_ip, osal_callerISR);
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

  


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------



static void s_task_ipal_periodic_tick(void *p)
{
    osal_task_period_set((osal_time_t)p);

    for(;;)
    {
        osal_task_period_wait();
        // do action .... tick ipal
        ipal_sys_timetick_increment();
    }
}



static void s_ipal_nomemory_anymore(void)
{
    for(;;);
}

static void s_task_udp_owner(void *p)
{
    osal_result_t res;

    datagram_t *rxdgram = NULL;

    for(;;)
    {
        res = osal_messagequeue_get(s_rxpktqueue, (osal_message_t*)&rxdgram, 100*1000, osal_callerTSK); 
        
        if(osal_res_NOK_timeout != res)
        {
            upd_core_manage_cmd(rxdgram);
        }
    }
}

static void onrec_dgram(void *arg, ipal_udpsocket_t *skt, ipal_packet_t *pkt, ipal_ipv4addr_t adr, ipal_port_t por)
{
    datagram_t *dgram = calloc(sizeof(datagram_header_t) + pkt->size, 1);

    if(NULL != dgram)
    {
        dgram->head.ipaddr = adr;
        dgram->head.port = por;
        dgram->head.size = pkt->size;
        memcpy(&dgram->data[0], pkt->data, pkt->size);
        
        osal_messagequeue_put(s_rxpktqueue, (osal_message_t)dgram, 10*1000, osal_callerTSK);    
    }
}

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



