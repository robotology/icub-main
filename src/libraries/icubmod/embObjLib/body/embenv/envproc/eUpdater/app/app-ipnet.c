
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
#include "osal.h"
#include "ipal.h"
#include "fsal.h"

#include "app-telnet.h"

#include "shalINFO.h" 

// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------

extern ipal_cfg_t *ipal_cfgMINE;
 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "app-ipnet.h"

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
    ipal_ipv4addr_t     ipaddr;
    ipal_port_t         port;
    uint16_t            size;
} datagram_header_t;

typedef struct
{
    datagram_header_t   head;
    uint8_t             data[1];
} datagram_t;

typedef struct
{
    osal_mutex_t    *mtx;
    uint8_t         first;
    uint8_t         last;
    uint8_t         size;
    datagram_t      q[8];
} datagram_fifo_t;


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
static uint16_t s_tnet_onexec(const char *cmd, char *rep, uint8_t *quitflag);


static void onrec_dgram(void *arg, ipal_udpsocket_t *skt, ipal_packet_t *pkt, ipal_ipv4addr_t adr, ipal_port_t por);
static void s_task_udp_owner(void *p);

static void s_msg_do_action(datagram_t *rxdgram);
static datagram_t * s_msg_form_reply_if_needed(datagram_t *rxdgram);

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

static osal_task_t *s_task_udp      = NULL;

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



//    const shalinfo_macaddr_t *macaddr = NULL;
//    const shalinfo_ipv4addr_t  *ipaddr = NULL;
//    const shalinfo_ipv4addr_t  *netmask = NULL;
    const ipal_tos_t tos = { .precedence = ipal_prec_immediate, .lowdelay = 1, .highthroughput = 0, .highreliability = 1, .unused = 0 };


    // build the cfg w/ mac + ip + msk from the shalINFO.
    memcpy(&s_ipal_ram_cfg, ipal_cfgMINE, sizeof(ipal_cfg_t));

    shalinfo_deviceinfo_get(&s_deviceinfo);
//    s_ipal_ram_cfg.eth_mac  = s_deviceinfo->ipnetwork.macaddress;
//    s_ipal_ram_cfg.eth_ip   = s_deviceinfo->ipnetwork.ipaddress;
//    s_ipal_ram_cfg.eth_mask = s_deviceinfo->ipnetwork.ipnetmask;

    
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
    s_task_udp = osal_task_new(s_task_udp_owner, NULL, 31, 512);
    
    
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


static uint16_t s_tnet_onexec(const char *cmd, char *rep, uint8_t *quitflag)
{
    uint16_t cmdlen =0;
    static uint16_t cnt = 0;
    uint16_t retlen = 0;
    int arg1 = 0;

    cmdlen = strlen(cmd);
    *quitflag = 0;

    if(0 == strncmp(cmd, "help", strlen("help")))
    {
        if(cmdlen > 5)
        {
            sscanf((const char*)(cmd+5), "%d", &arg1);
            ipal_ftp_stop();
        } 
        
        retlen = sprintf ((char *)rep,"\r\n help found # %d w/ arg %d", cnt++, arg1);    

    }
    else if(0 == strncmp(cmd, "quit", strlen("quit")))
    {
        retlen = sprintf ((char *)rep,"\r\n quitting telnet... bye\r\n"); 
        *quitflag = 1;
        ipal_ftp_restart();
    }
    else
    {
        retlen = sprintf ((char *)rep,"\r\n command not found. type help for supported commands");
    }

    return(retlen);
}


static void s_task_udp_owner(void *p)
{
    osal_result_t res;

    datagram_t *rxdgram = NULL;
    datagram_t *txdgram = NULL;


    for(;;)
    {
        res = osal_messagequeue_get(s_rxpktqueue, (osal_message_t*)&rxdgram, 100*1000, osal_callerTSK); 

        
        if(osal_res_NOK_timeout != res)
        {
            // ok, parse it and then free the received pointer. 

            printf("received a udp packet\n");

            // do action
            s_msg_do_action(rxdgram);
            // prepare reply
            txdgram = s_msg_form_reply_if_needed(rxdgram);
            // free rx pkt before sending message
            free(rxdgram);
            // tx reply
            if(NULL != txdgram)
            {
                osal_messagequeue_put(s_txpktqueue, (osal_message_t)txdgram, 10*1000, osal_callerTSK);  
                osal_eventflag_set(0x0002, s_tskid_ip, osal_callerTSK);
            }

        }

 

    }


}


static void onrec_dgram(void *arg, ipal_udpsocket_t *skt, ipal_packet_t *pkt, ipal_ipv4addr_t adr, ipal_port_t por)
{

    datagram_t *dgram = NULL;

    if(pkt->size > 32)
    {
        // we assume that no message we want to receive is longer than 32 bytes.
        return;
    }

    // all other messages .... we pass them to the parser

    dgram = calloc(sizeof(datagram_header_t) + pkt->size, 1);

    if(NULL != dgram)
    {
        dgram->head.ipaddr = adr;
        dgram->head.port = por;
        dgram->head.size = pkt->size;
        memcpy(&dgram->data[0], pkt->data, pkt->size);
        
        osal_messagequeue_put(s_rxpktqueue, (osal_message_t)dgram, 10*1000, osal_callerTSK);    
    }

}


static void s_msg_do_action(datagram_t *rxdgram)
{
    // parse it. if its opcode is a write .... then do something: a restart or something else
}


static datagram_t * s_msg_form_reply_if_needed(datagram_t *rxdgram)
{
    datagram_t *rep = NULL;
    static alpacket_header_t alpkt_hea_queryip = {0x01, 0x00, HTONS(0xf001)};
    static alpacket_header_t alpkt_hea_replyip = {0x02, 0x00, HTONS(0xf001)};

    if(0 == memcmp(&rxdgram->data[0], &alpkt_hea_queryip, 4))
    {
        rep = calloc(sizeof(datagram_header_t) + sizeof(alpacket_header_t) + sizeof(uint32_t), 1);

        rep->head.ipaddr = rxdgram->head.ipaddr;
        rep->head.port   = 1001; 
        rep->head.size   = sizeof(alpacket_header_t) + sizeof(uint32_t);

        shalinfo_deviceinfo_get(&s_deviceinfo);

        memcpy(&rep->data[0], &alpkt_hea_replyip, 4);                       // header
        memcpy(&rep->data[4], &s_deviceinfo->ipnetwork.ipaddress, 4);       // data
    }

    return(rep);
}




// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



