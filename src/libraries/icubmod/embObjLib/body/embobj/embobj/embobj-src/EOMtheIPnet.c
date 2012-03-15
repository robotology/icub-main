
/* @file       EOMtheIPnet.c
	@brief      This file implements the IP net singleton for MEE
	@author     marco.accame@iit.it
    @date       08/24/2011
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "string.h"

#include "EoCommon.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"
#include "EOtimer.h"
#include "EOlist.h"

#include "EOVtheIPnet.h"
#include "EOVtheIPnet_hid.h"
#include "EOsocketDatagram_hid.h"
#include "EOVtask.h"
#include "EOMmutex.h"

#include "EOpacket_hid.h"
#include "EOsocket_hid.h"

#include "osal.h"
#include "ipal.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOMtheIPnet.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOMtheIPnet_hid.h" 
#include "EOMmutex_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

#undef SLOWERMODEONRXDATAGRAM
//#define SLOWERMODEONRXDATAGRAM

#define _MAXTOKENS_SEM_CMD_  2  // but as it is used as a binary semaphore, then 1 would be enough 


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------

const eOmipnet_cfg_t eom_ipnet_DefaultCfg = 
{ 
    .procpriority               = 220, 
    .procstacksize              = 1024, 
    .procmaxidletime            = 20000, 
    .procwakeuponrxframe        = eobool_true, 
    .tickpriority               = 219, 
    .tickstacksize              = 128
};

const eOmipnet_cfg_dtgskt_t eom_ipnet_dtgskt_DefaultCfg = 
{   
    .numberofsockets            = 2, 
    .maxdatagramenqueuedintx    = 8
};
 
const eOmipnet_cfg_dtgskt_t eom_ipnet_dtgskt_NOsocketCfg = 
{
    .numberofsockets            = 0, 
    .maxdatagramenqueuedintx    = 0
};

const eOmipnet_cfg_addr_t eom_ipnet_addr_DefaultCfg = 
{
    .macaddr                    = 0,
    .ipaddr                     = 0,
    .ipmask                     = 0
};


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void e_eom_ipnet_signal_new_frame_is_available(void);

static void s_eom_ipnet_tsktick_forever(EOMtask *rt, uint32_t n);

static void s_eom_ipnet_tskproc_startup(EOMtask *rt, uint32_t n);

static void s_eom_ipnet_tskproc_forever(EOMtask *rt, uint32_t msg);

static eOresult_t s_eom_ipnet_ARP(EOVtheIPnet *ip, eOipv4addr_t ipaddr, eOreltime_t tout);
static eOresult_t s_eom_ipnet_AttachSocket(EOVtheIPnet* ip, EOsocketDerived *s);
static eOresult_t s_eom_ipnet_DetachSocket(EOVtheIPnet* ip, EOsocketDerived *s);
static eOresult_t s_eom_ipnet_Alert(EOVtheIPnet* ip, void *eobjcaller, eOevent_t evt);
static eOresult_t s_eom_ipnet_WaitPacket(EOVtheIPnet* ip, EOsocketDerived *s, eOreltime_t tout);

static void s_eom_ipnet_OnReceptionDatagram(void *arg, ipal_udpsocket_t *skt, ipal_packet_t *pkt, ipal_ipv4addr_t adr, ipal_port_t por);



static void s_eom_ipnet_process_command(void);
static void s_eom_ipnet_stop_command(void);
static void s_eom_ipnet_repeat_command(void);

static void s_eom_ipnet_process_transmission_datagram(void);


static eObool_t s_eom_ipnet_attach_rqst_dtgsocket(EOVtheIPnet *vip, EOsocketDatagram *dgmskt);
static void s_eom_ipnet_attach_proc_dtgsocket(EOsocketDatagram *dtgs);
static eObool_t s_eom_ipnet_detach_rqst_dtgsocket(EOVtheIPnet *vip, EOsocketDatagram *dgmskt);
static void s_eom_ipnet_detach_proc_dtgsocket(EOsocketDatagram *dtgs);

#ifdef SLOWERMODEONRXDATAGRAM
static eOresult_t s_eom_ipnet_DatagramSocketHas(void *item, void *param);
#endif




// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
 
static const char s_eobj_ownname[] = "EOMtheIPnet";

static EOMtheIPnet s_eom_theipnet = 
{
    .ipnet                  = NULL,                               // ipnet
    .tskproc                = NULL,                               // tskproc
    .tsktick                = NULL,                               // tsktick
    .cmd                    =
                            {
                                .mtxcaller  = NULL, 
                                .opcode     = cmdDoNONE, 
                                .repeatcmd  = 0,
                                .result     = 0,
                                .par16b     = 0,
                                .par32b     = 0,
                                .tout       = 0, 
                                .semaphore  = NULL, 
                                .stoptmr    = NULL,
                                .stopact    = NULL
                            },   // cmd
    .rxpacket               = NULL,                                
    .maxwaittime            = 0,                                  // maxwaittime 
    .dgramsocketready2tx    = NULL,                               // dgramsocketready2tx  
//    .ipcfg                  = {0},                                // ipcfg
    .taskwakeuponrxframe    = eobool_false                            // taskwakeuponrxframe
};



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern EOMtheIPnet * eom_ipnet_Initialise(const eOmipnet_cfg_t *ipnetcfg,
                                          const ipal_cfg_t *ipcfg, 
                                          const eOmipnet_cfg_addr_t *addrcfg,
                                          const eOmipnet_cfg_dtgskt_t *dtgskcfg 
                                         ) 
{
    if(NULL != s_eom_theipnet.ipnet) 
    {
        // already initialised
        return(&s_eom_theipnet);
    }
    

    if(NULL == ipnetcfg)
    {
        ipnetcfg = &eom_ipnet_DefaultCfg;
    }
   
    // trying to initialise with no ipcfg ?? error
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != ipcfg), s_eobj_ownname, "ipcfg is NULL");   

    // trying to initialise with wrong params error
    eo_errman_Assert(eo_errman_GetHandle(), (0 != ipnetcfg->procmaxidletime), s_eobj_ownname, "ipnetcfg->procmaxidletime is 0");
    eo_errman_Assert(eo_errman_GetHandle(), (0 != ipnetcfg->procstacksize), s_eobj_ownname, "ipnetcfg->procstacksize is 0");
    eo_errman_Assert(eo_errman_GetHandle(), (0 != ipnetcfg->procpriority), s_eobj_ownname, "ipnetcfg->procpriority is 0");
    
    
        
    // we can initialise w/out datagram sockets. we do so by using dtgskcfg  which contains zero and zero
    if(NULL == dtgskcfg)
    {
        dtgskcfg = &eom_ipnet_dtgskt_DefaultCfg;
    }

    // cannot have maxdatagramenqueuedintx equal to zero if we have non-zero numberofsockets
    eo_errman_Assert(eo_errman_GetHandle(), 
                     (0 != dtgskcfg->maxdatagramenqueuedintx) || (0 == dtgskcfg->numberofsockets), 
                     s_eobj_ownname, 
                     "must have non-zero maxdatagramenqueuedintx if numberofsockets is non-zero"); 


   
    
//    #warning the mutex mutexactivedgram is used to protect multiple access to the activedgramlist.
//    #warning but it may be not needed. the caller is unique (protected by another mutex) and the ipnet use the list only when the caller is waiting for its action
    // i get a basic ip net with proper osal mutex and attach and detach functions which are proper for rtos
    s_eom_theipnet.ipnet = eov_ipnet_hid_Initialise(dtgskcfg->numberofsockets, NULL, //eom_mutex_New(), 
                                                    s_eom_ipnet_AttachSocket, 
                                                    s_eom_ipnet_DetachSocket, 
                                                    s_eom_ipnet_Alert,
                                                    s_eom_ipnet_ARP,
                                                    s_eom_ipnet_WaitPacket
                                                    //s_eom_ipnet_GetTask // removed as it is not necessary anymore
                                                    );

                                             
    // i get cmd.semaphore, initted with zero tokens
    s_eom_theipnet.cmd.semaphore = osal_semaphore_new(_MAXTOKENS_SEM_CMD_, 0);
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != s_eom_theipnet.cmd.semaphore), s_eobj_ownname, "osal cannot give a sem");

    s_eom_theipnet.cmd.mtxcaller = osal_mutex_new();
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != s_eom_theipnet.cmd.mtxcaller), s_eobj_ownname, "osal cannot give a mtx");

    s_eom_theipnet.cmd.stoptmr = eo_timer_New();
    s_eom_theipnet.cmd.stopact = eo_action_New();


    // i get a messagequeue which will keep addresses of datagramsockets
    if(0 != dtgskcfg->maxdatagramenqueuedintx)
    {
        s_eom_theipnet.dgramsocketready2tx = osal_messagequeue_new(dtgskcfg->maxdatagramenqueuedintx);
        eo_errman_Assert(eo_errman_GetHandle(), (NULL != s_eom_theipnet.dgramsocketready2tx), s_eobj_ownname, "osal cannot give a mq");
    }
    else
    {
        s_eom_theipnet.dgramsocketready2tx = NULL;
    }

    // i init the ipconfig
    memcpy((uint8_t*)&s_eom_theipnet.ipcfg, (uint8_t*)ipcfg, sizeof(ipal_cfg_t)); 
     
    // change s_eom_theipnet.ipcfg to reflect changes passed as parameters, such as macaddr, ipaddr, ipmask
    if(NULL != addrcfg)
    {
        if(0x0000000000000000 != addrcfg->macaddr)
        {
            s_eom_theipnet.ipcfg.eth_mac = addrcfg->macaddr;
        }
        
        if(0x00000000 != addrcfg->ipaddr)
        {
            s_eom_theipnet.ipcfg.eth_ip = addrcfg->ipaddr;       
        }    
    
        if(0x00000000 != addrcfg->ipmask)
        {
            s_eom_theipnet.ipcfg.eth_mask = addrcfg->ipmask;
        } 
    }
    
    
    // complete other things    
    s_eom_theipnet.rxpacket     = eo_packet_New(0); // zero capacity .... because we want to assign data later on
    s_eom_theipnet.maxwaittime  = eok_reltimeINFINITE; //(eok_reltimeINFINITE == maxidle) ? (eok_reltimeINFINITE) : (4*maxidle); 

    
    s_eom_theipnet.taskwakeuponrxframe = ipnetcfg->procwakeuponrxframe;


    // and finally, i prepare the task able to process the ip net
    s_eom_theipnet.tskproc = eom_task_New(eom_mtask_EventDriven, ipnetcfg->procpriority, ipnetcfg->procstacksize,
                                          s_eom_ipnet_tskproc_startup, s_eom_ipnet_tskproc_forever,
                                          0, ipnetcfg->procmaxidletime,
                                          NULL,
                                          eom_ipnetproc, 
                                          "ipnet.proc");
                                              
    // and task which ticks the timers
    s_eom_theipnet.tsktick = eom_task_New(eom_mtask_Periodic, ipnetcfg->tickpriority, ipnetcfg->tickstacksize,
                                          NULL, s_eom_ipnet_tsktick_forever,
                                          0, s_eom_theipnet.ipcfg.sys_timetick,
                                          NULL, 
                                          eom_ipnettick,
                                          "ipnet.tick");
   
    return(&s_eom_theipnet);
}    

    
extern EOMtheIPnet* eom_ipnet_GetHandle(void) 
{
    if(NULL == s_eom_theipnet.ipnet) 
    {
        return(NULL);
    }
    
    return(&s_eom_theipnet);
}


extern eOresult_t eom_ipnet_ResolveIP(EOMtheIPnet *ip, eOipv4addr_t ipaddr, eOreltime_t tout)
{
    if(NULL == ip)
    {
        return(eores_NOK_nullpointer);
    }

    return(s_eom_ipnet_ARP(ip->ipnet, ipaddr, tout));
}


extern eOresult_t eom_ipnet_IGMPgroupJoin(EOMtheIPnet *ip, eOipv4addr_t igmp)
{
    if(NULL == ip)
    {
        return(eores_NOK_nullpointer);
    }

    return(eov_ipnet_IGMPgroupJoin(ip->ipnet, igmp));
}


extern eOresult_t eom_ipnet_IGMPgroupLeave(EOMtheIPnet *ip, eOipv4addr_t igmp)
{
    if(NULL == ip)
    {
        return(eores_NOK_nullpointer);
    }

    return(eov_ipnet_IGMPgroupLeave(ip->ipnet, igmp));
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

/*  @brief      called by an external task, via eom_ipnet_ResolveIP(), to resolve IP address
    @details    it sends a command to the tskproc, which sends several ARP requests until
                it receives a reply or until a maximum number of requests are sent or a maximum timeout
                is reached.
 **/
static eOresult_t s_eom_ipnet_ARP(EOVtheIPnet *ip, eOipv4addr_t ipaddr, eOreltime_t tout)
{
    eOresult_t res = eores_NOK_generic;

    static const uint16_t maxarptx = 65000;
    
    if(NULL == ip) 
    {
        return(eores_NOK_nullpointer);    
    } 

    // verify we dont have a zero ip address
    if(0x00000000 == ipaddr)
    {
        return(eores_NOK_generic);
    } 

    // - block other possible tasks from sending a command to the ipnet and also to execute this function from here onward
    osal_mutex_take(s_eom_theipnet.cmd.mtxcaller, osal_reltimeINFINITE);
    
    // - build the command to be sent to the proc task

    // copy the wanted command: doarp
    s_eom_theipnet.cmd.par16b = maxarptx;
    s_eom_theipnet.cmd.par32b = ipaddr;
    s_eom_theipnet.cmd.result  = 0;
    s_eom_theipnet.cmd.tout  = tout;
    s_eom_theipnet.cmd.opcode = cmdDoARP;

 
    // - tell the task that there is a command to process, so that a switch context is forced.
    s_eom_ipnet_Alert(s_eom_theipnet.ipnet, &s_eom_theipnet, EOK_ipnet_evt_CMD2process);

    // - now the calling task is placed in hold until this semaphore is incremented  
    // - by the ipnet task when it has finished executing the command or its time has expired
    osal_semaphore_decrement(s_eom_theipnet.cmd.semaphore, osal_reltimeINFINITE);

    // - verify operation success
    if(1 == s_eom_theipnet.cmd.result)
    {
        res = eores_OK;
    }
    else
    {
        res = eores_NOK_timeout;
    }

    // - enable again another task to send a command to the ipnet
    osal_mutex_release(s_eom_theipnet.cmd.mtxcaller);

    return(res);
}

/*  @brief      called by an external task, via some methods of a socket, to add a socket to the management of the ipnet
    @details    it sends a command to the tskproc, which adds the sockets and then unblocks the caller.
 **/
static eOresult_t s_eom_ipnet_AttachSocket(EOVtheIPnet* ip, EOsocketDerived *s) 
{
    eOresult_t res = eores_NOK_generic;
    eObool_t canattach = eobool_false;;
    eOsocketType_t t;
    
    if((NULL == ip) || (NULL == s)) 
    {
        return(eores_NOK_nullpointer);    
    }

    t = eo_socket_hid_derived_Get_Type(s);
    
    // for now i only do datagrams  ...
    eo_errman_Assert(eo_errman_GetHandle(), (eo_skttyp_datagram == t), s_eobj_ownname, "only datagram sockets ...");


    // - block other possible tasks from sending a command to the ipnet and also to execute this function from here onward
    osal_mutex_take(s_eom_theipnet.cmd.mtxcaller, osal_reltimeINFINITE);


    // - fill the command and also verify if the socket can be attached
    canattach = s_eom_ipnet_attach_rqst_dtgsocket(ip, (EOsocketDatagram*)s);
 

    // - if the socket can be attached, send a command to ipnet main task and wait for the end of command
    if(eobool_true == canattach)
    {   
        // - tell the task that there is a command to process, so that a switch context is forced.
        s_eom_ipnet_Alert(s_eom_theipnet.ipnet, &s_eom_theipnet, EOK_ipnet_evt_CMD2process);

        // - now the calling task is placed in hold until this semaphore is incremented 
        // - by the ipnet task when it has finished executing the command or its time has expired
        osal_semaphore_decrement(s_eom_theipnet.cmd.semaphore, osal_reltimeINFINITE);

        // verify the success of the operation
        if(1 == s_eom_theipnet.cmd.result)
        {
            res = eores_OK;
        }
        else
        {
            res = eores_NOK_timeout;
        }
    }


    // - enable again another task to send a command to the ipnet
    osal_mutex_release(s_eom_theipnet.cmd.mtxcaller);

    return(res);    
}


/*  @brief      called by an external task, via some methods of a socket, to remove a socket from the management of the ipnet
    @details    it sends a command to the tskproc, which removes the sockets and then unblocks the caller.
 **/
static eOresult_t s_eom_ipnet_DetachSocket(EOVtheIPnet* ip, EOsocketDerived *s) 
{
    eOresult_t res = eores_NOK_generic;
    eObool_t candetach = eobool_false;;
    eOsocketType_t t;
    
    if((NULL == ip) || (NULL == s)) 
    {
        return(eores_NOK_nullpointer);    
    }

    t = eo_socket_hid_derived_Get_Type(s);
    
    // for now i only do datagrams  ...
    eo_errman_Assert(eo_errman_GetHandle(), (eo_skttyp_datagram == t), s_eobj_ownname, "only datagram sockets ...");


    // - block other possible tasks from sending a command to the ipnet and also to execute this function from here onward
    osal_mutex_take(s_eom_theipnet.cmd.mtxcaller, osal_reltimeINFINITE);


    // - fill the command and also verify if the socket can be detached
    candetach = s_eom_ipnet_detach_rqst_dtgsocket(ip, (EOsocketDatagram*)s);
    
    
    // - if the socket can be detached, send a command to ipnet main task and wait for the end of command
    if(eobool_true == candetach)
    {   
        // - tell the task that there is a command to process, so that a switch context is forced.
        s_eom_ipnet_Alert(s_eom_theipnet.ipnet, &s_eom_theipnet, EOK_ipnet_evt_CMD2process);

        // - now the calling task is placed in hold until this semaphore is incremented 
        // - by the ipnet task when it has finished executing the command or its time has expired
        osal_semaphore_decrement(s_eom_theipnet.cmd.semaphore, osal_reltimeINFINITE);

        // verify the success of the operation
        if(1 == s_eom_theipnet.cmd.result)
        {
            res = eores_OK;
        }
        else
        {
            res = eores_NOK_timeout;
        }

    }    


    // - enable again another task to send a command to the ipnet
    osal_mutex_release(s_eom_theipnet.cmd.mtxcaller);

    return(res);    
}

/*  @brief      sends an event to the tskproc
 **/
static eOresult_t s_eom_ipnet_Alert(EOVtheIPnet* ip, void *eobjcaller, eOevent_t evt) 
{
   
    if(NULL == ip) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    // - simply send an event to the ip task. if the caller is an isr use the correct method. if a socket ...

    switch(evt)
    {
        case EOK_ipnet_evt_RXipframe:
        {
            // caller is the eth isr
            eom_task_isrSetEvent(s_eom_theipnet.tskproc, EOK_ipnet_evt_RXipframe);
        } break;

        case EOK_ipnet_evt_TXdatagram:
        {
            // caller is a socket object which ... wants to tx a datagram packet. we need to put inside a fifo the reference to the socket
            // contained in eobjcaller
            if(osal_res_OK == osal_messagequeue_put(s_eom_theipnet.dgramsocketready2tx, (osal_message_t)eobjcaller, osal_reltimeINFINITE, osal_callerTSK))
            {
                // send event
                eom_task_SetEvent(s_eom_theipnet.tskproc, EOK_ipnet_evt_TXdatagram);
            }

        } break;

        default:
        {
            // caller is ... any other object from inside a task
            eom_task_SetEvent(s_eom_theipnet.tskproc, evt);
        } break;
    }

    return(eores_OK);    
}


/*  @brief      called by a socket configured in blocking mode when it attempts to retrieve a packet
    @details    the base socket only has a void* variable (blkgethandle) because it is osal-agnostic
                and does not know about osal_semaphore_t type.    
 **/
static eOresult_t s_eom_ipnet_WaitPacket(EOVtheIPnet* ip, EOsocketDerived *s, eOreltime_t tout) 
{
    eOresult_t res = eores_NOK_generic;
    EOsocket *bs = (EOsocket*)eo_common_getbaseobject(s);
    eOsocketType_t t;
    
    if((NULL == ip) || (NULL == s)) 
    {
        return(eores_NOK_nullpointer);    
    }

    t = eo_socket_hid_derived_Get_Type(s);

    
    // for now i only do datagrams and synchros ...
    eo_errman_Assert(eo_errman_GetHandle(), (eo_skttyp_datagram == t), s_eobj_ownname, "no stream sockets ...");

    
    // - waits in here but only if the semaphore really exists
    // - the semaphore is incremented by one each time the ipnet received a packet for this particular socket.
    if(NULL != bs->blkgethandle)
    {
        if(osal_res_OK == osal_semaphore_decrement(bs->blkgethandle, tout))
        {
            res = eores_OK;
        }
        else
        {
            res = eores_NOK_timeout;
        }
    }

    return(res);    
}


///*  @brief      used by eov_ipnet_GetTask() to give a pointer to the EOMtaskDerived object which runs teh ipnet.
//    @details    so far it is used by teh synchro-socket to send an event to the task ...     
// **/
//static EOVtaskDerived * s_eom_ipnet_GetTask(EOVtheIPnet* ip)
//{
//    if(NULL == ip) 
//    {
//        return(NULL);    
//    }
//
//    return(s_eom_theipnet.tskproc);
//}


// name of the task as it is shown in uvision
void eom_ipnetproc(void *p)
{
    eom_task_Start(p);
}

// name of the task as it is shown in uvision
void eom_ipnettick(void *p)
{
     eom_task_Start(p);
}



static void e_eom_ipnet_signal_new_frame_is_available(void)
{
    // the presence of this wakeup event which the ethernet isr sends 
    // to the task whcih process the tcp/ip stack improved response time of
    // ping from values around 200-300 ms or more down to values always <1ms.
    // the test is done on the iit intranet.

    eom_task_isrSetEvent(s_eom_theipnet.tskproc, EOK_ipnet_evt_RXipframe);   
}


/*  @brief      the body of tsktick.
 **/
static void s_eom_ipnet_tsktick_forever(EOMtask *rt, uint32_t n)
{
    ipal_sys_timetick_increment();
}



/*  @brief      the startup of tsktproc.
    @details    in here we initialise and start the ipal, 
 **/
static void s_eom_ipnet_tskproc_startup(EOMtask *rt, uint32_t n)
{
    uint32_t ram32sizeip;
    uint32_t *ram32dataip = NULL;

    // initialise the ipal
    ipal_base_memory_getsize(&s_eom_theipnet.ipcfg, &ram32sizeip);
    if(0 != ram32sizeip)
    {
        ram32dataip = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, ram32sizeip, 1);
    }

    if(eobool_true == s_eom_theipnet.taskwakeuponrxframe)
    {
        s_eom_theipnet.ipcfg.extfn.usr_on_ethframe_received = e_eom_ipnet_signal_new_frame_is_available;
    }
    else
    {
        s_eom_theipnet.ipcfg.extfn.usr_on_ethframe_received = NULL;
    }

    ipal_base_initialise(&s_eom_theipnet.ipcfg, ram32dataip);
    
    // start the ipal
    ipal_sys_start();

}

/*  @brief      the body of tsktproc.
    @details    in here we: (a) process any incoming packet from the network, (b) transmit any packet of a synchro socket
                whose time has come  because the timer of the synsock has sent an event, (c) transmit any packet from datagrams
                with something in the output queues, (d) process a command request, (e) process a command stop, and (f) process
                a repetition of a command.                
 **/
static void s_eom_ipnet_tskproc_forever(EOMtask *rt, uint32_t evtmsk)
{
    // evtmask contains the events which the task has just received.

        
    // - (a) process the tcp/ip stack.
    
    ipal_sys_process_communication();



//    if(EOK_ipnet_evt_RXipframe == (evtmsk & EOK_ipnet_evt_RXipframe))
//    {   // the ethernet isr has just received and processed an ethernet frame
//
//        // i could maybe return if i have this event and only this ????  no: i go on  
//    }


    // - (b) process transmission of a synchronised packet
    
//    if((0 != s_eom_theipnet.ipnet->synchsock_txmask) && 0 != (evtmsk & EOK_ipnet_evt_TXsynchroMASK))
//    {
//        s_eom_ipnet_process_transmission_synchro(evtmsk & EOK_ipnet_evt_TXsynchroMASK);
//    }


    // - (c) process transmission of a datagram packet
    
    // if we keep the tx of datagrams in here and before of teh processing of commands, then we
    // allow the following mechanism: transmit one or mor packets (by a very high priority task)
    // and then ... the same task closes teh socket .... well: the IPnet before transmit every packet
    // in the fifooutput and then closes the socket.
    if(EOK_ipnet_evt_TXdatagram == (evtmsk & EOK_ipnet_evt_TXdatagram))
    {
        s_eom_ipnet_process_transmission_datagram();
    }

    
    // - (d) process request of a command
    
    if(EOK_ipnet_evt_CMD2process == (evtmsk & EOK_ipnet_evt_CMD2process))
    {
        s_eom_ipnet_process_command();
    }

    
    // - (e) process termination of processing of a command (the termination is internally issued)
    
    if(EOK_ipnet_evt_CMD2stop == (evtmsk & EOK_ipnet_evt_CMD2stop))
    {
        s_eom_ipnet_stop_command();
    }
    
    
    // - (e) process repetion of a command (the repetition is internally issued)
    
    if(1 == s_eom_theipnet.cmd.repeatcmd)
    {
        s_eom_ipnet_repeat_command();
    }

}

#ifdef SLOWERMODEONRXDATAGRAM
static eOresult_t s_eom_ipnet_DatagramSocketHas(void *item, void *param)
{
    EOsocketDatagram* s   = *((EOsocketDatagram**)item);
    ipal_udpsocket_t* skt = (ipal_udpsocket_t*)param;

    if(skt == eo_socket_hid_derived_Get_Handle(s))
    {
        return(eores_OK);
    }

    return(eores_NOK_generic);
}
#endif


/*  @brief      called by ipal_sys_process_communication() on reception of a datagram socket
    @details    the ipal calls a function with this parameters. we search for the correct EOsocketDatagram and then
                we link the received data inside a temporary packet which is then copied inside the input queue of
                the datagram socket.
                finally we alert the caller or unblock its wait                
 **/
static void s_eom_ipnet_OnReceptionDatagram(void *arg, ipal_udpsocket_t *skt, ipal_packet_t *pkt, ipal_ipv4addr_t adr, ipal_port_t por)
{
    EOsocketDatagram *dtgskt = NULL;
    volatile eOresult_t res = eores_NOK_timeout;
    static uint32_t fails = 0;
   

#ifdef SLOWERMODEONRXDATAGRAM
    EOlistIter *li = NULL;
    // we are sure that the s_eom_theipnet.ipnet->activedgramsocksptrlist is non NULL because we setup this callback
    // only if we have support for the datagramsockets, hence we have this list.

    // find the socket with descriptor soc inside the list s_eom_theipnet.ipnet->activedgramsocksptrlist
    // the function s_eom_ipnet_DatagramSocketHas() returns eores_OK and thus the search is ok when an item
    // of the list (a pointer to a EOsocketDatagram) has a ->socket->skthandle which is uqual to skt

    li = eo_list_Find(s_eom_theipnet.ipnet->activedgramsocksptrlist, s_eom_ipnet_DatagramSocketHas, skt);

    if(NULL == li)
    {   // did not find the socket ... quit
        return;
    }

    // ok ... dtgskt is the datagram socket
    dtgskt = *(EOsocketDatagram**)eo_list_At(s_eom_theipnet.ipnet->activedgramsocksptrlist, li);
#else
    // as specified by ipal_udpsocket_recv(), the variable arg keeps the socketdatagram  
    dtgskt = (EOsocketDatagram *)arg;
#endif

    if(eo_sktdir_TXonly == dtgskt->socket->dir)
    {
        // it is transmitting-only socket ... quit
        return;
    }

            
    // build a packet, s_eom_theipnet.rxpacket, with rx payload, its size, address and port
    // and then use it to copy info into the input queue.
    
    // for now, i use the uint32_t in the simplest way: as a holder for a uint8_t addr[4].
    eo_packet_Full_LinkTo(s_eom_theipnet.rxpacket, adr, por, pkt->size, pkt->data);
   
    res = eo_fifo_Put(dtgskt->dgramfifoinput, s_eom_theipnet.rxpacket, s_eom_theipnet.maxwaittime);

    if(eores_OK != res)
    {
        fails ++;
        eo_errman_Error(eo_errman_GetHandle(), eo_errortype_warning, s_eobj_ownname, "cannot put a datagram in rx fifo");
        // return because ... we did not put the message in teh queue and thus ... we dont want do any action on reception
        return;
    }


    // do registered action on reception
    eo_action_Execute(dtgskt->socket->onreception, eok_reltimeZERO);

    if(eobool_true == dtgskt->socket->block2wait4packet)
    {
        // unblock the reception
        osal_semaphore_increment(dtgskt->socket->blkgethandle);
    }

  

    return;
}




static void s_eom_ipnet_process_command(void)
{
//    EOsocket *s = NULL;
    EOsocketDerived *sdrv = NULL;
    eOsizecntnr_t size = 0;


    switch(s_eom_theipnet.cmd.opcode)
    {
        case cmdDoARP:
        {
            // par16b: max number of arp packets sent
            // par32b: ip address
            // in here we are sure that we have valid params (non-zero so far)
            if(0 != s_eom_theipnet.cmd.par16b)
            { 
                s_eom_theipnet.cmd.par16b --;
                s_eom_theipnet.cmd.repeatcmd = 1;

                if(ipal_res_OK == ipal_arp_request(s_eom_theipnet.cmd.par32b, ipal_arp_cache_permanently))
                { 
                    // ok at the first time .....  
                    s_eom_theipnet.cmd.result = 1;
                    s_eom_theipnet.cmd.par16b = 0;
                    s_eom_theipnet.cmd.repeatcmd = 0;
                    s_eom_theipnet.cmd.tout = 0;
                    // release the caller
                    osal_semaphore_increment(s_eom_theipnet.cmd.semaphore);
                }
                else if(0 != s_eom_theipnet.cmd.tout)
                {
                    // unlucky. retry with a timeout
                    // start a one-shot timer with tout and which sends an event of stop to this task
                    eo_action_SetEvent(s_eom_theipnet.cmd.stopact, EOK_ipnet_evt_CMD2stop, s_eom_theipnet.tskproc);
                    eo_timer_Start(s_eom_theipnet.cmd.stoptmr, eok_abstimeNOW, s_eom_theipnet.cmd.tout, eo_tmrmode_ONESHOT, s_eom_theipnet.cmd.stopact);
                }

            }

 
        } break;

        case cmdAttachDTG:
        {
            // get the derived socket
            sdrv = (((void*) s_eom_theipnet.cmd.par32b)); 
            
            // initialise the base socket, commit the derived socket in proper data stratcure of ipnet
            s_eom_ipnet_attach_proc_dtgsocket((EOsocketDatagram*)sdrv);

            // increment the semaphore to allow execution of the caller
            osal_semaphore_increment(s_eom_theipnet.cmd.semaphore); 

        } break;

        case cmdDetachDTG:
        {
 
            // get the derived socket
            sdrv = (((void*) s_eom_theipnet.cmd.par32b)); 

            // initialise the base socket, commit the derived socket in proper data stratcure of ipnet

            // if we are asked to close a datagram socket which has some more packets to transmit, then
            // we decide to empty the fifo first. we do that by sending a EOK_ipnet_evt_TXdatagram
            // event but also a EOK_ipnet_evt_CMD2process to get here inside again. 

            if(eores_OK == eo_fifo_Size(((EOsocketDatagram*)sdrv)->dgramfifooutput, &size, s_eom_theipnet.maxwaittime))
            {
                if(0 != size)
                {
                    // sends an event of kind tx-pkt and cmd
                    eom_task_SetEvent(s_eom_theipnet.tskproc, EOK_ipnet_evt_TXdatagram | EOK_ipnet_evt_CMD2process);
                    // and then return ....
                    return;
                }
            }

            // if we have not returned, then we dont have any more datagram to send, thus ... detach the datagram socket
            s_eom_ipnet_detach_proc_dtgsocket((EOsocketDatagram*)sdrv);


            // increment the semaphore to allow execution of the caller
            osal_semaphore_increment(s_eom_theipnet.cmd.semaphore); 
                              
        } break;


        default:
        {
        } break;

    }
}

static void s_eom_ipnet_repeat_command(void)
{

    switch(s_eom_theipnet.cmd.opcode)
    {
        case cmdDoARP:
        {
            // par16b: max number of arp packets sent
            // par32b: ip address
            // in here we are sure that we have valid params (non-zero so far)
            if(0 != s_eom_theipnet.cmd.par16b)
            { 
                if(0 == s_eom_theipnet.cmd.tout)
                {
                    // with this control, the exit is done not with par16 being decremented but with timeout expiry
                    s_eom_theipnet.cmd.par16b --;
                }
                s_eom_theipnet.cmd.repeatcmd = 1;

                if(ipal_res_OK == ipal_arp_request(s_eom_theipnet.cmd.par32b, ipal_arp_cache_permanently))
                { 
                    // ok at the first time .....  
                    s_eom_theipnet.cmd.result = 1;
                    s_eom_theipnet.cmd.par16b = 0;
                    s_eom_theipnet.cmd.repeatcmd = 0;
                    s_eom_theipnet.cmd.tout = 0;
                    // release the timer (if any)
                    if(0 != s_eom_theipnet.cmd.tout)
                    {
                        eo_timer_Stop(s_eom_theipnet.cmd.stoptmr);
                        s_eom_theipnet.cmd.tout = 0;
                    }
                    // release the caller
                    osal_semaphore_increment(s_eom_theipnet.cmd.semaphore);
                }

            }
            else
            {
                // reset command, give result ko, clear params, release semaphore
                s_eom_theipnet.cmd.result = 0;
                s_eom_theipnet.cmd.par16b = 0;
                s_eom_theipnet.cmd.opcode = cmdDoNONE;
                s_eom_theipnet.cmd.par32b = 0;
                s_eom_theipnet.cmd.repeatcmd = 0;
 
                // release the timer (if any)
                if(0 != s_eom_theipnet.cmd.tout)
                {
                    eo_timer_Stop(s_eom_theipnet.cmd.stoptmr);
                    s_eom_theipnet.cmd.tout = 0;
                }
                // release the caller
                osal_semaphore_increment(s_eom_theipnet.cmd.semaphore);                
            } 
 
        } break;


        default:
        {
            s_eom_theipnet.cmd.repeatcmd = 0;
        } break;

    }
}

static void s_eom_ipnet_stop_command(void)
{
    s_eom_theipnet.cmd.par16b = 0;
    s_eom_ipnet_repeat_command();
}

static void s_eom_ipnet_process_transmission_datagram(void)
{
    EOsocketDatagram *s = NULL;
    eOresult_t res;
    const void *vitem = NULL;
    EOpacket *ditem = NULL;
    ipal_packet_t ipalpkt;

#if 0
// we can use a getrem only if we have a pre-allocated and big enough packet where to copy
    static EOpacket *txpkt = NULL;
    txpkt = eo_packet_New(bignumber); // bif number depends on the socket.
#endif
    // get the socket which has something to transmit
    s = (EOsocketDatagram*) osal_messagequeue_getquick(s_eom_theipnet.dgramsocketready2tx, osal_reltimeINFINITE, osal_callerTSK);

    if(NULL == s)
    {
        return;
    }
        
    // do action depending on status of the socket
    switch(s->socket->status)
    {
        case STATUS_SOCK_OPENED:
        {
            // get dgram from output queue. 
            // VERY IMPORTANT: i use eok_reltimeZERO because it may be that we have a periodic
            // transmission of teh socket and the user does not have a packet to transmit, thus that fifooutput is empty.
            // well: we dont want to block in there


            res = eo_fifo_Get(s->dgramfifooutput, &vitem, eok_reltimeZERO);
            ditem = ((EOpacket *)vitem);
            // if i use a quicker get-rem ... uncomments the following two line and comment the eo_fifo_Rem()
            //res = eo_fifo_GetRem(s->dgramfifooutput, txpkt, eok_reltimeZERO);
            //ditem = ((EOpacket *)txpkt);
            if(eores_OK == res) 
            {   // transmit the datagram
                ipalpkt.data = ditem->data;
                ipalpkt.size = ditem->size;
                if(ipal_res_OK == ipal_udpsocket_sendto(s->socket->skthandle, &ipalpkt, ditem->remoteaddr, ditem->remoteport))
                {
                    // remove the datagram being transmitted
                    eo_fifo_Rem(s->dgramfifooutput, s_eom_theipnet.maxwaittime);

                    // do action on tx-done
                    if((NULL != s->socket->ontransmission) && (eo_actypeNONE != eo_action_GetType(s->socket->ontransmission)))
                    {
                        eo_action_Execute(s->socket->ontransmission, eok_reltimeZERO);
                    }
                }
                else
                {
                     // remove the datagram being transmitted also if you fail to tx it.
                    eo_fifo_Rem(s->dgramfifooutput, s_eom_theipnet.maxwaittime);
                    // but put a warning on it.
                    eo_errman_Error(eo_errman_GetHandle(), eo_errortype_warning, s_eobj_ownname, "ipal cannot send a datagram");

                }
            }

            // if message queue is not empty, then sends another tx event because there is another socket which needs transmission
            if(0 != osal_messagequeue_size(s_eom_theipnet.dgramsocketready2tx, osal_callerTSK))
            {
                eom_task_SetEvent(s_eom_theipnet.tskproc, EOK_ipnet_evt_TXdatagram);
            } 
            
        } break;
            
        default:
        {
        } break;            
        
    }
        

}




/*  @brief      this function is executed by teh calling task.
                if the socket can be attached: completes the OSAL-related initialisation of socket, 
                fills the command to be sent to tskproc with opcode = cmdAttachDTG and returns eobool_true. 
 **/
static eObool_t s_eom_ipnet_attach_rqst_dtgsocket(EOVtheIPnet *vip, EOsocketDatagram *dgmskt)
{

    EOlist *socklist = vip->activedgramsocksptrlist;

    if(NULL == socklist)
    {
        // teh ipnet was initted without datagram sockets ... return false
        return(eobool_false);
    }
    
    // block socklist ... no need
    //eom_mutex_Take(s_eom_theipnet.ipnet->mutexactivedgram, eok_reltimeINFINITE);

    // if list is already full then return false, otherwise go on with building up a command.
    if(eobool_true == eo_list_Full(socklist)) 
    {
        // unblock ... no need
        //eom_mutex_Release(s_eom_theipnet.ipnet->mutexactivedgram);
        return(eobool_false);
    }

    // unblock socklist ... no need
    //eom_mutex_Release(s_eom_theipnet.ipnet->mutexactivedgram);
    
    // create the semaphore for the socket if we have blocking mode and the semaphore is still NULL.
    // we init the sempahore with maxtokens equal to the capacity of the input fifo + 1.
    if((NULL == dgmskt->socket->blkgethandle) && (eobool_true == dgmskt->socket->block2wait4packet))
    {
        eOsizecntnr_t maxtokens;
        eo_fifo_Capacity(dgmskt->dgramfifoinput, &maxtokens, eok_reltimeINFINITE);
        dgmskt->socket->blkgethandle = osal_semaphore_new(maxtokens+1, 0);
        eo_errman_Assert(eo_errman_GetHandle(), (NULL != dgmskt->socket->blkgethandle), s_eobj_ownname, "no osal blocking semaphore ...");
    }    
    
    // build the command
    s_eom_theipnet.cmd.par32b = (uint32_t)dgmskt;
    s_eom_theipnet.cmd.tout  = 0;
    s_eom_theipnet.cmd.opcode = cmdAttachDTG;
 
  
    return(eobool_true);  
}


/*  @brief      this function is executed by taskproc at reception of a command with opcode = cmdAttachDTG. 
                it just create the ipal udp socket and store it inside the skthandle
 **/
static void s_eom_ipnet_attach_proc_dtgsocket(EOsocketDatagram *dtgs)
{
    const ipal_tos_t tos = {.precedence = ipal_prec_priority, .lowdelay = 1, .highthroughput = 1, .highreliability = 1, .unused = 0};
    
    EOsocket *s = eo_common_getbaseobject(dtgs);
 
    // reset the command
    s_eom_theipnet.cmd.result = 0;
    s_eom_theipnet.cmd.opcode = cmdDoNONE;
    s_eom_theipnet.cmd.par32b = 0; 

    // create the ipal socket, bind it, set the callback on receive
    s->skthandle = ipal_udpsocket_new(tos);

    if(NULL == s->skthandle)
    {
        return;
    }

    if(ipal_res_OK != ipal_udpsocket_bind(s->skthandle, IPAL_ipv4addr_INADDR_ANY, s->localport))
    {
        return;
    }

    if(ipal_res_OK != ipal_udpsocket_recv(s->skthandle, s_eom_ipnet_OnReceptionDatagram, dtgs))
    {
        return;
    }


    // we are ok ... set other things
    s->status = STATUS_SOCK_OPENED;
    // take the list of active datagrams
    //eom_mutex_Take(s_eom_theipnet.ipnet->mutexactivedgram, eok_reltimeINFINITE); 

    // add the pointer of the datagram socket inside the list of active datagrams managed by the ipnet
    eo_list_PushFront(s_eom_theipnet.ipnet->activedgramsocksptrlist, &dtgs);

    // release it
    //eom_mutex_Release(s_eom_theipnet.ipnet->mutexactivedgram); 
    
    s_eom_theipnet.cmd.result = 1;
}


/*  @brief      this function is executed by the calling task.
                if the socket can be detached: fills the command to be sent to tskproc with opcode = cmdDetachDTG 
                and returns eobool_true. 
 **/
static eObool_t s_eom_ipnet_detach_rqst_dtgsocket(EOVtheIPnet *vip, EOsocketDatagram *dgmskt)
{
    // returns eobool_true if the socket can be detached

    EOlist *socklist = vip->activedgramsocksptrlist;

    if(NULL == socklist)
    {
        // the ipnet was initted without datagram sockets ... return false
        return(eobool_false);
    }

    
    // block socklist ... no need
    //eom_mutex_Take(s_eom_theipnet.ipnet->mutexactivedgram, eok_reltimeINFINITE);

    // if cannot find the socket in list of active datagram managed by the ipnet then return false, 
    // otherwise go on with building up a command.
    if(NULL == eo_list_FindItem(socklist, &dgmskt)) 
    {
        // unblock ... no need
        //eom_mutex_Release(s_eom_theipnet.ipnet->mutexactivedgram);
        return(eobool_false);
    }

    // unblock socklist ... no need
    //eom_mutex_Release(s_eom_theipnet.ipnet->mutexactivedgram);
    
    // build the command
    s_eom_theipnet.cmd.par32b = (uint32_t)dgmskt;
    s_eom_theipnet.cmd.tout  = 0;
    s_eom_theipnet.cmd.opcode = cmdDetachDTG;
  
    return(eobool_true);  
}


/*  @brief      this function is executed by taskproc at reception of a command with opcode = cmdDetachDTG. 
                ...
 **/
static void s_eom_ipnet_detach_proc_dtgsocket(EOsocketDatagram *dtgs)
{
    EOlist *socks = s_eom_theipnet.ipnet->activedgramsocksptrlist;
    
    EOsocket *s = eo_common_getbaseobject(dtgs);

    // reset the command
    s_eom_theipnet.cmd.result = 0;
    s_eom_theipnet.cmd.opcode = cmdDoNONE;
    s_eom_theipnet.cmd.par32b = 0; 


    // close and delete the osal socket
    ipal_udpsocket_delete(s->skthandle);

    // clear data structure
    s->skthandle = NULL;
    s->status = STATUS_SOCK_NONE;

    // remove the datagram socket from list
    eo_list_Erase(socks, eo_list_FindItem(socks, &dtgs));  

    // result is ok now
    s_eom_theipnet.cmd.result = 1;
}







// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------





