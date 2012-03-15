
/* @file       EOsocketDatagram.c
    @brief      This file implements internal implementation of a datagram socket object.
    @author     marco.accame@iit.it
    @date       08/25/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"
#include "EOVtheCallbackManager.h"

#include "EOVtheIPnet.h"
#include "EOsocket_hid.h"
#include "EOpacket.h"
#include "EOpacket_hid.h"

/*
see:
1. http://en.wikipedia.org/wiki/Internet_socket
2. http://opengroup.org/onlinepubs/007908799/xns/connect.html
3. http://opengroup.org/onlinepubs/007908799/xns/syssocket.h.html
4. http://www.opengroup.org/onlinepubs/000095399/basedefs/sys/socket.h.html

http://java.sun.com/javase/6/docs/api/java/net/Socket.html
http://java.sun.com/javase/6/docs/api/java/net/DatagramSocket.html

http://technet.microsoft.com/en-us/library/bb463220.aspx
*/


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOsocketDatagram.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOsocketDatagram_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------

const eOsktdtgTXmode_t eo_sktdtg_TXnow = { .startat = EOK_abstimeNOW, .after = 0, .periodic = eobool_false};



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_eo_socketdtg_txcallback(void *arg);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOsocketDatagram";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOsocketDatagram* eo_socketdtg_New(uint8_t dtg_in_num, uint16_t dtg_in_size, EOVmutexDerived *mtx_fifo_in,
                                          uint8_t dtg_out_num, uint16_t dtg_out_size, EOVmutexDerived *mtx_fifo_out)
{
    EOsocketDatagram *retptr = NULL;    

    // i get the memory for the object
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOsocketDatagram), 1);

    eo_errman_Assert(eo_errman_GetHandle(), (0 != (dtg_in_num+dtg_out_num)), s_eobj_ownname, "at least one pkt queue");

    // get the base object
    retptr->socket = eo_socket_New();

    // now the obj has valid memory. i need to initialise it
    retptr->dgramfifoinput      = (0 == dtg_in_num) ? (NULL) :  eo_fifo_New(sizeof(EOpacket), dtg_in_num, 
                                                                                eo_packet_hid_DefInit, dtg_in_size, 
                                                                                eo_packet_hid_DefCopy, eo_packet_hid_DefClear, mtx_fifo_in);

    retptr->dgramfifooutput     = (0 == dtg_out_num) ? (NULL) : eo_fifo_New(sizeof(EOpacket), dtg_out_num, 
                                                                                eo_packet_hid_DefInit, dtg_out_size,
                                                                                eo_packet_hid_DefCopy, eo_packet_hid_DefClear, mtx_fifo_out);

    retptr->toutfifos           = eok_reltimeINFINITE;
    
    retptr->txtimer             = eo_timer_New();

    memcpy(&retptr->txmode, &eo_sktdtg_TXnow, sizeof(eOsktdtgTXmode_t));

 
    retptr->actiontx            = eo_action_New();
    eo_action_SetCallback(retptr->actiontx, s_eo_socketdtg_txcallback, retptr, eov_callbackman_GetTask(eov_callbackman_GetHandle()));

    return(retptr);

}


extern eOresult_t eo_socketdtg_Open(EOsocketDatagram *p, eOipv4port_t localport, eOsocketDirection_t dir, eObool_t block2wait4packet, eOsktdtgTXmode_t *txmode, EOaction *onrx, EOaction *ontx)
{

    eOresult_t res = eores_NOK_generic;

    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }


    if(((dir == eo_sktdir_TXonly) ||  (dir == eo_sktdir_TXRX)) && (NULL == p->dgramfifooutput))
    {
        eo_errman_Assert(eo_errman_GetHandle(), 0, s_eobj_ownname, "eo_socketdtg_Open() needs an output queue");
    }

    if(((dir == eo_sktdir_RXonly) ||  (dir == eo_sktdir_TXRX)) && (NULL == p->dgramfifoinput))
    {
        eo_errman_Assert(eo_errman_GetHandle(), 0, s_eobj_ownname, "eo_socketdtg_Open() needs an input queue");
    }
    
    if((NULL == p->dgramfifoinput) && (eobool_true == block2wait4packet))
    {
        eo_errman_Assert(eo_errman_GetHandle(), 0, s_eobj_ownname, "eo_socketdtg_Open(): socket cannot be in blocking mode if tx only");
    }
    
    if(NULL != txmode)
    {
        // if periodic must be non-zero
        memcpy(&p->txmode, txmode, sizeof(eOsktdtgTXmode_t));
    }
    else
    {
        memcpy(&p->txmode, &eo_sktdtg_TXnow, sizeof(eOsktdtgTXmode_t));
    }
    
    // open only if we had not opened it before
    if(STATUS_SOCK_NONE == p->socket->status)
    {
        eo_socket_hid_derived_Prepare(p, eo_skttyp_datagram, localport, onrx, ontx, dir, block2wait4packet);
        res = eov_ipnet_AttachSocket(eov_ipnet_GetHandle(), p);
        
        if(eores_OK == res)
        {
            if(eobool_true == p->txmode.periodic)
            {
                eo_timer_Start(p->txtimer, p->txmode.startat, p->txmode.after, eo_tmrmode_FOREVER, p->actiontx);
            }
        }
    }
    
    return(res);
}


extern eOresult_t eo_socketdtg_Close(EOsocketDatagram *p)
{
    eOresult_t res = eores_NOK_generic;

    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }
    
    if(STATUS_SOCK_NONE != p->socket->status)
    {
        res = eov_ipnet_DetachSocket(eov_ipnet_GetHandle(), p);
    }

    eo_timer_Stop(p->txtimer);
    
    return(res);
}


extern eOresult_t eo_socketdtg_Connect(EOsocketDatagram *p, eOipv4addr_t ipaddr, eOreltime_t tout)
{
    eOresult_t res = eores_NOK_generic;

    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }
    
    if(0x00000000 == ipaddr)
    {
        // cannot connect to an invalid address
        return(eores_NOK_generic);
    }
    
     
    // do arp.
    res = eov_ipnet_ResolveIP(eov_ipnet_GetHandle(), ipaddr, tout);

    return(res);
}

extern eOresult_t eo_socketdtg_Put(EOsocketDatagram *p, EOpacket *pkt)
{
    eOresult_t res = eores_NOK_generic;

    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }
    
    if(NULL == eo_socket_hid_derived_Get_Handle(p))
    //if(NULL == p->socket->skthandle)
    {
        // task net failed to open the socket or the socket has been closed
        // it failed because it already has the mx number of sockets or maybe because it does not support any.
        return(eores_NOK_generic);
    }

    // if rx-only ... i cannot transmit
    if(eo_sktdir_RXonly == p->socket->dir)
    {
        return(eores_NOK_generic);
    }
    
    // put pkt into output queue.
    res = eo_fifo_Put(p->dgramfifooutput, pkt, p->toutfifos);
    
    if(eores_OK != res)
    {
        return(res);
    }
    
    // alert that a new pkt is available from this socket. immediately or deferred by a one-shot timer or even at periodic slots
    if((eok_abstimeNOW == p->txmode.startat) && (0 == p->txmode.after))
    {
        res = eov_ipnet_Alert(eov_ipnet_GetHandle(), p, EOK_ipnet_evt_TXdatagram);
    }
    else if(eobool_false == p->txmode.periodic)
    {
        if(eo_tmrstat_Running == eo_timer_GetStatus(p->txtimer))
        {
            res = eores_NOK_generic;
        }
        else
        {
            res = eo_timer_Start(p->txtimer, p->txmode.startat, p->txmode.after, eo_tmrmode_ONESHOT, p->actiontx);
        }
    }
    else
    {
        res = eores_OK;
    }

    return(res);
}


extern eOresult_t eo_socketdtg_Get(EOsocketDatagram *p, EOpacket *pkt, eOreltime_t blockingtimeout)
{
    const void *titem = NULL;
    eOresult_t res = eores_NOK_generic;


    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }
    
    if(NULL == eo_socket_hid_derived_Get_Handle(p))
    //if(NULL == p->socket->skthandle)
    {
        // task net failed to open the socket ... or we have just closed the socket and we want to retrieve
        // any received packets laying in the fifoinput.
        // THUS, we dont return.
        //return(eores_NOK_generic);
    }

    // if tx-only ... i cannot receive
    if(eo_sktdir_TXonly == p->socket->dir)
    {
        return(eores_NOK_generic);
    }
    
    // get pkt from input queue.

    if(eobool_true == p->socket->block2wait4packet)
    {
        res = eov_ipnet_WaitPacket(eov_ipnet_GetHandle(), p, blockingtimeout);

        if(eores_OK != res)
        {
            return(eores_NOK_timeout);
        }
    }
 
    res = eo_fifo_Get(p->dgramfifoinput, &titem, p->toutfifos);

    if(eores_OK == res) 
    {
        eo_packet_hid_DefCopy(pkt, (void*)titem);
        eo_fifo_Rem(p->dgramfifoinput, eok_reltimeINFINITE);  // we want to be sure that we remove the packet.
    }
    
    return(res);        
}



extern eOresult_t eo_socketdtg_Received_NumberOf(EOsocketDatagram *p, eOsizecntnr_t *numberof)
{
    eOresult_t res = eores_NOK_generic;


    if((NULL == p) || (NULL == numberof)) 
    {
        return(eores_NOK_nullpointer);
    }
    

    // if tx-only ... i cannot receive
    if(eo_sktdir_TXonly == p->socket->dir)
    {
        *numberof = 0;
        return(eores_NOK_generic);
    }
    
    // get pkt from input queue.

    res = eo_fifo_Size(p->dgramfifoinput, numberof, p->toutfifos); 

    
    return(res);        
}





// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void s_eo_socketdtg_txcallback(void *arg)
{
    eov_ipnet_Alert(eov_ipnet_GetHandle(), arg, EOK_ipnet_evt_TXdatagram);
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




