
/* @file       EOsynchroSocket.c
    @brief      This file implements internal implementation of a datagram socket object synchronised in tx.
    @author     marco.accame@iit.it
    @date       12/24/2009
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"

#include "EOVtheIPnet.h"
#include "EOpacket.h"
#include "EOpacket_hid.h"
#include "EOsocket_hid.h"
#include "EOaction_hid.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOsynchroSocket.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOsynchroSocket_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOsynchroSocket";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOsynchroSocket* eo_synsocket_New(uint16_t payloadsize)
{
    EOsynchroSocket *retptr = NULL;    

    eo_errman_Assert(eo_errman_GetHandle(), (0 != payloadsize), s_eobj_ownname, "payload size is zero");

    // i get the memory for the object
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOsynchroSocket), 1);

    // get the base object
    retptr->socket = eo_socket_New();

    // now the otehr fields
    retptr->txpkt           = eo_packet_New(payloadsize);
    retptr->rxpkt           = eo_packet_New(payloadsize);
    retptr->txenable        = 0;
    
    retptr->timertx           = eo_timer_New();
 
    return(retptr);
}


extern eOresult_t eo_synsocket_Open(EOsynchroSocket *p, eOipv4port_t localport, 
                                    eOsocketDirection_t dir, eOabstime_t starttime, eOreltime_t period,
                                    eObool_t block2wait4packet, EOaction *onrec, EOaction *ontxdone)
{

    eOresult_t res = eores_NOK_generic;
    EOaction paction;

    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }


    if(((0 == localport) ||  (0 == period)))
    {
        eo_errman_Assert(eo_errman_GetHandle(), 0, s_eobj_ownname, "eo_synsocket_Open() needs a port and a period");
    }

    
    // open only if we did not open it before
    if(STATUS_SOCK_NONE == p->socket->status)
    {


        eo_socket_hid_derived_Prepare(p, eo_skttyp_synchro, localport, onrec, ontxdone, dir, block2wait4packet); 

//        p->socket->localport            = localport;
//        p->socket->skthandle           = NULL;    // reset it ...
//        p->socket->dir                  = dir;
//        p->socket->getmode              = getmode;
//
//        // copy action into socket.
//        if(NULL != onrec)
//        {
//            eo_action_Copy(p->socket->onreception, onrec);
//        }
//        else
//        {
//            eo_action_Clear(p->socket->onreception); 
//        }
//
//
//        // copy tx actions into socket.
//        if(NULL != ontxdone)
//        {
//            eo_action_Copy(p->socket->ontransmission, ontxdone);
//        }
//        else
//        {
//            eo_action_Clear(p->socket->ontransmission); 
//        }


        p->starttime    = starttime;
        p->period       = period;

        res = eov_ipnet_AttachSocket(eov_ipnet_GetHandle(), p);
        
        if((eores_OK == res) && ((eo_sktdir_TXonly == p->socket->dir) || (eo_sktdir_TXRX == p->socket->dir) ))
        {
            eo_action_SetEvent(&paction, (EOK_ipnet_evt_TXsynchroBASE << p->socket->tag), eov_ipnet_GetTask(eov_ipnet_GetHandle()));
            eo_timer_Start(p->timertx, starttime, period, eo_tmrmode_FOREVER, &paction);
            p->txenable = 0;
        }
    }
    
    return(res);
}


extern eOresult_t eo_synsocket_Close(EOsynchroSocket *p)
{
    eOresult_t res = eores_NOK_generic;

    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }
    
    if(STATUS_SOCK_NONE != p->socket->status)
    {
        res = eov_ipnet_DetachSocket(eov_ipnet_GetHandle(), p);

        if( (eo_sktdir_TXonly == p->socket->dir) || (eo_sktdir_TXRX == p->socket->dir) )
        {
            eo_timer_Stop(p->timertx);
        }
    }

    p->starttime    = 0;
    p->period       = 0;
    
    return(res);
}


extern eOresult_t eo_synsocket_Connect(EOsynchroSocket *p, eOipv4addr_t ipaddr, eOreltime_t tout)
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

extern eOresult_t eo_synsocket_Put(EOsynchroSocket *p, EOpacket *pkt)
{
    eOresult_t res = eores_OK;

    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }
    
    if(NULL == eo_socket_hid_derived_Get_Handle(p))
    //if(NULL == p->socket->skthandle)
    {
        // task net failed to open the socket or the socket has been closed
        return(eores_NOK_generic);
    }


    // if rx-only ... i cannot transmit
    if(eo_sktdir_RXonly == p->socket->dir)
    {
        return(eores_NOK_generic);
    }

    // acemor-warning..... it should be atomic
    p->txenable = 0;
    eov_ipnet_DisableSynTx(eov_ipnet_GetHandle(), p->socket->tag);
    
    // copy pkt into local tx pkt.
    eo_packet_hid_DefCopy(p->txpkt, pkt);
    
    // set the enable flag
    // acemor-warning..... it should be atomic
    p->txenable = 1;
    eov_ipnet_EnableSynTx(eov_ipnet_GetHandle(), p->socket->tag);
    
    return(res);
}


extern eOresult_t eo_synsocket_Get(EOsynchroSocket *p, EOpacket *pkt, eOreltime_t blockingtimeout)
{
    eOresult_t res = eores_NOK_generic;

    if((NULL == p) || (NULL == pkt)) 
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


    if(eobool_true == p->socket->block2wait4packet)
    {
        res = eov_ipnet_WaitPacket(eov_ipnet_GetHandle(), p, blockingtimeout);

        if(eores_OK != res)
        {
            return(eores_NOK_timeout);
        }
    }

    
    eo_packet_hid_DefCopy(pkt, p->rxpkt);
    
    return(eores_OK);        
}




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




