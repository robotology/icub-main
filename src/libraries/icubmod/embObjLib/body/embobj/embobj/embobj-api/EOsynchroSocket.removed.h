
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOSYNCHROSOCKET_H_
#define _EOSYNCHROSOCKET_H_



/** @file       EOsynchroSocket.h
    @brief      This header file implements public interface to a datagram socket object.
    @author     marco.accame@iit.it
    @date       12/23/2009
**/

/** @defgroup eo_synsocket Object EOsynchroSocket
    The EOsynchroSocket object is used to manage a socket for UDP packets which transmits only at pre-defined periodic
    instants in time.  The reception is instead a-synchronous and the notification of a reception can be done either
    immediately or at the same pre-defined periodic instants in time at which the transmission happens.
     
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOaction.h"
#include "EOVmutex.h"
#include "EOpacket.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


 

/** @typedef    typedef struct EOsynchroSocket_hid EOsynchroSocket
    @brief      EOsynchroSocket is an opaque struct. It is used to implement data abstraction for the timer 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOsynchroSocket_hid EOsynchroSocket;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
/** @fn         extern EOsynchroSocket* eo_synsocket_New(uint16_t payloadsize)
    @brief      Creates a new synchronised socket object. 
    @param      payloadsize         The max size of the managed packet.
    @return     The pointer to the required object.
 **/
extern EOsynchroSocket* eo_synsocket_New(uint16_t payloadsize);


/** @fn         extern eOresult_t eo_synsocket_Open(EOsynchroSocket *p, eOipv4port_t localport, 
                                    eOsocketDirection_t dir, eOabstime_t starttime, eOreltime_t period,
                                    eObool_t block2wait4packet, EOaction *onrec, EOaction *ontxdone)
    @brief      Opens a datagram socket and registers an action on reception
    @param      p               The object pointer. 
    @param      localport       The local port of the socket. 
    @param      dir             The direction of the socket
    @param      starttime       the absolute time of begin of periodic transmission slots
    @param      period          The period of transmission slots
    @param      block2wait4packet    It tells if the socket is a blocking socket or a signal-based one. 
    @param      onrec           When the socket is signal-based, then this pointer contains the action to be executed on 
                                reception of a datagram onto the localport. The action can be used to call eo_synsocket_Get()
                                so that one can retrieve a received datagram with an signal-driven strategy.  If NULL, the IPnet
                                puts messages in queue but does no action.
    @param      ontxdone        If not NULL it is exscuted at transmissione of a packet
    @return     eores_OK upon success, otherwise: eores_NOK_nullpointer if p is NULL or there is no IP service 
                active (the service is given by any EOVtheIPnet-derived singleton), eores_NOK_generic upon 
                failure to open the socket.
 **/
extern eOresult_t eo_synsocket_Open(EOsynchroSocket *p, eOipv4port_t localport, 
                                    eOsocketDirection_t dir, eOabstime_t starttime, eOreltime_t period,
                                    eObool_t block2wait4packet, EOaction *onrec, EOaction *ontxdone);



/** @fn         extern eOresult_t eo_synsocket_Close(EOsynchroSocket *p)
    @brief      Closes a datagram socket but does not destroy the object.
    @param      p               The object pointer. 
    @return     eores_OK upon success, otherwise: eores_NOK_nullpointer if p is NULL or there is no IP service 
                active (the service is given by any EOVtheIPnet-derived singleton), eores_NOK_generic upon 
                failure to close the socket.
 **/
extern eOresult_t eo_synsocket_Close(EOsynchroSocket *p);

/** @fn         extern eOresult_t eo_synsocket_Connect(EOsynchroSocket *p, eOipv4addr_t ipaddr, eOreltime_t tout)
    @brief      Connects to a remote host and makes it possible the transmission of datagrams. This function
                operates by resolving the IP address using ARP to achieve a corresponding MAC address.
                The operation can take a few seconds on some networks, thus is executed with a timeout. 
                The found correspondence IP-MAC is permanently stored inside a table, which is shared among
                every socket. Thus connection to an already connected IP address is executed without sending 
                any ARP packet.
    @param      p               The object pointer. 
    @parm       ipaddr          The ipaddress.
    @parm       tout            The maximum waiting time.
    @return     eores_OK upon success, otherwise: eores_NOK_nullpointer if p is NULL or there is no IP service 
                active (the service is given by any EOVtheIPnet-derived singleton), eores_NOK_generic upon 
                failure to connect the socket.
 **/
extern eOresult_t eo_synsocket_Connect(EOsynchroSocket *p, eOipv4addr_t ipaddr, eOreltime_t tout);


/** @fn         extern eOresult_t eo_synsocket_Put(EOsynchroSocket *p, EOpacket *pkt, eOreltime_t timeout)
    @brief      Copies a datagram pointed by @e pkt into the trasmitting queue of the socket. It waits 
                upto @e timeout microsec to have access to the queue. After the datagram is put inside
                the queue, the active EOVtheIPnet singleton is alerted to trasmit the datagram.  In this
                way, the tx delay is reduced to the minimum possible by the CPU.
    @param      p               The object pointer. 
    @param      pkt             The pointer to the datagram to be copied into the TX queue.
    @return     eores_OK upon success, otherwise: eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the
                socket was not succesfully opened.
 **/
extern eOresult_t eo_synsocket_Put(EOsynchroSocket *p, EOpacket *pkt);


/** @fn         extern eOresult_t eo_synsocket_Get(EOsynchroSocket *p, EOpacket *pkt, eOreltime_t blockingtimeout)
    @brief      Retrieves (and removes) from internal FIFO queue a datagram received by the socket and
                copies it into datagram pointed by @e pkt. It waits upto @e timeout microsec to have 
                access to the queue. 
    @param      p               The object pointer. 
    @param      pkt             The pointer of the packet onto which it will be copied what the socket has
                                received.
    @param      blockingtimeout If non-zero, the function blocks until there is an available packet or until
                                the timeout expires.
    @return     eores_OK upon success, otherwise: eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the
                socket was not succesfully opened, eores_NOK_timeout if the timeout expired.
 **/
extern eOresult_t eo_synsocket_Get(EOsynchroSocket *p, EOpacket *pkt, eOreltime_t blockingtimeout);



/** @}            
    end of group eo_synsocket  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

