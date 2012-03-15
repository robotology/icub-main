
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOSOCKETDATAGRAM_H_
#define _EOSOCKETDATAGRAM_H_



/** @file       EOsocketDatagram.h
    @brief      This header file implements public interface to a socket object bearing datagrams using UDP
    @author     marco.accame@iit.it
    @date       08/25/2011
**/

/** @defgroup eo_socketdtg Object EOsocketDatagram
    The EOsocketDatagram object is used to manage a socket for UDP datagrams.  The socket is created, opened for listening
    on a given IP port, connected for transmission to a given IP address. The socket also allows to transmit a packet
    of type EOpacket, and also to retrieve a received packet.  The socket is implemented with a transmission queue, so that 
    the sender is not blocked to wait effective transmission. It also has the feauture that the datagram can be put in the queue
    at a deferred time or on a periodic base.
    It also has a reception queue which allows an event-based reception scheme: a task can be idle or doing something else
    until an action EOaction is executed upon reception of a packet.  The action can for instance be the sending of an event
    which tells the relevant task to retrieve the received packet.
    The socket can also be used with a blocking reception scheme: the receiver task is blocked inside the receiving function
    until a packet arrives.
    The socket can also be closed and reopened, maybe on another port or with another reception mode. In case of closing,
    those packets succesfully put inside the sending queue are guaranteed to be transmitted before the socket is closed.
    The caller just waits for teh end of transmission. This can can happen if the IPnet task has a lower priority of the
    user of the socket.
    Also, those packets received before teh socket closure and that are inside the reception queue can be retrieved even if
    the socket is already closed.
     
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOaction.h"
#include "EOVmutex.h"
#include "EOpacket.h"
#include "EOsocket.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 



/** @typedef    typedef struct EOsocketDatagram_hid EOsocketDatagram
    @brief      EOsocketDatagram is an opaque struct. It is used to implement data abstraction for the timer 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOsocketDatagram_hid EOsocketDatagram;


/** @typedef    typedef struct eOsktdtgTXmode_t
    @brief      eOsktdtgTXmode_t contains the mode with which a datagram socket sends packets out.
                The EOsocketDatagram does not transmit by itself and just sends a request to a EOVtheIPnetDerived
                object. To send this request immediately (mode1) the user must use the values {EOK_abstimeNOW, 0, eobool_false}.
                To send it with a delay (mode2) for instance of 100 ms the values are {EOK_abstimeNOW, EOK_reltime100ms, eobool_false}.
                If the user specifies a value of startat different of EOK_abstimeNOW, then the request is done at absolute
                time @e startat + @after. If @periodic is false the request is done only once unsless teh socket is reopened
                with a different mode. Thus, this latter mode is of little utility.  Far more useful is the mode3 in which the
                request is done periodically with period @after microseconds with a given starting absolute time @startat.
                For instance with {0, EOK_reltime100ms, eobool_true} the socket transmits a datagram at precise time-slots.
 **/ 
typedef struct
{
     eOabstime_t startat;       /**< The absolute time of start of a countdown. If EOKabstimeNOW then it starts immediately  */
     eOreltime_t after;         /**< The initial value of a countdown that when expired issues a tx request to the IP net   */
     eObool_t periodic;         /**< If eobool_true the countdown is retriggered                           */
} eOsktdtgTXmode_t;




    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const eOsktdtgTXmode_t eo_sktdtg_TXnow; // = {EOK_abstimeNOW, 0, eobool_false};


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
/** @fn         extern EOsocketDatagram* eo_socketdtg_New(uint8_t dtg_in_num, uint16_t dtg_in_size, EOVmutexDerived *mtx_fifo_in,
                                          uint8_t dtg_out_num, uint16_t dtg_out_size, EOVmutexDerived *mtx_fifo_out)
    @brief      Creates a new datagram socket object. 
    @param      dtg_in_num          The max number of datagrams that can be enqueued at reception. 
    @param      dtg_in_size         The max size of datagrams accepted by the input queue.
    @param      mtx_fifo_in         The mutex used by the input queue. 
    @param      dtg_out_num         The max number of datagrams that can be enqueued before transmission. 
    @param      dtg_out_size        The max size of datagrams accepted by the output queue.
    @param      mtx_fifo_out        The mutex used by the output queue. 
    @return     The pointer to the required object.
 **/
extern EOsocketDatagram* eo_socketdtg_New(uint8_t dtg_in_num, uint16_t dtg_in_size, EOVmutexDerived *mtx_fifo_in,
                                          uint8_t dtg_out_num, uint16_t dtg_out_size, EOVmutexDerived *mtx_fifo_out);


/** @fn         extern eOresult_t eo_socketdtg_Open(EOsocketDatagram *p, eOipv4port_t localport, eOsocketDirection_t dir, 
                                                    eObool_t block2wait4packet, eOsktdtgTXmode_t *txmode, 
                                                    EOaction *onrx, EOaction *ontx)
    @brief      Opens a datagram socket and registers an action on reception
    @param      p               The object pointer. 
    @param      localport       The local port of the socket. 
    @param      dir             The direction of the socket
    @param      block2wait4packet        It tells if the socket is a blocking socket where a call to eo_socketdtg_Get() is blocked
                                until a packet is available. 
    @param      txmode          Tells when is done the transmission of the packet. If NULL, then the function uses eo_sktdtg_TXnow.
    @param      onrx            Action executed upon arrival of a datagram inside the internal fifo. It can be used to send an event
                                to a processing task to retrieve a datagram when the socket is in non-blocking mode.
    @return     eores_OK upon success, otherwise: eores_NOK_nullpointer if p is NULL or there is no IP service 
                active (the service is given by any EOVtheIPnet-derived singleton), eores_NOK_generic upon 
                failure to open the socket.
 **/
extern eOresult_t eo_socketdtg_Open(EOsocketDatagram *p, eOipv4port_t localport, eOsocketDirection_t dir, eObool_t block2wait4packet, eOsktdtgTXmode_t *txmode, EOaction *onrx, EOaction *ontx);


/** @fn         extern eOresult_t eo_socketdtg_Close(EOsocketDatagram *p)
    @brief      Closes a datagram socket but does not destroy the object.
    @param      p               The object pointer. 
    @return     eores_OK upon success, otherwise: eores_NOK_nullpointer if p is NULL or there is no IP service 
                active (the service is given by any EOVtheIPnet-derived singleton), eores_NOK_generic upon 
                failure to close the socket.
 **/
extern eOresult_t eo_socketdtg_Close(EOsocketDatagram *p);

/** @fn         extern eOresult_t eo_socketdtg_Connect(EOsocketDatagram *p, eOipv4addr_t ipaddr, eOreltime_t tout)
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
extern eOresult_t eo_socketdtg_Connect(EOsocketDatagram *p, eOipv4addr_t ipaddr, eOreltime_t tout);


/** @fn         extern eOresult_t eo_socketdtg_Put(EOsocketDatagram *p, EOpacket *pkt, eOreltime_t timeout)
    @brief      Copies a datagram pointed by @e pkt into the trasmitting queue of the socket. After the datagram 
                is put inside the queue, the active EOVtheIPnet singleton is alerted to trasmit the datagram in the mode
                defined by the argument @e txmode of eo_socketdtg_Open().
    @param      p               The object pointer. 
    @param      pkt             The pointer to the datagram to be copied into the TX queue.
    @return     eores_OK upon success, otherwise: eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the
                socket was not succesfully opened.
 **/
extern eOresult_t eo_socketdtg_Put(EOsocketDatagram *p, EOpacket *pkt);


/** @fn         extern eOresult_t eo_socketdtg_Get(EOsocketDatagram *p, EOpacket *pkt, eOreltime_t blockingtimeout)
    @brief      Retrieves (and removes) from internal FIFO queue a datagram received by the socket and
                copies it into datagram pointed by @e pkt. If teh socket is a blocking socket, it waits upto @e blockingtimeout
                microsec a packet to arrive. 
    @param      p               The object pointer. 
    @param      pkt             The pointer of the datagram onto which it will be copied what the socket has
                                received.
    @param      blockingtimeout If the socket was opened with @e block2wait4packet equal to eobool_true,
                                the function blocks for @e blockingtimeout microseconds.
    @return     eores_OK upon success, otherwise: eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the
                socket was not succesfully opened, eores_NOK_timeout if the timeout expires.
 **/
extern eOresult_t eo_socketdtg_Get(EOsocketDatagram *p, EOpacket *pkt, eOreltime_t blockingtimeout);


extern eOresult_t eo_socketdtg_Received_NumberOf(EOsocketDatagram *p, eOsizecntnr_t *numberof);



/** @}            
    end of group eo_socketdtg  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

