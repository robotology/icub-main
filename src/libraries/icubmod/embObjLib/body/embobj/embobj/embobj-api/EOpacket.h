
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOPACKET_H_
#define _EOPACKET_H_


/** @file       EOpacket.h
    @brief      This header file implements public interface to a packet.
    @author     marco.accame@iit.it
    @date       01/11/2010
**/

/** @defgroup eo_packet Object EOpacket
    The EOpacket object is used as a container of UDP datagrams and TCP packets.
    As such it has an IP address, a port and a data payload.
    The EOpacket can be created in two different modes. In the first mode the object has internal storage for the payload
    that is allocated at creation of the object. In such a way, data is copied to and from the internal buffer. 
    In the second mode, the object just cointains a reference to an externally allocated payload.
         
    @{        
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"




// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/** @typedef    typedef struct EOpacket_hid EOpacket
    @brief      EOpacket is an opaque struct. It is used to implement data abstraction for the datagram 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOpacket_hid EOpacket;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
/** @fn         extern EOpacket* eo_packet_New(uint16_t capacity)
    @brief      Creates a new packet object and allocates memory able to store @e capacity bytes. If @e capacity is
                zero, then the object shall have external storage mode.
    @param      capacity   The max size of the packet.
    @return     The pointer to the required object.
 **/
extern EOpacket* eo_packet_New(uint16_t capacity);


/** @fn         extern eOresult_t eo_packet_Full_LinkTo(EOpacket *p, eOipv4addr_t addr, eOipv4port_t port, uint16_t size, uint8_t *data)
    @brief      Sets the content of a packet by simply making a reference the @e data payload.  It must be used on a
                EOpacket object created with eo_packet_New(0), otherwise there will be a failure of the operation. 
    @param      p               The packet.
    @param      addr            The address
    @param      port            The port
    @param      size            The size of teh data
    @param      data            Pointer to data to be stored in the packet
    @return     eores_OK or eores_NOK_nullpointer if either @e p or @e data are NULL. or eores_NOK_generic if the
                packet was created with internal storage.
    @warning    As a eo_packet_Set() operation on a EOpacket created with non zero capacity may wast memory, this oepration
                is forbidden in such a case.
 **/
extern eOresult_t eo_packet_Full_LinkTo(EOpacket *p, eOipv4addr_t addr, eOipv4port_t port, uint16_t size, uint8_t *data);



/** @fn         extern eOresult_t eo_packet_Full_Set(EOpacket *p, eOipv4addr_t addr, eOipv4port_t port, uint16_t size, uint8_t *data)
    @brief      Sets the full content of a packet by setting destination and by copying data internally. It must be used on a
                EOpacket object created with eo_packet_New(capacity), otherwise there will be no copy at all. 
    @param      p               The packet.
    @param      addr            The address
    @param      port            The port
    @param      size            The size of teh data
    @param      data            Pointer to data to be stored in the packet
    @return     eores_OK or eores_NOK_nullpointer if either @e p or @e data are NULL. or eores_NOK_generic if the
                packet was not created with internal storage.
    @warning    As a eo_packet_Set() operation on a EOpacket created with zero capacity does not work.
 **/
extern eOresult_t eo_packet_Full_Set(EOpacket *p, eOipv4addr_t addr, eOipv4port_t port, uint16_t size, uint8_t *data);





/** @fn         extern eOresult_t eo_packet_Addressing_Set(EOpacket *p, eOipv4addr_t remaddr, eOipv4port_t remport)
    @brief      Sets the destination. 
    @param      p               The packet.
    @param      remaddr         The address
    @param      remport         The port
    @return     eores_OK or eores_NOK_nullpointer if @e p is NULL.
 **/
extern eOresult_t eo_packet_Addressing_Set(EOpacket *p, eOipv4addr_t remaddr, eOipv4port_t remport);
#define eo_packet_Destination_Set eo_packet_Addressing_Set

/** @fn         extern eOresult_t eo_packet_Addressing_Get(EOpacket *p, eOipv4addr_t *remaddr, eOipv4port_t *remport)
    @brief      Gets the destination. 
    @param      p               The packet.
    @param      remaddr         pointer to the address
    @param      remport         pointer to the port
    @return     eores_OK or eores_NOK_nullpointer if any passed pointer is NULL.
 **/
extern eOresult_t eo_packet_Addressing_Get(EOpacket *p, eOipv4addr_t *remaddr, eOipv4port_t *remport);
#define eo_packet_Destination_Get eo_packet_Addressing_Get


/** @fn         extern eOresult_t eo_packet_Payload_Set(EOpacket *p, uint8_t *data, uint16_t size)
    @brief      Sets the content of a packet by copying the data pointer by @e data into the internal buffer.
                It must be used on a EOpacket object created with eo_packet_New(capacity).
                If size is smaller than internal capacity, then the function copies just to fill internal capacity
                and sets an internal incomplete flag. 
    @param      p               The packet.
    @param      data            Pointer to data to be copied in the packet
    @param      size            The size of the data
    @return     eores_OK or eores_NOK_nullpointer if either @e p or @e data are NULL.
 **/
extern eOresult_t eo_packet_Payload_Set(EOpacket *p, uint8_t *data, uint16_t size);


extern eOresult_t eo_packet_Size_Set(EOpacket *p, uint16_t size);

// consider using ... assign()


/** @fn         extern uint16_t eo_packet_Payload_PushBack(EOpacket *p, uint8_t *data, uint16_t size)
    @brief      Write the content of a packet by appending the data pointer by @e data at the end of data
                previously placed in the internal buffer.
                It must be used on a EOpacket object created with eo_packet_New(capacity).
                If size is smaller than internal capacity, then the function copies just to fill internal capacity
                and sets an internal incomplete flag. 
    @param      p               The packet.
    @param      data            Pointer to data to be copied in the packet
    @param      size            The size of the data
    @return     The number of effectively written bytes.
 **/
extern uint16_t eo_packet_Payload_PushBack(EOpacket *p, uint8_t *data, uint16_t size);


/** @fn         extern eOresult_t eo_packet_Payload_Get(EOpacket *p, uint8_t **data, uint16_t *size)
    @brief      Retrieved the content of a packet by simpel reference to internal buffer.
    @param      p               The packet.
    @param      data            Pointer to internal data
    @param      size            pointer to the size of the data
    @return     eores_OK or eores_NOK_nullpointer if any passed pointer is NULL.
 **/
extern eOresult_t eo_packet_Payload_Get(EOpacket *p, uint8_t **data, uint16_t *size);


/** @fn         extern uint16_t eo_packet_Payload_ProgressiveRead(EOpacket *p, uint16_t size, uint8_t **data)
    @brief      Progressively read the content of a packet by simple reference to internal buffer.
    @param      p               The packet.
    @param      size            The number of bytes to read
    @param      data            Pointer to internal data
    @return     The number of read bytes.
 **/
extern uint16_t eo_packet_Payload_ProgressiveRead(EOpacket *p, uint16_t size, uint8_t **data);


/** @fn         extern eOresult_t eo_packet_Full_Clear(EOpacket *p, uint8_t val)
    @brief      Sets to @e val the content of the internal buffer of the packet and sets its size to zero.
                If size is smaller than internal capacity, then the function copies just to fill internal capacity
                and sets an internal incomplete flag. 
    @param      p               The packet.
    @parm       val             The value (typically it is 0).
    @return     eores_OK or eores_NOK_nullpointer if @e p is NULL.
 **/
extern eOresult_t eo_packet_Full_Clear(EOpacket *p, uint8_t val);


/** @fn         extern eOresult_t eo_packet_Capacity_Get(EOpacket *p, uint16_t *capacity)
    @brief      Retrieved the capacity of the packet.
    @param      p               The packet.
    @param      capacity        Pointer to capacity value
    @return     eores_OK or eores_NOK_nullpointer if any passed pointer is NULL.
 **/
extern eOresult_t eo_packet_Capacity_Get(EOpacket *p, uint16_t *capacity);


/** @fn         extern uint16_t eo_packet_Payload_Pad(EOpacket *p, uint16_t finalsize, uint8_t val)
    @brief      Append the value @e val to end of packet so that its final size becomes @e finalsize
    @param      p               The packet.
    @param      finalsize       The target final size of the packet.
    @parm       val             The value used to pad data
    @return     The final size of the packet.
 **/
extern uint16_t eo_packet_Payload_Pad(EOpacket *p, uint16_t finalsize, uint8_t val);

extern eOresult_t eo_packet_Copy(EOpacket *p, const EOpacket *source);



/** @}            
    end of group eo_packet  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

