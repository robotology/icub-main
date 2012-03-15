
/* @file       EOpacket.c
    @brief      This file implements internal implementation of a datagram socket object.
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





// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOpacket.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOpacket_hid.h" 


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

//static const char s_eobj_ownname[] = "EOpacket";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOpacket* eo_packet_New(uint16_t capacity)
{
    EOpacket *retptr = NULL;    

    // i get the memory for the object
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOpacket), 1);
    
    retptr->remoteaddr          = 0;
    retptr->remoteport          = 0;
    retptr->incomplete_flag     = 0;
    retptr->size                = 0;
    retptr->capacity            = capacity; 
    retptr->write_index         = 0;
    retptr->read_index          = 0;
    retptr->data                = (0 == capacity) ? 
                                  (NULL) : 
                                  (eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_08bit, capacity, 1));

    retptr->externaldatastorage = (0 == capacity) ? (1) : (0);

    return(retptr);
}


extern eOresult_t eo_packet_Full_LinkTo(EOpacket *p, eOipv4addr_t addr, eOipv4port_t port, uint16_t size, uint8_t *data)
{
    if((NULL == p) || (NULL == data)) 
    {
        return(eores_NOK_nullpointer);
    }

    if(1 != p->externaldatastorage)
    {
        // waste of memory .... avoid it
        return(eores_NOK_generic);
    }


    p->remoteaddr = addr;
    p->remoteport = port;
    p->incomplete_flag = 0;
    p->capacity = p->size = size; 
    p->write_index = size;      
    p->read_index = 0;  
    
    p->data = data;

    return(eores_OK);
}

extern eOresult_t eo_packet_Full_Set(EOpacket *p, eOipv4addr_t addr, eOipv4port_t port, uint16_t size, uint8_t *data)
{
    if((NULL == p) || (NULL == data)) 
    {
        return(eores_NOK_nullpointer);
    }

    if(0 != p->externaldatastorage)
    {
        // cannot copy into internal storage
        return(eores_NOK_generic);
    }


    eo_packet_Payload_Set(p, data, size);

    p->remoteaddr = addr;
    p->remoteport = port;

    return(eores_OK);
}

extern eOresult_t eo_packet_Addressing_Set(EOpacket *p, eOipv4addr_t remaddr, eOipv4port_t remport)
{

    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }
    
    p->remoteaddr   = remaddr;
    p->remoteport   = remport;

    return(eores_OK);
}


extern eOresult_t eo_packet_Addressing_Get(EOpacket *p, eOipv4addr_t *remaddr, eOipv4port_t *remport)
{

    if((NULL == p) || (NULL == remaddr) || (NULL == remport)) 
    {
        return(eores_NOK_nullpointer);
    }
    
    *remaddr = p->remoteaddr;
    *remport = p->remoteport;

    return(eores_OK);
}


extern eOresult_t eo_packet_Payload_Set(EOpacket *p, uint8_t *data, uint16_t size)
{
    eOresult_t res = eores_NOK_generic;

    if((NULL == p) || (NULL == data)) 
    {
        return(eores_NOK_nullpointer);
    }

    if(0 != p->externaldatastorage)
    {
        return(eores_NOK_generic);
    }
    
    if(size <= p->capacity)
    {
        memcpy(p->data, data, size);
        p->size = size;
        p->write_index = size;
        p->read_index = 0;
        res = eores_OK;
    }
    else
    {
        memcpy(p->data, data, p->capacity);
        p->size = p->capacity;
        p->write_index = p->capacity;
        p->read_index = 0;
        p->incomplete_flag = 1;
        res = eores_OK;
    }

    return(res);
}

extern eOresult_t eo_packet_Size_Set(EOpacket *p, uint16_t size)
{
    eOresult_t res = eores_NOK_generic;

    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }

  
    if(size <= p->capacity)
    {
        p->size = size;
        p->write_index = size;
        p->read_index = 0;
        res = eores_OK;
    }
    else
    {
        p->size = p->capacity;
        p->write_index = p->capacity;
        p->read_index = 0;
        p->incomplete_flag = 1;
        res = eores_OK;
    }

    return(res);
}


extern uint16_t eo_packet_Payload_PushBack(EOpacket *p, uint8_t *data, uint16_t size)
{
    uint16_t res = 0;

    if((NULL == p) || (NULL == data)) 
    {
        return(0);
    }

    if(0 != p->externaldatastorage)
    {
        return(0);
    }
    
    if((p->size+size) <= p->capacity)
    {   // can append the whole data
        res = size;
        memcpy(&p->data[p->size], data, res);
        p->size += size;
        p->write_index = p->size;
        //p->read_index = 0;
    }
    else if(p->size < p->capacity)
    {   // can append only a portion of data
        res = p->capacity - p->size;
        memcpy(&p->data[p->size], data, res);
        p->size = p->capacity;
        p->write_index = p->size;
        //p->read_index = 0;
        p->incomplete_flag = 1;
    }

    return(res);
}


extern eOresult_t eo_packet_Payload_Get(EOpacket *p, uint8_t **data, uint16_t *size)
{

    if((NULL == p) || (NULL == data) || (NULL == size)) 
    {
        return(eores_NOK_nullpointer);
    }

    p->read_index = p->size;
    
    *data = p->data;
    *size = p->size;

    return(eores_OK);
}


extern uint16_t eo_packet_Payload_ProgressiveRead(EOpacket *p, uint16_t size, uint8_t **data)
{
    uint16_t ret = 0;

    if((NULL == p) || (NULL == data) || (0 == size)) 
    {
        return(0);
    }

    if(p->read_index == p->size)
    {
        ret = 0;
        *data = NULL;
    }
    else if((p->read_index + size) <= p->size)
    {
        ret = size;
        *data = &p->data[p->read_index];
        p->read_index += size;
    }
    else
    {
        ret = p->size - p->read_index;
        *data = &p->data[p->read_index];
        p->read_index = p->size;
    }

    return(ret);
}


extern eOresult_t eo_packet_Full_Clear(EOpacket *p, uint8_t val)
{
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }
    p->remoteaddr = 0;
    p->remoteport = 0;
    memset(p->data, val, p->capacity);
    p->size = 0;
    p->write_index = 0;
    p->read_index = 0;
    p->incomplete_flag = 0;

    return(eores_OK);
}

extern eOresult_t eo_packet_Capacity_Get(EOpacket *p, uint16_t *capacity)
{

    if((NULL == p) || (NULL == capacity)) 
    {
        return(eores_NOK_nullpointer);
    }

    *capacity = p->capacity;

    return(eores_OK);
}

extern uint16_t eo_packet_Payload_Pad(EOpacket *p, uint16_t finalsize, uint8_t val)
{
    if(NULL == p) 
    {
        return(0);
    }
	
	if(p->capacity == p->size) // pacchetto pieno
	{
	  return(p->capacity);
	}    

    if(p->capacity > finalsize) // cannot write beyond capacity
    {
        finalsize = p->capacity;
    } 

    memset(&p->data[p->size], val, (finalsize - p->size));
    p->size = finalsize;
	p->write_index = p->size;

    return(finalsize);
}


extern eOresult_t eo_packet_Copy(EOpacket *p, const EOpacket *source)
{
    return(eo_packet_hid_DefCopy(p, (void*)source));
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

extern eOresult_t eo_packet_hid_DefInit(void *p, uint32_t a)
{
    EOpacket *dtg = (EOpacket *)p;

    // arg is maxsize 

    dtg->remoteaddr         = 0;
    dtg->remoteport         = 0;
    dtg->incomplete_flag    = 0;
    dtg->size               = 0;
    dtg->write_index        = 0;
    dtg->read_index         = 0;
    dtg->capacity           = a;
    dtg->data               = (0 ==a ) ? (NULL) : (eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_08bit, sizeof(uint8_t), a));
    dtg->externaldatastorage = (0 == a) ? (1) : (0);

    return(eores_OK);
}


extern eOresult_t eo_packet_hid_DefCopy(void *d, void *s)
{
    // it is a safe copy. we just dont verify vs pointer not being NULL, 
    // since we assume the dest and orig must be not NULL
    // we also assume that dest->data and orig->data are not NULL (already allocated)
    EOpacket *dest = (EOpacket *)d;
    EOpacket *orig = (EOpacket *)s;



    dest->remoteaddr        = orig->remoteaddr;
    dest->remoteport        = orig->remoteport;
    dest->incomplete_flag   = (dest->capacity>orig->size) ? (0) : (1);
    dest->size              = (0 == dest->incomplete_flag) ? (orig->size) : (dest->capacity);
    dest->write_index       = dest->size;
    dest->read_index        = 0;
    // the dest->capacity  remains constant as it expresses size of memory allocated by the object
    // the dest->externaldatastorage remains constant as it expresses .... actually it should be 0 in here.
    memcpy(dest->data, orig->data, dest->size);

    return(eores_OK);
}

extern eOresult_t eo_packet_hid_DefClear(void *p)
{
    EOpacket *dtg = (EOpacket *)p;

    dtg->remoteaddr         = 0;
    dtg->remoteport         = 0;
    dtg->incomplete_flag    = 0;
    dtg->size               = 0;
    dtg->write_index        = 0;
    dtg->read_index         = 0;
    // dtg->capacity = keep it as it is
    // dtg->data = keep as it is
    // dtg->externaldatastorage = keep as it is
    //memset(dtg->data, 0, dtg->capacity);

    return(eores_OK);
}




// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




