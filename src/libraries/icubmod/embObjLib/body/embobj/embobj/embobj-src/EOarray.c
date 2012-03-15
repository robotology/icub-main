
/* @file       EOarray.c
    @brief      This file implements internal implementation of a array object.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOarray.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOarray_hid.h" 


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

//static const char s_eobj_ownname[] = "EOarray";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOarray* eo_array_New(uint16_t capacity, uint8_t itemsize, void *memory)
{
    EOarray *retptr = NULL;    

    
    if(NULL != memory)
    {   // i use external memory
        retptr = memory;
    }
    else
    {   // i get the memory for the object
        retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(eOarray_head_t) + capacity*itemsize, 1);
    }

    retptr->head.capacity       = capacity;
    retptr->head.itemsize       = itemsize;
    retptr->head.size           = 0;
    
    eo_array_Reset(retptr);

    return(retptr);
}

extern eOresult_t eo_array_Reset(EOarray *p)
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }

    p->head.size = 0;
    memset(p->data, 0, p->head.capacity*p->head.itemsize);

    return(eores_OK);
}

extern uint16_t eo_array_Capacity(EOarray *p)
{
    if(NULL == p)
    {
        return(0);
    }

    return( p->head.capacity);
}

extern uint8_t eo_array_ItemSize(EOarray *p)
{
    if(NULL == p)
    {
        return(0);
    }

    return(p->head.itemsize);
}

extern uint8_t eo_array_Size(EOarray *p)
{
    if(NULL == p)
    {
        return(0);
    }


    return(p->head.size);
}

extern uint16_t eo_array_UsedBytes(EOarray *p)
{
    if(NULL == p)
    {
        return(0);
    }

    if(p->head.capacity < p->head.size)
    {
        // there must be an error
        return(0);
    }

    return(sizeof(eOarray_head_t) + p->head.size*p->head.itemsize);
}

extern eOresult_t eo_array_PushBack(EOarray *p, const void *item)
{
    if((NULL == p) || (NULL == item))
    {
        return(eores_NOK_nullpointer);
    }
    
    if(p->head.size == p->head.capacity)
    {
        return(eores_NOK_generic);
    }
    
    memcpy(&(p->data[p->head.size*p->head.itemsize]), item, p->head.itemsize);
    p->head.size++;

    return(eores_NOK_generic);
}



extern void * eo_array_At(EOarray *p, uint8_t pos)
{

    if((NULL == p) || (pos >= p->head.capacity))
    {
        return(NULL);
    }

    return(&(p->data[pos*p->head.itemsize]));
}

extern eOresult_t eo_array_PopBack(EOarray *p)
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }
    
    if(0 == p->head.size)
    {
        return(eores_NOK_generic);
    }
    p->head.size--;

    return(eores_OK);
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section




// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




