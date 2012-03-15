
/* @file       EOconstvector.c
	@brief      This file implements internal implementation of a constvector object.
	@author     marco.accame@iit.it
    @date       08/04/2011
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

#include "EOconstvector.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOconstvector_hid.h" 


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

static const char s_eobj_ownname[] = "EOconstvector";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern EOconstvector * eo_constvector_New(eOsizeitem_t item_size, eOsizecntnr_t size, const void *data)
{
    EOconstvector *retptr = NULL;
 

    // i get the memory for the object. no need to check versus NULL because the memory pool already does it
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOconstvector), 1);


    // now the obj has valid memory. i need to initialise it with user-defined data,

    eo_errman_Assert(eo_errman_GetHandle(), (0 != item_size), s_eobj_ownname, "item_size is zero");
    eo_errman_Assert(eo_errman_GetHandle(), (0 != size), s_eobj_ownname, "size is zero");

    retptr->item_size           = item_size;
    retptr->size                = size;
    retptr->item_array_data     = data;     
    
    return(retptr);   
}


extern eOsizecntnr_t eo_constvector_Size(const EOconstvector * cvect) 
{
    if(NULL == cvect) 
    {
        // invalid cvect
        return(0);    
    }
    
    return(cvect->size);        
}


extern const void * eo_constvector_At(const EOconstvector * cvect, eOsizecntnr_t pos) 
{
    // here we require uint8_t to access item_array_data because we work with bytes.
    uint8_t *start = NULL;
    
    if(NULL == cvect) 
    {
        return(NULL);    
    }
    
    if(pos >= cvect->size) 
    { 
        // cvect does not have any element in pos
        return(NULL);     
    }
    
   
    start = (uint8_t*) (cvect->item_array_data);
    // cast to uint32_t to tell the reader that index of array start[] can be bigger than max eOsizecntnr_t
    start = &start[(uint32_t)pos * cvect->item_size];
    
    return((const void*) start);         
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



