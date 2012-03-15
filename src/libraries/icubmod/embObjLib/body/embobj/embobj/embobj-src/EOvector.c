
/* @file       EOvector.c
	@brief      This file implements internal implementation of a vector object.
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

#include "EOvector.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOvector_hid.h" 


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

#if 0
EO_static_inline void s_eo_vector_default_clear(void *p, uint16_t size)
{
     memset(p, 0, size);
}
#else
EO_static_inline void s_eo_vector_default_clear(void *p, uint16_t size)
{
}    
#endif


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOvector";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern EOvector * eo_vector_New(eOsizeitem_t item_size, eOsizecntnr_t capacity,
                              eOres_fp_voidp_uint32_t item_init, uint32_t init_par,  
                              eOres_fp_voidp_voidp_t item_copy, eOres_fp_voidp_t item_clear)
{
    EOvector *retptr = NULL;
    uint8_t *start = NULL;
    uint8_t *obj = NULL;
    eOsizecntnr_t i = 0; 
    eOmempool_alignment_t align = eo_mempool_align_08bit;


    // i get the memory for the object. no need to check versus NULL because the memory pool already does it
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOvector), 1);



    // now the obj has valid memory. i need to initialise it with user-defined data,
    
    retptr->size                = 0;

    eo_errman_Assert(eo_errman_GetHandle(), (0 != item_size), s_eobj_ownname, "item_size is zero");
    eo_errman_Assert(eo_errman_GetHandle(), (0 != capacity), s_eobj_ownname, "capacity is zero");

    retptr->item_size           = item_size;
    retptr->capacity            = capacity;
    retptr->item_copy_fn        = item_copy;
    retptr->item_clear_fn       = item_clear;

    // now we get memory for copying objects inside
    if(1 == item_size)
    {
        align = eo_mempool_align_08bit;
    }
    else if(2 == item_size)
    {
        align = eo_mempool_align_16bit;
    }
    else
    {   // use 4-bytes alignment for everything else
        align = eo_mempool_align_32bit;
    }

    // here is the memory from the correct memory pool
    retptr->item_array_data  = eo_mempool_GetMemory(eo_mempool_GetHandle(), align, item_size, capacity);     
    
    
    if(NULL != item_init)
    {
        start = (uint8_t*) (retptr->item_array_data);
        for(i=0; i<capacity; i++) 
        {
            // cast to uint32_t to tell the reader that index of array start[] can be bigger than max eOsizecntnr_t
            obj = &start[(uint32_t)i * item_size];
            // construct each item 
            item_init(obj, init_par);
        }
    }
    else 
    {
        // clean items all together
        memset(retptr->item_array_data, 0, retptr->capacity*retptr->item_size);
    } 

    return(retptr);

    
}


extern eOsizecntnr_t eo_vector_Capacity(EOvector * vector) 
{
    if(NULL == vector) 
    {
        return(0);    
    }
    
    return(vector->capacity);    
}


extern void eo_vector_PushBack(EOvector * vector, void *p) 
{
    // here we require uint8_t to access item_array_data because we work with bytes.
    uint8_t *start = NULL;
        
    if((NULL == vector) || (NULL == p)) 
    {    
        // invalid data
        return;    
    }
    
    if(vector->capacity == vector->size) 
    { 
        // vector is full
        return;
    }
    
    
    start = (uint8_t*) (vector->item_array_data);
    // cast to uint32_t to tell the reader that index of array start[] can be bigger than max eOsizecntnr_t
    start = &start[(uint32_t)vector->size * vector->item_size]; 
    
    if(NULL != vector->item_copy_fn) 
    {
        vector->item_copy_fn(start, p);
    }
    else
    {
        memcpy(start, p, vector->item_size);
    }
    
    vector->size ++;
    
    return; 
}


extern void * eo_vector_Back(EOvector * vector) 
{
    
    // here we require uint8_t to access item_array_data because we work with bytes.
    uint8_t *start = NULL;
    
    if(NULL == vector) 
    {
        // invalid vector. return NULL
        return(NULL);    
    }
    
    if(0 == vector->size) 
    { 
        // vector is empty. return NULL
        return(start);     
    }
    
 
    start = (uint8_t*) (vector->item_array_data);
    start = &start[(uint32_t)(vector->size-1) * vector->item_size];
    
    return((void*) start);         
}


extern void eo_vector_PopBack(EOvector * vector) 
{
    // here we require uint8_t to access item_array_data because we work with bytes.
    uint8_t *start = NULL;

    
    if(NULL == vector) 
    {
        // invalid vector
        return;    
    }
    
    if(0 == vector->size) 
    { 
        // vector is empty
        return;     
    }


    start = (uint8_t*) (vector->item_array_data);
    start = &start[(uint32_t)(vector->size - 1) * vector->item_size];            
    
    if(NULL != vector->item_clear_fn) 
    {
        vector->item_clear_fn(start);
    } 
    else 
    { 
        // clean the removed item
        s_eo_vector_default_clear(start, vector->item_size);
    }

                
    vector->size --;

}


extern eOsizecntnr_t eo_vector_Size(EOvector * vector) 
{
    if(NULL == vector) 
    {
        // invalid vector
        return(0);    
    }
    
    return(vector->size);        
}


extern void eo_vector_Clear(EOvector * vector) 
{
    // here we require uint8_t to access item_array_data because we work with bytes.
    uint8_t *start = NULL;
    uint8_t *obj = NULL;
    eOsizecntnr_t i = 0;        
    
    
    if(NULL == vector) 
    {
        // invalid vector
        return;    
    }

    if(0 == vector->size) 
    { 
        // vector is empty
        return;     
    }
    
    if(NULL != vector->item_clear_fn) 
    {
        start = (uint8_t*) (vector->item_array_data);
        for(i=0; i<vector->size; i++) 
        {
            // cast to uint32_t to tell the reader that index of array start[] can be bigger than max eOsizecntnr_t
            obj = &start[(uint32_t)i * vector->item_size];
            vector->item_clear_fn(obj);
        }
        
    }
    else
    {  
        // clean all items
        s_eo_vector_default_clear(vector->item_array_data, vector->size*vector->item_size);
    }
    
    vector->size     = 0;

}


extern void * eo_vector_At(EOvector * vector, eOsizecntnr_t pos) 
{
    // here we require uint8_t to access item_array_data because we work with bytes.
    uint8_t *start = NULL;
    
    if(NULL == vector) 
    {
        return(NULL);    
    }
    
    if(pos >= vector->size) 
    { 
        // vector does not have any element in pos
        return(NULL);     
    }
    
   
    start = (uint8_t*) (vector->item_array_data);
    // cast to uint32_t to tell the reader that index of array start[] can be bigger than max eOsizecntnr_t
    start = &start[(uint32_t)pos * vector->item_size];
    
    return((void*) start);         
}


extern void eo_vector_Assign(EOvector * vector, void *p, eOsizecntnr_t pos) 
{
    // here we require uint8_t to access item_array_data because we work with bytes.
    uint8_t *start = NULL;
        
    if((NULL == vector) || (NULL == p)) 
    {    
        // invalid data
        return;    
    }
    
    if(pos >= vector->capacity) 
    { 
        // beyond the capacity of the vector
        return;
    }
 
    
    if(pos >= vector->size)
    {
        eo_vector_Resize(vector, pos+1); 
    }
    
    // now fill the pos-th position w/ object p
    
    
    start = (uint8_t*) (vector->item_array_data);
    // cast to uint32_t to tell the reader that index of array start[] can be bigger than max eOsizecntnr_t
    start = &start[(uint32_t)pos * vector->item_size]; 
    
    if(NULL != vector->item_copy_fn) 
    {
        vector->item_copy_fn(start, p);
    }
    else
    {
        memcpy(start, p, vector->item_size);
    }
    
    return; 
}


extern void eo_vector_Resize(EOvector * vector, eOsizecntnr_t size) 
{
    // here we require uint8_t to access item_array_data because we work with bytes.
    uint8_t *start = NULL;
    uint8_t *obj = NULL;
    eOsizecntnr_t first;
    eOsizecntnr_t last;

    eOsizecntnr_t i = 0;        
    
    
    if(NULL == vector) 
    {
        // invalid vector
        return;    
    }
    

    
    if(size == vector->size)
    {
        return; // nothing to do
    }

    if(size > vector->capacity)
    {
        return; // nothing to do
    }
    
    if(size < vector->size)
    {
        first   = size;
        last    = vector->size;
    }
    else
    {
        first   = vector->size;
        last    = size;
    }
    
    // new size
    vector->size = size;
    
    
    // clear the removed elements or the added ones 
    
    if(NULL != vector->item_clear_fn) 
    {
        start = (uint8_t*) (vector->item_array_data);
        for(i=first; i<last; i++) 
        {
            // cast to uint32_t to tell the reader that index of array start[] can be bigger than max eOsizecntnr_t
            obj = &start[(uint32_t)i * vector->item_size];
            vector->item_clear_fn(obj);
        }
    }
    else
    {  
        // clean all items
        start = (uint8_t*) (vector->item_array_data);
        s_eo_vector_default_clear(&start[(uint32_t)(first*vector->item_size)], (last-first)*vector->item_size);
        //memset(&start[(uint32_t)(first*vector->item_size)], 0, (last-first)*vector->item_size);
    }    

}


extern eObool_t eo_vector_Full(EOvector * vector) 
{
    if(NULL == vector) 
    {
        // invalid deque
        return(eobool_true);    
    }
    
    return((vector->size == vector->capacity) ? (eobool_true) : (eobool_false));        
}

extern eObool_t eo_vector_Empty(EOvector * vector) 
{
    if(NULL == vector) 
    {
        // invalid deque
        return(eobool_true);    
    }
    
    return((vector->size == 0) ? (eobool_true) : (eobool_false));        
}

extern eObool_t eo_vector_Find(EOvector * vector, void *p, eOsizecntnr_t *index)
{
    eOsizecntnr_t i = 0;
    uint8_t *datainside;

    if((NULL == vector) || (p == NULL) || (0 == vector->size)) 
    {
        // invalid deque
        return(eobool_false);    
    }

    
    for(i=0, datainside = (uint8_t*) (vector->item_array_data); i<vector->size; i++, datainside += vector->item_size)
    {
        if(0 == memcmp(p, datainside, vector->item_size))
        {
            if(NULL != index)
            {
                *index = i;
            }
            return(eobool_true);
        }
    }

    return(eobool_false);
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



