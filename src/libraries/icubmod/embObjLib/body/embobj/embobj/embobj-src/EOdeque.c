
/* @file       EOdeque.c
	@brief      This file contains internal implementation of a deque object.
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

#include "EOdeque.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOdeque_hid.h" 


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
EO_static_inline void s_eo_deque_default_clear(void *p, uint16_t size)
{
     memset(p, 0, size);
}
#else
EO_static_inline void s_eo_deque_default_clear(void *p, uint16_t size)
{
}    
#endif


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOdeque";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern EOdeque * eo_deque_New(eOsizeitem_t item_size, eOsizecntnr_t capacity,
                              eOres_fp_voidp_uint32_t item_init, uint32_t init_par,  
                              eOres_fp_voidp_voidp_t item_copy, eOres_fp_voidp_t item_clear)
{
    EOdeque *retptr = NULL;
    uint8_t *start = NULL;
    uint8_t *obj = NULL;
    uint32_t pos = 0;
    eOsizecntnr_t i = 0; 
    eOmempool_alignment_t align = eo_mempool_align_08bit;


    // i get the memory for the object. no need to check versus NULL because the memory pool already does it
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOdeque), 1);



    // now the obj has valid memory. i need to initialise it with user-defined data,
    
    retptr->size                = 0;
    retptr->first               = 0;
    retptr->next                = 0;

    eo_errman_Assert(eo_errman_GetHandle(), (0 != item_size), s_eobj_ownname, "item_size is zero");
    eo_errman_Assert(eo_errman_GetHandle(), (0 != capacity), s_eobj_ownname, "capacity is zero");

    retptr->item_size           = item_size;
    retptr->max_items           = capacity;
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
        pos = 0;
        for(i=0; i<capacity; i++) 
        {
            // cast to uint32_t to tell the reader that index of array start[] can be bigger than max eOsizecntnr_t
            obj = &start[(uint32_t)pos * item_size];
            // construct each item 
            item_init(obj, init_par);
            // go to next position
            pos = (pos + 1) % (capacity);
        }
    }
    else 
    {
        // clean items all together
        memset(retptr->item_array_data, 0, retptr->max_items*retptr->item_size);
    } 

    return(retptr);

    
}


extern eOsizecntnr_t eo_deque_Capacity(EOdeque * deque) 
{
    if(NULL == deque) 
    {
        return(0);    
    }
    
    return(deque->max_items);    
}


extern eObool_t eo_deque_Full(EOdeque * deque) 
{
    if(NULL == deque) 
    {
        // invalid deque
        return(eobool_true);    
    }
    
    return((deque->size == deque->max_items) ? (eobool_true) : (eobool_false));        
}


extern eObool_t eo_deque_Empty(EOdeque * deque) 
{
    if(NULL == deque) 
    {
        // invalid deque
        return(eobool_true);    
    }
    
    return((0 == deque->size) ? (eobool_true) : (eobool_false));        
}
 

extern void eo_deque_PushBack(EOdeque * deque, void *p) 
{
    // here we require uint8_t to access item_array_data because we work with bytes.
    uint8_t *start = NULL;
        
    if((NULL == deque) || (NULL == p)) 
    {    
        // invalid data
        return;    
    }
    
    if(deque->max_items == deque->size) 
    { 
        // deque is full
        return;
    }
    
    
    start = (uint8_t*) (deque->item_array_data);
    // cast to uint32_t to tell the reader that index of array start[] can be bigger than max eOsizecntnr_t
    start = &start[(uint32_t)deque->next * deque->item_size]; 
    
    if(NULL != deque->item_copy_fn) 
    {
        deque->item_copy_fn(start, p);
    }
    else
    {
        memcpy(start, p, deque->item_size);
    }
    
    deque->next = (deque->next + 1) % (deque->max_items);
    deque->size ++;
    
    return; 
}


extern void eo_deque_PushFront(EOdeque * deque, void *p) 
{
    // here we require uint8_t to access item_array_data because we work with bytes.
    uint8_t *start = NULL;
    // we use uint32_t because .... see note xxx.
    uint32_t pos = 0;
            
    if((NULL == deque) || (NULL == p)) 
    {
        // invalid data
        return;    
    }
    
    if(deque->max_items == deque->size) 
    { 
        // deque is full
        return;
    }
    
    // note xxx.
    // cast to uint32_t is used to avoid problems when (max_items + first - 1) goes beyond max value of 
    // eOsizecntnr_t.
    // example for eOsizecntnr_t equal to uint8_t, max_items = 255, and first = 10: 
    // - we want that 265 is correctly represented before it is extrated the remainder of division by 255, 
    // - thus we must use a larger variable for pos .... uint32_t is large enough.
    pos = (uint32_t)(deque->max_items + deque->first - 1) % deque->max_items;  
    start = (uint8_t*) (deque->item_array_data);
    // cast to uint32_t to tell the reader that index of array start[] can be bigger than max eOsizecntnr_t
    start = &start[(uint32_t)pos * deque->item_size];
    
    if(NULL != deque->item_copy_fn) 
    {
        deque->item_copy_fn(start, p);
    }
    else
    {
        memcpy(start, p, deque->item_size);
    }
    
    // cast is used to avoid possible compiler complains ... the cast is safe because ... see note xxx
    deque->first = (eOsizecntnr_t)pos;
    deque->size ++;
    
    return; 
}


extern void * eo_deque_Front(EOdeque * deque) 
{
    // here uint8_t is required to access item_array_data because we work with bytes.
    uint8_t *start = NULL;
    
    if(NULL == deque) 
    {
        // invalid data
        return(NULL);    
    }
    
    if(0 == deque->size) 
    { 
        // deque is empty. returns NULL
        return(start);     
    }
    
    start = (uint8_t*) (deque->item_array_data);
    // cast to uint32_t to tell the reader that index of array start[] can be bigger than max eOsizecntnr_t
    start = &start[(uint32_t)deque->first * deque->item_size];
    
    return((void*) start);         
}


extern void eo_deque_PopFront(EOdeque * deque) 
{
    // here we require uint8_t to access item_array_data because we work with bytes.
    uint8_t *start = NULL;
    
    if(NULL == deque) 
    {
        // invalid data
        return;    
    }
    
    if(0 == deque->size) 
    { 
        // deque is empty
        return;     
    }

    start = (uint8_t*) (deque->item_array_data);
    // cast to uint32_t to tell the reader that index of array start[] can be bigger than max eOsizecntnr_t
    start = &start[(uint32_t)deque->first * deque->item_size];

    if(NULL != deque->item_clear_fn) 
    {
        deque->item_clear_fn(start);
    } 
    else 
    { 
        // clean the removed item
        s_eo_deque_default_clear(start, deque->item_size);
    }
    
    // in here there is no need to cast to a bigger integer. 
    // suppose uint8_t: even if max_items is 255, deque->first can reach at most 254. 
    // thus 254+1 = 255 can still be managed. 
    deque->first = (deque->first + 1) % (deque->max_items);
    deque->size --;    
    
    
}


extern void * eo_deque_Back(EOdeque * deque) 
{
    
    // here we require uint8_t to access item_array_data because we work with bytes.
    uint8_t *start = NULL;
    // we use uint32_t because .... see note xxx.
    uint32_t pos = 0;
    
    if(NULL == deque) 
    {
        // invalid deque. return NULL
        return(NULL);    
    }
    
    if(0 == deque->size) 
    { 
        // deque is empty. return NULL
        return(start);     
    }
    
    // cast to uint32_t ... see note xxx
    pos = (uint32_t)(deque->next + deque->max_items - 1) % deque->max_items; 
    start = (uint8_t*) (deque->item_array_data);
    // cast to uint32_t to tell the reader that index of array start[] can be bigger than max eOsizecntnr_t
    start = &start[(uint32_t)pos * deque->item_size];
    
    return((void*) start);         
}


extern void eo_deque_PopBack(EOdeque * deque) 
{
    // here we require uint8_t to access item_array_data because we work with bytes.
    uint8_t *start = NULL;
    // we use uint32_t because .... see note xxx.
    uint32_t pos = 0;
    
    if(NULL == deque) 
    {
        // invalid deque
        return;    
    }
    
    if(0 == deque->size) 
    { 
        // deque is empty
        return;     
    }

    // cast to uint32_t. see note xxx.
    pos = (uint32_t)(deque->next + deque->max_items - 1) % deque->max_items; 

    start = (uint8_t*) (deque->item_array_data);
    // cast to uint32_t to tell the reader that index of array start[] can be bigger than max eOsizecntnr_t
    start = &start[(uint32_t)pos * deque->item_size];            
    
    if(NULL != deque->item_clear_fn) 
    {
        deque->item_clear_fn(start);
    } 
    else 
    { 
        // clean the removed item
       s_eo_deque_default_clear(start, deque->item_size);
    }

                
    // cast is used to avoid possible compiler complains ... the cast is safe because ... see note xxx
    deque->next = (eOsizecntnr_t)pos;
    deque->size --;    
    
}


extern eOsizecntnr_t eo_deque_Size(EOdeque * deque) 
{
    if(NULL == deque) 
    {
        // invalid deque
        return(0);    
    }
    
    return(deque->size);        
}


extern void eo_deque_Clear(EOdeque * deque) 
{
    // here we require uint8_t to access item_array_data because we work with bytes.
    uint8_t *start = NULL;
    uint8_t *obj = NULL;
    // we use uint32_t because .... see note xxx.
    uint32_t pos = 0;
    eOsizecntnr_t i = 0;        
    
    
    if(NULL == deque) 
    {
        // invalid deque
        return;    
    }

    if(0 == deque->size) 
    { 
        // deque is empty
        return;     
    }
    
    if(NULL != deque->item_clear_fn) 
    {
        start = (uint8_t*) (deque->item_array_data);
        pos = deque->first;
        for(i=0; i<deque->size; i++) 
        {
            // cast to uint32_t to tell the reader that index of array start[] can be bigger than max eOsizecntnr_t
            obj = &start[(uint32_t)pos * deque->item_size];
            deque->item_clear_fn(obj);
            pos = (pos + 1) % (deque->max_items);
        }
        
    }
    else
    {  
        // clean all items
        memset(deque->item_array_data, 0, deque->max_items*deque->item_size);
    }
    
    deque->size     = 0;
    deque->first     = 0;
    deque->next     = 0;
    
}


extern void * eo_deque_At(EOdeque * deque, eOsizecntnr_t pos) 
{
    // here we require uint8_t to access item_array_data because we work with bytes.
    uint8_t *start = NULL;
    
    if(NULL == deque) 
    {
        return(NULL);    
    }
    
    if(pos >= deque->size) 
    { 
        // deque does not have any element in pos
        return(start);     
    }
    
    // we use uint32_t because .... see note xxx.
    pos = (uint32_t)(pos + deque->first) % (deque->max_items);
    
    start = (uint8_t*) (deque->item_array_data);
    // cast to uint32_t to tell the reader that index of array start[] can be bigger than max eOsizecntnr_t
    start = &start[(uint32_t)pos * deque->item_size];
    
    return((void*) start);         
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

extern void eo_deque_hid_QuickPopFront(EOdeque * deque) 
{
    // here we require uint8_t to access item_array_data because we work with bytes.
    uint8_t *start = NULL;

// remove controls in order to speed-up things    
//    if(NULL == deque) 
//    {
//        // invalid data
//        return;    
//    }
//    
//    if(0 == deque->size) 
//    { 
//        // deque is empty
//        return;     
//    }

    start = (uint8_t*) (deque->item_array_data);
    // cast to uint32_t to tell the reader that index of array start[] can be bigger than max eOsizecntnr_t
    start = &start[(uint32_t)deque->first * deque->item_size];

    if(NULL != deque->item_clear_fn) 
    {
        deque->item_clear_fn(start);
    } 
    else 
    { 
        // clean the removed item
        s_eo_deque_default_clear(start, deque->item_size);
    }
    
    // in here there is no need to cast to a bigger integer. 
    // suppose uint8_t: even if max_items is 255, deque->first can reach at most 254. 
    // thus 254+1 = 255 can still be managed. 
    deque->first = (deque->first + 1) % (deque->max_items);
    deque->size --;          
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section




// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



