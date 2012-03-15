
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

#include "EOvectorMutex.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOvectorMutex_hid.h" 


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

static const char s_eobj_ownname[] = "EOvectorMutex";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern EOvectorMutex * eo_vectorMutex_New(eOsizeitem_t item_size, eOsizecntnr_t capacity,
                              eOres_fp_voidp_uint32_t item_init, uint32_t init_par,  
                              eOres_fp_voidp_voidp_t item_copy, eOres_fp_voidp_t item_clear,
                              EOVmutexDerived *mutex)
{
    EOvectorMutex *retptr = NULL;

    // i get the memory for the object. no need to check versus NULL because the memory pool already does it
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOvectorMutex), 1);

    // now the obj has valid memory. i need to initialise it with user-defined data,
    retptr->vec =  eo_vector_New(item_size, capacity, item_init, init_par, item_copy, item_clear);         
    retptr->mutex = mutex;

    return(retptr);    
}


extern eOresult_t eo_vectorMutex_Capacity(EOvectorMutex * vector, eOsizecntnr_t *capacity, eOreltime_t tout) 
{
    
    if(NULL == vector) 
    {
        return(eores_NOK_nullpointer);    
    }

    if(NULL == vector->mutex)    
    {
        // the fifo is not protected with a mutex, thus it is simple.
        *capacity = eo_vector_Capacity(vector->vec);
        return(eores_OK);    
    }

    // If I'm here the vector is protected with a mutex, thus i need to check it
    // the mutex can be any type derived from mutexObj_t ... however, we use polymorphism
    if(eores_OK == eov_mutex_Take(vector->mutex, tout)) 
    {
        *capacity = eo_vector_Capacity(vector->vec);
        eov_mutex_Release(vector->mutex);
        return(eores_OK); 
    }
    else 
    {
        // unfortunately we did not get the mutex for timeout
        return(eores_NOK_timeout);
    }
   
}


extern eOresult_t eo_vectorMutex_PushBack(EOvectorMutex * vector, void *p, eOreltime_t tout) 
{
    eOresult_t res;
           
    if((NULL == vector) || (NULL == p)) 
    {    
        // invalid data
        return(eores_NOK_nullpointer);    
    }


    if(NULL != vector->mutex)
    {
        if(eores_OK != eov_mutex_Take(vector->mutex, tout)) 
        {
            // unfortunately we did not get the mutex for timeout
            return(eores_NOK_timeout);
        }
    }
    
    if(eobool_true == eo_vector_Full(vector->vec)) 
    { 
        // vector is full
        res = eores_NOK_busy;
    }
    else
    {
        eo_vector_PushBack(vector->vec, p);
        res = eores_OK;
    }
    
    if(NULL != vector->mutex)
    {
        eov_mutex_Release(vector->mutex);
    }

    return(res);

}


extern eOresult_t eo_vectorMutex_Back(EOvectorMutex * vector, void **ppitem, eOreltime_t tout) 
{
    eOresult_t res;
    
    if((NULL == vector) || (NULL == ppitem)) 
    {    
        // invalid data
        return(eores_NOK_nullpointer);    
    }
    
    if(NULL != vector->mutex)
    {
        if(eores_OK != eov_mutex_Take(vector->mutex, tout)) 
        {
            // unfortunately we did not get the mutex for timeout
            return(eores_NOK_timeout);
        }
    }
    
    if(eobool_true == eo_vector_Full(vector->vec)) 
    { 
        // vector is full
        res = eores_NOK_busy;
    }
    else
    {
        *ppitem = eo_vector_Back(vector->vec);
        res = eores_OK;
    }
    
    if(NULL != vector->mutex)
    {
        eov_mutex_Release(vector->mutex);
    }

    return(res);
    
}


extern eOresult_t eo_vectorMutex_PopBack(EOvectorMutex * vector, eOreltime_t tout) 
{
    eOresult_t res; 
       
    if(NULL == vector)
    {    
        // invalid data
        return(eores_NOK_nullpointer);    
    }
    
    if(NULL != vector->mutex)
    {
        if(eores_OK != eov_mutex_Take(vector->mutex, tout)) 
        {
            // unfortunately we did not get the mutex for timeout
            return(eores_NOK_timeout);
        }
    }
    
    if(eobool_true == eo_vector_Empty(vector->vec)) 
    { 
        // vector is full
        res = eores_NOK_nodata;
    }
    else
    {
        eo_vector_PopBack(vector->vec);
        res = eores_OK;
    }
    
    if(NULL != vector->mutex)
    {
        eov_mutex_Release(vector->mutex);
    }

    return(res);

}


extern eOresult_t eo_vectorMutex_Size(EOvectorMutex * vector, eOsizecntnr_t *size, eOreltime_t tout) 
{
    eOresult_t res = eores_NOK_generic;

    if(NULL == vector) 
    {
        // invalid vector
        return(eores_NOK_nullpointer);    
    }
        
    
    if(NULL != vector->mutex)
    {
        if(eores_OK != eov_mutex_Take(vector->mutex, tout)) 
        {
            // unfortunately we did not get the mutex for timeout
            return(eores_NOK_timeout);
        }
    }
    
    *size = eo_vector_Size(vector->vec);
    res = eores_OK;
    
    if(NULL != vector->mutex)
    {
        eov_mutex_Release(vector->mutex);
    }

    return(res);
      
}


extern eOresult_t eo_vectorMutex_Clear(EOvectorMutex * vector, eOreltime_t tout) 
{
    eOresult_t res = eores_NOK_generic;

    if(NULL == vector) 
    {
        // invalid vector
        return(eores_NOK_nullpointer);    
    }
        
    
    if(NULL != vector->mutex)
    {
        if(eores_OK != eov_mutex_Take(vector->mutex, tout)) 
        {
            // unfortunately we did not get the mutex for timeout
            return(eores_NOK_timeout);
        }
    }
    
    eo_vector_Clear(vector->vec);
    res = eores_OK;
    
    if(NULL != vector->mutex)
    {
        eov_mutex_Release(vector->mutex);
    }

    return(res);
}


extern eOresult_t eo_vectorMutex_At(EOvectorMutex * vector, eOsizecntnr_t pos, void **ppitem, eOreltime_t tout) 
{
    eOresult_t res;
    
    if((NULL == vector) || (NULL == ppitem)) 
    {    
        // invalid data
        return(eores_NOK_nullpointer);    
    }
    
    if(NULL != vector->mutex)
    {
        if(eores_OK != eov_mutex_Take(vector->mutex, tout)) 
        {
            // unfortunately we did not get the mutex for timeout
            return(eores_NOK_timeout);
        }
    }
    
    *ppitem = eo_vector_At(vector->vec,pos);
    res = eores_OK;
    
    if(NULL != vector->mutex)
    {
        eov_mutex_Release(vector->mutex);
    }

    return(res);     
}


extern eOresult_t eo_vectorMutex_Assign(EOvectorMutex * vector, void *p, eOsizecntnr_t pos, eOreltime_t tout) 
{
    eOresult_t res;
    
    if((NULL == vector) || (NULL == p)) 
    {    
        // invalid data
        return(eores_NOK_nullpointer);    
    }
//TODO: dare errore in questo caso...    
//    if(pos >= vector->capacity) 
//    { 
//        // beyond the capacity of the vector
//        return;
//    }
// 
//    
//    if(pos >= vector->capacity)
//    {
//        eo_vector_Resize(vector, pos+1); 
//    }
    
    // now fill the pos-th position w/ object p
    
    
    if(NULL != vector->mutex)
    {
        if(eores_OK != eov_mutex_Take(vector->mutex, tout)) 
        {
            // unfortunately we did not get the mutex for timeout
            return(eores_NOK_timeout);
        }
    }
    
    eo_vector_Assign(vector->vec, p, pos);
    res = eores_OK;
    
    if(NULL != vector->mutex)
    {
        eov_mutex_Release(vector->mutex);
    }

    return(res); 
}


extern eOresult_t eo_vectorMutex_Resize(EOvectorMutex * vector, eOsizecntnr_t size, eOreltime_t tout) 
{
    eOresult_t res;

//TODO: non si verifica se size == 0    
    if(NULL == vector) 
    {    
        // invalid data
        return(eores_NOK_nullpointer);    
    }
    
   if(NULL != vector->mutex)
    {
        if(eores_OK != eov_mutex_Take(vector->mutex, tout)) 
        {
            // unfortunately we did not get the mutex for timeout
            return(eores_NOK_timeout);
        }
    }
    
    eo_vector_Resize(vector->vec, size);
    res = eores_OK;
    
    if(NULL != vector->mutex)
    {
        eov_mutex_Release(vector->mutex);
    }

    return(res); 
}


extern eOresult_t eo_vectorMutex_Full(EOvectorMutex * vector, eObool_t *isFull, eOreltime_t tout)
{  
    if(NULL == vector) 
    {
        return(eores_NOK_nullpointer);    
    }


    if(NULL == vector->mutex)    
    {
        // the fifo is not protected with a mutex, thus it is simple.
        *isFull = eo_vector_Full(vector->vec); 
        return(eores_OK);    
    }

    // If I'm here the vector is protected with a mutex, thus i need to check it
    // the mutex can be any type derived from mutexObj_t ... however, we use polymorphism
    if(eores_OK == eov_mutex_Take(vector->mutex, tout)) 
    {
        *isFull = eo_vector_Full(vector->vec);
        eov_mutex_Release(vector->mutex);
        return(eores_OK); 
    }
    else 
    {
        // unfortunately we did not get the mutex for timeout
        return(eores_NOK_timeout);
    }
   
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



