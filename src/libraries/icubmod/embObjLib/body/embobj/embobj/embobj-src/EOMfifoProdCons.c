// --------------------------------------------------------------------------------------------------------------------
// - doxy
// --------------------------------------------------------------------------------------------------------------------
/** @file       EOMfifoProdCons.h
    @brief      @brief      This file contains internal implementation for the EOMfifoProdCons object.
    @author     valentina.gaggero@iit.it
    @date       12/13/2011
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------
#include "stdlib.h"     // to see NULL, calloc etc.
#include "EOtheErrorManager.h"
#include "EOtheMemoryPool.h"
#include "EOMmutex.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------
#include "EOMfifoProdCons.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
#include "EOMfifoProdCons_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables. deprecated: better using _get(), _set() on static variables 
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
static const char s_eobj_ownname[] = "EOMfifoProdCons";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern EOMfifoProdCons* eom_fifoProdCons_New(eOsizeitem_t item_size, eOsizecntnr_t capacity)
{
    EOMfifoProdCons *p = NULL;

    eo_errman_Assert(eo_errman_GetHandle(), (0 != item_size), s_eobj_ownname, "item_size is zero");
    eo_errman_Assert(eo_errman_GetHandle(), (0 != capacity), s_eobj_ownname, "capacity is zero");
    
    p = (EOMfifoProdCons*)eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOMfifoProdCons), 1);
    
    p->fifo = eo_fifo_New(item_size, capacity, NULL, 0, NULL, NULL,
                            eom_mutex_New());

    /* Note: here it is not necessary verify if memory has been allacated really, 
       because in case of error, the ErrorManager is called automatically. */ 
    p->item_size = item_size;
    p->capacity = capacity;    
    
    p->signalToProducer = NULL;
    p->argprod = NULL;
    
    p->signalToConsumer = NULL;
    p->argcons = NULL;
    
    return(p);

}


extern eOresult_t eom_fifoProdCons_GetItemInfo(EOMfifoProdCons *p, eOsizeitem_t *item_size, eOsizecntnr_t *capacity)
{
    if((NULL == p) || (NULL== item_size) || (NULL == capacity))
    {
        return(eores_NOK_nullpointer);
    }
    *item_size = p->item_size;
    *capacity = p->capacity;
    
    return(eores_OK);
}


extern eOresult_t eom_fifoProdCons_SetCallback_OnGet(EOMfifoProdCons *p, eOcallback_t callbackFunc, void *arg)
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }
    //TODO: devo garantire mutua esclusione su questi campi??
    //A meno di bachi e  se usato correttamente il producer e il consumer usano campi differenti anche se della stessa struttura.    
    p->signalToProducer = callbackFunc;
    p->argprod = arg;

    return(eores_OK);
}


extern eOresult_t eom_fifoProdCons_SetCallback_OnSet(EOMfifoProdCons *p, eOcallback_t callbackFunc, void *arg)
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }
    //TODO: devo garantire mutua esclusione su questi campi??
    //A meno di bachi e  se usato correttamente il producer e il consumer usano campi differenti anche se della stessa struttura.    
    p->signalToConsumer = callbackFunc;
    p->argcons = arg;

    return(eores_OK);
}



extern eOresult_t eom_fifoProdCons_Put(EOMfifoProdCons *p, void *item, eOreltime_t tout)
{
    eOresult_t res;

    if( (NULL == p) ||  (NULL == item))
    {
        return(eores_NOK_nullpointer);
    }
    
    res = eo_fifo_Put(p->fifo, item, tout);

    if(eores_OK == res)
    {
        if(NULL != p->signalToConsumer)
        {
            p->signalToConsumer(p->argcons);   
        }
    }
    return(res);
}



extern eOresult_t eom_fifoProdCons_Get(EOMfifoProdCons *p, void *item, eOreltime_t tout)
{
    eOresult_t res;

    if( (NULL == p) ||  (NULL == item))
    {
        return(eores_NOK_nullpointer);
    }
    
    res = eo_fifo_GetRem(p->fifo, item, tout);
    
    if(eores_OK == res)
    {
        if(NULL != p->signalToProducer)
        {
            p->signalToProducer(p->argprod);   
        }
    }
    return(res);
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



