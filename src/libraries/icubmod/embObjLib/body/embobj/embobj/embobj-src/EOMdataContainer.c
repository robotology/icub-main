// --------------------------------------------------------------------------------------------------------------------
// - doxy
// --------------------------------------------------------------------------------------------------------------------

/* @file       EOMdataContainer.c
    @brief      This file contains internal implementation for the utiliteis a SW entity.
    @author     valentina.gaggero@iit.it
    @date       16/12/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------
#include "stdlib.h"     // to see NULL, calloc etc.
#include "string.h"

#include "EOMmutex.h"
#include "EOtheMemoryPool.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------
#include "EOMdataContainer.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
#include "EOMdataContainer_hid.h" 
#include "EOvector_hid.h"

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
//static const char s_eobj_ownname[] = "EOMdataContainer";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------
extern EOMdataContainer* eom_dataContainer_New(eOsizecntnr_t capacity)
{
    EOMdataContainer *retptr = NULL;

    if(0 == capacity)
    {
        return(retptr);
    }
        
    retptr = (EOMdataContainer*)eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOMdataContainer), 1);

    retptr->vector = eo_vector_New(sizeof(EOMdataContainer_itemStruct_hid_t), capacity,
                                   NULL, 0, NULL, NULL);
    
    retptr->readers_mutex = eom_mutex_New();
    retptr->obj_mutex = eom_mutex_New();
    
    return(retptr);

}


extern eOresult_t eom_dataContainer_AddItem(EOMdataContainer* p, eOsizeitem_t itemsize, void *itemdata, eOdataContainerIndex_t itemIndex)
{
    EOMdataContainer_itemStruct_hid_t item;
    EOMdataContainer_itemStruct_hid_t *item_ptr;
    eOmempool_alignment_t align = eo_mempool_align_32bit;

    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }
    
    if( (0 == itemsize) || (itemIndex >= p->vector->capacity))
    {
        return(eores_NOK_generic);
    }
    

    if(1 == itemsize)
    {
        align = eo_mempool_align_08bit;
    }
    else if(2 == itemsize)
    {
        align = eo_mempool_align_16bit;
    }

    item.datasize = itemsize;
    item.dataptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), align, itemsize, 1);
    if(NULL != itemdata)
    {
        memcpy( item.dataptr, itemdata, itemsize);    
    }

    eom_mutex_Take(p->obj_mutex, eok_reltimeINFINITE);    
    //eo_vector_Assign(p->vector, (void*)&item, (eOsizecntnr_t)index);
    item_ptr = (EOMdataContainer_itemStruct_hid_t*)p->vector->item_array_data;
    item_ptr = &item_ptr[itemIndex];
    memcpy(item_ptr , &item, sizeof(EOMdataContainer_itemStruct_hid_t)); 
    p->vector->size++;
    eom_mutex_Release(p->obj_mutex);

    return(eores_OK);
}


//extern eOresult_t eom_dataContainer_LinkTo(EOMdataContainer* dataCont, eOsizeitem_t datasize, uint8_t *data)
//{
//   if((NULL == dataCont) || (NULL == data))
//   {
//        return(eores_NOK_nullpointer);
//   }
//
//   if(0 == datasize)
//   {
//        return(eores_NOK_generic);
//   }
//   //if the object is used correctly, any task is using this obj, so there are not wait to take mutex
//   eom_mutex_Take(dataCont->obj_mutex, eok_reltimeINFINITE);
//   dataCont->data = data;
//   dataCont->datasize = datasize;
//   eom_mutex_Release(dataCont->obj_mutex);
//
//   return(eores_OK);
//
//}



//extern eOresult_t eom_dataContainer_GetSize(EOMdataContainer *dataCont, eOsizeitem_t *datasize)
//{
//   if((NULL == dataCont) || (NULL == datasize))
//   {
//        return(eores_NOK_nullpointer);
//   }
//
//
//   //if the object is used correctly, any task change datasize...
//   eom_mutex_Take(dataCont->obj_mutex, eok_reltimeINFINITE);
//   *datasize = dataCont->datasize;
//   eom_mutex_Release(dataCont->obj_mutex);
//
//   return(eores_OK);
//
//}

extern eOresult_t eom_dataContainer_ReadItem(EOMdataContainer *p, eOdataContainerIndex_t itemIndex, void *data)
{
    EOMdataContainer_itemStruct_hid_t *item_ptr;
    eOresult_t res;

    if((NULL == p) || (NULL == data))
    {
        return(eores_NOK_nullpointer);
    }
    if(itemIndex >= p->vector->capacity)
    {
       return(eores_NOK_generic);
    }
    
    eom_mutex_Take(p->readers_mutex, eok_reltimeINFINITE);
    p->readers_num++;
    if(1 == p->readers_num)
    {
        eom_mutex_Take(p->obj_mutex, eok_reltimeINFINITE);
    }
    eom_mutex_Release(p->readers_mutex);
    
    item_ptr = (EOMdataContainer_itemStruct_hid_t*)p->vector->item_array_data;
    item_ptr = &item_ptr[itemIndex];
    if(NULL != item_ptr->dataptr)
    {
        memcpy(data, item_ptr->dataptr, item_ptr->datasize);
        res = eores_OK;
    }
    else
    {
        res = eores_NOK_nodata;
    }
    
    eom_mutex_Take(p->readers_mutex, eok_reltimeINFINITE);
    p->readers_num--;
    if(0 == p->readers_num)
    {
        eom_mutex_Release(p->obj_mutex);
    }
    eom_mutex_Release(p->readers_mutex);
    
    return(res);

}


extern eOresult_t eom_dataContainer_WriteItem(EOMdataContainer *p, eOdataContainerIndex_t itemIndex, void *data)
{
    EOMdataContainer_itemStruct_hid_t *item_ptr;
    eOresult_t res;

    if((NULL == p) || (NULL == data))
    {
        return(eores_NOK_nullpointer);
    }
    if(itemIndex >= p->vector->capacity)
    {
        return(eores_NOK_generic);
    }

    eom_mutex_Take(p->obj_mutex, eok_reltimeINFINITE);


    item_ptr = (EOMdataContainer_itemStruct_hid_t*)p->vector->item_array_data;
    item_ptr = &item_ptr[itemIndex];
    if(NULL != item_ptr->dataptr)
    {
        memcpy(item_ptr->dataptr, data,  item_ptr->datasize);
        res = eores_OK;
    }
    else
    {
        res = eores_NOK_nodata;
    }

    eom_mutex_Release(p->obj_mutex);

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



