// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMFIFOPRODCONS_H_
#define _EOMFIFOPRODCONS_H_


// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       EOMfifoProdCons.h
    @brief      This header file implements public interface to EOMfifoProdCons object.
    @author     valentina.gaggero@iit.it
    @date       12/13/2011
**/

/** @defgroup Object EOMfifoProdCons
    EOMprodConsSharedData is a conteiner data object.
    it provides data exchange mechanism between 2 task: a producer and a consumer; the
    first produces some data and must give them to the second one in thread safety mode.
  
    @todo put documentation proper to the entity.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EOcommon.h"
#include "EOMtask.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 

/** @typedef    typedef struct Object_hid Object
    @brief      Object is an opaque struct. It is used to implement data abstraction for the generic
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOMfifoProdCons_hid EOMfifoProdCons;


    
// - declaration of extern public variables, ...deprecated: better using use _get/_set instead ------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

/** @fn         extern EOMprodConsSharedData* EOMprodConsSharedData_New(eOsizeitem_t item_size, eOsizecntnr_t capacity)
    @brief      Creates a new EOMprodConsSharedData.
    @param      item_size              size of an item saved in the shared data.
    @param      capacity               number of item that the shared data can contain.
    @return     The pointer to the required object.The pointer is always not NULL. Any error in parameters
                will cause a call to the error manager.
 **/
extern EOMfifoProdCons* eom_fifoProdCons_New(eOsizeitem_t item_size, eOsizecntnr_t capacity);

 
/** @fn         extern eOresult_t EOMprodConsSharedData_ProducerRegister(EOMprodConsSharedData* pcShData, EOMtask *producer, void (*signal_to_producer)(void));
    @brief      This function must be invoked by producer task and let it to register itself and configure 
                callback that will be invoked when consumer has cosumed an item.
    @param      pcShData                Pointer to EOMprodConsSharedData object.
    @param      callbackFunc        Callback used by the object to advise the producer. It will be invoked when consumer has removednan item
                                        (GET operation has been performed)
    @param      arg                     pinter to argument, that will be passed to the callback @e signalToProducer 
    @return     eores_OK upon success, eores_NOK_nullpointer if pcShData is NULL.
 **/
extern eOresult_t eom_fifoProdCons_eom_fifoProdCons_SetCallback_OnGet(EOMfifoProdCons *p, eOcallback_t callbackFunc, void *arg);


/** @fn         extern eOresult_t EOMprodConsSharedData_ConsumerRegister(EOMprodConsSharedData* pcShData, EOMtask *consumer, void (*signal_to_consumer)(void))
    @brief      This function must be invoked by consumer task and let it to register itself and configure 
                callback that will be invoked when producer has insert an item.
    @param      pcShData                Pointer to EOMprodConsSharedData object.
    @param      callbackFunc        Callback used by the object to advise the consumer. It will be invoked when producer has inserted an item
                                        (SET operation has been performed)
    @param      arg                     pinter to argument, that will be passed to the callback @e signalToconsumer 
    @return     eores_OK upon success, eores_NOK_nullpointer if pcShData is NULL.
 **/
extern eOresult_t eom_fifoProdCons_SetCallback_OnSet(EOMfifoProdCons *p, eOcallback_t callbackFunc, void *arg);



/** @fn         extern eOresult_t EOMprodConsSharedData_Put(EOMprodConsSharedData* pcShData, void *item, eOreltime_t tout)
    @brief      This function must be invoked by producer task and let it to insert an item in the shared memory.
                Automatically, the function advise the consumer. 
    @param      pcShData                Pointer to EOMprodConsSharedData object.
    @param      consumer                Pointer to the consumer task.
    @param      signal_to_consumer      Cthat will be invoked when producer has insert an item
    @return     eores_OK upon success, eores_NOK_nullpointer if pcShData or consumer are NULL.
 **/
extern eOresult_t eom_fifoProdCons_Put(EOMfifoProdCons *p, void *item, eOreltime_t tout);


extern eOresult_t eom_fifoProdCons_Get(EOMfifoProdCons *p, void *item, eOreltime_t tout);


extern eOresult_t eom_fifoProdCons_GetItemInfo(EOMfifoProdCons *p, eOsizeitem_t *item_size, eOsizecntnr_t *capacity);





// - doxy end ---------------------------------------------------------------------------------------------------------

/** @}            
    end of group object  
 **/

 
#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


