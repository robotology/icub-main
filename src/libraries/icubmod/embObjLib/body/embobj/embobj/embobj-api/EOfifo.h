
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOFIFO_H_
#define _EOFIFO_H_

/** @file       EOfifo.h
    @brief      This header file implements public interface to a fifo queue protected by a mutex.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/

/** @defgroup eo_fifo Object EOfifo
    The EOfifo implements a FIFO queue of generic object items which is also protected by a mutex. 
    It contains an object EOdeque and an object derived from EOVmutex.
    It can be used alone with void * items or can be used inside another object to act such as
    a template in C++.  For example see EOfifoByte.
   
   @{        
 */
 
// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVmutex.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/** @typedef    typedef struct EOfifo_hih EOfifo
    @brief      EOfifo_hid is an opaque struct. It is used to implement data abstraction for the fifo 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOfifo_hid EOfifo;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

/** @fn         extern EOfifo * eo_fifo_New(eOsizeitem_t item_size, eOsizecntnr_t capacity,
                                            eOres_fp_voidp_uint32_t item_init, uint32_t init_arg, 
                                            eOres_fp_voidp_voidp_t item_copy, eOres_fp_voidp_t item_clear,
                                            EOVmutexDerived *mutex)
    @brief      Creates a new EOfifo object able to contain capacity item objects of size item_size bytes. 
    @param      item_size       Size in bytes of the item object.
    @param      capacity        Max number of item objects which can be stored in the fifo.
    @param      item_init       Pointer to a specialised init function for the item object to be called at
                                creation of the object for each contained item with arguments item_init(item, item_par). 
                                If NULL, memory is just set to zero.
    @param      item_par        Argument used for @e item_init(item, item_par).      
    @param      item_copy       Pointer to a constructor copy for the item object to be called at each copy of an item 
                                object inside the EOdeque (NULL if the item object does not require initialisation).
    @param      item_clear      Pointer to a destructor for the item object to be called at each removal of an item 
                                object from the EOdeque (NULL if the object does not require destruction).
    @param      mutex           Pointer to an object derived by mutex which will protect from concurrent access.
                                Note: a pointer to a simple EOmutex can be used but is meaningless. 
    @return     Const pointer to the required EOfifo object. The pointer is always not NULL. Any error in parameters
                will cause a call to the error manager.
 **/
extern EOfifo * eo_fifo_New(eOsizeitem_t item_size, eOsizecntnr_t capacity,
                            eOres_fp_voidp_uint32_t item_init, uint32_t init_arg, 
                            eOres_fp_voidp_voidp_t item_copy, eOres_fp_voidp_t item_clear,
                            EOVmutexDerived *mutex);


/** @fn         extern eOresult_t eo_fifo_Capacity(EOfifo *fifo, eOsizecntnr_t *capacity, eOreltime_t tout)
    @brief      Returns the maximum number of items that the fifo queue can contain.
    @param      fifo            Pointer to the EOfifo object.
    @param      capacity        Pointer to the max number of items that the fifo queue can contain. 
                                Note: the user must pass a &capacity.
    @param      tout      Timeout for the operation in micro-seconds.
    @return     eores_OK upon success, eores_NOK_nullpointer if fifo is NULL, eores_NOK_timeout if the mutex
                was busy within the specified timeout.
 **/
extern eOresult_t eo_fifo_Capacity(EOfifo *fifo, eOsizecntnr_t *capacity, eOreltime_t tout);


/** @fn         extern eOresult_t eo_fifo_Size(EOfifo *fifo, eOsizecntnr_t *size, eOreltime_t tout)
    @brief      Returns the current number of items in the fifo queue.
    @param      fifo            Pointer to the EOfifo object.
    @param      size            Pointer to the number of items in the fifo queue. 
                                Note: the user must pass a &size.
    @param      tout      Timeout for the operation in micro-seconds.
    @return     eores_OK upon success, eores_NOK_nullpointer if fifo is NULL, eores_NOK_timeout if the 
                mutex was busy within the specified timeout.
 **/
extern eOresult_t eo_fifo_Size(EOfifo *fifo, eOsizecntnr_t *size, eOreltime_t tout);


/** @fn         eOresult_t eo_fifo_Put(EOfifo *fifo, void *item, eOreltime_t tout)
    @brief      Copies the object pointed by @e item in the fifo queue.
    @param      fifo            Pointer to the EOfifo object.
    @param      item            Pointer to the object to be copied. 
    @param      tout            Timeout for the operation in micro-seconds.
    @return     eores_OK upon successful copy, eores_NOK_busy if the queue is full, eores_NOK_nullpointer 
                if fifo is NULL, eores_NOK_timeout if the muetx was busy within the specified timeout.
 **/
extern eOresult_t eo_fifo_Put(EOfifo *fifo, void *item, eOreltime_t tout);


/** @fn         extern eOresult_t eo_fifo_Get(EOfifo *fifo, const void **ppitem, eOreltime_t tout)
    @brief      Retrieves a pointer to the first-in object from the fifo queue without removing it
    @param      fifo            Pointer to the EOfifo object.
    @param      ppitem          Address in which the function will copy the retrieved object. 
                                With const void *dekitem = NULL, use  eo_fifo_Get( .., &dekitem, ..).
    @param      tout      Timeout for the operation in micro-seconds.
    @return     eores_OK upon success (retrieval of valid data), eores_NOK_nodata if fifo is empty, 
                eores_NOK_nullpointer if fifo is NULL, eores_NOK_timeout if the mutex was busy within 
                the specified timeout. 
 **/
extern eOresult_t eo_fifo_Get(EOfifo *fifo, const void **ppitem, eOreltime_t tout);


/** @fn         eOresult_t eo_fifo_Rem(EOfifo *fifo, eOreltime_t tout)
    @brief      removes the first-in object in the fifo queue.
    @param      fifo            Pointer to the EOfifo object.
    @param      tout            Timeout for the operation in micro-seconds.
    @return     eores_OK upon success, eores_NOK_nullpointer if fifo is NULL, eores_NOK_timeout if 
                the mutex was busy within the specified timeout.
 **/
extern eOresult_t eo_fifo_Rem(EOfifo *fifo, eOreltime_t tout); 


/** @fn         extern eOresult_t eo_fifo_GetRem(EOfifo *fifo, void *pitem, eOreltime_t tout);
    @brief      Copies in the memory pointed by pitem the fist element of fifo; afetr remove it from fifo.
    @param      fifo            Pointer to the EOfifo object.
    @param      pitem          pointer to memory where will be copied first item data
    @param      tout      Timeout for the operation in micro-seconds.
    @return     eores_OK upon success (retrieval of valid data), eores_NOK_nodata if fifo is empty, 
                eores_NOK_nullpointer if fifo is NULL, eores_NOK_timeout if the mutex was busy within 
                the specified timeout. 
 **/
extern eOresult_t eo_fifo_GetRem(EOfifo *fifo, void *pitem, eOreltime_t tout);


/** @fn         eOresult_t eo_fifo_Clear(EOfifo *fifo, eOreltime_t tout)
    @brief      removes every object in the fifo queue.
    @param      fifo            Pointer to the EOfifo object.
    @param      tout            Timeout for the operation in micro-seconds.
    @return     eores_OK upon success, eores_NOK_nullpointer if fifo is NULL, eores_NOK_timeout
                if the mutex was busy within the specified timeout.
 **/ 
extern eOresult_t eo_fifo_Clear(EOfifo *fifo, eOreltime_t tout);



/** @}            
    end of group eo_fifo  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



