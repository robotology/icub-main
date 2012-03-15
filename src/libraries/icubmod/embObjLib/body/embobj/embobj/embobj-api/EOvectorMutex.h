
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOVECTORMUTEX_H_
#define _EOVECTORMUTEX_H_


/** @file       EOvectorMutex.h
	@brief      This header file implements public interface to a vectorMutex object.
	@author     valentina.gaggero@iit.it
	@date       07/12/2011
**/

/** @defgroup eo_vectorMutex Object EOvectorMutex
    The EOvectorMutex inherits from EOvector object; for this, it allows to manipulate vectors of any item object, implementing the vector<type> template 
    of the standard C++ library. (for more information see EOvector).
    Unlike EOvector, EOvectorMutex provides concurrent access protection in multitask environment.    
    @{		
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/**	@typedef    typedef struct EOvectorMutex_hid EOvectorMutex
 	@brief 		EOvectorMutex is an opaque struct. It is used to implement data abstraction for the vector 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOvectorMutex_hid EOvectorMutex;

    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------


 
/** @fn         extern EOvectorMutex * eo_vectorMutex_New(eOsizeitem_t item_size, eOsizecntnr_t capacity,
                              eOres_fp_voidp_uint32_t item_init, uint32_t init_par, 
                              eOres_fp_voidp_voidp_t item_copy, eOres_fp_voidp_t item_clear,
                              EOVmutexDerived *mutex)
    @brief      Creates a new EOvector object and reserves memory for the items that will be stored in its
                inside, taking it from the memory pool.
    @param      item_size       The size in bytes of the item object managed by the EOvector.
    @param      capacity        The max number of item objects stored by the EOvector.
    @param      item_init       Pointer to a specialised init function for the item object to be called at
                                creation of the object for each contained item with arguments item_init(item, item_par). 
                                If NULL, memory is just set to zero.
    @param      item_par        Argument used for @e item_init(item, item_par).                                
    @param      item_copy       Pointer to a specialised copy function for the item object to be called 
                                at each copy of an item object inside the EOvector with arguments item_copy(dest, orig).
                                If NULL it will be executed a simple memcpy of the size of the item object.                                
    @param      item_clear      Pointer to a specialised remove function for the item object to be called at each 
                                removal of an item object from the EOvector. If NULL the memory inside the container
                                will be simply set to zero.
    @param      mutex           Pointer to an object derived by mutex which will protect from concurrent access.
                                Note: if this param is NULL, this object is equal to EOvector; so if you need a vector data
                                struct without concurrent access protection, it is better you use EOvector object.
    @return     Pointer to the required EOvectorMutex object. The pointer is guaranteed to be always valid and will 
                never be NULL, because failure is managed by the memory pool.
 **/
extern EOvectorMutex * eo_vectorMutex_New(eOsizeitem_t item_size, eOsizecntnr_t capacity,
                              eOres_fp_voidp_uint32_t item_init, uint32_t init_par, 
                              eOres_fp_voidp_voidp_t item_copy, eOres_fp_voidp_t item_clear,
                              EOVmutexDerived *mutex);


/** @fn         extern eOresult_t eo_vectorMutex_Capacity(EOvectorMutex * vector, eOsizecntnr_t*capacity, eOreltime_t tout);
    @brief      Returns the maximum number of item objects that the EOvectorMutex is able to contain.
    @param      vector           Pointer to the EOvectorMutex object.
    @param      capacity        Pointer to the max number of items that the fifo queue can contain. 
                                Note: the user must pass a &capacity.
    @param      tout      Timeout for the operation in micro-seconds.
    @return     ores_OK upon success, eores_NOK_nullpointer if vector is NULL, eores_NOK_timeout if the mutex
                was busy within the specified timeout.
 **/
extern eOresult_t eo_vectorMutex_Capacity(EOvectorMutex * vector, eOsizecntnr_t *capacity, eOreltime_t tout);


/** @fn         extern eOresult_t eo_vectorMutex_Size(EOvector * vector, eOsizecntnr_t *size, eOreltime_t tout)
    @brief      Returns the number of item objects that are currently stored in the EOvector.
    @param      vector          Pointer to the EOvectorMutex object. 
    @param      size            Pointer to the number of items in the vector. 
                                Note: the user must pass a &size.
    @param      tout            Timeout for the operation in micro-seconds.
    @return     eores_OK upon success, eores_NOK_nullpointer if fifo is NULL, eores_NOK_timeout if the 
                mutex was busy within the specified timeout.
 **/
extern eOresult_t eo_vectorMutex_Size(EOvectorMutex * vector, eOsizecntnr_t *size, eOreltime_t tout);


/**  @fn        extern eOresult_t eo_vectorMutex_PushBack(EOvector * vector, void *p, eOreltime_t tout) 
     @brief     Copies the item object pointed by @e p at the back of the EOvectorMutex and calls its constructor 
                @e item_ctor(p) if passed not NULL in eo_vector_New().
     @param     vector           pointer to the EOvectorMutex object.
     @param     p                pointer to the item object to be copied into the EOvector.
     @param     tout             Timeout for the operation in micro-seconds.
     @return    ores_OK upon success, eores_NOK_nullpointer if vector is NULL, eores_NOK_timeout if the mutex
                was busy within the specified timeout.   
 **/
extern eOresult_t eo_vectorMutex_PushBack(EOvectorMutex * vector, void *p, eOreltime_t tout) ;


/** @fn         extern eOresult_t eo_vectorMutex_Back(EOvector * vector, void **ppitem, eOreltime_t tout) 
    @brief      Retrieves a reference to the item object in the back of the EOvectorMutex without removing it. 
    @param      vector           Pointer to the EOvector object.
    @param      ppitem           Address in which the function will copy the retrieved object.
                                 In output it ca value NULL, if there is no data. Vector empty.  
    @param      tout             Timeout for the operation in micro-seconds.
    @return     eores_OK upon success (retrieval of valid data), eores_NOK_nullpointer if vector is NULL, 
                eores_NOK_timeout if the mutex was busy within the specified timeout. 
    @warning    Before use, the returned pointer needs to be casted to the desidered object type.
 **/
extern eOresult_t eo_vectorMutex_Back(EOvectorMutex * vector, void **ppitem, eOreltime_t tout); 


/** @fn         extern eOresult_t eo_vectorMutex_PopBack(EOvector * vector, eOreltime_t tout)
    @brief      Removes the item object from the back of the EOvector, calls its destructor
                @e item_dtor(p) if passed not NULL in eo_vector_new(), and finally sets memory to zero.
    @param      vector           Pointer to the EOvector object. 
    @param      tout             Timeout for the operation in micro-seconds.
    @return     eores_OK upon success, eores_NOK_nullpointer if vector is NULL, eores_NOK_nodata if vector is empty, 
                eores_NOK_timeout if the mutex was busy within the specified timeout. 
    @warning    After this function call, previously obtained references to the removed item object 
                will point to zero-ed data
 **/
extern eOresult_t eo_vectorMutex_PopBack(EOvectorMutex * vector, eOreltime_t tout); 


/** @fn         extern eOresult_t eo_vectorMutex_Clear(EOvector * vector, eOreltime_t tout)
    @brief      Removes all item objects from the EOvectorMutex and for each one calls its destructor 
                @e item_dtor() if passed not NULL in eo_vector_new(). Finally sets memory to zero.
    @param      vector           Pointer to the EOvectorMutex object.
    @param      tout             Timeout for the operation in micro-seconds.
    @return     eores_OK upon success, eores_NOK_nullpointer if vector is NULL,
                eores_NOK_timeout if the mutex was busy within the specified timeout. 
    @warning    After this function call, previously obtained references to any item object 
                will point to zero-ed data
 **/
extern eOresult_t eo_vectorMutex_Clear(EOvectorMutex * vector, eOreltime_t tout);


/** @fn         extern eOresult_t eo_vectorMutex_At(EOvector * vector, eOsizecntnr_t pos, void **ppitem, eOreltime_t tout)
    @brief      Retrieves a reference to the item object in position pos 
    @param      vector          Pointer to the EOvector object. 
    @param      pos             Position of the desired item object.
    @param      ppitem          Address in which the function will copy the retrieved object.
                                In output it cab value NULL, if pos points to an empty position or 
                                beyond reserved space 
    @param      tout            Timeout for the operation in micro-seconds.
    @return     eores_OK upon success (retrieval of valid data), eores_NOK_nullpointer if vector is NULL, 
                eores_NOK_timeout if the mutex was busy within the specified timeout. 
    @warning    Before use, the returned pointer in ppitem needs to be casted to the desidered object type.
                If the item object is a uint32_t, use: 
                uint32_t *p = (uint32_t*) eo_vector_At(vector, pos); 
 **/
extern eOresult_t eo_vectorMutex_At(EOvectorMutex * vector, eOsizecntnr_t pos, void **ppitem, eOreltime_t tout);


/** @fn         extern eOresult_t eo_vectorMutex_Assign(EOvector * vector, void *p, eOsizecntnr_t pos, eOreltime_t tout) 
    @brief      Assigns the item pointed by @e p to the position @e pos. If pos > size, then the
                function shall resize the EOvector. A call with pos > capacity shall do nothing.
    @param      vector          Pointer to the EOvector object.
    @par        p               Item to be assigned 
    @par        pos             Position where to assign.
    @param      tout            Timeout for the operation in micro-seconds.
    @return     eores_OK upon success (retrieval of valid data), eores_NOK_nullpointer if vector is NULL, 
                eores_NOK_timeout if the mutex was busy within the specified timeout.
    @warning    If called with @e pos higher than the capacity of the EOvector, the function shall do nothing

 **/
extern eOresult_t eo_vectorMutex_Assign(EOvectorMutex * vector, void *p, eOsizecntnr_t pos, eOreltime_t tout) ;


/** @fn         extern eOresult_t eo_vectorMutex_Resize(EOvector * vector, eOsizecntnr_t size, eOreltime_t tout) 
    @brief      Resize the vector and calls proper constructor or destructors for teh item. 
                A call with pos > capacity shall do nothing. 
    @param      vector          Pointer to the EOvector object.
    @par        size            The new size 
    @param      tout            Timeout for the operation in micro-seconds.
    @return     eores_OK upon success, eores_NOK_nullpointer if vector is NULL, 
                eores_NOK_timeout if the mutex was busy within the specified timeout.
 **/
extern eOresult_t eo_vectorMutex_Resize(EOvectorMutex * vector, eOsizecntnr_t size, eOreltime_t tout) ;



/** @fn         extern eObool_t eo_vectorMutex_Full(EOvectorMutex * vector, eObool_t *isFull, eOreltime_t tout) 
    @brief      says if the vector is full
    @param      vector          Pointer to the EOvector object.
    @par        isFull          in output sys if the vactor is full if result is positive.
    @param      tout            Timeout for the operation in micro-seconds.
    @return     eores_OK upon success, eores_NOK_nullpointer if vector is NULL, 
                eores_NOK_timeout if the mutex was busy within the specified timeout.
 **/
extern eOresult_t eo_vectorMutex_Full(EOvectorMutex * vector, eObool_t *isFull, eOreltime_t tout);

/** @}            
    end of group eo_vector  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


