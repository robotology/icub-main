
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EODEQUE_H_
#define _EODEQUE_H_


/** @file       EoDeque.h
	@brief      This header file implements public interface to a deque object.
	@author     marco.accame@iit.it
	@date       08/03/2011
**/

/** @defgroup eo_deque Object EOdeque
    The EOdeque allows to manipulate double ended queues of any item object, implementing the deque<type> template 
    of the standard C++ library.
    At creation, the EOdeque receives the dimension of the item object which will contain, their maximum number
    and optional copy and remove functions for the item objects.
    The EOdeque receives a pointer to an item object and copies the pointed item inside its internal memory,
    calling the optional copy function on it or the default memcpy. When the EOdeque is asked for an item object, 
    it returns a pointer to an internal item object. If the EOdeque is requested to remove the item object, the 
    optional user-defined remove function is called or the default remove which set memory to zero.
    The EOdeque is an object that can be directly used as it is but also to derive a new object to manipulate specific 
    items. For an example of a fifo queue that has been derived from the EOdeque, see EOfifo
    
    @{		
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/**	@typedef    typedef struct EOdeque_hid EOdeque
 	@brief 		EOdeque is an opaque struct. It is used to implement data abstraction for the deque 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOdeque_hid EOdeque;

    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------


 
/** @fn         extern EOdeque * eodeque_New(eOsizeitem_t item_size, eOsizecntnr_t capacity, 
                                             eOres_fp_voidp_uint32_t item_init, uint32_t init_par, 
                                             eOres_fp_voidp_voidp_t item_copy, eOres_fp_voidp_t item_clear);
    @brief      Creates a new EOdeque object and reserves memory for the items that will be stored in its
                inside, taking it from the memory pool.
    @param      item_size       The size in bytes of the item object managed by the EOdeque.
    @param      capacity        The max number of item objects stored by the EOdeque.
    @param      item_init       Pointer to a specialised init function for the item object to be called at
                                creation of the object for each contained item with arguments item_init(item, item_par). 
                                If NULL, memory is just set to zero.
    @param      item_par        Argument used for @e item_init(item, item_par).                                
    @param      item_copy       Pointer to a specialised copy function for the item object to be called 
                                at each copy of an item object inside the EOdeque with arguments item_copy(dest, orig).
                                If NULL it will be executed a simple memcpy of the size of the item object.                                
    @param      item_clear      Pointer to a specialised remove function for the item object to be called at each 
                                removal of an item object from the EOdeque. If NULL the memory inside the container
                                will be simply set to zero.
    @return     Pointer to the required EOdeque object. The pointer is guaranteed to be always valid and will 
                never be NULL, because failure is managed by the memory pool.
 **/
extern EOdeque * eo_deque_New(eOsizeitem_t item_size, eOsizecntnr_t capacity,
                              eOres_fp_voidp_uint32_t item_init, uint32_t init_par, 
                              eOres_fp_voidp_voidp_t item_copy, eOres_fp_voidp_t item_clear);


/** @fn         extern eOsizecntnr_t eo_deque_Capacity(EOdeque * deque)
    @brief      Returns the maximum number of item objects that the EOdeque is able to contain.
    @param      deque           Pointer to the EOdeque object.
    @return     Max number of storable item objects.
 **/
extern eOsizecntnr_t eo_deque_Capacity(EOdeque * deque);


/** @fn         extern eOsizecntnr_t eo_deque_Size(EOdeque * deque)
    @brief      Returns the number of item objects that are currently stored in the EOdeque.
    @param      deque           Pointer to the EOdeque object. 
    @return     Number of item objects.
 **/
extern eOsizecntnr_t eo_deque_Size(EOdeque * deque);


/** @fn         extern eObool_t eo_deque_Empty(EOdeque * deque)
    @brief      Tells if the EOdeque is empty
    @param      deque           Pointer to the EOdeque object. 
    @return     eobool_true or eobool_false.
 **/
extern eObool_t eo_deque_Empty(EOdeque * deque);


/** @fn         extern eObool_t eo_deque_Full(EOdeque * deque)
    @brief      Tells if the EOdeque is full
    @param      deque           Pointer to the EOdeque object. 
    @return     eobool_true or eobool_false.
 **/
extern eObool_t eo_deque_Full(EOdeque * deque); 


/**  @fn        extern void eo_deque_PushBack(EOdeque * deque, void *p)
     @brief     Copies the item object pointed by @e p at the back of the EOdeque and calls its constructor 
                @e item_ctor(p) if passed not NULL in eo_deque_New().
     @param     deque           pointer to the EOdeque object.
     @param     p               pointer to the item object to be copied into the EOdeque. 
 **/
extern void eo_deque_PushBack(EOdeque * deque, void *p);


/** @fn         extern void eo_deque_PushFront(EOdeque *deque, void *p)
    @brief      Copies item object pointed by @e p at the front of the EOdeque and calls its constructor
                @e item_ctor(p) if passed not NULL in eo_deque_New().
    @param      deque           pointer to the EOdeque object.
    @param      p               Pointer to the item object to be copied into the EOdeque. 
 **/
extern void eo_deque_PushFront(EOdeque * deque, void *p);


/** @fn         extern void* eo_deque_Front(EOdeque *deque)
    @brief      Retrieves a reference to the item object in the front of the EOdeque without removing it.
    @param      deque           Pointer to the EOdeque object.
    @return     Pointer to the item object (or NULL if the EOdeque is empty). 
    @warning    Before use, the returned pointer needs to be casted to the desidered object type.
 **/
extern void* eo_deque_Front(EOdeque * deque);


/** @fn         extern void* eo_deque_Back(EOdeque *deque)
    @brief      Retrieves a reference to the item object in the back of the EOdeque without removing it. 
    @param      deque           Pointer to the EOdeque object.
    @return     Pointer to the item object (or NULL if the deque is empty).
    @warning    Before use, the returned pointer needs to be casted to the desidered object type.
 **/
extern void* eo_deque_Back(EOdeque * deque);

/** @fn         extern void eo_deque_PopFront(EOdeque * deque)
    @brief      Removes the item object from the front of the EOdeque, calls its destructor 
                @e item_dtor(p) if passed not NULL in eo_deque_New(), and finally sets memory to zero.
    @param      deque           Pointer to the EOdeque object.
    @warning    After this function call, previously obtained references to the removed item object 
                will point to zero-ed data
 **/
extern void eo_deque_PopFront(EOdeque * deque); 


/** @fn         extern void eo_deque_PopBack(EOdeque * deque)
    @brief      Removes the item object from the back of the EOdeque, calls its destructor
                @e item_dtor(p) if passed not NULL in eo_deque_new(), and finally sets memory to zero.
    @param      deque           Pointer to the EOdeque object. 
    @warning    After this function call, previously obtained references to the removed item object 
                will point to zero-ed data
 **/
extern void eo_deque_PopBack(EOdeque * deque); 


/** @fn         extern void eo_deque_clear(EOdeque * deque)
    @brief      Removes all item objects from the EOdeque and for each one calls its destructor 
                @e item_dtor() if passed not NULL in eo_deque_new(). Finally sets memory to zero.
    @param      deque           Pointer to the EOdeque object. 
    @warning    After this function call, previously obtained references to any item object 
                will point to zero-ed data
 **/
extern void eo_deque_Clear(EOdeque * deque);


/** @fn         extern void* eo_deque_At(EOdeque * deque, eOsizecntnr_t pos)
    @brief      Retrieves a reference to the item object in position pos 
    @param      deque           Pointer to the EOdeque object. 
    @param      pos             Position of the desired item object.
    @return     Pointer to the item object (or NULL if pos points to an empty position or 
                beyond reserved space).
    @warning    Before use, the returned pointer needs to be casted to the desidered object type.
                If the item object is a uint32_t, use: 
                uint32_t *p = (*uint32_t) eo_deque_At(deque, pos); 
 **/
extern void* eo_deque_At(EOdeque * deque, eOsizecntnr_t pos);



/** @}            
    end of group eo_deque  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


