
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOLIST_H_
#define _EOLIST_H_


/** @file       EOlist.h
	@brief      This header file implements public interface to a list object.
	@author     marco.accame@iit.it
	@date       08/03/2011
**/

/** @defgroup eo_list Object EOlist
    The EOlist allows to manipulate double ended lists of any item object, implementing a simplified version 
    of the list<type> template of the standard C++ library.
    At creation, the EOlist receives the dimension of the item object which will contain, their maximum number
    and optional constructor and destructor for the item objects.
    The EOlist receives a pointer to an item object and copies the pointed item inside its internal memory,
    calling the optional constructor on it. When the EOlist is asked for an item object, it returns a pointer 
    to an internal item object. If the EOlist is requested to remove the item object, the optional
    destructor is called and then the memory is set to zero.
    The EOlist is a base object and can be used to derive new objects to manipulate specific
    items. Its main use is to manipulate items whcih can be inserted or removed in any position of the list.
    
    @{		
 **/
 
// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types -------------------------------------------------------------------------

/**	@typedef    typedef struct EoListIter_hid EOlistIter
    @brief      EOlistIter is an opaque struct. It is used to implement data abstraction for the list iterator 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions.
 **/ 
typedef struct EOlistIter_hid EOlistIter;


/** @typedef    typedef struct EoList_hid EOlist
    @brief      EOlist is an opaque struct. It is used to implement data abstraction for the list 
                object so that the user cannot see its private fields so that he/she is forced to manipulate the
                object only with the proper public functions. 
 **/ 
typedef struct EOlist_hid EOlist;



// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

 
/** @fn         extern EOlist* eo_list_New(eOsizeitem_t item_size, eOsizecntnr_t capacity, 
                                           eOres_fp_voidp_uint32_t item_init, uint32_t init_par,
                                           eOvoid_fp_voidp_t item_copy, eOvoid_fp_voidp_t item_clear)
    @brief      Creates a new list object able to contain at most capacity items of size item_size bytes.   
    @param      item_size       The size in bytes of the item object managed by the EOlist.
    @param      capacity        The max number of item objects stored by the EOlist.
    @param      item_init       Pointer to a specialised init function for the item object to be called at
                                creation of the object for each contained item with arguments item_init(item, item_par). 
                                If NULL, memory is just set to zero.
    @param      item_par        Argument used for @e item_init(item, item_par).                                
    @param      item_copy       Pointer to a copy constructor for the item object to be called at each copy of an item 
                                object inside the EOlist (NULL if the item object does not require initialisation).
    @param      item_clear      Pointer to a destructor for the item object to be called at each removal of an item 
                                object from the EOlist (NULL if the object does not require destruction).
    @return     Pointer to the required EOlist object. The pointer is guaranteed to be always valid and will 
                never be NULL, because failure is managed by the memory pool.
 **/ 
extern EOlist* eo_list_New(eOsizeitem_t item_size, eOsizecntnr_t capacity, 
                           eOres_fp_voidp_uint32_t item_init, uint32_t init_par,
                           eOres_fp_voidp_voidp_t item_copy, eOres_fp_voidp_t item_clear);

/** @fn         extern eOsizecntnr_t eo_list_Capacity(EOlist *list)
    @brief      Returns the maximum number of item objects that the EOlist is able to contain.
    @param      list           Pointer to the EOlist object.
    @return     Max number of storable item objects.
 **/
extern eOsizecntnr_t eo_list_Capacity(EOlist *list);


/** @fn         extern eOsizecntnr_t eo_list_Size(EOlist *list)
    @brief      Returns the number of item objects that are currently stored in the EOlist.
    @param      list            Pointer to the EOlist object. 
    @return     Number of item objects.
 **/
extern eOsizecntnr_t eo_list_Size(EOlist *list);


/** @fn         extern eObool_t eo_list_Empty(EOlist *list)
    @brief      Tells if the list is empty (size is zero)
    @param      list            Pointer to the EOlist object. 
    @return     eobool_true or eobool_false.
 **/
extern eObool_t eo_list_Empty(EOlist *list);


/** @fn         extern eObool_t eo_list_Full(EOlist *list)
    @brief      Tells if the list is full (size is equal to capacity).
    @param      list            Pointer to the EOlist object. 
    @return     eobool_true or eobool_false.
 **/
extern eObool_t eo_list_Full(EOlist *list);

/** @fn         extern void eo_list_PushFront(EOlist *list, void *p)
    @brief      Copies item object pointed by @e p at the front of the EOlist and calls its copy constructor
                @e item_copy(item, p) if passed not NULL in eo_list_New().
    @param      list            pointer to the EOlist object.
    @param      p               Pointer to the item object to be copied into the EOlist. 
 **/
extern void eo_list_PushFront(EOlist *list, void *p);


/** @fn         extern void * eo_list_Front(EOlist *list)
    @brief      Retrieves a reference to the item object in the front of the EOlist without removing it.
    @param      list           Pointer to the EOlist object.
    @return     Pointer to the item object (or NULL if the EoDeque is empty). 
    @warning    Before use, the returned pointer needs to be casted to the desidered object type.

 **/
extern void * eo_list_Front(EOlist *list);


/** @fn         extern void eo_list_PushBack(EOlist *list, void *p)
    @brief      Copies item object pointed by @e p at the end of the EOlist and calls its copy constructor
                @e item_copy(item, p) if passed not NULL in eo_list_New().
    @param      list            pointer to the EOlist object.
    @param      p               Pointer to the item object to be copied into the EOlist. 
 **/
extern void eo_list_PushBack(EOlist *list, void *p);


/** @fn         extern void * eo_list_Back(EOlist *list)
    @brief      Retrieves a reference to the item object in the tail of the EOlist without removing it.
    @param      list           Pointer to the EOlist object.
    @return     Pointer to the item object (or NULL if the EoDeque is empty). 
    @warning    Before use, the returned pointer needs to be casted to the desidered object type.

 **/
extern void * eo_list_Back(EOlist *list);


/** @fn         extern void eo_list_Insert(EOlist *list, EOlistIter *li, void *p)
    @brief      Insert item object pointed by @e p BEFORE the iterator @e li and calls its copy constructor
                @e item_copy(item, p) if passed not NULL in eo_list_New().
    @param      list            pointer to the EOlist object.
    @param      li              The list iterator.
    @param      p               Pointer to the item object to be copied into the EOlist. 
 **/
extern void eo_list_Insert(EOlist *list, EOlistIter *li, void *p);


/** @fn         extern void * eo_list_At(EOlist *list, EOlistIter *li)
    @brief      Retrieves a reference to the item object in position pos 
    @param      list            Pointer to the EOlist object. 
    @param      pos             Position of the desired item object.
    @return     Pointer to the item object (or NULL if pos points to an empty position or 
                beyond reserved space).
    @warning    Before use, the returned pointer needs to be casted to the desidered object type.
                If the item object is a uint32_t, use: 
                const uint32_t *p = (const *uint32_t) eo_list_At(list, pos); 
 **/
extern void * eo_list_At(EOlist *list, EOlistIter *li);


/** @fn         extern void eo_list_PopFront(EOlist *list)
    @brief      Removes the item object from the front of the EOlist, calls its destructor 
                @e item_clear(p) if passed not NULL in eo_list_New(), and finally sets memory to zero.
    @param      list            Pointer to the EOlist object.
    @warning    After this function call, previously obtained references to the removed item object 
                will point to zero-ed data
 **/
extern void eo_list_PopFront(EOlist *list);


/** @fn         extern void eo_list_Erase(EOlist *list, EOlistIter *li)
    @brief      Remove the item stored in iterator @e li, calls its destructor 
                @e item_clear(p) if passed not NULL in eo_list_New(), and finally sets memory to zero.  
    @param      list            Pointer to the EOlist object.
    @param      li              The list iterator.
 **/
extern void eo_list_Erase(EOlist *list, EOlistIter *li);


/** @fn         extern void eo_list_Clear(EOlist *list)
    @brief      Removes all item objects from the EOlist and for each one calls its destructor 
                @e item_dtor() if passed not NULL in eo_list_new(). Finally sets memory to zero.
    @param      list           Pointer to the EOlist object. 
    @warning    After this function call, previously obtained references to any item object 
                will point to zero-ed data
 **/
extern void eo_list_Clear(EOlist *list);


/** @fn         extern EOlistIter* eo_list_Begin(EOlist *list)
    @brief      Returns a list iterator pointing to the beginning of the list.
    @param      list            Pointer to the EOlist object.
    @return     The list iterator (or NULL if the list is empty).
 **/
extern EOlistIter* eo_list_Begin(EOlist *list);


/** @fn         extern EOlistIter* eo_list_Next(EOlist *list, EOlistIter *li)
    @brief      Increments the iterator to next position in the list. 
    @param      list            Pointer to the EOlist object.
    @param      li              The list iterator.
    @return     The next iterator (or NULL if list is NULL or empty / @e li is the last of the list 
                / @e li does not belong to list / @e li is NULL).
 **/
extern EOlistIter* eo_list_Next(EOlist *list, EOlistIter *li);


/** @fn         extern EOlistIter* eo_list_Prev(EOlist *list, EOlistIter *li)
    @brief      Decrements the iterator to previous position in the list. 
    @param      list            Pointer to the EOlist object.
    @param      li              The list iterator.
    @return     The iterator (or NULL if list is NULL or empty / @e li is already the first in list 
                / @e li does not belong to list / @e li is NULL).
 **/
extern EOlistIter* eo_list_Prev(EOlist *list, EOlistIter *li);


/** @fn         extern eObool_t eo_list_IsIterInside(EOlist *list, EOlistIter *li)
    @brief      Tells if the iterator li is in the list. 
    @param      list            Pointer to the EOlist object.
    @param      li              The list iterator.
    @return     eobool_true if iterator is in list, eobool_false if not / @e li or @e list are NULL.
 **/
extern eObool_t eo_list_IsIterInside(EOlist *list, EOlistIter *li);


/** @fn         extern EOlistIter* eo_list_FindItem(EOlist *list, void *p)
    @brief      Finds the iterator which contains a copy of the object item pointer by p. 
    @param      list            Pointer to the EOlist object.
    @param      p               Pointer to the item object.
    @return     The iterator (or NULL if list is NULL or empty / @e p is not in the list / @e p 
                is NULL).
 **/
extern EOlistIter* eo_list_FindItem(EOlist *list, void *p); 


/** @fn         extern EOlistIter* eo_list_Find(EOlist *list, eOresult_t (matching_rule)(void *item, void *param), void *param)
    @brief      Finds the first iterator of the list which satisfies the function matching_rule() when called with
                the item inside the list and a fixed param. 
    @param      list            Pointer to the EOlist object.
    @param      matching_rule() The function which is internally called until it returns eores_OK.
                                The first argument is a pointer to an item stored inside the list, in the format for example 
                                returned by function @e eo_list_At(), whereas the second argument cointains the pointer to 
                                an object which is to be compared with the item.
                                An example could be the following: in a list which contains a pointer to a struct ST_t
                                with a eOabstime_t value lifetime, if we want to find inside teh list the item with lifetime 
                                just bigger than a value target_lifetime we build a function which when internally called as 
                                matching_rule(item, &target_lifetime) returns eores_OK when (*((ST_t**)item))->lifetime is higher
                                than *((eOabstime_t*)param); 
    @param      param           The second argument of @e matching_rule().
    @return     The iterator (or NULL if list is NULL or empty / the function matching_rule() is NULL or
                is never satisfied
 **/
extern EOlistIter* eo_list_Find(EOlist *list, eOresult_t (matching_rule)(void *item, void *param), void *param);



/** @fn         extern void eo_list_ForEach(EOlist *list, void (execute)(void *item, void *param), void *param)
    @brief      Executes the function execute() on each item inside the list. 
    @param      list            Pointer to the EOlist object.
    @param      execute()       The function which is internally called for each item.
                                The first argument is a pointer to an item stored inside the list, in the format for example 
                                returned by function @e eo_list_At(), whereas the second argument cointains a user-defineable
                                parameter. 
         
    @param      param           The second argument of @e execute().
 **/
extern void eo_list_ForEach(EOlist *list, void (execute)(void *item, void *param), void *param);

extern void eo_list_FromIterForEach(EOlist *list, EOlistIter *li, void (execute)(void *item, void *param), void *param);

/** @}            
    end of group eo_list  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



