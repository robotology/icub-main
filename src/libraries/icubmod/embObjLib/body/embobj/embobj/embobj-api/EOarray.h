
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOARRAY_H_
#define _EOARRAY_H_


/** @file       EOarray.h
    @brief      This header file implements public interface to a array object.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/

/** @defgroup eo_array Object EOarray
    The EOarray object is a container which is work-in-progress. It could be used as a slim vector
    in which the content of memory is exposed outsize via a number of data strcuctures such as eOarrayofbytes_t
    or eOarrayofhalves_t or eOarrayofwords_t
     
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 



typedef struct
{
    uint16_t        capacity;
    uint8_t         itemsize;
    uint8_t         size;
} eOarray_head_t;   EO_VERIFYsizeof(eOarray_head_t, 4);

typedef struct
{
    eOarray_head_t          head;
    uint8_t                 data[4];
} EOarray_of;


/** @typedef    typedef struct EOarray_hid EOarray
    @brief      EOarray is an opaque struct. It is used to implement data abstraction for the  
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
//typedef struct EOarray_hid EOarray;
typedef EOarray_of EOarray;
    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
/** @fn         extern EOarray* eo_array_New(uint16_t capacity, uint8_t sizeofitem, void *memory)
    @brief      Creates a new EOarray object. If the argument @e memory is not NULL, then it is used for storage
                inside the object. The function resets
                the data. If we pass a non NULL @e memory then we consume @a capacity bytes + 4 bytes of memory.
    @param      capacity        The capacity of the array.
    @param      itemsize        If not zero, then the array contains fixed-sized items, otherwise just bytes
    @param      memory          If not NULL the memory to use for the object. It is required a size equal to:
                                4 + @e capacity * @e itemsize.
    @return     The pointer to the required object.
    @warning    The function resets the memory pointed by @e memory for the size it requires which is:
                4+ @e capacity * @e itemsize and puts  @e capacity and @e itemsize at its beginning
 **/
extern EOarray* eo_array_New(uint16_t capacity, uint8_t itemsize, void *memory);



/** @fn         extern eOresult_t eo_array_Reset(EOarray *t)
    @brief      Resets the array
    @param      p               The pointer to the array object.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL
 **/
extern eOresult_t eo_array_Reset(EOarray *p);


extern uint16_t eo_array_Capacity(EOarray *p);

extern uint8_t eo_array_ItemSize(EOarray *p);

extern uint8_t eo_array_Size(EOarray *p);

extern uint16_t eo_array_UsedBytes(EOarray *p);

/** @fn         extern eOresult_t eo_array_PushBackItem(EOarray *p, const void *item)
    @brief      Adds an item at the back of the array. It does that only if the EOarray was created with non-zero itemsize
    @param      p               The pointer to the array object.
    @param      item            The item to be pushed back
    @return     eores_OK upon success, eores_NOK_nullpointer if @e p or @e data are NULL, eores_NOK_generic if @e data
                cannot be pushed inside.
 **/
extern eOresult_t eo_array_PushBack(EOarray *p, const void *item);


/** @fn         extern void * eo_array_GetItem(EOarray *p, uint16_t pos, uint16_t *size)
    @brief      Gets a pointer to the item in position @e pos inside the object 
    @param      p               The pointer to the object.
    @param      pos             The position of the iterm
    @param      size            pointer to the size in bytes occupied by the returned pointer.
    @return     The pointer, or NULL upon failure.
 **/
extern void * eo_array_At(EOarray *p, uint8_t pos);


/** @fn         extern eOresult_t eo_array_PushBackItem(EOarray *p, const void *item)
    @brief      Adds an item at the back of the array. It does that only if the EOarray was created with non-zero itemsize
    @param      p               The pointer to the array object.
    @param      item            The item to be pushed back
    @return     eores_OK upon success, eores_NOK_nullpointer if @e p or @e data are NULL, eores_NOK_generic if @e data
                cannot be pushed inside.
 **/
extern eOresult_t eo_array_PopBack(EOarray *p);

 


/** @}            
    end of group eo_array  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

