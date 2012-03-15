
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOCONSTARRAY_H_
#define _EOCONSTARRAY_H_


/** @file       EOconstarray.h
    @brief      This header file implements public interface to a array object.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/

/** @defgroup eo_constarray Object EOconstarray
    The EOconstarray object is a container which is work-in-progress. It could be used as a slim vector
    in which the content of memory is exposed outsize via a number of data strcuctures such as eOarrayofbytes_t
    or eOarrayofhalves_t or eOarrayofwords_t
     
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/** @typedef    typedef struct EOconstarray_hid EOconstarray
    @brief      EOconstarray is an opaque struct. It is used to implement data abstraction for the  
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOconstarray_hid EOconstarray;




    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
/** @fn         extern EOconstarray* eo_constarray_New(eOsizecntnr_t capacity, eOsizeitem_t itemsize, void *memory)
    @brief      Creates a new EOconstarray object. If the argument @e memory is not NULL, then it is used for storage
                inside the object. The function resets
                the data. If we pass a non NULL @e memory then we consume @a capacity bytes + 4 bytes of memory.
    @param      capacity        The capacity of the array.
    @param      itemsize        If not zero, then the array contains fixed-sized items, otherwise just bytes
    @return     The pointer to the required object.
 **/
extern EOconstarray* eo_constarray_New(eOsizecntnr_t size, eOsizeitem_t itemsize);


/** @fn         extern eOsizecntnr_t eo_constarray_Size(EOconstarray *p)
    @brief      Gets the number of items inside the object 
    @param      p               The pointer to the object.
    @return     The size.
 **/
extern eOsizecntnr_t eo_constarray_Size(EOconstarray *p);


/** @fn         extern eOsizecntnr_t eo_constarray_SizeOfItem(EOconstarray *p)
    @brief      Gets the size of items inside the object 
    @param      p               The pointer to the object.
    @return     The size.
 **/
extern eOsizecntnr_t eo_constarray_SizeOfItem(EOconstarray *p);


/** @fn         extern const void * eo_constarray_At(EOconstarray *p, eOsizecntnr_t pos)
    @brief      Gets a pointer to the item in position @e pos inside the object 
    @param      p               The pointer to the object.
    @param      pos             The position of teh iterm
    @param      size            pointer to the size in bytes occupied by the returned pointer.
    @return     The pointer, or NULL upon failure.
 **/
extern const void * eo_constarray_At(EOconstarray *p, eOsizecntnr_t pos);




/** @}            
    end of group eo_constarray  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

