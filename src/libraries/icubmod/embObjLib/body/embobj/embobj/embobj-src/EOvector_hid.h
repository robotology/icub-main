
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOVECTOR_HID_H_
#define _EOVECTOR_HID_H_


/* @file       EOvector_hid.h
    @brief      This header file implements hidden interface to a EOvector object.
    @author     marco.accame@iit.it
    @date       08/03/2011
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOvector.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     EOvector_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOvector_hid 
{
    /* @private                 number of items in the vector. used only by the vector. */                
    eOsizecntnr_t               size;                                                            
    /* @private                 size in bytes of the item object. */
    eOsizeitem_t                item_size;                
    /* @private                 array of item object. */
    void                        *item_array_data; 
    /* @private                 max number of item objects in the array. */
    eOsizecntnr_t               capacity;         
    /* @private                 specialised copy for a single item object. It is called upon copy into the vector*/
    eOres_fp_voidp_voidp_t      item_copy_fn;
    /* @private                 specialised remove for a single item object. It is called upon removal from the vector*/
    eOres_fp_voidp_t            item_clear_fn;           
};


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



