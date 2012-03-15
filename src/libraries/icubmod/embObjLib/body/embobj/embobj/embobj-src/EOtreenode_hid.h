
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTREENODE_HID_H_
#define _EOTREENODE_HID_H_


/* @file       EOtreenode_hid.h
    @brief      This header file implements hidden interface to a rop object.
    @author     marco.accame@iit.it
    @date       09/06/2011
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOtreenode.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------

#define EOTREENODE_NCHILDREN      14


// - definition of the hidden struct implementing the object ----------------------------------------------------------


/** @struct     EOtreenode_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/
struct EOtreenode_hid 
{
    void*                   data;                               // pointer to the data 
    uint8_t                 index;                              // index in the array
    uint8_t                 nchildren;                          // number of dependant netvars
    uint8_t                 ichildren[EOTREENODE_NCHILDREN];     // using indices in a common array rather than pointers which would require more space  
    EOtreenode*             pchildren[EOTREENODE_NCHILDREN];     // using indices in a common array rather than pointers which would require more space  
};   
 



// - declaration of extern hidden functions ---------------------------------------------------------------------------

///** @fn         extern EOtreenode * eo_treenode_hid_New(void)
//    @brief      Creates a new nv object. 
//    @return     The pointer to the required object.
// **/
//extern EOtreenode * eo_treenode_hid_New(void);

 

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




