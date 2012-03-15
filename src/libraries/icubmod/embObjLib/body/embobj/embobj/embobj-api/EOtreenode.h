
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTREENODE_H_
#define _EOTREENODE_H_


/** @file       EOtreenode.h
    @brief      This header file implements public interface to a node which contains a nv
    @author     marco.accame@iit.it
    @date       04/20/2011
**/

/** @defgroup eo_nvnode Object EOtreenode
    The EOtreenode object contains a EOnv plus its relation with other netvars inside the tree of
    the network variables. It is not created but loaded as a constant object from a configuration mapped in ROM.
     
    @{        
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
//#include "EOnv.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
 

// - declaration of public user-defined types -------------------------------------------------------------------------    



/** @typedef    typedef const struct EOtreenode_hid EOtreenode
    @brief      EOtreenode is an opaque struct. It is used to implement data abstraction for the  
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef const struct EOtreenode_hid EOtreenode;



    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------


// - declaration of extern public functions ---------------------------------------------------------------------------
 


///** @fn         extern EOnv * eo_treemode_hid_New(uint8_t fun, uint8_t typ, uint32_t otherthingsmaybe)
//    @brief      Creates a new nv object. 
//    @return     The pointer to the required object.
// **/

extern uint8_t eo_treenode_GetIndex(EOtreenode *node);

extern void* eo_treenode_GetData(EOtreenode *node);

extern uint8_t eo_treenode_GetNumberOfChilden(EOtreenode *node);

extern uint8_t eo_treenode_GetIndexOfChild(EOtreenode *node, uint8_t index);

extern EOtreenode* eo_treenode_GetChild(EOtreenode *node, uint8_t index);

extern eObool_t eo_treenode_isLeaf(EOtreenode *node);


/** @}            
    end of group eo_nvnode 
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

