
/* @file       EOnv.c
    @brief      This file implements internal implementation of a nv object.
    @author     marco.accame@iit.it
    @date       09/03/2010
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"




// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtreenode.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtreenode_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

//static const char s_eobj_ownname[] = "EOnv";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern uint8_t eo_treenode_GetIndex(EOtreenode *node)
{
    if(NULL == node)
    {
        return(EOK_uint08dummy);
    }
    
    return(node->index);
}

extern void* eo_treenode_GetData(EOtreenode *node)
{
    if(NULL == node)
    {
        return(NULL);
    }
    
    return(node->data);
}

extern uint8_t eo_treenode_GetNumberOfChilden(EOtreenode *node)
{
    if(NULL == node)
    {
        return(0);
    }
    
    return(node->nchildren);
}

extern uint8_t eo_treenode_GetIndexOfChild(EOtreenode *node, uint8_t index)
{
    if((NULL == node) || (index >= node->nchildren))
    {
        return(EOK_uint08dummy);
    }
    
    return(node->ichildren[index]);
}


extern EOtreenode* eo_treenode_GetChild(EOtreenode *node, uint8_t index)
{
    if((NULL == node) || (index >= node->nchildren))
    {
        return(NULL);
    }
    
    return(node->pchildren[index]);

}


extern eObool_t eo_treenode_isLeaf(EOtreenode *node)
{
    if((NULL == node) || (0 != node->nchildren))
    {
        return(eobool_false);
    }
    else
    {
        return(eobool_true);
    }
} 

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

//extern EOtreenode * eo_treenode_hid_New(void)
//{
//    EOtreenode *retptr = NULL;    
//
//    // i get the memory for the object
//    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOtreenode), 1);
//    
//    retptr->nchildren = 0;
//    memset(retptr->children, EOK_uint08dummy, EOTREENODE_NCHILDREN);
//    retptr->nv = NULL;
//    
//    return(retptr);
//}


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------





// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




