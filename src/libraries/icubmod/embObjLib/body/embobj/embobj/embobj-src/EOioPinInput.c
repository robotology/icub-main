
/* @file       EOioPinInput.c
    @brief      This file implements internal implementation of a input pin object.
    @author     marco.accame@iit.it
    @date       10/16/2009
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"

#include "EOioPin_hid.h"
#include "EOtheGPIO_hid.h"




// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOioPinInput.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOioPinInput_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



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

static const char s_eobj_ownname[] = "EOioPinInput";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern EOioPinInput * eo_iopininp_GetHandle(eOid08_t id) 
{
    EOioPinInput *ret = NULL;
    EOtheGPIO *gpio = NULL;
    
    gpio = eo_gpio_GetHandle();
    
//    if(NULL == gpio) 
//    {
//        return(NULL);    
//    }

    eo_errman_Assert(eo_errman_GetHandle(), (0 != gpio), s_eobj_ownname, "cannot get the EOtheGPIO singleton");
    
    // we are sure that the valid IDs are number ranging from 0 to ninp-1 
    // because the gpio tested correctned tof the cfg
    if(id < gpio->cfg->ninp) 
    {
        ret = &gpio->inp[id];    
    }
    else
    {
        eo_errman_Assert(eo_errman_GetHandle(), 0, s_eobj_ownname, "used an incorrect id");
    }
    
    return(ret);
}

extern eOiopinVal_t eo_iopininp_GetVal(EOioPinInput *const p)
{
    // must check p vs NULL if we use p-> 
    if(NULL == p) 
    {
        return(eo_iopinvalNONE);
    }
    
    return(eo_iopin_GetVal(p->iopin));    
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

extern EOioPinInput * eo_iopininp_hid_NewArray(uint8_t n)
{
    EOioPinInput *retptr = NULL;    
    uint8_t i=0;

   // i get the memory for the object. remember.... mempool never returns NULL.
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOioPinInput), n);
    
    // now the obj has valid memory. i need to initialise it with user-defined data
    for(i=0; i<n; i++) 
    {
        retptr[i].iopin = eo_iopin_New();
    }

    return(retptr);      
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



