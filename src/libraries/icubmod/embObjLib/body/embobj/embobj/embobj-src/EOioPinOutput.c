
/* @file       EOioPinOutput.c
    @brief      This file implements internal implementation of a output pin object.
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

#include "EOioPinOutput.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOioPinOutput_hid.h" 


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

static const char s_eobj_ownname[] = "EOioPinOutput";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern EOioPinOutput * eo_iopinout_GetHandle(eOid08_t id) 
{
    EOioPinOutput *ret = NULL;
    EOtheGPIO *gpio = NULL;
    
    gpio = eo_gpio_GetHandle();
    
//    if(NULL == gpio) 
//    {
//        return(NULL);    
//    }
    
    eo_errman_Assert(eo_errman_GetHandle(), (0 != gpio), s_eobj_ownname, "cannot get the EOtheGPIO singleton");
    
    // we are sure that the valid IDs are number ranging from 0 to nout-1 
    // because the gpio tested correctness of the cfg
    if(id < gpio->cfg->nout) 
    {
        ret = &gpio->out[id];    
    }
    else
    {
        eo_errman_Assert(eo_errman_GetHandle(), 0, s_eobj_ownname, "used an incorrect id");
    }
    
    return(ret);
}

extern eOiopinVal_t eo_iopinout_GetVal(EOioPinOutput *const p)
{
    // must check p vs NULL if we use p-> 
    if(NULL == p) 
    {
        return(eo_iopinvalNONE);
    }
    
    return(eo_iopin_GetVal(p->iopin));    
}

extern eOresult_t eo_iopinout_SetVal(EOioPinOutput *const p, eOiopinVal_t val) 
{
    // must check p vs NULL if we use p-> 
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);	
    }

    return(eo_iopin_SetVal(p->iopin, val));
}
 
extern eOresult_t eo_iopinout_ToggleVal(EOioPinOutput *const p) 
{
    // must check p vs NULL if we use p-> 
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);	
    }

    return(eo_iopin_ToggleVal(p->iopin));
}

 
// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

extern EOioPinOutput * eo_iopinout_hid_NewArray(uint8_t n)
{
    EOioPinOutput *retptr = NULL;    
    uint8_t i=0;

    // i get the memory for the object. remember.... mempool never returns NULL.
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOioPinOutput), n);
    
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



