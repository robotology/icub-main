
/* @file       EOioPinOutputManaged.c
    @brief      This file implements internal implementation of a managed input pin object.
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
#include "EOiopin_hid.h"
#include "EOtheGPIO_hid.h"
#include "EOtheGPIOManager_hid.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOioPinOutputManaged.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOioPinOutputManaged_hid.h" 


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

static const char s_eobj_ownname[] = "EOioPinOutputManaged";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern EOioPinOutputManaged * eo_iopinoutman_GetHandle(eOid08_t id) 
{
    EOioPinOutputManaged *ret = NULL;
    EOtheGPIO *gpio = NULL;
    
    gpio = eo_gpio_GetHandle();
    
    if(NULL == gpio) 
    {
        return(NULL);    
    }
    
    // we are sure that the valid IDs are number ranging from 0 to nmngout-1 
    // because the gpio tested correctness of the cfg
    if(id < gpio->cfg->nmngout) 
    {
        ret = &gpio->mngout[id];    
    }
    
    return(ret);
}

extern eOiopinVal_t eo_iopinoutman_GetVal(EOioPinOutputManaged *const p)
{
    // must check p vs NULL if we use p-> 
    if(NULL == p) 
    {
        return(eo_iopinvalNONE);
    }
    
    return(eo_iopin_GetVal(p->iopin));    
}


extern eOiopinStatus_t eo_iopinoutman_GetStatus(EOioPinOutputManaged *const p)
{
    // must check p vs NULL if we use p-> 
    if(NULL == p) 
    {
        return(eo_iopinStatUNDEF);
    }
    
    return(eo_iopin_GetStatus(p->iopin));    
}

 
extern eOresult_t eo_iopinoutman_SetVal(EOioPinOutputManaged *const p, eOiopinVal_t val)
{
    // must check p vs NULL if we use p->
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    // see if the pin is running
    if(eo_iopinStatDRIVEN == eo_iopin_GetStatus(p->iopin))
    {
        return(eores_NOK_busy);
    }
    
    return(eo_iopin_SetVal(p->iopin, val));
}






extern eOresult_t eo_iopinoutman_Waveform_Start(EOioPinOutputManaged *const p, eOiopinVal_t val1st, eOreltime_t ustime1st, eOreltime_t ustime2nd, uint32_t numwaves) 
{
    eOresult_t res = eores_NOK_generic;
    EOtheGPIOManager *gm = eo_gpioman_GetHandle();
    
    if((NULL == gm) || (NULL == p)) 
    {
        return(eores_NOK_nullpointer);    
    }

    if(eores_NOK_timeout == eo_gpioman_hid_Take(gm, eok_reltimeINFINITE)) 
    {
         return(eores_NOK_busy);
    }

    // verify if we can give the out pin to the manager: the pin must not have already been given
    // to the manager, and the manager must be able to accept it
    if((eo_iopinStatDRIVEN != eo_iopin_GetStatus(p->iopin)) && (eores_OK == eo_gpioman_hid_CanManagePin(gm, eo_iopindirOUT))) 
    {
        // prepare the out pin
        p->iopin->defval        = val1st;
        p->iopin->status        = eo_iopinStatDRIVEN;
        
        p->counter             = 0;
        p->ushalf01            = ustime1st;
        p->ushalf02            = ustime2nd;
        p->numwaves            = numwaves;
        
        // add the out pin to the manager. the function returns error if the manager is full
        // and cannot accept pins any more. however, (a) we asked before to the manager if it was not
        // full, (b) we locked the manager. hence: we will surely have a success in here.
        res = eo_gpioman_hid_ManagePin(gm, p, eo_iopindirOUT);
        
        // if the assert fails, then .... something is wrong.
        eo_errman_Assert(eo_errman_GetHandle(), (eores_OK == res), s_eobj_ownname, "eo_iopinoutman_Waveform_Start() failed");
    }
    
    // unlock the gpio manager
    eo_gpioman_hid_Release(gm);
 
    return(eores_OK); 
}
 

extern eOresult_t eo_iopinoutman_Waveform_Stop(EOioPinOutputManaged *const p)
{
    EOtheGPIOManager *gm = eo_gpioman_GetHandle();
    
    if((NULL == gm) || (NULL == p)) 
    {
        return(eores_NOK_nullpointer);    
    }

    if(eores_NOK_timeout == eo_gpioman_hid_Take(gm, eok_reltimeINFINITE)) 
    {
         return(eores_NOK_busy);
    }

    // verify if the pin is (still) running and if the pin is in care of the manager 
    if((eo_iopinStatDRIVEN == eo_iopin_GetStatus(p->iopin)) && (eores_OK == eo_gpioman_hid_IsPinManaged(gm, p, eo_iopindirOUT))) 
    {
        // remove the gpio from the manager. 
        eo_gpioman_hid_UnManagePin(gm, p, eo_iopindirOUT, eo_iopinTrig_NONE);
    }
    
    // reset the out pin
    eo_iopinoutman_hid_Reset(p);


    // unlock the gpio manager
    eo_gpioman_hid_Release(gm);


    // in any case the outpin is not running anymore.
    return(eores_OK);

}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


extern EOioPinOutputManaged * eo_iopinoutman_hid_NewArray(uint8_t n)
{
    EOioPinOutputManaged *retptr = NULL;    
    uint8_t i=0;

    // i get the memory for the object. remember.... mempool never returns NULL.
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOioPinOutputManaged), n);
    
    // now the obj has valid memory. i need to initialise it with user-defined data
    for(i=0; i<n; i++) 
    {
        retptr[i].iopin = eo_iopin_New();
        eo_iopinoutman_hid_Reset(&retptr[i]);
    }

    return(retptr);      
}



extern void eo_iopinoutman_hid_Reset(EOioPinOutputManaged *const p)
{
    // reset the out pin
    p->iopin->defval        = eo_iopinvalNONE;
    p->iopin->status        = eo_iopinStatIDLE;
        
    p->counter              = 0;
    p->ushalf01             = 0;
    p->ushalf02             = 0;
    p->numwaves             = 0;
}





// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




