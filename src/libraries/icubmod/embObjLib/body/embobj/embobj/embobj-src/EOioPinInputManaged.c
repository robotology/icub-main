
/* @file       EOioPinInputManaged.c
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

#include "EOioPinInputManaged.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOioPinInputManaged_hid.h" 


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

static const char s_eobj_ownname[] = "EOioPinInputManaged";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern EOioPinInputManaged * eo_iopininpman_GetHandle(eOid08_t id) 
{
    EOioPinInputManaged *ret = NULL;
    EOtheGPIO *gpio = NULL;
    
    gpio = eo_gpio_GetHandle();
    
//    if(NULL == gpio) 
//    {
//        return(NULL);    
//    }

    eo_errman_Assert(eo_errman_GetHandle(), (0 != gpio), s_eobj_ownname, "cannot get the EOtheGPIO singleton");
    
    // we are sure that the valid IDs are number ranging from 0 to nmnginp-1 
    // because the gpio tested correctned tof the cfg
    if(id < gpio->cfg->nmnginp) 
    {
        ret = &gpio->mnginp[id];    
    }
    else
    {
        eo_errman_Assert(eo_errman_GetHandle(), 0, s_eobj_ownname, "used an incorrect id");
    }
    
    return(ret);
}

extern eOiopinVal_t eo_iopininpman_GetVal(EOioPinInputManaged *const p)
{
    // must check p vs NULL if we use p-> 
    if(NULL == p) 
    {
        return(eo_iopinvalNONE);
    }
    
    return(eo_iopin_GetVal(p->iopin));    
}


extern eOiopinStatus_t eo_iopininpman_GetStatus(EOioPinInputManaged *const p)
{
    // must check p vs NULL if we use p-> 
    if(NULL == p) 
    {
        return(eo_iopinStatUNDEF);
    }
    
    return(eo_iopin_GetStatus(p->iopin));    
}



extern eOresult_t eo_iopininpman_ActionOn_Register(EOioPinInputManaged *mi, EOaction *act, eOiopinTrigger_t trigger, eOreltime_t after)
{
    eOresult_t res = eores_NOK_generic;
    EOtheGPIOManager *gm = eo_gpioman_GetHandle();
    eOresult_t alreadymanaged = eores_NOK_generic;
    uint8_t index = 0;
    eOreltime_t timerise = 0;
    eOreltime_t timefall = 0;

    
    if((NULL == gm) || (NULL == mi)) 
    {
        return(eores_NOK_nullpointer);    
    }

    if(eores_NOK_timeout == eo_gpioman_hid_Take(gm, eok_reltimeINFINITE)) 
    {
         return(eores_NOK_busy);
    }

    // verify if we can add a pin to the manager: the pin must not have already been given to the manager.
    
    // allow registration also if this pin is already in the active list. that because there 
    // are up to 4 triggers available for each one.
    
    alreadymanaged = eo_gpioman_hid_IsPinManaged(gm, mi, eo_iopindirINP);
    
    if((eores_OK == alreadymanaged) || (eores_OK == eo_gpioman_hid_CanManagePin(gm, eo_iopindirINP))) 
    {
        // assign the index
        switch(trigger) 
        {
            case eo_iopinTrig_OnRiseHit:
            {
                mi->trigonrise_hit = 1; 
                index = 0; 
            } break; 
            case eo_iopinTrig_OnFallHit:        
            {
                mi->trigonfall_hit = 1; 
                index = 1; 
            } break; 
            case eo_iopinTrig_OnRiseStay:
            {
                mi->trigonrise_stay = 1; 
                index = 2; 
                timerise = after; 
            } break; 
            case eo_iopinTrig_OnFallStay:    
            {
                mi->trigonfall_stay = 1; 
                index = 3; 
                timefall = after; 
            } break;
//            default:                    
//                return(res);
        } 
        
        
        // prepare the inp pin
        mi->iopin->defval   = eo_iopin_GetVal(mi->iopin); // inside here we store the memory of the prev input val 
        mi->iopin->status   = eo_iopinStatDRIVEN;
        mi->counter         = 0;
        mi->afterrise       = (0 != timerise) ? (timerise) : (mi->afterrise); 
        mi->afterfall       = (0 != timefall) ? (timefall) : (mi->afterfall);        
        mi->counting        = 0;
        
        memcpy(&mi->actionon[index], act, sizeof(EOaction));
        
        // add the inp pin to the manager only if it was not already in the list.
        // the adding function returns error if the manager is full and cannot accept pins any more. 
        // however, (a) we asked before to the manager if it was not
        // full, (b) we locked the manager, hence: we will surely have a success in here.
        
        if(eores_OK != alreadymanaged) 
        {
            eo_gpioman_hid_ManagePin(gm, mi, eo_iopindirINP);
        }
        
        res = eores_OK;
    }
        

    // unlock the gpio manager
    eo_gpioman_hid_Release(gm);
 
    return(res); 
}
 

 
extern eOresult_t eo_iopininpman_ActionOn_Unregister(EOioPinInputManaged *mi, eOiopinTrigger_t trigger)
{
    EOtheGPIOManager *gm = eo_gpioman_GetHandle();
    
    if((NULL == gm) || (NULL == mi)) 
    {
        return(eores_NOK_nullpointer);    
    }

    if(eores_NOK_timeout == eo_gpioman_hid_Take(gm, eok_reltimeINFINITE)) 
    {
        return(eores_NOK_busy);
    }

    // verify if the pin is driven and also if the pin is in care of the manager 
    if((eo_iopinStatDRIVEN == eo_iopin_GetStatus(mi->iopin)) && (eores_OK == eo_gpioman_hid_IsPinManaged(gm, mi, eo_iopindirINP))) 
    {
        // remove the gpio from the manager. 
        eo_gpioman_hid_UnManagePin(gm, mi, eo_iopindirINP, trigger);
    }
    
    // reset the inp pin
    eo_iopininpman_hid_Reset(mi);


    // unlock the gpio manager
    eo_gpioman_hid_Release(gm);

    // in any case the input is not driven anymore.
    return(eores_OK);
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

extern EOioPinInputManaged * eo_iopininpman_hid_NewArray(uint8_t n)
{
    EOioPinInputManaged *retptr = NULL;    
    uint8_t i=0;

    // i get the memory for the object. remember.... mempool never returns NULL.
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOioPinInputManaged), n);
    
    // now the obj has valid memory. i need to initialise it with user-defined data
    for(i=0; i<n; i++) 
    {
        retptr[i].iopin = eo_iopin_New();
        eo_iopininpman_hid_Reset(&retptr[i]);
    }

    return(retptr);      
}


extern void eo_iopininpman_hid_Reset(EOioPinInputManaged *const p)
{
    // reset the managed out pin
    p->iopin->defval    = eo_iopinvalNONE;
    p->iopin->status    = eo_iopinStatIDLE;
    p->actionon[0].actiontype = p->actionon[1].actiontype = 
    p->actionon[2].actiontype = p->actionon[3].actiontype = eo_actypeNONE; 
    p->counter          = 0;
    p->afterrise        = 0;
    p->afterfall        = 0;
    p->counting         = 0;
    p->trigonrise_hit   = 0;
    p->trigonfall_hit   = 0;
    p->trigonrise_stay  = 0;
    p->trigonfall_stay  = 0;
    p->maxactions       = NACT;
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------





