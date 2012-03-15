
/* @file       EOioPin.c
    @brief      This file implements internal implementation of a pin object.
    @author     marco.accame@iit.it
    @date       10/06/2009
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"

#include "hal.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOioPin.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOioPin_hid.h" 


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

static const char s_eobj_ownname[] = "EOioPin";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern EOioPin * eo_iopin_New(void)
{
    EOioPin *retptr = NULL;    
    // i get the memory for the object. remember.... mempool never returns NULL.
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOioPin), 1);
     
    // now the obj has valid memory. i need to initialise it with user-defined data
    
    // reset to default valued
    eo_iopin_Reset(retptr);
    
    return(retptr);        
}


extern eOresult_t eo_iopin_SetVal(EOioPin *p, eOiopinVal_t val) 
{
    eOresult_t res = eores_NOK_generic;
    
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    if(eo_iopindirOUT == p->dir) 
    {
        p->val = val;    
        // hardware dependency is encapulated in hal
        hal_gpio_setval((hal_gpio_port_t)p->halport, (hal_gpio_pin_t)p->halpin, (hal_gpio_val_t)p->val);    
        res = eores_OK;    
    }
    
    return(res);
}



extern eOresult_t eo_iopin_ToggleVal(EOioPin *p)
{
    eOresult_t res = eores_NOK_generic;
    
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    if((eo_iopindirOUT == p->dir) && (eo_iopinvalNONE != p->val)) 
    {
        p->val = (eo_iopinvalLOW == p->val) ? (eo_iopinvalHIGH) : (eo_iopinvalLOW);    
        // hardware dependency are encapulated in hal
        hal_gpio_setval((hal_gpio_port_t)p->halport, (hal_gpio_pin_t)p->halpin, (hal_gpio_val_t)p->val);    
        res = eores_OK;    
    }
    
    return(res);
}


extern eOiopinVal_t eo_iopin_GetVal(EOioPin *p)
{
    eOiopinVal_t ret = eo_iopinvalNONE;
    
    if(NULL == p) 
    {
        return(ret);
    }    
    
    if(eo_iopindirNONE != p->dir) 
    {
        // hardware dependency are encapulated in hal
        p->val = hal_gpio_getval((hal_gpio_port_t)p->halport, (hal_gpio_pin_t)p->halpin);
        ret = (eOiopinVal_t)p->val;        
    }
    
    return(ret);    
}


extern eOresult_t eo_iopin_derived_SetVal(EOioPinDerived *p, eOiopinVal_t val)
{
    return(eo_iopin_SetVal(eo_common_getbaseobject(p), val));
}


extern eOresult_t eo_iopin_derived_ToggleVal(EOioPinDerived *p)
{
     return(eo_iopin_ToggleVal(eo_common_getbaseobject(p)));
}


extern eOiopinVal_t eo_iopin_derived_GetVal(EOioPinDerived *p)
{
     return(eo_iopin_GetVal(eo_common_getbaseobject(p)));
}


extern eOresult_t eo_iopin_SetCfg(EOioPin *p, eOiopinPort_t port, eOiopinPos_t pos, eOiopinDir_t dir, eOiopinVal_t val)
{
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);    
    }    
    
    p->dir          = dir;        
    p->halpin       = pos;
    p->halport      = port;
    
    hal_gpio_init((hal_gpio_port_t)port, (hal_gpio_pin_t)pos, (hal_gpio_dir_t)dir, hal_gpio_speed_low);

    eo_iopin_SetVal(p, val);
        
    return(eores_OK);        
}


extern eOresult_t eo_iopin_GetCfg(EOioPin *p, eOiopinPort_t *pport, eOiopinPos_t *ppos, eOiopinDir_t *pdir) 
{
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);    
    }    
    
    *pdir       = (eOiopinDir_t)p->dir;
    *ppos       = (eOiopinPos_t)p->halpin;
    *pport      = (eOiopinPort_t)p->halport;
    
    return(eores_OK);        
}



extern eOiopinStatus_t eo_iopin_GetStatus(EOioPin *p) 
{
    eOiopinStatus_t ret = eo_iopinStatUNDEF;
    
    if(NULL != p) 
    {
        ret = (eOiopinStatus_t) p->status;
#if 0        
        switch(p->status) 
        {
            case 0:        ret = ioStatNOTMNG;
            break;
            case 1:        ret = ioStatIDLE;
            break;
            case 2:        ret = ioStatDRIVEN;
            break;
            case 3:        ret = eo_iopinStatUNDEF;
            break;
        }
#endif        
        
    }
    
    return(ret);

}


extern eOresult_t eo_iopin_SetStatus(EOioPin *p, eOiopinStatus_t st) 
{
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);    
    }

    p->status = st; // we could do the masking, but i assume the compiler already does it: ((uint8_t)st) & 0x03;

    return(eores_OK);
}


extern eOresult_t eo_iopin_Reset(EOioPin *p) 
{
//    if(NULL == p) 
//    {
//        return(eores_NOK_nullpointer);    
//    } 
    
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != p), s_eobj_ownname, "doing reset of a NULL object");   
    
    p->val      = eo_iopinvalNONE;
    p->dir      = eo_iopindirNONE;
    p->defval   = eo_iopinvalNONE;
    p->status   = eo_iopinStatUNDEF;
    p->halpin   = eo_iopinposNONE;
    p->halport  = eo_iopinportNONE;

    return(eores_OK);            
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------



//extern eOresult_t eo_iopin_hid_SetVal(EOioPin *const p, eOiopinVal_t val) 
//{
//    eOresult_t res = eores_NOK_generic;
//    
//    if(NULL == p) 
//    {
//        return(eores_NOK_nullpointer);    
//    }
//    
//    if(eo_iopindirOUT == p->dir) 
//    {
//        p->val = val;    
//        // hardware dependency is encapulated in hal
//        hal_gpio_setval((hal_gpio_port_t)p->halport, (hal_gpio_pin_t)p->halpin, (hal_gpio_val_t)p->val);    
//        res = eores_OK;    
//    }
//    
//    return(res);
//}


//extern eOresult_t eo_iopin_hid_ToggleVal(EOioPin *const p)
//{
//    eOresult_t res = eores_NOK_generic;
//    
//    if(NULL == p) 
//    {
//        return(eores_NOK_nullpointer);    
//    }
//    
//    if((eo_iopindirOUT == p->dir) && (eo_iopinvalNONE != p->val)) 
//    {
//        p->val = (eo_iopinvalLOW == p->val) ? (eo_iopinvalHIGH) : (eo_iopinvalLOW);    
//        // hardware dependency are encapulated in hal
//        hal_gpio_setval((hal_gpio_port_t)p->halport, (hal_gpio_pin_t)p->halpin, (hal_gpio_val_t)p->val);    
//        res = eores_OK;    
//    }
//    
//    return(res);
//}


//extern eOiopinVal_t eo_iopin_hid_GetVal(EOioPin *const p)
//{
//    eOiopinVal_t ret = eo_iopinvalNONE;
//    
//    if(NULL == p) 
//    {
//        return(ret);
//    }    
//    
//    if(eo_iopindirNONE != p->dir) 
//    {
//        // hardware dependency are encapulated in hal
//        p->val = hal_gpio_getval((hal_gpio_port_t)p->halport, (hal_gpio_pin_t)p->halpin);
//        ret = (eOiopinVal_t)p->val;        
//    }
//    
//    return(ret);    
//}


//extern eOresult_t eo_iopin_hid_SetCfg(EOioPin *const p, eOiopinPort_t port, eOiopinPos_t pos, eOiopinVal_t val, eOiopinDir_t dir)
//{
//    if(NULL == p) 
//    {
//        return(eores_NOK_nullpointer);    
//    }    
//    
//    p->dir          = dir;        
//    p->halpin       = pos;
//    p->halport      = port;
//    
//    hal_gpio_init((hal_gpio_port_t)port, (hal_gpio_pin_t)pos, (hal_gpio_dir_t)dir, hal_gpio_speed_low);
//
//    eo_iopin_hid_SetVal(p, val);
//        
//    return(eores_OK);        
//}
//
//
//extern eOresult_t eo_iopin_hid_GetCfg(EOioPin *const p, eOiopinPort_t *pport, eOiopinPos_t *ppos, eOiopinDir_t *pdir) 
//{
//    if(NULL == p) 
//    {
//        return(eores_NOK_nullpointer);    
//    }    
//    
//    *pdir       = (eOiopinDir_t)p->dir;
//    *ppos       = (eOiopinPos_t)p->halpin;
//    *pport      = (eOiopinPort_t)p->halport;
//    
//    return(eores_OK);        
//}
//
//
//
//extern eOiopinStatus_t eo_iopin_hid_GetStatus(EOioPin *const p) 
//{
//    eOiopinStatus_t ret = eo_iopinStatUNDEF;
//    
//    if(NULL != p) 
//    {
//        ret = (eOiopinStatus_t) p->status;
//#if 0        
//        switch(p->status) 
//        {
//            case 0:        ret = ioStatNOTMNG;
//            break;
//            case 1:        ret = ioStatIDLE;
//            break;
//            case 2:        ret = ioStatDRIVEN;
//            break;
//            case 3:        ret = eo_iopinStatUNDEF;
//            break;
//        }
//#endif        
//        
//    }
//    
//    return(ret);
//
//}
//
//
//extern eOresult_t eo_iopin_hid_SetStatus(EOioPin *const p, eOiopinStatus_t st) 
//{
//    if(NULL == p) 
//    {
//        return(eores_NOK_nullpointer);    
//    }
//
//    p->status = st; // we could do the masking, but i assume the compiler already does it: ((uint8_t)st) & 0x03;
//
//    return(eores_OK);
//}
//
//
//extern eOresult_t eo_iopin_hid_Reset(EOioPin *const p) 
//{
////    if(NULL == p) 
////    {
////        return(eores_NOK_nullpointer);    
////    } 
//    
//    eo_errman_Assert(eo_errman_GetHandle(), (NULL != p), s_eobj_ownname, "doing reset of a NULL object");   
//    
//    p->val      = eo_iopinvalNONE;
//    p->dir      = eo_iopindirNONE;
//    p->defval   = eo_iopinvalNONE;
//    p->status   = eo_iopinStatUNDEF;
//    p->halpin   = eo_iopinposNONE;
//    p->halport  = eo_iopinportNONE;
//
//    return(eores_OK);            
//}



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



