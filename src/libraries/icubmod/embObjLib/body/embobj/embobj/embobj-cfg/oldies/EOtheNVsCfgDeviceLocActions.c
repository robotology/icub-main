
/** @file       EOtheNVsCfgDeviceLocActions.c
    @brief      This file implements internal implementation to the configuration of the NVs of a device
    @author     marco.accame@iit.it
    @date       10/15/2009
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h" 
#include "stdio.h"  
#include "string.h"

#include "EoCommon.h"
#include "EOnetvar_hid.h"
#include "EOnetvar.h"
#include "EOvport_hid.h"
#include "EOtheNVsCfgDeviceLoc_hid.h"




// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgDeviceLocActions.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgDeviceLocActions_hid.h" 


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
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern void nvscfg_device_loc_vportROnly_configure(void *p) 
{
    eo_vport_hid_LoadCfg(&loc_device_vol.vportROnly, &loc_device_vol.vportROnly.cfg, eo_nv_ownership_local, 0);
}

extern void nvscfg_device_loc_vportWOnly_update(void *p)
{
    eo_vport_hid_MoveDat2NVs(&loc_device_vol.vportWOnly, eo_nv_ownership_local, 0);
}

extern void nvscfg_device_loc_act_STARTUP_var_globalconfiguration(void *p)
{
     EOnetvar *nv = (EOnetvar*)p;
     nv = nv;
     
     // 1. initialise some static data so that .....
     
     // 2. init a mutex mutex_globcfg.
     
     // init a variable which is a volatile pointer to the whole globalconfig value in ram: globcfg
     
     // 3. upon value of globcfg->withperiodinmicrosecs start a periodic timer with callback s_do_it_and_also_that()
}


extern void nvscfg_device_loc_act_BEF_SET_var_globalconfiguration(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    
    // take the mutex mutex_globcfg
 
}

extern void nvscfg_device_loc_act_AFT_SET_var_globalconfiguration(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    
    // stop the timer
    // start the timer with a period globcfg->withperiodinmicrosecs
    
    // release the mutex mutex_globcfg
 
}

extern void nvscfg_device_loc_act_BEF_SET_var__doit(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    
    // take the mutex mutex_globcfg
 
}

extern void nvscfg_device_loc_act_AFT_SET_var__doit(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    
    // release the mutex mutex_globcfg
 
}


extern void nvscfg_device_loc_act_BEF_SET_var__doalsothat(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    
    // take the mutex mutex_globcfg
 
}

extern void nvscfg_device_loc_act_AFT_SET_var__doalsothat(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    
    // release the mutex mutex_globcfg
 
}

extern void nvscfg_device_loc_act_BEF_SET_var__withperiodinmicrosecs(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    
    // take the mutex mutex_globcfg
    
    // stop the timer
 
}

extern void nvscfg_device_loc_act_AFT_SET_var__withperiodinmicrosecs(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    
    // start the timer with a new period globcfg->withperiodinmicrosecs
    
    // release the mutex mutex_globcfg
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

void s_do_it_and_also_that(void)
{
    // take the mutex with zero timeout
    // if fail then return
    // else 
    // { if globcfg->doit is true ... toggle led1
    //   if globcfg->doalsothat is true ... toggle led2
    // }
}





// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



