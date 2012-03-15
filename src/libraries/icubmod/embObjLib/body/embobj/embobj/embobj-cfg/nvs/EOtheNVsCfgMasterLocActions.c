
/** @file       EOtheNVsCfgMasterLocActions.c
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
#include "EOtheNVsCfgMasterLoc_hid.h"






// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgMasterLocActions.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgMasterLocActions_hid.h" 


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

extern const eOnetvar_fn_datainterface_t master_loc_datainterface_generic =
{
    .init       = master_loc_action_init_generic,
    .update     = master_loc_action_update_generic
};


extern const eOnetvar_fn_datainterface_t master_loc_datainterface_nvOBJ_root =
{
    .init       = master_loc_action_init_nvOBJ_root,
    .update     = master_loc_action_update_nvOBJ_root
};


extern const eOnetvar_fn_datainterface_t master_loc_datainterface_nvOBJ__isactive =
{
    .init       = master_loc_action_init_nvOBJ__isactive,
    .update     = master_loc_action_update_nvOBJ__isactive
};
// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern void master_loc_action_init_generic(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    printf("initted: generic nv w/ id = %d\n", nv->id);
}

extern void master_loc_action_update_generic(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    printf("updated: generic nv w/ id = %d\n", nv->id);
}

extern void master_loc_action_init_nvOBJ__isactive(void *p)
{
    printf("initted: nvOBJ__isactive\n");
}

extern void master_loc_action_init_nvOBJ_root(void *p)
{
    printf("initted: nvOBJ_root\n");
}

extern void master_loc_action_update_nvOBJ__isactive(void *p)
{
    printf("updated: nvOBJ__isactive\n");
}

extern void master_loc_action_update_nvOBJ_root(void *p)
{
    printf("updated: nvOBJ_root\n");
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



