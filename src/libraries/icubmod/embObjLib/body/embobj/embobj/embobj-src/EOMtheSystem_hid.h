
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMTHESYSTEM_HID_H_
#define _EOMTHESYSTEM_HID_H_


/* @file       EOMtheSystem_hid.h
    @brief      This header file implements hidden interface to the base timer manager singleton.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVtheSystem.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOMtheSystem.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section



// - definition of the hidden struct implementing the object ----------------------------------------------------------

/* @struct     EOMtheSystem_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOMtheSystem_hid 
{
    // base object
    EOVtheSystem                *thevsys;

    // other stuff
    const osal_cfg_t            *osalcfg;
    const eOmtimerman_cfg_t     *tmrmancfg;
    const eOmcallbackman_cfg_t  *cbkmancfg;
    eOvoid_fp_void_t            user_init_fn;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------





#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

