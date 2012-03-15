
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOVTHEENVIRONMENT_HID_H_
#define _EOVTHEENVIRONMENT_HID_H_


/* @file       EOVtheEnvironment_hid.h
    @brief      This header file implements hidden interface to the base timer manager singleton.
    @author     marco.accame@iit.it
    @date       08/30/2011
**/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOVtheEnvironment.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
#define VF00_env_shareddata_sync        0
#define VF01_env_proc_offset_get        1
#define VF02_env_ipnet_get              2
#define VF03_env_cannets_get            3
#define VF04_env_eproc_get              4
#define VTABLESIZE_venv                 5



// - definition of the hidden struct implementing the object ----------------------------------------------------------


//typedef     eOresult_t  (*eOres_fp_venvp_t)                 (EOVtheEnvironment*);
//typedef     eOresult_t  (*eOres_fp_venvp_uint32p_t)         (EOVtheEnvironment*, uint32_t*);

/** @struct     EOVtheEnvironment_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOVtheEnvironment_hid 
{
    // - vtable: must be on top of the struct
    void * vtable[VTABLESIZE_venv];

}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------


/** @fn         extern EOVtheEnvironment * eov_env_hid_Initialise(eOres_fp_vcbkmanp_cbk_voidp_uint32_t execute_fn, EOVtaskDerived *task)
    @brief      Initialise the singleton EOVtheEnvironment. The function is hidden because this singleton can be used only
                by a derived object.

    @return     The handle to the singleton, or never return if any argument is NULL.
 
 **/

extern EOVtheEnvironment * eov_env_hid_Initialise(eOres_fp_voidp_t shareddata_sync, 
                                                  eOres_fp_voidp_uint32p_t proc_offset_get,
                                                  eOres_fp_voidp_cvoidpp_t ipnet_get,
                                                  eOres_fp_voidp_cvoidpp_uint8p_t cannets_get,
                                                  eOres_fp_voidp_uint8p_t eproc_get);



#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

