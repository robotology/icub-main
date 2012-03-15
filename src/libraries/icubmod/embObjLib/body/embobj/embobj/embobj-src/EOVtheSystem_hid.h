
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOVTHESYSTEM_HID_H_
#define _EOVTHESYSTEM_HID_H_


/* @file       EOVtheSystem_hid.h
    @brief      This header file implements hidden interface to the base timer manager singleton.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

#include "EOVtask.h"
#include "EOtheMemoryPool.h"



// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOVtheSystem.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
#define VF00_start                      0
#define VF01_getcurrtask                1
#define VF02_timeget                    2
#define VF03_timeset                    3
#define VF04_nanotimeget                4
#define VF04_stop                       5
#define VTABLESIZE_sys                  6



// - definition of the hidden struct implementing the object ----------------------------------------------------------


/** @struct     EOVtheSystem_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOVtheSystem_hid 
{
    // - vtable: must be on top of the struct
    void * vtable[VTABLESIZE_sys];

    // - other stuff
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------


/** @fn         extern EOVtheSystem * eov_sys_hid_Initialise(const eOmempool_cfg_t *mpcfg,
                                             const eOerrman_cfg_t *emcfg,
                                             eOres_fp_voidfpvoid_t   start, 
                                             eOvoidp_fp_void_t              getcurrtask,
                                             eOuint64_fp_void_t             timeget,
                                             eOvoid_fp_uint64_t             timeset,
                                             eOvoid_fp_voidp_t              nanotimeget,
                                             eOvoid_fp_void_t               stop
                                             )
    @brief      Initialise the singleton EOVtheSystem. The function is hidden because this singleton can be used only
                by a derived object.
 
 **/

extern EOVtheSystem * eov_sys_hid_Initialise(const eOmempool_cfg_t *mpcfg,
                                             const eOerrman_cfg_t *emcfg,
                                             eOres_fp_voidfpvoid_t   start, 
                                             eOvoidp_fp_void_t              getcurrtask,
                                             eOuint64_fp_void_t             timeget,
                                             eOvoid_fp_uint64_t             timeset,
                                             eOuint64_fp_void_t             nanotimeget,
                                             eOvoid_fp_void_t               stop
                                             );

/** @fn         extern eOpurevirtual eOresult_t eov_sys_hid_Start(EOVtheSystem *p, void (*init_fn)(void))
    @brief      Starts the system.
    @param      p               The pointer to the system singleton. 
    @param      init_fn         The init function. 
    @return     eores_NOK_nullpointer if @e p is NULL, eores_OK. The behaviour however
                depends on how the parent object has specified this pure virtual method.
    @todo       document return
 **/
extern eOpurevirtual eOresult_t eov_sys_hid_Start(EOVtheSystem *p, void (*init_fn)(void));

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

