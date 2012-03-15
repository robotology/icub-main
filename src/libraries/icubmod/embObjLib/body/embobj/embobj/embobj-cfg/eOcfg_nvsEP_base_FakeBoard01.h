
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOCFG_NVSEP_BASE_USR_LOC_ANYDEV_H_
#define _EOCFG_NVSEP_BASE_USR_LOC_ANYDEV_H_




/** @file       eOcfg_nvsEP_base_usr_loc_anydev.h
	@brief      This header file gives the configuration for the NVs in the management port of any device
	@author     marco.accame@iit.it
	@date       09/06/2011
**/

/** @defgroup eo_asfdgr Configuation of the NVs for the updater
    Tcecece 
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOconstvector.h"
#include "eOcfg_nvsEP_base_con.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section


// - declaration of public user-defined types ------------------------------------------------------------------------- 
// empty-section

    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const EOconstvector* const eo_cfg_nvsEP_base_usr_loc_anydev_constvector_of_EOnv_usr;

extern eo_cfg_nvsEP_base_t* eo_cfg_nvsEP_base_usr_loc_anydev_mem_local;



// - declaration of extern public functions ---------------------------------------------------------------------------

extern void eo_cfg_nvsEP_base_usr_loc_anydev_initialise(void* loc, void* rem);


/** @}            
    end of group eo_asfdgr  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




