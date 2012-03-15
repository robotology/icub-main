
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGMASTERLOC_H_
#define _EOTHENVSCFGMASTERLOC_H_




/** @file       EOtheNVsCfgMasterLoc.h
	@brief      This header file implements public interface to the configuration of the NVs for an example application
	@author     marco.accame@iit.it
	@date       09/08/2010
**/

/** @defgroup eo_thenvsconfigmasterloc Object EOtheNVsCfgMasterLoc
  
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

#include "EOnetvar.h"
#include "EOVstorage.h"



// - public #define  --------------------------------------------------------------------------------------------------


//-- ip address of the device
#define master_loc_ipaddr       EO_COMMON_IPV4ADDR(10, 1, 1, 100)


//-- number of the nvs manipulated with rops.
#define master_loc_NUMNETVARS   8


//-- ids of variables which can be manipulated with rops. 
//-- each entity is separated by a blank row, a leaf nv ends with a __nameofvariable

#define master_loc_nvID_root                                            eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 0)

#define master_loc_nvID_globalconstants                                 eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 1)
#define master_loc_nvID_globcon__macaddr                                eo_nv_getID(eo_nv_FUN_con, eo_nv_TYP_u64, 2)
#define master_loc_nvID_globcon__ipaddr                                 eo_nv_getID(eo_nv_FUN_con, eo_nv_TYP_u32, 3)

#define master_loc_nvID_globalconfiguration                             eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 4)
#define master_loc_nvID_globcfg__doit                                   eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 5)
#define master_loc_nvID_globcfg__doalsothat                             eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 6)

#define master_loc_nvID__isactive                                       eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_u08, 7)

//-- identifier of local storage
#define master_loc_storage_ID                                           100



// - declaration of public user-defined types ------------------------------------------------------------------------- 
// empty-section
    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------


// - declaration of extern public functions ---------------------------------------------------------------------------

extern void master_loc_init_volatile_data(void);

extern void* master_loc_get_storage(void);




/** @}            
    end of group eo_thenvsconfigmasterloc  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




