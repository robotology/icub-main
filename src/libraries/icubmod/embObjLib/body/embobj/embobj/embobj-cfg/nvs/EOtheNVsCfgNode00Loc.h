
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGNODE00LOC_H_
#define _EOTHENVSCFGNODE00LOC_H_




/** @file       EOtheNVsCfgNode00Loc.h
	@brief      This header file implements public interface to the configuration of the NVs for an example application
	@author     marco.accame@iit.it
	@date       09/08/2010
**/

/** @defgroup eo_thenvsconfigmasterloc Object EOtheNVsCfgNode00Loc
  
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

#include "EOnetvar.h"
#include "EOVstorage.h"



// - public #define  --------------------------------------------------------------------------------------------------


//-- ip address of the device
#define node00_loc_ipaddr       EO_COMMON_IPV4ADDR(10, 1, 1, 0)


//-- number of the nvs manipulated with rops.
#define node00_loc_NUMNETVARS   27


//-- ids of variables which can be manipulated with rops. 
//-- each entity is separated by a blank row, a leaf nv ends with a __nameofvariable

#define node00_loc_nvID_root                                            eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 0)

#define node00_loc_nvID_vportRO                                         eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 1)
#define node00_loc_nvID_vportRO__cfg                                    eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_arr, 2)
#define node00_loc_nvID_vportRO__dat                                    eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_arr, 3)

#warning i dont think that vportWO__cfg is ever used in a local node because it is used the __dat instead
#define node00_loc_nvID_vportWO                                         eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 4)
#define node00_loc_nvID_vportWO__cfg                                    eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_arr, 5)
#define node00_loc_nvID_vportWO__dat                                    eo_nv_getID(eo_nv_FUN_out, eo_nv_TYP_arr, 6)

#define node00_loc_nvID_globalconstants                                 eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 7)
#define node00_loc_nvID_globcon__macaddr                                eo_nv_getID(eo_nv_FUN_con, eo_nv_TYP_u64, 8)
#define node00_loc_nvID_globcon__ipaddr                                 eo_nv_getID(eo_nv_FUN_con, eo_nv_TYP_u32, 9)

#define node00_loc_nvID_globalconfiguration                             eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 10)
#define node00_loc_nvID_globcfg__acquireinput                           eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 11)
#define node00_loc_nvID_globcfg__acquisitionperiod                      eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 12)
#define node00_loc_nvID_globcfg__applyoutput                            eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 13)
#define node00_loc_nvID_globcfg__signalvportro                          eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 14)
#define node00_loc_nvID_globcfg__toipaddr                               eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 15)
#define node00_loc_nvID_globcfg__withperiod                             eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 16)

#define node00_loc_nvID_button                                          eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 17)
#define node00_loc_nvID_button__inputval                                eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u08, 18)
#define node00_loc_nvID_button__acquisitiontime                         eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u64, 19)

#define node00_loc_nvID_led00                                           eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 20)
#define node00_loc_nvID_led00__outputval                                eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u08, 21)
#define node00_loc_nvID_led00__lednumber                                eo_nv_getID(eo_nv_FUN_con, eo_nv_TYP_u08, 22)

#define node00_loc_nvID_led01                                           eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 23)
#define node00_loc_nvID_led01__outputval                                eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u08, 24)
#define node00_loc_nvID_led01__lednumber                                eo_nv_getID(eo_nv_FUN_con, eo_nv_TYP_u08, 25)

#define node00_loc_nvID__timeoflife                                     eo_nv_getID(eo_nv_FUN_out, eo_nv_TYP_u64, 26)



// -- identifier of local storage
#define node00_loc_STGid                                                0

// - declaration of public user-defined types ------------------------------------------------------------------------- 
// empty-section
    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------


// - declaration of extern public functions ---------------------------------------------------------------------------

extern void node00_loc_init_volatile_data(void);

extern void* node00_loc_get_storage(void);




/** @}            
    end of group eo_thenvsconfigmasterloc  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




