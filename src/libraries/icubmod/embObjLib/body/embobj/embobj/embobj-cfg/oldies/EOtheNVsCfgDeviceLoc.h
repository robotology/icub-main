
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGDEVICELOC_H_
#define _EOTHENVSCFGDEVICELOC_H_




/** @file       EOtheNVsCfgDeviceLoc.h
	@brief      This header file implements public interface to the configuration of the NVs for an example application
	@author     marco.accame@iit.it
	@date       09/08/2010
**/

/** @defgroup eo_thenvsconfigdeviceloc Object EOtheNVsCfgDeviceLoc
    The EOtheNVsCfgDeviceLoc is a singleton derived from the base object EOVtheNVsCfg which configures the NVs for a 
    typical device. 
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

#include "EOnetvar.h"
#include "EOVstorage.h"



// - public #define  --------------------------------------------------------------------------------------------------


//-- ip address of the device
#define loc_device_ipaddr       EO_COMMON_IPV4ADDR(10, 1, 1, 0)


//-- number of the nvs manipulated with rops.
#define loc_device_NUMNETVARS   37


//-- ids of variables which can be manipulated with rops. 
//-- each entity is separated by a blank row, a leaf nv ends with a __nameofvariable

#define nvIDlocdevice                                                 eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 0)

#define nvIDlocdev_vportROnly                                         eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 1)
#define nvIDlocdev_vportROnly__cfg                                    eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_arr, 2)
#define nvIDlocdev_vportROnly__dat                                    eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_arr, 3)


#define nvIDlocdev_vportWOnly                                         eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 4)
#define nvIDlocdev_vportWOnly__cfg                                    eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_arr, 5)
#define nvIDlocdev_vportWOnly__dat                                    eo_nv_getID(eo_nv_FUN_out, eo_nv_TYP_arr, 6)

#define nvIDlocdev_globalconfiguration                              eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 7)
#define nvIDlocdev_globcfg__doit                                    eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 8)
#define nvIDlocdev_globcfg__doalsothat                              eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 9)
#define nvIDlocdev_globcfg__withperiodinmicrosecs                   eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 10)

#define nvIDlocdev_input08                                          eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 11)
#define nvIDlocdev_input08__input08value                            eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u08, 12)
#define nvIDlocdev_input08__acquisitiontime                         eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u64, 13)
#define nvIDlocdev_input08_inputconfiguration                       eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 14)
#define nvIDlocdev_input08_inpcfg__acquisitionenabled               eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 15)
#define nvIDlocdev_input08_inpcfg__acquisitionperiod                eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 16)

#define nvIDlocdev_input16                                          eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 17)
#define nvIDlocdev_input16__input16value                            eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u16, 18)
#define nvIDlocdev_input16__acquisitiontime                         eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u64, 19)
#define nvIDlocdev_input16_inputconfiguration                       eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 20)
#define nvIDlocdev_input16_inpcfg__acquisitionenabled               eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 21)
#define nvIDlocdev_input16_inpcfg__acquisitionperiod                eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 22)

#define nvIDlocdev_input32                                          eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 23)
#define nvIDlocdev_input32__input32value                            eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u32, 24)
#define nvIDlocdev_input32__acquisitiontime                         eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u64, 25)
#define nvIDlocdev_input32_inputconfiguration                       eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 26)
#define nvIDlocdev_input32_inpcfg__acquisitionenabled               eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 27)
#define nvIDlocdev_input32_inpcfg__acquisitionperiod                eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 28)

#define nvIDlocdev_output                                           eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 29)
#define nvIDlocdev_output__outputvalue                              eo_nv_getID(eo_nv_FUN_out, eo_nv_TYP_u32, 30)
#define nvIDlocdev_output__applicationtime                          eo_nv_getID(eo_nv_FUN_out, eo_nv_TYP_u64, 31)
#define nvIDlocdev_output_outputconfiguration                       eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 32)
#define nvIDlocdev_output_outcfg__acquisitionenabled                eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 33)
#define nvIDlocdev_output_outcfg__applicationmode                   eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 34)



#define nvIDlocdev__fixedarray                                      eo_nv_getID(eo_nv_FUN_out, eo_nv_TYP_pkd, 35)

#define nvIDlocdev__varsizearray                                    eo_nv_getID(eo_nv_FUN_out, eo_nv_TYP_arr, 36)


//-- indices for mirror of vportRO inside the local node for the nvs whcih can be used with the vportRO.
//-- if those NVs are written inside the vportRO.cfg, then the mirror of those variables will be used to
//-- replicate in vportRO.dat the relevant value,
#define nvROmir_locdev_input08__input08value                              0
#define nvROmir_locdev_input16__input16value                              1
#define nvROmir_locdev_input32__input32value                              2
#define nvROmir_locdev_output__outputvalue                                3
#define nvROmir_locdev_output__applicationtime                            4
#define nvROmir_locdev__varsizearray                                      5


// - declaration of public user-defined types ------------------------------------------------------------------------- 
// empty-section
    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------


// - declaration of extern public functions ---------------------------------------------------------------------------

extern void nvscfg_device_loc_init_volatile_data(void);

extern void * nvscfg_device_loc_get_storage(void);




/** @}            
    end of group eo_thenvsconfigdeviceloc  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




