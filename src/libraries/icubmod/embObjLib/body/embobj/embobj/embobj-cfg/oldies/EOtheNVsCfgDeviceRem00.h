
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGDEVICEREM00_H_
#define _EOTHENVSCFGDEVICEREM00_H_




/** @file       EOtheNVsCfgDeviceRem00.h
	@brief      This header file implements public interface to the configuration of the NVs for an example application
	@author     marco.accame@iit.it
	@date       09/08/2010
**/

/** @defgroup eo_thenvsconfigdevicerem Object EOtheNVsCfgDeviceRem00
    The EOtheNVsCfgDeviceRem00 is a singleton derived from the base object EOVtheNVsCfg which configures the NVs for a 
    typical device. 
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

#include "EOnetvar.h"



// - public #define  --------------------------------------------------------------------------------------------------


//-- ip address of the device manipulated by the smart node.
#define rem00_device_ipaddr       EO_COMMON_IPV4ADDR(10, 10, 1, 0)


//-- number of the nvs manipulated by the by the smart node.
#define rem00_device_NUMNETVARS   37


//-- ids of variables which can be manipulated by the smart node. 
//-- each entity is separated by a blank row, a leaf nv ends with a __nameofvariable

#define nvIDrem00device                                             eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 0)

#define nvIDrem00dev_vportROnly                                     eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 1)
#define nvIDrem00dev_vportROnly__cfg                                eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_arr, 2)
#define nvIDrem00dev_vportROnly__dat                                eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_arr, 3)

#define nvIDrem00dev_vportWOnly                                     eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 4)
#define nvIDrem00dev_vportWOnly__cfg                                eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_arr, 5)
#define nvIDrem00dev_vportWOnly__dat                                eo_nv_getID(eo_nv_FUN_out, eo_nv_TYP_arr, 6)

#define nvIDrem00dev_globalconfiguration                              eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 7)
#define nvIDrem00dev_globcfg__doit                                    eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 8)
#define nvIDrem00dev_globcfg__doalsothat                              eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 9)
#define nvIDrem00dev_globcfg__withperiodinmicrosecs                   eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 10)

#define nvIDrem00dev_input08                                          eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 11)
#define nvIDrem00dev_input08__input08value                            eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u08, 12)
#define nvIDrem00dev_input08__acquisitiontime                         eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u64, 13)
#define nvIDrem00dev_input08_inputconfiguration                       eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 14)
#define nvIDrem00dev_input08_inpcfg__acquisitionenabled               eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 15)
#define nvIDrem00dev_input08_inpcfg__acquisitionperiod                eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 16)

#define nvIDrem00dev_input16                                          eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 17)
#define nvIDrem00dev_input16__input16value                            eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u16, 18)
#define nvIDrem00dev_input16__acquisitiontime                         eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u64, 19)
#define nvIDrem00dev_input16_inputconfiguration                       eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 20)
#define nvIDrem00dev_input16_inpcfg__acquisitionenabled               eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 21)
#define nvIDrem00dev_input16_inpcfg__acquisitionperiod                eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 22)

#define nvIDrem00dev_input32                                          eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 23)
#define nvIDrem00dev_input32__input32value                            eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u32, 24)
#define nvIDrem00dev_input32__acquisitiontime                         eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u64, 25)
#define nvIDrem00dev_input32_inputconfiguration                       eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 26)
#define nvIDrem00dev_input32_inpcfg__acquisitionenabled               eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 27)
#define nvIDrem00dev_input32_inpcfg__acquisitionperiod                eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 28)

#define nvIDrem00dev_output                                           eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 29)
#define nvIDrem00dev_output__outputvalue                              eo_nv_getID(eo_nv_FUN_out, eo_nv_TYP_u32, 30)
#define nvIDrem00dev_output__applicationtime                          eo_nv_getID(eo_nv_FUN_out, eo_nv_TYP_u64, 31)
#define nvIDrem00dev_output_outputconfiguration                       eo_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 32)
#define nvIDrem00dev_output_outcfg__acquisitionenabled                eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 33)
#define nvIDrem00dev_output_outcfg__applicationmode                   eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 34)

#define nvIDrem00dev__fixedarray                                      eo_nv_getID(eo_nv_FUN_out, eo_nv_TYP_pkd, 35)

#define nvIDrem00dev__varsizearray                                    eo_nv_getID(eo_nv_FUN_out, eo_nv_TYP_arr, 36)


//-- indices for mirror of vportWO inside the smart node for the nvs whcih can be used with the vportWO.
//-- 1. the smart node shall fill the relevant nvIDs into a temporary vportcfg,
//-- 2. the smart node shall use eo_netvar_Set(nv_vportWOcfg, vportcfg, eobool_true, eo_nv_upd_always) to write its vportWO
//-- 3. the smart node shall send a set<nvIDrem00dev_vportWOnly__dat, data> to the device rem00 to set all
//--    the nvIDs inside vportWO with data automatically retrieved by the EOtheAgent 

#define nvWOmir_rem00dev_output__outputvalue                                0
#define nvWOmir_rem00dev_output__applicationtime                            1
#define nvWOmir_rem00dev_output__outputconfiguration                        2
#define nvWOmir_rem00dev_output_outcfg__acquisitionenabled                  3
#define nvWOmir_rem00dev_output_outcfg__applicationmode                     4
#define nvWOmir_rem00dev__fixedarray                                        5
#define nvWOmir_rem00dev__varsizearray                                      6

//-- indices for mirror of vportRO inside the local node for the nvs whcih can be used with the vportRO.
//-- 1. the smart node shall fill the relevant nvIDs into a temporary vportcfg,
//-- 2. the smart node shall use eo_netvar_Set(nv_vportROcfg, vportcfg, eobool_true, eo_nv_upd_dontdo) to write its vportRO
//-- 3. the smart node shall send a set<nvIDrem00dev_vportROnly__cfg, data> to the device rem00 to configure the
//--    vportRO of device rem00 to keep all the relevant nvIDs
//-- 4. the node rem00 can send a sig<nvIDrem00dev_vportROnly__dat, data> or a say<> to the smart node.
#define nvROmir_rem00dev_input08__input08value                              0
#define nvROmir_rem00dev_input16__input16value                              1
#define nvROmir_rem00dev_input32__input32value                              2
#define nvROmir_rem00dev_output__outputvalue                                3
#define nvROmir_rem00dev_output__applicationtime                            4
#define nvROmir_rem00dev__varsizearray                                      5

  

// - declaration of public user-defined types ------------------------------------------------------------------------- 
// empty-section
    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------



// - declaration of extern public functions ---------------------------------------------------------------------------

extern void nvscfg_device_rem00_init_volatile_data(void);


/** @}            
    end of group eo_thenvsconfigdevicerem  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




