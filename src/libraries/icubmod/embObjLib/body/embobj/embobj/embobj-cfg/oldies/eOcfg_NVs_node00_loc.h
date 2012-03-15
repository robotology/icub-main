
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGNODE00LOC_H_
#define _EOTHENVSCFGNODE00LOC_H_




/** @file       eOcfg_NVs_node00_loc.h
	@brief      This header file implements public interface to the configuration of the NVs for an example application
	@author     marco.accame@iit.it
	@date       09/06/2011
**/

/** @defgroup eo_thenvsconfigmasterloc Object EOtheNVsCfgNode00Loc
  
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

#include "EOnetvar.h"
#include "EOnetvarNode.h"
#include "EOvport.h"



// - public #define  --------------------------------------------------------------------------------------------------




//-- number of the nvs manipulated with rops.
#define EOK_cfg_nvs_node00_loc_NUMNETVARS   27


//-- ids of variables which can be manipulated with rops. 
//-- each entity is separated by a blank row, a leaf nv ends with a __nameofvariable

#define EOK_cfg_nvs_node00_loc_nvID_root                                            EO_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 0)

#define EOK_cfg_nvs_node00_loc_nvID_vportRO                                         EO_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 1)
#define EOK_cfg_nvs_node00_loc_nvID_vportRO__cfg                                    EO_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_arr, 2)
#define EOK_cfg_nvs_node00_loc_nvID_vportRO__dat                                    EO_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_arr, 3)

#define EOK_cfg_nvs_node00_loc_nvID_vportWO                                         EO_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 4)
#define EOK_cfg_nvs_node00_loc_nvID_vportWO__cfg                                    EO_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_arr, 5)
#define EOK_cfg_nvs_node00_loc_nvID_vportWO__dat                                    EO_nv_getID(eo_nv_FUN_out, eo_nv_TYP_arr, 6)

#define EOK_cfg_nvs_node00_loc_nvID_globalconstants                                 EO_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 7)
#define EOK_cfg_nvs_node00_loc_nvID_globcon__macaddr                                EO_nv_getID(eo_nv_FUN_con, eo_nv_TYP_u64, 8)
#define EOK_cfg_nvs_node00_loc_nvID_globcon__ipaddr                                 EO_nv_getID(eo_nv_FUN_con, eo_nv_TYP_u32, 9)

#define EOK_cfg_nvs_node00_loc_nvID_globalconfiguration                             EO_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 10)
#define EOK_cfg_nvs_node00_loc_nvID_globcfg__acquireinput                           EO_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 11)
#define EOK_cfg_nvs_node00_loc_nvID_globcfg__acquisitionperiod                      EO_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 12)
#define EOK_cfg_nvs_node00_loc_nvID_globcfg__applyoutput                            EO_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 13)
#define EOK_cfg_nvs_node00_loc_nvID_globcfg__signalvportro                          EO_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 14)
#define EOK_cfg_nvs_node00_loc_nvID_globcfg__toipaddr                               EO_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 15)
#define EOK_cfg_nvs_node00_loc_nvID_globcfg__withperiod                             EO_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 16)

#define EOK_cfg_nvs_node00_loc_nvID_button                                          EO_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 17)
#define EOK_cfg_nvs_node00_loc_nvID_button__inputval                                EO_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u08, 18)
#define EOK_cfg_nvs_node00_loc_nvID_button__acquisitiontime                         EO_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u64, 19)

#define EOK_cfg_nvs_node00_loc_nvID_led00                                           EO_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 20)
#define EOK_cfg_nvs_node00_loc_nvID_led00__outputval                                EO_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u08, 21)
#define EOK_cfg_nvs_node00_loc_nvID_led00__lednumber                                EO_nv_getID(eo_nv_FUN_con, eo_nv_TYP_u08, 22)

#define EOK_cfg_nvs_node00_loc_nvID_led01                                           EO_nv_getID(eo_nv_FUN_mix, eo_nv_TYP_pkd, 23)
#define EOK_cfg_nvs_node00_loc_nvID_led01__outputval                                EO_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u08, 24)
#define EOK_cfg_nvs_node00_loc_nvID_led01__lednumber                                EO_nv_getID(eo_nv_FUN_con, eo_nv_TYP_u08, 25)

#define EOK_cfg_nvs_node00_loc_nvID__timeoflife                                     EO_nv_getID(eo_nv_FUN_out, eo_nv_TYP_u64, 26)



// -- identifier of local storage
#define EOK_cfg_nvs_node00_loc_STGid                                                0


// - declaration of public user-defined types ------------------------------------------------------------------------- 


typedef __packed struct
{
	uint64_t			                macaddr;
	uint32_t			                ipaddr;
} eOcfg_nvs_node00_loc_globalconstants_t;


typedef __packed struct
{
	uint8_t			                    acquireinput;
    uint32_t                            acquisitionperiod;
 	uint8_t			                    applyoutput;
    uint8_t                             signalvportro;
    uint32_t                            toipaddr;
    uint32_t                            withperiod;
} eOcfg_nvs_node00_loc_globalconfiguration_t;


typedef __packed struct
{
	uint8_t			                    inputval;
    uint64_t                            acquisitiontime;
} eOcfg_nvs_node00_loc_input_t;

typedef __packed struct
{
	uint8_t			                    outputval;
    uint8_t			                    lednumber;
} eOcfg_nvs_node00_loc_output_t;

typedef __packed struct
{
    EOvport                                     vportRO;
    EOvport                                     vportWO;        
	eOcfg_nvs_node00_loc_globalconstants_t      globalconstants;
	eOcfg_nvs_node00_loc_globalconfiguration_t  globalconfiguration;
    eOcfg_nvs_node00_loc_input_t                button;
    eOcfg_nvs_node00_loc_output_t               led00;
    eOcfg_nvs_node00_loc_output_t               led01;
    uint64_t                                    timeoflife;
} eOcfg_nvs_node00_loc_t;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const EOnetvarNode eo_cfg_nvs_node00_loc_thenetvarnodes[];

extern eOcfg_nvs_node00_loc_t eo_cfg_nvs_node00_loc_vol;

// - declaration of extern public functions ---------------------------------------------------------------------------

extern void eo_cfg_nvs_node00_loc_init_volatile_data(void);

extern void* eo_cfg_nvs_node00_loc_get_storage(void);




/** @}            
    end of group eo_thenvsconfigmasterloc  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




