
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOCFG_NVS_NODE_LOC_ACT_H_
#define _EOCFG_NVS_NODE_LOC_ACT_H_




/** @file       eOcfg_NVs_node00_loc_act.h
	@brief      This header file implements public interface to ...
	@author     marco.accame@iit.it
	@date       09/08/2010
**/

/** @defgroup eo_thenvsconfigmasterlocactions Object EOtheNVsCfgNode00LocActions
    In here the user defines what the NVs (or entities) of the device should do on some situations:
    - on  startup:   typically it is defined by entity and starts some activity upon some configuration nvs
    - on  update:    typically it is defined by leaf which is an input or output and contains the rules of
                     acquiring / applying the inp/ out value.
    - bef operation: done before exec of a command. typically the local device uses bef_{set, rst, upd} for cfg and out and bef_{upd} for inp
    - aft oepration: done after exec of a command. typically ....  
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

#include "EOnetvar.h"
#include "EOVstorage.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section


// - declaration of public user-defined types ------------------------------------------------------------------------- 
// empty-section
    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_root;
extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_generic;
extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_vportRO__cfg;
extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_vportRO__dat;
extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_globalconfiguration__any;
extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_button;
extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_button__inputval;
extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_led00;
extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_led00__outputval;
extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_led01;
extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_led01__outputval;
extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ__timeoflife;

// - declaration of extern public functions ---------------------------------------------------------------------------

extern void node00_loc_action_init_generic(void *p);
extern void node00_loc_action_update_generic(void *p);

extern void node00_loc_action_init_nvOBJ_root(void *p);
extern void node00_loc_action_update_nvOBJ_root(void *p);

extern void node00_loc_action_init_nvOBJ_vportRO__cfg(void *p);
extern void node00_loc_action_update_nvOBJ_vportRO__cfg(void *p);

extern void node00_loc_action_init_nvOBJ_vportRO__dat(void *p);

extern void node00_loc_action_update_nvOBJ_globalconfiguration__any(void *p);

extern void node00_loc_action_init_nvOBJ_button(void *p);
extern void node00_loc_action_update_nvOBJ_button__inputval(void *p);

extern void node00_loc_action_init_nvOBJ_led00(void *p);
extern void node00_loc_action_update_nvOBJ_led00__outputval(void *p);

extern void node00_loc_action_init_nvOBJ_led01(void *p);
extern void node00_loc_action_update_nvOBJ_led01__outputval(void *p);

extern void node00_loc_action_init_nvOBJ__timeoflife(void *p);
extern void node00_loc_action_update_nvOBJ__timeoflife(void *p);

/** @}            
    end of group eo_thenvsconfigmasterlocactions  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




