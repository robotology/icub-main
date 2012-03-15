
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGNODE00LOCACTIONS_H_
#define _EOTHENVSCFGNODE00LOCACTIONS_H_




/** @file       EOtheNVsCfgNode00LocActions.h
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

extern const eOnetvar_fn_datainterface_t node00_loc_action_datainterface_nvOBJ_root;
extern const eOnetvar_fn_datainterface_t node00_loc_action_datainterface_generic;
extern const eOnetvar_fn_datainterface_t node00_loc_action_datainterface_nvOBJ_vportRO__cfg;
extern const eOnetvar_fn_datainterface_t node00_loc_action_datainterface_nvOBJ_vportRO__dat;
extern const eOnetvar_fn_datainterface_t node00_loc_action_datainterface_nvOBJ_globalconfiguration__any;
extern const eOnetvar_fn_datainterface_t node00_loc_action_datainterface_nvOBJ_button;
extern const eOnetvar_fn_datainterface_t node00_loc_action_datainterface_nvOBJ_button__inputval;
extern const eOnetvar_fn_datainterface_t node00_loc_action_datainterface_nvOBJ_led00;
extern const eOnetvar_fn_datainterface_t node00_loc_action_datainterface_nvOBJ_led00__outputval;
extern const eOnetvar_fn_datainterface_t node00_loc_action_datainterface_nvOBJ_led01;
extern const eOnetvar_fn_datainterface_t node00_loc_action_datainterface_nvOBJ_led01__outputval;
extern const eOnetvar_fn_datainterface_t node00_loc_action_datainterface_nvOBJ__timeoflife;

// - declaration of extern public functions ---------------------------------------------------------------------------
// empty-section

/** @}            
    end of group eo_thenvsconfigmasterlocactions  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




