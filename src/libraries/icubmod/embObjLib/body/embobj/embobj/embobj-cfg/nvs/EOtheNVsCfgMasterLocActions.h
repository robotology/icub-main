
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGMASTERLOCACTIONS_H_
#define _EOTHENVSCFGMASTERLOCACTIONS_H_




/** @file       EOtheNVsCfgMasterLocActions.h
	@brief      This header file implements public interface to ...
	@author     marco.accame@iit.it
	@date       09/08/2010
**/

/** @defgroup eo_thenvsconfigmasterlocactions Object EOtheNVsCfgMasterLocActions
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

extern const eOnetvar_fn_datainterface_t master_loc_datainterface_generic;
extern const eOnetvar_fn_datainterface_t master_loc_datainterface_nvOBJ_root;
extern const eOnetvar_fn_datainterface_t master_loc_datainterface_nvOBJ__isactive;

// - declaration of extern public functions ---------------------------------------------------------------------------


extern void master_loc_action_init_generic(void *p);
extern void master_loc_action_update_generic(void *p);

extern void master_loc_action_init_nvOBJ__isactive(void *p);
extern void master_loc_action_init_nvOBJ_root(void *p);

extern void master_loc_action_update_nvOBJ__isactive(void *p);
extern void master_loc_action_update_nvOBJ_root(void *p);



/** @}            
    end of group eo_thenvsconfigmasterlocactions  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




