
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGDEVICELOCACTIONS_H_
#define _EOTHENVSCFGDEVICELOCACTIONS_H_




/** @file       EOtheNVsCfgDeviceLocActions.h
	@brief      This header file implements public interface to the configuration of the NVs for an example application
	@author     marco.accame@iit.it
	@date       09/08/2010
**/

/** @defgroup eo_thenvsconfigdevicelocactions Object EOtheNVsCfgDeviceLocActions
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
// empty-section

// - declaration of extern public functions ---------------------------------------------------------------------------


extern void nvscfg_device_loc_vportROnly_configure(void *p);

extern void nvscfg_device_loc_vportWOnly_update(void *p);

extern void nvscfg_device_loc_act_STARTUP_var_globalconfiguration(void *p);

extern void nvscfg_device_loc_act_BEF_SET_var_globalconfiguration(void *p);

extern void nvscfg_device_loc_act_AFT_SET_var_globalconfiguration(void *p);


extern void nvscfg_device_loc_act_BEF_SET_var__doit(void *p);

extern void nvscfg_device_loc_act_AFT_SET_var__doit(void *p);

extern void nvscfg_device_loc_act_BEF_SET_var__doalsothat(void *p);

extern void nvscfg_device_loc_act_AFT_SET_var__doalsothat(void *p);

extern void nvscfg_device_loc_act_BEF_SET_var__withperiodinmicrosecs(void *p);

extern void nvscfg_device_loc_act_AFT_SET_var__withperiodinmicrosecs(void *p);


/** @}            
    end of group eo_thenvsconfigdevicelocactions  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




