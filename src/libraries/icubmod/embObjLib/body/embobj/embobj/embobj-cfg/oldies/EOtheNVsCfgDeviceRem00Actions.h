
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGDEVICEREM00ACTIONS_H_
#define _EOTHENVSCFGDEVICEREM00ACTIONS_H_




/** @file       EOtheNVsCfgDeviceRem00Actions.h
	@brief      This header file implements public interface to the configuration of the NVs for an example application
	@author     marco.accame@iit.it
	@date       09/08/2010
**/

/** @defgroup eo_thenvsconfigdeviceremactions Object EOtheNVsCfgDeviceRem00Actions
    In here the user defines what the device shoud do after it receives message related to NVs (or entities) of other devices.
    - on  startup:   never used
    - on  update:    never used.
    - bef operation: never he device receives a sig or say command, it copies the value inside the relevant nv. the
                     functions aft_{sig, say} can then be used to do something with this data. for instance to send 
                     it to another process or ....                     
    
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

extern const eOnetvar_fn_datainterface_t ondata_r00_generic_datainterface;

extern const eOnetvar_fn_datainterface_t ondata_r00_vportROnly__dat;

extern const eOnetvar_fn_datainterface_t ondata_r00_vportWOnly__cfg;

extern const eOnetvar_fn_datainterface_t ondata_r00_input08__input08value;

extern const eOnetvar_fn_datainterface_t ondata_r00_input16__input16value;

extern const eOnetvar_fn_datainterface_t ondata_r00_output__outputvalue;

extern const eOnetvar_fn_datainterface_t ondata_r00_output__applicationtime;

extern const eOnetvar_onrop_rx_t onrop_r00_generic_onrop;
extern const eOnetvar_onrop_rx_t onrop_r00_vportROnly__dat;
extern const eOnetvar_onrop_rx_t onrop_r00_vportWOnly__dat;
extern const eOnetvar_onrop_rx_t onrop_r00_input08__input08value;
extern const eOnetvar_onrop_rx_t onrop_r00_input16__input16value;

// - declaration of extern public functions ---------------------------------------------------------------------------



extern void nvscfg_device_rem00_update_generic(void *p);
extern void nvscfg_device_rem00_configure_vportWOnly(void *p);
extern void nvscfg_device_rem00_outputvalue_update(void *p);
extern void nvscfg_device_rem00_applicationtime_update(void *p);

extern void nvscfg_device_rem00_input08value_update(void *p);
extern void nvscfg_device_rem00_act_AFT_SIG_var_input08value(void *p);
extern void nvscfg_device_rem00_act_AFT_SAY_var_input08value(void *p);


extern void nvscfg_device_rem00_input16value_update(void *p);
extern void nvscfg_device_rem00_act_AFT_SIG_var_input16value(void *p);
extern void nvscfg_device_rem00_act_AFT_SAY_var_input16value(void *p);

extern void nvscfg_device_rem00_move_vportROnly_dat_to_nvs(void *p);

extern void nvscfg_device_rem00_act_AFT_SIG_var_vportROnly_dat(void *p);
extern void nvscfg_device_rem00_act_AFT_SAY_var_vportROnly_dat(void *p);

extern void nvscfg_device_rem00_act_AFT_SIG_var_vportWOnly_dat(void *p);
extern void nvscfg_device_rem00_act_AFT_SAY_var_vportWOnly_dat(void *p);


extern void nvscfg_device_rem00_before_sig(void *p);
extern void nvscfg_device_rem00_after_sig(void *p);
extern void nvscfg_device_rem00_before_say(void *p);
extern void nvscfg_device_rem00_after_say(void *p);



/** @}            
    end of group eo_thenvsconfigdeviceremactions  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




