
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGDEVICE_H_
#define _EOTHENVSCFGDEVICE_H_




/** @file       EOtheNVsCfgDevice.h
	@brief      This header file implements public interface to the configuration of the NVs for an example application
	@author     marco.accame@iit.it
	@date       09/08/2010
**/

/** @defgroup eo_thenvsconfigdevice Object EOtheNVsCfgDevice
    The EOtheNVsCfgDevice is a singleton derived from the base object EOVtheNVsCfg which configures the NVs for a 
    typical device. 
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

#include "EOnetvar.h"



// - public #define  --------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgDeviceLoc.h"
#include "EOtheNVsCfgDeviceRem00.h"


// - declaration of public user-defined types ------------------------------------------------------------------------- 

typedef struct EOtheNVsCfgDevice_hid EOtheNVsCfgDevice;
    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

 
/** @fn         extern const EOtheNVsCfgExample * eo_nvscfg_device_GetHandle(void) 
    @brief      Gets the handle of the EoVtheNVsCfg derived object.
    @return     Pointer to the required EoVtheNVsCfg object. 
 **/
extern const EOtheNVsCfgDevice * eo_nvscfg_device_GetHandle(void);



/** @}            
    end of group eo_thenvsconfigdevice  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




