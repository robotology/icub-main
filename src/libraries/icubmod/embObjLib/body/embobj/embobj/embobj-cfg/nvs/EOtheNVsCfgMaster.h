
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGMASTER_H_
#define _EOTHENVSCFGMASTER_H_




/** @file       EOtheNVsCfgMaster.h
	@brief      This header file implements public interface to the configuration of the NVs for an example master
	@author     marco.accame@iit.it
	@date       09/08/2010
**/

/** @defgroup eo_thenvsconfigmaster Object EOtheNVsCfgMaster
    The EOtheNVsCfgMaster is a singleton derived from the base object EOVtheNVsCfg which configures the NVs for a 
    typical master. 
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

#include "EOnetvar.h"



// - public #define  --------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgMasterLoc.h"
#include "EOtheNVsCfgMasterR00.h"


// - declaration of public user-defined types ------------------------------------------------------------------------- 

typedef struct EOtheNVsCfgMaster_hid EOtheNVsCfgMaster;
    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

 
/** @fn         extern const EOtheNVsCfgExample * eo_nvscfg_master_GetHandle(void) 
    @brief      Gets the handle of the EoVtheNVsCfg derived object.
    @return     Pointer to the required EoVtheNVsCfg object. 
 **/
extern const EOtheNVsCfgMaster * eo_nvscfg_master_GetHandle(void);



/** @}            
    end of group eo_thenvsconfigdevice  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




