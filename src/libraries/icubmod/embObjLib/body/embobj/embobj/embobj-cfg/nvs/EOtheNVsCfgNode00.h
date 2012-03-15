
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGNODE00_H_
#define _EOTHENVSCFGNODE00_H_




/** @file       EOtheNVsCfgNode00.h
	@brief      This header file implements public interface to the configuration of the NVs for an example node
	@author     marco.accame@iit.it
	@date       09/08/2010
**/

/** @defgroup eo_thenvsconfignode00 Object EOtheNVsCfgNode00
    The EOtheNVsCfgNode00 is a singleton derived from the base object EOVtheNVsCfg which configures the NVs for a 
    typical master. 
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

#include "EOnetvar.h"



// - public #define  --------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgNode00Loc.h"


// - declaration of public user-defined types ------------------------------------------------------------------------- 

typedef struct EOtheNVsCfgNode00_hid EOtheNVsCfgNode00;
    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

 
/** @fn         extern const EOtheNVsCfgExample * eo_nvscfg_node00_GetHandle(void) 
    @brief      Gets the handle of the EoVtheNVsCfg derived object.
    @return     Pointer to the required EoVtheNVsCfg object. 
 **/
extern const EOtheNVsCfgNode00 * eo_nvscfg_node00_GetHandle(void);



/** @}            
    end of group eo_thenvsconfignode00  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




