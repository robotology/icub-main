
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGEXAMPLE_H_
#define _EOTHENVSCFGEXAMPLE_H_


#error dont use it


/** @file       EOtheNVsCfgExample.h
	@brief      This header file implements public interface to the configuration of the NVs for an example application
	@author     marco.accame@iit.it
	@date       09/08/2010
**/

/** @defgroup eo_thenvsconfigexample Object EOtheNVsCfgExample
    The EOtheNVsCfgExample is a singleton derived from the base object EOVtheNVsCfg which configures the NVs for an
    example application. 
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section

  

// - declaration of public user-defined types ------------------------------------------------------------------------- 

typedef struct EOtheNVsCfgExample_hid EOtheNVsCfgExample ;
    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

 
/** @fn         extern const EOtheNVsCfgExample * eo_nvscfg_example_GetHandle(void) 
    @brief      Gets the handle of the EoVtheNVsCfg derived object.
    @return     Pointer to the required EoVtheNVsCfg object. 
 **/
extern const EOtheNVsCfgExample * eo_nvscfg_example_GetHandle(void);




/** @}            
    end of group eo_thenvsconfigexample  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




