
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOCFG_NVS_NODE00_H_
#define _EOCFG_NVS_NODE00_H_




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

//-- ip address of the device
#define EOK_cfg_nvs_node00_ipaddr                                               EO_COMMON_IPV4ADDR(10, 1, 1, 0)

#include "eOcfg_NVs_node00_loc.h" // to see the nvIDs
#warning -> put the nvIDs of the node00: local (no remote)


// - declaration of public user-defined types ------------------------------------------------------------------------- 

    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

 
/** @fn         extern eOnvs_cfg_t * eo_cfg_nvs_node00_Get(void) 
    @brief      Gets configuration for the NVS of node00.
    @return     Pointer to the required configuration 
 **/
extern eOnvs_cfg_t * eo_cfg_nvs_node00_Get(void);



/** @}            
    end of group eo_thenvsconfignode00  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




