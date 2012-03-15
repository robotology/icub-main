
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOCFG_NVS_NODE00_H_
#define _EOCFG_NVS_NODE00_H_




/** @file       EOtheNVsCfgNode00.h
	@brief      This header file gives the configuration for the NVs of an example node00
	@author     marco.accame@iit.it
	@date       09/06/2011
**/

/** @defgroup eo_thenvsconfignode00 Configuation of the NVs for node00
    Tcecece 
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

#include "EOnetvar.h"



// - public #define  --------------------------------------------------------------------------------------------------

//-- ip address of the device
#define EOK_cfg_nvs_node00_ipaddr                                               EO_COMMON_IPV4ADDR(10, 1, 1, 0)

//-- the nvIDs owned locally
#include "eOcfg_NVs_node00_loc.h" 



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




