
// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _HAL_ARCH_CFG_HOST_H_
#define _HAL_ARCH_CFG_HOST_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       hal_arch_cfg_arm.h
    @brief      This header file keeps public interface to the cfg of hal parts which are specific of arm architecture.
    @author     marco.accame@iit.it
    @date       09/16/2011
**/

/** @defgroup hal_arch_cfg_arm HAL CFG ARCH ARM

    cecece
 
    @todo acemor-facenda: review documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "emBODYporting.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/** @typedef    typedef struct hal_arch_arm_cfg_t 
    @brief      hal_arch_arm_cfg_t contains hal configuration for the parts which are specifics only of ARM architecture.
 **/  
typedef struct
{   
    uint32_t        nothingsofar;           /**< nothing so far */
} hal_arch_host_cfg_t;

 
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
// empty-section


/** @}            
    end of group hal_arch_cfg_arm  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



