
// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _HAL_ARCH_CFG_H_
#define _HAL_ARCH_CFG_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       hal_arch_cfg.h
    @brief      This header file keeps public interface to the cfg of hal parts which are specific of architecture.
    @author     valentina.gaggero@iit.it, marco.accame@iit.it
    @date       09/16/2011
**/

/** @defgroup hal_arch_cfg_generic HAL ARCH CFG GENERIC

    cecece
 
    @todo acemor-facenda: review documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "hal_arch_cfg_host.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 

typedef hal_arch_host_cfg_t hal_arch_cfg_t;

 
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
// empty-section


/** @}            
    end of group hal_arch_cfg_generic  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

