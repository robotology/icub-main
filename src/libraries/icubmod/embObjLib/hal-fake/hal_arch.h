
// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _HAL_ARCH_H_
#define _HAL_ARCH_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       hal_arch.h
    @brief      This header file keeps public interface to the hal parts which are specific of architecture.
    @author     valentina.gaggero@iit.it, marco.accame@iit.it
    @date       09/16/2011
**/

/** @defgroup hal_arch_generic HAL ARCH GENERIC

    cecece
 
    @todo acemor-facenda: review documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "hal_arch_arm.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 

typedef hal_arch_arm_cfg_t hal_arch_cfg_t;

 
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
// empty-section


/** @}            
    end of group hal_arch_generic  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

