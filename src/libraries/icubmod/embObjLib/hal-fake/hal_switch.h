           
// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _HAL_SWITCH_H_
#define _HAL_SWITCH_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       hal_switch.h
    @brief      This header file implements public interface to the hal switch module.
    @author     valentina.gaggero@iit.it
    @date       10/18/2011
**/

/** @defgroup arm_hal_switch HAL SWITCH

    The HAL SWITCH ....
 
    @todo acemor-facenda: review documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------
#include "hal_base.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 

/** @typedef    typedef struct hal_switch_cfg_t;
    @brief      contains configuration data of switch.
 **/
typedef struct
{
    uint8_t dummy;          /**< dummy...     */
} hal_switch_cfg_t;

 
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const hal_switch_cfg_t hal_switch_cfg_default;   // = { .dummy = 0};


// - declaration of extern public functions ---------------------------------------------------------------------------

/** @fn			extern hal_result_t hal_switch_init(const hal_switch_cfg_t *cfg)
    @brief  	This function initializes the switch attached to the MPU
    @param  	cfg 	        The configuration of the switch. It can be NULL.
    @return 	hal_res_NOK_generic in case the switch cannot be configured, else hal_res_OK
    @warning    The initialisation may temporarily stop the switch, thus multiple calls to this function
                should be avoided as they can corrupt network traffic.
  */
extern hal_result_t hal_switch_init(const hal_switch_cfg_t *cfg);


/** @fn			extern hal_bool_t hal_switch_initted_is(void)
    @brief  	This function tells if the switch is already initialsied.
    @return 	hal_true or hal_false.
  */
extern hal_bool_t hal_switch_initted_is(void);

/** @}            
    end of group arm_hal_switch  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



