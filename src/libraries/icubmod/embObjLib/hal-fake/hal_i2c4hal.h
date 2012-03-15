
// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _HAL_I2C4HAL_H_
#define _HAL_I2C4HAL_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       hal_i2c4hal.h
    @brief      This header file implements interface to the hal i2c module used for eeprom
    @author     marco.accame@iit.it
    @date       09/09/2011
**/

/** @defgroup arm_hal_i2c_4eeprom HAL I2C4HAL

    The HAL I2C is so far just used for the EEPROM thus is incomplete and incoherent. SO ... DONT USE IT!
    In teh future this .h file may become the interface to a i2c module
 
    @todo acemor-facenda: review documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "hal_base.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/** @typedef    typedef enum hal_i2c_port_t 
    @brief      hal_i2c_port_t contains the possible I2C ports 
 **/
typedef enum
{
    hal_i2c_port1 = 0,
    hal_i2c_port2 = 1,
} hal_i2c_port_t;

enum { hal_i2c_ports_number = 2 };


/** @typedef    typedef enum hal_i2c_cfg_t 
    @brief      hal_i2c_cfg_t contains the configuation for i2c
 **/
typedef struct
{
    uint32_t                    dummy;   
} hal_i2c_cfg_t;
 
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const hal_i2c_cfg_t hal_i2c_cfg_default; // = { dummy = 0 };


// - declaration of extern public functions ---------------------------------------------------------------------------


/** @fn			extern hal_result_t hal_i2c_init(hal_i2c_port_t port, const hal_i2c_cfg_t *cfg)
  * @brief  	This function initializes i2c interface
  * @param  	i2cx	        I2C port
  * @param  	cfg 	        pointer to configuration data
  * @return 	hal_res_NOK_generic on error else hal_res_OK
  * @warning    this function is used just for ... the eeprom ... thus shoudl not be used by the user
  */
extern hal_result_t hal_i2c4hal_init(hal_i2c_port_t port, const hal_i2c_cfg_t *cfg);


/** @}            
    end of group arm_hal_i2c_4eeprom  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



