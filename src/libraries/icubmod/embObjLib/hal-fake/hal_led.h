
// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _HAL_LED_H_
#define _HAL_LED_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       hal_led.h
    @brief      This header file implements public interface to the hal led module.
    @author     marco.accame@iit.it
    @date       09/16/2011
**/

/** @defgroup arm_hal_led HAL LED

    The HAL LED ...
 
    @todo acemor-facenda: review documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "hal_base.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/** @typedef    typedef enum hal_led_t 
    @brief      hal_led_t contains every possible led peripheral.
 **/ 
typedef enum  
{ 
    hal_led0 = 0,         
    hal_led1 = 1,
	hal_led2 = 2,
    hal_led3 = 3,         
    hal_led4 = 4,
	hal_led5 = 5,
    hal_led6 = 6,         
    hal_led7 = 7
} hal_led_t;

enum { hal_leds_num = 8 };



/** @typedef    typedef struct hal_led_cfg_t;
    @brief      contains configuration data of led peripheral.
 **/
typedef struct
{
    uint8_t         dummy;              /**< dummy   */         
} hal_led_cfg_t;

 
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

/** @fn			extern hal_result_t hal_led_init(hal_led_t led, const hal_led_cfg_t *cfg);
    @brief  	This function inits a led.
    @details    c rfce.
    @param      led             The led to initialise. 
    @param      cfg             The configuration. It can be NULL.
    @return 	hal_res_NOK_unsupported in case the led is not supported, else hal_res_OK
  */
extern hal_result_t hal_led_init(hal_led_t led, const hal_led_cfg_t *cfg);


/**
    @fn         extern hal_result_t hal_led_on(hal_led_t led)
    @brief      sets the led on
    @param      led             The led. It must be initted before.
    @return     hal_res_NOK_generic in case the led wasn't initted, else hal_res_OK
 **/
extern hal_result_t hal_led_on(hal_led_t led);


/**
    @fn         extern hal_result_t hal_led_on(hal_led_t led)
    @brief      sets the led off
    @param      led             The led. It must be initted before.
    @return     hal_res_NOK_generic in case the led wasn't initted, else hal_res_OK
 **/
extern hal_result_t hal_led_off(hal_led_t led);


/**
    @fn         extern hal_result_t hal_led_on(hal_led_t led)
    @brief      toggles the led
    @param      led             The led. It must be initted before.
    @return     hal_res_NOK_generic in case the led wasn't initted, else hal_res_OK
 **/
extern hal_result_t hal_led_toggle(hal_led_t led);



/** @}            
    end of group arm_hal_led  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



