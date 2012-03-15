
// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _HAL_ENCODER_H_
#define _HAL_ENCODER_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       hal_encoder.h
    @brief      This header file implements public interface to the hal encoder module.
    @author     marco.accame@iit.it
    @date       09/09/2011
**/

/** @defgroup arm_hal_encoder HAL ENCODER

    The HAL ENCODER ....
 
    @todo acemor-facenda: review documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "hal_base.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 

/** @typedef    typedef enum hal_encoder_t 
    @brief      hal_encoder_t contains every possible encoder peripheral.
 **/ 
typedef enum  
{ 
    hal_encoder1 = 0,
	hal_encoder2 = 1,
    hal_encoder3 = 2,         
    hal_encoder4 = 3,
	hal_encoder5 = 4,
    hal_encoder6 = 5,         
    hal_encoder7 = 6,
	hal_encoder8 = 7,
    hal_encoder9 = 8
} hal_encoder_t;

enum { hal_encoders_num = 9 };


/** @typedef    typedef struct hal_encoder_cfg_t;
    @brief      contains configuration data of encoder peripheral.
 **/
typedef struct
{
    hal_interrupt_priority_t priority;          /**< the priority if the ISR underlying the encoder */
    void (*callback_on_rx)(void *arg);          /**< callback called when new data for the encoder is available     */
    void* arg;                                  /**< argument of the callback                                       */
} hal_encoder_cfg_t;

 
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const hal_encoder_cfg_t hal_encoder_cfg_default;   // = { .priority = hal_int_priority15, .callback_on_rx = NULL, .arg = NULL };


// - declaration of extern public functions ---------------------------------------------------------------------------

/** @fn			extern hal_result_t hal_encoder_init(hal_encoder_t port, const hal_encoder_cfg_t *cfg)
    @brief  	This function initializes spix interface (x=1 or x=2 or x=3).
    @param  	encoder 	    the encoder
    @param  	cfg 	        The configuration of teh encoder. 
    @return 	hal_res_NOK_generic in case the encoder isn't configured, else hal_res_OK
  */
extern hal_result_t hal_encoder_init(hal_encoder_t encoder, const hal_encoder_cfg_t *cfg);


/** @fn			extern hal_result_t hal_encoder_read_start(hal_encoder_t encoder)
    @brief  	This function reads datas from encoder in no-blocked way.
                In oreder to get those datas, it is necessary to invoke hal_encoder_get_value.
    @param  	encoder 	    the encoder
    @return 	hal_res_NOK_generic on error else hal_res_OK
  */
extern hal_result_t hal_encoder_read_start(hal_encoder_t encoder);


/** @fn			extern uint32_t hal_encoder_read_block(hal_encoder_t encoder, uint32_t *value);
    @brief  	This function reads datas from encoder in blocked way and returns read bytes.
    @param  	encoder 	    the encoder
    @param  	value 	        keeps the value.
    @return 	hal_res_NOK_generic on error else hal_res_OK
  */
extern hal_result_t hal_encoder_read_block(hal_encoder_t encoder, uint32_t *value);


/** @fn			extern hal_result_t hal_encoder_get_value(hal_encoder_t encoder, uint32_t *value)
    @brief  	This function reads returns encoder read datas.
    @param  	encoder 	    the encoder
    @param  	value 	        keeps the value.
    @return 	hal_res_NOK_generic on error else hal_res_OK
  */
extern hal_result_t hal_encoder_get_value(hal_encoder_t encoder, uint32_t *value);



/** @}            
    end of group arm_hal_encoder  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



