
// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _HAL_SPI4ENCODER_H_
#define _HAL_SPI4ENCODER_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       hal_spi4encoder.h
    @brief      This header file implements public interface to the hal module with spi functions for teh encoder
    @author     marco.accame@iit.it / valentina.gaggero@iit.it
    @date       09/09/2011
**/

/** @defgroup arm_hal_spi4encoder HAL SPI for ENCODER

    The HAL SPI 4 encoder ... is an incomplete module
 
    @todo acemor-facenda: review documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "hal_base.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 

/** @typedef    typedef enum hal_spi_port_t 
    @brief      hal_spi_port_t contains the values that a SPI port can have.
    @warning    On the MCBSTM32C board, the values are mapped into SPI1, SPI2, SPI3.
 **/
typedef enum  
{ 
    hal_spi_port1 = 0,        
	hal_spi_port2 = 1,       
    hal_spi_port3 = 2         
} hal_spi_port_t;

enum { hal_spi_ports_number = 3 }; 


/** @typedef    typedef enum hal_spi_mux_t 
    @brief      hal_spi_mux_t contains the IDs of all possible multiplex on an SPI port.
                This type is used only for internal implementation of encoders which can be
                multiplex on the same spi port up to a number of three.
 **/
typedef enum  
{ 
    hal_spi_mux1 = 0,         
    hal_spi_mux2 = 1,
	hal_spi_mux3 = 2
} hal_spi_mux_t; 

enum { hal_spi_muxes_number = 3 }; 


/** @typedef    typedef enum hal_spi4encoder_baudrate_t 
    @brief      hal_spi4encoder_baudrate_t contains the baudrates of the SPI port.
                This type is used only for internal implementation of encoders.
 **/
typedef enum  
{ 
    hal_spi_baudrate500kbps = 1,         
    hal_spi_baudrate1000kbps = 2
} hal_spi4encoder_baudrate_t; 


/** @typedef    typedef struct hal_spi4encoder_cfg_t;
    @brief      contains configuration data of can peripheral.
 **/
typedef struct
{
    hal_spi4encoder_baudrate_t baudrate;/**< spi baudrate */
    hal_interrupt_priority_t priority;
    void (*callback_on_rx)(void *arg);  /**< call back function called after a frame of 3 mux has been read */
    void *arg;                          /**< argument of the callback */
} hal_spi4encoder_cfg_t;

 
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

/** @fn			extern hal_result_t hal_spi4encoder_init(hal_spi_port_t port, hal_spi4encoder_cfg_t *cfg)
  * @brief  	This function initializes spix interface (x=1 or x=2 or x=3).
  * @param  	port 	    	the spi port
  * @param  	cfg 	        the config 
  * @return 	hal_res_NOK_generic in case spix interface isn't configured by configuration wizard, else hal_res_OK
  */
extern hal_result_t hal_spi4encoder_init(hal_spi_port_t port, hal_spi4encoder_cfg_t *cfg);




/** @}            
    end of group arm_hal_spi4encoder  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



