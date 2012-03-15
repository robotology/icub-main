
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOCFG_GPIO_MCBSTM32C_H_
#define _EOCFG_GPIO_MCBSTM32C_H_




/** @file       eOcfg_GPIO_MCBSTM32c.h
	@brief      This header file implements public interface to the configuration of the GPIOs for the MCBSTM32C board
	@author     marco.accame@iit.it
	@date       09/01/2011
**/

/** @defgroup eo_thekeilgpioconfig Object eOcfgGPIO_MCBSTM32c
    The eOcfg_GPIO_MCBSTM32C module encapsulates the GPIO configuration for the Keil's MCBSTM32c board. 
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOtheGPIO.h"



// - public #define  --------------------------------------------------------------------------------------------------

  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/**	@typedef    typedef enum iopinID_Inp_mcbstm32c_t
    @brief      iopinID_Inp_mcbstm32c_t contains the user-defined ID labels for the simple input pins of the
                Keil MCBSTM32C board and used as argument for #eo_iopinoutman_GetHandle().
 **/ 
typedef enum  
{   // first used value must be = 0 and last position must be dummy and = 255   
//  mcbstm32cInpBUTTON00 = 0,            // i dont use it
    iopinID_Inp_mcbstm32c_NONE = EOK_uint08dummy           // dummy value. keep it as last
} iopinID_Inp_mcbstm32c_t;


/** @typedef    typedef enum iopinID_Out_mcbstm32c_t
    @brief      iopinID_Out_mcbstm32c_t contains the user-defined ID labels for the simple output pins of the
                Keil MCBSTM32 board and used as argument for #eo_iopinoutman_GetHandle().
 **/ 
typedef enum  
{   // first used value must be = 0 and last position must be dummy and = 255
    iopinID_Out_mcbstm32c_LED_009 = 0,
    iopinID_Out_mcbstm32c_LED_010,
    iopinID_mcbstm32c_OutNONE = EOK_uint08dummy           // dummy value. keep it as last
} iopinID_Out_mcbstm32c_t;


/** @typedef    typedef enum iopinID_Iman_mcbstm32c_t
    @brief      iopinID_Iman_mcbstm32c_t contains the user-defined ID labels for the managed input pins of the
                Keil MCBSTM32C board and used as argument for #eo_iopinoutman_GetHandle().
 **/
typedef enum  
{   // first used value must be = 0 and last position must be dummy and = 255
    iopinID_Iman_mcbstm32c_BUTTON_USER = 0,
    iopinID_Iman_mcbstm32c_BUTTON_TAMP,
    iopinID_Iman_mcbstm32c_BUTTON_WKUP,
    iopinID_Iman_mcbstm32c_NONE = EOK_uint08dummy        // dummy value. keep it as last
} iopinID_Iman_mcbstm32c_t;


/** @typedef    typedef enum iopinID_Oman_mcbstm32c_t
    @brief      iopinID_Oman_mcbstm32c_t contains the user-defined ID labels for the managed output pins of the
                Keil MCBSTM32C board and used as argument for #eo_iopinoutman_GetHandle()
 **/            
typedef enum  
{   // first used value must be = 0 and last position must be dummy and = 255
    iopinID_Oman_mcbstm32c_LED_011 = 0,
    iopinID_Oman_mcbstm32c_LED_012,
    iopinID_Oman_mcbstm32c_LED_013,
    iopinID_Oman_mcbstm32c_LED_014,
    iopinID_Oman_mcbstm32c_LED_015,
    iopinID_Oman_mcbstm32c_NONE = EOK_uint08dummy        // dummy value. keep it as last
} iopinID_Oman_mcbstm32c_t;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

 
/** @fn         extern const eOgpio_cfg_t * eo_cfg_gpio_mcbstm32c_Get(void) 
    @brief      Gets the pointer to the eOgpio_cfg_t for the board.
    @return     Pointer to the required eOgpio_cfg_t data. 
 **/
extern const eOgpio_cfg_t * eo_cfg_gpio_mcbstm32c_Get(void);




/** @}            
    end of group eo_thekeilgpioconfig  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




