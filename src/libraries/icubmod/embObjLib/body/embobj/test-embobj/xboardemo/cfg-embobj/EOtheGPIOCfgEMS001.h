
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHEGPIOCFGEMS001_H_
#define _EOTHEGPIOCFGEMS001_H_




/** @file       EOtheGPIOCfgEMS001.h
	@brief      This header file implements public interface to the configuration of the GPIOs for the MCBSTM32 board
	@author     marco.accame@iit.it
	@date       10/15/2009
**/

/** @defgroup eo_thekeilgpioconfig Object EOtheGPIOCfgMCBSTM32x
    The EOtheGPIOCfgEMS001 is a singleton derived from the base object EOVtheGPIOCfg which configures the GPIOs for the
    keil EMS001 board. 
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"



// - public #define  --------------------------------------------------------------------------------------------------

  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/**	@typedef    typedef enum keilInpPinID_t
    @brief      keilInpPinID_t contains the user-defined ID labels for the simple input pins of the
                EMS001 board.
 **/ 
typedef enum  
{   // first used value must be = 0 and last position must be dummy and = 255   
//  mcbstm32xInpBUTTON00 = 0,            // i dont use it
    xbrdInpNONE = EOK_uint08dummy           // dummy value. keep it as last
} xbrdPinInpID_t;


/** @typedef    typedef enum keilOutPinID_t
    @brief      keilOutPinID_t contains the user-defined ID labels for the simple output pins of the
                Keil MCBSTM32 board.
 **/ 
typedef enum  
{   // first used value must be = 0 and last position must be dummy and = 255
//    xbrdOutLED_009 = 0,
//    xbrdOutLED_010,
    xbrdOutNONE = EOK_uint08dummy           // dummy value. keep it as last
} xbrdPinOutID_t;


/** @typedef    typedef enum keilMngInpPinID_t
    @brief      keilMngInpPinID_t contains the user-defined ID labels for the managed input pins of the
                Keil MCBSTM32 board.
 **/
typedef enum  
{   // first used value must be = 0 and last position must be dummy and = 255
//    xbrdItrigBUTTON_WKUP = 0,
//    xbrdItrigBUTTON_TAMP,
    xbrdItrigNONE = EOK_uint08dummy        // dummy value. keep it as last
} xbrdPinItrigID_t;


/** @typedef    typedef enum keilMngOutPinID_t
    @brief      keilMngOutPinID_t contains the user-defined ID labels for the managed output pins of the
                Keil MCBSTM32 board.
 **/            
typedef enum  
{   // first used value must be = 0 and last position must be dummy and = 255
    xbrdwaveLED_ONE = 0,
    xbrdwaveLED_TWO,
    xbrdwaveLED_THREE,
    xbrdwaveLED_FOUR,
    xbrdwaveNONE = EOK_uint08dummy        // dummy value. keep it as last
} xbrdPinOwaveID_t;


/**	@typedef    typedef struct EOtheGPIOCfgEMS001_hid EOtheGPIOCfgEMS001
 	@brief 		EOtheGPIOCfgEMS001 is an opaque struct. It is used to implement data abstraction for the gpio cfg
 **/  
typedef struct EOtheGPIOCfgEMS001_hid EOtheGPIOCfgEMS001;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

 
/** @fn         extern const EOtheGPIOCfgEMS001 * eo_gpiocfg_xbrd_GetHandle(void) 
    @brief      Gets the handle of the EoGPIOConfig derived object.
    @return     Pointer to the required EoGPIOConfig object. 
 **/
extern const EOtheGPIOCfgEMS001 * eo_gpiocfg_ems001_GetHandle(void);




/** @}            
    end of group eo_theems001gpioconfig  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




