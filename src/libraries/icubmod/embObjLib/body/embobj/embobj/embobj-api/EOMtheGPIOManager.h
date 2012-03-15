
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMTHEGPIOMANAGER_H_
#define _EOMTHEGPIOMANAGER_H_


/** @file       EOMtheGPIOManager.h
    @brief      This header file implements public interface to the GPIO manager singleton for the MEE
    @author     marco.accame@iit.it
    @date       08/25/2011
**/

/** @defgroup eom_thegpiomanager Singleton EOMtheGPIOManager
    The EOMtheGPIOManager is a type derived from #EOtheGPIOManager and allows management of GPIOs
    in a multi-tasking execution environment MEE.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOtheGPIO.h"

// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 
 

/** @typedef    typedef struct EOMtheGPIOManager_hid EOMtheGPIOManager
    @brief      EOMtheGPIOManager is an opaque struct. It is used to implement data abstraction for the GPIO manager 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOMtheGPIOManager_hid EOMtheGPIOManager;



/**	@typedef    typedef struct eOmgpioman_cfg_t 
 	@brief      Contains the configuration for the eOmgpioman_cfg_t. 
 **/ 
typedef struct
{
    uint8_t             priority;           /**< The priority of the worker task */
    uint16_t            stacksize;          /**< The stack size of the worker task */
    eOreltime_t         period;             /**< The execution period of the worker task in microsec */
} eOmgpioman_cfg_t;

    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const eOmgpioman_cfg_t eom_gpioman_DefaultCfg; // = {200, 512, 10000};


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
 
/** @fn         extern EOMtheGPIOManager * eom_gpioman_Initialise(EOtheGPIO *thegpio, const eOmgpioman_cfg_t *gpiomancfg)
    @brief      Initialises the singleton gpio manager for the MEE.
    @arg        thegpio         The EOtheGPIO singleton which we want to manage.
    @arg        gpiomancfg      The configuration. If NULL, then it is used eom_gpioman_DefaultCfg.
    @return     The EOMtheGPIOManager singleton. If any problem, the #EOtheErrorManager issues a fatal error.
 **/
extern EOMtheGPIOManager * eom_gpioman_Initialise(EOtheGPIO *thegpio, const eOmgpioman_cfg_t *gpiomancfg); 


/** @fn         extern EOMtheGPIOManager * eom_gpioman_GetHandle(void)
    @brief      Gets the initialised singleton.
    @return     The EOMtheGPIOManager singleton or NULL
 **/
extern EOMtheGPIOManager * eom_gpioman_GetHandle(void); 
 

/** @}            
    end of group eom_thegpiomanager  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



