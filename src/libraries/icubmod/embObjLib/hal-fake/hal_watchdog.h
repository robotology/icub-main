
// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _HAL_WATCHDOG_H_
#define _HAL_WATCHDOG_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       hal_watchdog.h
    @brief      This header file implements public interface to the hal watchdog module.
    @author     marco.accame@iit.it
    @date       21/10/2011
**/

/** @defgroup arm_hal_watchdog HAL watchdog

    The HAL watchdog ...
 
    @todo acemor-facenda: review documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "hal_base.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/** @typedef    typedef enum hal_watchdog_t 
    @brief      hal_watchdog_t contains every possible watchdog peripheral.
 **/ 
typedef enum  
{ 
    hal_watchdog_normal = 0,    /**< if not refreshed within its countdown, then it forces a reset of the system */
    hal_watchdog_window = 1     /**< if not refreshed within its countdown, then it executes a callback function and if not refreshed
                                     it forces a reset of the system */
} hal_watchdog_t;

enum { hal_watchdogs_num = 2 };



/** @typedef    typedef struct hal_watchdog_cfg_t;
    @brief      contains configuration data of watchdog peripheral.
 **/
typedef struct
{
    hal_time_t                  countdown;                  /**< the countdown of the watchdog in microseconds      */
    hal_interrupt_priority_t    priority;                   /**< priority of the ISR for teh window watchdog */
    hal_callback_t              onwindowexpiry_cbk;         /**< callback called by the ISR when the window watchdog expires   */
    void*                       onwindowexpiry_arg;         /**< argument of the callback                           */    
} hal_watchdog_cfg_t;


// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const hal_watchdog_cfg_t hal_watchdog_cfg_default; //= { .countdown = 20000, .priority = hal_int_priorityNONE, .onwindowexpiry_cbk = NULL, .onwindowexpiry_arg = NULL}



// - declaration of extern public functions ---------------------------------------------------------------------------

/** @fn			extern hal_result_t hal_watchdog_init(hal_watchdog_t watchdog, const hal_watchdog_cfg_t *cfg)
    @brief  	This function initializes a watchdog. 
    @details    c rfce.
    @param      watchdog        The watchdog to initialise. 
    @param      cfg             The configuration. If NULL it uses its default.
                                The normal watchdog has a countdown range from 10 msec upto 10 seconds.
                                The window watchdog has a countdown range from 5 ms upto 50 ms. The callback function has 
                                up to 800 usec to be executed.
    @return 	hal_res_NOK_generic in case the watchdog cannot be configured, hal_res_NOK_unsupported if it is not
                supported, hal_res_OK if successful
  */
extern hal_result_t hal_watchdog_init(hal_watchdog_t watchdog, const hal_watchdog_cfg_t *cfg);


/**
    @fn         extern hal_result_t hal_watchdog_start(hal_watchdog_t watchdog)
    @brief      starts the watchdog @e watchdog
    @param      watchdog        The watchdog to start. It must be initted before.
    @return     hal_res_NOK_generic in case the watchdog wasn't configured, else hal_res_OK
 **/
extern hal_result_t hal_watchdog_start(hal_watchdog_t watchdog);


/**
    @fn         extern hal_result_t hal_watchdog_refresh(hal_watchdog_t watchdog)
    @brief      refreshes the watchdog @e watchdog
    @param      watchdog        The watchdog to start. It must be initted before.
    @return     hal_res_NOK_generic in case the watchdog wasn't configured, else hal_res_OK
 **/
extern hal_result_t hal_watchdog_refresh(hal_watchdog_t watchdog);



/** @}            
    end of group arm_hal_watchdog  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



