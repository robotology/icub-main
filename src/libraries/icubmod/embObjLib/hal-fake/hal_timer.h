
// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _HAL_TIMER_H_
#define _HAL_TIMER_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       hal_timer.h
    @brief      This header file implements public interface to the hal timer module.
    @author     marco.accame@iit.it
    @date       09/12/2011
**/

/** @defgroup arm_hal_timer HAL TIMER

    The HAL TIMER ...
 
    @todo acemor-facenda: review documentation.

    @warning    
    On ARM only the some functions are implemented. They are main ones: 
    hal_timer_init(), hal_timer_start(), hal_timer_stop(), hal_timer_remainingtime_get(), and hal_timer_status_get().
    Plus some others: 
    hal_timer_countdown_set(), 


    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "hal_base.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/** @typedef    typedef enum hal_timer_t 
    @brief      hal_timer_t contains every possible timer peripheral.
 **/ 
typedef enum  
{ 
    hal_timer1 = 0,
	hal_timer2 = 1,
    hal_timer3 = 2,
    hal_timer4 = 3,
	hal_timer5 = 4,
    hal_timer6 = 5,
    hal_timer7 = 6,
    hal_timer8 = 7
} hal_timer_t;

enum { hal_timers_num = 8 };


/** @typedef    typedef enum hal_timer_status_t 
    @brief      hal_timer_status_t contains the states of the timer peripheral.
 **/ 
typedef enum  
{ 
    hal_timer_status_none       = 0,        /**< when the timer is yet to be initted */
    hal_timer_status_idle       = 1,        /**< when the timer is initted but not running yet */         
    hal_timer_status_running    = 2,        /**< when the timer is running in one-shot mode or forever mode */
    hal_timer_status_expired    = 3         /**< when the timer is running in one-shot mode and its countdown has expired */
} hal_timer_status_t;


/** @typedef    typedef enum hal_timer_mode_t 
    @brief      hal_timer_mode_t contains the modes of the timer peripheral.
 **/ 
typedef enum  
{ 
    hal_timer_mode_oneshot  = 0,         
    hal_timer_mode_periodic = 1
} hal_timer_mode_t;


/** @typedef    typedef enum hal_timer_prescaler_t;
    @brief      represents the possible prescaler values
 **/
typedef enum
{
    hal_timer_prescalerAUTO = hal_NA08,     /**< Prescaler is internally selected to best match the countdown */
    hal_timer_prescaler1    = 1,            /**< Prescaler 1:1 */
    hal_timer_prescaler2    = 2,            /**< Prescaler 1:2 */
    hal_timer_prescaler4    = 3,            /**< Prescaler 1:4 */
    hal_timer_prescaler8    = 4,            /**< Prescaler 1:8 */
    hal_timer_prescaler16   = 5,            /**< Prescaler 1:16 */
    hal_timer_prescaler32   = 6,            /**< Prescaler 1:32 */
    hal_timer_prescaler64   = 7,            /**< Prescaler 1:64 */
    hal_timer_prescaler128  = 8,            /**< Prescaler 1:128 */
    hal_timer_prescaler256  = 9             /**< Prescaler 1:256 */
} hal_timer_prescaler_t; 


/** @typedef    typedef struct hal_timer_cfg_t;
    @brief      contains configuration data of timer peripheral.
 **/
typedef struct
{
    hal_timer_prescaler_t           prescaler;          /**< the prescaler one wants to use */
    hal_time_t                      countdown;          /**< the countdown before the ISR executes the callback */
    hal_interrupt_priority_t        priority;           /**< if hal_int_priorityNONE then there is no ISR and mode cannot be hal_timer_mode_oneshot */
    hal_timer_mode_t                mode;               /**< the mode of the timer: hal_timer_mode_oneshot or hal_timer_mode_periodic */
    void (*callback_on_exp)(void *arg);                 /**< callback called by the ISR executed at the expiry of the timer    */
    void* arg;                                          /**< argument of the callback                               */    
} hal_timer_cfg_t;


// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

/** @fn			extern hal_result_t hal_timer_init(hal_timer_t timer, const hal_timer_cfg_t *cfg, hal_time_t *error)
    @brief  	This function initializes a timer. It can be called more times. If called on a running timer, it
                stops it first. If priority field of @e cfg has value different from hal_int_priorityNONE,
                the function enable timer's ISR which is responsible of calling the associated callback function.
                In case the prescaler field is set to hal_timer_prescalerAUTO, the internal counter is set in order to
                satisfy the precisione required by the specified countdown. More accuracy means lower range. 
    @details    On ARM-STM32 architecture, the registers of the timers are 16 bits, and holds bot the counter and the prescaler.
                As a result, the precision and maximum range is chosen as follows: (prec, max) = (1000 us, 6400ms), 
                (100 us, 640ms), (10 us, 64ms), (1 us, 8ms). 
    @param      timer           The timer to initialise. 
    @param      cfg             The configuration. It cannot be NULL.
    @param      error           Gives back the error in microseconds by using a period and a prescaler given the
                                main frequency. This param can be NULL: in this case error is not calculated.
    @return 	hal_res_NOK_generic in case the timer isn't configured or not supported by board.
                hal_res_NOK_unsupported if the chosen prescaler is not supported
                hal_res_NOK_wrongparam if timer is configured in hal_timer_mode_oneshot and its interrupt priority
                has value hal_int_priorityNONE.
                hal_res_NOK_wrongparam if coundown is zero.
                hal_res_NOK_nullpointer if @e cfg is NULL
                hal_res_OK otherwise
  */
extern hal_result_t hal_timer_init(hal_timer_t timer, const hal_timer_cfg_t *cfg, hal_time_t *error);


/** @fn         extern hal_result_t timer_start(timer_t timer);
    @brief      starts timer @e timer
    @param      timer           The timer to start. It must be initted before.
    @return     hal_res_NOK_generic in case the timer wasn't configured, else hal_res_OK
 **/
extern hal_result_t hal_timer_start(hal_timer_t timer);


/** @fn         extern hal_result_t timer_stop(timer_t timer);
    @brief      stops timer @e timer
    @param      timer           The timer to start. It must be initted and started before.
    @return     hal_res_NOK_generic in case the timer wasn't configured or started, else hal_res_OK
 **/
extern hal_result_t hal_timer_stop(hal_timer_t timer);


/** @fn         extern hal_result_t hal_timer_remainingtime_get(hal_timer_t timer, hal_time_t *remaining_time)
    @brief      reads timer's actual value
    @param      timer
    @param      remaining_time      Gives back the timer's value expressed microseconds
    @return     hal_res_NOK_generic if timer is not initialized or configured in hal_cfg,
                hal_res_OK otherwise.
 **/
extern hal_result_t hal_timer_remainingtime_get(hal_timer_t timer, hal_time_t *remaining_time);


/** @fn         extern hal_timer_status_t hal_timer_status_get(hal_timer_t timer, hal_timer_status_t *status)
    @brief      gets the status of the timer
    @param      timer
    @return     timer's status
 **/
extern hal_timer_status_t hal_timer_status_get(hal_timer_t timer);


/** @fn         extern hal_result_t hal_timer_countdown_set(hal_timer_t timer, hal_time_t countdown, hal_time_t *error);
    @brief      sets timer countdown. A running timer is stopped before setting the value and restarted afterwards. 
    @param      timer
    @param      countdown       Timer's countdown expressed in microseconds.
    @param      error           The error in microseconds. This param can be NULL: in this case error is not calculated.
    @return     hal_res_NOK_generic / hal_res_OK
    @warning    This function is a wrapper of other functions. It does some controls and then calls hal_timer_init() with 
                proper parameters. This call is contained within hal_timer_stop() and hal_timer_start() if the timer was running
 **/
extern hal_result_t hal_timer_countdown_set(hal_timer_t timer, hal_time_t countdown, hal_time_t *error);


/** @fn         extern hal_result_t hal_timer_priority_set(hal_timer_t timer, hal_interrupt_priority_t prio);
    @brief      configures priority interrupt of timer @e timer. It stops and restart a running timer.
    @param      timer       timer to configure
    @param      prio        value of timer's interrupt priority
    @return     hal_res_NOK_wrongparam if timer is in hal_timer_mode_oneshot mode and @e prio has value hal_int_priorityNONE
                hal_res_NOK_generic if timer is not initialized.
                hal_res_ok otherwise
 **/
extern hal_result_t hal_timer_priority_set(hal_timer_t timer, hal_interrupt_priority_t prio);


/** @fn         extern hal_result_t hal_timer_interrupt_enable(hal_timer_t timer);
    @brief      enables timer interrupt
    @param      timer
    @return     hal_res_NOK_generic if timer is not initialized.
                hal_res_ok otherwise
 **/
extern hal_result_t hal_timer_interrupt_enable(hal_timer_t timer);


/** @fn         extern hal_result_t hal_timer_interrupt_disable(hal_timer_t timer);
    @brief      disables timer interrupt
    @param      timer
    @return     hal_res_NOK_generic if timer is not initialized.
                hal_res_ok otherwise
 
 **/
extern hal_result_t hal_timer_interrupt_disable(hal_timer_t timer);


/** @fn         extern hal_result_t hal_timer_offset_write(hal_timer_t timer, hal_nanotime_t offset)
    @brief      writes timer's offset. When the timer starts, it counts from offset value. 
    @param      timer
    @param      offset              number of nanosecconds to start from.
    @return     hal_res_NOK_generic if timer is not initialized
                hal_res_NOK_wrongparam if offset is bigger than countdown configured with init function.
                hal_res_OK otherwise
 **/
extern hal_result_t hal_timer_offset_write(hal_timer_t timer, hal_nanotime_t offset);


/** @}            
    end of group arm_hal_timer  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



