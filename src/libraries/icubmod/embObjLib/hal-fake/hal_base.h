
// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _HAL_BASE_H_
#define _HAL_BASE_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       hal_base.h
    @brief      This header file implements public interface to the common data structures and functions of hal
    @author     marco.accame@iit.it
    @date       09/09/2011
**/

/** @defgroup arm_hal_base HAL base

    The HAL ...
 
    @todo acemor-facenda: review documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include <stdint.h>
#include "hal_arch_cfg.h"


// - public #define  --------------------------------------------------------------------------------------------------

#define hal_NA08    ((uint8_t)(0xFF))
#define hal_NA16    ((uint16_t)(0xFFFF))
#define hal_NA32    ((uint32_t)(0xFFFFFFFF)) 
#define hal_NA64    ((uint64_t)(0xFFFFFFFFFFFFFFFF))

#define hal_BASE_U8toU32(a)                       ((uint32_t)(a)&0xff)
#define hal_BASE_U16toU32(a)                      ((uint32_t)(a)&0xffff)
#define hal_BASE_U8toU64(a)                       ((uint64_t)(a)&0xff)
#define hal_BASE_4BtoU32(a, b, c, d)              ((hal_BASE_U8toU32(a))|(hal_BASE_U8toU32(b)<<8)|(hal_BASE_U8toU32(c)<<16)|(hal_BASE_U8toU32(d)<<24)) 
#define hal_BASE_8BtoU64(a, b, c, d, e, f, g, h)  (((uint64_t)hal_BASE_4BtoU32(a, b, c, d))|((uint64_t)hal_BASE_4BtoU32(e, f, g, h)<<32)) 


// - declaration of public user-defined types ------------------------------------------------------------------------- 

/** @typedef    typedef enum hal_cpufamily_t 
    @brief      hal_cpufamily_t contains all possible families of CPUs supported by HAL
 **/ 
typedef enum
{
    hal_cpufam_armcm3           = 0,
    hal_cpufam_armcm4           = 1,
    hal_cpufam_dspic            = 2
} hal_cpufamily_t;


/** @typedef    typedef enum hal_cputype_t 
    @brief      hal_cputype_t contains a type of CPU within a given family
 **/ 
typedef enum
{
    hal_cputyp_stm32f1x         = 0,
    hal_cputyp_stm32f2x         = 1,
    hal_cputyp_stm32f4x         = 2,
    hal_cputyp_stellaris        = 3,
    hal_cputyp_dspic33          = 4
} hal_cputype_t;


/** @typedef    typedef enum hal_result_t 
    @brief      hal_result_t contains all possible return values in functions.
 **/  
typedef enum 
{
    hal_res_OK                 = 0,
    hal_res_NOK_generic        = -1,
    hal_res_NOK_nullpointer   = -2,       /**< error: a null pointer was used */
//    hal_res_NOK_timeout       = -3,       /**< error: the caller (task) did not get the osal resource after it waited (even for zero time) */
//    hal_res_NOK_nowait        = -4,       /**< error: the calling isr did not get the osal resource when called the not waiting version of the function */ 
    hal_res_NOK_nodata        = -5,       /**< error: there was not data to be retrieved */
    hal_res_NOK_busy          = -6,       /**< error: the service was busy and request has not been served */
    hal_res_NOK_unsupported   = -15,      /**< error: unsupported feature */
    hal_res_NOK_wrongparam    = -16,      /**< error: one or more params are incorrect */
    hal_res_NOK_hwerror       = -17       /**< error: hardware error */
} hal_result_t;


/** @typedef    typedef enum hal_fatalerror_t 
    @brief      hal_fatalerror_t contains all possible fatal error codes.
 **/ 
typedef enum
{
    hal_fatalerror_generic                   = 0xE0,             
    hal_fatalerror_missingconfiguration      = 0xE1,       
    hal_fatalerror_missingmemory             = 0xE2,       
    hal_fatalerror_incorrectparameter        = 0xE3,       
    hal_fatalerror_unsupportedbehaviour      = 0xE4,
    hal_fatalerror_runtimefault              = 0xE5
} hal_fatalerror_t;


/** @typedef    typedef enum hal_boolval_t 
    @brief      hal_boolval_t contains values for bool type (hal_bool_t).
    @warning    C99 contains bool, true, and false. To use C99 include "stdbool.h" and -c99 compiler option.
                At this point please redefine eo_false and eo_true to be equal to false and true.
                Also, eObool_t must be typedef-ed as bool.

 **/ 
typedef enum
{
    hal_false = 0,      /**< false */
    hal_true  = 1       /**< true */
} hal_boolval_t;


/** @typedef    typedef enum hal_bool_t 
    @brief      hal_bool_t is the bool type.If C99 is available, use bool instead of uint8_t.
 **/ 
typedef uint8_t     hal_bool_t;


/** @typedef    typedef uint32_t hal_reltime_t 
    @brief      hal_reltime_t express time in hal in form of usec (micro-seconds). It is used for relative timing
                operations because its maximum value is about 4294 seconds, which is more than enough for most needs.
 **/  
typedef uint32_t    hal_reltime_t;


/** @typedef    typedef hal_reltime_t  hal_time_t
    @brief      hal_time_t is an alias for hal_reltime_t
 **/ 
typedef hal_reltime_t  hal_time_t;


/** @typedef    typedef uint64_t hal_abstime_t 
    @brief      hal_abstime_t express absolute time in hal in form of usec (micro-seconds) on a very long range.
                A hal_absttime_t variable keeps time up to 584 thousand years.
 **/  
typedef uint64_t    hal_abstime_t;


/** @typedef    typedef uint64_t hal_nanotime_t 
    @brief      hal_nanotime_t express very precise relative time in hal in form of nsec (nano-seconds).
                A hal_nanoime_t variable keeps time up to 584 years.
 **/  
typedef uint64_t    hal_nanotime_t;


/** @typedef    typedef void (*hal_void_fp_void_t) (void)
    @brief      hal_void_fp_void_t is a pointer to a function which returns void and has a void argument.
 **/
typedef void (*hal_void_fp_void_t) (void);


/** @typedef    typedef void (*hal_callback_t) (void* p)
    @brief      hal_callback_t is a pointer to a callback function which returns void and has a void pointer argument.
 **/
typedef void (*hal_callback_t) (void* p);


/** @typedef    typedef enum hal_interrupt_priority_t;
    @brief      rappresents the possible values for interrupt priorities.
                On ARM:     0 is highest priority. 
                On DSPIC:   0 is lowest priority
 **/
typedef enum
{
    hal_int_priorityNONE    = hal_NA08,
    hal_int_priority00      = 0,
    hal_int_priority01      = 1,
    hal_int_priority02      = 2,
    hal_int_priority03      = 3,
    hal_int_priority04      = 4,
    hal_int_priority05      = 5,
    hal_int_priority06      = 6,
    hal_int_priority07      = 7,
    hal_int_priority08      = 8,
    hal_int_priority09      = 9,
    hal_int_priority10      = 10,
    hal_int_priority11      = 11,
    hal_int_priority12      = 12,
    hal_int_priority13      = 13,
    hal_int_priority14      = 14,
    hal_int_priority15      = 15
} hal_interrupt_priority_t;  


/** @typedef    typedef struct hal_extfn_cfg_t
    @brief      hal_extfn_cfg_t keeps pointers to externally defined functions which HAL may need.
 **/
typedef struct
{
    /** The function which is executed when a fatal error is detected. If not redefined (thus this value is NULL), 
        it just enters in a forever loop. */ 
    void            (*usr_on_fatal_error)(hal_fatalerror_t errorcode, const char * errormsg);
    /** The function which is executed to suspend scheduling. Use the function osal_system_scheduling_suspend() of OSAL.
        If not redefined (thus this value is NULL) it does nothing.  It is necessary to redefine only if the HAL is used 
        in a multitasking execution environment (with an RTOS) */ 
    void            (*osal_system_scheduling_suspend)(void);
    /** The function which is executed to restart scheduling. Use the function osal_system_scheduling_restart() of OSAL.
        If not redefined (thus this value is NULL) it does nothing.  It is necessary to redefine only if the HAL is used 
        in a multitasking execution environment (with an RTOS) */ 
    void            (*osal_system_scheduling_restart)(void);
} hal_extfn_cfg_t;


/** @typedef    typedef struct hal_cfg_t 
    @brief      hal_cfg_t contains hal configuration.
 **/  
typedef struct
{   
    hal_cpufamily_t cpu_family;                 /**< The CPU family: an enum hal_cpufamily_t                */
    hal_cputype_t   cpu_type;                   /**< The CPU type: an enum hal_cputype_t                    */
    uint32_t        cpu_freq;                   /**< The CPU frequency in Hz                                */
    uint32_t        sys_stacksize;              /**< The size of stack available to the system in bytes     */
    uint32_t        sys_heapsize;               /**< The size of heap available to the system in bytes      */
    hal_boolval_t   display_enable;             /**< Enable of the display: 1 if enabled, 0 not             */
    hal_boolval_t   eth_enable;                 /**< Enable of ethernet: 1 enabled, 0 not                   */
    uint8_t         eth_dmatxbuffer_num;        /**< Number of eth frames in the tx buffer (>0)             */
    uint8_t         eth_dmarxbuffer_num;        /**< Number of eth frames in the rx buffer  (>0)            */
    hal_boolval_t   can1_enable;                /**< Enable of CAN1 peripheral: 1 enabled, 0 not            */
    uint8_t         can1_rxqnorm_num;           /**< Number of CAN frames in the rx queue of CAN1  (>=0)    */
    uint8_t         can1_txqnorm_num;           /**< Number of CAN frames in the tx queue of CAN1  (>=0)    */
    uint8_t         can1_txqhigh_num;           /**< Number of CAN frames in the high priority tx queue of CAN1  (>=0)   */
    hal_boolval_t   can2_enable;                /**< Enable of CAN2 peripheral: 1 enabled, 0 not            */
    uint8_t         can2_rxqnorm_num;           /**< Number of CAN frames in the rx queue of CAN2  (>=0)    */
    uint8_t         can2_txqnorm_num;           /**< Number of CAN frames in the tx queue of CAN2  (>=0)    */
    uint8_t         can2_txqhigh_num;           /**< Number of CAN frames in the high priority tx queue of CAN2  (>=0)   */
	hal_boolval_t   spi1_enable;                /**< Enable of SPI1 peripheral: 1 enabled, 0 not            */
	hal_boolval_t   spi2_enable;                /**< Enable of SPI2 peripheral: 1 enabled, 0 not            */
	hal_boolval_t   spi3_enable;                /**< Enable of SPI3 peripheral: 1 enabled, 0 not            */
    uint8_t         timers_num;                 /**< Number of timers to use                                */
    hal_arch_cfg_t  arch;                       /**< Configuration of behaviour which are architecture dependant */
    hal_extfn_cfg_t extfn;                      /**< External functionalities offered to the HAL            */
} hal_cfg_t;

 
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

/** @fn         extern uint32_t hal_base_memory_getsize(const hal_cfg_t *cfg, uint32_t *size04aligned)
    @brief      Gets the size of the 4-aligned memory required by the hal in order to work according to a given
                configuration. 
    @param      cfg             The target configuration. 
    @param      size04aligned   The number of bytes of teh 4-aligned RAM memory which is required. (if not NULL)
    @return     The number of bytes of the 4bytes-aligned RAM which is required

 **/
extern uint32_t hal_base_memory_getsize(const hal_cfg_t *cfg, uint32_t *size04aligned);


/** @fn         extern hal_result_t hal_base_initialise(const hal_cfg_t *cfg, uint32_t *data04aligned)
    @brief      Initialise the hal to work for a given configuration and with a given external memory.
    @param      cfg             The target configuration. 
    @param      data04aligned   The 4bytes-aligned RAM which is required, or NULL if none is required.
    @return     hal_res_OK or hal_res_NOK_generic 
 **/
extern hal_result_t hal_base_initialise(const hal_cfg_t *cfg, uint32_t *data04aligned); 


/** @fn         extern inline uint32_t hal_memory_getsize(const hal_cfg_t *cfg, uint32_t *data04aligned)
    @brief      Legacy version of hal_base_memory_getsize().
 **/
//extern inline uint32_t hal_memory_getsize(const hal_cfg_t *cfg, uint32_t *size04aligned)
//{
//    return(hal_base_memory_getsize(cfg, size04aligned));
//}


/** @fn         extern inline uint32_t hal_initialise(const hal_cfg_t *cfg, uint32_t *data04aligned)
    @brief      Legacy version of hal_base_initialise().
 **/
//extern inline hal_result_t hal_initialise(const hal_cfg_t *cfg, uint32_t *data04aligned)
//{ 
//    return(hal_base_initialise(cfg, data04aligned)); 
//} 



/** @}            
    end of group arm_hal_base  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



