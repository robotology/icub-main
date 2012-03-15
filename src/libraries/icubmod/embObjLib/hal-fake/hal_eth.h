
// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _HAL_ETH_H_
#define _HAL_ETH_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       hal_eth.h
    @brief      This header file implements public interface to the hal eth module.
    @author     marco.accame@iit.it
    @date       09/09/2011
**/

/** @defgroup arm_hal_eth HAL ETH

    The HAL ETH ...
 
    @todo acemor-facenda: review documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "hal_base.h"



// - public #define  --------------------------------------------------------------------------------------------------

#define hal_ETH_macaddress(m1, m2, m3, m4, m5, m6)   ((uint64_t)hal_BASE_8BtoU64(m1, m2, m3, m4, m5, m6, 0, 0))
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 

/** @typedef    typedef uint64_t hal_eth_macaddress_t 
    @brief      hal_eth_macaddress_t is used to store a mac address in its 6 least significant bytes
                in such a way that ...
 **/ 
typedef uint64_t hal_eth_macaddress_t;


/** @typedef    typedef struct hal_eth_frame_t 
    @brief      hal_eth_frame_t is used to exchange ethernet frames to hal. The frame data must begin with
                member datafirstbyte. One method is to use a hal_eth_frame_t pointer and to cast it to
                a portion of memory with size length+4.
 **/ 
typedef struct
{
    uint16_t length;                /**< the length of the frame in bytes */
    uint16_t index;                 /**< index to the used byte */
    uint8_t  datafirstbyte[1];      /**< keeps first byte of the frame. The other length-1 bytes must be consecutive to this */
} hal_eth_frame_t; 


/** @typedef    typedef struct hal_eth_onframereception_t 
    @brief      hal_eth_onframereception_t is used to configure the behavior of the hal ethernet on reception of
                one frame. The eth ISR will ask memory for copying the new frame using the frame_new() method,
                will give the frame back to application with frame_movetohigherlevel(), and also will call
                a method frame_alerthigherlevel() to let somone known that the frame is available.
 **/
typedef struct
{
    hal_eth_frame_t * (*frame_new)(uint32_t size);
    void (*frame_movetohigherlayer)(hal_eth_frame_t *frame);
    void (*frame_alerthigherlayer)(void);
} hal_eth_onframereception_t;


/** @typedef    typedef struct hal_eth_cfg_t;
    @brief      contains configuration data of eth peripheral.
 **/
typedef struct
{
    hal_interrupt_priority_t        priority;
    hal_eth_onframereception_t*     onframerx;  /**< The user must customise the behavior in reception using these functions.
                                                     Only the alert function can be NULL. */
    hal_eth_macaddress_t            macaddress; /**< Keeps the mac address */
} hal_eth_cfg_t;


/** @typedef    typedef struct hal_eth_network_functions_t 
    @brief      hal_eth_network_functions_t is used to to group into a single variables all the functions which are
                needed to run a stack based on ethernet, such as a tcp/ip stack.
 **/
typedef struct
{
    hal_result_t (*eth_init)(const hal_eth_cfg_t *cfg);
    hal_result_t (*eth_enable)(void);
    hal_result_t (*eth_disable)(void);
    hal_result_t (*eth_sendframe)(hal_eth_frame_t *fr);
} hal_eth_network_functions_t;

 
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section



// - declaration of extern public functions ---------------------------------------------------------------------------

/** @fn         extern hal_result_t hal_eth_init(const hal_eth_cfg_t *cfg)
    @brief      Initialise an external interface. Memory must have been already allocated using 
                the hal_initialise() function. If @e onrx is NULL or if anything goes wrong, 
                the function calls hal_on_fatalerror() with proper error code.
    @param      cfg         Contains the eth configuration.
    @warning    It is generally used by a TCP-IP stack.
 **/
extern hal_result_t hal_eth_init(const hal_eth_cfg_t *cfg);


/** @fn         extern hal_result_t hal_eth_enable(void)
    @brief      Enable the eth peripheral. 
    @warning    It is generally used by a TCP-IP stack
 **/
extern hal_result_t hal_eth_enable(void);


/** @fn         extern hal_result_t hal_eth_disable(void)
    @brief      Disable the eth peripheral. 
    @warning    It is generally used by a TCP-IP stack
 **/
extern hal_result_t hal_eth_disable(void);


/** @fn         extern hal_result_t hal_eth_sendframe(hal_eth_frame_t *frame)
    @brief      Sends a frame over the eth peripheral. 
    @param      frame           The frame to send
    @warning    It is generally used by a TCP-IP stack
 **/
extern hal_result_t hal_eth_sendframe(hal_eth_frame_t *frame);


/** @fn         extern const hal_eth_network_functions_t * hal_eth_get_network_functions(void)
    @brief      retrieves the network functions used by HAL
    @return     a pointer containing {hal_eth_init, hal_eth_enable, hal_eth_disable, hal_eth_sendframe}
    @warning    It is generally used to initialise a TCP-IP stack
 **/
extern const hal_eth_network_functions_t * hal_eth_get_network_functions(void);



/** @}            
    end of group arm_hal_eth  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



