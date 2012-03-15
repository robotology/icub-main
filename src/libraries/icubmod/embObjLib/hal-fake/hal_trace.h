
// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _HAL_TRACE_H_
#define _HAL_TRACE_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       hal_tarce.h
    @brief      This header file implements public interface to the hal trace module.
    @author     marco.accame@iit.it
    @date       09/09/2011
**/

/** @defgroup arm_hal_trace HAL TRACE

    The HAL TRACE ...
 
    @todo acemor-facenda: review documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "hal_base.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 
// empty-section

 
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section





// - declaration of extern public functions ---------------------------------------------------------------------------


/** @fn         extern void hal_trace_init(void)
    @brief      Initialises the trace peripheral for STDIO operations.
    @warning    The function may require a proper .ini file for the toolchain.
 **/
extern void hal_trace_init(void);


/** @fn         extern int hal_trace_getchar(void)
    @brief      Retrieves a character from the trace devices until a character is
                available. The function can be used to redefine the read from the STDIN.
    @return     The character in input on the trace
    @warning    The function is blocking
 **/
extern int hal_trace_getchar(void);


/** @fn         extern int hal_trace_putchar(int ch)
    @brief      Sends a character to the trace peripheral.
                The function can be used to redefine the write onto the STDOUT.
    @return     The character sent on the trace peripheral or -1 in case of error
 **/
extern int hal_trace_putchar(int ch);


/** @fn         extern int hal_trace_puts(const char * str)
    @brief      Sends a '\0' terminated string to the trace peripheral and adds a '\n'.
    @return     The number of characters sent on the trace peripheral or -1 in case of error
 **/
extern int hal_trace_puts(const char * str);
 
 
/** @fn         inline extern void hal_sys_itm_init(void)
    @brief      Legacy version of hal_trace_init().
 **/
inline extern void hal_sys_itm_init(void)
{
    hal_trace_init();
}


/** @fn         inline extern int hal_sys_itm_getchar(void)
    @brief      Legacy version of hal_trace_getchar().
 **/
inline extern int hal_sys_itm_getchar(void)
{
    return(hal_trace_getchar());
}


/** @fn         inline extern int hal_sys_itm_putchar(int ch)
    @brief      Legacy version of hal_trace_putchar().
 **/
inline extern int hal_sys_itm_putchar(int ch)
{
    return(hal_trace_putchar(ch));
}
 
 
 /** @}            
    end of group arm_hal_trace  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



