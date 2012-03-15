

// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHEGPIO_HID_H_
#define _EOTHEGPIO_HID_H_


/* @file       EoTheGPIO_hid.h
    @brief      This header file implements hidden interface to the GPIO singleton.
    @author     marco.accame@iit.it
    @date       10/15/2009
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOioPinInput.h"
#include "EOioPinOutput.h"
#include "EOioPinInputManaged.h"   
#include "EOioPinOutputManaged.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOtheGPIO.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section



// - definition of the hidden struct implementing the object ----------------------------------------------------------

/* @struct     EOtheGPIO_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOtheGPIO_hid 
{
    const eOgpio_cfg_t      *cfg;
    EOioPinInput            *inp;           /*< array of input pins                                    **/
    EOioPinOutput           *out;           /*< array of output pins                                   **/
    EOioPinInputManaged     *mnginp;        /*< array of managed input pins                            **/
    EOioPinOutputManaged    *mngout;        /*< array of managed output pins                           **/
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------


/* @fn         extern uint8_t eo_gpio_hid_GetNumMngInp(EOtheGPIO *const p)
    @brief      Gets the number of managed input pins 
    @param      The handle to the object.
    @return     The numer of managed input pins.
 **/
extern uint8_t eo_gpio_hid_GetNumMngInp(EOtheGPIO *const p);

/* @fn         extern uint8_t eo_gpio_hid_GetNumMngOut(EOtheGPIO *const p)
    @brief      Gets the number of managed output pins 
    @param      The handle to the object.
    @return     The numer of managed output pins.
 **/
extern uint8_t eo_gpio_hid_GetNumMngOut(EOtheGPIO *const p);


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------







