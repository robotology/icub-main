
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMTHEGPIOMANAGER_HID_H_
#define _EOMTHEGPIOMANAGER_HID_H_


/* @file       EOMtheGPIOManager_hid.h
    @brief      This header file implements hidden interface to the gpio manager singleton used in MEE
    @author     marco.accame@iit.it
    @date       08/24/2011
**/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOtheGPIOManager.h"
#include "EOMtask.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOMtheGPIOManager.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section



// - definition of the hidden struct implementing the object ----------------------------------------------------------


/** @struct     EOMtheGPIOManager_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOMtheGPIOManager_hid 
{
    // base object
    EOtheGPIOManager            *gpioman;
    // other stuff
    EOMtask                     *tskproc;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------

// name used by u-vision
void eom_gpioman(void *p);


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




