
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOIOPININPUT_HID_H_
#define _EOIOPININPUT_HID_H_


/* @file       EOioPinInput_hid.h
    @brief      This header file implements hidden interface to an input pin object.
    @author     marco.accame@iit.it
    @date       10/15/2009
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOioPin.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOioPinInput.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     EOiopinInp_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOioPinInput_hid 
{
    EOioPin         *iopin;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------


/** @fn         extern EOioPinInput * eo_iopininp_hid_NewArray(uint8_t n))
    @brief      Creates a new array of EOioPinInput objects. 
    @param      n               The size of the array. It must be > 0.
    @return     The pointer to the required array of object. The pointer is guaranteed to be always valid and never 
                to be NULL, because failure is managed by the memory pool.
 **/ 
extern EOioPinInput * eo_iopininp_hid_NewArray(uint8_t n);
 

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

