
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOIOPINOUTPUT_HID_H_
#define _EOIOPINOUTPUT_HID_H_


/* @file       EOioPinOutput_hid.h
    @brief      This header file implements hidden interface to an output pin object.
    @author     marco.accame@iit.it
    @date       10/15/2009
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOioPin.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOioPinOutput.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     EOioPinOutput_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOioPinOutput_hid 
{
    EOioPin         *iopin;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------


/** @fn         extern EOioPinOutput * eo_iopinout_hid_NewArray(uint8_t n))
    @brief      Creates a new array of EOioPinOutput objects. 
    @param      n               The size of the array. It must be > 0.
    @return     The pointer to the required array of object. The pointer is guaranteed to be always valid and never 
                to be NULL, because failure is managed by the memory pool.
 **/ 
 extern EOioPinOutput * eo_iopinout_hid_NewArray(uint8_t n);
 

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

