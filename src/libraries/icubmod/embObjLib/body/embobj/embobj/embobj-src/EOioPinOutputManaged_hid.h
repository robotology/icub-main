
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOIOPINOUTPUTMANAGED_HID_H_
#define _EOIOPINOUTPUTMANAGED_HID_H_


/* @file       EOioPinOutputManaged_hid.h
    @brief      This header file implements hidden interface to a managed output pin object.
    @author     marco.accame@iit.it
    @date       10/19/2009
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOioPin.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOioPinOutputManaged.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     EOioPinOutputManaged_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOioPinOutputManaged_hid 
{
    EOioPin        *iopin;
    eOreltime_t        counter;                // keeps usecs as time goes by         
    eOreltime_t        ushalf01;               // keeps duration in usecs of first half of the waveform        
    eOreltime_t        ushalf02;               // keeps duration in usecs of second half of the waveform
    uint32_t        numwaves;               // keeps number of times the waveform is wanted    
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------


/** @fn         extern EOioPinOutputManaged * eo_iopinoutman_hid_NewArray(uint8_t n)
    @brief      Creates a new array of EOioPinOutputManaged objects. 
    @param      n               The size of the array. It must be > 0.
    @return     The pointer to the required array of object. The pointer is guaranteed to be always valid and never 
                to be NULL, because failure is managed by the memory pool.
 **/ 
extern EOioPinOutputManaged * eo_iopinoutman_hid_NewArray(uint8_t n);
 

/** @fn         extern void eo_iopinoutman_hid_Reset(EOioPinOutputManaged *const p)
    @brief      Resets one EOioPinOutputManaged object. 
 **/
extern void eo_iopinoutman_hid_Reset(EOioPinOutputManaged *const p);
 

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------






