
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOIOPININPUTMANAGED_HID_H_
#define _EOIOPININPUTMANAGED_HID_H_


/* @file       EOioPinInputManaged_hid.h
    @brief      This header file implements hidden interface to a managed input pin object.
    @author     marco.accame@iit.it
    @date       10/16/2009
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOioPin.h"
#include "EOaction_hid.h" // to allow access to the full type

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOioPinInputManaged.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------

#define NACT    4


// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     EOioPinInputManaged_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOioPinInputManaged_hid 
{
    EOioPin        *iopin;
    EOaction        actionon[NACT];         // points to actions to be performed upon one of the onrise or onfall 
    eOreltime_t        counter;                // keeps usecs as time goes by    
    eOreltime_t        afterrise;              // stores usecs to wait for after rise before to issue an action
    eOreltime_t        afterfall;              // stores usecs to wait for after fall before to issue an action
    uint8_t         counting:           1;  //if 1 there is at least one action active
    uint8_t         trigonrise_hit:     1;  // if 1, then when pin goes high an action is done immediately
    uint8_t         trigonfall_hit:     1;  // if 1, then when pin goes low an action is done immediately.
    uint8_t         trigonrise_stay:    1;  // if 1, then when pin stays high for afterrise time, an action is done
    uint8_t         trigonfall_stay:    1;  // if 1, then when pin stays low for afterfall time, an action is done
    uint8_t         maxactions:         3;  // max number of actions that the object can act on    
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------


/** @fn         extern EOioPinInputManaged * eo_iopininpman_hid_NewArray(uint8_t n))
    @brief      Creates a new array of EOioPinInputManaged objects. 
    @param      n               The size of the array. It must be > 0.
    @return     The pointer to the required array of object. The pointer is guaranteed to be always valid and never 
                to be NULL, because failure is managed by the memory pool.
 **/ 
extern EOioPinInputManaged * eo_iopininpman_hid_NewArray(uint8_t n);
 

/** @fn         extern void eo_iopininpman_hid_Reset(EOioPinInputManaged const* p)
    @brief      Resets one EOioPinInputManaged object. 
 **/
extern void eo_iopininpman_hid_Reset(EOioPinInputManaged *const p);
 

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

