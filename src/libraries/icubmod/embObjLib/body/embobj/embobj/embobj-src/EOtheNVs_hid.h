

// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVS_HID_H_
#define _EOTHENVS_HID_H_


/* @file       EoTheNVs_hid.h
    @brief      This header file implements hidden interface to the NVs singleton.
    @author     marco.accame@iit.it
    @date       10/15/2009
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOtheNVs.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section



// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     EOtheNVs_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOtheNVs_hid 
{
    EOVmutexDerived*    mutex;
    EOnvsCfg*           nvscfg;   
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------

// used to load a cfg which is different from the one passed in _Initialise()
extern void eo_nvs_hid_Load(EOtheNVs *p, EOnvsCfg *c);


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------







