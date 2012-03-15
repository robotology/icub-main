
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHEERRORMANAGER_HID_H_
#define _EOTHEERRORMANAGER_HID_H_


/* @file       EOtheErrorManager_hid.h
    @brief      This header file implements hidden interface to the error manager
    @author     marco.accame@iit.it
    @date       10/15/2009
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOtheErrorManager.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section

// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     EOtheErrorManager_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
struct EOtheErrorManager_hid 
{
	eOerrman_cfg_t  cfg;
};


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

