
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOSTHECALLBACKMANAGER_HID_H_
#define _EOSTHECALLBACKMANAGER_HID_H_


/* @file       EOStheCallbackManager_hid.h
    @brief      This header file implements hidden interface to ...
    @author     marco.accame@iit.it
    @date       08/04/2011
**/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVtask.h"
#include "EOStheFOOP.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOStheCallbackManager.h"
#include "EOVtheCallbackManager_hid.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------


/** @struct     EOStheCallbackManager_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
struct EOStheCallbackManager_hid 
{
    EOVtheCallbackManager   *vcm; 
    EOStheFOOP              *tsk;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------



#endif  // include guard




