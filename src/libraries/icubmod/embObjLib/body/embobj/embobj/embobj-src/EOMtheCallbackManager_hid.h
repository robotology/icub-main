
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMTHECALLBACKMANAGER_HID_H_
#define _EOMTHECALLBACKMANAGER_HID_H_


/* @file       EOMtheCallbackManager_hid.h
    @brief      This header file implements hidden interface to the singleton.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOMtask.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOMtheCallbackManager.h"
#include "EOVtheCallbackManager_hid.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------


/** @struct     EOMtheCallbackManager_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOMtheCallbackManager_hid 
{ 
    EOVtheCallbackManager   *vcm;
    EOMtask                 *tsk;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------

void sys_cbackman(void *p);


#endif  // include guard




