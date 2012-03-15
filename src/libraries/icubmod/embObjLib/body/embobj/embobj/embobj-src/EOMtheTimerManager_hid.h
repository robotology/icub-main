
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMTHETIMERMANAGER_HID_H_
#define _EOMTHETIMERMANAGER_HID_H_


/* @file       EOMtheTimerManager_hid.h
    @brief      This header file implements hidden interface to the MEE timer manager singleton.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVtheTimerManager.h"
#include "EOMtask.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOMtheTimerManager.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------


/** @struct     EOMtheTimerManager_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOMtheTimerManager_hid 
{ 
    // base object
    EOVtheTimerManager          *tmrman;

    // other stuff
    EOMtask                     *tskproc;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------

// name of the task as it is shown in uvision
void sys_timerman(void *p);

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




