
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOSTHETIMERMANAGER_HID_H_
#define _EOSTHETIMERMANAGER_HID_H_


/* @file       EOStheTimerManager_hid.h
    @brief      This header file implements hidden interface to the rtos timer manager singleton.
    @author     marco.accame@iit.it
    @date       08/04/2011
**/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVtheTimerManager.h"
#include "EOlist.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOStheTimerManager.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------


/* @struct     EOStheTimerManager_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOStheTimerManager_hid 
{ 
    // base object
    EOVtheTimerManager          *tmrman;

    // other stuff
    EOlist                      *activetimers;
    uint64_t                    lastprocessedsystick;
    uint32_t                    tickperiod;
    EOtimer                     **tmptmrs;
    uint8_t                     numtimers;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------

/* @fn         extern void eos_hid_timerman_Synch(EOStheTimerManager *p, uint64_t oldtick, uint64_t newtick)
    @brief      Adjusts the internal expiry times of the manged EOtimer objects according to current time of life.
                It is automatically called by eos_sys_TimelifeSet() after a change of time.
 **/
extern void eos_hid_timerman_Synch(EOStheTimerManager *p, uint64_t oldtick, uint64_t newtick);


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




