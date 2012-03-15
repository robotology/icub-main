
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOSTHESYSTEM_HID_H_
#define _EOSTHESYSTEM_HID_H_


/* @file       EONtheSystem_hid.h
    @brief      This header file implements hidden interface to the system singleton.
    @author     marco.accame@iit.it
    @date       04/07/2011
**/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVtheSystem.h"
#include "EOVtask_hid.h" 
#include "EOfifoWord.h"
#include "EOfifo.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EONtheSystem.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section



// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     EONtheSystem_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EONtheSystem_hid 
{
    // base object
    EOVtheSystem                *thevsys;
    // other stuff
    eOabstime_t                 lifetime;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------

extern uint64_t eon_sys_hid_tickperiodget(void);

extern uint64_t eon_sys_hid_tickoflifeget(void);



#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

