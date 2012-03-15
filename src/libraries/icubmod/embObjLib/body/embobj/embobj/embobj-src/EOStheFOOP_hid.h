
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOSTHEFOOP_HID_H_
#define _EOSTHEFOOP_HID_H_


/* @file       EOStheFOOP_hid.h
    @brief      This header file implements hidden interface to the foop singleton.
    @author     marco.accame@iit.it
    @date       08/04/2011
**/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVtask_hid.h" 
#include "EOfifoWord.h"
#include "EOfifo.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOStheFOOP.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section



// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     EOStheFOOP_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
struct EOStheFOOP_hid
{
    // base object
    EOVtask                         *tsk;

    // other stuff
    eOsfoop_cfg_t                   cfg;
    eObasicabstr_hal_sys_fn_t       hfn;
    eOvoid_fp_void_t                ontick;
    volatile uint32_t               flags;
    volatile uint32_t               events_mask;
    EOfifoWord                      *message_fifo;
    EOfifo                          *callback_fifo;        
    EOfifo                          *argument_fifo;  
};


// - declaration of extern hidden functions ---------------------------------------------------------------------------


extern eOresult_t eos_foop_hid_SetOnTick(EOStheFOOP *p, eOvoid_fp_void_t ontick);

extern eOresult_t eos_foop_hid_Tick(EOStheFOOP *p);


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

