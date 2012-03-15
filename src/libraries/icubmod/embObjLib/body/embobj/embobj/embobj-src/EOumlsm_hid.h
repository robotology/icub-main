
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOUMLSM_HID_H_
#define _EOUMLSM_HID_H_


/* @file       EOeo_umlsm_hid.h
    @brief      This header file implements hidden interface to a uml state machine.
    @author     marco.accame@iit.it
    @date       09/02/2011
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOfifoByte.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOumlsm.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------

/* @struct     EOeo_umlsm_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  

struct EOumlsm_hid 
{
    void                        *ram;
    eOumlsm_cfg_t               *cfg;
    uint8_t                     initialised;            /**< set to true first time eo_umlsm_Init() is called to avoid re-init again */
    uint8_t                     activestate;            /**< index inside states_table for the active state */
    EOfifoByte                  *internal_event_fifo;   /**< fifo queue of internal events */
//    const sm_state_t    *state;                 /**< pointer to active state */        
};


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------





