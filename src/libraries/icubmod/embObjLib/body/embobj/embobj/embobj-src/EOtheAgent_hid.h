

// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHEAGENT_HID_H_
#define _EOTHEAGENT_HID_H_


/* @file       EOtheAgent_hid.h
    @brief      This header file implements hidden interface to the EOtheAgent singleton.
    @author     marco.accame@iit.it
    @date       09/03/2010
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOtheAgent.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section



// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     EOtheAgent_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOtheAgent_hid 
{
    uint8_t                 initted;
    const eOagent_cfg_t*    cfg;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------

// to be called only once just before transmission
extern eOresult_t eo_agent_hid_OutROPonTransmission(EOtheAgent *p, EOrop *rop);

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------







