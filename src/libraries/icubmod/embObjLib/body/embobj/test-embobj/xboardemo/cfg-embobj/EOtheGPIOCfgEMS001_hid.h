
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHEGPIOCFGEMS001_HID_H_
#define _EOTHEGPIOCFGEMS001_HID_H_


/** @file       EOtheGPIOCfgEMS001_hid.h
    @brief      This header file implements hidden interface to the configuration of gpio for the ems001 board
    @author     marco.accame@iit.it
    @date       10/15/2009
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVtheGPIOCfg.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOtheGPIOCfgEMS001.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------


/** @struct     EOtheGPIOCfgEMS001_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/ 

struct EOtheGPIOCfgEMS001_hid 
{
    /** Derive a EOVtheGPIOCfg object by placing an object identifier for type EOVtheGPIOCfg first       **/
    const uint32_t                  eoidentifier;
    /** And finally add a EOVtheGPIOCfg pointer                                                          **/
    const EOVtheGPIOCfg * const     theconfig;
};


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



