
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGMASTER_HID_H_
#define _EOTHENVSCFGMASTER_HID_H_


/** @file       EOtheNVsCfgMaster_hid.h
    @brief      This header file implements hidden interface to the configuration of nvs for an example
    @author     marco.accame@iit.it
    @date       10/15/2009
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOtheNVs.h"
#include "EOVtheNVsCfg.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOtheNVsCfgMaster.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------


// - typedef with hidden scope ----------------------------------------------------------------------------------------


// - definition of the hidden struct implementing the object ----------------------------------------------------------


/** @struct     EOtheNVsCfgMaster_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/ 

struct EOtheNVsCfgMaster_hid 
{
    /** Derive a EOVtheGPIOCfg object by placing an object identifier for type EOVtheNVsCfg first       **/
    const uint32_t                  eoidentifier;
    /** And finally add a EOVtheNVsfg pointer                                                          **/
    const EOVtheNVsCfg * const      theconfig;
};


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



