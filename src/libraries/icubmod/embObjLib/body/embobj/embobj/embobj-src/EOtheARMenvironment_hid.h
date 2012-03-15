
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHEARMENVIRONMENT_HID_H_
#define _EOTHEARMENVIRONMENT_HID_H_


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "eEcommon.h"
#include "shalINFO.h"
#include "EOVtheEnvironment.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOtheARMenvironment.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------

struct EOtheARMenvironment_hid 
{
    // the base object
    EOVtheEnvironment           *env;
    // other stuff
    const eEmoduleInfo_t        *modinfo; 
    const eEboardInfo_t         *brdinfo;
    const shalinfo_deviceinfo_t *devinfo;
    uint32_t                    codeprocoffset;
    eEprocess_t                 eprocess; 
}; 


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


