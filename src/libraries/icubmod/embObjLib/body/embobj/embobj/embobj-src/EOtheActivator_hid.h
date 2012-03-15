
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHEACTIVATOR_HID_H_
#define _EOTHEACTIVATOR_HID_H_


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EoAction_hid.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOtheActivator.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------

struct EOtheActivator_hid 
{
    EOvectorMutex           *vector;
    eOreltime_t             tout;
    EOaction                act;
    eOsizecntnr_t           onnumitems;
    eObool_t                activated;   
}; 


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


