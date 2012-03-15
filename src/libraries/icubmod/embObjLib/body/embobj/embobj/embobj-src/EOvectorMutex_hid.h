
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOVECTORMUTEX_HID_H_
#define _EOVECTORMUTEX_HID_H_


/* @file       EOvectorMutex_hid.h
    @brief      This header file implements hidden interface to a EOvectorMutex object.
    @author     valentina.gaggero@iit.it
    @date       07/12/2011
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOvector.h"
#include "EOVmutex.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOvectorMutex.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     EOvectorMutex_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOvectorMutex_hid 
{
    /* @private                pointer to vector object. */ 
    EOvector                    *vec;
    /* @private                provide mutual exclusion access to vector object. */ 
    EOVmutexDerived             *mutex;

};


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



