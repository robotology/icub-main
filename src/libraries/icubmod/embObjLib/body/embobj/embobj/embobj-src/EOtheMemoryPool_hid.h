
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _THEMEMORYPOOL_HID_H_
#define _THEMEMORYPOOL_HID_H_


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOtheMemoryPool.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------

struct EOtheMemoryPool_hid 
{
    const eOmempool_cfg_t   *cfg;
    EOVmutex                *mutex;
    eOreltime_t             tout;
    eOmempool_allocmode_t   allocmode;
    uint8_t                 staticmask;
    uint8_t                 initted;
    uint16_t                uint08s_num;
    uint16_t                uint16s_num;
    uint16_t                uint32s_num;
    uint16_t                uint64s_num;
    uint32_t                usedbytes;
}; 


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


