
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOROPFRAME_HID_H_
#define _EOROPFRAME_HID_H_


/* @file       EOropframe_hid.h
    @brief      This header file implements hidden interface to a packet object.
    @author     marco.accame@iit.it
    @date       0111/2010
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOropframe.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------

#define EOFRAME_START   0x12345678
#define EOFRAME_END     0x87654321


// - definition of the hidden struct implementing the object ----------------------------------------------------------

typedef struct  // 16 bytes
{
    uint32_t    startofframe;
    uint16_t    ropssizeof;
    uint16_t    ropsnumberof;
    uint64_t    ageofframe;
} EOropframeHeader_t;

typedef struct  // 20 bytes
{
    EOropframeHeader_t header;
    uint8_t         ropsfooter[4];
} EOropframeHeaderRopsFooter_t;

typedef struct  // 04 bytes
{
    uint32_t    endoframe;
} EOropframeFooter_t;

/** @struct     EOropframe_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOropframe_hid 
{
    eOipv4addr_t                fromipaddr;
    uint16_t                    capacity;
    uint16_t                    size;
    uint16_t                    index;
    uint16_t                    currop;
    uint16_t                    remainingbytes;
    uint8_t                     externaldatastorage;
    EOropframeHeaderRopsFooter_t*  headropsfooter;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------

uint8_t* eo_ropframe_hid_get_pointer_offset(EOropframe *p, uint16_t offset);

extern eOresult_t eo_ropframe_hid_rop_rem(EOropframe *p, uint16_t startat, uint16_t size);


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




