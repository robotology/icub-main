
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOROPFRAME_H_
#define _EOROPFRAME_H_


/** @file       EOropframe.h
    @brief      This header file implements public interface to a frame.
    @author     marco.accame@iit.it
    @date       01/11/2010
**/

/** @defgroup eo_ropframe Object EOropframe
    The EOropframe object is used as ...
         
    @{        
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOrop.h"




// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/** @typedef    typedef struct EOropframe_hid EOropframe
    @brief      EOropframe is an opaque struct. It is used to implement data abstraction for the datagram 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOropframe_hid EOropframe;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
/** @fn         extern EOropframe* eo_ropframe_New(uint16_t capacity)
    @brief      Creates a new frame object and allocates memory able to store @e capacity bytes. If @e capacity is
                zero, then the object shall have external storage mode.
    @param      capacity   The max size of the packet.
    @return     The pointer to the required object.
 **/
extern EOropframe* eo_ropframe_New(void);

extern eOresult_t eo_ropframe_Load(EOropframe *p, uint8_t *framedata, uint16_t framesize, uint16_t framecapacity);

extern eOresult_t eo_ropframe_Unload(EOropframe *p);

extern eOresult_t eo_ropframe_Get(EOropframe *p, uint8_t **framedata, uint16_t* framesize, uint16_t* framecapacity);

extern eOresult_t eo_ropframe_Size_Get(EOropframe *p, uint16_t* framesize);

extern eOresult_t eo_ropframe_Clear(EOropframe *p);

extern eOresult_t eo_ropframe_Append(EOropframe *p, EOropframe *rfr, uint16_t *remainingbytes);

extern eObool_t eo_ropframe_IsValid(EOropframe *p);

extern uint16_t eo_ropframe_ROP_NumberOf(EOropframe *p);

// does not check anything about p safety
extern uint16_t eo_ropframe_ROP_NumberOf_quickversion(EOropframe *p);

extern eOresult_t eo_ropframe_ROP_Get(EOropframe *p, EOrop *rop, uint16_t *unparsedbytes);

extern eOresult_t eo_ropframe_ROP_Set(EOropframe *p, const EOrop *rop, uint16_t* addedinpos, uint16_t* consumedbytes, uint16_t *remainingbytes);


extern eOresult_t eo_ropframe_age_Set(EOropframe *p, eOabstime_t age);

extern eOabstime_t eo_ropframe_age_Get(EOropframe *p);



/** @}            
    end of group eo_ropframe  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

