

// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHEFORMER_H_
#define _EOTHEFORMER_H_


/** @file       EOtheFormer.h
    @brief      This header file implements public interface to the former singleton used for communication protocol
    @author     marco.accame@iit.it
    @date       09/06/2011
**/

/** @defgroup eo_theformer Object EOtheFormer
    The EOtheFormer is a singleton which is only responsible to create the content of a message in raw data form
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOrop.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


 

/** @typedef    typedef struct EOtheFormer_hid EOtheFormer
    @brief      EOtheFormer is an opaque struct. It is used to implement data abstraction for the former  
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOtheFormer_hid EOtheFormer;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
 
/** @fn         extern EOtheFormer * eo_former_Initialise(void)
    @brief      Initialise the singleton EOtheFormer. 
    @return     A valid and not-NULL const pointer to the EOtheFormer singleton.
 **/
extern EOtheFormer * eo_former_Initialise(void);


/** @fn         extern EOtheFormer * eo_former_GetHandle(void)
    @brief      Gets the handle of the EOtheFormer singleton 
    @return     Constant pointer to the singleton.
 **/
extern EOtheFormer * eo_former_GetHandle(void);


extern uint16_t eo_former_GetSizeOfStream(EOtheFormer *p, const EOrop *rop);

/** @fn         extern eOresult_t eo_former_GetStream(EOtheFormer *p, EOrop *rop_in, uint8_t *data, uint16_t size, EOrop *reply, uint8_t *replyneeded)
    @brief      Builds the packet in stream form and executes actions required by the ROP on transmission.
    @param      p               The former.
    @param      rop             The ROP to send
    @return     Normally eores_OK, eores_NOK_generic upon failure.
 **/

extern eOresult_t eo_former_GetStream(EOtheFormer *p, const EOrop *rop, const uint16_t streamcapacity, uint8_t *streamdata, uint16_t *streamsize, eOipv4addr_t *ipaddr);




/** @}            
    end of group eo_theformer  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



