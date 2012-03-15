

// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHEPARSER_H_
#define _EOTHEPARSER_H_


/** @file       EOtheParser.h
    @brief      This header file implements public interface to the parser singleton used for communication protocol
    @author     marco.accame@iit.it
    @date       09/06/2011
**/

/** @defgroup eo_theparser Object EOthePARSER
    The EOthePARSER is a singleton which is only responsible to evaluate the content of a received ip message and to 
    fill a EOrop object. The processing of the EOrop is done by the EOrop object itself by means of EOtheAgent.    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOrop.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


 

/** @typedef    typedef struct EOtheParser_hid EOtheParser
    @brief      EOtheParser is an opaque struct. It is used to implement data abstraction for the Parser  
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOtheParser_hid EOtheParser;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
 
/** @fn         extern EOtheParser * eo_parser_Initialise(void)
    @brief      Initialise the singleton EOtheParser. 
    @return     A valid and not-NULL pointer to the EOtheParser singleton.
 **/
extern EOtheParser * eo_parser_Initialise(void);


/** @fn         extern EOtheParser * eo_parser_GetHandle(void)
    @brief      Gets the handle of the EOtheParser singleton 
    @return     Constant pointer to the singleton.
 **/
extern EOtheParser * eo_parser_GetHandle(void);


/** @fn         extern eOresult_t eo_parser_GetROP(EOtheParser *p, uint8_t *pktdata, uint16_t pktsize, EOrop *rop, uint16_t *consumedbytes)
    @brief      Gets a single ROP from a received packet and tells how much of the packet data is used by the rop, so that a new
                call of the function will be passed the packet data with an offset. 
    @param      pktdata         The input data
    @param      pktsize         The size of the input data
    @param      rop             The extracted rop.
    @param      consumedbytes   The number of bytes used by the retrieved rop.
    @return     The value eores_NOK_nullpointer if any is a NULL pointer, eores_NOK_generic if pktdata does not have a valid rop, 
                eores_OK if the function can fill @e rop with meaninful data.
 **/
extern eOresult_t eo_parser_GetROP(EOtheParser *p, const uint8_t *streamdata, const uint16_t streamsize, const eOipv4addr_t ipaddr, EOrop *rop, uint16_t *consumedbytes);





/** @}            
    end of group eo_theparser  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



