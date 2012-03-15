
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EUPDATERPARSER_H_
#define _EUPDATERPARSER_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       eupdater-parser.h
    @brief      This header file implements ....
    @author     marco.accame@iit.it
    @date       01/11/2012
**/

/** @defgroup eupdaterinfo cedcew fcevw
    The embENV allows ...... 
 
    @todo acemor-facenda: do documentation.
    

    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOpacket.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section

// - declaration of public user-defined types ------------------------------------------------------------------------- 
// empty-section

// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------



// - declaration of extern public functions ---------------------------------------------------------------------------


extern void eupdater_parser_init(void);

extern eObool_t eupdater_parser_process_rop(EOpacket *rxpkt, EOpacket *txpkt);

extern eObool_t eupdater_parser_process_data(EOpacket *rxpkt, EOpacket *txpkt);
 

/** @}            
    end of group eupdaterparser 
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


