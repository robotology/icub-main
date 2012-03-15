
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _DEMOINFO_H_
#define _DEMOINFO_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       demo-info.h
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

#include "eEcommon.h"
#include "ipal.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section

// - declaration of public user-defined types ------------------------------------------------------------------------- 
// empty-section

// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const eEboardInfo_t              demoinfo_boardinfo;

extern const eEmoduleInfo_t             demoinfo_modinfo;

extern const eOmsystem_cfg_t            demoinfo_syscfg;

extern const ipal_cfg_t*                demoinfo_ipal_cfg;


// - declaration of extern public functions ---------------------------------------------------------------------------
// empty-section
 

/** @}            
    end of group demoinfo 
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


