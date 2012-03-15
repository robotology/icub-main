
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _SHALIPAL_H_
#define _SHALIPAL_H_


/** @file       shalFSAL.h
    @brief      This header file implements public interface to the shalFSAL shared library.
    @author     marco.accame@iit.it
    @date       03/25/2011
**/

/** @defgroup shal_ipal Library shalIPAL
    The shalIPAL library allows ...... 
 
    @todo acemor-facenda: do documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "stdint.h"
#include "eEcommon.h"



// - public #define  --------------------------------------------------------------------------------------------------


#define SHALIPAL_ROMADDR             (EENV_MEMMAP_SHALIPAL_ROMADDR) 
#define SHALIPAL_ROMSIZE             (EENV_MEMMAP_SHALIPAL_ROMSIZE) 

#define SHALIPAL_RAMADDR             (EENV_MEMMAP_SHALIPAL_RAMADDR) 
#define SHALIPAL_RAMSIZE             (EENV_MEMMAP_SHALIPAL_RAMSIZE) 

#define SHALIPAL_STGTYPE             (ee_strg_none)
#define SHALIPAL_STGADDR             (0)
#define SHALIPAL_STGSIZE             (0)


#define SHALIPAL_SIGNATURE           ee_shalOSAL          

#define SHALIPAL_MAJOR               0x01                // change of APIs
#define SHALIPAL_MINOR               0x00                // change of internals

#define SHALIPAL_NAME                "shalFSAL"           // name ... at most 12 chars 

#define SHALIPAL_BUILDDATE_YEAR      2010
#define SHALIPAL_BUILDDATE_MONTH     3
#define SHALIPAL_BUILDDATE_DAY       7
#define SHALIPAL_BUILDDATE_HOUR      18
#define SHALIPAL_BUILDDATE_MIN       0


// - declaration of public user-defined types ------------------------------------------------------------------------- 


 

// - declaration of extern public functions ---------------------------------------------------------------------------


// inline to make it independent from presence of library
inline eEmoduleInfo_t * shalipal_moduleinfo_get(void)
{ 
    return((eEmoduleInfo_t*)(SHALIPAL_ROMADDR+EENV_MODULEINFO_OFFSET)); 
}	

// inline to make it independent from presence of library
inline eEresult_t shalipal_isvalid(void) 
{ 
    const eEmoduleInfo_t *mi = shalipal_moduleinfo_get();
    if((mi->type==ee_sharlib) &&
       (mi->signature==SHALIPAL_SIGNATURE) && (mi->version.major==SHALIPAL_MAJOR) && (mi->version.minor==SHALIPAL_MINOR) &&
       (mi->builddate.byear==SHALIPAL_BUILDDATE_YEAR) && (mi->builddate.bmonth==SHALIPAL_BUILDDATE_MONTH) &&
       (mi->builddate.bday==SHALIPAL_BUILDDATE_DAY) && (mi->builddate.bhour==SHALIPAL_BUILDDATE_HOUR) &&
       (mi->builddate.bmin==SHALIPAL_BUILDDATE_MIN)) 
       { 
            return(ee_res_OK); 
       } 
       else 
       { 
            return(ee_res_NOK_generic); 
       }
}
	

extern eEresult_t shalipal_init(uint8_t forceinit);

 

/** @}            
    end of group shal_ipal 
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



