
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _SHALOSAL_H_
#define _SHALOSAL_H_


/** @file       shalOSAL.h
    @brief      This header file implements public interface to the shalOSAL shared library.
    @author     marco.accame@iit.it
    @date       03/25/2011
**/

/** @defgroup shal_osal Library shalOSAL
    The shalOSAL library allows ...... 
 
    @todo acemor-facenda: do documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "stdint.h"
#include "eEcommon.h"

//#include "shalHALmemmap.h"


// - public #define  --------------------------------------------------------------------------------------------------


#define SHALOSAL_ROMADDR             (EENV_MEMMAP_SHALOSAL_ROMADDR) //(EENV_ROMSTART + 192*1024)
#define SHALOSAL_ROMSIZE             (EENV_MEMMAP_SHALOSAL_ROMSIZE) //(16*1024)

#define SHALOSAL_RAMADDR             (EENV_MEMMAP_SHALOSAL_RAMADDR) //(EENV_RAMSTART + 60*1024)
#define SHALOSAL_RAMSIZE             (EENV_MEMMAP_SHALOSAL_RAMSIZE) //(256)

#define SHALOSAL_STGTYPE             (ee_strg_none)
#define SHALOSAL_STGADDR             (0)
#define SHALOSAL_STGSIZE             (0)


#define SHALOSAL_SIGNATURE           ee_shalOSAL          

#define SHALOSAL_MAJOR               0x01                // change of APIs
#define SHALOSAL_MINOR               0x00                // change of internals

#define SHALOSAL_NAME                "shalOSAL"           // name ... at most 12 chars 

#define SHALOSAL_BUILDDATE_YEAR      2010
#define SHALOSAL_BUILDDATE_MONTH     3
#define SHALOSAL_BUILDDATE_DAY       7
#define SHALOSAL_BUILDDATE_HOUR      18
#define SHALOSAL_BUILDDATE_MIN       0


// - declaration of public user-defined types ------------------------------------------------------------------------- 


 

// - declaration of extern public functions ---------------------------------------------------------------------------


// inline to make it independent from presence of library
inline eEmoduleInfo_t * shalosal_moduleinfo_get(void)
{ 
    return((eEmoduleInfo_t*)(SHALOSAL_ROMADDR+EENV_MODULEINFO_OFFSET)); 
}	

// inline to make it independent from presence of library
inline eEresult_t shalosal_isvalid(void) 
{ 
    const eEmoduleInfo_t *mi = shalosal_moduleinfo_get();
    if((mi->type==ee_sharlib) &&
       (mi->signature==SHALOSAL_SIGNATURE) && (mi->version.major==SHALOSAL_MAJOR) && (mi->version.minor==SHALOSAL_MINOR) &&
       (mi->builddate.byear==SHALOSAL_BUILDDATE_YEAR) && (mi->builddate.bmonth==SHALOSAL_BUILDDATE_MONTH) &&
       (mi->builddate.bday==SHALOSAL_BUILDDATE_DAY) && (mi->builddate.bhour==SHALOSAL_BUILDDATE_HOUR) &&
       (mi->builddate.bmin==SHALOSAL_BUILDDATE_MIN)) 
       { 
            return(ee_res_OK); 
       } 
       else 
       { 
            return(ee_res_NOK_generic); 
       }
}
	

extern eEresult_t shalosal_init(uint8_t forceinit);

 

/** @}            
    end of group shal_osal 
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



