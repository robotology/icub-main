
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _SHALFSAL_H_
#define _SHALFSAL_H_


/** @file       shalFSAL.h
    @brief      This header file implements public interface to the shalFSAL shared library.
    @author     marco.accame@iit.it
    @date       03/25/2011
**/

/** @defgroup shal_Fsal Library shalFSAL
    The shalFSAL library allows ...... 
 
    @todo acemor-facenda: do documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "stdint.h"
#include "eEcommon.h"

//#include "shalHALmemmap.h"


// - public #define  --------------------------------------------------------------------------------------------------


#define SHALFSAL_ROMADDR             (EENV_MEMMAP_SHALFSAL_ROMADDR) 
#define SHALFSAL_ROMSIZE             (EENV_MEMMAP_SHALFSAL_ROMSIZE) 

#define SHALFSAL_RAMADDR             (EENV_MEMMAP_SHALFSAL_RAMADDR) 
#define SHALFSAL_RAMSIZE             (EENV_MEMMAP_SHALFSAL_RAMSIZE) 

#define SHALFSAL_STGTYPE             (ee_strg_none)
#define SHALFSAL_STGADDR             (0)
#define SHALFSAL_STGSIZE             (0)


#define SHALFSAL_SIGNATURE           ee_shalOSAL          

#define SHALFSAL_MAJOR               0x01                // change of APIs
#define SHALFSAL_MINOR               0x00                // change of internals

#define SHALFSAL_NAME                "shalFSAL"           // name ... at most 12 chars 

#define SHALFSAL_BUILDDATE_YEAR      2010
#define SHALFSAL_BUILDDATE_MONTH     3
#define SHALFSAL_BUILDDATE_DAY       7
#define SHALFSAL_BUILDDATE_HOUR      18
#define SHALFSAL_BUILDDATE_MIN       0


// - declaration of public user-defined types ------------------------------------------------------------------------- 


 

// - declaration of extern public functions ---------------------------------------------------------------------------


// inline to make it independent from presence of library
inline eEmoduleInfo_t * shalfsal_moduleinfo_get(void)
{ 
    return((eEmoduleInfo_t*)(SHALFSAL_ROMADDR+EENV_MODULEINFO_OFFSET)); 
}	

// inline to make it independent from presence of library
inline eEresult_t shalfsal_isvalid(void) 
{ 
    const eEmoduleInfo_t *mi = shalfsal_moduleinfo_get();
    if((mi->type==ee_sharlib) &&
       (mi->signature==SHALFSAL_SIGNATURE) && (mi->version.major==SHALFSAL_MAJOR) && (mi->version.minor==SHALFSAL_MINOR) &&
       (mi->builddate.byear==SHALFSAL_BUILDDATE_YEAR) && (mi->builddate.bmonth==SHALFSAL_BUILDDATE_MONTH) &&
       (mi->builddate.bday==SHALFSAL_BUILDDATE_DAY) && (mi->builddate.bhour==SHALFSAL_BUILDDATE_HOUR) &&
       (mi->builddate.bmin==SHALFSAL_BUILDDATE_MIN)) 
       { 
            return(ee_res_OK); 
       } 
       else 
       { 
            return(ee_res_NOK_generic); 
       }
}
	

extern eEresult_t shalfsal_init(uint8_t forceinit);

 

/** @}            
    end of group shal_fsal 
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



