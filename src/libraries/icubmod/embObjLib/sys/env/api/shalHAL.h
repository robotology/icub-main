
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _SHALHAL_H_
#define _SHALHAL_H_


/** @file       shalHAL.h
    @brief      This header file implements public interface to the shalHAL shared library.
    @author     marco.accame@iit.it
    @date       03/23/2011
**/

/** @defgroup shal_hal Library shalHAL
    The shalHAL library allows ...... 
 
    @todo acemor-facenda: do documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "stdint.h"
#include "eEcommon.h"

//#include "shalHALmemmap.h"


// - public #define  --------------------------------------------------------------------------------------------------


#define SHALHAL_ROMADDR             (EENV_MEMMAP_SHALHAL_ROMADDR) //(EENV_ROMSTART + 192*1024)
#define SHALHAL_ROMSIZE             (EENV_MEMMAP_SHALHAL_ROMSIZE) //(16*1024)

#define SHALHAL_RAMADDR             (EENV_MEMMAP_SHALHAL_RAMADDR) //(EENV_RAMSTART + 60*1024)
#define SHALHAL_RAMSIZE             (EENV_MEMMAP_SHALHAL_RAMSIZE) //(256)

#define SHALHAL_STGTYPE             (ee_strg_none)
#define SHALHAL_STGADDR             (0)
#define SHALHAL_STGSIZE             (0)


#define SHALHAL_SIGNATURE           ee_shalHAL          

#define SHALHAL_MAJOR               0x01                // change of APIs
#define SHALHAL_MINOR               0x00                // change of internals

#define SHALHAL_NAME                "shalHAL"           // name ... at most 12 chars 

#define SHALHAL_BUILDDATE_YEAR      2010
#define SHALHAL_BUILDDATE_MONTH     3
#define SHALHAL_BUILDDATE_DAY       7
#define SHALHAL_BUILDDATE_HOUR      18
#define SHALHAL_BUILDDATE_MIN       0


// - declaration of public user-defined types ------------------------------------------------------------------------- 


 

// - declaration of extern public functions ---------------------------------------------------------------------------


// inline to make it independent from presence of library
inline eEmoduleInfo_t * shalhal_moduleinfo_get(void)
{ 
    return((eEmoduleInfo_t*)(SHALHAL_ROMADDR+EENV_MODULEINFO_OFFSET)); 
}	

// inline to make it independent from presence of library
inline eEresult_t shalhal_isvalid(void) 
{ 
    const eEmoduleInfo_t *mi = shalhal_moduleinfo_get();
    if((mi->type==ee_sharlib) &&
       (mi->signature==SHALHAL_SIGNATURE) && (mi->version.major==SHALHAL_MAJOR) && (mi->version.minor==SHALHAL_MINOR) &&
       (mi->builddate.byear==SHALHAL_BUILDDATE_YEAR) && (mi->builddate.bmonth==SHALHAL_BUILDDATE_MONTH) &&
       (mi->builddate.bday==SHALHAL_BUILDDATE_DAY) && (mi->builddate.bhour==SHALHAL_BUILDDATE_HOUR) &&
       (mi->builddate.bmin==SHALHAL_BUILDDATE_MIN)) 
       { 
            return(ee_res_OK); 
       } 
       else 
       { 
            return(ee_res_NOK_generic); 
       }
}
	

extern eEresult_t shalhal_init(uint8_t forceinit);

 

/** @}            
    end of group shal_hal 
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



