
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _SHALPART_H_
#define _SHALPART_H_


/** @file       shalPART.h
    @brief      This header file implements public interface to the shalPART shared library.
    @author     marco.accame@iit.it
    @date       03/07/2011
**/

/** @defgroup shal_part  shalPART
    The shalPART shared library allows ...... 
 
    @todo acemor-facenda: do documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "stdint.h"

#include "eEcommon.h"


// - public #define  --------------------------------------------------------------------------------------------------

      
#define SHALPART_NAME                   "shalPART"          // name ... 

#define SHALPART_VER_MAJOR              0x01                // change of APIs
#define SHALPART_VER_MINOR              0x01                // change of internals

#define SHALPART_BUILDDATE_YEAR         2011
#define SHALPART_BUILDDATE_MONTH        11
#define SHALPART_BUILDDATE_DAY          4
#define SHALPART_BUILDDATE_HOUR         18
#define SHALPART_BUILDDATE_MIN          0



// - declaration of public user-defined types ------------------------------------------------------------------------- 

typedef enum 
{
    shalpart_reset_default  = 0, 
    shalpart_reset_rawvals = 1
} shalpart_reset_mode_t;

// - declaration of extern public functions ---------------------------------------------------------------------------


#if defined(SHALPART_MODE_STATICLIBRARY)  || defined(SHALS_MODE_STATIC)

extern const eEmoduleInfo_t * shalpart_moduleinfo_get(void);
extern const eEentity_t * shalpart_moduleinfo_entity_get(void);
extern eEresult_t shalpart_isvalid(void);

#else

// inline and with reference only to rom addresses to make it independent from actual presence of shalib in rom

EO_extern_inline const eEmoduleInfo_t * shalpart_moduleinfo_get(void)
{ 
    return((const eEmoduleInfo_t*)(EENV_MEMMAP_SHALPART_ROMADDR+EENV_MODULEINFO_OFFSET)); 
}

EO_extern_inline const eEentity_t * shalpart_moduleinfo_entity_get(void)
{ 
    return((const eEentity_t*)(EENV_MEMMAP_SHALPART_ROMADDR+EENV_MODULEINFO_OFFSET)); 
}
	
EO_extern_inline eEresult_t shalpart_isvalid(void) 
{ 
    const eEentity_t *en = shalpart_moduleinfo_entity_get();
    if((en->type==ee_entity_sharlib) && (en->signature==ee_shalPART) && 
       (en->version.major==SHALPART_VER_MAJOR) && (en->version.minor==SHALPART_VER_MINOR) &&
       (en->builddate.year==SHALPART_BUILDDATE_YEAR) && (en->builddate.month==SHALPART_BUILDDATE_MONTH) &&
       (en->builddate.day==SHALPART_BUILDDATE_DAY) && (en->builddate.hour==SHALPART_BUILDDATE_HOUR) &&
       (en->builddate.min==SHALPART_BUILDDATE_MIN)) 
       { 
            return(ee_res_OK); 
       } 
       else 
       { 
            return(ee_res_NOK_generic); 
       }
}

#endif



extern eEresult_t shalpart_init(void);
extern eEresult_t shalpart_reset(shalpart_reset_mode_t rm);
extern eEresult_t shalpart_deinit(void);

extern eEresult_t shalpart_proc_synchronise(eEprocess_t proc, const eEmoduleInfo_t *moduleinfo);
extern eEresult_t shalpart_proc_def2run_get(eEprocess_t *proc);
extern eEresult_t shalpart_proc_def2run_set(eEprocess_t proc);
extern eEresult_t shalpart_proc_runaddress_get(eEprocess_t proc, uint32_t *addr);

extern eEresult_t shalpart_proc_allavailable_get(const eEprocess_t **table, uint8_t *size);

extern eEresult_t shalpart_proc_add(eEprocess_t proc, eEmoduleInfo_t *moduleinfo);
extern eEresult_t shalpart_proc_rem(eEprocess_t proc);
extern eEresult_t shalpart_proc_set(eEprocess_t proc, eEmoduleInfo_t *moduleinfo);
extern eEresult_t shalpart_proc_get(eEprocess_t proc, const eEmoduleInfo_t **moduleinfo);

extern eEresult_t shalpart_shal_synchronise(eEsharlib_t shal, const eEmoduleInfo_t *moduleinfo);

extern eEresult_t shalpart_shal_allavailable_get(const eEsharlib_t **table, uint8_t *size);

extern eEresult_t shalpart_shal_add(eEsharlib_t shal, eEmoduleInfo_t *moduleinfo);
extern eEresult_t shalpart_shal_rem(eEsharlib_t shal);
extern eEresult_t shalpart_shal_set(eEsharlib_t shal, eEmoduleInfo_t *moduleinfo);
extern eEresult_t shalpart_shal_get(eEsharlib_t shal, const eEmoduleInfo_t **moduleinfo);
                                                

 
 
 

/** @}            
    end of group shal_part 
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



