
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _SHALBASE_H_
#define _SHALBASE_H_


/** @file       shalBASE.h
    @brief      This header file implements public interface to the shalBASE shared library.
    @author     marco.accame@iit.it
    @date       03/10/2011
**/

/** @defgroup shal_base Library shalBASE
    The shalBASE library allows ...... 
 
    @todo acemor-facenda: do documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "eEcommon.h"


// - public #define  --------------------------------------------------------------------------------------------------

#define SHALBASE_NAME                   "shalBASE"          // name ... 

#define SHALBASE_VER_MAJOR              0x01                // change of APIs
#define SHALBASE_VER_MINOR              0x01                // change of internals

#define SHALBASE_BUILDDATE_YEAR         2011
#define SHALBASE_BUILDDATE_MONTH        11
#define SHALBASE_BUILDDATE_DAY          4
#define SHALBASE_BUILDDATE_HOUR         12
#define SHALBASE_BUILDDATE_MIN          0




// - declaration of public user-defined types ------------------------------------------------------------------------- 


 

// - declaration of extern public functions ---------------------------------------------------------------------------


#if defined(SHALBASE_MODE_STATICLIBRARY) || defined(SHALS_MODE_STATIC)

extern const eEmoduleInfo_t * shalbase_moduleinfo_get(void);
extern const eEentity_t * shalbase_moduleinfo_entity_get(void);
extern eEresult_t shalbase_isvalid(void);

#else

// inline and with reference only to rom addresses to make it independent from actual presence of shalib in rom

EO_extern_inline const eEmoduleInfo_t * shalbase_moduleinfo_get(void)
{ 
    return((const eEmoduleInfo_t*)(EENV_MEMMAP_SHALBASE_ROMADDR+EENV_MODULEINFO_OFFSET)); 
}

EO_extern_inline const eEentity_t * shalbase_moduleinfo_entity_get(void)
{ 
    return((const eEentity_t*)(EENV_MEMMAP_SHALBASE_ROMADDR+EENV_MODULEINFO_OFFSET)); 
}
	
EO_extern_inline eEresult_t shalbase_isvalid(void) 
{ 
    const eEentity_t *en = shalbase_moduleinfo_entity_get();
    if((en->type==ee_entity_sharlib) && (en->signature==ee_shalBASE) && 
       (en->version.major==SHALBASE_VER_MAJOR) && (en->version.minor==SHALBASE_VER_MINOR) &&
       (en->builddate.year==SHALBASE_BUILDDATE_YEAR) && (en->builddate.month==SHALBASE_BUILDDATE_MONTH) &&
       (en->builddate.day==SHALBASE_BUILDDATE_DAY) && (en->builddate.hour==SHALBASE_BUILDDATE_HOUR) &&
       (en->builddate.min==SHALBASE_BUILDDATE_MIN)) 
       { 
            return(ee_res_OK); 
       } 
       else 
       { 
            return(ee_res_NOK_generic); 
       }
}

#endif	



extern eEresult_t shalbase_init(uint8_t forcestorageinit);
extern eEresult_t shalbase_deinit(void);

// only the eLoader calls shalbase_boardinfo_synchronise()
extern eEresult_t shalbase_boardinfo_synchronise(const eEboardInfo_t* boardinfo);
extern eEresult_t shalbase_boardinfo_get(const eEboardInfo_t** boardinfo);

extern eEresult_t shalbase_ipc_gotoproc_get(eEprocess_t *pr);			
extern eEresult_t shalbase_ipc_gotoproc_set(eEprocess_t pr);
extern eEresult_t shalbase_ipc_gotoproc_clr(void);

extern eEresult_t shalbase_ipc_volatiledata_get(uint8_t *data, uint8_t *size, const uint8_t maxsize);		
extern eEresult_t shalbase_ipc_volatiledata_set(uint8_t *data, uint8_t size);
extern eEresult_t shalbase_ipc_volatiledata_clr(void);

extern eEresult_t shalbase_system_canjump(uint32_t addr);
extern eEresult_t shalbase_system_canjump_to_proc(uint32_t addr, eEmoduleInfo_t *procinfo);
extern eEresult_t shalbase_system_jumpnow(uint32_t addr);
extern eEresult_t shalbase_system_restart(void);

extern eEresult_t shalbase_storage_get(const eEstorage_t *strg, void *data, uint32_t size);
extern eEresult_t shalbase_storage_set(const eEstorage_t *strg, const void *data, uint32_t size);
extern eEresult_t shalbase_storage_clr(const eEstorage_t *strg, const uint32_t size);
 
 
 

/** @}            
    end of group shal_base 
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



