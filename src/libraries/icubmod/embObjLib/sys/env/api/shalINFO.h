
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _SHALINFO_H_
#define _SHALINFO_H_


/** @file       shalINFO.h
    @brief      This header file implements public interface to the shalINFO library.
    @author     marco.accame@iit.it
    @date       03/11/2011
**/

/** @defgroup shal_info Library shalINFO
    The shalINFO library allows ...... 
 
    @todo acemor-facenda: do documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "stdint.h"
#include "eEcommon.h"


// - public #define  --------------------------------------------------------------------------------------------------

#define SHALINFO_NAME                   "shalINFO"           

#define SHALINFO_VER_MAJOR              0x01                // change of APIs
#define SHALINFO_VER_MINOR              0x00                // change of internals

#define SHALINFO_BUILDDATE_YEAR         2011
#define SHALINFO_BUILDDATE_MONTH        11
#define SHALINFO_BUILDDATE_DAY          3
#define SHALINFO_BUILDDATE_HOUR         18
#define SHALINFO_BUILDDATE_MIN          0




// - declaration of public user-defined types ------------------------------------------------------------------------- 


typedef enum
{
    shalinfo_ipnet          = 0,
    shalinfo_can1net        = 1,
    shalinfo_can2net        = 2,
    shalinfo_page08         = 3,
    shalinfo_page32         = 4,
    shalinfo_page64         = 5,
    shalinfo_page128        = 6
} shalinfo_deviceinfo_part_t;

typedef struct                  // 256B              
{
    eEipnetwork_t               ipnetwork;      //016B 
    eEcannetwork_t              can1network;    //004B
    eEcannetwork_t              can2network;    //004B
    uint8_t                     page08[8];      //008B
    uint8_t                     page32[32];     //032B
    uint8_t                     page64[64];     //064B
    uint8_t                     page128[128];   //128B
} shalinfo_deviceinfo_t;        EECOMMON_VERIFYsizeof(shalinfo_deviceinfo_t, 256);


// - declaration of extern public functions ---------------------------------------------------------------------------

#if defined(SHALINFO_MODE_STATICLIBRARY) || defined(SHALS_MODE_STATIC)

extern const eEmoduleInfo_t * shalinfo_moduleinfo_get(void);
extern const eEentity_t * shalinfo_moduleinfo_entity_get(void);
extern eEresult_t shalinfo_isvalid(void);

#else

// inline and with reference only to rom addresses to make it independent from actual presence of shalib in rom

EO_extern_inline const eEmoduleInfo_t * shalinfo_moduleinfo_get(void)
{ 
    return((const eEmoduleInfo_t*)(EENV_MEMMAP_SHALINFO_ROMADDR+EENV_MODULEINFO_OFFSET)); 
}

EO_extern_inline const eEentity_t * shalinfo_moduleinfo_entity_get(void)
{ 
    return((const eEentity_t*)(EENV_MEMMAP_SHALINFO_ROMADDR+EENV_MODULEINFO_OFFSET)); 
}
	
EO_extern_inline eEresult_t shalinfo_isvalid(void) 
{ 
    const eEentity_t *en = shalinfo_moduleinfo_entity_get();
    if((en->type==ee_entity_sharlib) && (en->signature==ee_shalINFO) && 
       (en->version.major==SHALINFO_VER_MAJOR) && (en->version.minor==SHALINFO_VER_MINOR) &&
       (en->builddate.year==SHALINFO_BUILDDATE_YEAR) && (en->builddate.month==SHALINFO_BUILDDATE_MONTH) &&
       (en->builddate.day==SHALINFO_BUILDDATE_DAY) && (en->builddate.hour==SHALINFO_BUILDDATE_HOUR) &&
       (en->builddate.min==SHALINFO_BUILDDATE_MIN)) 
       { 
            return(ee_res_OK); 
       } 
       else 
       { 
            return(ee_res_NOK_generic); 
       }
}

#endif	

	


extern eEresult_t shalinfo_init(void);
extern eEresult_t shalinfo_deinit(void);
extern eEresult_t shalinfo_erase(void);


// only the loader calls it with its own board-info.
extern eEresult_t shalinfo_boardinfo_synchronise(const eEboardInfo_t* boardinfo);
// other eprocesses or shared libraries just get it.
extern eEresult_t shalinfo_boardinfo_get(const eEboardInfo_t** boardinfo);


extern eEresult_t shalinfo_deviceinfo_clr(void);
extern eEresult_t shalinfo_deviceinfo_get(const shalinfo_deviceinfo_t** deviceinfo);
extern eEresult_t shalinfo_deviceinfo_set(const shalinfo_deviceinfo_t* deviceinfo);


extern eEresult_t shalinfo_deviceinfo_part_clr(shalinfo_deviceinfo_part_t part);
extern eEresult_t shalinfo_deviceinfo_part_get(shalinfo_deviceinfo_part_t part, const void** data);
extern eEresult_t shalinfo_deviceinfo_part_set(shalinfo_deviceinfo_part_t part, const void* data);


 
 

/** @}            
    end of group shal_info 
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



