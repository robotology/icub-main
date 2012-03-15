
/* @file       shalBASE.c
    @brief      This header file implements the shalIPC library.
    @author     marco.accame@iit.it
    @date       05/07/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#ifdef DONTOPTIMISEBASE 
#pragma O0
#endif

#include "hal.h"

#include "string.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "shalBASE.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------

#define SHALBASE_F2USESIZE          (20) 


typedef struct                                      // 16B
{
    eEentity_t              entity;                 // 8B
    uint8_t                 defflag;                // 1B
    uint8_t                 cached;                 // 1B
    uint8_t                 strginitted;            // 1B
    uint8_t                 forfutureuse1;          // 1B
    uint8_t                 filler[4];              // 4B
} baseHead_t;               EECOMMON_VERIFYsizeof(baseHead_t, 16); 

typedef struct                                      // 24B
{
    uint8_t                 gotoflag;
    eEprocess_t             gotoval;
    uint8_t                 free2useflag;
    uint8_t                 free2usesize;
    uint8_t                 free2usedata[SHALBASE_F2USESIZE];
} baseData_t;               EECOMMON_VERIFYsizeof(baseData_t, 24);

typedef struct                                      // 40B
{
    baseHead_t              head;                   // 16B
    baseData_t              data;                   // 24B
} baseInfo_t;               EECOMMON_VERIFYsizeof(baseInfo_t, 40);


typedef struct              // 80B
{
    baseHead_t              head;                   // 16B
    eEboardInfo_t           boardinfo;              // 64B
} baseBoardInfo_t;          EECOMMON_VERIFYsizeof(baseBoardInfo_t, 80);


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------


#define SHALBASE_ROMADDR            (EENV_MEMMAP_SHALBASE_ROMADDR)
#define SHALBASE_ROMSIZE            (EENV_MEMMAP_SHALBASE_ROMSIZE)

#define SHALBASE_RAMADDR            (EENV_MEMMAP_SHALBASE_RAMADDR)
#define SHALBASE_RAMSIZE            (EENV_MEMMAP_SHALBASE_RAMSIZE)

#define SHALBASE_STGTYPE            (ee_strg_eeprom)
#define SHALBASE_STGADDR            (EENV_MEMMAP_SHALBASE_STGADDR)
#define SHALBASE_STGSIZE            (EENV_MEMMAP_SHALBASE_STGSIZE)

// the ram size to be used in scatter-file and the one used by the program for static ram
#define SHALBASE_RAMFOR_RWDATA      (EENV_MEMMAP_SHALBASE_RAMFOR_RWDATA)

// the ram size to be used with __attribute__((at(SHALBASE_RAMADDR))), which is sizeof(baseInfo_t)
#define SHALBASE_RAMFOR_ZIDATA      (EENV_MEMMAP_SHALBASE_RAMFOR_ZIDATA)

// and its control
typedef int dummy1[sizeof(baseInfo_t)     <= ((SHALBASE_RAMSIZE-SHALBASE_RAMFOR_RWDATA)) ? 1 : -1];
typedef int dummy2[SHALBASE_RAMFOR_ZIDATA <= ((SHALBASE_RAMSIZE-SHALBASE_RAMFOR_RWDATA)) ? 1 : -1];


#define EENV_MEMMAP_SHALBASE_STGADDR_BOARDINFO    (EENV_MEMMAP_SHALBASE_STGADDR+sizeof(baseInfo_t))


// - flags ------------------------------------------------------------------------------------------------------------
#define FLAG_OK                         0x01
#define FLAG_KO                         0x00




// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_shalbase_storage_init(const eEstorageType_t strgtype);
static void s_shalbase_jump_to(uint32_t appaddr);

static void s_shalbase_permanent_boardinfo_init(void);
static baseBoardInfo_t* s_shalbase_permanent_boardinfo_get(void);
static void s_shalbase_permanent_boardinfo_set(baseBoardInfo_t *baseboardinfo);
static void s_shalbase_permanent_boardinfo_cache_invalidate(void);



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


static volatile baseInfo_t s_shalbase_ram_baseinfo      __attribute__((at(SHALBASE_RAMADDR)));

static volatile baseBoardInfo_t s_shalbase_temporary_baseboardinfo  __attribute__((at(SHALBASE_RAMADDR+sizeof(baseInfo_t))));

#if defined(SHALBASE_MODE_STATICLIBRARY) || defined(SHALS_MODE_STATIC)
static const eEmoduleInfo_t s_shalbase_moduleinfo =
#else
static const eEmoduleInfo_t s_shalbase_moduleinfo __attribute__((at(SHALBASE_ROMADDR+EENV_MODULEINFO_OFFSET))) =  
#endif
{
    .info           =
    {
        .entity     =
        {
            .type       = ee_entity_sharlib,
            .signature  = ee_shalBASE,
            .version    = 
            { 
                .major = SHALBASE_VER_MAJOR, 
                .minor = SHALBASE_VER_MINOR
            },  
            .builddate  = 
            {
                .year  = SHALBASE_BUILDDATE_YEAR,
                .month = SHALBASE_BUILDDATE_MONTH,
                .day   = SHALBASE_BUILDDATE_DAY,
                .hour  = SHALBASE_BUILDDATE_HOUR,
                .min   = SHALBASE_BUILDDATE_MIN
            }
        },
        .rom        = 
        {   
            .addr   = SHALBASE_ROMADDR,
            .size   = SHALBASE_ROMSIZE
        },
        .ram        = 
        {   
            .addr   = SHALBASE_RAMADDR,
            .size   = SHALBASE_RAMSIZE
        },
        .storage    = 
        {
            .type   = SHALBASE_STGTYPE,
            .size   = SHALBASE_STGSIZE,
            .addr   = SHALBASE_STGADDR
        },
        .communication  = ee_commtype_none,
        .name           = SHALBASE_NAME
    },
    .protocols  =
    {
        .udpprotversion  = { .major = 0, .minor = 0},
        .can1protversion = { .major = 0, .minor = 0},
        .can2protversion = { .major = 0, .minor = 0},
        .gtwprotversion  = { .major = 0, .minor = 0}
    },
    .extra      = {0}
};




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

#if defined(SHALBASE_MODE_STATICLIBRARY) || defined(SHALS_MODE_STATIC)

extern const eEmoduleInfo_t * shalbase_moduleinfo_get(void)
{
    return((const eEmoduleInfo_t*)&s_shalbase_moduleinfo);
}

extern const eEentity_t * shalbase_moduleinfo_entity_get(void)
{
    return((const eEentity_t*)&s_shalbase_moduleinfo.info.entity);
}

extern eEresult_t shalbase_isvalid(void)
{
    return(ee_res_OK);
}

#endif


extern eEresult_t shalbase_init(uint8_t forcestorageinit)
{
    // this function can be called multiple times by any e-process or e-sharlib and should be executed only once,
    // thus i use this guard ...   
    if(0 == memcmp((void*)&s_shalbase_ram_baseinfo.head.entity, (void*)&s_shalbase_moduleinfo, sizeof(eEentity_t)))
    {   // the base signature is ok, thus we have already initted the shalBASE
        if(0 == forcestorageinit)
        {
            return(ee_res_OK);
        }
    }
    
    // set the base signature
    memcpy((void*)&s_shalbase_ram_baseinfo.head.entity, (void*)&s_shalbase_moduleinfo, sizeof(eEentity_t));
    
    // set the remaining of the s_shalbase_ram_baseinfo.head
    s_shalbase_ram_baseinfo.head.defflag        = 0;
    s_shalbase_ram_baseinfo.head.cached         = 0;
    s_shalbase_ram_baseinfo.head.strginitted    = 1;
    s_shalbase_ram_baseinfo.head.forfutureuse1  = 0;

    // dont ever set the s_shalbase_ram_baseinfo.data
//    shalbase_ipc_gotoproc_clr();
//    shalbase_ipc_volatiledata_clr();
    
    // initialise the eeprom storage
    s_shalbase_storage_init(ee_strg_eeprom);
    // initialise the flash storage
    s_shalbase_storage_init(ee_strg_eflash);
    
    // if we have the sharPART and it does not hold info about this shalBASE shared library, add it
//    if(ee_res_OK == shalpart_isvalid())
//    {
//        shalpart_init();
//        shalpart_shal_synchronise(ee_shalBASE, &s_shalbase_moduleinfo);
//    }


    // initialise the management of boardinfo
    s_shalbase_permanent_boardinfo_init();
    s_shalbase_permanent_boardinfo_cache_invalidate();

    // ok, return
    return(ee_res_OK);
}

extern eEresult_t shalbase_deinit(void)
{
    // deinit the storage

    // finally, ... invalidate the ram
    memset((void*)&s_shalbase_ram_baseinfo.head, 0, sizeof(baseHead_t));
    
    return(ee_res_OK);
}


extern eEresult_t shalbase_boardinfo_synchronise(const eEboardInfo_t* boardinfo)
{
    baseBoardInfo_t* baseboardinfo = s_shalbase_permanent_boardinfo_get();

    if(0 != memcmp(&baseboardinfo->boardinfo, boardinfo, sizeof(eEboardInfo_t)))
    {
        memcpy(&baseboardinfo->boardinfo, boardinfo, sizeof(eEboardInfo_t));
        s_shalbase_permanent_boardinfo_set(baseboardinfo); 
    }

    // retrieve the eEboardInfo_t stored in storage. compare it w/ boardinfo. if different, then write boardinfo in storage
    return(ee_res_OK);
}

extern eEresult_t shalbase_boardinfo_get(const eEboardInfo_t** boardinfo)
{
    baseBoardInfo_t* baseboardinfo = s_shalbase_permanent_boardinfo_get();

    *boardinfo = &baseboardinfo->boardinfo;

    // copy the eEboardInfo_t stored in storage to the local ram, and give back its pointer
    return(ee_res_OK);
}

extern eEresult_t shalbase_ipc_gotoproc_get(eEprocess_t *pr)
{
    // the base signature is used to validate the ram
    if(0 != memcmp((void*)&s_shalbase_ram_baseinfo.head.entity, (void*)&s_shalbase_moduleinfo, sizeof(eEentity_t)))
    {
        return(ee_res_NOK_generic); 
    }
    
    if(FLAG_OK != s_shalbase_ram_baseinfo.data.gotoflag)
    {
        return(ee_res_NOK_generic);
    }
    else
    {
        *pr = (s_shalbase_ram_baseinfo.data.gotoval);
        return(ee_res_OK);
    }

}

extern eEresult_t shalbase_ipc_gotoproc_set(eEprocess_t pr)
{
    memcpy((void*)&s_shalbase_ram_baseinfo.head.entity, (void*)&s_shalbase_moduleinfo, sizeof(eEentity_t));
    s_shalbase_ram_baseinfo.data.gotoflag  = FLAG_OK;
    s_shalbase_ram_baseinfo.data.gotoval   = pr;
    
    return(ee_res_OK);
}


extern eEresult_t shalbase_ipc_gotoproc_clr(void)
{
    memcpy((void*)&s_shalbase_ram_baseinfo.head.entity, (void*)&s_shalbase_moduleinfo, sizeof(eEentity_t));
    s_shalbase_ram_baseinfo.data.gotoflag  = FLAG_KO;
    s_shalbase_ram_baseinfo.data.gotoval   = ee_procNone;
    
    return(ee_res_OK);
}


extern eEresult_t shalbase_ipc_volatiledata_get(uint8_t *data, uint8_t *size, const uint8_t maxsize)
{

    if(0 != memcmp((void*)&s_shalbase_ram_baseinfo.head.entity, (void*)&s_shalbase_moduleinfo, sizeof(eEentity_t)))
    {
        return(ee_res_NOK_generic); 
    }
    
    if(FLAG_OK != s_shalbase_ram_baseinfo.data.free2useflag)
    {
        return(ee_res_NOK_generic);
    }
    else
    {
        *size = (maxsize > s_shalbase_ram_baseinfo.data.free2usesize) ? (s_shalbase_ram_baseinfo.data.free2usesize) : (maxsize);
        memcpy(data, (uint8_t *)s_shalbase_ram_baseinfo.data.free2usedata, *size);
        return(ee_res_OK);
    }
}	

	
extern eEresult_t shalbase_ipc_volatiledata_set(uint8_t *data, uint8_t size)
{
    memcpy((void*)&s_shalbase_ram_baseinfo.head.entity, (void*)&s_shalbase_moduleinfo, sizeof(eEentity_t));
    s_shalbase_ram_baseinfo.data.free2useflag         = FLAG_OK;
    s_shalbase_ram_baseinfo.data.free2usesize         = (size < SHALBASE_F2USESIZE) ? (size) : (SHALBASE_F2USESIZE);
    memcpy((void*)s_shalbase_ram_baseinfo.data.free2usedata, data, s_shalbase_ram_baseinfo.data.free2usesize); 
    
    return(ee_res_OK);
}


extern eEresult_t shalbase_ipc_volatiledata_clr(void)
{
    memcpy((void*)&s_shalbase_ram_baseinfo.head.entity, (void*)&s_shalbase_moduleinfo, sizeof(eEentity_t));
    s_shalbase_ram_baseinfo.data.free2useflag  = FLAG_KO;
    s_shalbase_ram_baseinfo.data.free2usesize  = 0;
    memset((void*)s_shalbase_ram_baseinfo.data.free2usedata, 0, SHALBASE_F2USESIZE);
    
    return(ee_res_OK);
}


extern eEresult_t shalbase_system_canjump(uint32_t addr)
{
    return((hal_res_OK == hal_sys_canexecuteataddress(addr)) ? (ee_res_OK) : (ee_res_NOK_generic));
}

extern eEresult_t shalbase_system_canjump_to_proc(uint32_t addr, eEmoduleInfo_t *procinfo)
{
    // test if the proc described by procinfo is at address addr
    volatile eEmoduleInfo_t *mi = (volatile eEmoduleInfo_t*)(addr+EENV_MODULEINFO_OFFSET);
//    volatile eEentity_t *en = (eEentity_t*)mi;
    
    if(ee_res_NOK_generic == shalbase_system_canjump(addr))
    {
        return(ee_res_NOK_generic);
    }
    
    if(NULL == procinfo)
    {   // verify only if the module info placed in rom is coherent for an eprocess 
        if((ee_entity_process != mi->info.entity.type) || (addr != mi->info.rom.addr))
        {
            return(ee_res_NOK_generic);
        }
    }
    else if(0 != memcmp((void*)procinfo, (void*)mi, sizeof(eEmoduleInfo_t)))
    {   // we have a procinfo, thus we verify the full integrity
        return(ee_res_NOK_generic); 
    }  

    return(ee_res_OK);    
}

extern eEresult_t shalbase_system_jumpnow(uint32_t addr)
{

    if(ee_res_NOK_generic == shalbase_system_canjump(addr))
    {
        return(ee_res_NOK_generic);
    }
    
    // jump
    s_shalbase_jump_to(addr);
    
    // it never returns
    return(ee_res_NOK_generic);    
}

extern eEresult_t shalbase_system_restart(void)
{
    hal_sys_irq_disable();
    hal_sys_systemreset();

    // it never returns
    return(ee_res_NOK_generic);  
}



extern eEresult_t shalbase_storage_get(const eEstorage_t *strg, void *data, uint32_t size)
{
    volatile hal_result_t res = hal_res_NOK_generic;

    if(0 != memcmp((void*)&s_shalbase_ram_baseinfo.head.entity, (void*)&s_shalbase_moduleinfo, sizeof(eEentity_t)))
    {
        return(ee_res_NOK_generic); 
    }

    if(NULL == strg)
    {
        return(ee_res_NOK_generic);
    }
    
    if((size > strg->size) || (ee_strg_none == strg->type))
    {
        return(ee_res_NOK_generic);
    }
    
    
    if(ee_strg_eflash == strg->type)
    {
#ifndef EECOMMON_DONTUSESTRGFLASH
        // just read from flash
        memcpy(data, (const void*)strg->addr, size);
#else
        return(ee_res_NOK_generic);
#endif
    }
    else if(ee_strg_eeprom == strg->type)
    {
        res = hal_eeprom_read(hal_eeprom_i2c_01, strg->addr, size, data);
        res =  res;
    }


    return(ee_res_OK);  
}


extern eEresult_t shalbase_storage_set(const eEstorage_t *strg, const void *data, uint32_t size)
{
    volatile hal_result_t res = hal_res_NOK_generic;

    if(0 != memcmp((void*)&s_shalbase_ram_baseinfo.head.entity, (void*)&s_shalbase_moduleinfo, sizeof(eEentity_t)))
    {
        return(ee_res_NOK_generic); 
    }
    
    
    if(NULL == strg)
    {
        return(ee_res_NOK_generic);
    }
    
    if((size > strg->size) || (ee_strg_none == strg->type))
    {
        return(ee_res_NOK_generic);
    }

    if(ee_strg_eflash == strg->type)
    {
#ifndef EECOMMON_DONTUSESTRGFLASH
        hal_flash_erase(strg->addr, 2048);
        hal_flash_write(strg->addr, size, (void*)data);
#endif
    }
    else if(ee_strg_eeprom == strg->type)
    {
        res = hal_eeprom_write(hal_eeprom_i2c_01, strg->addr, size, (void*)data);
        res =  res;
    } 
    
    return(ee_res_OK);       
}

extern eEresult_t shalbase_storage_clr(const eEstorage_t *strg, const uint32_t size)
{
    volatile hal_result_t res = hal_res_NOK_generic;

    if(0 != memcmp((void*)&s_shalbase_ram_baseinfo.head.entity, (void*)&s_shalbase_moduleinfo, sizeof(eEentity_t)))
    {
        return(ee_res_NOK_generic); 
    }

    if(NULL == strg)
    {
        return(ee_res_NOK_generic);
    }
    
    if((size > strg->size) || (ee_strg_none == strg->type))
    {
        return(ee_res_NOK_generic);
    }

    if(ee_strg_eflash == strg->type)
    {
#ifndef EECOMMON_DONTUSESTRGFLASH
        hal_flash_erase(strg->addr, 2048);
#endif
    }
    else if(ee_strg_eeprom == strg->type)
    {
        //#warning add hal_eeprom_clear
        res = hal_eeprom_erase(hal_eeprom_i2c_01, strg->addr, size);
        res =  res;
    }    

    return(ee_res_OK); 
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

extern void shalbase_entrypoint(void)
{
    shalbase_init(0);
    shalbase_deinit();

    shalbase_ipc_gotoproc_get(NULL);			
    shalbase_ipc_gotoproc_set((eEprocess_t)0);
    shalbase_ipc_gotoproc_clr();

    shalbase_ipc_volatiledata_get(NULL, NULL, 0);		
    shalbase_ipc_volatiledata_set(NULL, 0);
    shalbase_ipc_volatiledata_clr();

    shalbase_system_canjump(0);
    shalbase_system_canjump_to_proc(0, NULL);
    shalbase_system_jumpnow(0);
    shalbase_system_restart();


//    shalbase_storage_init((eEstorageType_t)0);
//    shalbase_storage_deinit((eEstorageType_t)0);

    shalbase_storage_get(NULL, NULL, 0);
    shalbase_storage_set(NULL, NULL, 0);
    shalbase_storage_clr(NULL, 0);

}



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static void s_shalbase_jump_to(uint32_t appaddr)
{
    hal_sys_executenowataddress(appaddr);
}


static void s_shalbase_storage_init(const eEstorageType_t strgtype)
{
    volatile hal_result_t res = hal_res_NOK_generic;

    hal_sys_irq_disable();
    
    if(ee_strg_eflash == strgtype)
    {
#ifndef EECOMMON_DONTUSESTRGFLASH
        hal_flash_unlock();
#endif
    }
    else if(ee_strg_eeprom == strgtype)
    {
        res = hal_eeprom_init(hal_eeprom_i2c_01, NULL);
        res =  res;
    }
    
    hal_sys_irq_enable();
}






static void s_shalbase_permanent_boardinfo_init(void)
{
    memset((void*)&s_shalbase_temporary_baseboardinfo, 0, sizeof(s_shalbase_temporary_baseboardinfo));
}

static baseBoardInfo_t* s_shalbase_permanent_boardinfo_get(void)
{
    if(0 == s_shalbase_temporary_baseboardinfo.head.cached)
    {
        shalbase_storage_get(&s_shalbase_moduleinfo.info.storage, (void*)&s_shalbase_temporary_baseboardinfo, sizeof(baseBoardInfo_t));
    }

    s_shalbase_temporary_baseboardinfo.head.cached = 1;

    return((baseBoardInfo_t*)&s_shalbase_temporary_baseboardinfo);
}


static void s_shalbase_permanent_boardinfo_set(baseBoardInfo_t *baseboardinfo)
{
    baseboardinfo->head.cached = 0;
    shalbase_storage_set(&s_shalbase_moduleinfo.info.storage, baseboardinfo, sizeof(baseBoardInfo_t));
}

static void s_shalbase_permanent_boardinfo_cache_invalidate(void)
{
    s_shalbase_temporary_baseboardinfo.head.cached = 0;
}






// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------

