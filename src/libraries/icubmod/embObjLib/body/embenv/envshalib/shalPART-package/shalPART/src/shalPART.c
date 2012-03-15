
/* @file       shalPART.c
    @brief      This header file implements the shalPART shared library.
    @author     marco.accame@iit.it
    @date       07/05/2011
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "string.h"

#include "shalBASE.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "shalPART.h"


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


typedef struct              // 16B                                  
{
    eEentity_t              entity;                 // 8B
    uint8_t                 defflag;                // 1B
    uint8_t                 cached;                 // 1B
    uint8_t                 forfutureuse0;          // 1B
    uint8_t                 forfutureuse1;          // 1B
    uint8_t                 filler[4];              // 4B
} partHead_t;               EECOMMON_VERIFYsizeof(partHead_t, 16);



typedef struct              // numer of bytes depends on many things. it is 984 under following conditions
{                           // 5+ee_procMaxNum(5)+ee_shalMaxNum(10)+ee_procMaxNum*64+ee_shalMaxNum*64
    uint8_t                 NprocMax;                   // constant and fixed to 10 ...  as it is size of array inside
    uint8_t                 NshalMax;                   // constant and fixed to 10 ...  as it is size of array inside 
    uint8_t                 Nproc;                      // can be changed in runtime if a new partition w/ a proc is added
    uint8_t                 Nshal;                      // can be changed in runtime if a new partition w/ a shal is added
    eEprocess_t             defProc2run;                // it is the default process the eLoader runs (if no other info)
    eEprocess_t             tableprocs[ee_procMaxNum];  // contains list of Nproc procs
    eEsharlib_t             tableshals[ee_shalMaxNum];  // contains list of Nshar shals
    uint8_t                 filler4[4];                 // to align to multiple of 8
    eEmoduleInfo_t          procInfo[ee_procMaxNum];    // the process info
    eEmoduleInfo_t          shalInfo[ee_shalMaxNum];    // the shallib info
} partData_t;               EECOMMON_VERIFYsizeof(partData_t, 984);


typedef struct              // 1000B 
{
    partHead_t              head;   // 16B
    partData_t              data;   // 984B
} partInfo_t;               EECOMMON_VERIFYsizeof(partInfo_t, 1000);



// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

#define SHALPART_ROMADDR            (EENV_MEMMAP_SHALPART_ROMADDR)
#define SHALPART_ROMSIZE            (EENV_MEMMAP_SHALPART_ROMSIZE)

#define SHALPART_RAMADDR            (EENV_MEMMAP_SHALPART_RAMADDR)
#define SHALPART_RAMSIZE            (EENV_MEMMAP_SHALPART_RAMSIZE)

#define SHALPART_STGTYPE            (ee_strg_eeprom)
#define SHALPART_STGADDR            (EENV_MEMMAP_SHALPART_STGADDR)
#define SHALPART_STGSIZE            (EENV_MEMMAP_SHALPART_STGSIZE)


// the ram size to be used in scatter-file and the one used by the program for static ram
#define SHALPART_RAMFOR_RWDATA      (EENV_MEMMAP_SHALPART_RAMFOR_RWDATA) 

// the ram size to be used with __attribute__((at(SHALPART_RAMADDR))), which is sizeof(partInfo_t)
#define SHALPART_RAMFOR_ZIDATA      (EENV_MEMMAP_SHALPART_RAMFOR_ZIDATA)


typedef int dummy1[sizeof(partInfo_t)     <= (SHALPART_RAMSIZE) ? 1 : -1];
typedef int dummy2[SHALPART_RAMFOR_ZIDATA <= ((SHALPART_RAMSIZE-SHALPART_RAMFOR_RWDATA)) ? 1 : -1];



//#define VOID_ADDR                          0xFFFFFFFF
//#define VOID_SIGN                          255

//#define VOID_MODINFO     {.type = ee_none, .signature = VOID_SIGN, .version = {.major=0, .minor=0}, .builddate = {0}, .romaddr = VOID_ADDR, .romsize = 0, .storage = {0}, .name = "", .communication = {0}}


#define DEFFLAG_TRUE                          0x01
#define DEFFLAG_FALSE                         0x00




// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static eEresult_t s_itisinside_tableprocs(partInfo_t *partinfo, eEprocess_t proc);
static void s_update_tableprocs(partInfo_t *partinfo);
static eEresult_t s_itisinside_tableshals(partInfo_t *partinfo, eEsharlib_t shal);
static void s_update_tableshals(partInfo_t *partinfo);

static void s_shalpart_permanent_partinfo_init(void);
static void s_shalpart_permanent_partinfo_reset(partInfo_t *partinfo);
static void s_shalpart_permanent_partinfo_cache_invalidate(void);
static partInfo_t* s_shalpart_permanent_partinfo_get(void);
static void s_shalpart_permanent_partinfo_set(partInfo_t *partinfo);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


// - default values ---------------------------------------------------------------------------------------------------

//#if 0 // we cannot afford such a big constant in flash space
//static const partInfo_t s_shalpart_default_partinfo = 
//{ 
//    .head = 
//    {
//        .version = 
//        {
//            .major = SHALPART_MAJOR,
//            .minor = SHALPART_MINOR
//        }, 
//        .defflag = DEFFLAG_TRUE,
//        .cached  = 0
//    },
//    .data = 
//    { 
//        .NprocMax       = ee_procMaxNum,
//        .NshalMax       = ee_shalMaxNum,
//        .Nproc          = 0,
//        .Nshal          = 0,
//        .defProc2run    = ee_procNone,
//        .tableprocs     =
//        {
//            ee_procNone, ee_procNone, ee_procNone, ee_procNone, ee_procNone,
//            ee_procNone, ee_procNone, ee_procNone, ee_procNone, ee_procNone
//        }, 
//        .tableshals     =
//        {   
//            ee_shalNone, ee_shalNone, ee_shalNone, ee_shalNone, ee_shalNone,
//            ee_shalNone, ee_shalNone, ee_shalNone, ee_shalNone, ee_shalNone
//        },
//        .procInfo       =
//        {
//            VOID_MODINFO, VOID_MODINFO, VOID_MODINFO, VOID_MODINFO, VOID_MODINFO,
//            VOID_MODINFO, VOID_MODINFO, VOID_MODINFO, VOID_MODINFO, VOID_MODINFO
//        },
//        .shalInfo       =
//        {
//            VOID_MODINFO, VOID_MODINFO, VOID_MODINFO, VOID_MODINFO, VOID_MODINFO,
//            VOID_MODINFO, VOID_MODINFO, VOID_MODINFO, VOID_MODINFO, VOID_MODINFO
//
//        }
//    }
//};
//#endif

//static const eEmoduleInfo_t s_shalpart_voidmodinfo = VOID_MODINFO;

// we dont need to place the variable in here. just the SHALPART_STGADDR is required ...
//static const partInfo_t    s_shalpart_permflash_partinfo __attribute__((at(SHALPART_STGADDR))) = { 0 }; 



static volatile partInfo_t s_shalpart_temporary_partinfo __attribute__((at(SHALPART_RAMADDR))); 


#if defined(SHALPART_MODE_STATICLIBRARY) || defined(SHALS_MODE_STATIC)
static const eEmoduleInfo_t s_shalpart_moduleinfo =
#else
static const eEmoduleInfo_t s_shalpart_moduleinfo __attribute__((at(SHALPART_ROMADDR+EENV_MODULEINFO_OFFSET))) = 
#endif
{
    .info           =
    {
        .entity     =
        {
            .type       = ee_entity_sharlib,
            .signature  = ee_shalPART,
            .version    = 
            { 
                .major = SHALPART_VER_MAJOR, 
                .minor = SHALPART_VER_MINOR
            },  
            .builddate  = 
            {
                .year  = SHALPART_BUILDDATE_YEAR,
                .month = SHALPART_BUILDDATE_MONTH,
                .day   = SHALPART_BUILDDATE_DAY,
                .hour  = SHALPART_BUILDDATE_HOUR,
                .min   = SHALPART_BUILDDATE_MIN
            }
        },
        .rom        = 
        {   
            .addr   = SHALPART_ROMADDR,
            .size   = SHALPART_ROMSIZE
        },
        .ram        = 
        {   
            .addr   = SHALPART_RAMADDR,
            .size   = SHALPART_RAMSIZE
        },
        .storage    = 
        {
            .type   = SHALPART_STGTYPE,
            .size   = SHALPART_STGSIZE,
            .addr   = SHALPART_STGADDR
        },
        .communication  = ee_commtype_none,
        .name           = SHALPART_NAME
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


#if defined(SHALPART_MODE_STATICLIBRARY) || defined(SHALS_MODE_STATIC)

extern const eEmoduleInfo_t * shalpart_moduleinfo_get(void)
{
    return((const eEmoduleInfo_t*)&s_shalpart_moduleinfo);
}

extern const eEentity_t * shalpart_moduleinfo_entity_get(void)
{
    return((const eEentity_t*)&s_shalpart_moduleinfo.info.entity);
}

extern eEresult_t shalpart_isvalid(void)
{
    return(ee_res_OK);
}

#endif


// acemorhasverified
extern eEresult_t shalpart_init(void)
{
    partInfo_t  * volatile partinfo = NULL;

    // in the beginning ... the base

    if(ee_res_NOK_generic == shalbase_isvalid())
    {
        return(ee_res_NOK_generic);
    }

    shalbase_init(0);

    // then the rest 

    s_shalpart_permanent_partinfo_init();

    s_shalpart_permanent_partinfo_cache_invalidate();

    partinfo = s_shalpart_permanent_partinfo_get();

    if((s_shalpart_moduleinfo.info.entity.type != partinfo->head.entity.type) && (s_shalpart_moduleinfo.info.entity.signature != partinfo->head.entity.signature))
    {
        // first time we run ... need to write the default data into permanent data
        s_shalpart_permanent_partinfo_reset(partinfo);
        //s_shalpart_permanent_partinfo_set((partInfo_t*)&s_shalpart_default_partinfo);
    }

    if(SHALPART_VER_MAJOR != partinfo->head.entity.version.major)
    {
        // the rom data has surely been externally manipulated (maybe updated by the updater)
        // and is not synchronised with the manipulating code .... what to do? decide it.
        //ee_common_onerror(ee_shalPART);
    }


    // now i synchronise the module info of shalBASE
    shalpart_shal_synchronise(ee_shalBASE, shalbase_moduleinfo_get());
        
    // now i synchronise the module info of shalPART
    shalpart_shal_synchronise(ee_shalPART, &s_shalpart_moduleinfo);


    return(ee_res_OK);
}

extern eEresult_t shalpart_reset(shalpart_reset_mode_t rm)
{
    if(shalpart_reset_rawvals == rm)
    {
        memset((void*)&s_shalpart_temporary_partinfo, 0xff, sizeof(partInfo_t));
        shalbase_storage_set(&s_shalpart_moduleinfo.info.storage, (void*)&s_shalpart_temporary_partinfo, sizeof(partInfo_t));
        //ee_common_permanent_data_set(&s_shalpart_moduleinfo.info.storage, (void*)&s_shalpart_temporary_partinfo, sizeof(partInfo_t));
    }
    else
    {  
        s_shalpart_permanent_partinfo_reset((partInfo_t*)&s_shalpart_temporary_partinfo);
    }

    return(ee_res_OK);
}

extern eEresult_t shalpart_deinit(void)
{
    // so far we dont need to undo anything. whenever we support eeprom (on i2c) we need to ....
    return(ee_res_OK);
}


//acemorhasverified
extern eEresult_t shalpart_proc_synchronise(eEprocess_t proc, const eEmoduleInfo_t *moduleinfo)
{
    partInfo_t  * volatile partinfo = s_shalpart_permanent_partinfo_get();

    if(ee_res_NOK_generic == s_itisinside_tableprocs(partinfo, proc))
    { 
        // add it
        shalpart_proc_add(proc, (eEmoduleInfo_t *)moduleinfo);
    }
    else
    {
        // verify if it is equal. if not copy  
        if(0 != memcmp(&(partinfo->data.procInfo[proc]), moduleinfo, sizeof(eEmoduleInfo_t)))
        {
            shalpart_proc_set(proc, (eEmoduleInfo_t *)moduleinfo);
        }          
    }
    
    return(ee_res_OK);    
}

//acemorhasverified
extern eEresult_t shalpart_proc_def2run_get(eEprocess_t *proc)
{
    partInfo_t  * volatile partinfo = s_shalpart_permanent_partinfo_get();

    *proc = partinfo->data.defProc2run;

    if(ee_procNone == partinfo->data.defProc2run)
    {
        return(ee_res_NOK_generic);   
    }
    else
    {
   
        return(ee_res_OK);
    }
}

//acemorhasverified
extern eEresult_t shalpart_proc_def2run_set(eEprocess_t proc)
{
    partInfo_t  * volatile partinfo = s_shalpart_permanent_partinfo_get();

    if(partinfo->data.defProc2run != proc)
    {
        partinfo->data.defProc2run = proc;
        partinfo->head.defflag = DEFFLAG_FALSE;
        s_shalpart_permanent_partinfo_set(partinfo); // aka: s_program_page();
    }

    return(ee_res_OK);
}

//acemorhasverified
extern eEresult_t shalpart_proc_allavailable_get(const eEprocess_t **table, uint8_t *size)
{
    partInfo_t  * volatile partinfo = s_shalpart_permanent_partinfo_get();

    *table = partinfo->data.tableprocs;
    *size  = partinfo->data.Nproc;

    return(ee_res_OK);
}

//acemorhasverified
extern eEresult_t shalpart_proc_add(eEprocess_t proc, eEmoduleInfo_t *moduleinfo) 
{
    partInfo_t  * volatile partinfo = s_shalpart_permanent_partinfo_get();
 
    if((proc >= ee_procMaxNum) || (NULL == moduleinfo))
    {
        return(ee_res_NOK_generic);     
    }

    if((proc != moduleinfo->info.entity.signature) || (ee_entity_process != moduleinfo->info.entity.type))
    {
        return(ee_res_NOK_generic);
    }

    memcpy((void*)&(partinfo->data.procInfo[proc]), moduleinfo, sizeof(eEmoduleInfo_t));

    // 2. update the proc table
    s_update_tableprocs(partinfo);

    // 3. touch the default flag
    partinfo->head.defflag = DEFFLAG_FALSE;

    // 4. save in permanent memory
    s_shalpart_permanent_partinfo_set(partinfo); // aka s_program_page();


    return(ee_res_OK);

}

//acemorhasverified
extern eEresult_t shalpart_proc_rem(eEprocess_t proc)
{
    partInfo_t  * volatile partinfo = s_shalpart_permanent_partinfo_get();

    if(ee_res_NOK_generic == s_itisinside_tableprocs(partinfo, proc))
    {
        return(ee_res_NOK_generic);
    }

    // copy a null entry into proper place
    //memcpy((void*)&(partinfo->data.procInfo[proc]), (void*)&s_shalpart_voidmodinfo, sizeof(eEmoduleInfo_t));
    memset((void*)&(partinfo->data.procInfo[proc]), 0, sizeof(eEmoduleInfo_t));

    // update the proc table
    s_update_tableprocs(partinfo);

    // if i remove the proc2run ...
    if(proc == partinfo->data.defProc2run)
    {
        partinfo->data.defProc2run = ee_procNone;
    }

    // touch the default flag
    partinfo->head.defflag = DEFFLAG_FALSE;

    // copy into flash
    s_shalpart_permanent_partinfo_set(partinfo); // aka: s_program_page();

    return(ee_res_OK);
}

//acemorhasverified
extern eEresult_t shalpart_proc_set(eEprocess_t proc, eEmoduleInfo_t *moduleinfo) 
{
    partInfo_t  * volatile partinfo = s_shalpart_permanent_partinfo_get();

    if((ee_res_NOK_generic == s_itisinside_tableprocs(partinfo, proc)) || (NULL == moduleinfo))
    {
        return(ee_res_NOK_generic);
    }

    if((proc != moduleinfo->info.entity.signature) || (ee_entity_process != moduleinfo->info.entity.type))
    {
        return(ee_res_NOK_generic);
    } 
    
    // copy entry into proper place
    memcpy((void*)&(partinfo->data.procInfo[proc]), moduleinfo, sizeof(eEmoduleInfo_t));

    // 2. update the proc table: no need if it is already inside
    // s_update_tableprocs();

    // 3. touch the default flag
    partinfo->head.defflag = DEFFLAG_FALSE;

    // 4. copy into flash
    s_shalpart_permanent_partinfo_set(partinfo); // aka:    s_program_page();


    return(ee_res_OK);

}

//acemorhasverified
extern eEresult_t shalpart_proc_get(eEprocess_t proc, const eEmoduleInfo_t **moduleinfo)
{
    partInfo_t  * volatile partinfo = s_shalpart_permanent_partinfo_get();

    if(ee_res_NOK_generic == s_itisinside_tableprocs(partinfo, proc))
    {
        return(ee_res_NOK_generic);
    }


    *moduleinfo = &partinfo->data.procInfo[proc];

    return(ee_res_OK);
}

// acemorhasverified
extern eEresult_t shalpart_proc_runaddress_get(eEprocess_t proc, uint32_t *addr)
{
    partInfo_t  * volatile partinfo = s_shalpart_permanent_partinfo_get();

    if(ee_res_NOK_generic == s_itisinside_tableprocs(partinfo, proc))
    {
        return(ee_res_NOK_generic);
    }

   
    *addr = partinfo->data.procInfo[proc].info.rom.addr;

    return(ee_res_OK);
}



// acemorhasverified
extern eEresult_t shalpart_shal_synchronise(eEsharlib_t shal, const eEmoduleInfo_t *moduleinfo)
{
    partInfo_t  * volatile partinfo = s_shalpart_permanent_partinfo_get();
    

    if(ee_res_NOK_generic == s_itisinside_tableshals(partinfo, shal))
    { 
        // add it
        shalpart_shal_add(shal, (eEmoduleInfo_t *)moduleinfo);
    }
    else
    {
        // verify if it is equal. if not copy  
        if(0 != memcmp(&(partinfo->data.shalInfo[shal]), moduleinfo, sizeof(eEmoduleInfo_t)))
        {
            shalpart_shal_set(shal, (eEmoduleInfo_t *)moduleinfo);
        }          
    }
                            
    return(ee_res_OK);    
}



//acemorhasverified
extern eEresult_t shalpart_shal_allavailable_get(const eEsharlib_t **table, uint8_t *size)
{
    partInfo_t  * volatile partinfo = s_shalpart_permanent_partinfo_get();

    *table = partinfo->data.tableshals;
    *size  = partinfo->data.Nshal;

    return(ee_res_OK);
}

//acemorhasverified
extern eEresult_t shalpart_shal_add(eEsharlib_t shal, eEmoduleInfo_t *moduleinfo)
{
    partInfo_t  * volatile partinfo = s_shalpart_permanent_partinfo_get();
 
    if((shal >= ee_shalMaxNum) || (NULL == moduleinfo))
    {
        return(ee_res_NOK_generic);     
    }

    if((shal != moduleinfo->info.entity.signature) || (ee_entity_sharlib != moduleinfo->info.entity.type))
    {
        return(ee_res_NOK_generic);
    }   
    
    memcpy((void*)&(partinfo->data.shalInfo[shal]), moduleinfo, sizeof(eEmoduleInfo_t));

    // 2. update the shal table
    s_update_tableshals(partinfo);

    // 3. touch the default flag
    partinfo->head.defflag = DEFFLAG_FALSE;

    // 4. copy into flash
    s_shalpart_permanent_partinfo_set(partinfo); // aka: s_program_page();


    return(ee_res_OK);

}

//acemorhasverified
extern eEresult_t shalpart_shal_rem(eEsharlib_t shal)
{
    partInfo_t  * volatile partinfo = s_shalpart_permanent_partinfo_get();

    if(ee_res_NOK_generic == s_itisinside_tableshals(partinfo, shal))
    {
        return(ee_res_NOK_generic);
    }

    // copy a null entry into proper place
    //memcpy((void*)&(partinfo->data.shalInfo[shal]), (void*)&s_shalpart_voidmodinfo, sizeof(eEmoduleInfo_t));
    memset((void*)&(partinfo->data.shalInfo[shal]), 0, sizeof(eEmoduleInfo_t));
    //memcpy((void*)&(partinfo->data.shalInfo[shal]), (void*)s_shalpart_voidmodinfo_ptr, sizeof(eEmoduleInfo_t));

    // update the shal table
    s_update_tableshals(partinfo);

    // touch the default flag
    partinfo->head.defflag = DEFFLAG_FALSE;

    // copy into flash
    s_shalpart_permanent_partinfo_set(partinfo); // aka:  s_program_page();

    return(ee_res_OK);
}


//acemorhasverified
extern eEresult_t shalpart_shal_set(eEsharlib_t shal, eEmoduleInfo_t *moduleinfo)
{
    partInfo_t  * volatile partinfo = s_shalpart_permanent_partinfo_get();

    if((ee_res_NOK_generic == s_itisinside_tableshals(partinfo, shal)) || (NULL == moduleinfo))
    {
        return(ee_res_NOK_generic);
    }

    if((shal != moduleinfo->info.entity.signature) || (ee_entity_sharlib != moduleinfo->info.entity.type))
    {
        return(ee_res_NOK_generic);
    } 

    // copy entry into proper place
    memcpy((void*)&(partinfo->data.shalInfo[shal]), moduleinfo, sizeof(eEmoduleInfo_t));

    // 2. update the shal table: no need as we have verified that shal is already inside.
    // s_update_tableshals();

    // 3. touch the default flag
    partinfo->head.defflag = DEFFLAG_FALSE;

    // 4. copy into permanent storage
    s_shalpart_permanent_partinfo_set(partinfo); // aka:  s_program_page();


    return(ee_res_OK);
}

//acemorhasverified
extern eEresult_t shalpart_shal_get(eEsharlib_t shal, const eEmoduleInfo_t **moduleinfo)
{
    partInfo_t  * volatile partinfo = s_shalpart_permanent_partinfo_get();

    if(ee_res_NOK_generic == s_itisinside_tableshals(partinfo, shal))
    {
        return(ee_res_NOK_generic);
    }

    *moduleinfo = &partinfo->data.shalInfo[shal];

    return(ee_res_OK);
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

extern void shalpart_hid_entrypoint(void)
{   
    shalpart_init();
    shalpart_reset((shalpart_reset_mode_t)0);
    shalpart_deinit();
    shalpart_proc_synchronise((eEprocess_t) 0, NULL);
    shalpart_proc_def2run_get(NULL);
    shalpart_proc_def2run_set((eEprocess_t) 0);
    shalpart_proc_runaddress_get((eEprocess_t)0, NULL);
    shalpart_proc_allavailable_get(NULL, NULL);
    shalpart_proc_add((eEprocess_t)0, NULL);
    shalpart_proc_rem((eEprocess_t)0);
    shalpart_proc_set((eEprocess_t)0, NULL);
    shalpart_proc_get((eEprocess_t)0, NULL);
    shalpart_shal_synchronise((eEsharlib_t)0, NULL);
    shalpart_shal_allavailable_get(NULL, NULL);
    shalpart_shal_add((eEsharlib_t)0, NULL);
    shalpart_shal_rem((eEsharlib_t)0);
    shalpart_shal_set((eEsharlib_t)0, NULL);
    shalpart_shal_get((eEsharlib_t)0, NULL);
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


//acemorhasverified
static eEresult_t s_itisinside_tableprocs(partInfo_t *partinfo, eEprocess_t proc)
{   
    uint8_t i;


    if(proc >= ee_procMaxNum)
    {
        return(ee_res_NOK_generic);
    }

    for(i=0; i<partinfo->data.Nproc; i++)
    {
        if(partinfo->data.tableprocs[i] == proc)
        {
            return(ee_res_OK);
        }
    }

    return(ee_res_NOK_generic);
}

//acemorhasverified
static void s_update_tableprocs(partInfo_t *partinfo)
{
    uint8_t i = 0;

    partinfo->data.Nproc = 0;

    for(i=0; i<ee_procMaxNum; i++)
    {
        partinfo->data.tableprocs[i] = ee_procNone;
    }

    for(i=0; i<ee_procMaxNum; i++)
    {
        if(ee_none != partinfo->data.procInfo[i].info.entity.type)
        {
            // found one
            partinfo->data.tableprocs[ partinfo->data.Nproc] = (eEprocess_t)i;
            partinfo->data.Nproc++; 
        }
    }
}

//acemorhasverified
static eEresult_t s_itisinside_tableshals(partInfo_t *partinfo, eEsharlib_t shal)
{   
    uint8_t i;

    if(shal >= ee_shalMaxNum)
    {
        return(ee_res_NOK_generic);
    }

    for(i=0; i<partinfo->data.Nshal; i++)
    {
        if(partinfo->data.tableshals[i] == shal)
        {
            return(ee_res_OK);
        }
    }

    return(ee_res_NOK_generic);
}

//acemorhasverified
static void s_update_tableshals(partInfo_t *partinfo)
{
    uint8_t i = 0;

    partinfo->data.Nshal = 0;

    for(i=0; i<ee_shalMaxNum; i++)
    {
        partinfo->data.tableshals[i] = ee_shalNone;
    }

    for(i=0; i<ee_shalMaxNum; i++)
    {
        if(ee_none != partinfo->data.shalInfo[i].info.entity.type)
        {
            // found one
            partinfo->data.tableshals[partinfo->data.Nshal] = (eEsharlib_t)i;
            partinfo->data.Nshal++; 
        }
    }
}


static partInfo_t* s_shalpart_permanent_partinfo_get(void)
{
    if(0 == s_shalpart_temporary_partinfo.head.cached)
    {
        shalbase_storage_get(&s_shalpart_moduleinfo.info.storage, (void*)&s_shalpart_temporary_partinfo, sizeof(partInfo_t));
        //ee_common_permanent_data_get(&s_shalpart_moduleinfo.storage, (void*)&s_shalpart_temporary_partinfo, sizeof(partInfo_t));
    }

    s_shalpart_temporary_partinfo.head.cached = 1;

    return((partInfo_t*)&s_shalpart_temporary_partinfo);
}

static void s_shalpart_permanent_partinfo_init(void)
{
    memset((void*)&s_shalpart_temporary_partinfo, 0, sizeof(s_shalpart_temporary_partinfo));
    // the storage in eeprom or flash is initted by shalBASE: shalbase_init()
    // shalbase_init();
}

static void s_shalpart_permanent_partinfo_cache_invalidate(void)
{
    s_shalpart_temporary_partinfo.head.cached = 0;
}

static void s_shalpart_permanent_partinfo_reset(partInfo_t *partinfo)
{
    uint8_t i = 0;

    partinfo->head.entity.type              = s_shalpart_moduleinfo.info.entity.type;
    partinfo->head.entity.signature         = s_shalpart_moduleinfo.info.entity.signature;
    partinfo->head.entity.version.major     = SHALPART_VER_MAJOR;
    partinfo->head.entity.version.minor     = SHALPART_VER_MINOR;
    partinfo->head.defflag                  = DEFFLAG_TRUE;
    partinfo->head.cached                   = 0;

    partinfo->data.NprocMax         = ee_procMaxNum;
    partinfo->data.NshalMax         = ee_shalMaxNum;
    partinfo->data.Nproc            = 0;
    partinfo->data.Nshal            = 0;
    partinfo->data.defProc2run      = ee_procNone;

    memset(partinfo->data.tableprocs, ee_procNone, sizeof(partinfo->data.tableprocs));
    memset(partinfo->data.tableshals, ee_procNone, sizeof(partinfo->data.tableshals));

    for(i=0; i<ee_procMaxNum; i++)
    {
        //memcpy(&partinfo->data.procInfo[i], &s_shalpart_voidmodinfo, sizeof(s_shalpart_voidmodinfo));
        memset(&partinfo->data.procInfo[i], 0, sizeof(eEmoduleInfo_t));
    }

    for(i=0; i<ee_shalMaxNum; i++)
    {
        //memcpy(&partinfo->data.shalInfo[i], &s_shalpart_voidmodinfo, sizeof(s_shalpart_voidmodinfo));
        memset(&partinfo->data.shalInfo[i], 0, sizeof(eEmoduleInfo_t));
    }

    s_shalpart_permanent_partinfo_set(partinfo);
}


static void s_shalpart_permanent_partinfo_set(partInfo_t *partinfo)
{
    partinfo->head.cached = 0;
    //ee_common_permanent_data_set(&s_shalpart_moduleinfo.info.storage, partinfo, sizeof(partInfo_t));
    shalbase_storage_set(&s_shalpart_moduleinfo.info.storage, partinfo, sizeof(partInfo_t));
}




// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------

