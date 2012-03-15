
/* @file       shalINFO.c
    @brief      This header file implements the shalINFO library.
    @author     marco.accame@iit.it
    @date       03/11/2011
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------



#include "string.h"

#include "shalBASE.h"
#include "shalPART.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "shalINFO.h"


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
} infoHead_t;               EECOMMON_VERIFYsizeof(infoHead_t, 16);



typedef struct              // 80B
{
    infoHead_t              head;                   // 16B
    eEboardInfo_t           boardinfo;              // 64B
} infoBoardInfo_t;          EECOMMON_VERIFYsizeof(infoBoardInfo_t, 80);



typedef struct              // 272B
{
    infoHead_t              head;                   // 16B
    shalinfo_deviceinfo_t   deviceinfo;             // 256B
} infoDeviceInfo_t;         EECOMMON_VERIFYsizeof(infoDeviceInfo_t, 272);




// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

#define SHALINFO_ROMADDR            (EENV_MEMMAP_SHALINFO_ROMADDR)
#define SHALINFO_ROMSIZE            (EENV_MEMMAP_SHALINFO_ROMSIZE)

#define SHALINFO_RAMADDR            (EENV_MEMMAP_SHALINFO_RAMADDR)
#define SHALINFO_RAMSIZE            (EENV_MEMMAP_SHALINFO_RAMSIZE)

#define SHALINFO_STGTYPE            (ee_strg_eeprom)
#define SHALINFO_STGADDR            (EENV_MEMMAP_SHALINFO_STGADDR)
#define SHALINFO_STGSIZE            (EENV_MEMMAP_SHALINFO_STGSIZE)

// the ram size to be used in scatter-file and the one used by the program for static ram
#define SHALINFO_RAMFOR_RWDATA      (EENV_MEMMAP_SHALINFO_RAMFOR_RWDATA) // of which only ?? used

// the ram size to be used with __attribute__((at(SHALINFO_RAMADDR))), which is sizeof(infoInfo_t)
#define SHALINFO_RAMFOR_ZIDATA      (EENV_MEMMAP_SHALINFO_RAMFOR_ZIDATA)


typedef int dummy1[(sizeof(infoBoardInfo_t)+sizeof(infoDeviceInfo_t))     <= (SHALINFO_RAMSIZE) ? 1 : -1];
typedef int dummy2[SHALINFO_RAMFOR_ZIDATA <= ((SHALINFO_RAMSIZE-SHALINFO_RAMFOR_RWDATA)) ? 1 : -1];


#define DEFFLAG_TRUE                          0x0001
#define DEFFLAG_FALSE                         0x0000




// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------


static void s_shalinfo_permanent_boardinfo_init(void);
static infoBoardInfo_t* s_shalinfo_permanent_boardinfo_get(void);
static void s_shalinfo_permanent_boardinfo_set(infoBoardInfo_t *infoboardinfo);
static void s_shalinfo_permanent_boardinfo_cache_invalidate(void);


static void s_shalinfo_permanent_deviceinfo_init(void);
static infoDeviceInfo_t* s_shalinfo_permanent_deviceinfo_get(void);
static void s_shalinfo_permanent_deviceinfo_set(infoDeviceInfo_t *infodeviceinfo);
static void s_shalinfo_permanent_deviceinfo_cache_invalidate(void);

static eEresult_t s_shalinfo_deviceinfo_ipnetwork_clr(void);

static eEresult_t s_shalinfo_deviceinfo_can1network_clr(void);
static eEresult_t s_shalinfo_deviceinfo_can2network_clr(void);

static void s_shalinfo_ipnetwork_default_set(eEipnetwork_t* ntw);
static void s_shalinfo_can1network_default_set(eEcannetwork_t* ntw);
static void s_shalinfo_can2network_default_set(eEcannetwork_t* ntw);
static void s_shalinfo_pages_default_set(void *startofpages);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


const eEstorage_t s_shalinfo_strg_boardinfo =
{
    .type   = SHALINFO_STGTYPE,
    .size   = sizeof(infoBoardInfo_t),
    .addr   = SHALINFO_STGADDR    
};

const eEstorage_t s_shalinfo_strg_deviceinfo =
{
    .type   = SHALINFO_STGTYPE,
    .size   = sizeof(infoDeviceInfo_t),
    .addr   = SHALINFO_STGADDR + sizeof(infoBoardInfo_t)    
};


// - default values ---------------------------------------------------------------------------------------------------

static const infoHead_t s_shalinfo_default_infohead =
{
    .entity     =
    {
        .type       = ee_entity_sharlib,
        .signature  = ee_shalINFO,
        .version    = 
        { 
            .major = SHALINFO_VER_MAJOR, 
            .minor = SHALINFO_VER_MINOR
        },  
        .builddate  = 
        {
            .year  = SHALINFO_BUILDDATE_YEAR,
            .month = SHALINFO_BUILDDATE_MONTH,
            .day   = SHALINFO_BUILDDATE_DAY,
            .hour  = SHALINFO_BUILDDATE_HOUR,
            .min   = SHALINFO_BUILDDATE_MIN
        }
    },
    .defflag    = 1,
    .cached     = 0,
    .forfutureuse0 = 0,
    .forfutureuse1 = 0
};



// - volatile values --------------------------------------------------------------------------------------------------

static volatile infoBoardInfo_t s_shalinfo_temporary_infoboardinfo  __attribute__((at(SHALINFO_RAMADDR)));

static volatile infoDeviceInfo_t s_shalinfo_temporary_infodeviceinfo  __attribute__((at(SHALINFO_RAMADDR+sizeof(infoBoardInfo_t))));


// - module info ------------------------------------------------------------------------------------------------------

#if defined(SHALINFO_MODE_STATICLIBRARY) || defined(SHALS_MODE_STATIC)
static const eEmoduleInfo_t s_shalinfo_moduleinfo = 
#else
static const eEmoduleInfo_t s_shalinfo_moduleinfo __attribute__((at(SHALINFO_ROMADDR+EENV_MODULEINFO_OFFSET))) = 
#endif
{
    .info           =
    {
        .entity     =
        {
            .type       = ee_entity_sharlib,
            .signature  = ee_shalINFO,
            .version    = 
            { 
                .major = SHALINFO_VER_MAJOR, 
                .minor = SHALINFO_VER_MINOR
            },  
            .builddate  = 
            {
                .year  = SHALINFO_BUILDDATE_YEAR,
                .month = SHALINFO_BUILDDATE_MONTH,
                .day   = SHALINFO_BUILDDATE_DAY,
                .hour  = SHALINFO_BUILDDATE_HOUR,
                .min   = SHALINFO_BUILDDATE_MIN
            }
        },
        .rom        = 
        {   
            .addr   = SHALINFO_ROMADDR,
            .size   = SHALINFO_ROMSIZE
        },
        .ram        = 
        {   
            .addr   = SHALINFO_RAMADDR,
            .size   = SHALINFO_RAMSIZE
        },
        .storage    = 
        {
            .type   = SHALINFO_STGTYPE,
            .size   = SHALINFO_STGSIZE,
            .addr   = SHALINFO_STGADDR
        },
        .communication  = ee_commtype_none,
        .name           = SHALINFO_NAME
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

#if defined(SHALINFO_MODE_STATICLIBRARY) || defined(SHALS_MODE_STATIC)

extern const eEmoduleInfo_t * shalinfo_moduleinfo_get(void)
{
    return((const eEmoduleInfo_t*)&s_shalinfo_moduleinfo);
}

extern const eEentity_t * shalinfo_moduleinfo_entity_get(void)
{
    return((const eEentity_t*)&s_shalinfo_moduleinfo.info.entity);
}

extern eEresult_t shalinfo_isvalid(void)
{
    return(ee_res_OK);
}

#endif



extern eEresult_t shalinfo_init(void)
{
    // in the beginning ... the base

    if(ee_res_NOK_generic == shalbase_isvalid())
    {
        return(ee_res_NOK_generic);
    }

    shalbase_init(0);

    // then the rest 

    // if we have the sharPART and it does not hold info about this shalINFO shared library, add it
    if(ee_res_OK == shalpart_isvalid())
    {
        shalpart_init();
        shalpart_shal_synchronise(ee_shalINFO, &s_shalinfo_moduleinfo);
    }

    // now we initialise the rest

    // initialise the management of boardinfo
    s_shalinfo_permanent_boardinfo_init();
    s_shalinfo_permanent_boardinfo_cache_invalidate();

    // initialise the management of deviceinfo
    s_shalinfo_permanent_deviceinfo_init();
    s_shalinfo_permanent_deviceinfo_cache_invalidate();



    return(ee_res_OK);

}


extern eEresult_t shalinfo_deinit(void)
{
    // so far we dont need to undo anything. whenever we support eeprom (on i2c) we need to ....
    return(ee_res_OK);
}

extern eEresult_t shalinfo_erase(void)
{
    volatile eEresult_t res;
    res = shalbase_storage_clr(&s_shalinfo_strg_boardinfo, sizeof(infoBoardInfo_t));
    res = shalbase_storage_clr(&s_shalinfo_strg_deviceinfo, sizeof(infoDeviceInfo_t));
    res =  res;
    return(ee_res_OK);
}



extern eEresult_t shalinfo_boardinfo_synchronise(const eEboardInfo_t* boardinfo)
{
    infoBoardInfo_t* infoboardinfo = s_shalinfo_permanent_boardinfo_get();

    if(0 != memcmp(&infoboardinfo->boardinfo, boardinfo, sizeof(eEboardInfo_t)))
    {
        memcpy(&infoboardinfo->head, &s_shalinfo_default_infohead, sizeof(infoHead_t));
        memcpy(&infoboardinfo->boardinfo, boardinfo, sizeof(eEboardInfo_t));
        s_shalinfo_permanent_boardinfo_set(infoboardinfo); 

        // it sets the mac address, the ip, the netmask according to the boardinfo.identifier
        // and also initialises the other device info parts
        shalinfo_deviceinfo_clr();
        //s_shalinfo_deviceinfo_ipnetwork_clr();
    }


    // retrieve the eEboardInfo_t stored in storage. compare it w/ boardinfo. if different, then write boardinfo in storage
    return(ee_res_OK);
}


extern eEresult_t shalinfo_boardinfo_get(const eEboardInfo_t** boardinfo)
{
    eEresult_t res = ee_res_OK;
    infoBoardInfo_t* infoboardinfo = s_shalinfo_permanent_boardinfo_get();

    *boardinfo = &infoboardinfo->boardinfo;

    if(0 == memcmp(&infoboardinfo->head.entity, &s_shalinfo_default_infohead.entity, sizeof(eEentity_t)))
    {
        res = ee_res_OK;
    }
    else
    {
        res = ee_res_NOK_generic;
    }
    return(res);
}



extern eEresult_t shalinfo_deviceinfo_clr(void)
{
    eEresult_t res = ee_res_OK;
    infoDeviceInfo_t* infodeviceinfo = s_shalinfo_permanent_deviceinfo_get();

    s_shalinfo_ipnetwork_default_set(&infodeviceinfo->deviceinfo.ipnetwork);

    s_shalinfo_can1network_default_set(&infodeviceinfo->deviceinfo.can1network);
    s_shalinfo_can2network_default_set(&infodeviceinfo->deviceinfo.can2network);
    s_shalinfo_pages_default_set(&infodeviceinfo->deviceinfo.page08[0]);

    memcpy(&infodeviceinfo->head, &s_shalinfo_default_infohead, sizeof(infoHead_t));
    infodeviceinfo->head.defflag = DEFFLAG_FALSE;
    s_shalinfo_permanent_deviceinfo_set(infodeviceinfo); 

    return(res);
}

extern eEresult_t shalinfo_deviceinfo_get(const shalinfo_deviceinfo_t** deviceinfo)
{
    eEresult_t res = ee_res_OK;
    infoDeviceInfo_t* infodeviceinfo = s_shalinfo_permanent_deviceinfo_get();

    *deviceinfo = &infodeviceinfo->deviceinfo;

    if(0 == memcmp(&infodeviceinfo->head.entity, &s_shalinfo_default_infohead.entity, sizeof(eEentity_t)))
    {
        res = ee_res_OK;
    }
    else
    {
        res = ee_res_NOK_generic;
    }
    return(res);
}


extern eEresult_t shalinfo_deviceinfo_set(const shalinfo_deviceinfo_t* deviceinfo)
{
    eEresult_t res = ee_res_OK;
    infoDeviceInfo_t* infodeviceinfo = s_shalinfo_permanent_deviceinfo_get();

    if(0 != memcmp(&(infodeviceinfo->deviceinfo), deviceinfo, sizeof(shalinfo_deviceinfo_t)))
    {
        memcpy(&infodeviceinfo->head, &s_shalinfo_default_infohead, sizeof(infoHead_t));
        memcpy((void*)&(infodeviceinfo->deviceinfo), deviceinfo, sizeof(shalinfo_deviceinfo_t));
        infodeviceinfo->head.defflag = DEFFLAG_FALSE;
        s_shalinfo_permanent_deviceinfo_set(infodeviceinfo); // aka: s_program_page();
    }

    return(res);
}



extern eEresult_t shalinfo_deviceinfo_part_get(shalinfo_deviceinfo_part_t part, const void** data)
{
    eEresult_t res = ee_res_OK;
    infoDeviceInfo_t* infodeviceinfo = s_shalinfo_permanent_deviceinfo_get();

    switch(part)
    {
        case shalinfo_ipnet:    
        {
            *data = &infodeviceinfo->deviceinfo.ipnetwork;
        } break;
        case shalinfo_can1net:    
        {
            *data = &infodeviceinfo->deviceinfo.can1network;
        } break;
        case shalinfo_can2net:    
        {
            *data = &infodeviceinfo->deviceinfo.can2network;
        } break;
        case shalinfo_page08:    
        {
            *data = &infodeviceinfo->deviceinfo.page08[0];
        } break;
        case shalinfo_page32:    
        {
            *data = &infodeviceinfo->deviceinfo.page32[0];
        } break;
        case shalinfo_page64:    
        {
            *data = &infodeviceinfo->deviceinfo.page64[0];
        } break;
        case shalinfo_page128:    
        {
            *data = &infodeviceinfo->deviceinfo.page128[0];
        } break;
        default:
        {
            *data = NULL;
            res =  ee_res_NOK_generic;
        } break;
    };

    if(NULL == *data)
    {
        return(res);
    }

    if(0 == memcmp(&infodeviceinfo->head.entity, &s_shalinfo_default_infohead.entity, sizeof(eEentity_t)))
    {
        res = ee_res_OK;
    }
    else
    {
        res = ee_res_NOK_generic;
    }
    return(res);
}


extern eEresult_t shalinfo_deviceinfo_part_set(shalinfo_deviceinfo_part_t part, const void* data)
{
    eEresult_t res = ee_res_OK;
    infoDeviceInfo_t* infodeviceinfo = s_shalinfo_permanent_deviceinfo_get();
    void *datapart =  NULL;
    uint16_t sizepart = 0;

    switch(part)
    {
        case shalinfo_ipnet:    
        { 
            datapart = &infodeviceinfo->deviceinfo.ipnetwork;
            sizepart = sizeof(eEipnetwork_t);
        } break;
        case shalinfo_can1net:    
        {
            datapart = &infodeviceinfo->deviceinfo.can1network;
            sizepart = sizeof(eEcannetwork_t);
        } break;
        case shalinfo_can2net:    
        {
            datapart = &infodeviceinfo->deviceinfo.can2network;
            sizepart = sizeof(eEcannetwork_t);
        } break;
        case shalinfo_page08:    
        {
            datapart = &infodeviceinfo->deviceinfo.page08[0];
            sizepart = sizeof(infodeviceinfo->deviceinfo.page08);
        } break;
        case shalinfo_page32:    
        {
            datapart = &infodeviceinfo->deviceinfo.page32[0];
            sizepart = sizeof(infodeviceinfo->deviceinfo.page32);
        } break;
        case shalinfo_page64:    
        {
            datapart = &infodeviceinfo->deviceinfo.page64[0];
            sizepart = sizeof(infodeviceinfo->deviceinfo.page64);
        } break;
        case shalinfo_page128:    
        {
            datapart = &infodeviceinfo->deviceinfo.page128[0];
            sizepart = sizeof(infodeviceinfo->deviceinfo.page128);
        } break;
        default:
        {
            datapart = NULL;
            sizepart = 0;
            res = ee_res_NOK_generic;
        } break;
    };

    if(NULL == datapart)
    {
        return(res);
    }

    if(0 != memcmp(datapart, data, sizepart))
    {
        memcpy(&infodeviceinfo->head, &s_shalinfo_default_infohead, sizeof(infoHead_t));
        memcpy(datapart, data, sizepart);
        infodeviceinfo->head.defflag = DEFFLAG_FALSE;
        s_shalinfo_permanent_deviceinfo_set(infodeviceinfo); // aka: s_program_page();
    }

    return(res);
}

extern eEresult_t shalinfo_deviceinfo_part_clr(shalinfo_deviceinfo_part_t part)
{
    eEresult_t res = ee_res_OK;
    infoDeviceInfo_t* infodeviceinfo = s_shalinfo_permanent_deviceinfo_get();
    void *datapart =  NULL;
    uint16_t sizepart = 0;
    
    switch(part)
    {
        case shalinfo_ipnet:    
        { 
            res = s_shalinfo_deviceinfo_ipnetwork_clr();
            datapart = NULL;
            sizepart = 0;
        } break;
        case shalinfo_can1net:    
        {
            res = s_shalinfo_deviceinfo_can1network_clr();
            datapart = NULL;
            sizepart = 0;
        } break;
        case shalinfo_can2net:    
        {
            res = s_shalinfo_deviceinfo_can2network_clr();
            datapart = NULL;
            sizepart = 0;
        } break;
        case shalinfo_page08:    
        {
            datapart = &infodeviceinfo->deviceinfo.page08[0];
            sizepart = sizeof(infodeviceinfo->deviceinfo.page08);
        } break;
        case shalinfo_page32:    
        {
            datapart = &infodeviceinfo->deviceinfo.page32[0];
            sizepart = sizeof(infodeviceinfo->deviceinfo.page32);
        } break;
        case shalinfo_page64:    
        {
            datapart = &infodeviceinfo->deviceinfo.page64[0];
            sizepart = sizeof(infodeviceinfo->deviceinfo.page64);
        } break;
        case shalinfo_page128:    
        {
            datapart = &infodeviceinfo->deviceinfo.page128[0];
            sizepart = sizeof(infodeviceinfo->deviceinfo.page128);
        } break;
        default:
        {
            datapart = NULL;
            sizepart = 0;
            res = ee_res_NOK_generic;
        } break;
    };

    if(NULL == datapart)
    {
        return(res);
    }

    memcpy(&infodeviceinfo->head, &s_shalinfo_default_infohead, sizeof(infoHead_t));
    memset(datapart, 0xFF, sizepart);
    infodeviceinfo->head.defflag = DEFFLAG_FALSE;
    s_shalinfo_permanent_deviceinfo_set(infodeviceinfo); 

    return(res);
}


//extern eEresult_t shalinfo_deviceinfo_can1network_clr(void)
//{
//    eEresult_t res = ee_res_OK;
//    infoBoardInfo_t* infoboardinfo = s_shalinfo_permanent_boardinfo_get();
//    infoDeviceInfo_t* infodeviceinfo = s_shalinfo_permanent_deviceinfo_get();
//    
//    eEcannetwork_t ntw;
//    ntw.idcan  = 1;
//
//    if(0 != memcmp(&(infodeviceinfo->deviceinfo.can1network), &ntw, sizeof(eEcannetwork_t)))
//    {
//        memcpy(&infodeviceinfo->head, &s_shalinfo_default_infohead, sizeof(infoHead_t));
//        memcpy((void*)&(infodeviceinfo->deviceinfo.can1network), &ntw, sizeof(eEcannetwork_t));
//        infodeviceinfo->head.defflag = DEFFLAG_FALSE;
//        s_shalinfo_permanent_deviceinfo_set(infodeviceinfo); 
//    }
//
//    return(res);
//}

//extern eEresult_t shalinfo_macaddr_get(const shalinfo_macaddr_t **macaddr)
//{
////    infoInfo_t  * volatile infoinfo = s_shalinfo_permanent_infoinfo_get();
////
////    *macaddr = &(infoinfo->data.macaddr);
//    return(ee_res_OK);
//}
//    
//extern eEresult_t shalinfo_macaddr_set(const shalinfo_macaddr_t *macaddr)
//{
////    infoInfo_t  * volatile infoinfo = s_shalinfo_permanent_infoinfo_get();
////
////    if(0 != memcmp(&(infoinfo->data.macaddr), macaddr, sizeof(shalinfo_macaddr_t)))
////    {
////        memcpy((void*)&(infoinfo->data.macaddr), macaddr, sizeof(shalinfo_macaddr_t));
////        infoinfo->head.defflag = DEFFLAG_FALSE;
////        s_shalinfo_permanent_infoinfo_set(infoinfo); // aka: s_program_page();
////    }
//
//    return(ee_res_OK);
//}
//
//extern eEresult_t shalinfo_ipaddr_get(const shalinfo_ipv4addr_t **ipaddr)
//{
////    infoInfo_t  * volatile infoinfo = s_shalinfo_permanent_infoinfo_get();
////
////    *ipaddr = &(infoinfo->data.ipaddr);
//    return(ee_res_OK);
//}
//
//    
//extern eEresult_t shalinfo_ipaddr_set(const shalinfo_ipv4addr_t *ipaddr)
//{
////    infoInfo_t  * volatile infoinfo = s_shalinfo_permanent_infoinfo_get();
////
////    if(0 != memcmp(&(infoinfo->data.ipaddr), ipaddr, sizeof(shalinfo_ipv4addr_t)))
////    {
////        memcpy((void*)&(infoinfo->data.ipaddr), ipaddr, sizeof(shalinfo_ipv4addr_t));
////        infoinfo->head.defflag = DEFFLAG_FALSE;
////        s_shalinfo_permanent_infoinfo_set(infoinfo); // aka: s_program_page();
////    }
//
//    return(ee_res_OK);
//}
//
//extern eEresult_t shalinfo_netmask_get(const shalinfo_ipv4addr_t **netmask)
//{
////    infoInfo_t  * volatile infoinfo = s_shalinfo_permanent_infoinfo_get();
////
////    *netmask = &(infoinfo->data.netmask);
//    return(ee_res_OK);
//}
//
//    
//extern eEresult_t shalinfo_netmask_set(const shalinfo_ipv4addr_t *netmask)
//{
////    infoInfo_t  * volatile infoinfo = s_shalinfo_permanent_infoinfo_get();
////
////    if(0 != memcmp(&(infoinfo->data.netmask), netmask, sizeof(shalinfo_ipv4addr_t)))
////    {
////        memcpy((void*)&(infoinfo->data.netmask), netmask, sizeof(shalinfo_ipv4addr_t));
////        infoinfo->head.defflag = DEFFLAG_FALSE;
////        s_shalinfo_permanent_infoinfo_set(infoinfo); // aka: s_program_page();
////    }
//
//    return(ee_res_OK);
//
//}





// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------



// -- board info routines

static void s_shalinfo_permanent_boardinfo_init(void)
{
    memset((void*)&s_shalinfo_temporary_infoboardinfo, 0, sizeof(s_shalinfo_temporary_infoboardinfo));
}

static infoBoardInfo_t* s_shalinfo_permanent_boardinfo_get(void)
{
    if(0 == s_shalinfo_temporary_infoboardinfo.head.cached)
    {
        shalbase_storage_get(&s_shalinfo_strg_boardinfo, (void*)&s_shalinfo_temporary_infoboardinfo, sizeof(infoBoardInfo_t));
    }

    s_shalinfo_temporary_infoboardinfo.head.cached = 1;

    return((infoBoardInfo_t*)&s_shalinfo_temporary_infoboardinfo);
}

static void s_shalinfo_permanent_boardinfo_set(infoBoardInfo_t *infoboardinfo)
{
    infoboardinfo->head.cached = 0;
    shalbase_storage_set(&s_shalinfo_strg_boardinfo, infoboardinfo, sizeof(infoBoardInfo_t));
}

static void s_shalinfo_permanent_boardinfo_cache_invalidate(void)
{
    s_shalinfo_temporary_infoboardinfo.head.cached = 0;
}


// -- device info routines

static void s_shalinfo_permanent_deviceinfo_init(void)
{
    memset((void*)&s_shalinfo_temporary_infodeviceinfo, 0, sizeof(s_shalinfo_temporary_infodeviceinfo));
}

static infoDeviceInfo_t* s_shalinfo_permanent_deviceinfo_get(void)
{
    if(0 == s_shalinfo_temporary_infodeviceinfo.head.cached)
    {
        shalbase_storage_get(&s_shalinfo_strg_deviceinfo, (void*)&s_shalinfo_temporary_infodeviceinfo, sizeof(infoDeviceInfo_t));
    }

    s_shalinfo_temporary_infodeviceinfo.head.cached = 1;

    return((infoDeviceInfo_t*)&s_shalinfo_temporary_infodeviceinfo);
}

static void s_shalinfo_permanent_deviceinfo_set(infoDeviceInfo_t *infodeviceinfo)
{
    infodeviceinfo->head.cached = 0;
    shalbase_storage_set(&s_shalinfo_strg_deviceinfo, infodeviceinfo, sizeof(infoDeviceInfo_t));
}

static void s_shalinfo_permanent_deviceinfo_cache_invalidate(void)
{
    s_shalinfo_temporary_infodeviceinfo.head.cached = 0;
}



// - various

static eEresult_t s_shalinfo_deviceinfo_ipnetwork_clr(void)
{
    eEresult_t res = ee_res_OK;
    infoDeviceInfo_t* infodeviceinfo = s_shalinfo_permanent_deviceinfo_get();
    
    eEipnetwork_t ntw;
    s_shalinfo_ipnetwork_default_set(&ntw);


    if(0 != memcmp(&(infodeviceinfo->deviceinfo.ipnetwork), &ntw, sizeof(eEipnetwork_t)))
    {
        memcpy(&infodeviceinfo->head, &s_shalinfo_default_infohead, sizeof(infoHead_t));
        memcpy((void*)&(infodeviceinfo->deviceinfo.ipnetwork), &ntw, sizeof(eEipnetwork_t));
        infodeviceinfo->head.defflag = DEFFLAG_FALSE;
        s_shalinfo_permanent_deviceinfo_set(infodeviceinfo); 
    }

    return(res);
}


static eEresult_t s_shalinfo_deviceinfo_can1network_clr(void)
{
    eEresult_t res = ee_res_OK;
    infoBoardInfo_t* infoboardinfo = s_shalinfo_permanent_boardinfo_get();
    infoDeviceInfo_t* infodeviceinfo = s_shalinfo_permanent_deviceinfo_get();
    
    eEcannetwork_t ntw;
    ntw.idcan  = 1;


    if(0 != memcmp(&(infodeviceinfo->deviceinfo.can1network), &ntw, sizeof(eEcannetwork_t)))
    {
        memcpy(&infodeviceinfo->head, &s_shalinfo_default_infohead, sizeof(infoHead_t));
        memcpy((void*)&(infodeviceinfo->deviceinfo.can1network), &ntw, sizeof(eEcannetwork_t));
        infodeviceinfo->head.defflag = DEFFLAG_FALSE;
        s_shalinfo_permanent_deviceinfo_set(infodeviceinfo); 
    }

    return(res);
}

static eEresult_t s_shalinfo_deviceinfo_can2network_clr(void)
{
    eEresult_t res = ee_res_OK;
    infoBoardInfo_t* infoboardinfo = s_shalinfo_permanent_boardinfo_get();
    infoDeviceInfo_t* infodeviceinfo = s_shalinfo_permanent_deviceinfo_get();
    
    eEcannetwork_t ntw;
    ntw.idcan  = 2;


    if(0 != memcmp(&(infodeviceinfo->deviceinfo.can2network), &ntw, sizeof(eEcannetwork_t)))
    {
        memcpy(&infodeviceinfo->head, &s_shalinfo_default_infohead, sizeof(infoHead_t));
        memcpy((void*)&(infodeviceinfo->deviceinfo.can2network), &ntw, sizeof(eEcannetwork_t));
        infodeviceinfo->head.defflag = DEFFLAG_FALSE;
        s_shalinfo_permanent_deviceinfo_set(infodeviceinfo); 
    }

    return(res);
}


static void s_shalinfo_ipnetwork_default_set(eEipnetwork_t* ntw)
{
    infoBoardInfo_t* infoboardinfo = s_shalinfo_permanent_boardinfo_get();

    ee_common_ipnetwork_clr(ntw, infoboardinfo->boardinfo.uniqueid);
#ifdef DONTUSEUID64 
    ntw->ipaddress   = EECOMMON_ipaddr_from(10, 255, 39, 151);
    ntw->ipnetmask   = EECOMMON_ipaddr_from(255, 255, 252, 0); 
#endif   
}

static void s_shalinfo_can1network_default_set(eEcannetwork_t* ntw)
{
    ntw->idcan  = 0;
}

static void s_shalinfo_can2network_default_set(eEcannetwork_t* ntw)
{
    ntw->idcan  = 0;
}

static void s_shalinfo_pages_default_set(void *startofpages)
{
     memset(startofpages, 0xFF, 8+32+64+128);
}
   

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------

//
//extern eEresult_t shalinfo_deviceinfo_ipnetwork_get(const eEipnetwork_t** ipntw)
//{
//    eEresult_t res = ee_res_OK;
//    infoDeviceInfo_t* infodeviceinfo = s_shalinfo_permanent_deviceinfo_get();
//
//    *ipntw = &infodeviceinfo->deviceinfo.ipnetwork;
//
//    if(0 == memcmp(&infodeviceinfo->head, &s_shalinfo_default_infohead, sizeof(infoHead_t)))
//    {
//        res = ee_res_OK;
//    }
//    else
//    {
//        res = ee_res_NOK_generic;
//    }
//    return(res);
//}
//
//extern eEresult_t shalinfo_deviceinfo_ipnetwork_set(const eEipnetwork_t* ipntw)
//{
//    eEresult_t res = ee_res_OK;
//    infoDeviceInfo_t* infodeviceinfo = s_shalinfo_permanent_deviceinfo_get();
//
//    if(0 != memcmp(&(infodeviceinfo->deviceinfo.ipnetwork), ipntw, sizeof(eEipnetwork_t)))
//    {
//        memcpy(&infodeviceinfo->head, &s_shalinfo_default_infohead, sizeof(infoHead_t));
//        memcpy((void*)&(infodeviceinfo->deviceinfo.ipnetwork), ipntw, sizeof(eEipnetwork_t));
//        infodeviceinfo->head.defflag = DEFFLAG_FALSE;
//        s_shalinfo_permanent_deviceinfo_set(infodeviceinfo); // aka: s_program_page();
//    }
//
//    return(res);
//}
//
//
//
//extern eEresult_t shalinfo_deviceinfo_can1network_get(const eEcannetwork_t** canntw)
//{
//    eEresult_t res = ee_res_OK;
//    infoDeviceInfo_t* infodeviceinfo = s_shalinfo_permanent_deviceinfo_get();
//
//    *canntw = &infodeviceinfo->deviceinfo.can1network;
//
//    if(0 == memcmp(&infodeviceinfo->head.entity, &s_shalinfo_default_infohead.entity, sizeof(eEentity_t)))
//    {
//        res = ee_res_OK;
//    }
//    else
//    {
//        res = ee_res_NOK_generic;
//    }
//    return(res);
//}
//
//extern eEresult_t shalinfo_deviceinfo_can1network_set(const eEcannetwork_t* can1ntw)
//{
//    eEresult_t res = ee_res_OK;
//    infoDeviceInfo_t* infodeviceinfo = s_shalinfo_permanent_deviceinfo_get();
//
//    if(0 != memcmp(&(infodeviceinfo->deviceinfo.can1network), can1ntw, sizeof(eEcannetwork_t)))
//    {
//        memcpy(&infodeviceinfo->head, &s_shalinfo_default_infohead, sizeof(infoHead_t));
//        memcpy((void*)&(infodeviceinfo->deviceinfo.can1network), can1ntw, sizeof(eEcannetwork_t));
//        infodeviceinfo->head.defflag = DEFFLAG_FALSE;
//        s_shalinfo_permanent_deviceinfo_set(infodeviceinfo); // aka: s_program_page();
//    }
//
//    return(res);
//}
//
//extern eEresult_t shalinfo_deviceinfo_can1network_clr(void)
//{
//    eEresult_t res = ee_res_OK;
//    infoBoardInfo_t* infoboardinfo = s_shalinfo_permanent_boardinfo_get();
//    infoDeviceInfo_t* infodeviceinfo = s_shalinfo_permanent_deviceinfo_get();
//    
//    eEcannetwork_t ntw;
//    ntw.idcan  = 1;
//
//
//    if(0 != memcmp(&(infodeviceinfo->deviceinfo.can1network), &ntw, sizeof(eEcannetwork_t)))
//    {
//        memcpy(&infodeviceinfo->head, &s_shalinfo_default_infohead, sizeof(infoHead_t));
//        memcpy((void*)&(infodeviceinfo->deviceinfo.can1network), &ntw, sizeof(eEcannetwork_t));
//        infodeviceinfo->head.defflag = DEFFLAG_FALSE;
//        s_shalinfo_permanent_deviceinfo_set(infodeviceinfo); 
//    }
//
//    return(res);
//}
//
//
//extern eEresult_t shalinfo_deviceinfo_can2network_get(const eEcannetwork_t** canntw)
//{
//    eEresult_t res = ee_res_OK;
//    infoDeviceInfo_t* infodeviceinfo = s_shalinfo_permanent_deviceinfo_get();
//
//    *canntw = &infodeviceinfo->deviceinfo.can2network;
//
//    if(0 == memcmp(&infodeviceinfo->head.entity, &s_shalinfo_default_infohead.entity, sizeof(eEentity_t)))
//    {
//        res = ee_res_OK;
//    }
//    else
//    {
//        res = ee_res_NOK_generic;
//    }
//    return(res);
//}
//
//extern eEresult_t shalinfo_deviceinfo_can2network_set(const eEcannetwork_t* can2ntw)
//{
//    eEresult_t res = ee_res_OK;
//    infoDeviceInfo_t* infodeviceinfo = s_shalinfo_permanent_deviceinfo_get();
//
//    if(0 != memcmp(&(infodeviceinfo->deviceinfo.can2network), can2ntw, sizeof(eEcannetwork_t)))
//    {
//        memcpy(&infodeviceinfo->head, &s_shalinfo_default_infohead, sizeof(infoHead_t));
//        memcpy((void*)&(infodeviceinfo->deviceinfo.can2network), can2ntw, sizeof(eEcannetwork_t));
//        infodeviceinfo->head.defflag = DEFFLAG_FALSE;
//        s_shalinfo_permanent_deviceinfo_set(infodeviceinfo); // aka: s_program_page();
//    }
//
//    return(res);
//}
//
//extern eEresult_t shalinfo_deviceinfo_can2network_clr(void)
//{
//    eEresult_t res = ee_res_OK;
//    infoBoardInfo_t* infoboardinfo = s_shalinfo_permanent_boardinfo_get();
//    infoDeviceInfo_t* infodeviceinfo = s_shalinfo_permanent_deviceinfo_get();
//    
//    eEcannetwork_t ntw;
//    ntw.idcan  = 2;
//
//
//    if(0 != memcmp(&(infodeviceinfo->deviceinfo.can2network), &ntw, sizeof(eEcannetwork_t)))
//    {
//        memcpy(&infodeviceinfo->head, &s_shalinfo_default_infohead, sizeof(infoHead_t));
//        memcpy((void*)&(infodeviceinfo->deviceinfo.can2network), &ntw, sizeof(eEcannetwork_t));
//        infodeviceinfo->head.defflag = DEFFLAG_FALSE;
//        s_shalinfo_permanent_deviceinfo_set(infodeviceinfo); 
//    }
//
//    return(res);
//}

