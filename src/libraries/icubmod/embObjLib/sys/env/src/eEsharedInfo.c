
/* @file       eEsharedInfo.c
    @brief      This header file implements the shared info library.
    @author     marco.accame@iit.it
    @date       12/12/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "string.h"
#include "eEcommon.h"
#include "eEmemorymap.h"
#include "eEpermdata.h"
#include "hal.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eEsharedInfo.h"


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



// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_ee_sharinfo_partable_init(const eEsharinfo_cfg_t *cfg);

static void s_ee_sharinfo_boardinfo_init(const eEsharinfo_cfg_t *cfg);

static void s_ee_sharinfo_deviceinfo_init(const eEsharinfo_cfg_t *cfg);

static void s_ee_sharinfo_loaderinfo_init(const eEsharinfo_cfg_t *cfg);
static void s_ee_sharinfo_applicinfo_init(const eEsharinfo_cfg_t *cfg);
static void s_ee_sharinfo_ipcdata_init(const eEsharinfo_cfg_t *cfg);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

const eEsharinfo_cfg_t s_ee_sharinfo_cfg_default =
{
    .haleeprom      = hal_eeprom_emulatedflash,
    .checkmode      = check_crc16_ccitt,
    .halcrc         = hal_crc0
};


// - partition table --------------------------------------------------------------------------------------------------

static const eEsharinfoPartable_t s_sharinfo_ptab_data =
{
    .proc_loader        =       { .addr = EE_MEMMAP_LOADER_ROM_ADDR, .size = EE_MEMMAP_LOADER_ROM_SIZE },
    .proc_application   =       { .addr = EE_MEMMAP_APPLICATION_ROM_ADDR, .size = EE_MEMMAP_APPLICATION_ROM_SIZE },
    .strg_ro_boardinfo  =       { .type = ee_strg_emuleeprom, .size = EE_MEMMAP_STRG_SHRO_BRDINFO_SIZE, .addr = EE_MEMMAP_STRG_SHRO_BRDINFO_ADDR},
    .strg_rw_deviceinfo =       { .type = ee_strg_emuleeprom, .size = EE_MEMMAP_STRG_SHRW_DEVINFO_SIZE, .addr = EE_MEMMAP_STRG_SHRW_DEVINFO_ADDR},
    .strg_ro_procinfo_loader =  { .type = ee_strg_emuleeprom, .size = EE_MEMMAP_STRG_SHRO_LOADERINFO_SIZE, .addr = EE_MEMMAP_STRG_SHRO_LOADERINFO_ADDR},
    .strg_rw_procinfo_applic =  { .type = ee_strg_emuleeprom, .size = EE_MEMMAP_STRG_SHRW_APPLICINFO_SIZE, .addr = EE_MEMMAP_STRG_SHRW_APPLICINFO_ADDR}
};

static const eEstorage_t s_sharinfo_ptab_stg = 
{ 
    .type = ee_strg_emuleeprom, 
    .size = EE_MEMMAP_STRG_SHRO_PTABLE_SIZE, 
    .addr = EE_MEMMAP_STRG_SHRO_PTABLE_ADDR
};

static uint8_t s_sharinfo_ptab_buffer[sizeof(eEsharinfoPartable_t)+8] = {0}; 

static eEpermdataInfo_t s_sharinfo_ptab_permdatainfo;


// - board info -------------------------------------------------------------------------------------------------------

static const eEstorage_t s_sharinfo_brdi_stg = 
{ 
    .type = ee_strg_emuleeprom, 
    .size = EE_MEMMAP_STRG_SHRO_BRDINFO_SIZE, 
    .addr = EE_MEMMAP_STRG_SHRO_BRDINFO_ADDR
};

static uint8_t s_sharinfo_brdi_buffer[sizeof(eEboardInfo_t)+8] = {0}; 

static eEpermdataInfo_t s_sharinfo_brdi_permdatainfo;



// - device info ------------------------------------------------------------------------------------------------------

static const eEstorage_t s_sharinfo_devi_stg = 
{ 
    .type = ee_strg_emuleeprom, 
    .size = EE_MEMMAP_STRG_SHRW_DEVINFO_SIZE, 
    .addr = EE_MEMMAP_STRG_SHRW_DEVINFO_ADDR
};

static uint8_t s_sharinfo_devi_buffer[sizeof(eEsharinfoDeviceInfo_t)+8] = {0}; 

static eEpermdataInfo_t s_sharinfo_devi_permdatainfo;


// - proc info loader ------------------------------------------------------------------------------------------------

static const eEstorage_t s_sharinfo_ldri_stg = 
{ 
    .type = ee_strg_emuleeprom, 
    .size = EE_MEMMAP_STRG_SHRO_LOADERINFO_SIZE, 
    .addr = EE_MEMMAP_STRG_SHRO_LOADERINFO_ADDR
};

static uint8_t s_sharinfo_ldri_buffer[sizeof(eEmoduleInfo_t)+8] = {0}; 

static eEpermdataInfo_t s_sharinfo_ldri_permdatainfo;


// - proc info application --------------------------------------------------------------------------------------------

static const eEstorage_t s_sharinfo_appi_stg = 
{ 
    .type = ee_strg_emuleeprom, 
    .size = EE_MEMMAP_STRG_SHRW_APPLICINFO_SIZE, 
    .addr = EE_MEMMAP_STRG_SHRW_APPLICINFO_ADDR
};

static uint8_t s_sharinfo_appi_buffer[sizeof(eEmoduleInfo_t)+8] = {0}; 

static eEpermdataInfo_t s_sharinfo_appi_permdatainfo;


// - ipc data ---------------------------------------------------------------------------------------------------------

static const eEstorage_t s_sharinfo_ipcd_stg = 
{ 
    .type = ee_strg_emuleeprom, 
    .size = EE_MEMMAP_STRG_SHRW_IPROCCOMM_SIZE, 
    .addr = EE_MEMMAP_STRG_SHRW_IPROCCOMM_ADDR
};

static uint8_t s_sharinfo_ipcd_buffer[sizeof(eEsharinfoIPCdata_t)+8] = {0}; 

static eEpermdataInfo_t s_sharinfo_ipcd_permdatainfo;
                                                                             
// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern eEresult_t ee_sharinfo_init(const eEsharinfo_cfg_t *cfg)
{
    // the caller initialise the eeprom, synchronises the eEsharinfoPartable_t, if invalid resets the eEsharinfoProcTable_t,
    // if invalid resets the eEsharinfoDeviceInfo_t

    if(NULL == cfg)
    {
        cfg = &s_ee_sharinfo_cfg_default;
    }
    
    s_ee_sharinfo_partable_init(cfg);
    s_ee_sharinfo_boardinfo_init(cfg);
    s_ee_sharinfo_deviceinfo_init(cfg);

    s_ee_sharinfo_loaderinfo_init(cfg);
    s_ee_sharinfo_applicinfo_init(cfg);

    s_ee_sharinfo_ipcdata_init(cfg);

    return(ee_res_OK);

}


extern eEresult_t ee_sharinfo_partable_sync(void)
{
    return(ee_permdata_synch(&s_sharinfo_ptab_permdatainfo, &s_sharinfo_ptab_data, sizeof(s_sharinfo_ptab_data)));  
}

extern eEresult_t ee_sharinfo_partable_get(const eEsharinfoPartable_t **ptab)
{
    return(ee_permdata_get(&s_sharinfo_ptab_permdatainfo, (const void**)ptab, sizeof(s_sharinfo_ptab_data)));
}


extern eEresult_t ee_sharinfo_boardinfo_sync(const eEboardInfo_t *brdinfo)
{
    return(ee_permdata_synch(&s_sharinfo_brdi_permdatainfo, brdinfo, sizeof(eEboardInfo_t)));
}

extern eEresult_t ee_sharinfo_boardinfo_get(const eEboardInfo_t **brdinfo)
{
    return(ee_permdata_get(&s_sharinfo_brdi_permdatainfo, (const void**)brdinfo, sizeof(eEboardInfo_t)));
}


extern eEresult_t ee_sharinfo_deviceinfo_clr(void)
{
    eEsharinfoDeviceInfo_t *devinfo = NULL;
    eEresult_t res;

    ee_permdata_get(&s_sharinfo_devi_permdatainfo, (const void**)&devinfo, sizeof(eEsharinfoDeviceInfo_t));

    memset(devinfo, 0, sizeof(eEsharinfoDeviceInfo_t)); 

    res = ee_permdata_buffer_synch(&s_sharinfo_devi_permdatainfo, sizeof(eEsharinfoDeviceInfo_t));    

    return(res);
}

extern eEresult_t ee_sharinfo_deviceinfo_set(const eEsharinfoDeviceInfo_t *devinfo)
{
    return(ee_permdata_set(&s_sharinfo_devi_permdatainfo, (const void*)devinfo, sizeof(eEsharinfoDeviceInfo_t)));
}

extern eEresult_t ee_sharinfo_deviceinfo_get(const eEsharinfoDeviceInfo_t **devinfo)
{
    return(ee_permdata_get(&s_sharinfo_devi_permdatainfo, (const void**)devinfo, sizeof(eEsharinfoDeviceInfo_t)));
}

extern eEresult_t ee_sharinfo_deviceinfo_item_set(const void *data, eEsharinfoDeviceInfoItem_t item)
{
    eEsharinfoDeviceInfo_t *devinfo = NULL;
    eEresult_t res;

    res = ee_permdata_get(&s_sharinfo_devi_permdatainfo, (const void**)&devinfo, sizeof(eEsharinfoDeviceInfo_t));

    if(ee_res_OK != res)
    {
        return(res);
    }

    switch(item)
    {
        case devinfo_can1:
        {
            memcpy(&devinfo->can1network, data, sizeof(devinfo->can1network));
        } break;

        case devinfo_page04:
        {
            memcpy(devinfo->page04, data, sizeof(devinfo->page04));
        } break;

        case devinfo_page08:
        {
            memcpy(devinfo->page08, data, sizeof(devinfo->page08));
        } break;

        case devinfo_page16:
        {
            memcpy(devinfo->page16, data, sizeof(devinfo->page16));
        } break;

        case devinfo_page32:
        {
            memcpy(devinfo->page32, data, sizeof(devinfo->page32));
        } break;

        default:
        {
            res = ee_res_NOK_generic;
        } break;
    } 

    if(ee_res_OK == res)
    {
        res = ee_permdata_buffer_synch(&s_sharinfo_devi_permdatainfo, sizeof(eEsharinfoDeviceInfo_t));    
    }

    return(res);
}

extern eEresult_t ee_sharinfo_deviceinfo_item_get(const void **data, eEsharinfoDeviceInfoItem_t item )
{
    const eEsharinfoDeviceInfo_t *devinfo = NULL;
    eEresult_t res;

    res = ee_permdata_get(&s_sharinfo_devi_permdatainfo, (const void**)&devinfo, sizeof(eEsharinfoDeviceInfo_t));

    if(ee_res_OK != res)
    {
        return(res);
    }

    switch(item)
    {
        case devinfo_can1:
        {
            *data = &devinfo->can1network;
        } break;

        case devinfo_page04:
        {
            *data = &devinfo->page04;
        } break;

        case devinfo_page08:
        {
            *data = &devinfo->page08;
        } break;

        case devinfo_page16:
        {
            *data = &devinfo->page16;
        } break;

        case devinfo_page32:
        {
            *data = &devinfo->page32;
        } break;

        default:
        {
            *data = NULL;
            res = ee_res_NOK_generic;
        } break;
    } 

    return(res);
}


extern eEresult_t ee_sharinfo_procinfo_clr(eEprocess_t proc)
{
    eEmoduleInfo_t *procinfo = NULL;
    eEpermdataInfo_t *pdatainfo = NULL;
    eEresult_t res;
    
    switch(proc)
    {
        case ee_procLoader:
        {
            pdatainfo = &s_sharinfo_ldri_permdatainfo;
        } break;

        case ee_procApplication:
        {
            pdatainfo = &s_sharinfo_appi_permdatainfo;
        } break;

        default:
        {
            pdatainfo = NULL;            
        } break;
    } 

    if(NULL == pdatainfo)
    {
        return(ee_res_NOK_generic);
    }

    ee_permdata_get(pdatainfo, (const void**)&procinfo, sizeof(eEmoduleInfo_t));
    memset(procinfo, 0, sizeof(eEmoduleInfo_t)); 
    res = ee_permdata_buffer_synch(pdatainfo, sizeof(eEmoduleInfo_t));    

    return(res);
}

extern eEresult_t ee_sharinfo_procinfo_sync(const eEmoduleInfo_t *info, eEprocess_t proc)
{
    eEpermdataInfo_t *pdatainfo = NULL;
    
    switch(proc)
    {
        case ee_procLoader:
        {
            pdatainfo = &s_sharinfo_ldri_permdatainfo;
        } break;

        case ee_procApplication:
        {
            pdatainfo = &s_sharinfo_appi_permdatainfo;
        } break;

        default:
        {
            pdatainfo = NULL;            
        } break;
    } 

    if(NULL == pdatainfo)
    {
        return(ee_res_NOK_generic);
    }
       
    return(ee_permdata_synch(pdatainfo, info, sizeof(eEmoduleInfo_t)));
}

extern eEresult_t ee_sharinfo_procinfo_get(const eEmoduleInfo_t **info, eEprocess_t proc)
{
    eEpermdataInfo_t *pdatainfo = NULL;
    
    switch(proc)
    {
        case ee_procLoader:
        {
            pdatainfo = &s_sharinfo_ldri_permdatainfo;
        } break;

        case ee_procApplication:
        {
            pdatainfo = &s_sharinfo_appi_permdatainfo;
        } break;

        default:
        {
            pdatainfo = NULL;            
        } break;
    } 

    if(NULL == pdatainfo)
    {
        return(ee_res_NOK_generic);
    }

    return(ee_permdata_get(pdatainfo, (const void**)info, sizeof(eEmoduleInfo_t)));
}

extern eEresult_t ee_sharinfo_ipcdata_clr(void)
{
    eEsharinfoIPCdata_t *ipcdata = NULL;
    eEresult_t res;

    ee_permdata_get(&s_sharinfo_ipcd_permdatainfo, (const void**)&ipcdata, sizeof(eEsharinfoIPCdata_t));

    memset(ipcdata, 0, sizeof(eEsharinfoIPCdata_t)); 

    res = ee_permdata_buffer_synch(&s_sharinfo_ipcd_permdatainfo, sizeof(eEsharinfoIPCdata_t));    

    return(res);
}

extern eEresult_t ee_sharinfo_ipcdata_set(const eEsharinfoIPCdata_t* ipcdata)
{
    return(ee_permdata_set(&s_sharinfo_ipcd_permdatainfo, (const void*)ipcdata, sizeof(eEsharinfoIPCdata_t)));
}

extern eEresult_t ee_sharinfo_ipcdata_get(const eEsharinfoIPCdata_t* ipcdata)
{
    return(ee_permdata_get(&s_sharinfo_ipcd_permdatainfo, (const void**)ipcdata, sizeof(eEsharinfoIPCdata_t)));
}




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void s_ee_sharinfo_partable_init(const eEsharinfo_cfg_t *cfg)     
{
    s_sharinfo_ptab_permdatainfo.strg   = &s_sharinfo_ptab_stg;
    s_sharinfo_ptab_permdatainfo.buffer = s_sharinfo_ptab_buffer;
    s_sharinfo_ptab_permdatainfo.size   = sizeof(s_sharinfo_ptab_buffer);
    s_sharinfo_ptab_permdatainfo.checkmode = cfg->checkmode;
    s_sharinfo_ptab_permdatainfo.haleeprom2use = cfg->haleeprom;
    s_sharinfo_ptab_permdatainfo.halcrc2use = cfg->halcrc;


    ee_permdata_init(&s_sharinfo_ptab_permdatainfo);    
}

static void s_ee_sharinfo_boardinfo_init(const eEsharinfo_cfg_t *cfg)     
{
    s_sharinfo_brdi_permdatainfo.strg   = &s_sharinfo_brdi_stg;
    s_sharinfo_brdi_permdatainfo.buffer = s_sharinfo_brdi_buffer;
    s_sharinfo_brdi_permdatainfo.size   = sizeof(s_sharinfo_brdi_buffer);
    s_sharinfo_brdi_permdatainfo.checkmode = cfg->checkmode;
    s_sharinfo_brdi_permdatainfo.haleeprom2use = cfg->haleeprom;
    s_sharinfo_brdi_permdatainfo.halcrc2use = cfg->halcrc;


    ee_permdata_init(&s_sharinfo_brdi_permdatainfo);    
}


static void s_ee_sharinfo_deviceinfo_init(const eEsharinfo_cfg_t *cfg)     
{
    s_sharinfo_devi_permdatainfo.strg   = &s_sharinfo_devi_stg;
    s_sharinfo_devi_permdatainfo.buffer = s_sharinfo_devi_buffer;
    s_sharinfo_devi_permdatainfo.size   = sizeof(s_sharinfo_devi_buffer);
    s_sharinfo_devi_permdatainfo.checkmode = cfg->checkmode;
    s_sharinfo_devi_permdatainfo.haleeprom2use = cfg->haleeprom;
    s_sharinfo_devi_permdatainfo.halcrc2use = cfg->halcrc;


    ee_permdata_init(&s_sharinfo_devi_permdatainfo);    
}


static void s_ee_sharinfo_loaderinfo_init(const eEsharinfo_cfg_t *cfg)     
{
    s_sharinfo_ldri_permdatainfo.strg   = &s_sharinfo_ldri_stg;
    s_sharinfo_ldri_permdatainfo.buffer = s_sharinfo_ldri_buffer;
    s_sharinfo_ldri_permdatainfo.size   = sizeof(s_sharinfo_ldri_buffer);
    s_sharinfo_ldri_permdatainfo.checkmode = cfg->checkmode;
    s_sharinfo_ldri_permdatainfo.haleeprom2use = cfg->haleeprom;
    s_sharinfo_ldri_permdatainfo.halcrc2use = cfg->halcrc;


    ee_permdata_init(&s_sharinfo_ldri_permdatainfo);    
}

static void s_ee_sharinfo_applicinfo_init(const eEsharinfo_cfg_t *cfg)     
{
    s_sharinfo_appi_permdatainfo.strg   = &s_sharinfo_appi_stg;
    s_sharinfo_appi_permdatainfo.buffer = s_sharinfo_appi_buffer;
    s_sharinfo_appi_permdatainfo.size   = sizeof(s_sharinfo_appi_buffer);
    s_sharinfo_appi_permdatainfo.checkmode = cfg->checkmode;
    s_sharinfo_appi_permdatainfo.haleeprom2use = cfg->haleeprom;
    s_sharinfo_appi_permdatainfo.halcrc2use = cfg->halcrc;


    ee_permdata_init(&s_sharinfo_appi_permdatainfo);    
}

static void s_ee_sharinfo_ipcdata_init(const eEsharinfo_cfg_t *cfg)     
{
    s_sharinfo_ipcd_permdatainfo.strg   = &s_sharinfo_ipcd_stg;
    s_sharinfo_ipcd_permdatainfo.buffer = s_sharinfo_ipcd_buffer;
    s_sharinfo_ipcd_permdatainfo.size   = sizeof(s_sharinfo_ipcd_buffer);
    s_sharinfo_ipcd_permdatainfo.checkmode = cfg->checkmode;
    s_sharinfo_ipcd_permdatainfo.haleeprom2use = cfg->haleeprom;
    s_sharinfo_ipcd_permdatainfo.halcrc2use = cfg->halcrc;


    ee_permdata_init(&s_sharinfo_ipcd_permdatainfo);    
}

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------

