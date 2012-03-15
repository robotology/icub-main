
/* @file       eEpermdata.c
    @brief      This header file implements ....
    @author     marco.accame@iit.it
    @date       12/12/2011
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdint.h"
#include "string.h"
#include "hal_base.h"
#include "hal_eeprom.h"
#include "hal_crc.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eEpermdata.h"


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

typedef struct 
{
    uint8_t         cached;
    uint8_t         initted;
} eEcommon_permdata_ctrl_t;


typedef struct      // 8 bytes
{
    uint8_t         mode;
    uint8_t         dummy0;       
    uint16_t        dummy1;
    uint32_t        checkdata;
} eEcheck_t;





// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

inline static void s_ss_common_permdata_ctrl_cached_clr(const eEpermdataInfo_t *pdatainfo)
{
    eEcommon_permdata_ctrl_t *ctrl = (eEcommon_permdata_ctrl_t*)&pdatainfo->ctrl[0];
    ctrl->cached = 0;
}

inline static void s_ss_common_permdata_ctrl_cached_set(const eEpermdataInfo_t *pdatainfo)
{
    eEcommon_permdata_ctrl_t *ctrl = (eEcommon_permdata_ctrl_t*)&pdatainfo->ctrl[0];
    ctrl->cached = 1;
}

inline static uint8_t s_ss_common_permdata_ctrl_cached_is(const eEpermdataInfo_t *pdatainfo)
{
    eEcommon_permdata_ctrl_t *ctrl = (eEcommon_permdata_ctrl_t*)&pdatainfo->ctrl[0];
    return(ctrl->cached);
}

inline static void s_ss_common_permdata_ctrl_initted_set(const eEpermdataInfo_t *pdatainfo)
{
    eEcommon_permdata_ctrl_t *ctrl = (eEcommon_permdata_ctrl_t*)&pdatainfo->ctrl[0];
    ctrl->initted = 1;
}

inline static void s_ss_common_permdata_ctrl_initted_clr(const eEpermdataInfo_t *pdatainfo)
{
    eEcommon_permdata_ctrl_t *ctrl = (eEcommon_permdata_ctrl_t*)&pdatainfo->ctrl[0];
    ctrl->initted = 0;
}

inline static uint8_t s_ss_common_permdata_ctrl_initted_is(const eEpermdataInfo_t *pdatainfo)
{
    eEcommon_permdata_ctrl_t *ctrl = (eEcommon_permdata_ctrl_t*)&pdatainfo->ctrl[0];
    return(ctrl->initted);
}


//static eEresult_t s_ee_common_permanent_data_init(const eEstorage_t *strg, hal_eeprom_cfg_t *eepromcfg);

static eEresult_t s_ee_common_permanent_data_set(const eEstorage_t *strg, hal_eeprom_t haleeprom2use, const void *data, uint32_t size);

static eEresult_t s_ee_common_permanent_data_get(const eEstorage_t *strg, hal_eeprom_t haleeprom2use, void *data, uint32_t size);



/** @fn         extern eEresult_t ee_common_check_isvalid(const eEcheck_t *check, const void *data, const uint32_t size)
    @brief      it verifies if the crc described by check is correct on data
    @param      check               pointer to check mode
    @param      data                the data
    @param      size                teh size of teh data
    @return     ee_res_OK if correct and ee_res_NOK_generic if not.
 **/
extern eEresult_t ee_common_check_isvalid(const eEcheck_t *check, hal_crc_t halcrc2use, const void *data, const uint32_t size);


/** @fn         extern eEresult_t ee_common_check_compute(eEcheckmode_t checkmode, const void *data, const uint32_t size, eEcheck_t *check)
    @brief      it computes the crc on data and fills check
    @param      usezero             if one it does not use crc but just a zero.
    @param      data                the data
    @param      size                teh size of teh data
    @param      check               pointer to check mode
    @return     ee_res_OK if correct and ee_res_NOK_generic if not.
 **/
extern eEresult_t ee_common_check_compute(eEcheckmode_t checkmode, hal_crc_t halcrc2use, const void *data, const uint32_t size, eEcheck_t *check);



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// empty-section

                                                                             
// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern eEresult_t ee_permdata_init(const eEpermdataInfo_t *pdatainfo)
{
    eEresult_t res;
    
    if(NULL == pdatainfo)
    {
        return(ee_res_NOK_generic);
    }
    
    s_ss_common_permdata_ctrl_cached_clr(pdatainfo);
    
    if((NULL == pdatainfo->buffer) || (NULL == pdatainfo->strg) || (pdatainfo->strg->size < pdatainfo->size))
    {
        return(ee_res_NOK_generic);
    }
    
    memset((void*)pdatainfo->buffer, 0, pdatainfo->size);
     
//    res = s_ee_common_permanent_data_init(pdatainfo->strg, eepromcfg);
//    
//    if(ee_res_NOK_generic == res)
//    {
//        return(ee_res_NOK_generic);
//    }
    
    s_ss_common_permdata_ctrl_initted_set(pdatainfo);
    
    return(ee_res_OK);
}

extern eEresult_t ee_permdata_deinit(const eEpermdataInfo_t *pdatainfo)
{
    
    if(NULL == pdatainfo)
    {
        return(ee_res_NOK_generic);
    }
    
    s_ss_common_permdata_ctrl_cached_clr(pdatainfo);
    s_ss_common_permdata_ctrl_initted_clr(pdatainfo);
     
    return(ee_res_OK);
}

extern eEresult_t ee_permdata_synch(const eEpermdataInfo_t *pdatainfo, const void *data, uint32_t size)
{
    eEresult_t res = ee_res_OK;
    eEcheck_t* check = NULL; 

    if((NULL == pdatainfo) || (0 == s_ss_common_permdata_ctrl_initted_is(pdatainfo)))
    {
        return(ee_res_NOK_generic);
    }
    
    if((NULL == data) || (pdatainfo->size < size) || (pdatainfo->buffer == data))
    {
        return(ee_res_NOK_generic);
    }
    
    if(0 == s_ss_common_permdata_ctrl_cached_is(pdatainfo))
    {
        // retrieve it from its storage ... 
        s_ee_common_permanent_data_get(pdatainfo->strg, pdatainfo->haleeprom2use, (void*)pdatainfo->buffer, size + sizeof(eEcheck_t));
        // sign it as cached
        s_ss_common_permdata_ctrl_cached_set(pdatainfo);        
    }

    // if data from storage is not coherent with its check, then ... re-synchonise
    // else if data is coherent but not equal to what i gave as argument, then ... re-synchronise.

    check = (eEcheck_t*) (&(((uint8_t*)(pdatainfo->buffer))[size]));

    if((ee_res_OK != ee_common_check_isvalid(check, pdatainfo->halcrc2use, (const void*)pdatainfo->buffer, size)) ||
       (0 != memcmp(pdatainfo->buffer, data, size) )                                                                                    )
    {   // the check is incorrect or data is different
        memcpy((void*)(pdatainfo->buffer), data, size);
        s_ss_common_permdata_ctrl_cached_clr(pdatainfo);
        // compute the check
        ee_common_check_compute(pdatainfo->checkmode, pdatainfo->halcrc2use, (const void*)pdatainfo->buffer, size, check);
        // write data and the check
        res = s_ee_common_permanent_data_set(pdatainfo->strg, pdatainfo->haleeprom2use, pdatainfo->buffer, size + sizeof(eEcheck_t));
    }
    else
    {
        res = ee_res_OK;
    }

    return(res);         
}


extern eEresult_t ee_permdata_set(const eEpermdataInfo_t *pdatainfo, const void *data, uint32_t size)
{
    eEresult_t res = ee_res_OK;
    eEcheck_t* check = NULL;

    if((NULL == pdatainfo) || (0 == s_ss_common_permdata_ctrl_initted_is(pdatainfo)))
    {
        return(ee_res_NOK_generic);
    }
    
    if((NULL == data) || (pdatainfo->size < size) || (pdatainfo->buffer == data))
    {
        return(ee_res_NOK_generic);
    }
    
    // i dont verify if data is cached ... i just copy it into storage
    memcpy((void*)(pdatainfo->buffer), data, size);
    s_ss_common_permdata_ctrl_cached_clr(pdatainfo);
    check = (eEcheck_t*) (&(((uint8_t*)(pdatainfo->buffer))[size]));
    ee_common_check_compute(pdatainfo->checkmode, pdatainfo->halcrc2use, (const void*)pdatainfo->buffer, size, check);
    res = s_ee_common_permanent_data_set(pdatainfo->strg, pdatainfo->haleeprom2use, pdatainfo->buffer, size + sizeof(eEcheck_t));

    return(res);         
}

extern eEresult_t ee_permdata_get(const eEpermdataInfo_t *pdatainfo, const void **pdata, uint32_t size)
{
    eEresult_t res = ee_res_OK;
    eEcheck_t* check = NULL;

    if((NULL == pdatainfo) || (0 == s_ss_common_permdata_ctrl_initted_is(pdatainfo)))
    {
        return(ee_res_NOK_generic);
    }
    
    if((NULL == pdata) || (pdatainfo->size < size))
    {
        return(ee_res_NOK_generic);
    }

    if(0 == s_ss_common_permdata_ctrl_cached_is(pdatainfo))
    {
        // retrieve it from its storage ... 
        s_ee_common_permanent_data_get(pdatainfo->strg, pdatainfo->haleeprom2use, (void*)pdatainfo->buffer, size + sizeof(eEcheck_t));
        check = (eEcheck_t*) (&(((uint8_t*)(pdatainfo->buffer))[size]));
        if(ee_res_OK != ee_common_check_isvalid(check, pdatainfo->halcrc2use, (const void*)pdatainfo->buffer, size))
        {
            res = ee_res_NOK_generic;
        }
        else
        {
            // sign it as cached
            s_ss_common_permdata_ctrl_cached_set(pdatainfo); 
            res = ee_res_OK; 
        }      
    }
    
    *pdata = (pdatainfo->buffer);
    return(res);    
}

extern void * ee_permdata_buffer_get(const eEpermdataInfo_t *pdatainfo)
{
 
    if((NULL == pdatainfo) || (NULL == pdatainfo->buffer))
    {
        return(NULL);
    }

    s_ss_common_permdata_ctrl_cached_clr(pdatainfo); 
    
    return((void*)pdatainfo->buffer);
}

extern eEresult_t ee_permdata_buffer_synch(const eEpermdataInfo_t *pdatainfo, uint32_t size)
{
    eEresult_t res = ee_res_OK;
    eEcheck_t* check = NULL;

    if((NULL == pdatainfo) || (0 == s_ss_common_permdata_ctrl_initted_is(pdatainfo)))
    {
        return(ee_res_NOK_generic);
    }
    
    if(pdatainfo->size < size)
    {
        return(ee_res_NOK_generic);
    }
        
    s_ss_common_permdata_ctrl_cached_clr(pdatainfo);
    check = (eEcheck_t*) (&(((uint8_t*)(pdatainfo->buffer))[size]));
    ee_common_check_compute(pdatainfo->checkmode, pdatainfo->halcrc2use, (const void*)pdatainfo->buffer, size, check);
    res = s_ee_common_permanent_data_set(pdatainfo->strg, pdatainfo->haleeprom2use, pdatainfo->buffer, size + sizeof(eEcheck_t));

    return(res);  

}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

//static eEresult_t s_ee_common_permanent_data_init(const eEstorage_t *strg, hal_eeprom_cfg_t *eepromcfg)
//{
//    hal_result_t res;
//    
//    if(NULL == strg)
//    {
//        return(ee_res_NOK_generic);
//    }
//    
//    
//    if(ee_strg_eflash == strg->type)
//    {
//        #warning --> la eeprom emulata va inizializzata una volta sola e richiede ram. 
//        res = hal_eeprom_init(hal_eeprom_emulatedflash, eepromcfg);
//    }
//    else if(ee_strg_eeprom == strg->type)
//    {
//        res = hal_eeprom_init(hal_eeprom_i2c_01, eepromcfg);
//        //hal_eeprom_erase(hal_eeprom_i2c_01, strg->addr, 20);
//    }
//
//    hal_crc_init(hal_crc0, NULL);
//
//    return((hal_res_OK == res) ? (ee_res_OK) : (ee_res_NOK_generic));  
//}

static eEresult_t s_ee_common_permanent_data_set(const eEstorage_t *strg, hal_eeprom_t haleeprom2use, const void *data, uint32_t size)
{
    hal_result_t res;
    
    if(NULL == strg)
    {
        return(ee_res_NOK_generic);
    }
    
    if((size > strg->size) || (ee_strg_none == strg->type))
    {
        return(ee_res_NOK_generic);
    }

//    if(ee_strg_eflash == strg->type)
//    {
//
////        hal_flash_erase(strg->addr, 2048);
// //       hal_flash_write(strg->addr, size, (void*)data);
//        res = hal_eeprom_write(hal_eeprom_emulatedflash, strg->addr, size, (void*)data);
//    }
//    else if(ee_strg_eeprom == strg->type)
//    {
//        res = hal_eeprom_write(hal_eeprom_i2c_01, strg->addr, size, (void*)data);
//    }

    res = hal_eeprom_write(haleeprom2use, strg->addr, size, (void*)data);
    
    return((hal_res_OK == res) ? (ee_res_OK) : (ee_res_NOK_generic));
}


static eEresult_t s_ee_common_permanent_data_get(const eEstorage_t *strg, hal_eeprom_t haleeprom2use, void *data, uint32_t size)
{
    hal_result_t res;
    
    if(NULL == strg)
    {
        return(ee_res_NOK_generic);
    }
    
    if((size > strg->size) || (ee_strg_none == strg->type))
    {
        return(ee_res_NOK_generic);
    }
    
//    if(ee_strg_eflash == strg->type)
//    {
////        // just read from flash
////        memcpy(data, (const void*)strg->addr, size);
//        res = hal_eeprom_read(hal_eeprom_emulatedflash, strg->addr, size, data);
//    }
//    else if(ee_strg_eeprom == strg->type)
//    {
//        res = hal_eeprom_read(hal_eeprom_i2c_01, strg->addr, size, data);
//    }

    res = hal_eeprom_read(haleeprom2use, strg->addr, size, data);


    return((hal_res_OK == res) ? (ee_res_OK) : (ee_res_NOK_generic));   
}


extern eEresult_t ee_common_check_isvalid(const eEcheck_t *check, hal_crc_t halcrc2use, const void *data, const uint32_t size)
{
    if((NULL == check) || (NULL == data) || (0 == size))
    {
        return(ee_res_NOK_generic);
    }
    
    if((check_fixedsignature == check->mode) && (0 == check->dummy0) && (0xcaac == check->dummy1) && (0xAAAAAAAA == check->checkdata))
    {   // ok with zero crc. as it is a micture of 1 and 0 it is nok also for un-initted flash    
        return(ee_res_OK);
    }
    else if(check_crc16_ccitt == check->mode)
    {   // verify crc on size bytes pointed by data
        uint32_t computedcrc = 0;
        hal_crc_compute(halcrc2use, hal_crc_mode_clear, data, size, &computedcrc);

        if(check->checkdata == computedcrc)
        {
            return(ee_res_OK);
        }
        return(ee_res_NOK_generic);
    }
    else
    {
        return(ee_res_NOK_generic);
    }
}


//extern eEresult_t ee_common_check_isvalid_less(const eEcheck_t *check, const void *data, const uint32_t size)
//{
//    if((NULL == check) || (NULL == data) || (0 == size))
//    {
//        return(ee_res_NOK_generic);
//    }
//    
// 
//    {   // verify crc on size bytes pointed by data
//        uint16_t computedcrc;
//        hal_crc_compute(hal_crc0, data, size, &computedcrc);
//
//        if(check->crc16 == computedcrc)
//        {
//            return(ee_res_OK);
//        }
//        return(ee_res_NOK_generic);
//    }
//}

extern eEresult_t ee_common_check_compute(eEcheckmode_t checkmode, hal_crc_t halcrc2use, const void *data, const uint32_t size, eEcheck_t *check)
{
    if((NULL == data) || (0 == size) || (NULL == check) )
    {
        return(ee_res_NOK_generic);
    }
    
    if(check_fixedsignature == checkmode)
    {
        check->mode         = check_fixedsignature;
        check->dummy0       = 0;
        check->dummy1       = 0xcaac;
        check->checkdata    = 0xAAAAAAAA;
    }
    else if(check_crc16_ccitt == checkmode)
    {
        check->mode         = check_crc16_ccitt;
        check->dummy0       = 0;
        check->dummy1       = 0xcaac;
        check->checkdata    = 0;
        hal_crc_compute(halcrc2use, hal_crc_mode_clear, data, size, &check->checkdata);
    }
    else
    {
        return(ee_res_NOK_generic);
    }
    
    return(ee_res_OK);
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



