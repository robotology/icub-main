
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EEPERMDATA_H_
#define _EEPERMDATA_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       eEpermdata
    @brief      This header file implements public interface to ....
    @author     marco.accame@iit.it
    @date       12/12/2011
**/

/** @defgroup permdata cedcewcew
    The embENV allows ...... 
 
    @todo acemor-facenda: do documentation.
    

    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "eEcommon.h"
#include "hal.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section




// - declaration of public user-defined types ------------------------------------------------------------------------- 


typedef enum
{
    check_fixedsignature    = 0,
    check_crc16_ccitt       = 1,
    check_crc32_0x4c11db7   = 2
} eEcheckmode_t;

typedef struct
{
    const eEstorage_t*  strg;           // pointer to the eEstorage to use
    uint8_t             ctrl[4];        // hidden control data
    const void*         buffer;         // buffer which must be assigned by the user and never used for anything else
    uint32_t            size;           // size of the buffer. it must be sizeof(data2store) + 8 bytes for check
    eEcheckmode_t       checkmode;      // the check of the storage. 
    hal_eeprom_t        haleeprom2use;     // initialisation must be done externally
    hal_crc_t           halcrc2use;        // initialisation must be done externally
} eEpermdataInfo_t;

// - declaration of extern public functions ---------------------------------------------------------------------------


/** @fn         extern eEresult_t ee_permdata_init(const eEpermdataInfo_t *pdatainfo)
    @brief      init a permanent data object module according to what is specified in the eEpermdataInfo_t object.
    @param      pdatainfo           pointer to the permanent data information
    @return     ee_res_OK or ee_res_NOK_generic upon failure
    @warning    hal_eeprom and hal-crc must be properly initted
 **/
extern eEresult_t ee_permdata_init(const eEpermdataInfo_t *pdatainfo);


/** @fn         extern eEresult_t ee_permdata_deinit(const eEpermdataInfo_t *pdatainfo)
    @brief      deinit a permanent data object module so that one can re-use it after a successive ee_permdata_deinit().
    @param      pdatainfo           pointer to the permanent data information
    @return     ee_res_OK or ee_res_NOK_generic upon failure
 **/
extern eEresult_t ee_permdata_deinit(const eEpermdataInfo_t *pdatainfo);


/** @fn         extern eEresult_t ee_permdata_synch(const eEpermdataInfo_t *pdatainfo, const void *data, uint32_t size)
    @brief      makes sure that the size bytes pointed by data are in the permanent storage described by pdatainfo according
                to the crc protection specified in pdatainfo. if data is already equal then no copy is done.
    @param      pdatainfo           pointer to the permanent data information
    @param      data                the data
    @param      size                teh size of teh data
    @return     ee_res_OK or ee_res_NOK_generic upon failure
 **/
extern eEresult_t ee_permdata_synch(const eEpermdataInfo_t *pdatainfo, const void *data, uint32_t size);


/** @fn         extern eEresult_t ee_permdata_set(const eEpermdataInfo_t *pdatainfo, const void *data, uint32_t size)
    @brief      copies the size bytes pointed by data in the permanent storage described by pdatainfo using teh crc protetion
                specified in pdatainfo.
    @param      pdatainfo           pointer to the permanent data information
    @param      data                the data
    @param      size                the size of teh data
    @return     ee_res_OK or ee_res_NOK_generic upon failure
 **/
extern eEresult_t ee_permdata_set(const eEpermdataInfo_t *pdatainfo, const void *data, uint32_t size);


/** @fn         extern eEresult_t ee_permdata_get(const eEpermdataInfo_t *pdatainfo, const void **pdata, uint32_t size)
    @brief      retrieves with data the size bytes inside the permanent storage described by pdatainfo.
                it uses a cache mechanism, so taht multiple get() do not need an effective read in permanent memory .
                it verifies integrity using the crc contained in the stored data.   
    @param      pdatainfo           pointer to the permanent data information
    @param      data                the data
    @param      size                teh size of teh data
    @return     ee_res_OK or ee_res_NOK_generic upon failure
    @warning    read only operations allow to retrieve data stored with a different crc method, as long as we can manage it.
 **/
extern eEresult_t ee_permdata_get(const eEpermdataInfo_t *pdatainfo, const void **pdata, uint32_t size);


/** @fn         extern void * ee_permdata_buffer_get(const eEpermdataInfo_t *pdatainfo)
    @brief      retrieves the internal buffer in eEpermdataInfo_t and invalidate the internal data.
                this function is used to be able to change the inetrnal buffer directly without using further ram. 
                changed to the buffer must be followed by a ee_permdata_buffer_synch();
    @param      pdatainfo           pointer to the permanent data information
    @return     pointer to the buffer or NULL upon failure
 **/
extern void * ee_permdata_buffer_get(const eEpermdataInfo_t *pdatainfo);


/** @fn         extern eEresult_tee_permdata_buffer_synch(const eEpermdataInfo_t *pdatainfo)
    @brief      synchronise the internal buffer to its storage
    @param      pdatainfo           pointer to the permanent data information
    @param      size                the size to synchronise
    @return     ee_res_OK or ee_res_NOK_generic upon failure
 **/
extern eEresult_t ee_permdata_buffer_synch(const eEpermdataInfo_t *pdatainfo, uint32_t size);


/** @}            
    end of group permdata 
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



