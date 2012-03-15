
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EESHAREDINFO_H_
#define _EESHAREDINFO_H_


/** @file       eEsharedInfo.h
    @brief      This header file implements public interface to the shared info in the micro.
    @author     marco.accame@iit.it
    @date       12/12/2011
**/

/** @defgroup ee_shared_info Library shared INFO
    The shared INFO library allows ...... 
 
    @todo acemor-facenda: do documentation.
    
    @{        
 **/

 
// - external dependencies --------------------------------------------------------------------------------------------

#include "stdint.h"
#include "eEcommon.h"


// - public #define  --------------------------------------------------------------------------------------------------





// - declaration of public user-defined types ------------------------------------------------------------------------- 

typedef struct
{
    hal_eeprom_t    haleeprom;
    eEcheckmode_t   checkmode;
    hal_crc_t       halcrc;
} eEsharinfo_cfg_t;

typedef struct          // 48 BYTES
{
    eEsysmemory_t       proc_loader;                /**< the loader and the updater process */
    eEsysmemory_t       proc_application;           /**< the application process */
    eEstorage_t         strg_ro_boardinfo;          /**< the storage is read only, shared, and keeps board info: board info */
    eEstorage_t         strg_rw_deviceinfo;         /**< the storage is read write, shared, and its content depends on boards: device info */
    eEstorage_t         strg_ro_procinfo_loader;    /**<  */
    eEstorage_t         strg_rw_procinfo_applic;    /**<  */
} eEsharinfoPartable_t;


typedef enum
{
    devinfo_can1    = 0,
    devinfo_page04  = 1,
    devinfo_page08  = 2,
    devinfo_page16  = 3,
    devinfo_page32  = 4   
} eEsharinfoDeviceInfoItem_t;


typedef struct          // 64 BYTES              
{
    eEcannetwork_t      can1network;    /*004B*/
    uint8_t             page04[4];      /*004B*/
    uint8_t             page08[8];      /*008B*/
    uint8_t             page16[16];     /*016B*/
    uint8_t             page32[32];     /*032B*/
} eEsharinfoDeviceInfo_t;


typedef struct          // 08 BYTES
{
    uint8_t             size;
    uint8_t             data[7];
} eEsharinfoIPCdata_t;


typedef struct
{
    eEprocess_t     from;
    eEprocess_t     to;
    uint8_t         opcode;             // i.e., delete_appl sent to the eloader from the 
    uint8_t         param;
} eEsharinfoIPCcmd_t;

// for instance: the eloader processes: delete_appl and delete_appl_keeping_its_strg



// - declaration of extern public functions ---------------------------------------------------------------------------

extern eEresult_t ee_sharinfo_init(const eEsharinfo_cfg_t *cfg);

extern eEresult_t ee_sharinfo_partable_sync(void);
extern eEresult_t ee_sharinfo_partable_get(const eEsharinfoPartable_t **ptab);

extern eEresult_t ee_sharinfo_boardinfo_sync(const eEboardInfo_t *brdinfo);
extern eEresult_t ee_sharinfo_boardinfo_get(const eEboardInfo_t **brdinfo);

extern eEresult_t ee_sharinfo_deviceinfo_clr(void);
extern eEresult_t ee_sharinfo_deviceinfo_set(const eEsharinfoDeviceInfo_t *devinfo);
extern eEresult_t ee_sharinfo_deviceinfo_get(const eEsharinfoDeviceInfo_t **devinfo);
extern eEresult_t ee_sharinfo_deviceinfo_item_set(const void *data, eEsharinfoDeviceInfoItem_t item);
extern eEresult_t ee_sharinfo_deviceinfo_item_get(const void **data, eEsharinfoDeviceInfoItem_t item );

extern eEresult_t ee_sharinfo_procinfo_clr(eEprocess_t proc);
extern eEresult_t ee_sharinfo_procinfo_sync(const eEmoduleInfo_t *info, eEprocess_t proc);
extern eEresult_t ee_sharinfo_procinfo_get(const eEmoduleInfo_t **info, eEprocess_t proc);

extern eEresult_t ee_sharinfo_ipcdata_clr(void);
extern eEresult_t ee_sharinfo_ipcdata_set(const eEsharinfoIPCdata_t* ipcdata);
extern eEresult_t ee_sharinfo_ipcdata_get(const eEsharinfoIPCdata_t* ipcdata);

#warning --> posso fare un get, modificare il puntatore e poi fare un set dello stesso puntatore? sarebbe bello poterlo fare. 

/** @}            
    end of group ee_shared_info 
 **/
 

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



