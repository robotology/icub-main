
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EECOMMON_H_
#define _EECOMMON_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       eEcommon.h
    @brief      This header file implements public interface to the embENV.
    @author     marco.accame@iit.it
    @date       11/03/2011
**/

/** @defgroup embenv embENV environment
    The embENV allows ...... 
 
    @todo acemor-facenda: do documentation.
    

    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "emBODYporting.h"

#include "stdint.h"
#include "eEmemorymap.h"


// - public #define  --------------------------------------------------------------------------------------------------

#define EECOMMON_VERIFYsizeof(sname, ssize)         __emBODYportingVERIFYsizeof(sname, ssize)

//#define EECOMMON_ipaddr_from(ip1, ip2, ip3, ip4)    ( (((uint32_t)(ip4)&0x000000ff) << 24) | \
 //                                                     (((uint32_t)(ip3)&0x000000ff) << 16) | \
//                                                      (((uint32_t)(ip2)&0x000000ff) << 8)  | \
//                                                      (((uint32_t)(ip1)&0x000000ff) << 0) )

#define EECOMMON_ipaddr_from(ip1, ip2, ip3, ip4)    ( (((uint32_t)(ip4) << 24)&0xff000000) | \
                                                      (((uint32_t)(ip3) << 16)&0x00ff0000) | \
                                                      (((uint32_t)(ip2) << 8)&0x0000ff00)  | \
                                                      (((uint32_t)(ip1) << 0)&0x000000ff) )
#define EECOMMON_mac_oui_iit                        (0x0000000000332211)
#define EECOMMON_ipaddr_base_iit                    EECOMMON_ipaddr_from(10, 0, 0, 0)
#define EECOMMON_ipmask_default_iit                 EECOMMON_ipaddr_from(255, 255, 0, 0)



// - declaration of public user-defined types ------------------------------------------------------------------------- 


/** @typedef    typedef enum eEresult_t
    @brief      eEresult_t is used to communicate function result. 
 **/  
typedef enum               
{
    ee_res_OK               = 0,        // compatible with common type for future change
    ee_res_NOK_generic      = -1 
} eEresult_t; 


/** @typedef    typedef struct eEdate_t
    @brief      eEdate_t keeps the date of when a module is built or a board is made
    @warning    this type must be of a given fized size: 4B
 **/ 
typedef struct          // 4B               
{
    uint32_t            year  : 12;    /**< the year a.d. upto 2047 */
    uint32_t            month : 4;     /**< the month, where jan is 1, dec is 12 */
    uint32_t            day   : 5;     /**< the day from 1 to 31 */
    uint32_t            hour  : 5;     /**< the hour from 0 to 23 */
    uint32_t            min   : 6;     /**< the minute from 0 to 59 */
} eEdate_t;             __emBODYportingVERIFYsizeof(eEdate_t, 4);


/** @typedef    typedef struct eEversion_t
    @brief      eEversion_t is used to keep the version of an eEmodule. 
    @warning    this type must be of a given fized size: 4B
 **/  
typedef struct                  // 02B
{
    uint8_t             major;          /**< major number   */ 
    uint8_t             minor;          /**< minor number  */  
} eEversion_t;          __emBODYportingVERIFYsizeof(eEversion_t, 2);


  
//typedef uint8_t eEmodule_t;


/** @typedef    typedef enum eEmoduleType_t
    @brief      eEmoduleType_t keeps teh kinds of modules. 
 **/  
typedef enum               
{
    ee_none                 = 0,
    ee_process              = 1,        
    ee_sharlib              = 2,
    ee_storage              = 3    
} eEmoduleType_t;


/** @typedef    typedef struct eEsysmemory_t
    @brief      eEsysmemory_t keeps information on memory. 
    @warning    this type must be of a given fized size: 8B
  **/
typedef struct          // 8B 
{
    uint32_t            addr;               /**< the address of memory */
    uint32_t            size;               /**< the size of memory */
} eEsysmemory_t;        __emBODYportingVERIFYsizeof(eEsysmemory_t, 8);


/** @typedef    typedef enum eEstorageType_t
    @brief      eEstorageType_t specifies what kind of storage it can be used. 
 **/  
typedef enum
{
    ee_strg_none            = 0,                    /**< no storage */
    ee_strg_eflash          = 1,                    /**< embedded flash */
    ee_strg_emuleeprom      = 2,                    /**< eeprom emulated on flash */
    ee_strg_eeprom          = 3                     /**< eeprom */
} eEstorageType_t;


/** @typedef    typedef struct eEstorage_t
    @brief      eEstorage_t keeps information on storage. 
    @warning    this type must be of a given fized size: 8B
  **/
typedef struct          // 08 BYTES 
{
    uint32_t            type   : 2;         /**< the type of storage medium: use eEstorageType_t */
    uint32_t            size   : 30;        /**< the size of storage medium */
    uint32_t            addr   : 32;        /**< the starting address of storage medium */
} eEstorage_t;          __emBODYportingVERIFYsizeof(eEstorage_t, 8);


typedef enum
{
    ee_commtype_none    = 0,
    ee_commtype_eth     = 1 << 0,
    ee_commtype_can1    = 1 << 1,
    ee_commtype_can2    = 1 << 2,
    ee_commtype_gtw     = 1 << 3,
    ee_commtype_exp4    = 1 << 4,
    ee_commtype_exp5    = 1 << 5,
    ee_commtype_exp6    = 1 << 6,
    ee_commtype_exp7    = 1 << 7
} eEcommunicationType_t;


/** @typedef    typedef struct eEprotocolInfo_t
    @brief      eEprotocolInfo_t keeps information on protocol. 
    @warning    this type must be of a given fized size: 8B
  **/
typedef struct          // 08 BYTES
{
    eEversion_t         udpprotversion;     /**< the protocol version of the udp communication (non-zero if exists) */
    eEversion_t         can1protversion;    /**< the protocol version of the can1 communication (non-zero if exists) */
    eEversion_t         can2protversion;    /**< the protocol version of the can2 communication (non-zero if exists) */
    eEversion_t         gtwprotversion;    /**< the protocol version of the gtw communication (non-zero if exists) */
} eEprotocolInfo_t;     __emBODYportingVERIFYsizeof(eEprotocolInfo_t, 8);


/** @typedef    typedef struct eEipnetwork_t
    @brief      eEipnetwork_t keeps information on the ipnetwork. 
    @warning    this type must be of a given fized size: 16B
  **/
typedef struct          // 16 BYTES
{
    uint64_t            macaddress;         /**< the mac address is contained in the 6 lsb */
    uint32_t            ipaddress;          /**< the ip address. if zero dchp is used */
    uint32_t            ipnetmask;          /**< the netmask */
} eEipnetwork_t;        __emBODYportingVERIFYsizeof(eEipnetwork_t, 16);



/** @typedef    typedef struct eEcannetwork_t
    @brief      eEcannetwork_t keeps information on the can network. 
    @warning    this type must be of a given fized size: 4B
  **/
typedef struct          // 4 BYTES
{
    uint32_t            idcan;
} eEcannetwork_t;       __emBODYportingVERIFYsizeof(eEcannetwork_t, 4);


typedef enum
{
    ee_entity_none                 = 0,
    ee_entity_board                = 1, 
    ee_entity_process              = 2,        
    ee_entity_sharlib              = 3,
    ee_entity_filesys              = 4,  
    ee_entity_forfuture5           = 5,
    ee_entity_forfuture6           = 6,
    ee_entity_forfuture7           = 7,
} eEtypeOfEntity_t;


/** @typedef    typedef struct eEcannetwork_t
    @brief      eEcannetwork_t keeps information on the can network. 
    @warning    this type must be of a given fized size: 8B
  **/
typedef struct          // 8 BYTES (1+1+2+4)
{
    uint8_t             type;               /**< the type of entity: use enum type eEtypeOfEntity_t */
    uint8_t             signature;          /**< the signature of the entity. use enum values in eEprocess_t or eEsharlib_t or eEboard_t etc. */
    eEversion_t         version;            /**< the version of the entity w/ major + minor */
    eEdate_t            builddate;          /**< the build date of the entity */
} eEentity_t;           __emBODYportingVERIFYsizeof(eEentity_t, 8);


/** @typedef    typedef struct eEinfo_t
    @brief      eEinfo_t keeps information about a basic embedded entity: a board, a sw module, a shared library, etc. 
    @warning    this type must be of a given fized size: 48B
  **/
typedef struct          // 48 BYTES upto name[15]
{
    eEentity_t          entity;             /**< the entity */
    eEsysmemory_t       rom;                /**< the total size of rom which is available */
    eEsysmemory_t       ram;                /**< the total size of ram which is available */
    eEstorage_t         storage;            /**< the storage space available and its medium */
    uint8_t             communication;      /**< the supported communication types: use enum eEcommunicationType_t in | combination */
    uint8_t             name[15];           /**< a string containing a descriptive name */
} eEinfo_t;             __emBODYportingVERIFYsizeof(eEinfo_t, 48);
        

/** @typedef    typedef struct eEinfoBoard_t
    @brief      eEinfoBoard_t keeps information about a hw board. 
    @warning    this type must be of a given fized size: 64B
  **/
typedef struct          // 64 BYTES (48 + 8 + 8)              
{
    eEinfo_t            info;               /**< the base info                                                  */
    uint64_t            uniqueid;           /**< a unique id for the board                                      */
    uint8_t             extra[8];           /**< extra space for information                                    */
} eEboardInfo_t;        __emBODYportingVERIFYsizeof(eEboardInfo_t, 64);



/** @typedef    typedef struct eEinfoModule_t
    @brief      eEinfoModule_t keeps information about a sw module (process or sharlib).
    @warning    this type must be of a given fized size: 64B    
  **/
typedef struct          // 64 BYTES (48 + 8 + 8)              
{
    eEinfo_t            info;               /**< the base info                                                  */
    eEprotocolInfo_t    protocols;          /**< information on communication capabilities of the sw module     */
    uint8_t             extra[8];           /**< extra space for information                                    */
} eEmoduleInfo_t;       __emBODYportingVERIFYsizeof(eEmoduleInfo_t, 64);


/** @typedef    typedef enum eEprocess_t
    @brief      eEprocess_t keep the allowed eprocesses in embENV.
    @warning    this type must be of a given fized size: 1B    
 **/ 
typedef uint8_t eEprocess_t;


/** @typedef    typedef enum eEprocessvalues_t
    @brief      eEprocess_t keep identifiers of the allowed eprocesses in embENV. 
 **/ 
typedef enum               
{
    ee_procNone              = 255,
    ee_procLoader            = 0, 
    ee_procUpdater           = 1,
    ee_procApplication       = 2,
    ee_procApplUser03        = 3,
    ee_procApplUser04        = 4
} eEprocessvalues_t;

enum { ee_procMaxNum = 5 };


/** @typedef    typedef enum eEsharlib_t
    @brief      eEsharlib_t keeps identifier of the allowed shared libraries in embENV. 
 **/ 
typedef enum               
{
    ee_shalNone             = 255,
    ee_shalBASE             = 0, 
    ee_shalPART             = 1,
    ee_shalINFO             = 2,
    ee_shalHAL              = 3,
    ee_shalOSAL             = 4,
    ee_shalIPAL             = 5,
    ee_shalFSAL             = 6,
    ee_shalDSPAL            = 7,
    ee_shalUser08           = 8, 
    ee_shalUser09           = 9
} eEsharlib_t; 

enum { ee_shalMaxNum = 10 }; 


///** @typedef    typedef struct eEshalflags_t
//    @brief      eEshalflags_t keeps what shared libraries are available. 
//  **/
//typedef struct
//{
//    uint32_t    shalbase    : 1;
//    uint32_t    shalpart    : 1;
//    uint32_t    shalinfo    : 1; 
//    uint32_t    shalhal     : 1;
//    uint32_t    shalosal    : 1;
//    uint32_t    shalipal    : 1;
//    uint32_t    shalfsal    : 1;
//    uint32_t    shaldspal   : 1;
//    uint32_t    others      : 24;
//} eEshalflags_t;


/** @typedef    typedef struct eEinfoModule_t
    @brief      eEinfoModule_t keeps information about a sw module (process or sharlib).
    @warning    this type must be of a given fized size: 40B    
  **/
typedef struct          // 40 BYTES
{
    eEsysmemory_t       proc_loader;        /**< the loader and the updater process */
    eEsysmemory_t       proc_applic;        /**< the application process */
    eEstorage_t         strg_ro_shar_brd;   /**< the storage is read only, shared, and keeps board info: board info */
    eEstorage_t         strg_rw_shar_brd;   /**< the storage is read write, shared, and its content depends on boards: device info */
    eEstorage_t         strg_rw_priv_app;   /**< the storage is read write, private, and its content depends on application: applic info */
} eEbasicPartable_t;    __emBODYportingVERIFYsizeof(eEbasicPartable_t, 40);


// - declaration of extern public functions ---------------------------------------------------------------------------

EO_extern_inline const eEentity_t * ee_common_moduleinfo_to_entity(const eEmoduleInfo_t *mi)
{ 
    return((const eEentity_t*)&(mi->info.entity)); 
}

extern eEresult_t ee_common_ipnetwork_clr(eEipnetwork_t* ntw, uint64_t uniqueid);
 

/** @}            
    end of group embenv 
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



