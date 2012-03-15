

// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVS_H_
#define _EOTHENVS_H_


/** @file       EOtheNVs.h
    @brief      This header file implements public interface to the NVs singleton.
    @author     marco.accame@iit.it
    @date       09/06/2011
**/

/** @defgroup eo_thenvs Object EOtheNVs
    The EOtheNVs is a singleton which contains knowledge of all the NVs owned locally by a device or knownfrom other
    devices.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVmutex.h"
#include "EOnv.h"
//#include "EOnetvarNode.h"
#include "EOVstorage.h"
#include "EOconstarray.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 



/** @typedef    typedef struct EOtheNVs_hid EOtheNVs
    @brief      EOtheNVs is an opaque struct. It is used to implement data abstraction for the NVs  
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOtheNVs_hid EOtheNVs;


///** @typedef    typedef const eOnvs_NVarray_t
//    @brief      Contains an array of NVs
// **/ 
//typedef const struct
//{
//    eOipv4addr_t            ipaddr;
//    uint16_t                nvnum;
////    EOnetvarNode*           nvars;
//} eOnvs_NVnodearray_t;


///** @typedef    typedef const struct eOnvs_cfg_t
//    @brief      Contains the configuration for the EOtheNVs object
// **/ 
//typedef const struct  
//{
//    eOvoidp_fp_void_t       fn_get_loc_storage;         /**< retrieves the pointer to the EOVstorageDerived object used to store permanent data of NVs */
//    eOvoid_fp_void_t        fn_init_volatile_data;      /**< Initialises the volatile data in all the NVs, both local and remote */
//    eOuint16_fp_uint32_t    fn_from_ipv4_to_remotedev_index; /**< retrieves the index of array @e remotenvs, given the IP address */
//    eOnvs_NVnodearray_t     localnvs;                   /**< array of NV nodes owned locally by the device  */
//    uint16_t                remotenum;                  /**< number of devices with NVs known by this device  */
//    eOnvs_NVnodearray_t*    remotenvs;                  /**< array of array of NV nodes owned by other devices but known by this device */
//} eOnvs_OLDcfg_t;




/** @typedef    typedef const struct eOnvs_NEW_cfg_t
    @brief      Contains the configuration for the EOtheNVs object
 **/ 
//typedef const struct  
//{
//    eOvoidp_fp_void_t       fn_get_loc_storage;         /**< retrieves the pointer to the EOVstorageDerived object used to store permanent data of NVs */
//    eOvoid_fp_void_t        fn_init_volatile_data;      /**< Initialises the volatile data in all the NVs, both local and remote */
//    eOuint16_fp_uint16_t    fn_from_localport_to_index; 
//    EOconstarray*           nvsloc_port_id;           
//} eOnvs_NEW_cfg_t;
//
//typedef eOnvs_NEW_cfg_t eOnvs_cfg_t;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
 
/** @fn         extern EOtheNVs * eo_nvs_Initialise(const eOnvs_cfg_t * const cfg)
    @brief      Initialise the singleton EOtheNVs with a given configuration which keeps information of how
                the network variable are mapped into this specific device. 
    @param      cfg             A const pointer to the constant configuration data.
    @return     A valid and not-NULL const pointer to the EOtheNVs singleton. In case of invalid cfg the function 
                will call the error manager.
 **/
extern EOtheNVs * eo_nvs_Initialise(EOnvsCfg* cfg, EOVmutexDerived *mtx);


/** @fn         extern EOtheNVs * eo_nvs_GetHandle(void)
    @brief      Gets the handle of the EOtheNVs singleton 
    @return     Constant pointer to the singleton.
 **/
extern EOtheNVs * eo_nvs_GetHandle(void);


/** @fn         extern eOresult_t eo_nvs_Take(EOtheNVs *nvs, eOreltime_t tout)
    @brief      Takes the EOtheNVs singleton so that, for instance, writing in volatile data of the contained NVs can be protected 
    @param      nvs             The singleton
    @param      tout            The timeout
    @return     eores_OK in case of success. eores_NOK_timeout upon failure to take the mutex, or 
                or eores_NOK_nullpointer if argument or internal mutex is NULL.
 **/
extern eOresult_t eo_nvs_Take(EOtheNVs *nvs, eOreltime_t tout);


/** @fn         extern eOresult_t eo_nvs_Release(EOtheNVs *nvs)
    @brief      Releases the EOtheNVs singleton 
    @param      nvs             The singleton
    @return     eores_OK in case of success. osal_res_NOK_generic upon failure to release the mutex, or 
                or eores_NOK_nullpointer if argument or internal mutex is NULL.
 **/
extern eOresult_t eo_nvs_Release(EOtheNVs *nvs);


extern EOnetvarNode * eo_nvs_GetNVnodeByID(EOtheNVs *p, eOnvOwnership_t ownership, eOipv4addr_t ipaddr, eOipv4port_t port, eOnetvarID_t id);


extern EOnetvarNode * eo_nvs_GetNVnodeByIndex(EOtheNVs *p, eOnvOwnership_t ownership, eOipv4addr_t ipaddr, eOipv4port_t port, uint16_t index);


extern EOVstorage * eo_nvs_GetLocalStorage(EOtheNVs *p);


//extern eOresult_t eo_nvs_InitNVs(EOtheNVs *p,  eOnetvarOwnership_t ownership);




/** @}            
    end of group eo_thenvs  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



