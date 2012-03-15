
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EONVSCFG_H_
#define _EONVSCFG_H_



/** @file       EOnvsCfg.h
    @brief      This header file implements public interface to the configuration of netvars.
    @author     marco.accame@iit.it
    @date       09/06/2011
**/

/** @defgroup eo_n Objevct EOnvsCfg
    Offers methods for reading and writing some data on RAM, ROM and storage and for doing some
    actions upon such operations. It is to be manipulated only via its public methods.
    It is not created but loaded as a constant object from a configuration mapped in ROM
      
    @{        
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOnv.h"
#include "EOconstvector.h"
#include "EOtreenode.h"
#include "EOVmutex.h"
#include "EOVstorage.h"


// - public #define  --------------------------------------------------------------------------------------------------





  

// - declaration of public user-defined types -------------------------------------------------------------------------    


/** @typedef    typedef const struct EOnvsCfg_hid EOnvsCfg
    @brief      EOaction is an opaque struct. It is used to implement data abstraction for the datagram 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOnvsCfg_hid EOnvsCfg;


typedef enum
{
    eo_nvscfg_ownership_local       = 0,
    eo_nvscfg_ownership_remote      = 1
} eOnvscfgOwnership_t; 

typedef enum
{
    eo_nvscfg_devicesownership_none                  = 0,
    eo_nvscfg_devicesownership_onelocal              = 1,
    eo_nvscfg_devicesownership_onelocalsomeremote    = 2,
    eo_nvscfg_devicesownership_someremote            = 3
} eOnvscfgDevicesOwnership_t;



typedef struct                  // size is 16 bytes
{
    eOnvEP_t                    endpoint;
    uint16_t                    sizeof_endpoint_data;
    eOuint16_fp_uint16_t        hashfunction_id2index;
    const EOconstvector* const  constvector_of_treenodes_EOnv_con;
    const EOconstvector* const  constvector_of_EOnv_usr;
    eOvoid_fp_voidp_voidp_t     endpoint_data_init;
} eOnvscfg_EP_t;

    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------





// - declaration of extern public functions ---------------------------------------------------------------------------

extern EOnvsCfg* eo_nvscfg_New(uint16_t ndevices, EOVstorageDerived* stg);

extern eOresult_t eo_nvscfg_PushBackDevice(EOnvsCfg* p, eOnvscfgOwnership_t ownership, eOipv4addr_t ipaddress, eOuint16_fp_uint16_t hashfn_ep2index, uint16_t nendpoints);


extern uint8_t eo_nvscfg_GetIndexOfLocalDevice(EOnvsCfg* p);

extern eOresult_t eo_nvscfg_ondevice_PushBackEndpoint(EOnvsCfg* p, uint8_t ondevindex, eOnvEP_t endpoint, eOuint16_fp_uint16_t hashfn_id2index, const EOconstvector* treeofnvs_con, const EOconstvector* datanvs_usr, uint32_t datanvs_size, eOvoid_fp_voidp_voidp_t datanvs_init, EOVmutexDerived* mtx);


extern eOresult_t eo_nvscfg_data_Initialise(EOnvsCfg* p);

// removed. it was useful to retrieve ram of endpoin, but much better using the variable instead
//extern void * eo_nvscfg_localdev_endpoint_GetRAM(EOnvsCfg* p, eOnvEP_t endpoint);


extern eOresult_t eo_nvscfg_GetIndices(EOnvsCfg* p, 
                                    eOipv4addr_t ip, eOnvEP_t ep, eOnvID_t id, 
                                    uint8_t *ipindex, uint8_t *epindex, uint8_t *idindex); 


extern EOtreenode* eo_nvscfg_GetTreeNode(EOnvsCfg* p, uint8_t ondevindex, uint8_t onendpointindex, uint8_t onidindex);


extern EOnv* eo_nvscfg_GetNV(EOnvsCfg* p, uint8_t ondevindex, uint8_t onendpointindex, uint8_t onidindex, EOtreenode* treenode, EOnv* nvtarget);



/** @}            
    end of group eo_nv 
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

