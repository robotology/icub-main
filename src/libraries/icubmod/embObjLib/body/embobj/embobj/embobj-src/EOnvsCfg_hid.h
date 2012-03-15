
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EONVSCFG_HID_H_
#define _EONVSCFG_HID_H_


/* @file       EOnvsCfg_hid.h
    @brief      This header file implements hidden interface to a netvar object.
    @author     marco.accame@iit.it
    @date       09/06/2011
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOnv_hid.h"
#include "EOvector.h"
#include "EOconstvector.h"
#include "EOVmutex.h"
#include "EOmatrix3d.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOnvsCfg.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------


// - definition of the hidden struct implementing the object ----------------------------------------------------------

typedef struct
{
    eOnvEP_t                endpoint;
    eObool_t                initted;
    uint8_t                 thenvs_numberof; 
    const EOconstvector*    thetreeofnvs_con;      
    const EOconstvector*    thenvs_usr;
    void*                   thenvs_vol;
    void*                   thenvs_rem;
    eOvoid_fp_voidp_voidp_t thenvs_initialise; // vol_local, vol_remote
    uint32_t                thenvs_sizeof;   
    eOuint16_fp_uint16_t    hashfn_id2index; 
    EOVmutexDerived*        mtx_endpoint;                 
} EOnvsCfg_ep_t;

typedef struct
{
    uint16_t                index;
    eOnvEP_t                ephashed;
} eOnvsCfgEPhash_t;

typedef struct
{
    eOipv4addr_t            ipaddress;
    EOvector*               theendpoints;
    eOnvscfgOwnership_t     ownership;
    uint16_t                theendpoints_numberof; 
    eOnvsCfgEPhash_t*       ephashtable;  
    eOuint16_fp_uint16_t    hashfn_ep2index;    
} EOnvsCfg_device_t;



/** @struct     EOnvsCfg_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/
struct EOnvsCfg_hid 
{
    EOvector*                   thedevices;
    EOvector*                   ip2index;
    uint8_t                     indexoflocaldevice;
    eOnvscfgDevicesOwnership_t  devicesowneship;
    EOVstorageDerived*          storage;
    EOmatrix3d*                 allnvs;
};   
 



// - declaration of extern hidden functions ---------------------------------------------------------------------------

extern uint8_t eo_nvscfg_hid_ip2index(EOnvsCfg* p, eOipv4addr_t ipaddress);

extern uint8_t eo_nvscfg_hid_ondevice_endpoint2index(EOnvsCfg* p, uint8_t ondevindex, eOnvEP_t endpoint);

extern uint8_t eo_nvscfg_hid_ondevice_onendpoint_id2index(EOnvsCfg* p, uint8_t ondevindex, uint8_t onendpointindex, eOnvID_t id);

extern EOtreenode* eo_nvscfg_hid_ondevice_onendpoint_withID_GetTreeNode(EOnvsCfg* p, uint8_t ondevindex, uint8_t onendpointindex, eOnvID_t id);



 

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




