
/** @file       EOtheNVsCfgDevice.c
    @brief      This file implements internal implementation to the configuration of the NVs of a device
    @author     marco.accame@iit.it
    @date       10/15/2009
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h" 
#include "string.h"
#include "stdio.h"

#include "EoCommon.h"

#include "EOVtheNVsCfg_hid.h"
#include "EOnetvar_hid.h"
#include "EOnetvarNode_hid.h"
#include "EOvport_hid.h"

#include "EOnetvar.h"


#include "EOtheNVsCfgDeviceLoc.h"
#include "EOtheNVsCfgDeviceLoc_hid.h"
// list of remote device which are managed
#include "EOtheNVsCfgDeviceRem00.h"
#include "EOtheNVsCfgDeviceRem00_hid.h"
//#include "EOtheNVsCfgDeviceRem01.h"
//#include "EOtheNVsCfgDeviceRem02_hid.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgDevice.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgDevice_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section





// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_nvscfg_init_volatile_data(void);
static void * s_nvscfg_get_loc_storage(void);
static uint16_t s_nvscfg_from_ipv4_to_remotedevice_index(eOipv4addr_t ipv4);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


#define s_nvs_remote_devices_NUM    2

static const eov_nvscfg_NVarray_t s_nvs_remote_devices[s_nvs_remote_devices_NUM] = 
{
    {   // index 0: ip = ...
        .ipaddr = rem00_device_ipaddr,
        .nvnum  = rem00_device_NUMNETVARS,
        .nvars  = rem00_device_thenetvarnodes
    }//,
//    {   // index 1: ip = ...
//        .ipaddr = rem01_device_ipaddr,
//        .nvnum  = rem01_device_NUMNETVARS,
//        .nvars  = rem01_device_thenetvarnodes
//    }
};




/** @var        s_theconfiguration
    @brief      It is the required configuration, which is the collection of previously defined 
                const variables.
    @details    Use the name s_theconfiguration.
 **/
static const EOVtheNVsCfg s_theconfiguration_device =    
{
    .fn_get_loc_storage                 = s_nvscfg_get_loc_storage,
    .fn_init_volatile_data              = s_nvscfg_init_volatile_data,
    .fn_from_ipv4_to_remotedev_index    = s_nvscfg_from_ipv4_to_remotedevice_index,
    .localnvs = 
    {
        .ipaddr = loc_device_ipaddr,  
        .nvnum  = loc_device_NUMNETVARS,
        .nvars  = loc_device_thenetvarnodes
    },
    .remotenum              = s_nvs_remote_devices_NUM,
    .remotenvs              = s_nvs_remote_devices, 
}; 



static const EOtheNVsCfgDevice zzz =
{
    .eoidentifier = EOIDNVSCONFIG,                  // eoidentifier
    .theconfig = &s_theconfiguration_device      // theconfig
};


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------



extern const EOtheNVsCfgDevice * eo_nvscfg_device_GetHandle(void)
{
    return(&zzz);
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------




static void s_nvscfg_init_volatile_data(void)
{
    // init the netvars owned in local by the device
    nvscfg_device_loc_init_volatile_data();


    // init the netvars owned in the remote devices 

    nvscfg_device_rem00_init_volatile_data();

//    nvscfg_device_rem01_init_volatile_data();
}


static void * s_nvscfg_get_loc_storage(void)
{
    return(nvscfg_device_loc_get_storage());
}


static uint16_t s_nvscfg_from_ipv4_to_remotedevice_index(eOipv4addr_t ipv4)
{
    uint16_t ret = 0;
    
    // place in here a look-up-table or a rule to have the index from the ipv4 of the remote device

    for(ret=0; ret<zzz.theconfig->remotenum; ret++)
    {
        if(ipv4 == zzz.theconfig->remotenvs[ret].ipaddr)
        {
            return(ret);
        }
    }

    return(0xffff);

}


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



