
/** @file       EOtheNVsCfgNode00.c
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

// the management of local device
#include "EOtheNVsCfgNode00Loc.h"
#include "EOtheNVsCfgNode00Loc_hid.h"
// list of remote device which are managed
// none

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgNode00.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgNode00_hid.h" 


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


#define s_nvs_remote_devices_NUM    0
//static const eov_nvscfg_NVarray_t* s_nvs_remote_devices = NULL;





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
        .ipaddr = node00_loc_ipaddr,  
        .nvnum  = node00_loc_NUMNETVARS,
        .nvars  = node00_loc_thenetvarnodes
    },
    .remotenum              = s_nvs_remote_devices_NUM,
    .remotenvs              = NULL, 
}; 



static const EOtheNVsCfgNode00 zzz =
{
    .eoidentifier = EOIDNVSCONFIG,                  // eoidentifier
    .theconfig = &s_theconfiguration_device         // theconfig
};


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------



extern const EOtheNVsCfgNode00 * eo_nvscfg_node00_GetHandle(void)
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
    node00_loc_init_volatile_data();


    // init the netvars owned in the remote devices 
    // none
}


static void * s_nvscfg_get_loc_storage(void)
{
    return(node00_loc_get_storage());
}


static uint16_t s_nvscfg_from_ipv4_to_remotedevice_index(eOipv4addr_t ipv4)
{   // there is no remote device, thus the function always returns 0xffff
//    uint16_t ret = 0;
//    
//    // place in here a look-up-table or a rule to have the index from the ipv4 of the remote device
//
//    for(ret=0; ret<zzz.theconfig->remotenum; ret++)
//    {
//        if(ipv4 == zzz.theconfig->remotenvs[ret].ipaddr)
//        {
//            return(ret);
//        }
//    }

    return(0xffff);
}


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



