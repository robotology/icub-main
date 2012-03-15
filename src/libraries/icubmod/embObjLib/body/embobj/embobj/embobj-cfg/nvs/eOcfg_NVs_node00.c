
/* @file       eOcfg_NVs_node00.c
    @brief      This file keeps constant configuration for the NVs of a device
    @author     marco.accame@iit.it
    @date       090/06/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h" 
#include "string.h"
#include "stdio.h"

#include "EoCommon.h"


#include "EOnetvar_hid.h"
#include "EOnetvarNode_hid.h"
#include "EOvport_hid.h"

#include "EOnetvar.h"

// the management of local device
#include "eOcfg_NVs_node00_loc.h"
// list of remote device which are managed
// none

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eOcfg_NVs_node00.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


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


//#define s_nvs_remote_devices_NUM    0
//static const eov_nvscfg_NVarray_t* s_nvs_remote_devices = NULL;





/* @var        s_theconfiguration
    @brief      It is the required configuration, which is the collection of previously defined 
                const variables.
    @details    Use the name s_theconfiguration.
 **/
static const eOnvs_cfg_t s_theconfiguration =    
{
    .fn_get_loc_storage                 = s_nvscfg_get_loc_storage,
    .fn_init_volatile_data              = s_nvscfg_init_volatile_data,
    .fn_from_ipv4_to_remotedev_index    = s_nvscfg_from_ipv4_to_remotedevice_index,
    .localnvs = 
    {
        .ipaddr = EOK_cfg_nvs_node00_ipaddr,  
        .nvnum  = EOK_cfg_nvs_node00_loc_NUMNETVARS,
        .nvars  = eo_cfg_nvs_node00_loc_thenetvarnodes
    },
    .remotenum              = 0,
    .remotenvs              = NULL, 
}; 


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------



extern eOnvs_cfg_t * eo_cfg_nvs_node00_Get(void)
{
    return(&s_theconfiguration);
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
    eo_cfg_nvs_node00_loc_init_volatile_data();


    // init the netvars owned in the remote devices 
    // none
}


static void * s_nvscfg_get_loc_storage(void)
{
    return(eo_cfg_nvs_node00_loc_get_storage());
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



