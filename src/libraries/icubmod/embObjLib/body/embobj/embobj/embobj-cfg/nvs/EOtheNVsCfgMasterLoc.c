
/** @file       EOtheNVsCfgMasterLoc.c
    @brief      This file implements internal implementation to the configuration of the NVs of a device
    @author     marco.accame@iit.it
    @date       10/15/2009
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOVtheNVsCfg_hid.h"
#include "EOnetvar_hid.h"
#include "EOnetvar.h"
#include "stdio.h"
#include "EOvport_hid.h"
#include "EOnetvarNode_hid.h"

#include "EOfakeStorage.h"
#include "EOVstorage.h"


#include "EOtheNVsCfgMasterLocActions.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgMasterLoc.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgMasterLoc_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------


// this structs contains the volatile values of the entities of the whole device

extern eOmaster_loc_t master_loc_vol = { 0 };

// this struct contains the default values of the entities of the whole device

extern const eOmaster_loc_t master_loc_def =
{
    .globalconstants                    =
    {
        .macaddr    = EO_COMMON_MACADDR(0x11, 0x22, 0x33, 0x00, 0x00, 100),
        .ipaddr     = master_loc_ipaddr
    },
    .globalconfiguration                =
    {
        .doit       = 1,
        .doalsothat = 1
    },
    .isactive       = 1
};



// now we have the netvars


const EOnetvar master_loc_nvOBJ_root =
{   // pos =  0
    .id                 = master_loc_nvID_root,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eOmaster_loc_t),
        .valuedef           = (void*)&master_loc_def, 
        .valuevol           = (void*)&master_loc_vol,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = NULL,
    .on_rop_reception   = NULL
};


const EOnetvar master_loc_nvOBJ_globalconstants =
{   // pos =  1
    .id                 = master_loc_nvID_globalconstants,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eOmaster_loc_globalconstants_t),
        .valuedef           = (void*)&master_loc_def.globalconstants, 
        .valuevol           = (void*)&master_loc_vol.globalconstants,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = NULL,
    .on_rop_reception   = NULL
};

const EOnetvar master_loc_nvOBJ_globcon__macaddr =
{   // pos =  2
    .id                 = master_loc_nvID_globcon__macaddr,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(uint64_t),
        .valuedef           = (void*)&master_loc_def.globalconstants.macaddr, 
        .valuevol           = (void*)&master_loc_vol.globalconstants.macaddr,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = NULL,
    .on_rop_reception   = NULL
};

const EOnetvar master_loc_nvOBJ_globcon__ipaddr =
{   // pos =  3
    .id                 = master_loc_nvID_globcon__ipaddr,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(uint32_t),
        .valuedef           = (void*)&master_loc_def.globalconstants.ipaddr, 
        .valuevol           = (void*)&master_loc_vol.globalconstants.ipaddr,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = NULL,
    .on_rop_reception   = NULL
};


const EOnetvar master_loc_nvOBJ_globalconfiguration =
{   // pos =  4
    .id                 = master_loc_nvID_globalconfiguration,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eOmaster_loc_globalconfiguration_t),
        .valuedef           = (void*)&master_loc_def.globalconfiguration, 
        .valuevol           = (void*)&master_loc_vol.globalconfiguration,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = NULL,
    .on_rop_reception   = NULL
};


const EOnetvar master_loc_nvOBJ_globcfg__doit =
{   // pos =  5
    .id                 = master_loc_nvID_globcfg__doit,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(uint8_t),
        .valuedef           = (void*)&master_loc_def.globalconfiguration.doit, 
        .valuevol           = (void*)&master_loc_vol.globalconfiguration.doit,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = NULL,
    .on_rop_reception   = NULL
};


const EOnetvar master_loc_nvOBJ_globcfg__doalsothat =
{   // pos =  6
    .id                 = master_loc_nvID_globcfg__doalsothat,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(uint32_t),
        .valuedef           = (void*)&master_loc_def.globalconfiguration.doalsothat, 
        .valuevol           = (void*)&master_loc_vol.globalconfiguration.doalsothat,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = NULL,
    .on_rop_reception   = NULL
};


const EOnetvar master_loc_nvOBJ__isactive =
{   // pos =  7
    .id                 = master_loc_nvID__isactive,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(uint8_t),
        .valuedef           = (void*)&master_loc_def.isactive, 
        .valuevol           = (void*)&master_loc_vol.isactive,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = NULL,
    .on_rop_reception   = NULL
};



// this array contains the tree-structured netvars of the device. 
// they are ordered upon a depth first reading of the tree

const EOnetvarNode master_loc_thenetvarnodes[] =
{
{   // pos = 0
    .nchildren  = 3,
    .children   = {1, 4, 7},     
    .netvar     = &master_loc_nvOBJ_root
},
    
/**/{   // pos = 1
        .nchildren  = 2,
        .children   = {2, 3},
        .netvar     = &master_loc_nvOBJ_globalconstants
    },
        {   // pos = 2
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &master_loc_nvOBJ_globcon__macaddr
        },
        {   // pos = 3
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &master_loc_nvOBJ_globcon__ipaddr
        },

/**/{   // pos = 4
        .nchildren  = 2,
        .children   = {5, 6},
        .netvar     = &master_loc_nvOBJ_globalconfiguration
    },
        {   // pos = 5
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &master_loc_nvOBJ_globcfg__doit
        },
        {   // pos = 6
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &master_loc_nvOBJ_globcfg__doalsothat
        },

/**/{   // pos = 7
        .nchildren  = 0
        .children   = {255},
        .netvar     = &master_loc_nvOBJ__isactive
    }

};



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
// empty-section

typedef uint8_t s_loc_dummy_t[((sizeof(master_loc_thenetvarnodes)/sizeof(EOnetvarNode)-master_loc_NUMNETVARS) == 0) ? 1 : 0];


static EOfakeStorage *s_loc_storage = NULL;


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern void master_loc_init_volatile_data(void)
{
    uint16_t i = 0;

    // 1. init the eeprom. if never initialised, then set it to contain teh default value master_loc_def
    s_loc_storage = eo_fakestrg_New(master_loc_storage_ID, sizeof(master_loc_def), &master_loc_def);

    // 2. copy eeprom into ram. the offset is zero because we get the root.
    #warning: fix the offset of storage and assign in here master_loc_storage_offNV_root

    eov_strg_Get(s_loc_storage, master_loc_addrPER_root, sizeof(master_loc_vol), &master_loc_vol);

    // 3. initialise every NV.
    for(i=0; i<master_loc_NUMNETVARS; i++)
    {
        eo_netvar_Init(master_loc_thenetvarnodes[i].netvar);
    }

}


extern void * nvscfg_device_loc_get_storage(void)
{
    return(s_loc_storage);    
}









// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------






// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



