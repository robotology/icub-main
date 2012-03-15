
/** @file       eOcfg_NVs_node00_loc.c
    @brief      This file implements internal implementation to the configuration of the NVs of a device
    @author     marco.accame@iit.it
    @date       09/06/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOnetvar_hid.h"
#include "EOnetvar.h"
#include "stdio.h"
#include "EOvport_hid.h"
#include "EOnetvarNode_hid.h"

#include "EOfakeStorage.h"
#include "EOVstorage.h"
#include "eOcfg_NVs_node00.h"


#include "eOcfg_NVs_node00_loc_act.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eOcfg_NVs_node00_loc.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

#define node00_loc_addrPER_root                         (0)

#define node00_loc_addrPER_vportRO__cfg                 (0)

#define node00_loc_addrPER_vportWO__cfg                 (sizeof(EOvport))

#define node00_loc_addrPER_globalconfiguration          (2*sizeof(EOvport)+sizeof(eOcfg_nvs_node00_loc_globalconstants_t))
#define node00_loc_addrPER_globcfg__acquireinput        (node00_loc_addrPER_globalconfiguration+0)                           
#define node00_loc_addrPER_globcfg__acquisitionperiod   (node00_loc_addrPER_globcfg__acquireinput+1)                   
#define node00_loc_addrPER_globcfg__applyoutput         (node00_loc_addrPER_globcfg__acquisitionperiod+4)                  
#define node00_loc_addrPER_globcfg__signalvportro       (node00_loc_addrPER_globcfg__applyoutput+1)                   
#define node00_loc_addrPER_globcfg__toipaddr            (node00_loc_addrPER_globcfg__signalvportro+1)                   
#define node00_loc_addrPER_globcfg__withperiod          (node00_loc_addrPER_globcfg__toipaddr+4)



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

extern eOcfg_nvs_node00_loc_t eo_cfg_nvs_node00_loc_vol = { 0 };

// this struct contains the default values of the entities of the whole device

extern const eOcfg_nvs_node00_loc_t eo_cfg_nvs_node00_loc_def =
{
    .vportRO                            =
    {
        .cfg =
        {   // USER-TODO: define the number of nvIDs and put the same in .sizeofarray and .n
            .sizeofarray = EOVPORT_GET_SizeOfArray(3),
            .n = 3,
            .nvids =  
            {   // USER-TODO: put the nvIDs in here ... or just one 0
                EOK_cfg_nvs_node00_loc_nvID_button__inputval, 
                EOK_cfg_nvs_node00_loc_nvID_led00__outputval,
                EOK_cfg_nvs_node00_loc_nvID_led01__outputval
            }
        },
        .dat =
        {   // always initted to zero
            .sizeofarray = EOVPORT_GET_SizeOfArray(0),
            .n = 0,
            .buffer_of_nvidsizeinfo = {0}
        },
        .mirrors = {0} // always initted to zero
    },

#warning -> i dont think that vportWO__cfg is ever used in a local node because it is used the __dat instead    
    .vportWO                         =
    {
        .cfg =
        {   // always initted to zero
            .sizeofarray = EOVPORT_GET_SizeOfArray(0),
            .n = 0,
            .nvids = {0}
        },
        .dat =
        {   // always initted to zero
            .sizeofarray = EOVPORT_GET_SizeOfArray(0),
            .n = 0,
            .buffer_of_nvidsizeinfo = {0}
        },
        .mirrors = {0}  // always initted to zero
    },    
    .globalconstants                    =
    {
        .macaddr    = EO_COMMON_MACADDR(0x11, 0x22, 0x33, 0x00, 0x00, 0),
        .ipaddr     = EOK_cfg_nvs_node00_ipaddr
    },
    .globalconfiguration                =
    {
        .acquireinput       = 1,
        .acquisitionperiod  = 100*1000*1000,
        .applyoutput        = 1,
        .signalvportro      = 0,
        .toipaddr           = EO_COMMON_IPV4ADDR(10, 1, 1, 100),
        .withperiod         = 100*1000*1000
    },
    .button                             =
    {
        .inputval           = 0,
        .acquisitiontime    = 0
    
    },
    .led00                              =
    {
        .outputval          = 1,
        .lednumber          = 8     // PE8 on mcbstm32c
    },
    .led01                              =
    {
        .outputval          = 0,
        .lednumber          = 9     // PE9 on mcbstm32c    
    },
    .timeoflife = 0
 };


// now we have the netvars


const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_root =
{   // pos =  0
    .id                 = EOK_cfg_nvs_node00_loc_nvID_root,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER  // no need to put node00_loc_addrPER_root  
    },
    .peripheralinterface      = &eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_root,
    .on_rop_reception   = NULL
};


const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_vportRO =
{   // pos =  1
    .id                 = EOK_cfg_nvs_node00_loc_nvID_vportRO,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.vportRO),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.vportRO, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.vportRO,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .peripheralinterface      = NULL,
    .on_rop_reception   = NULL
};


const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_vportRO__cfg =
{   // pos =  2
    .id                 = EOK_cfg_nvs_node00_loc_nvID_vportRO__cfg,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.vportRO.cfg),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.vportRO.cfg, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.vportRO.cfg,
        .mirror             = NULL,
        .flex.loc_addrperm  = node00_loc_addrPER_vportRO__cfg   // must store it   
    },
    .peripheralinterface      = &eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_vportRO__cfg,
    .on_rop_reception   = NULL
};


const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_vportRO__dat =
{   // pos =  3
    .id                 = EOK_cfg_nvs_node00_loc_nvID_vportRO__dat,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.vportRO.dat),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.vportRO.dat, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.vportRO.dat,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .peripheralinterface      = &eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_vportRO__dat,
    .on_rop_reception   = NULL
};


const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_vportWO =
{   // pos =  4
    .id                 = EOK_cfg_nvs_node00_loc_nvID_vportWO,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.vportWO),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.vportWO, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.vportWO,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .peripheralinterface      = NULL,
    .on_rop_reception   = NULL
};


const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_vportWO__cfg =
{   // pos =  5
    .id                 = EOK_cfg_nvs_node00_loc_nvID_vportWO__cfg,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.vportWO.cfg),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.vportWO.cfg, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.vportWO.cfg,
        .mirror             = NULL,
        .flex.loc_addrperm  = node00_loc_addrPER_vportWO__cfg    
    },
    .peripheralinterface      = NULL,
    .on_rop_reception   = NULL
};


const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_vportWO__dat =
{   // pos =  6
    .id                 = EOK_cfg_nvs_node00_loc_nvID_vportWO__dat,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.vportWO.dat),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.vportWO.dat, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.vportWO.dat,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .peripheralinterface      = NULL,
    .on_rop_reception   = NULL
};

const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_globalconstants =
{   // pos =  7
    .id                 = EOK_cfg_nvs_node00_loc_nvID_globalconstants,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eOcfg_nvs_node00_loc_globalconstants_t),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.globalconstants, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.globalconstants,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER   // not need to store it in eeprom as it is just constant   
    },
    .peripheralinterface      = NULL,
    .on_rop_reception   = NULL
};

const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_globcon__macaddr =
{   // pos =  8
    .id                 = EOK_cfg_nvs_node00_loc_nvID_globcon__macaddr,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.globalconstants.macaddr),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.globalconstants.macaddr, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.globalconstants.macaddr,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .peripheralinterface      = NULL,
    .on_rop_reception   = NULL
};

const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_globcon__ipaddr =
{   // pos =  9
    .id                 = EOK_cfg_nvs_node00_loc_nvID_globcon__ipaddr,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.globalconstants.ipaddr),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.globalconstants.ipaddr, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.globalconstants.ipaddr,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .peripheralinterface      = NULL,
    .on_rop_reception   = NULL
};


const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_globalconfiguration =
{   // pos =  10
    .id                 = EOK_cfg_nvs_node00_loc_nvID_globalconfiguration,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.globalconfiguration),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.globalconfiguration, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.globalconfiguration,
        .mirror             = NULL,
        .flex.loc_addrperm  = node00_loc_addrPER_globalconfiguration    
    },
    .peripheralinterface      = &eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_globalconfiguration__any,
    .on_rop_reception   = NULL
};


const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_globcfg__acquireinput =
{   // pos =  11
    .id                 = EOK_cfg_nvs_node00_loc_nvID_globcfg__acquireinput,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.globalconfiguration.acquireinput),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.globalconfiguration.acquireinput, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.globalconfiguration.acquireinput,
        .mirror             = NULL,
        .flex.loc_addrperm  = node00_loc_addrPER_globcfg__acquireinput    
    },
    .peripheralinterface      = &eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_globalconfiguration__any,
    .on_rop_reception   = NULL
};


const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_globcfg__acquisitionperiod =
{   // pos =  12
    .id                 = EOK_cfg_nvs_node00_loc_nvID_globcfg__acquisitionperiod,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.globalconfiguration.acquisitionperiod),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.globalconfiguration.acquisitionperiod, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.globalconfiguration.acquisitionperiod,
        .mirror             = NULL,
        .flex.loc_addrperm  = node00_loc_addrPER_globcfg__acquisitionperiod    
    },
    .peripheralinterface      = &eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_globalconfiguration__any,
    .on_rop_reception   = NULL
};

const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_globcfg__applyoutput =
{   // pos =  13
    .id                 = EOK_cfg_nvs_node00_loc_nvID_globcfg__applyoutput,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.globalconfiguration.applyoutput),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.globalconfiguration.applyoutput, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.globalconfiguration.applyoutput,
        .mirror             = NULL,
        .flex.loc_addrperm  = node00_loc_addrPER_globcfg__applyoutput    
    },
    .peripheralinterface      = &eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_globalconfiguration__any,
    .on_rop_reception   = NULL
};

const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_globcfg__signalvportro =
{   // pos =  14
    .id                 = EOK_cfg_nvs_node00_loc_nvID_globcfg__signalvportro,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.globalconfiguration.signalvportro),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.globalconfiguration.signalvportro, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.globalconfiguration.signalvportro,
        .mirror             = NULL,
        .flex.loc_addrperm  = node00_loc_addrPER_globcfg__signalvportro    
    },
    .peripheralinterface      = &eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_globalconfiguration__any,
    .on_rop_reception   = NULL
};

const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_globcfg__toipaddr =
{   // pos =  15
    .id                 = EOK_cfg_nvs_node00_loc_nvID_globcfg__toipaddr,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.globalconfiguration.toipaddr),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.globalconfiguration.toipaddr, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.globalconfiguration.toipaddr,
        .mirror             = NULL,
        .flex.loc_addrperm  = node00_loc_addrPER_globcfg__toipaddr    
    },
    .peripheralinterface      = &eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_globalconfiguration__any,
    .on_rop_reception   = NULL
};

const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_globcfg__withperiod =
{   // pos =  16
    .id                 = EOK_cfg_nvs_node00_loc_nvID_globcfg__withperiod,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.globalconfiguration.withperiod),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.globalconfiguration.withperiod, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.globalconfiguration.withperiod,
        .mirror             = NULL,
        .flex.loc_addrperm  = node00_loc_addrPER_globcfg__withperiod    
    },
    .peripheralinterface      = &eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_globalconfiguration__any,
    .on_rop_reception   = NULL
};

const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_button =
{   // pos =  17
    .id                 = EOK_cfg_nvs_node00_loc_nvID_button,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.button),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.button, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.button,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .peripheralinterface      = &eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_button,
    .on_rop_reception   = NULL
};


const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_button__inputval =
{   // pos =  18
    .id                 = EOK_cfg_nvs_node00_loc_nvID_button,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.button.inputval),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.button.inputval, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.button.inputval,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .peripheralinterface      = &eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_button__inputval,
    .on_rop_reception   = NULL
};

const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_button__acquisitiontime =
{   // pos =  19
    .id                 = EOK_cfg_nvs_node00_loc_nvID_button,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.button.acquisitiontime),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.button.acquisitiontime, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.button.acquisitiontime,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .peripheralinterface      = &eo_cfg_nvs_node00_loc_action_peripheralinterface_generic,
    .on_rop_reception   = NULL
};

const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_led00 =
{   // pos =  20
    .id                 = EOK_cfg_nvs_node00_loc_nvID_led00,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.led00),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.led00, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.led00,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .peripheralinterface      = &eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_led00,
    .on_rop_reception   = NULL
};


const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_led00__outputval =
{   // pos =  21
    .id                 = EOK_cfg_nvs_node00_loc_nvID_led00__outputval,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.led00.outputval),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.led00.outputval, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.led00.outputval,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .peripheralinterface      = &eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_led00__outputval,
    .on_rop_reception   = NULL
};


const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_led00__lednumber =
{   // pos =  22
    .id                 = EOK_cfg_nvs_node00_loc_nvID_led00__lednumber,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.led00.lednumber),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.led00.lednumber, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.led00.lednumber,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .peripheralinterface      = NULL,
    .on_rop_reception   = NULL
};

const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_led01 =
{   // pos =  23
    .id                 = EOK_cfg_nvs_node00_loc_nvID_led01,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.led01),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.led01, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.led01,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .peripheralinterface      = &eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_led01,
    .on_rop_reception   = NULL
};


const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_led01__outputval =
{   // pos =  24
    .id                 = EOK_cfg_nvs_node00_loc_nvID_led01__outputval,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.led01.outputval),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.led01.outputval, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.led01.outputval,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .peripheralinterface      = &eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_led01__outputval,
    .on_rop_reception   = NULL
};


const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_led01__lednumber =
{   // pos =  25
    .id                 = EOK_cfg_nvs_node00_loc_nvID_led01__lednumber,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.led01.lednumber),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.led01.lednumber, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.led01.lednumber,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .peripheralinterface    = NULL,
    .on_rop_reception       = NULL
};


const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ__timeoflife =
{   // pos =  26
    .id                 = EOK_cfg_nvs_node00_loc_nvID__timeoflife,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eo_cfg_nvs_node00_loc_def.timeoflife),
        .valuedef           = (void*)&eo_cfg_nvs_node00_loc_def.timeoflife, 
        .valuevol           = (void*)&eo_cfg_nvs_node00_loc_vol.timeoflife,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .peripheralinterface    = &eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ__timeoflife,
    .on_rop_reception       = NULL
};


// this array contains the tree-structured netvars of the device. 
// they are ordered upon a depth first reading of the tree

const EOnetvarNode eo_cfg_nvs_node00_loc_thenetvarnodes[] =
{
{   // pos = 0
    .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_root,
    .nchildren  = 8,
    .children   = {1, 4, 7, 10, 17, 20, 23, 26}     
 },

/**/{   // pos = 1
        .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_vportRO,
        .nchildren  = 2,
        .children   = {2, 3}
    },
        {   // pos = 2
            .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_vportRO__cfg,
            .nchildren  = 0,
            .children   = {255}
        },
        {   // pos = 3
            .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_vportRO__dat,
            .nchildren  = 0,
            .children   = {255}
        },

/**/{   // pos = 4
        .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_vportWO,
        .nchildren  = 2,
        .children   = {5, 6}
    },
        {   // pos = 5
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_vportWO__cfg
        },
        {   // pos = 6
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_vportWO__dat
        },

/**/{   // pos = 7
        .nchildren  = 2,
        .children   = {8, 9},
        .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_globalconstants
    },
        {   // pos = 8
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_globcon__macaddr
        },
        {   // pos = 9
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_globcon__ipaddr
        },

/**/{   // pos = 10
        .nchildren  = 6,
        .children   = {11, 12, 13, 14, 15, 16},
        .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_globalconfiguration
    },
        {   // pos = 11
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_globcfg__acquireinput
        },
        {   // pos = 12
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_globcfg__acquisitionperiod
        },
        {   // pos = 13
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_globcfg__applyoutput
        },
        {   // pos = 14
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_globcfg__signalvportro
        },
        {   // pos = 15
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_globcfg__toipaddr
        },
        {   // pos = 16
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_globcfg__withperiod
        },


/**/{   // pos = 17
        .nchildren  = 2,
        .children   = {18, 19},
        .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_button
    },
        {   // pos = 18
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_button__inputval
        },
        {   // pos = 19
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_button__acquisitiontime
        },
        
/**/{   // pos = 20
        .nchildren  = 2,
        .children   = {21, 22},
        .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_led00
    },
        {   // pos = 21
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_led00__outputval
        },
        {   // pos = 22
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_led00__lednumber
        },        
        
/**/{   // pos = 23
        .nchildren  = 2,
        .children   = {24, 25},
        .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_led01
    },
        {   // pos = 24
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_led01__outputval
        },
        {   // pos = 25
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ_led01__lednumber
        },          

/**/{   // pos = 26
        .nchildren  = 0,
        .children   = {255},
        .netvar     = &eo_cfg_nvs_node00_loc_nvOBJ__timeoflife
    }

};



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
// empty-section

typedef uint8_t s_loc_dummy_t[((sizeof(eo_cfg_nvs_node00_loc_thenetvarnodes)/sizeof(EOnetvarNode)-EOK_cfg_nvs_node00_loc_NUMNETVARS) == 0) ? 1 : 0];


static EOfakeStorage *s_loc_storage = NULL;


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern void eo_cfg_nvs_node00_loc_init_volatile_data(void)
{
    uint16_t i = 0;

    // 1. init the eeprom. if never initialised, then set it to contain teh default value eo_cfg_nvs_node00_loc_def
    s_loc_storage = eo_fakestrg_New(EOK_cfg_nvs_node00_loc_STGid, sizeof(eo_cfg_nvs_node00_loc_def), &eo_cfg_nvs_node00_loc_def, NULL);

    // 2. copy eeprom into ram
    eov_strg_Get(s_loc_storage, node00_loc_addrPER_root, sizeof(eo_cfg_nvs_node00_loc_vol), &eo_cfg_nvs_node00_loc_vol);

    // 3. initialise every NV.
    for(i=0; i<EOK_cfg_nvs_node00_loc_NUMNETVARS; i++)
    {
        eo_netvar_Init(eo_cfg_nvs_node00_loc_thenetvarnodes[i].netvar);
    }

}


extern void * eo_cfg_nvs_node00_loc_get_storage(void)
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



