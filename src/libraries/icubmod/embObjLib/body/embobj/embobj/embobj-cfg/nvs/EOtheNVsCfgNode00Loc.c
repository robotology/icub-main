
/** @file       EOtheNVsCfgNode00Loc.c
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


#include "EOtheNVsCfgNode00LocActions.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgNode00Loc.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgNode00Loc_hid.h" 


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

extern eOnode00_loc_t node00_loc_vol = { 0 };

// this struct contains the default values of the entities of the whole device

extern const eOnode00_loc_t node00_loc_def =
{
    .vportRO                            =
    {
        .cfg =
        {   // USER-TODO: define the number of nvIDs and put the same in .sizeofarray and .n
            .sizeofarray = EOVPORT_GET_SizeOfArray(3),
            .n = 3,
            .nvids =  
            {   // USER-TODO: put the nvIDs in here ... or just one 0
                node00_loc_nvID_button__inputval, 
                node00_loc_nvID_led00__outputval,
                node00_loc_nvID_led01__outputval
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
        .ipaddr     = node00_loc_ipaddr
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


const EOnetvar node00_loc_nvOBJ_root =
{   // pos =  0
    .id                 = node00_loc_nvID_root,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def),
        .valuedef           = (void*)&node00_loc_def, 
        .valuevol           = (void*)&node00_loc_vol,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER  // no need to put node00_loc_addrPER_root  
    },
    .datainterface      = &node00_loc_action_datainterface_nvOBJ_root,
    .on_rop_reception   = NULL
};


const EOnetvar node00_loc_nvOBJ_vportRO =
{   // pos =  1
    .id                 = node00_loc_nvID_vportRO,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.vportRO),
        .valuedef           = (void*)&node00_loc_def.vportRO, 
        .valuevol           = (void*)&node00_loc_vol.vportRO,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = NULL,
    .on_rop_reception   = NULL
};


const EOnetvar node00_loc_nvOBJ_vportRO__cfg =
{   // pos =  2
    .id                 = node00_loc_nvID_vportRO__cfg,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.vportRO.cfg),
        .valuedef           = (void*)&node00_loc_def.vportRO.cfg, 
        .valuevol           = (void*)&node00_loc_vol.vportRO.cfg,
        .mirror             = NULL,
        .flex.loc_addrperm  = node00_loc_addrPER_vportRO__cfg   // must store it   
    },
    .datainterface      = &node00_loc_action_datainterface_nvOBJ_vportRO__cfg,
    .on_rop_reception   = NULL
};


const EOnetvar node00_loc_nvOBJ_vportRO__dat =
{   // pos =  3
    .id                 = node00_loc_nvID_vportRO__dat,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.vportRO.dat),
        .valuedef           = (void*)&node00_loc_def.vportRO.dat, 
        .valuevol           = (void*)&node00_loc_vol.vportRO.dat,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = &node00_loc_action_datainterface_nvOBJ_vportRO__dat,
    .on_rop_reception   = NULL
};


const EOnetvar node00_loc_nvOBJ_vportWO =
{   // pos =  4
    .id                 = node00_loc_nvID_vportWO,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.vportWO),
        .valuedef           = (void*)&node00_loc_def.vportWO, 
        .valuevol           = (void*)&node00_loc_vol.vportWO,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = NULL,
    .on_rop_reception   = NULL
};


const EOnetvar node00_loc_nvOBJ_vportWO__cfg =
{   // pos =  5
    .id                 = node00_loc_nvID_vportWO__cfg,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.vportWO.cfg),
        .valuedef           = (void*)&node00_loc_def.vportWO.cfg, 
        .valuevol           = (void*)&node00_loc_vol.vportWO.cfg,
        .mirror             = NULL,
        .flex.loc_addrperm  = node00_loc_addrPER_vportWO__cfg    
    },
    .datainterface      = NULL,
    .on_rop_reception   = NULL
};


const EOnetvar node00_loc_nvOBJ_vportWO__dat =
{   // pos =  6
    .id                 = node00_loc_nvID_vportWO__dat,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.vportWO.dat),
        .valuedef           = (void*)&node00_loc_def.vportWO.dat, 
        .valuevol           = (void*)&node00_loc_vol.vportWO.dat,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = NULL,
    .on_rop_reception   = NULL
};

const EOnetvar node00_loc_nvOBJ_globalconstants =
{   // pos =  7
    .id                 = node00_loc_nvID_globalconstants,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(eOnode00_loc_globalconstants_t),
        .valuedef           = (void*)&node00_loc_def.globalconstants, 
        .valuevol           = (void*)&node00_loc_vol.globalconstants,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER   // not need to store it in eeprom as it is just constant   
    },
    .datainterface      = NULL,
    .on_rop_reception   = NULL
};

const EOnetvar node00_loc_nvOBJ_globcon__macaddr =
{   // pos =  8
    .id                 = node00_loc_nvID_globcon__macaddr,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.globalconstants.macaddr),
        .valuedef           = (void*)&node00_loc_def.globalconstants.macaddr, 
        .valuevol           = (void*)&node00_loc_vol.globalconstants.macaddr,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = NULL,
    .on_rop_reception   = NULL
};

const EOnetvar node00_loc_nvOBJ_globcon__ipaddr =
{   // pos =  9
    .id                 = node00_loc_nvID_globcon__ipaddr,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.globalconstants.ipaddr),
        .valuedef           = (void*)&node00_loc_def.globalconstants.ipaddr, 
        .valuevol           = (void*)&node00_loc_vol.globalconstants.ipaddr,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = NULL,
    .on_rop_reception   = NULL
};


const EOnetvar node00_loc_nvOBJ_globalconfiguration =
{   // pos =  10
    .id                 = node00_loc_nvID_globalconfiguration,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.globalconfiguration),
        .valuedef           = (void*)&node00_loc_def.globalconfiguration, 
        .valuevol           = (void*)&node00_loc_vol.globalconfiguration,
        .mirror             = NULL,
        .flex.loc_addrperm  = node00_loc_addrPER_globalconfiguration    
    },
    .datainterface      = &node00_loc_action_datainterface_nvOBJ_globalconfiguration__any,
    .on_rop_reception   = NULL
};


const EOnetvar node00_loc_nvOBJ_globcfg__acquireinput =
{   // pos =  11
    .id                 = node00_loc_nvID_globcfg__acquireinput,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.globalconfiguration.acquireinput),
        .valuedef           = (void*)&node00_loc_def.globalconfiguration.acquireinput, 
        .valuevol           = (void*)&node00_loc_vol.globalconfiguration.acquireinput,
        .mirror             = NULL,
        .flex.loc_addrperm  = node00_loc_addrPER_globcfg__acquireinput    
    },
    .datainterface      = &node00_loc_action_datainterface_nvOBJ_globalconfiguration__any,
    .on_rop_reception   = NULL
};


const EOnetvar node00_loc_nvOBJ_globcfg__acquisitionperiod =
{   // pos =  12
    .id                 = node00_loc_nvID_globcfg__acquisitionperiod,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.globalconfiguration.acquisitionperiod),
        .valuedef           = (void*)&node00_loc_def.globalconfiguration.acquisitionperiod, 
        .valuevol           = (void*)&node00_loc_vol.globalconfiguration.acquisitionperiod,
        .mirror             = NULL,
        .flex.loc_addrperm  = node00_loc_addrPER_globcfg__acquisitionperiod    
    },
    .datainterface      = &node00_loc_action_datainterface_nvOBJ_globalconfiguration__any,
    .on_rop_reception   = NULL
};

const EOnetvar node00_loc_nvOBJ_globcfg__applyoutput =
{   // pos =  13
    .id                 = node00_loc_nvID_globcfg__applyoutput,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.globalconfiguration.applyoutput),
        .valuedef           = (void*)&node00_loc_def.globalconfiguration.applyoutput, 
        .valuevol           = (void*)&node00_loc_vol.globalconfiguration.applyoutput,
        .mirror             = NULL,
        .flex.loc_addrperm  = node00_loc_addrPER_globcfg__applyoutput    
    },
    .datainterface      = &node00_loc_action_datainterface_nvOBJ_globalconfiguration__any,
    .on_rop_reception   = NULL
};

const EOnetvar node00_loc_nvOBJ_globcfg__signalvportro =
{   // pos =  14
    .id                 = node00_loc_nvID_globcfg__signalvportro,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.globalconfiguration.signalvportro),
        .valuedef           = (void*)&node00_loc_def.globalconfiguration.signalvportro, 
        .valuevol           = (void*)&node00_loc_vol.globalconfiguration.signalvportro,
        .mirror             = NULL,
        .flex.loc_addrperm  = node00_loc_addrPER_globcfg__signalvportro    
    },
    .datainterface      = &node00_loc_action_datainterface_nvOBJ_globalconfiguration__any,
    .on_rop_reception   = NULL
};

const EOnetvar node00_loc_nvOBJ_globcfg__toipaddr =
{   // pos =  15
    .id                 = node00_loc_nvID_globcfg__toipaddr,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.globalconfiguration.toipaddr),
        .valuedef           = (void*)&node00_loc_def.globalconfiguration.toipaddr, 
        .valuevol           = (void*)&node00_loc_vol.globalconfiguration.toipaddr,
        .mirror             = NULL,
        .flex.loc_addrperm  = node00_loc_addrPER_globcfg__toipaddr    
    },
    .datainterface      = &node00_loc_action_datainterface_nvOBJ_globalconfiguration__any,
    .on_rop_reception   = NULL
};

const EOnetvar node00_loc_nvOBJ_globcfg__withperiod =
{   // pos =  16
    .id                 = node00_loc_nvID_globcfg__withperiod,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.globalconfiguration.withperiod),
        .valuedef           = (void*)&node00_loc_def.globalconfiguration.withperiod, 
        .valuevol           = (void*)&node00_loc_vol.globalconfiguration.withperiod,
        .mirror             = NULL,
        .flex.loc_addrperm  = node00_loc_addrPER_globcfg__withperiod    
    },
    .datainterface      = &node00_loc_action_datainterface_nvOBJ_globalconfiguration__any,
    .on_rop_reception   = NULL
};

const EOnetvar node00_loc_nvOBJ_button =
{   // pos =  17
    .id                 = node00_loc_nvID_button,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.button),
        .valuedef           = (void*)&node00_loc_def.button, 
        .valuevol           = (void*)&node00_loc_vol.button,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = &node00_loc_action_datainterface_nvOBJ_button,
    .on_rop_reception   = NULL
};


const EOnetvar node00_loc_nvOBJ_button__inputval =
{   // pos =  18
    .id                 = node00_loc_nvID_button,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.button.inputval),
        .valuedef           = (void*)&node00_loc_def.button.inputval, 
        .valuevol           = (void*)&node00_loc_vol.button.inputval,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = &node00_loc_action_datainterface_nvOBJ_button__inputval,
    .on_rop_reception   = NULL
};

const EOnetvar node00_loc_nvOBJ_button__acquisitiontime =
{   // pos =  19
    .id                 = node00_loc_nvID_button,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.button.acquisitiontime),
        .valuedef           = (void*)&node00_loc_def.button.acquisitiontime, 
        .valuevol           = (void*)&node00_loc_vol.button.acquisitiontime,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = &node00_loc_action_datainterface_generic,
    .on_rop_reception   = NULL
};

const EOnetvar node00_loc_nvOBJ_led00 =
{   // pos =  20
    .id                 = node00_loc_nvID_led00,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.led00),
        .valuedef           = (void*)&node00_loc_def.led00, 
        .valuevol           = (void*)&node00_loc_vol.led00,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = &node00_loc_action_datainterface_nvOBJ_led00,
    .on_rop_reception   = NULL
};


const EOnetvar node00_loc_nvOBJ_led00__outputval =
{   // pos =  21
    .id                 = node00_loc_nvID_led00__outputval,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.led00.outputval),
        .valuedef           = (void*)&node00_loc_def.led00.outputval, 
        .valuevol           = (void*)&node00_loc_vol.led00.outputval,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = &node00_loc_action_datainterface_nvOBJ_led00__outputval,
    .on_rop_reception   = NULL
};


const EOnetvar node00_loc_nvOBJ_led00__lednumber =
{   // pos =  22
    .id                 = node00_loc_nvID_led00__lednumber,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.led00.lednumber),
        .valuedef           = (void*)&node00_loc_def.led00.lednumber, 
        .valuevol           = (void*)&node00_loc_vol.led00.lednumber,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = NULL,
    .on_rop_reception   = NULL
};

const EOnetvar node00_loc_nvOBJ_led01 =
{   // pos =  23
    .id                 = node00_loc_nvID_led01,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.led01),
        .valuedef           = (void*)&node00_loc_def.led01, 
        .valuevol           = (void*)&node00_loc_vol.led01,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = &node00_loc_action_datainterface_nvOBJ_led01,
    .on_rop_reception   = NULL
};


const EOnetvar node00_loc_nvOBJ_led01__outputval =
{   // pos =  24
    .id                 = node00_loc_nvID_led01__outputval,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.led01.outputval),
        .valuedef           = (void*)&node00_loc_def.led01.outputval, 
        .valuevol           = (void*)&node00_loc_vol.led01.outputval,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = &node00_loc_action_datainterface_nvOBJ_led01__outputval,
    .on_rop_reception   = NULL
};


const EOnetvar node00_loc_nvOBJ_led01__lednumber =
{   // pos =  25
    .id                 = node00_loc_nvID_led01__lednumber,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.led01.lednumber),
        .valuedef           = (void*)&node00_loc_def.led01.lednumber, 
        .valuevol           = (void*)&node00_loc_vol.led01.lednumber,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = NULL,
    .on_rop_reception   = NULL
};


const EOnetvar node00_loc_nvOBJ__timeoflife =
{   // pos =  26
    .id                 = node00_loc_nvID__timeoflife,
    .ownership          = eo_nv_ownership_local,
    .data = 
    {   
        .capacity           = sizeof(node00_loc_def.timeoflife),
        .valuedef           = (void*)&node00_loc_def.timeoflife, 
        .valuevol           = (void*)&node00_loc_vol.timeoflife,
        .mirror             = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface      = &node00_loc_action_datainterface_nvOBJ__timeoflife,
    .on_rop_reception   = NULL
};


// this array contains the tree-structured netvars of the device. 
// they are ordered upon a depth first reading of the tree

const EOnetvarNode node00_loc_thenetvarnodes[] =
{
{   // pos = 0
    .nchildren  = 8,
    .children   = {1, 4, 7, 10, 17, 20, 23, 26},     
    .netvar     = &node00_loc_nvOBJ_root
},

/**/{   // pos = 1
        .nchildren  = 2,
        .children   = {2, 3},
        .netvar     = &node00_loc_nvOBJ_vportRO
    },
        {   // pos = 2
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &node00_loc_nvOBJ_vportRO__cfg
        },
        {   // pos = 3
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &node00_loc_nvOBJ_vportRO__dat
        },

/**/{   // pos = 4
        .nchildren  = 2,
        .children   = {5, 6},
        .netvar     = &node00_loc_nvOBJ_vportWO
    },
        {   // pos = 5
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &node00_loc_nvOBJ_vportWO__cfg
        },
        {   // pos = 6
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &node00_loc_nvOBJ_vportWO__dat
        },

/**/{   // pos = 7
        .nchildren  = 2,
        .children   = {8, 9},
        .netvar     = &node00_loc_nvOBJ_globalconstants
    },
        {   // pos = 8
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &node00_loc_nvOBJ_globcon__macaddr
        },
        {   // pos = 9
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &node00_loc_nvOBJ_globcon__ipaddr
        },

/**/{   // pos = 10
        .nchildren  = 6,
        .children   = {11, 12, 13, 14, 15, 16},
        .netvar     = &node00_loc_nvOBJ_globalconfiguration
    },
        {   // pos = 11
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &node00_loc_nvOBJ_globcfg__acquireinput
        },
        {   // pos = 12
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &node00_loc_nvOBJ_globcfg__acquisitionperiod
        },
        {   // pos = 13
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &node00_loc_nvOBJ_globcfg__applyoutput
        },
        {   // pos = 14
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &node00_loc_nvOBJ_globcfg__signalvportro
        },
        {   // pos = 15
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &node00_loc_nvOBJ_globcfg__toipaddr
        },
        {   // pos = 16
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &node00_loc_nvOBJ_globcfg__withperiod
        },


/**/{   // pos = 17
        .nchildren  = 2,
        .children   = {18, 19},
        .netvar     = &node00_loc_nvOBJ_button
    },
        {   // pos = 18
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &node00_loc_nvOBJ_button__inputval
        },
        {   // pos = 19
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &node00_loc_nvOBJ_button__acquisitiontime
        },
        
/**/{   // pos = 20
        .nchildren  = 2,
        .children   = {21, 22},
        .netvar     = &node00_loc_nvOBJ_led00
    },
        {   // pos = 21
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &node00_loc_nvOBJ_led00__outputval
        },
        {   // pos = 22
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &node00_loc_nvOBJ_led00__lednumber
        },        
        
/**/{   // pos = 23
        .nchildren  = 2,
        .children   = {24, 25},
        .netvar     = &node00_loc_nvOBJ_led01
    },
        {   // pos = 24
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &node00_loc_nvOBJ_led01__outputval
        },
        {   // pos = 25
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &node00_loc_nvOBJ_led01__lednumber
        },          

/**/{   // pos = 26
        .nchildren  = 0,
        .children   = {255},
        .netvar     = &node00_loc_nvOBJ__timeoflife
    }

};



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
// empty-section

typedef uint8_t s_loc_dummy_t[((sizeof(node00_loc_thenetvarnodes)/sizeof(EOnetvarNode)-node00_loc_NUMNETVARS) == 0) ? 1 : 0];


static EOfakeStorage *s_loc_storage = NULL;


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern void node00_loc_init_volatile_data(void)
{
    uint16_t i = 0;

    // 1. init the eeprom. if never initialised, then set it to contain teh default value node00_loc_def
    s_loc_storage = eo_fakestrg_New(node00_loc_STGid, sizeof(node00_loc_def), &node00_loc_def);

    // 2. copy eeprom into ram
    eov_strg_Get(s_loc_storage, node00_loc_addrPER_root, sizeof(node00_loc_vol), &node00_loc_vol);

    // 3. initialise every NV.
    for(i=0; i<node00_loc_NUMNETVARS; i++)
    {
        eo_netvar_Init(node00_loc_thenetvarnodes[i].netvar);
    }

}


extern void * node00_loc_get_storage(void)
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



