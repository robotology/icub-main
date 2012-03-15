
/** @file       EOtheNVsCfgDeviceLoc.c
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


#include "EOtheNVsCfgDeviceLocActions.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgDeviceLoc.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgDeviceLoc_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

#define offsetVPORT1                (0)
#define offsetVPORT0                (sizeof(EOvport))

#define offsetGLOBCFG               (2*sizeof(EOvport))
#define offsetINP08CFG              (2*sizeof(EOvport)+sizeof(eOentity_global_cfg_t)+1+8)
#define offsetINP16CFG              (2*sizeof(EOvport)+sizeof(eOentity_global_cfg_t)+sizeof(eOentity_input08_t)+2+8)
#define offsetINP32CFG              (2*sizeof(EOvport)+sizeof(eOentity_global_cfg_t)+sizeof(eOentity_input08_t)+sizeof(eOentity_input16_t)+4+8)
#define offsetOUTPUTCFG             (2*sizeof(EOvport)+sizeof(eOentity_global_cfg_t)+sizeof(eOentity_input08_t)+sizeof(eOentity_input16_t)+sizeof(eOentity_input32_t)+4+8)



#define OFFSETSdev_vportROnly__cfg                        (offsetVPORT1)

#define OFFSETSdev_vportWOnly__cfg                        (offsetVPORT0)

#define OFFSETSdev_globcfg__doit                        (offsetGLOBCFG) 
#define OFFSETSdev_globcfg__doalsothat                  (offsetGLOBCFG + 1)  
#define OFFSETSdev_globcfg__withperiodinmicrosecs       (offsetGLOBCFG + 2)  
    

#define OFFSETSdev_input08_inpcfg__acquisitionenabled   (offsetINP08CFG) 
#define OFFSETSdev_input08_inpcfg__acquisitionperiod    (offsetINP08CFG + 1)

#define OFFSETSdev_input16_inpcfg__acquisitionenabled   (offsetINP16CFG) 
#define OFFSETSdev_input16_inpcfg__acquisitionperiod    (offsetINP16CFG + 1)

#define OFFSETSdev_input32_inpcfg__acquisitionenabled   (offsetINP32CFG)
#define OFFSETSdev_input32_inpcfg__acquisitionperiod    (offsetINP32CFG + 1)

#define OFFSETSdev_output_outcfg__applicationenabled    (offsetOUTPUTCFG)
#define OFFSETSdev_output_outcfg__applicationmode       (offsetOUTPUTCFG + 1)







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

extern eOdevice_Loc_t loc_device_vol = { 0 };

// this struct contains the default values of the entities of the whole device

extern const eOdevice_Loc_t loc_device_def =
{
    .vportROnly                  =
    {
        .cfg =
        {
            .sizeofarray = EOVPORT_GET_SizeOfArray(1),
            .n = 1,
            .nvids = { nvIDlocdev_input08__input08value, 0}
        },
        .dat =
        {
            .sizeofarray = EOVPORT_GET_SizeOfArray(0),
            .n = 0,
            .buffer_of_nvidsizeinfo = {0}
        },
        .mirrors = {0}
    },
    .vportWOnly               =
    {
        .cfg =
        {
            .sizeofarray =EOVPORT_GET_SizeOfArray(1),
            .n = 1,
            .nvids = { nvIDlocdev_output__outputvalue, 0}
        },
        .dat =
        {
            .sizeofarray = EOVPORT_GET_SizeOfArray(0),
            .n = 0,
            .buffer_of_nvidsizeinfo = {0}
        },
        .mirrors = {0}
    },
    .globalconfiguration    = 
    { 
        .doit = 1, 
        .doalsothat = 0, 
        .withperiodinmicrosecs =  0x12345678
    },
    .input08                = 
    {
        .input08value = 0,
        .acquisitiontime = 0x001122334455667788,
        .inputconfiguration =
        {
            .acquisitionenabled = 0,
            .acquisitionperiod = 0x22334455
        }
    },
    .input16                = 
    {
        .input16value = 0,
        .acquisitiontime = 0x001122334455667788,
        .inputconfiguration =
        {
            .acquisitionenabled = 0,
            .acquisitionperiod = 0x22334455
        }
    },
    .input32                = 
    {
        .input32value = 0,
        .acquisitiontime = 0x001122334455667788,
        .inputconfiguration =
        {
            .acquisitionenabled = 0,
            .acquisitionperiod = 0x22334455
        }
    },
    .output                 = 
    { 
        .outputvalue = 0x11111111,
        .applicationtime = 0x88aabbccddeeff00,
        .outputconfiguration = 
        {
            .applicationenabled = 0,
            .applicationmode = 0x11223344
        }
    },

    .fixedarray =
    {
        1, 2, 3, 4, 5, 6, 7, 8, 9    
    },
    .varsizearray =
    {
        4,0,  1, 2, 3, 4, 255, 255, 255   
    }

};





const EOnetvar netvar_loc_device =
{   // pos =  0
    .id         = nvIDlocdevice,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {   
        .capacity   = sizeof(eOdevice_Loc_t),
        .valuedef   = (void*)&loc_device_def, 
        .valuevol   = (void*)&loc_device_vol,
        .mirror     = NULL,
        .flex.loc_addrperm  = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL
};



const EOnetvar netvar_loc_vportROnly =
{   // pos =  1
    .id         = nvIDlocdev_vportROnly,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(loc_device_def.vportROnly),
        .valuedef   = (void*)&loc_device_def.vportROnly, 
        .valuevol   = (void*)&loc_device_vol.vportROnly,
        .mirror     = NULL,
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};




const eOnetvar_fn_datainterface_t ondata_loc_vportROnly__cfg =
{
    .init       = nvscfg_device_loc_vportROnly_configure,
    .update     = nvscfg_device_loc_vportROnly_configure
};

 
const EOnetvar netvar_loc_vportROnly__cfg =
{   // pos =  2
    .id         = nvIDlocdev_vportROnly__cfg,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(loc_device_def.vportROnly.cfg),
        .valuedef   = (void*)&loc_device_def.vportROnly.cfg, 
        .valuevol   = (void*)&loc_device_vol.vportROnly.cfg,
        .mirror     = NULL,
        .flex.loc_addrperm   = OFFSETSdev_vportROnly__cfg //EONETVAR_NOVALUEPER,
    },
    .datainterface = &ondata_loc_vportROnly__cfg,
    .on_rop_reception = NULL
};


const EOnetvar netvar_loc_vportROnly__dat =
{   // pos =  3
    .id         = nvIDlocdev_vportROnly__dat,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(loc_device_def.vportROnly.dat),
        .valuedef   = (void*)&loc_device_def.vportROnly.dat, 
        .valuevol   = (void*)&loc_device_vol.vportROnly.dat,
        .mirror     = NULL,
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};


const EOnetvar netvar_loc_vportWOnly =  
{   // pos =  4
    .id         = nvIDlocdev_vportWOnly,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(loc_device_def.vportWOnly),
        .valuedef   = (void*)&loc_device_def.vportWOnly, 
        .valuevol   = (void*)&loc_device_vol.vportWOnly,
        .mirror     = NULL,
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};


const eOnetvar_onrop_rx_t onrop_loc_vportWOnly__cfg =
{
    .loc =
    {
        .ask =
        {
            EONETVAR_ONROP_SET_2FN(NULL, NULL)
        },
        .set =
        {
            EONETVAR_ONROP_SET_2FN(NULL, NULL)
        },
        .rst =
        {
            EONETVAR_ONROP_SET_2FN(NULL, NULL)
        },
        .upd =
        {
            EONETVAR_ONROP_SET_2FN(NULL, NULL)
        }
    }
};

const EOnetvar netvar_loc_vportWOnly__cfg = 
{   // pos =  5
    .id         = nvIDlocdev_vportWOnly__cfg,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(loc_device_def.vportWOnly.cfg),
        .valuedef   = (void*)&loc_device_def.vportWOnly.cfg, 
        .valuevol   = (void*)&loc_device_vol.vportWOnly.cfg,
        .mirror     = NULL,
        .flex.loc_addrperm   = OFFSETSdev_vportWOnly__cfg    
    },
    .datainterface = NULL,
    .on_rop_reception = &onrop_loc_vportWOnly__cfg
};


const eOnetvar_fn_datainterface_t ondata_loc_vportWOnly__dat =
{
    .init       = NULL,
    .update     = nvscfg_device_loc_vportWOnly_update
};

const EOnetvar netvar_loc_vportWOnly__dat =
{   // pos =  6
    .id         = nvIDlocdev_vportWOnly__dat,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(loc_device_def.vportWOnly.dat),
        .valuedef   = (void*)&loc_device_def.vportWOnly.dat, 
        .valuevol   = (void*)&loc_device_vol.vportWOnly.dat,
        .mirror     = NULL,
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = &ondata_loc_vportWOnly__dat,
    .on_rop_reception = NULL 
};


const eOnetvar_fn_datainterface_t ondata_loc_globalconfiguration =
{
    .init       = nvscfg_device_loc_act_STARTUP_var_globalconfiguration,
    .update     = NULL
};

const eOnetvar_onrop_rx_t onrop_loc_globalconfiguration =
{
    .loc =
    {
        .ask =
        {
            EONETVAR_ONROP_SET_2FN(NULL, NULL)
        },
        .set =
        {
            EONETVAR_ONROP_SET_2FN(nvscfg_device_loc_act_BEF_SET_var_globalconfiguration, nvscfg_device_loc_act_AFT_SET_var_globalconfiguration)
        },
        .rst =
        {
            EONETVAR_ONROP_SET_2FN(nvscfg_device_loc_act_BEF_SET_var_globalconfiguration, nvscfg_device_loc_act_AFT_SET_var_globalconfiguration)
        },
        .upd =
        {
            EONETVAR_ONROP_SET_2FN(NULL, NULL)
        }
    }
};
 
const EOnetvar netvar_loc_globalconfiguration =
{   // pos =  7
    .id         = nvIDlocdev_globalconfiguration,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(eOentity_global_cfg_t),
        .valuedef   = (void*)&loc_device_def.globalconfiguration, 
        .valuevol   = (void*)&loc_device_vol.globalconfiguration,
        .mirror     = NULL,
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER
    },
    .datainterface = &ondata_loc_globalconfiguration,
    .on_rop_reception = &onrop_loc_globalconfiguration
};


const eOnetvar_onrop_rx_t onrop_loc_globcfg__doit =
{
    .loc =
    {
        .ask =
        {
            EONETVAR_ONROP_SET_2FN(NULL, NULL)
        },
        .set =
        {
            EONETVAR_ONROP_SET_2FN(nvscfg_device_loc_act_BEF_SET_var__doit, nvscfg_device_loc_act_AFT_SET_var__doit)
        },
        .rst =
        {
            EONETVAR_ONROP_SET_2FN(nvscfg_device_loc_act_BEF_SET_var__doit, nvscfg_device_loc_act_AFT_SET_var__doit)
        },
        .upd =
        {
            EONETVAR_ONROP_SET_2FN(NULL, NULL)
        }
    }
};
     
const EOnetvar netvar_loc_globcfg__doit =
{   // pos =  8
    .id         = nvIDlocdev_globcfg__doit,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(uint8_t),
        .valuedef   = (void*)&loc_device_def.globalconfiguration.doit, 
        .valuevol   = (void*)&loc_device_vol.globalconfiguration.doit,
        .mirror     = NULL,
        .flex.loc_addrperm   = OFFSETSdev_globcfg__doit    
    },
    .datainterface = NULL,
    .on_rop_reception = &onrop_loc_globcfg__doit
};


const eOnetvar_onrop_rx_t onrop_loc_globcfg__doalsothat =
{
    .loc =
    {
        .ask =
        {
            EONETVAR_ONROP_SET_2FN(NULL, NULL)
        },
        .set =
        {
            EONETVAR_ONROP_SET_2FN(nvscfg_device_loc_act_BEF_SET_var__doalsothat, nvscfg_device_loc_act_AFT_SET_var__doalsothat)
        },
        .rst =
        {
            EONETVAR_ONROP_SET_2FN(nvscfg_device_loc_act_BEF_SET_var__doalsothat, nvscfg_device_loc_act_AFT_SET_var__doalsothat)
        },
        .upd =
        {
            EONETVAR_ONROP_SET_2FN(NULL, NULL)
        }
    }
};

const EOnetvar netvar_loc_globcfg__doalsothat =
{   // pos =  9
    .id         = nvIDlocdev_globcfg__doalsothat,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(uint8_t),
        .valuedef   = (void*)&loc_device_def.globalconfiguration.doalsothat, 
        .valuevol   = (void*)&loc_device_vol.globalconfiguration.doalsothat,
        .mirror     = NULL,
        .flex.loc_addrperm   = OFFSETSdev_globcfg__doalsothat    
    },
    .datainterface = NULL,
    .on_rop_reception = &onrop_loc_globcfg__doalsothat
};


const eOnetvar_onrop_rx_t onrop_loc_globcfg__withperiodinmicrosecs =
{
    .loc =
    {
        .ask =
        {
            EONETVAR_ONROP_SET_2FN(NULL, NULL)
        },
        .set =
        {
            EONETVAR_ONROP_SET_2FN(nvscfg_device_loc_act_BEF_SET_var__withperiodinmicrosecs, nvscfg_device_loc_act_AFT_SET_var__withperiodinmicrosecs)
        },
        .rst =
        {
            EONETVAR_ONROP_SET_2FN(nvscfg_device_loc_act_BEF_SET_var__withperiodinmicrosecs, nvscfg_device_loc_act_AFT_SET_var__withperiodinmicrosecs)
        },
        .upd =
        {
            EONETVAR_ONROP_SET_2FN(NULL, NULL)
        }
    }
};

const EOnetvar netvar_loc_globcfg__withperiodinmicrosecs =
{   // pos =  10
    .id         = nvIDlocdev_globcfg__withperiodinmicrosecs,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(uint32_t),
        .valuedef   = (void*)&loc_device_def.globalconfiguration.withperiodinmicrosecs, 
        .valuevol   = (void*)&loc_device_vol.globalconfiguration.withperiodinmicrosecs,
        .mirror     = NULL,
        .flex.loc_addrperm   = OFFSETSdev_globcfg__withperiodinmicrosecs    
    },
    .datainterface = NULL,
    .on_rop_reception = &onrop_loc_globcfg__withperiodinmicrosecs
};


const EOnetvar netvar_loc_input08 =
{   // pos =  11
    .id         = nvIDlocdev_input08,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(eOentity_input08_t),
        .valuedef   = (void*)&loc_device_def.input08, 
        .valuevol   = (void*)&loc_device_vol.input08,
        .mirror     = NULL,
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};


const EOnetvar netvar_loc_input08__input08value =
{   // pos =  12
    .id         = nvIDlocdev_input08__input08value,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(uint8_t),
        .valuedef   = (void*)&loc_device_def.input08.input08value, 
        .valuevol   = (void*)&loc_device_vol.input08.input08value,
        .mirror     = EOVPORT_GET_Mirror(loc_device_vol.vportROnly, nvROmir_locdev_input08__input08value),
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};

 
const EOnetvar netvar_loc_input08__acquisitiontime =
{   // pos =  13
    .id         = nvIDlocdev_input08__acquisitiontime,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(uint64_t),
        .valuedef   = (void*)&loc_device_def.input08.acquisitiontime, 
        .valuevol   = (void*)&loc_device_vol.input08.acquisitiontime,
        .mirror     = NULL,
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};

 
const EOnetvar netvar_loc_input08_inputconfiguration =
{   // pos =  14
    .id         = nvIDlocdev_input08_inputconfiguration,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(eOentity_input_cfg_t),
        .valuedef   = (void*)&loc_device_def.input08.inputconfiguration, 
        .valuevol   = (void*)&loc_device_vol.input08.inputconfiguration,
        .mirror     = NULL,
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};


const EOnetvar netvar_loc_input08_inpcfg__acquisitionenabled =
{   // pos =  15
    .id         = nvIDlocdev_input08_inpcfg__acquisitionenabled,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(uint8_t),
        .valuedef   = (void*)&loc_device_def.input08.inputconfiguration.acquisitionenabled, 
        .valuevol   = (void*)&loc_device_vol.input08.inputconfiguration.acquisitionenabled,
        .mirror     = NULL,
        .flex.loc_addrperm   = OFFSETSdev_input08_inpcfg__acquisitionenabled    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};

 
const EOnetvar netvar_loc_input08_inpcfg__acquisitionperiod =
{   // pos =  16
    .id         = nvIDlocdev_input08_inpcfg__acquisitionperiod,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(uint32_t),
        .valuedef   = (void*)&loc_device_def.input08.inputconfiguration.acquisitionperiod, 
        .valuevol   = (void*)&loc_device_vol.input08.inputconfiguration.acquisitionperiod,
        .mirror     = NULL,
        .flex.loc_addrperm   = OFFSETSdev_input08_inpcfg__acquisitionperiod    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};


const EOnetvar netvar_loc_input16 =
{   // pos =  17
    .id         = nvIDlocdev_input16,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(eOentity_input16_t),
        .valuedef   = (void*)&loc_device_def.input16, 
        .valuevol   = (void*)&loc_device_vol.input16,
        .mirror     = NULL,
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};

 

const EOnetvar netvar_loc_input16__input16value =
{   // pos =  18
    .id         = nvIDlocdev_input16__input16value,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(uint16_t),
        .valuedef   = (void*)&loc_device_def.input16.input16value, 
        .valuevol   = (void*)&loc_device_vol.input16.input16value,
        .mirror     = EOVPORT_GET_Mirror(loc_device_vol.vportROnly, nvROmir_locdev_input16__input16value), 
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};

 
const EOnetvar netvar_loc_input16__acquisitiontime =
{   // pos =  19
    .id         = nvIDlocdev_input16__acquisitiontime,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(uint64_t),
        .valuedef   = (void*)&loc_device_def.input16.acquisitiontime, 
        .valuevol   = (void*)&loc_device_vol.input16.acquisitiontime,
        .mirror     = NULL,
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};


const EOnetvar netvar_loc_input16_inputconfiguration =
{   // pos =  20
    .id         = nvIDlocdev_input16_inputconfiguration,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(eOentity_input_cfg_t),
        .valuedef   = (void*)&loc_device_def.input16.inputconfiguration, 
        .valuevol   = (void*)&loc_device_vol.input16.inputconfiguration,
        .mirror     = NULL,
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};


const EOnetvar netvar_loc_input16_inpcfg__acquisitionenabled =
{   // pos =  21
    .id         = nvIDlocdev_input16_inpcfg__acquisitionenabled,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {   
        .capacity   = sizeof(uint8_t),
        .valuedef   = (void*)&loc_device_def.input16.inputconfiguration.acquisitionenabled, 
        .valuevol   = (void*)&loc_device_vol.input16.inputconfiguration.acquisitionenabled,
        .mirror     = NULL,
        .flex.loc_addrperm   = OFFSETSdev_input16_inpcfg__acquisitionenabled    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};


const EOnetvar netvar_loc_input16_inpcfg__acquisitionperiod =
{   // pos =  22
    .id         = nvIDlocdev_input16_inpcfg__acquisitionperiod,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(uint32_t),
        .valuedef   = (void*)&loc_device_def.input16.inputconfiguration.acquisitionperiod, 
        .valuevol   = (void*)&loc_device_vol.input16.inputconfiguration.acquisitionperiod,
        .mirror     = NULL,
        .flex.loc_addrperm   = OFFSETSdev_input16_inpcfg__acquisitionperiod    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};



const EOnetvar netvar_loc_input32 =
{   // pos =  23
    .id         = nvIDlocdev_input32,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(eOentity_input32_t),
        .valuedef   = (void*)&loc_device_def.input32, 
        .valuevol   = (void*)&loc_device_vol.input32,
        .mirror     = NULL,
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};

 
const EOnetvar netvar_loc_input32__input32value =
{   // pos =  24
    .id         = nvIDlocdev_input32__input32value,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(uint32_t),
        .valuedef   = (void*)&loc_device_def.input32.input32value, 
        .valuevol   = (void*)&loc_device_vol.input32.input32value,
        .mirror     = EOVPORT_GET_Mirror(loc_device_vol.vportROnly, nvROmir_locdev_input32__input32value), 
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};


const EOnetvar netvar_loc_input32__acquisitiontime =
{   // pos =  25
    .id         = nvIDlocdev_input32__acquisitiontime,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(uint64_t),
        .valuedef   = (void*)&loc_device_def.input32.acquisitiontime, 
        .valuevol   = (void*)&loc_device_vol.input32.acquisitiontime,
        .mirror     = NULL,
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};


const EOnetvar netvar_loc_input32_inputconfiguration =
{   // pos =  26
    .id         = nvIDlocdev_input32_inputconfiguration,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(eOentity_input_cfg_t),
        .valuedef   = (void*)&loc_device_def.input32.inputconfiguration, 
        .valuevol   = (void*)&loc_device_vol.input32.inputconfiguration,
        .mirror     = NULL,
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};


const EOnetvar netvar_loc_input32_inpcfg__acquisitionenabled =
{   // pos =  27
    .id         = nvIDlocdev_input32_inpcfg__acquisitionenabled,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(uint8_t),
        .valuedef   = (void*)&loc_device_def.input32.inputconfiguration.acquisitionenabled, 
        .valuevol   = (void*)&loc_device_vol.input32.inputconfiguration.acquisitionenabled,
        .mirror     = NULL,
        .flex.loc_addrperm   = OFFSETSdev_input32_inpcfg__acquisitionenabled    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};


const EOnetvar netvar_loc_input32_inpcfg__acquisitionperiod =
{   // pos =  28
    .id         = nvIDlocdev_input32_inpcfg__acquisitionperiod,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(uint32_t),
        .valuedef   = (void*)&loc_device_def.input32.inputconfiguration.acquisitionperiod, 
        .valuevol   = (void*)&loc_device_vol.input32.inputconfiguration.acquisitionperiod,
        .mirror     = NULL,
        .flex.loc_addrperm   = OFFSETSdev_input32_inpcfg__acquisitionperiod    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};



const EOnetvar netvar_loc_output =
{   // pos =  29
    .id         = nvIDlocdev_output,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(eOentity_output_t),
        .valuedef   = (void*)&loc_device_def.output, 
        .valuevol   = (void*)&loc_device_vol.output,
        .mirror     = NULL,
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};

 
const EOnetvar netvar_loc_output__outputvalue =
{   // pos =  30
    .id         = nvIDlocdev_output__outputvalue,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(uint32_t),
        .valuedef   = (void*)&loc_device_def.output.outputvalue, 
        .valuevol   = (void*)&loc_device_vol.output.outputvalue,
        .mirror     = EOVPORT_GET_Mirror(loc_device_vol.vportROnly, nvROmir_locdev_output__outputvalue),
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};


const EOnetvar netvar_loc_output__applicationtime =
{   // pos =  31
    .id         = nvIDlocdev_output__applicationtime,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(uint64_t),
        .valuedef   = (void*)&loc_device_def.output.applicationtime, 
        .valuevol   = (void*)&loc_device_vol.output.applicationtime,
        .mirror     = EOVPORT_GET_Mirror(loc_device_vol.vportROnly, nvROmir_locdev_output__applicationtime),
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};
                                 

const EOnetvar netvar_loc_output_outputconfiguration =
{   // pos =  32
    .id         = nvIDlocdev_output_outputconfiguration,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(eOentity_output_cfg_t),
        .valuedef   = (void*)&loc_device_def.output.outputconfiguration, 
        .valuevol   = (void*)&loc_device_vol.output.outputconfiguration,
        .mirror     = NULL,
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};


const EOnetvar netvar_loc_output_outcfg__acquisitionenabled =
{   // pos =  33
    .id         = nvIDlocdev_output_outcfg__acquisitionenabled,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(uint8_t),
        .valuedef   = (void*)&loc_device_def.output.outputconfiguration.applicationenabled, 
        .valuevol   = (void*)&loc_device_vol.output.outputconfiguration.applicationenabled,
        .mirror     = NULL,
        .flex.loc_addrperm   = OFFSETSdev_output_outcfg__applicationenabled    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};


const EOnetvar netvar_loc_output_outcfg__applicationmode =
{   // pos =  34
    .id         = nvIDlocdev_output_outcfg__applicationmode,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(uint32_t),
        .valuedef   = (void*)&loc_device_def.output.outputconfiguration.applicationmode, 
        .valuevol   = (void*)&loc_device_vol.output.outputconfiguration.applicationmode,
        .mirror     = NULL,
        .flex.loc_addrperm   = OFFSETSdev_output_outcfg__applicationmode    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};



const EOnetvar netvar_loc__fixedarray =
{   // pos =  35
    .id         = nvIDlocdev__fixedarray,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(loc_device_def.fixedarray),
        .valuedef   = (void*)&loc_device_def.fixedarray, 
        .valuevol   = (void*)&loc_device_vol.fixedarray,
        .mirror     = NULL,
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};


const EOnetvar netvar_loc__varsizearray =
{   // pos =  36
    .id         = nvIDlocdev__varsizearray,
    .ownership  = eo_nv_ownership_local,
    .data = 
    {
        .capacity   = sizeof(loc_device_def.varsizearray),
        .valuedef   = (void*)&loc_device_def.varsizearray, 
        .valuevol   = (void*)&loc_device_vol.varsizearray,
        .mirror     = EOVPORT_GET_Mirror(loc_device_vol.vportROnly, nvROmir_locdev__varsizearray),
        .flex.loc_addrperm   = EONETVAR_NOVALUEPER    
    },
    .datainterface = NULL,
    .on_rop_reception = NULL 
};



// this array contains the tree-structured netvars of the device. 
// they are ordered upon a depth first reading of the tree

const EOnetvarNode loc_device_thenetvarnodes[] =
{
/**/{   // pos = 0
        .nchildren  = 9,
        .children   = {1, 4, 7, 11, 17, 23, 29, 35, 36},     
        .netvar     = &netvar_loc_device
    },
    
/**/{   // pos = 1
        .nchildren  = 2,
        .children   = {2, 3},
        .netvar     = &netvar_loc_vportROnly
    },
        {   // pos = 2
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_loc_vportROnly__cfg
        },
        {   // pos = 3
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_loc_vportROnly__dat
        },

/**/{   // pos = 4
        .nchildren  = 2,
        .children   = {5, 6},
        .netvar     = &netvar_loc_vportWOnly
    },
        {   // pos = 5
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_loc_vportWOnly__cfg
        },
        {   // pos = 6
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_loc_vportWOnly__dat
        },

/**/{   // pos = 7
        .nchildren  = 3,
        .children   = {8, 9, 10},
        .netvar     = &netvar_loc_globalconfiguration
    },
        {   // pos = 8
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_loc_globcfg__doit
        },
        {   // pos = 9
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_loc_globcfg__doalsothat
        },
        {   // pos = 10
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_loc_globcfg__withperiodinmicrosecs
        },

/**/{   // pos = 11
        .nchildren  = 3,
        .children   = {12, 13, 14},
        .netvar     = &netvar_loc_input08
    },
        {   // pos = 12
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_loc_input08__input08value
        },
        {   // pos = 13
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_loc_input08__acquisitiontime
        },
        {   // pos = 14
            .nchildren  = 2,
            .children   = {15, 16},
            .netvar     = &netvar_loc_input08_inputconfiguration
        },
            {   // pos = 15
                .nchildren  = 0,
                .children   = {255},
                .netvar     = &netvar_loc_input08_inpcfg__acquisitionenabled
            },
            {   // pos = 16
                .nchildren  = 0,
                .children   = {255},
                .netvar     = &netvar_loc_input08_inpcfg__acquisitionperiod
            },

/**/{   // pos = 17
        .nchildren  = 3,
        .children   = {18, 19, 20},
        .netvar     = &netvar_loc_input16
    },
        {   // pos = 18
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_loc_input16__input16value
        },
        {   // pos = 19
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_loc_input16__acquisitiontime
        },
        {   // pos = 20
            .nchildren  = 2,
            .children   = {21, 22},
            .netvar     = &netvar_loc_input16_inputconfiguration
        },
            {   // pos = 21
                .nchildren  = 0,
                .children   = {255},
                .netvar     = &netvar_loc_input16_inpcfg__acquisitionenabled
            },
            {   // pos = 22
                .nchildren  = 0,
                .children   = {255},
                .netvar     = &netvar_loc_input16_inpcfg__acquisitionperiod
            },

/**/{   // pos = 23
        .nchildren  = 3,
        .children   = {24, 25, 26},
        .netvar     = &netvar_loc_input32
    },
        {   // pos = 24
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_loc_input32__input32value
        },
        {   // pos = 25
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_loc_input32__acquisitiontime
        },
        {   // pos = 26
            .nchildren  = 2,
            .children   = {27, 28},
            .netvar     = &netvar_loc_input32_inputconfiguration
        },
            {   // pos = 27
                .nchildren  = 0,
                .children   = {255},
                .netvar     = &netvar_loc_input32_inpcfg__acquisitionenabled
            },
            {   // pos = 28
                .nchildren  = 0,
                .children   = {255},
                .netvar     = &netvar_loc_input32_inpcfg__acquisitionperiod
            },

/**/{   // pos = 29
        .nchildren  = 3,
        .children   = {30, 31, 32},
        .netvar     = &netvar_loc_output
    },
        {   // pos = 30
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_loc_output__outputvalue
        },
        {   // pos = 31
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_loc_output__applicationtime
        },
        {   // pos = 32
            .nchildren  = 2,
            .children   = {33, 34},
            .netvar     = &netvar_loc_output_outputconfiguration
        },
            {   // pos = 33
                .nchildren  = 0,
                .children   = {255},
                .netvar     = &netvar_loc_output_outcfg__acquisitionenabled
            },
            {   // pos = 34
                .nchildren  = 0,
                .children   = {255},
                .netvar     = &netvar_loc_output_outcfg__applicationmode
            },

/**/{   // pos = 35
        .nchildren  = 0,
        .children   = {255},
        .netvar     = &netvar_loc__fixedarray
    },

/**/{   // pos = 36
        .nchildren  = 0,
        .children   = {255},
        .netvar     = &netvar_loc__varsizearray
    }

};



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
// empty-section

typedef uint8_t s_loc_dummy_t[((sizeof(loc_device_thenetvarnodes)/sizeof(EOnetvarNode)-loc_device_NUMNETVARS) == 0) ? 1 : 0];


static EOfakeStorage *s_loc_storage = NULL;


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern void nvscfg_device_loc_init_volatile_data(void)
{
    uint16_t i = 0;
    #warning TODO: change eo_fakestrg_New(666 ...) w/ eo_eepromstrg_New( ... )

    // 1. init the eeprom. if never initialised, then set it to contain teh default value loc_device_def
    s_loc_storage = eo_fakestrg_New(666, sizeof(loc_device_def), &loc_device_def);

    // 2. copy eeprom into ram
    eov_strg_Get(s_loc_storage, 0, sizeof(loc_device_vol), &loc_device_vol);


    // 3. initialise every NV.

    for(i=0; i<loc_device_NUMNETVARS; i++)
    {
        eo_netvar_Init(loc_device_thenetvarnodes[i].netvar);
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



