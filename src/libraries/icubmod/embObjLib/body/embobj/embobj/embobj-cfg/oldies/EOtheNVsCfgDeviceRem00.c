
/** @file       EOtheNVsCfgDeviceRem00.c
    @brief      This file implements internal implementation to the configuration of the NVs of a device
    @author     marco.accame@iit.it
    @date       10/15/2009
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "EoCommon.h"
#include "EOVtheNVsCfg_hid.h"
#include "EOnetvar_hid.h"
#include "EOnetvar.h"
#include "EOvport_hid.h"
#include "EOnetvarNode_hid.h"

#include "EOtheNVsCfgDeviceRem00Actions.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgDeviceRem00.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgDeviceRem00_hid.h" 


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



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// here are the ram variables 

// contains in ram the values of the netvars whcih we use for forming the rops set/rst<> 
extern eOdevice_Rem00_t rem00_device_vol = { 0 };
// contains in ram the values of the netvars received from the remote device with teh rops say/sig<> 
extern eOdevice_Rem00_t rem00_device_rec = { 0 };


// --------------------------------------------------------------------------------------------------------------------
// here is the default rom variable

// contains in rom the default values netvars of the remote device.
// they MUST be equal to the default values of teh remote device, with the only exception
// of the vports
extern const eOdevice_Rem00_t rem00_device_def =
{
    .vportROnly                  =
    {
        .cfg =
        {   // dont put anything in here
            .sizeofarray = EOVPORT_GET_SizeOfArray(0),
            .n = 0,
            .nvids = {0}
        },
        .dat =
        {   // dont put anything in here
            .sizeofarray = EOVPORT_GET_SizeOfArray(0),
            .n = 0,
            .buffer_of_nvidsizeinfo = {0}
        },
        .mirrors = {0}
    },
    .vportWOnly                  =
    {
        .cfg =
        {   // put in here the ids of the writeable nvs which the smart device can write w/ the vport in the remote device 
            .sizeofarray = EOVPORT_GET_SizeOfArray(1),
            .n = 1,
            .nvids = {nvIDrem00dev_output__outputvalue}
        },
        .dat =
        {   // dont put anything in here: the .init() of the vportWOnly.cfg will fill the values in the rem00_device_vol.vportWOnly.dat
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
        10, 12, 13, 14, 15, 16, 17, 18, 19    
    },
    .varsizearray =
    {
        0
    }    
};


// --------------------------------------------------------------------------------------------------------------------
// here are the netvars in rom


const EOnetvar netvar_r00_device =
{   // pos =  0
    .id         = nvIDrem00device,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {   
        .capacity   = sizeof(eOdevice_Rem00_t),
        .valuedef   = (void*)&rem00_device_def, 
        .valuevol   = (void*)&rem00_device_vol,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL 
};




const EOnetvar netvar_r00_vportROnly =
{   // pos =  1
    .id         = nvIDrem00dev_vportROnly,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(rem00_device_def.vportROnly),
        .valuedef   = (void*)&rem00_device_def.vportROnly, 
        .valuevol   = (void*)&rem00_device_vol.vportROnly,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.vportROnly    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};

 
const EOnetvar netvar_r00_vportROnly__cfg =
{   // pos =  2
    .id         = nvIDrem00dev_vportROnly__cfg,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(rem00_device_def.vportROnly.cfg),
        .valuedef   = (void*)&rem00_device_def.vportROnly.cfg, 
        .valuevol   = (void*)&rem00_device_vol.vportROnly.cfg,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.vportROnly.cfg    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};


const EOnetvar netvar_r00_vportROnly__dat =
{   // pos =  3
    .id         = nvIDrem00dev_vportROnly__dat,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(rem00_device_def.vportROnly.dat),
        .valuedef   = (void*)&rem00_device_def.vportROnly.dat, 
        .valuevol   = (void*)&rem00_device_vol.vportROnly.dat,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.vportROnly.dat    
    },
    .datainterface = &ondata_r00_vportROnly__dat,
    .on_rop_reception = &onrop_r00_vportROnly__dat
};


const EOnetvar netvar_r00_vportWOnly =  
{   // pos =  4
    .id         = nvIDrem00dev_vportWOnly,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(rem00_device_def.vportWOnly),
        .valuedef   = (void*)&rem00_device_def.vportWOnly, 
        .valuevol   = (void*)&rem00_device_vol.vportWOnly,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.vportWOnly    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};


const EOnetvar netvar_r00_vportWOnly__cfg = 
{   // pos =  5
    .id         = nvIDrem00dev_vportWOnly__cfg,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(rem00_device_def.vportWOnly.cfg),
        .valuedef   = (void*)&rem00_device_def.vportWOnly.cfg, 
        .valuevol   = (void*)&rem00_device_vol.vportWOnly.cfg,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.vportWOnly.cfg    
    },
    .datainterface = &ondata_r00_vportWOnly__cfg,
    .on_rop_reception = NULL
};



const EOnetvar netvar_r00_vportWOnly__dat =
{   // pos =  6
    .id         = nvIDrem00dev_vportWOnly__dat,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(rem00_device_def.vportWOnly.dat),
        .valuedef   = (void*)&rem00_device_def.vportWOnly.dat, 
        .valuevol   = (void*)&rem00_device_vol.vportWOnly.dat,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.vportWOnly.dat    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = &onrop_r00_vportWOnly__dat
};

 
const EOnetvar netvar_r00_globalconfiguration =
{   // pos =  7
    .id         = nvIDrem00dev_globalconfiguration,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(eOentity_global_cfg_t),
        .valuedef   = (void*)&rem00_device_def.globalconfiguration, 
        .valuevol   = (void*)&rem00_device_vol.globalconfiguration,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.globalconfiguration    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};

 
const EOnetvar netvar_r00_globcfg__doit =
{   // pos =  8
    .id         = nvIDrem00dev_globcfg__doit,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(uint8_t),
        .valuedef   = (void*)&rem00_device_def.globalconfiguration.doit, 
        .valuevol   = (void*)&rem00_device_vol.globalconfiguration.doit,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.globalconfiguration.doit    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};


const EOnetvar netvar_r00_globcfg__doalsothat =
{   // pos =  9
    .id         = nvIDrem00dev_globcfg__doalsothat,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(uint8_t),
        .valuedef   = (void*)&rem00_device_def.globalconfiguration.doalsothat, 
        .valuevol   = (void*)&rem00_device_vol.globalconfiguration.doalsothat,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.globalconfiguration.doalsothat    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};


const EOnetvar netvar_r00_globcfg__withperiodinmicrosecs =
{   // pos =  10
    .id         = nvIDrem00dev_globcfg__withperiodinmicrosecs,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(uint32_t),
        .valuedef   = (void*)&rem00_device_def.globalconfiguration.withperiodinmicrosecs, 
        .valuevol   = (void*)&rem00_device_vol.globalconfiguration.withperiodinmicrosecs,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.globalconfiguration.withperiodinmicrosecs    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};


const EOnetvar netvar_r00_input08 =
{   // pos =  11
    .id         = nvIDrem00dev_input08,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(eOentity_input08_t),
        .valuedef   = (void*)&rem00_device_def.input08, 
        .valuevol   = (void*)&rem00_device_vol.input08,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.input08    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};




const EOnetvar netvar_r00_input08__input08value =
{   // pos =  12
    .id         = nvIDrem00dev_input08__input08value,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(uint8_t),
        .valuedef   = (void*)&rem00_device_def.input08.input08value, 
        .valuevol   = (void*)&rem00_device_vol.input08.input08value,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.input08.input08value    
    },
    .datainterface = &ondata_r00_input08__input08value,
    .on_rop_reception = &onrop_r00_input08__input08value
};

 
const EOnetvar netvar_r00_input08__acquisitiontime =
{   // pos =  13
    .id         = nvIDrem00dev_input08__acquisitiontime,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(uint64_t),
        .valuedef   = (void*)&rem00_device_def.input08.acquisitiontime, 
        .valuevol   = (void*)&rem00_device_vol.input08.acquisitiontime,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.input08.acquisitiontime    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};

 
const EOnetvar netvar_r00_input08_inputconfiguration =
{   // pos =  14
    .id         = nvIDrem00dev_input08_inputconfiguration,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(eOentity_input_cfg_t),
        .valuedef   = (void*)&rem00_device_def.input08.inputconfiguration, 
        .valuevol   = (void*)&rem00_device_vol.input08.inputconfiguration,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.input08.inputconfiguration    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};


const EOnetvar netvar_r00_input08_inpcfg__acquisitionenabled =
{   // pos =  15
    .id         = nvIDrem00dev_input08_inpcfg__acquisitionenabled,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(uint8_t),
        .valuedef   = (void*)&rem00_device_def.input08.inputconfiguration.acquisitionenabled, 
        .valuevol   = (void*)&rem00_device_vol.input08.inputconfiguration.acquisitionenabled,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.input08.inputconfiguration.acquisitionenabled    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};

 
const EOnetvar netvar_r00_input08_inpcfg__acquisitionperiod =
{   // pos =  16
    .id         = nvIDrem00dev_input08_inpcfg__acquisitionperiod,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(uint32_t),
        .valuedef   = (void*)&rem00_device_def.input08.inputconfiguration.acquisitionperiod, 
        .valuevol   = (void*)&rem00_device_vol.input08.inputconfiguration.acquisitionperiod,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.input08.inputconfiguration.acquisitionperiod    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};


const EOnetvar netvar_r00_input16 =
{   // pos =  17
    .id         = nvIDrem00dev_input16,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(eOentity_input16_t),
        .valuedef   = (void*)&rem00_device_def.input16, 
        .valuevol   = (void*)&rem00_device_vol.input16,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.input16    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};



const EOnetvar netvar_r00_input16__input16value =
{   // pos =  18
    .id         = nvIDrem00dev_input16__input16value,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(uint16_t),
        .valuedef   = (void*)&rem00_device_def.input16.input16value, 
        .valuevol   = (void*)&rem00_device_vol.input16.input16value,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.input16.input16value    
    },
    .datainterface = &ondata_r00_input16__input16value,
    .on_rop_reception = &onrop_r00_input16__input16value
};

 
const EOnetvar netvar_r00_input16__acquisitiontime =
{   // pos =  19
    .id         = nvIDrem00dev_input16__acquisitiontime,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(uint64_t),
        .valuedef   = (void*)&rem00_device_def.input16.acquisitiontime, 
        .valuevol   = (void*)&rem00_device_vol.input16.acquisitiontime,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.input16.acquisitiontime    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};


const EOnetvar netvar_r00_input16_inputconfiguration =
{   // pos =  20
    .id         = nvIDrem00dev_input16_inputconfiguration,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(eOentity_input_cfg_t),
        .valuedef   = (void*)&rem00_device_def.input16.inputconfiguration, 
        .valuevol   = (void*)&rem00_device_vol.input16.inputconfiguration,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.input16.inputconfiguration    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};


const EOnetvar netvar_r00_input16_inpcfg__acquisitionenabled =
{   // pos =  21
    .id         = nvIDrem00dev_input16_inpcfg__acquisitionenabled,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {   
        .capacity   = sizeof(uint8_t),
        .valuedef   = (void*)&rem00_device_def.input16.inputconfiguration.acquisitionenabled, 
        .valuevol   = (void*)&rem00_device_vol.input16.inputconfiguration.acquisitionenabled,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.input16.inputconfiguration.acquisitionenabled    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};


const EOnetvar netvar_r00_input16_inpcfg__acquisitionperiod =
{   // pos =  22
    .id         = nvIDrem00dev_input16_inpcfg__acquisitionperiod,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(uint32_t),
        .valuedef   = (void*)&rem00_device_def.input16.inputconfiguration.acquisitionperiod, 
        .valuevol   = (void*)&rem00_device_vol.input16.inputconfiguration.acquisitionperiod,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.input16.inputconfiguration.acquisitionperiod    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};



const EOnetvar netvar_r00_input32 =
{   // pos =  23
    .id         = nvIDrem00dev_input32,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(eOentity_input32_t),
        .valuedef   = (void*)&rem00_device_def.input32, 
        .valuevol   = (void*)&rem00_device_vol.input32,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.input32    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};

 
const EOnetvar netvar_r00_input32__input32value =
{   // pos =  24
    .id         = nvIDrem00dev_input32__input32value,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(uint32_t),
        .valuedef   = (void*)&rem00_device_def.input32.input32value, 
        .valuevol   = (void*)&rem00_device_vol.input32.input32value,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.input32.input32value    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};


const EOnetvar netvar_r00_input32__acquisitiontime =
{   // pos =  25
    .id         = nvIDrem00dev_input32__acquisitiontime,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(uint64_t),
        .valuedef   = (void*)&rem00_device_def.input32.acquisitiontime, 
        .valuevol   = (void*)&rem00_device_vol.input32.acquisitiontime,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.input32.acquisitiontime    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};


const EOnetvar netvar_r00_input32_inputconfiguration =
{   // pos =  26
    .id         = nvIDrem00dev_input32_inputconfiguration,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(eOentity_input_cfg_t),
        .valuedef   = (void*)&rem00_device_def.input32.inputconfiguration, 
        .valuevol   = (void*)&rem00_device_vol.input32.inputconfiguration,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.input32.inputconfiguration    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};


const EOnetvar netvar_r00_input32_inpcfg__acquisitionenabled =
{   // pos =  27
    .id         = nvIDrem00dev_input32_inpcfg__acquisitionenabled,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(uint8_t),
        .valuedef   = (void*)&rem00_device_def.input32.inputconfiguration.acquisitionenabled, 
        .valuevol   = (void*)&rem00_device_vol.input32.inputconfiguration.acquisitionenabled,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.input32.inputconfiguration.acquisitionenabled    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};


const EOnetvar netvar_r00_input32_inpcfg__acquisitionperiod =
{   // pos =  28
    .id         = nvIDrem00dev_input32_inpcfg__acquisitionperiod, 
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(uint32_t),
        .valuedef   = (void*)&rem00_device_def.input32.inputconfiguration.acquisitionperiod, 
        .valuevol   = (void*)&rem00_device_vol.input32.inputconfiguration.acquisitionperiod,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.input32.inputconfiguration.acquisitionperiod    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};



const EOnetvar netvar_r00_output =
{   // pos =  29
    .id         = nvIDrem00dev_output,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(eOentity_output_t),
        .valuedef   = (void*)&rem00_device_def.output, 
        .valuevol   = (void*)&rem00_device_vol.output,
        .mirror     = NULL,
        .flex.rem_valuerec   = (void*)&rem00_device_rec.output    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};

 
const EOnetvar netvar_r00_output__outputvalue =
{   // pos =  30
    .id         = nvIDrem00dev_output__outputvalue,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(uint32_t),
        .valuedef   = (void*)&rem00_device_def.output.outputvalue, 
        .valuevol   = (void*)&rem00_device_vol.output.outputvalue,
        .mirror     = EOVPORT_GET_Mirror(rem00_device_vol.vportWOnly, nvWOmir_rem00dev_output__outputvalue),
        .flex.rem_valuerec   = (void*)&rem00_device_rec.output.outputvalue    
    },
    .datainterface = &ondata_r00_output__outputvalue,
    .on_rop_reception = NULL
};


const EOnetvar netvar_r00_output__applicationtime =
{   // pos =  31
    .id         = nvIDrem00dev_output__applicationtime,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(uint64_t),
        .valuedef   = (void*)&rem00_device_def.output.applicationtime, 
        .valuevol   = (void*)&rem00_device_vol.output.applicationtime,
        .mirror     = EOVPORT_GET_Mirror(rem00_device_vol.vportWOnly, nvWOmir_rem00dev_output__applicationtime), 
        .flex.rem_valuerec   = (void*)&rem00_device_rec.output.applicationtime    
    },
    .datainterface = &ondata_r00_output__applicationtime,
    .on_rop_reception = NULL
};
                                 

const EOnetvar netvar_r00_output_outputconfiguration =
{   // pos =  32
    .id         = nvIDrem00dev_output_outputconfiguration,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(eOentity_output_cfg_t),
        .valuedef   = (void*)&rem00_device_def.output.outputconfiguration, 
        .valuevol   = (void*)&rem00_device_vol.output.outputconfiguration,
        .mirror     = EOVPORT_GET_Mirror(rem00_device_vol.vportWOnly, nvWOmir_rem00dev_output__outputconfiguration),
        .flex.rem_valuerec   = (void*)&rem00_device_rec.output.outputconfiguration    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};


const EOnetvar netvar_r00_output_outcfg__acquisitionenabled =
{   // pos =  33
    .id         = nvIDrem00dev_output_outcfg__acquisitionenabled,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(uint8_t),
        .valuedef   = (void*)&rem00_device_def.output.outputconfiguration.applicationenabled, 
        .valuevol   = (void*)&rem00_device_vol.output.outputconfiguration.applicationenabled,
        .mirror     = EOVPORT_GET_Mirror(rem00_device_vol.vportWOnly, nvWOmir_rem00dev_output_outcfg__acquisitionenabled), 
        .flex.rem_valuerec   = (void*)&rem00_device_rec.output.outputconfiguration.applicationenabled    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};


const EOnetvar netvar_r00_output_outcfg__applicationmode =
{   // pos =  34
    .id         = nvIDrem00dev_output_outcfg__applicationmode,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(uint32_t),
        .valuedef   = (void*)&rem00_device_def.output.outputconfiguration.applicationmode, 
        .valuevol   = (void*)&rem00_device_vol.output.outputconfiguration.applicationmode,
        .mirror     = EOVPORT_GET_Mirror(rem00_device_vol.vportWOnly, nvWOmir_rem00dev_output_outcfg__applicationmode), 
        .flex.rem_valuerec   = (void*)&rem00_device_rec.output.outputconfiguration.applicationmode    
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};



const EOnetvar netvar_r00__fixedarray =
{   // pos =  35
    .id         = nvIDrem00dev__fixedarray,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(rem00_device_def.fixedarray),
        .valuedef   = (void*)&rem00_device_def.fixedarray, 
        .valuevol   = (void*)&rem00_device_vol.fixedarray,
        .mirror     = EOVPORT_GET_Mirror(rem00_device_vol.vportWOnly, nvWOmir_rem00dev__fixedarray),
        .flex.rem_valuerec   = (void*)&rem00_device_rec.fixedarray
    },
    .datainterface = &ondata_r00_generic_datainterface,
    .on_rop_reception = NULL
};


const EOnetvar netvar_r00__varsizearray =
{   // pos =  36
    .id                 = nvIDrem00dev__varsizearray,
    .ownership  = eo_nv_ownership_remote,
    .data = 
    {
        .capacity   = sizeof(rem00_device_def.varsizearray),
        .valuedef   = (void*)&rem00_device_def.varsizearray, 
        .valuevol   = (void*)&rem00_device_vol.varsizearray,
        .mirror     = EOVPORT_GET_Mirror(rem00_device_vol.vportWOnly, nvWOmir_rem00dev__varsizearray),
        .flex.rem_valuerec   = (void*)&rem00_device_rec.varsizearray
    },
    .datainterface     = &ondata_r00_generic_datainterface,
    .on_rop_reception   = NULL
};


// --------------------------------------------------------------------------------------------------------------------
// here is the array of netvar nodes.  
// they are ordered upon a depth first reading of the tree

const EOnetvarNode rem00_device_thenetvarnodes[] =
{
/**/{   // pos = 0
        .nchildren  = 9,
        .children   = {1, 4, 7, 11, 17, 23, 29, 35, 36},     
        .netvar     = &netvar_r00_device
    },
    
/**/{   // pos = 1
        .nchildren  = 2,
        .children   = {2, 3},
        .netvar     = &netvar_r00_vportROnly
    },
        {   // pos = 2
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_r00_vportROnly__cfg
        },
        {   // pos = 3
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_r00_vportROnly__dat
        },

/**/{   // pos = 4
        .nchildren  = 2,
        .children   = {5, 6},
        .netvar     = &netvar_r00_vportWOnly
    },
        {   // pos = 5
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_r00_vportWOnly__cfg
        },
        {   // pos = 6
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_r00_vportWOnly__dat
        },

/**/{   // pos = 7
        .nchildren  = 3,
        .children   = {8, 9, 10},
        .netvar     = &netvar_r00_globalconfiguration
    },
        {   // pos = 8
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_r00_globcfg__doit
        },
        {   // pos = 9
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_r00_globcfg__doalsothat
        },
        {   // pos = 10
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_r00_globcfg__withperiodinmicrosecs
        },

/**/{   // pos = 11
        .nchildren  = 3,
        .children   = {12, 13, 14},
        .netvar     = &netvar_r00_input08
    },
        {   // pos = 12
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_r00_input08__input08value
        },
        {   // pos = 13
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_r00_input08__acquisitiontime
        },
        {   // pos = 14
            .nchildren  = 2,
            .children   = {15, 16},
            .netvar     = &netvar_r00_input08_inputconfiguration
        },
            {   // pos = 15
                .nchildren  = 0,
                .children   = {255},
                .netvar     = &netvar_r00_input08_inpcfg__acquisitionenabled
            },
            {   // pos = 16
                .nchildren  = 0,
                .children   = {255},
                .netvar     = &netvar_r00_input08_inpcfg__acquisitionperiod
            },

/**/{   // pos = 17
        .nchildren  = 3,
        .children   = {18, 19, 20},
        .netvar     = &netvar_r00_input16
    },
        {   // pos = 18
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_r00_input16__input16value
        },
        {   // pos = 19
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_r00_input16__acquisitiontime
        },
        {   // pos = 20
            .nchildren  = 2,
            .children   = {21, 22},
            .netvar     = &netvar_r00_input16_inputconfiguration
        },
            {   // pos = 21
                .nchildren  = 0,
                .children   = {255},
                .netvar     = &netvar_r00_input16_inpcfg__acquisitionenabled
            },
            {   // pos = 22
                .nchildren  = 0,
                .children   = {255},
                .netvar     = &netvar_r00_input16_inpcfg__acquisitionperiod
            },

/**/{   // pos = 23
        .nchildren  = 3,
        .children   = {24, 25, 26},
        .netvar     = &netvar_r00_input32
    },
        {   // pos = 24
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_r00_input32__input32value
        },
        {   // pos = 25
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_r00_input32__acquisitiontime
        },
        {   // pos = 26
            .nchildren  = 2,
            .children   = {27, 28},
            .netvar     = &netvar_r00_input32_inputconfiguration
        },
            {   // pos = 27
                .nchildren  = 0,
                .children   = {255},
                .netvar     = &netvar_r00_input32_inpcfg__acquisitionenabled
            },
            {   // pos = 28
                .nchildren  = 0,
                .children   = {255},
                .netvar     = &netvar_r00_input32_inpcfg__acquisitionperiod
            },

/**/{   // pos = 29
        .nchildren  = 3,
        .children   = {30, 31, 32},
        .netvar     = &netvar_r00_output
    },
        {   // pos = 30
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_r00_output__outputvalue
        },
        {   // pos = 31
            .nchildren  = 0,
            .children   = {255},
            .netvar     = &netvar_r00_output__applicationtime
        },
        {   // pos = 32
            .nchildren  = 2,
            .children   = {33, 34},
            .netvar     = &netvar_r00_output_outputconfiguration
        },
            {   // pos = 33
                .nchildren  = 0,
                .children   = {255},
                .netvar     = &netvar_r00_output_outcfg__acquisitionenabled
            },
            {   // pos = 34
                .nchildren  = 0,
                .children   = {255},
                .netvar     = &netvar_r00_output_outcfg__applicationmode
            },

/**/{   // pos = 35
        .nchildren  = 0,
        .children   = {255},
        .netvar     = &netvar_r00__fixedarray
    },

/**/{   // pos = 36
        .nchildren  = 0,
        .children   = {255},
        .netvar     = &netvar_r00__varsizearray
    }

};


// --------------------------------------------------------------------------------------------------------------------
// control of the correct size of the array
typedef uint8_t s_r00_dummy_t[((sizeof(rem00_device_thenetvarnodes)/sizeof(EOnetvarNode)-rem00_device_NUMNETVARS) == 0) ? 1 : 0];



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern void nvscfg_device_rem00_init_volatile_data(void)
{
    uint16_t i = 0;
    // set _rec to zero, as it is only written with data inside the received rops 
    memset(&rem00_device_rec, 0, sizeof(eOdevice_Rem00_t));

    // set _vol to the default value 
    memcpy(&rem00_device_vol, &rem00_device_def, sizeof(eOdevice_Rem00_t));

    // run the .init() on the the nvs
    for(i=0; i<rem00_device_NUMNETVARS; i++)
    {
        eo_netvar_Init(rem00_device_thenetvarnodes[i].netvar);
    }
    
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



