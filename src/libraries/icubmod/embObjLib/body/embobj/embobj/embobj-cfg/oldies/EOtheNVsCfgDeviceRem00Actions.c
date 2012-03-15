
/** @file       EOtheNVsCfgDeviceRem00Actions.c
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
#include "EOnetvar_hid.h"
#include "EOnetvar.h"


#include "EOtheNVsCfgDeviceRem00_hid.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgDeviceRem00Actions.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgDeviceRem00Actions_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section





// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------

extern const eOnetvar_fn_datainterface_t ondata_r00_generic_datainterface =
{
    .init       = NULL,
    .update     = nvscfg_device_rem00_update_generic
};

extern const eOnetvar_fn_datainterface_t ondata_r00_vportROnly__dat =
{
    .init       = NULL,
    .update     = nvscfg_device_rem00_move_vportROnly_dat_to_nvs
};

extern const eOnetvar_fn_datainterface_t ondata_r00_vportWOnly__cfg =
{
    .init       = nvscfg_device_rem00_configure_vportWOnly,
    .update     = nvscfg_device_rem00_configure_vportWOnly
};

extern const eOnetvar_fn_datainterface_t ondata_r00_input08__input08value =
{
    .init       = NULL,
    .update     = nvscfg_device_rem00_input08value_update
};

extern const eOnetvar_fn_datainterface_t ondata_r00_input16__input16value =
{
    .init       = NULL,
    .update     = nvscfg_device_rem00_input16value_update
};

extern const eOnetvar_fn_datainterface_t ondata_r00_output__outputvalue =
{
    .init       = NULL,
    .update     = nvscfg_device_rem00_outputvalue_update
};

extern const eOnetvar_fn_datainterface_t ondata_r00_output__applicationtime =
{
    .init       = NULL,
    .update     = nvscfg_device_rem00_applicationtime_update
};


extern const eOnetvar_onrop_rx_t onrop_r00_generic_onrop =
{
    .rem = 
    { 
        .say =
        {
            EONETVAR_ONROP_SET_2FN(nvscfg_device_rem00_before_say, nvscfg_device_rem00_after_say)
        },
        .sig =
        {
            EONETVAR_ONROP_SET_2FN(nvscfg_device_rem00_before_sig, nvscfg_device_rem00_after_sig)
        }
    }
};


extern const eOnetvar_onrop_rx_t onrop_r00_vportROnly__dat =
{
    .rem = 
    { 
        .say =
        {
            EONETVAR_ONROP_SET_2FN(NULL, nvscfg_device_rem00_act_AFT_SAY_var_vportROnly_dat)
        },
        .sig =
        {
            EONETVAR_ONROP_SET_2FN(NULL, nvscfg_device_rem00_act_AFT_SIG_var_vportROnly_dat)
        }
    }
};

extern const eOnetvar_onrop_rx_t onrop_r00_vportWOnly__dat =
{
    .rem = 
    { 
        .say =
        {
            EONETVAR_ONROP_SET_2FN(NULL, nvscfg_device_rem00_act_AFT_SAY_var_vportWOnly_dat)
        },
        .sig =
        {
            EONETVAR_ONROP_SET_2FN(NULL, nvscfg_device_rem00_act_AFT_SIG_var_vportWOnly_dat)
        }
    }
};

extern const eOnetvar_onrop_rx_t onrop_r00_input08__input08value =
{
    .rem = 
    { 
        .say =
        {
            EONETVAR_ONROP_SET_2FN(NULL, nvscfg_device_rem00_act_AFT_SAY_var_input08value)
        },
        .sig =
        {
            EONETVAR_ONROP_SET_2FN(NULL, nvscfg_device_rem00_act_AFT_SIG_var_input08value)
        }
    }
};

extern const eOnetvar_onrop_rx_t onrop_r00_input16__input16value =
{
    .rem = 
    { 
        .say =
        {
            EONETVAR_ONROP_SET_2FN(NULL, nvscfg_device_rem00_act_AFT_SAY_var_input16value)
        },
        .sig =
        {
            EONETVAR_ONROP_SET_2FN(NULL, nvscfg_device_rem00_act_AFT_SIG_var_input16value)
        }
    }
}; 


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

void s_increment_sig(void);
void s_increment_say(void);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables

static const char s_funstr[][5] =
{
    "NO0",
    "NO1",
    "mix",
    "con",
    "cfg",
    "beh",
    "inp",
    "out"
};

static const char s_typstr[][5] =
{
    "U08",
    "U16",
    "U32",
    "U64",
    "NO4",
    "NO5",
    "arr",
    "pkd"
};


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------



extern void nvscfg_device_rem00_configure_vportWOnly(void *p) //s_configure_vportWOnly(void *p)
{
    eo_vport_hid_LoadCfg(&rem00_device_vol.vportWOnly, &rem00_device_vol.vportWOnly.cfg, eo_nv_ownership_remote, rem00_device_ipaddr);
}


extern void nvscfg_device_rem00_update_generic(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    uint8_t value[64] = {0};
    uint16_t size = 0;
    eOnetvarFunc_t fun;
    eOnetvarType_t typ;
    uint16_t off;
    uint16_t i;


    eo_netvar_remoteGet(nv, &value, &size);

    fun = eo_netvar_fromIDtoFUN(nv->id);
    typ = eo_netvar_fromIDtoTYP(nv->id);
    off = eo_netvar_fromIDtoOFF(nv->id);


    printf("smart node has received from rem00:\n");
    printf("  NV.id = 0x%x, fun = %s, typ = %s, off = %d\n", nv->id, s_funstr[fun], s_typstr[typ], off); 
    
    if(typ <= eo_nv_TYP_u64)
    {
    printf("  size = %d, value = 0x", size);
        for(i=0; i<size; i++)
                           printf("%x", value[size-i-1]);
    printf(".\n");
    }
    else if(typ == eo_nv_TYP_pkd)
    {
    printf("  size = %d, value = [", size);
        for(i=0; i<size; i++)
                         printf("0x%x ", value[i]);
    printf("].\n");
    }
    else if(typ == eo_nv_TYP_arr)
    {
    printf("  size = %d, n = %d value = [", size, *((uint16_t*)value));
        for(i=0; i<size-2; i++)
                         printf("0x%x ", value[i+2]);
    printf("].\n");
    }


}

extern void nvscfg_device_rem00_outputvalue_update(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    uint32_t value = 0;
    uint16_t size = 0;

    eo_netvar_remoteGet(nv, &value, &size);
    printf("smartnode has updated rem00::outputvalue of size %d w/ value %d\n", size, value);    
}


extern void nvscfg_device_rem00_applicationtime_update(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    uint64_t value = 0;
    uint16_t size = 0;

    eo_netvar_remoteGet(nv, &value, &size);
    printf("smartnode has updated rem00::applicationtime of size %d w/ value %lld\n", size, value);                        
}

extern void nvscfg_device_rem00_input08value_update(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    uint8_t value = 0;
    uint16_t size = 0;

    eo_netvar_remoteGet(nv, &value, &size);
    printf("smartnode has updated rem00::input08value of size %d w/ value %d\n", size, value); 
}

extern void nvscfg_device_rem00_act_AFT_SIG_var_input08value(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    
    s_increment_sig();
    
}

extern void nvscfg_device_rem00_act_AFT_SAY_var_input08value(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    
    s_increment_say();
    
}


extern void nvscfg_device_rem00_input16value_update(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    uint16_t value = 0;
    uint16_t size = 0;

    eo_netvar_remoteGet(nv, &value, &size);
    printf("smartnode has updated rem00::input16value of size %d w/ value %d\n", size, value);   
}

extern void nvscfg_device_rem00_act_AFT_SAY_var_input16value(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    
    s_increment_say();
    
}
                                              
extern void nvscfg_device_rem00_act_AFT_SIG_var_input16value(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    
    s_increment_say();
    
}


extern void nvscfg_device_rem00_move_vportROnly_dat_to_nvs(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    printf("i am a smart node which received data from vportROnly\n");
    eo_vport_hid_MoveDat2NVs(&rem00_device_rec.vportROnly, eo_nv_ownership_remote, rem00_device_ipaddr);
}

extern void nvscfg_device_rem00_act_AFT_SIG_var_vportROnly_dat(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
}

extern void nvscfg_device_rem00_act_AFT_SAY_var_vportROnly_dat(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
}


extern void nvscfg_device_rem00_act_AFT_SIG_var_vportWOnly_dat(void *p)
{
    
}

extern void nvscfg_device_rem00_act_AFT_SAY_var_vportWOnly_dat(void *p)
{
    
}


extern void nvscfg_device_rem00_before_sig(void *p)
{
}

extern void nvscfg_device_rem00_after_sig(void *p)
{
}

extern void nvscfg_device_rem00_before_say(void *p)
{
}

extern void nvscfg_device_rem00_after_say(void *p)
{
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


void s_increment_sig(void)
{
    static uint32_t signum = 0;
    
    signum ++;
}

void s_increment_say(void)
{
    static uint32_t saynum = 0;
    
    saynum ++;
}






// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



