
/** @file       eOcfg_NVs_node00_loc_act.c
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


#include "hal.h"
#include "EOtimer.h"
#include "EOaction_hid.h"
#include "EOVtheCallbackManager.h"

#include "eOcfg_NVs_node00_loc.h"

#include "EOvport_hid.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eOcfg_NVs_node00_loc_act.h"


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

static void s_print_stats(EOnetvar* nv);

static void s_node00_loc_action_button_update(void *p);

static void s_node00_loc_action_signal_vportrodat(void *p);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_root =
{
    .init       = node00_loc_action_init_nvOBJ_root,
    .update     = node00_loc_action_update_nvOBJ_root
};

extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_generic =
{
    .init       = node00_loc_action_init_generic,
    .update     = node00_loc_action_update_generic
};

extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_vportRO__cfg =
{
    .init       = node00_loc_action_init_nvOBJ_vportRO__cfg,
    .update     = node00_loc_action_update_nvOBJ_vportRO__cfg
};

extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_vportRO__dat =
{
    .init       = node00_loc_action_init_nvOBJ_vportRO__dat,
    .update     = NULL
};


extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_globalconfiguration__any =
{
    .init       = NULL,                          
    .update     = node00_loc_action_update_nvOBJ_globalconfiguration__any
};

extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_button =
{
    .init       = node00_loc_action_init_nvOBJ_button,
    .update     = NULL
};

extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_button__inputval =
{
    .init       = NULL,
    .update     = node00_loc_action_update_nvOBJ_button__inputval
};

extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_led00 =
{
    .init       = node00_loc_action_init_nvOBJ_led00,
    .update     = NULL
};

extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_led00__outputval =
{
    .init       = NULL,
    .update     = node00_loc_action_update_nvOBJ_led00__outputval
};

extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_led01 =
{
    .init       = node00_loc_action_init_nvOBJ_led01,
    .update     = NULL
};

extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ_led01__outputval =
{
    .init       = NULL,
    .update     = node00_loc_action_update_nvOBJ_led01__outputval
};

extern const eOnetvar_fn_peripheralinterface_t eo_cfg_nvs_node00_loc_action_peripheralinterface_nvOBJ__timeoflife =
{
    .init       = node00_loc_action_init_nvOBJ__timeoflife,
    .update     = node00_loc_action_update_nvOBJ__timeoflife
};




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

static EOaction *act = NULL;

static EOtimer *tmrbutton = NULL;
static EOtimer *tmrvportrodat = NULL;
// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern void node00_loc_action_init_generic(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    printf("\n node00loc init: generic (print stats)\n");
    s_print_stats(nv);
}

extern void node00_loc_action_update_generic(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    printf("\n node00loc update: generic (print stats)\n");
    s_print_stats(nv);
}


extern void node00_loc_action_init_nvOBJ_root(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    act = eo_action_New();
    printf("\n node00loc init: _root .. called eo_action_New() and put in here any initialisation of static variables\n");
}

extern void node00_loc_action_update_nvOBJ_root(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    printf("\n node00loc update: _root (do nothing)\n");
}

extern void node00_loc_action_init_nvOBJ_vportRO__cfg(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    printf("\n node00loc init: _vportro__cfg -> call eo_vport_hid_LoadCfg()\n");
    eo_vport_hid_LoadCfg(&eo_cfg_nvs_node00_loc_vol.vportRO, &eo_cfg_nvs_node00_loc_vol.vportRO.cfg, eo_nv_ownership_local, (eOipv4addr_t)0);
}

extern void node00_loc_action_update_nvOBJ_vportRO__cfg(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    printf("\n node00loc update: _vportro__cfg -> call eo_vport_hid_LoadCfg()\n");
    eo_vport_hid_LoadCfg(&eo_cfg_nvs_node00_loc_vol.vportRO, &eo_cfg_nvs_node00_loc_vol.vportRO.cfg, eo_nv_ownership_local, (eOipv4addr_t)0);
}

extern void node00_loc_action_init_nvOBJ_vportRO__dat(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    printf("\n node00loc init: _vportro__dat\n");
    printf("if __signalvportro is true then:\n");
    printf("start a timer w/ period __withperiod to send a sig<vportro__dat> to device __toipaddr\n");
    printf("but better moving it in side the config netvar __signalvportro\n");

    tmrvportrodat = eo_timer_New();

    if((1 == eo_cfg_nvs_node00_loc_vol.globalconfiguration.signalvportro) && (0!= eo_cfg_nvs_node00_loc_vol.globalconfiguration.withperiod))
    {
        printf("started the timer\n");
        eo_action_SetCallback(act, s_node00_loc_action_signal_vportrodat, NULL, eov_callbackman_GetTask(eov_callbackman_GetHandle()));
        eo_timer_Start(tmrvportrodat, eok_abstimeNOW, eo_cfg_nvs_node00_loc_vol.globalconfiguration.withperiod, eo_tmrmode_FOREVER, act);
    }
}

extern void node00_loc_action_update_nvOBJ_globalconfiguration__any(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    printf("\n node00loc update: any leaf node in _globalconfiguration:\n");
    printf("stop any activity related to the value of the leaf node, and restart w/ new value:\n");
    printf("IF ... the activities use a local copy of teh value taken at init() phase, then ok this way \n");
    printf("ELSE ... we need to use the before_ and after_ set/rst to stop activity and then restart it \n");
    printf("         and no need of the update() \n");

}


extern void node00_loc_action_init_nvOBJ_button(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;

    
    nv = nv;
    printf("\n node00loc init: _button\n");
    printf("if __acquireinput then start an osal timer w/ period __acquisitionperiod whcih ...\n");
    printf("uses a hal function to get the value of button_inputval and puts val and time in proper nv using eo_netvar_Set() so that mirrors are correctly managed\n");
    printf("the above is done by calling __inputval.update()\n");

    // init the gpio
    hal_gpio_init(hal_gpio_portB, hal_gpio_pin7, hal_gpio_dirINP, hal_gpio_speed_low);

    // create the timer
    tmrbutton = eo_timer_New();
    
    if((1 == eo_cfg_nvs_node00_loc_vol.globalconfiguration.acquireinput) && (0 != eo_cfg_nvs_node00_loc_vol.globalconfiguration.acquisitionperiod))
    {
        printf("start the timer\n");
        s_node00_loc_action_button_update(NULL);
       
        eo_action_SetCallback(act, s_node00_loc_action_button_update, NULL, eov_callbackman_GetTask(eov_callbackman_GetHandle()));
        eo_timer_Start(tmrbutton, eok_abstimeNOW, eo_cfg_nvs_node00_loc_vol.globalconfiguration.acquisitionperiod, eo_tmrmode_FOREVER, act);
    }
}


extern void node00_loc_action_update_nvOBJ_button__inputval(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    printf("\n node00loc update: _button__inputval\n");
    printf("use a hal function to get the value of button_inputval and puts val and time in proper nv using eo_netvar_Set() so that mirrors are correctly managed\n");
    
    s_node00_loc_action_button_update(NULL);
}

extern void node00_loc_action_init_nvOBJ_led00(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    eOcfg_nvs_node00_loc_output_t led00;
    uint16_t size;
    
    nv = nv;
   
    printf("\n node00loc init: _led00\n");
    printf("initted led00__lednumber w/ value led00_outputval using hal\n");
    
    eo_netvar_Get(nv, eo_nv_strg_volatile, &led00, &size);
    
    hal_gpio_init(hal_gpio_portE, (hal_gpio_pin_t)led00.lednumber, hal_gpio_dirOUT, hal_gpio_speed_low);
    hal_gpio_setval(hal_gpio_portE, (hal_gpio_pin_t)led00.lednumber, (hal_gpio_val_t)led00.outputval);
}


extern void node00_loc_action_update_nvOBJ_led00__outputval(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
//    eOcfg_nvs_node00_loc_output_t led00;
//    uint16_t size;
    
    nv = nv;
    
    printf("\n node00loc update: _led00__outputval\n");
    printf("only if __applyoutput then ...\n");
    printf("i must set led00__lednumber w/ value led00__outputval using hal\n");
    
    if(1 == eo_cfg_nvs_node00_loc_vol.globalconfiguration.applyoutput)
    {
//        eo_netvar_Get(nv, eo_nv_strg_volatile, &led00, &size);
        hal_gpio_setval(hal_gpio_portE, (hal_gpio_pin_t)eo_cfg_nvs_node00_loc_vol.led00.lednumber, (hal_gpio_val_t)eo_cfg_nvs_node00_loc_vol.led00.outputval);    
    }
}

extern void node00_loc_action_init_nvOBJ_led01(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    eOcfg_nvs_node00_loc_output_t led01;
    uint16_t size;
    
    printf("\n node00loc init: _led01\n");
    printf("initted led01__lednumber w/ value led01_outputval using hal\n");
    
    eo_netvar_Get(nv, eo_nv_strg_volatile, &led01, &size);
    
    hal_gpio_init(hal_gpio_portE, (hal_gpio_pin_t)led01.lednumber, hal_gpio_dirOUT, hal_gpio_speed_low);
    hal_gpio_setval(hal_gpio_portE, (hal_gpio_pin_t)led01.lednumber, (hal_gpio_val_t)led01.outputval);
}



extern void node00_loc_action_update_nvOBJ_led01__outputval(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
//    eOcfg_nvs_node00_loc_output_t led00;
//    uint16_t size;
    
    nv = nv;
    
    printf("\n node00loc update: _led01__outputval\n");
    printf("only if __applyoutput then ...\n");
    printf("i must set led01__lednumber w/ value led01__outputval using hal\n");
    
    if(1 == eo_cfg_nvs_node00_loc_vol.globalconfiguration.applyoutput)
    {
//        eo_netvar_Get(nv, eo_nv_strg_volatile, &led00, &size);
        hal_gpio_setval(hal_gpio_portE, (hal_gpio_pin_t)eo_cfg_nvs_node00_loc_vol.led01.lednumber, (hal_gpio_val_t)eo_cfg_nvs_node00_loc_vol.led01.outputval);    
    }
}

extern void node00_loc_action_init_nvOBJ__timeoflife(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    printf("\n node00loc init: __timeoflife\n");
    printf("nothing to do\n");
}

extern void node00_loc_action_update_nvOBJ__timeoflife(void *p)
{
    EOnetvar *nv = (EOnetvar*)p;
    nv = nv;
    printf("\n node00loc update: __timeoflife\n");
    printf("i must propagate the received value to the rtos w/ osal_system_time_set(timeoflife)\n");
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void s_print_stats(EOnetvar* nv)
{
    uint8_t value[128] = {0};
    uint16_t size = 0;
    eOnetvarFunc_t fun;
    eOnetvarType_t typ;
    uint16_t off;
    uint16_t i;
    
    eo_netvar_remoteGet(nv, &value, &size);

    fun = eo_netvar_fromIDtoFUN(nv->id);
    typ = eo_netvar_fromIDtoTYP(nv->id);
    off = eo_netvar_fromIDtoOFF(nv->id);

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


static void s_node00_loc_action_button_update(void *p)
{
    extern const EOnetvar eo_cfg_nvs_node00_loc_nvOBJ_button__inputval;
    hal_gpio_val_t val;
    val = hal_gpio_getval(hal_gpio_portB, hal_gpio_pin7);
    
    eo_netvar_Set(&eo_cfg_nvs_node00_loc_nvOBJ_button__inputval, &val, eobool_false, eo_nv_upd_dontdo);
}

static void s_node00_loc_action_signal_vportrodat(void *p)
{
    printf("need to send the vportrodat to a given ip address\n");    
}








// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------


