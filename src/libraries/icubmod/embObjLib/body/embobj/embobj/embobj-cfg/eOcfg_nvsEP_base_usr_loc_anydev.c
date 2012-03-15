
/* @file       eOcfg_nvsEP_base_usr_loc_anydev.c
    @brief      This file keeps constant configuration for the NVs of the updater
    @author     marco.accame@iit.it
    @date       09/06/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h" 
#include "string.h"
#include "stdio.h"

#include "EoCommon.h"
#include "eOcfg_nvsEP_base_con.h"
#include "EOtimer.h"
#include "EOMtheCallbackManager.h"
#include "hal_led.h"
#include "EOarray.h"


#include "EOnv_hid.h"


#include "EOconstvector_hid.h"

#include "EOtheARMenvironment.h"
#include "EOVtheEnvironment.h"

#include "eEcommon.h"
#include "shalBASE.h"
#include "shalPART.h"
#include "shalINFO.h"




// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eOcfg_nvsEP_base_usr_loc_anydev.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update__bootprocess(const EOnv* nv, const eOabstime_t time, const uint32_t sign);

static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update__gotoprocess(const EOnv* nv, const eOabstime_t time, const uint32_t sign);

static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update__forcerestart(const EOnv* nv, const eOabstime_t time, const uint32_t sign);

static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update__localise(const EOnv* nv, const eOabstime_t time, const uint32_t sign);

static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update_ipnetwork__macaddress(const EOnv* nv, const eOabstime_t time, const uint32_t sign);

static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update_ipnetwork__ipaddress(const EOnv* nv, const eOabstime_t time, const uint32_t sign);

static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update_ipnetwork__ipnetmask(const EOnv* nv, const eOabstime_t time, const uint32_t sign);
static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update__remoteipaddress(const EOnv* nv, const eOabstime_t time, const uint32_t sign);
static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update__remoteipport(const EOnv* nv, const eOabstime_t time, const uint32_t sign);



static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_gotoprocess_or_forcerestart_callback(void *p);
static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_localise_callback(void *p);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static EOtimer* s_timer = NULL;
static EOaction* s_action = NULL;
static eEprocess_t s_current_eprocess = ee_procNone;

static uint8_t s_counter = 0;


//static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface__bootprocess;
//static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface__gotoprocess;
//static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface__forcerestart;
//static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface__localise;

//static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface_ipnetwork__macaddress;
//static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface_ipnetwork__ipaddress;
//static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface_ipnetwork__ipnetmask;

//static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface__boardinfo =
//{
//    EO_INIT(.init)      NULL,
//    EO_INIT(.update)    NULL
//};


//static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface__applicationinfo =
//{
//    EO_INIT(.init)      NULL,
//    EO_INIT(.update)    NULL
//};


static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface__bootprocess =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update__bootprocess
};


static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface__gotoprocess =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update__gotoprocess
};


static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface__forcerestart =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update__forcerestart
};

static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface__localise =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update__localise
};

static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface_ipnetwork__macaddress =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update_ipnetwork__macaddress
};

static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface_ipnetwork__ipaddress =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update_ipnetwork__ipaddress
};

static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface_ipnetwork__ipnetmask =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update_ipnetwork__ipnetmask
};

static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface__remoteipaddress =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update__remoteipaddress
};

static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface__remoteipport =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update__remoteipport
};


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables
// --------------------------------------------------------------------------------------------------------------------

static const EOnv_usr_t s_eo_cfg_nvsEP_base_usr_loc_anydev_array_of_EOnv_usr[] =
{
    {   // 00 __boardinfo
        EO_INIT(.peripheralinterface)   NULL,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },
    {   // 01 __applicationinfo
        EO_INIT(.peripheralinterface)   NULL,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },    
    {   // 02  _ipnetwork
        EO_INIT(.peripheralinterface)   NULL,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },
    {   // 03 __macaddress
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface_ipnetwork__macaddress,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },    
    {   // 04 __ipaddress
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface_ipnetwork__ipaddress,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },
    {   // 05 __ipnetmask
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface_ipnetwork__ipnetmask,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },
    {   // 06 __bootprocess
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface__bootprocess,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },
    {   // 07 __gotoprocess
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface__gotoprocess,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },    
    {   // 08 __forcerestart
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface__forcerestart,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },
    {   // 09 __localise
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface__localise,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },
    {   // 10 __remoteipaddress
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface__remoteipaddress,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },
    {   // 11 __remoteipport
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_loc_anydev_action_peripheralinterface__remoteipport,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    }
};



const EOconstvector  s_eo_cfg_nvsEP_base_usr_loc_anydev_constvector_of_EOnv_usr = 
{
    EO_INIT(.size)              sizeof(s_eo_cfg_nvsEP_base_usr_loc_anydev_array_of_EOnv_usr)/sizeof(EOnv_usr_t), 
    EO_INIT(.item_size)         sizeof(EOnv_usr_t),
    EO_INIT(.item_array_data)   s_eo_cfg_nvsEP_base_usr_loc_anydev_array_of_EOnv_usr
};


extern const EOconstvector* const eo_cfg_nvsEP_base_usr_loc_anydev_constvector_of_EOnv_usr = &s_eo_cfg_nvsEP_base_usr_loc_anydev_constvector_of_EOnv_usr;

extern eo_cfg_nvsEP_base_t* eo_cfg_nvsEP_base_usr_loc_anydev_mem_local = NULL;


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern void eo_cfg_nvsEP_base_usr_loc_anydev_initialise(void* loc, void* rem)
{    
    eEprocess_t proc = ee_procNone;
    const eEboardInfo_t* boardinfo = NULL;
    const eEmoduleInfo_t *moduleinfo = NULL;
    const shalinfo_deviceinfo_t* deviceinfo = NULL;
    eo_cfg_nvsEP_base_t *vol = (eo_cfg_nvsEP_base_t*)loc;
    // in here rem is NULL....

    uint16_t sss = sizeof(eo_cfg_nvsEP_base_t);

    sss = sss;

    eo_cfg_nvsEP_base_usr_loc_anydev_mem_local = vol;

    // assign default values
    memcpy(vol, &eo_cfg_nvsEP_base_default, sizeof(eo_cfg_nvsEP_base_t)); //sizeof(eo_cfg_nvsEP_base_t));

    eov_env_RunningEprocess_Get(eo_armenv_GetHandle(), &s_current_eprocess);
    
    // boardinfo
    if(ee_res_OK == shalinfo_boardinfo_get(&boardinfo))
    {
        memcpy(&vol->boardinfo, boardinfo, sizeof(eEboardInfo_t));
    }

    // applicationinfo
    proc = s_current_eprocess;
    if(ee_res_OK == shalpart_proc_get(proc, &moduleinfo))
    {
        memcpy(&vol->applicationinfo, moduleinfo, sizeof(eEmoduleInfo_t));
    }

    // bootprocess
    if(ee_res_OK == shalpart_proc_def2run_get(&proc))
    {
        vol->bootprocess = proc;
    }


    // ipnetwork
    if(ee_res_OK == shalinfo_deviceinfo_get(&deviceinfo))
    {
        memcpy(&vol->ipnetwork, &deviceinfo->ipnetwork, sizeof(eEipnetwork_t));
    }

    // all teh rest
    s_timer = eo_timer_New();
    s_action = eo_action_New();

    hal_led_init(hal_led0, NULL);


}

//extern void eo_cfg_nvsEP_base_action_initialise__boardinfo(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
//{

//    void * eo_cfg_nvsEP_base_boardinfo_ptr = NULL;
//    //nv->data.valuedef = (void*)NULL;
//    nv->loc = (void*)eo_cfg_nvsEP_base_boardinfo_ptr;
//    // o piu' semplicemente: assegna a quello puntato da nv->loc quello che hai letto da eeprom
//    // e rimetti i const come prima.
//    // la const la lascio a zero, in quanto e' ronly e quandi non scrivibile.
//    #warning --> put code to retrieve boardinfo pointer.
//}
//
//
//extern void eo_cfg_nvsEP_base_action_initialise__applicationinfo(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
//{
//    void * eo_cfg_nvsEP_base_applicationinfo_ptr = NULL;
//    //nv->data.valuedef = (void*)NULL;
//    nv->loc = (void*)eo_cfg_nvsEP_base_applicationinfo_ptr;
//    #warning --> put code to retrieve applicationinfo pointer.
//}
//
//extern void eo_cfg_nvsEP_base_action_initialise_ipnetwork(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
//{
//}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update__bootprocess(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    eEprocess_t *eproc = nv->loc;

    // force ...

    shalpart_proc_def2run_set(*eproc);
}


static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update__gotoprocess(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    eEprocess_t *eproc = nv->loc;

    // start a timer which executes a callback after 500 msec

    if(eo_tmrstat_Idle != eo_timer_GetStatus(s_timer))
    {
        eo_timer_Stop(s_timer);
    }

    eo_action_SetCallback(s_action, s_eo_cfg_nvsEP_base_usr_loc_anydev_action_gotoprocess_or_forcerestart_callback, eproc, eom_callbackman_GetTask(eom_callbackman_GetHandle()));

    eo_timer_Start(s_timer, eok_abstimeNOW, 500*1000, eo_tmrmode_ONESHOT, s_action); 
    
}


static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update__forcerestart(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    eObool_t *restart = nv->loc;

    if(eobool_false == *restart)
    {
        return;
    }

    // else ... start a timer which executes a callback after 500 msec

    if(eo_tmrstat_Idle != eo_timer_GetStatus(s_timer))
    {
        eo_timer_Stop(s_timer);
    }

    eo_action_SetCallback(s_action, s_eo_cfg_nvsEP_base_usr_loc_anydev_action_gotoprocess_or_forcerestart_callback, NULL, eom_callbackman_GetTask(eom_callbackman_GetHandle()));

    eo_timer_Start(s_timer, eok_abstimeNOW, 500*1000, eo_tmrmode_ONESHOT, s_action); 
    
}


static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update__localise(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    eObool_t *localise = nv->loc;
    

    if(eobool_false == *localise)
    {
        return;
    }

    // else ... start a timer which executes a periodic callback every 250 msec for 20 times

    s_counter = 20;
    
    if(eo_tmrstat_Idle != eo_timer_GetStatus(s_timer))
    {
        eo_timer_Stop(s_timer);
    }

    eo_action_SetCallback(s_action, s_eo_cfg_nvsEP_base_usr_loc_anydev_action_localise_callback, s_timer, eom_callbackman_GetTask(eom_callbackman_GetHandle()));

    eo_timer_Start(s_timer, eok_abstimeNOW, 500*1000, eo_tmrmode_FOREVER, s_action); 
    
}



static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update_ipnetwork__macaddress(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    uint64_t *macaddress = nv->loc;
    eEipnetwork_t *ipnetwork = NULL;

    if(ee_procUpdater != s_current_eprocess)
    {
        return;    
    }

    shalinfo_deviceinfo_part_get(shalinfo_ipnet, (const void **)&ipnetwork);
    
    ipnetwork->macaddress = *macaddress;

    shalinfo_deviceinfo_part_set(shalinfo_ipnet, ipnetwork);
}

static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update_ipnetwork__ipaddress(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    uint32_t *ipaddress = nv->loc;
    eEipnetwork_t *ipnetwork = NULL;

    if(ee_procUpdater != s_current_eprocess)
    {
        return;    
    }

    shalinfo_deviceinfo_part_get(shalinfo_ipnet, (const void **)&ipnetwork);
    
    ipnetwork->ipaddress = *ipaddress;

    shalinfo_deviceinfo_part_set(shalinfo_ipnet, ipnetwork);
}

static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update_ipnetwork__ipnetmask(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    uint32_t *ipnetmask = nv->loc;
    eEipnetwork_t *ipnetwork = NULL;

    if(ee_procUpdater != s_current_eprocess)
    {
        return;    
    }

    shalinfo_deviceinfo_part_get(shalinfo_ipnet, (const void **)&ipnetwork);
    
    ipnetwork->ipnetmask = *ipnetmask;

    shalinfo_deviceinfo_part_set(shalinfo_ipnet, ipnetwork);
}


static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update__remoteipaddress(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    #warning --> in here put code to propagate the remoteipaddress towards the sockets etc (or  put it inside deviceinfo ....)
}

static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_update__remoteipport(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    #warning --> in here put code to propagate the remoteipport towards the sockets etc (or  put it inside deviceinfo ....)
}


// - functions used by _action_update-*


static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_gotoprocess_or_forcerestart_callback(void *p)
{
    eEprocess_t *eproc = (eEprocess_t*)p;

    if(NULL != eproc)
    {
        shalbase_ipc_gotoproc_set(*eproc);
    }

    shalbase_system_restart();   
}

static void s_eo_cfg_nvsEP_base_usr_loc_anydev_action_localise_callback(void *p)
{
    EOtimer *timer = (EOtimer*)p;

    hal_led_toggle(hal_led0);
    s_counter --;

    if((NULL != timer) &&(0 == s_counter))
    {
        hal_led_off(hal_led0);   
        eo_timer_Stop(timer);
    }

}


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



