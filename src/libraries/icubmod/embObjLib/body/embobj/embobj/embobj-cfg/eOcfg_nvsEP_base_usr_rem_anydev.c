
/* @file       eOcfg_nvsEP_base_usr_rem_anydev.c
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

#include "EOnv_hid.h"

#include "EOconstvector_hid.h"

#include "eEcommon.h"
#include "shalBASE.h"
#include "shalPART.h"
#include "shalINFO.h"


#include "hal.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eOcfg_nvsEP_base_usr_rem_anydev.h"


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

static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__boardinfo(const EOnv* nv, const eOabstime_t time, const uint32_t sign);

static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__applicationinfo(const EOnv* nv, const eOabstime_t time, const uint32_t sign);

static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__bootprocess(const EOnv* nv, const eOabstime_t time, const uint32_t sign);

static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__gotoprocess(const EOnv* nv, const eOabstime_t time, const uint32_t sign);

static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__forcerestart(const EOnv* nv, const eOabstime_t time, const uint32_t sign);

static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__localise(const EOnv* nv, const eOabstime_t time, const uint32_t sign);

static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update_ipnetwork(const EOnv* nv, const eOabstime_t time, const uint32_t sign);

static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update_ipnetwork__macaddress(const EOnv* nv, const eOabstime_t time, const uint32_t sign);

static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update_ipnetwork__ipaddress(const EOnv* nv, const eOabstime_t time, const uint32_t sign);

static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update_ipnetwork__ipnetmask(const EOnv* nv, const eOabstime_t time, const uint32_t sign);

static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__remoteipaddress(const EOnv* nv, const eOabstime_t time, const uint32_t sign);

static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__remoteipport(const EOnv* nv, const eOabstime_t time, const uint32_t sign);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface__boardinfo =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__boardinfo
};


static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface__applicationinfo =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__applicationinfo
};


static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface__bootprocess =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__bootprocess
};


static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface__gotoprocess =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__gotoprocess
};


static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface__forcerestart =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__forcerestart
};

static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface__localise =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__localise
};

static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface_ipnetwork =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update_ipnetwork
};

static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface_ipnetwork__macaddress =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update_ipnetwork__macaddress
};

static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface_ipnetwork__ipaddress =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update_ipnetwork__ipaddress
};

static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface_ipnetwork__ipnetmask =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update_ipnetwork__ipnetmask
};

static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface__remoteipaddress =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__remoteipaddress
};

static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface__remoteipport =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__remoteipport
};



static char s_str[128];

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables
// --------------------------------------------------------------------------------------------------------------------

static EOnv_usr_t s_eo_cfg_nvsEP_base_usr_rem_anydev_array_of_EOnv_usr[] =
{
    {   // 00 __boardinfo
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface__boardinfo,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },
    {   // 01 __applicationinfo
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface__applicationinfo,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    }, 
    {   // 02  _ipnetwork
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface_ipnetwork,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },
    {   // 03 __macaddress
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface_ipnetwork__macaddress,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },    
    {   // 04 __ipaddress
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface_ipnetwork__ipaddress,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },
    {   // 05 __ipnetmask
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface_ipnetwork__ipnetmask,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },
    {   // 06 __bootprocess
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface__bootprocess,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },
    {   // 07 __gotoprocess
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface__gotoprocess,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },    
    {   // 08 __forcerestart
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface__forcerestart,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },
    {   // 09 __localise
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface__localise,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },
    {   // 10 __remoteipaddress
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface__remoteipaddress,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    },
    {   // 11 __remoteipport
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_base_usr_rem_anydev_action_peripheralinterface__remoteipport,
        EO_INIT(.on_rop_reception)      NULL,
        EO_INIT(.stg_address)           EOK_uint32dummy
    }
};



const EOconstvector  s_eo_cfg_nvsEP_base_usr_rem_anydev_constvector_of_EOnv_usr = 
{
    EO_INIT(.size)              sizeof(s_eo_cfg_nvsEP_base_usr_rem_anydev_array_of_EOnv_usr)/sizeof(EOnv_usr_t), 
    EO_INIT(.item_size)         sizeof(EOnv_usr_t),
    EO_INIT(.item_array_data)   s_eo_cfg_nvsEP_base_usr_rem_anydev_array_of_EOnv_usr
};


extern const EOconstvector* const eo_cfg_nvsEP_base_usr_rem_anydev_constvector_of_EOnv_usr = &s_eo_cfg_nvsEP_base_usr_rem_anydev_constvector_of_EOnv_usr;


extern eo_cfg_nvsEP_base_t* eo_cfg_nvsEP_base_usr_rem_anydev_mem_local  = NULL;
extern eo_cfg_nvsEP_base_t* eo_cfg_nvsEP_base_usr_rem_anydev_mem_remote = NULL;

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern void eo_cfg_nvsEP_base_usr_rem_anydev_initialise(void* loc, void* rem)
{    
    eo_cfg_nvsEP_base_t *locmem = (eo_cfg_nvsEP_base_t*)loc;
    eo_cfg_nvsEP_base_t *remmem = (eo_cfg_nvsEP_base_t*)rem;

    eo_cfg_nvsEP_base_usr_rem_anydev_mem_local      = locmem;
    eo_cfg_nvsEP_base_usr_rem_anydev_mem_remote     = remmem;

    // assign default values
    memcpy(loc, &eo_cfg_nvsEP_base_default, sizeof(eo_cfg_nvsEP_base_t));
    memcpy(rem, &eo_cfg_nvsEP_base_default, sizeof(eo_cfg_nvsEP_base_t));



    //#warning --> initialise whatever we want with variables
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__boardinfo(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    eEboardInfo_t *boardinfo = nv->rem;

    boardinfo = boardinfo;

    snprintf(s_str, sizeof(s_str)-1, "REM has updated boardinfo w/ boardinfo.info.name = %s", boardinfo->info.name);
    hal_trace_puts(s_str);
}

static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__applicationinfo(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    eEmoduleInfo_t *applicationinfo = nv->rem;
    eOabstime_t t = time;
    uint32_t s = sign;
    applicationinfo = applicationinfo;
    t = t;
    s = s;
    snprintf(s_str, sizeof(s_str)-1, "REM has updated applinfo w/ appl.info.entity.version = (maj = %d, min = %d)", 
                                      applicationinfo->info.entity.version.major, applicationinfo->info.entity.version.minor);
    hal_trace_puts(s_str);

}


static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__bootprocess(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    eEprocess_t *eproc = nv->rem;
    eOabstime_t t = time;
    uint32_t s = sign;
    eproc = eproc;
    t = t;

    snprintf(s_str, sizeof(s_str)-1, "REM has updated bootprocess w/ = %d", *eproc);
    hal_trace_puts(s_str);

}


static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__gotoprocess(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    eEprocess_t *eproc = nv->rem;
    eproc = eproc;

    snprintf(s_str, sizeof(s_str)-1, "REM has updated gotoprocess w/ = %d", *eproc);
    hal_trace_puts(s_str);}


static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__forcerestart(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    eObool_t *restart = nv->rem;
    restart = restart;
    snprintf(s_str, sizeof(s_str)-1, "REM has updated forcerestar w/ = %d", *restart);
    hal_trace_puts(s_str);    
}



static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__localise(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    eObool_t *localise = nv->rem;
    localise = localise;
    snprintf(s_str, sizeof(s_str)-1, "REM has updated localise w/ = %d", *localise);
    hal_trace_puts(s_str);
}

static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update_ipnetwork(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    eEipnetwork_t *ipnetwork = nv->rem;
    eOabstime_t t = time;
    uint32_t s = sign;
    uint8_t* adr = (uint8_t*)&ipnetwork->ipaddress;
    uint8_t* msk = (uint8_t*)&ipnetwork->ipnetmask;
    snprintf(s_str, sizeof(s_str)-1, "REM has updated ipnetwork w/ mac = 0x%llx, addr = %d.%d.%d.%d, msk = %d.%d.%d.%d", 
                                      ipnetwork->macaddress, 
                                      adr[0], adr[1], adr[2], adr[3],
                                      msk[0], msk[1], msk[2], msk[3]);
    hal_trace_puts(s_str);
}


static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update_ipnetwork__macaddress(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    uint64_t *macaddress = nv->rem;
    macaddress = macaddress;
    snprintf(s_str, sizeof(s_str)-1, "REM has updated macaddress w/ = 0x%llx", *macaddress);
    hal_trace_puts(s_str);
}

static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update_ipnetwork__ipaddress(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    uint32_t *ipaddress = nv->rem;
    uint8_t* adr = nv->rem;
    ipaddress = ipaddress;
    snprintf(s_str, sizeof(s_str)-1, "REM has updated ipaddress w/ 0x%x = %d.%d.%d.%d", *ipaddress, adr[0], adr[1], adr[2], adr[3]);
    hal_trace_puts(s_str);
}

static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update_ipnetwork__ipnetmask(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    uint32_t *ipnetmask = nv->rem;
    uint8_t* adr = nv->rem;
    snprintf(s_str, sizeof(s_str)-1, "REM has updated ipnetmask w/ 0x%x = %d.%d.%d.%d", *ipnetmask, adr[0], adr[1], adr[2], adr[3]);
    hal_trace_puts(s_str);
}


static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__remoteipaddress(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    uint32_t *remoteipaddress = nv->rem;
    uint8_t* adr = nv->rem;
    snprintf(s_str, sizeof(s_str)-1, "REM has updated remote ipaddress w/ 0x%x = %d.%d.%d.%d", *remoteipaddress, adr[0], adr[1], adr[2], adr[3]);
    hal_trace_puts(s_str);
    //#warning --> in here put code to propagate the remoteipaddress towards the sockets etc (or  put it inside deviceinfo ....)
}


static void s_eo_cfg_nvsEP_base_usr_rem_anydev_action_update__remoteipport(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    uint16_t *remoteipport = nv->rem;
    snprintf(s_str, sizeof(s_str)-1, "REM has updated remote ipport w/%d", *remoteipport);
    hal_trace_puts(s_str);

    //#warning --> in here put code to propagate the remoteipport towards the sockets etc (or  put it inside deviceinfo ....)
}


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



