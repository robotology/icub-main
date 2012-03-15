
/* @file       eOcfg_nvsEP_mngmnt_usr_rem_board.c
    @brief      This file keeps the user-defined configuration for the NVs in the base endpoint port for a remote board
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
#include "EOnv_hid.h"
#include "eOcfg_nvsEP_mngmnt_con.h"


#include "EOconstvector_hid.h"


#include "hal.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eOcfg_nvsEP_mngmnt_usr_rem_board.h"


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

static void s_eo_cfg_nvsEP_mngmnt_usr_rem_board_action_update__upto10rop2signal(const EOnv* nv, const eOabstime_t time, const uint32_t sign);
static void s_eo_cfg_nvsEP_mngmnt_usr_rem_board_action_update__workingmode(const EOnv* nv, const eOabstime_t time, const uint32_t sign);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_mngmnt_usr_rem_board_action_peripheralinterface__upto10rop2signal =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_mngmnt_usr_rem_board_action_update__upto10rop2signal
};

static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_mngmnt_usr_rem_board_action_peripheralinterface__workingmode =
{
    EO_INIT(.init)      NULL,
    EO_INIT(.update)    s_eo_cfg_nvsEP_mngmnt_usr_rem_board_action_update__workingmode
};


// --- service static variables --------

static char s_service_str[128];


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables
// --------------------------------------------------------------------------------------------------------------------

static EOnv_usr_t s_eo_cfg_nvsEP_mngmnt_usr_rem_board_array_of_EOnv_usr[] =
{
    {   // 00 __upto10rop2signal
        EO_INIT(.peripheralinterface)&s_eo_cfg_nvsEP_mngmnt_usr_rem_board_action_peripheralinterface__upto10rop2signal,
        EO_INIT(.on_rop_reception)      NULL,             // we dont do anything on rop reception before or after its processing
        EO_INIT(.stg_address)           EOK_uint32dummy   // we dont give any storage address. by the way: the variable is BEH!
    },
    {   // 01 __workingmode
        EO_INIT(.peripheralinterface)&s_eo_cfg_nvsEP_mngmnt_usr_rem_board_action_peripheralinterface__workingmode,
        EO_INIT(.on_rop_reception)      NULL,             // we dont do anything on rop reception before or after its processing
        EO_INIT(.stg_address)           EOK_uint32dummy   // we dont give any storage address. by the way: the variable is BEH!
    }    
};



const EOconstvector  s_eo_cfg_nvsEP_mngmnt_usr_rem_board_constvector_of_EOnv_usr = 
{
    EO_INIT(.size)              sizeof(s_eo_cfg_nvsEP_mngmnt_usr_rem_board_array_of_EOnv_usr)/sizeof(EOnv_usr_t), 
    EO_INIT(.item_size)         sizeof(EOnv_usr_t),
    EO_INIT(.item_array_data)   s_eo_cfg_nvsEP_mngmnt_usr_rem_board_array_of_EOnv_usr
};


extern const EOconstvector* const eo_cfg_nvsEP_mngmnt_usr_rem_board_constvector_of_EOnv_usr = &s_eo_cfg_nvsEP_mngmnt_usr_rem_board_constvector_of_EOnv_usr;


extern eo_cfg_nvsEP_mngmnt_t* eo_cfg_nvsEP_mngmnt_usr_rem_board_mem_local  = NULL;
extern eo_cfg_nvsEP_mngmnt_t* eo_cfg_nvsEP_mngmnt_usr_rem_board_mem_remote = NULL;

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern void eo_cfg_nvsEP_mngmnt_usr_rem_board_initialise(void* loc, void* rem)
{    
    eo_cfg_nvsEP_mngmnt_t *mngmnt_loc = (eo_cfg_nvsEP_mngmnt_t*)loc;
    eo_cfg_nvsEP_mngmnt_t *mngmnt_rem = (eo_cfg_nvsEP_mngmnt_t*)rem; 
    
    // 0. init the extern variables which can be directly used by the application 
    eo_cfg_nvsEP_mngmnt_usr_rem_board_mem_local  = mngmnt_loc;
    eo_cfg_nvsEP_mngmnt_usr_rem_board_mem_remote = mngmnt_rem;
    
    
    
    // in here we initailise the ram allocated by the EOnvscfg object: 
    // 1. at least put it at its default value. 
    // 2. if you need to initialise other peripherals or objects linked to the values do it.
    //    HOWEVER: initialisation of NVs is done on specific functions 
    
    
    // 1. assign default values
    memcpy(mngmnt_loc, &eo_cfg_nvsEP_mngmnt_default, sizeof(eo_cfg_nvsEP_mngmnt_t));
    memcpy(mngmnt_rem, &eo_cfg_nvsEP_mngmnt_default, sizeof(eo_cfg_nvsEP_mngmnt_t));

    // 2. init other peripherals ...
    // i dont do it    
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static void s_eo_cfg_nvsEP_mngmnt_usr_rem_board_action_update__workingmode(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    ep_mngmnt_workingmode_t* wmode = (ep_mngmnt_workingmode_t*)nv->rem;

    snprintf(s_service_str, sizeof(s_service_str)-1, "The host has just been updated by a remote board w/ a workingmode = %d", *wmode);
    hal_trace_puts(s_service_str);
}


static void s_eo_cfg_nvsEP_mngmnt_usr_rem_board_action_update__upto10rop2signal(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    EOarray* array = (EOarray*)nv->rem;
    
    snprintf(s_service_str, sizeof(s_service_str)-1, "The host has just been updated by a remote board w/ a upto15 array w/ %d epids", eo_array_Size(array));
    hal_trace_puts(s_service_str);
}

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



