
/* @file       eOcfg_nvsEP_mngmnt_usr_loc_board.c
    @brief      This file keeps the user-defined local configuration for the NVs in the management port of any board
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
#include "eOcfg_nvsEP_mngmnt_con.h"

#include "EOtheBOARDtransceiver.h"

#include "EOnv_hid.h"


#include "EOconstvector_hid.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eOcfg_nvsEP_mngmnt_usr_loc_board.h"


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

static void s_eo_cfg_nvsEP_mngmnt_usr_loc_board_action_init__upto10rop2signal(const EOnv* nv);
static void s_eo_cfg_nvsEP_mngmnt_usr_loc_board_action_init__workingmode(const EOnv* nv);

static void s_eo_cfg_nvsEP_mngmnt_usr_loc_board_action_update__upto10rop2signal(const EOnv* nv, const eOabstime_t time, const uint32_t sign);
static void s_eo_cfg_nvsEP_mngmnt_usr_loc_board_action_update__workingmode(const EOnv* nv, const eOabstime_t time, const uint32_t sign);

// --- generic static functions ...
static void s_eo_cfg_nvsEP_mngmnt_usr_loc_generic_upto10rop2signal(EOarray* array);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_mngmnt_usr_loc_board_action_peripheralinterface__upto10rop2signal =
{
    EO_INIT(.init)      s_eo_cfg_nvsEP_mngmnt_usr_loc_board_action_init__upto10rop2signal,
    EO_INIT(.update)    s_eo_cfg_nvsEP_mngmnt_usr_loc_board_action_update__upto10rop2signal
};


static const eOnv_fn_peripheral_t s_eo_cfg_nvsEP_mngmnt_usr_loc_board_action_peripheralinterface__workingmode =
{
    EO_INIT(.init)      s_eo_cfg_nvsEP_mngmnt_usr_loc_board_action_init__workingmode,
    EO_INIT(.update)    s_eo_cfg_nvsEP_mngmnt_usr_loc_board_action_update__workingmode
};

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables
// --------------------------------------------------------------------------------------------------------------------

static const EOnv_usr_t s_eo_cfg_nvsEP_mngmnt_usr_loc_board_array_of_EOnv_usr[] =
{
    {   // 00 __upto10rop2signal
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_mngmnt_usr_loc_board_action_peripheralinterface__upto10rop2signal,
        EO_INIT(.on_rop_reception)      NULL,                 // we dont do anything on rop reception before or after its processing
        EO_INIT(.stg_address)           EOK_uint32dummy       // we dont give any storage address. by the way: the variable is BEH!
    },
    {   // 01 __workingmode
        EO_INIT(.peripheralinterface)   &s_eo_cfg_nvsEP_mngmnt_usr_loc_board_action_peripheralinterface__workingmode,
        EO_INIT(.on_rop_reception)      NULL,                 // we dont do anything on rop reception before or after its processing
        EO_INIT(.stg_address)           EOK_uint32dummy       // we dont give any storage address. by the way: the variable is BEH!
    }    
};


const EOconstvector  s_eo_cfg_nvsEP_mngmnt_usr_loc_board_constvector_of_EOnv_usr = 
{
    EO_INIT(.size)              sizeof(s_eo_cfg_nvsEP_mngmnt_usr_loc_board_array_of_EOnv_usr)/sizeof(EOnv_usr_t), 
    EO_INIT(.item_size)         sizeof(EOnv_usr_t),
    EO_INIT(.item_array_data)   s_eo_cfg_nvsEP_mngmnt_usr_loc_board_array_of_EOnv_usr
};


extern const EOconstvector* const eo_cfg_nvsEP_mngmnt_usr_loc_board_constvector_of_EOnv_usr = &s_eo_cfg_nvsEP_mngmnt_usr_loc_board_constvector_of_EOnv_usr;

extern eo_cfg_nvsEP_mngmnt_t* eo_cfg_nvsEP_mngmnt_usr_loc_board_mem_local  = NULL;


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern void eo_cfg_nvsEP_mngmnt_usr_loc_board_initialise(void* loc, void* rem)
{ 
    eo_cfg_nvsEP_mngmnt_t *mngmnt_loc = (eo_cfg_nvsEP_mngmnt_t*)loc;
    eo_cfg_nvsEP_mngmnt_t *mngmnt_rem = (eo_cfg_nvsEP_mngmnt_t*)rem;    // it is NULL if we are in a local ownership
    
    mngmnt_rem = mngmnt_rem;

    // 0. init the extern variable whcih can be directly used by the application 
    eo_cfg_nvsEP_mngmnt_usr_loc_board_mem_local = mngmnt_loc;
    
    
    
    // in here we initailise the ram allocated by the EOnvscfg object: 
    // 1. at least put it at its default value. 
    // 2. if you need to initialise other peripherals or objects linked to the values do it.
    //    HOWEVER: initialisation of NVs is done on specific functions 
    
    
    // 1. assign default values
    memcpy(mngmnt_loc, &eo_cfg_nvsEP_mngmnt_default, sizeof(eo_cfg_nvsEP_mngmnt_t));

    // 2. init other peripherals ...
    // i dont do it
    

}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static void s_eo_cfg_nvsEP_mngmnt_usr_loc_board_action_init__upto10rop2signal(const EOnv* nv)
{
    EOarray* array = (EOarray*)nv->loc;

    s_eo_cfg_nvsEP_mngmnt_usr_loc_generic_upto10rop2signal(array);
}

static void s_eo_cfg_nvsEP_mngmnt_usr_loc_board_action_init__workingmode(const EOnv* nv)
{
    nv = nv;
    
    #warning --> add code to signal to the relevant object which manages the working modes that we are in xxx mode (idle)
}

static void s_eo_cfg_nvsEP_mngmnt_usr_loc_board_action_update__upto10rop2signal(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    EOarray* array = (EOarray*)nv->loc;

    s_eo_cfg_nvsEP_mngmnt_usr_loc_generic_upto10rop2signal(array);
}


static void s_eo_cfg_nvsEP_mngmnt_usr_loc_board_action_update__workingmode(const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    ep_mngmnt_workingmode_t *workingmode = nv->loc;

    workingmode = workingmode;
    
    #warning --> add code to signal to the relevant object which manages the working modes that the working mode has changed 
}



// ----- generic static functions used by _init_ and _update_ 

static void s_eo_cfg_nvsEP_mngmnt_usr_loc_generic_upto10rop2signal(EOarray* array)
{
    uint8_t size, i;
    eOropSIGcfg_t *sigcfg;
    eo_transceiver_ropinfo_t ropinfo;
    EOtransceiver* theems00transceiver; 

    eOresult_t res;
    
    if(NULL == (theems00transceiver = eo_boardtransceiver_GetHandle()))
    {
        return;
    }
    
    if((eo_array_ItemSize(array) != sizeof(eOropSIGcfg_t)) || (10 != eo_array_Capacity(array)) || ((size = eo_array_Size(array)) > 10))
    {
        // error ...
        return;
    }

    eo_transceiver_rop_regular_Clear(theems00transceiver);

    for(i=0; i<size; i++)
    {
        sigcfg = (eOropSIGcfg_t*)eo_array_At(array, i);
        #warning --> so far the regular rops are sent with a eok_ropconfig_basic, that is to say: without time64bit
        ropinfo.ropcfg      = eok_ropconfig_basic;
        ropinfo.ropcfg.plustime = sigcfg->plustime;
        ropinfo.ropcode     = eo_ropcode_sig;
        ropinfo.nvep        = sigcfg->ep;    
        ropinfo.nvid        = sigcfg->id;
        res = eo_transceiver_rop_regular_Load(theems00transceiver, &ropinfo);
        res = res;
    }

}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



