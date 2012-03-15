
/* @file       eOcfg_EPs_loc_board.c
    @brief      This file keeps ...
    @author     marco.accame@iit.it
    @date       09/06/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h" 
#include "string.h"
#include "stdio.h"


#include "EOconstvector_hid.h"

#include "eOcfg_nvsEP_base_con.h"
#include "eOcfg_nvsEP_base_FakeBoard01.h"

#include "eOcfg_nvsEP_mngmnt_con.h"
#include "eOcfg_nvsEP_mngmnt_FakeBoard01.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eOcfg_EPs_FakeBoard01.h"

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

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

extern const EOconstvector  s_eo_cfg_nvsEP_base_constvector_of_treenodes_EOnv_con;
extern const EOconstvector  s_eo_cfg_nvsEP_base_usr_loc_anydev_constvector_of_EOnv_usr;

extern const EOconstvector  s_eo_cfg_nvsEP_mngmnt_constvector_of_treenodes_EOnv_con;
extern const EOconstvector  s_eo_cfg_nvsEP_mngmnt_usr_loc_board_constvector_of_EOnv_usr;

static const eOnvscfg_EP_t s_eo_cfg_EPs_vectorof_loc_board_data[] =
{              
    {   // 00-base
        EO_INIT(.endpoint)                          EOK_cfg_nvsEP_base_endpoint,
        EO_INIT(.sizeof_endpoint_data)              sizeof(eo_cfg_nvsEP_base_t),
        EO_INIT(.constvector_of_treenodes_EOnv_con) &s_eo_cfg_nvsEP_base_constvector_of_treenodes_EOnv_con, //eo_cfg_nvsEP_base_constvector_of_treenodes_EOnv_con,
        EO_INIT(.constvector_of_EOnv_usr)           &s_eo_cfg_nvsEP_base_usr_loc_anydev_constvector_of_EOnv_usr, //eo_cfg_nvsEP_base_usr_loc_anydev_constvector_of_EOnv_usr,
        EO_INIT(.endpoint_data_init)                eo_cfg_nvsEP_base_usr_loc_anydev_initialise
    },
    {   // 01-mngmnt
        EO_INIT(.endpoint)                          EOK_cfg_nvsEP_mngmnt_endpoint,
        EO_INIT(.sizeof_endpoint_data)              sizeof(eo_cfg_nvsEP_mngmnt_t),
        EO_INIT(.constvector_of_treenodes_EOnv_con) &s_eo_cfg_nvsEP_mngmnt_constvector_of_treenodes_EOnv_con, //eo_cfg_nvsEP_mngmnt_constvector_of_treenodes_EOnv_con,
        EO_INIT(.constvector_of_EOnv_usr)           &s_eo_cfg_nvsEP_mngmnt_usr_loc_board_constvector_of_EOnv_usr, //eo_cfg_nvsEP_mngmnt_usr_loc_board_constvector_of_EOnv_usr,
        EO_INIT(.endpoint_data_init)                eo_cfg_nvsEP_mngmnt_usr_loc_board_initialise
    }    
};

static const EOconstvector s_eo_cfg_EPs_vectorof_loc_board = 
{
    EO_INIT(.size)                  sizeof(s_eo_cfg_EPs_vectorof_loc_board_data)/sizeof(const eOnvscfg_EP_t),
    EO_INIT(.item_size)             sizeof(eOnvscfg_EP_t),
    EO_INIT(.item_array_data)       s_eo_cfg_EPs_vectorof_loc_board_data
};



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables
// --------------------------------------------------------------------------------------------------------------------


extern const EOconstvector* const eo_cfg_EPs_vectorof_loc_board = &s_eo_cfg_EPs_vectorof_loc_board;



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



