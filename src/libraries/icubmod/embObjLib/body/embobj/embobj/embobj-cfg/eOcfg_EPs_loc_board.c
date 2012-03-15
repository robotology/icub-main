
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
#include "eOcfg_nvsEP_base_usr_loc_anydev.h"

#include "eOcfg_nvsEP_mngmnt_con.h"
#include "eOcfg_nvsEP_mngmnt_usr_loc_board.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eOcfg_EPs_loc_board.h"


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

static uint16_t s_hash(uint16_t ep);

static uint16_t s_eo_cfg_nvsEP_loc_board_hashfunction_ep2index(uint16_t ep);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

extern uint16_t eo_cfg_nvsEP_base_hashfunction_id2index(uint16_t nvid);
extern const EOconstvector  s_eo_cfg_nvsEP_base_constvector_of_treenodes_EOnv_con;
extern const EOconstvector  s_eo_cfg_nvsEP_base_usr_loc_anydev_constvector_of_EOnv_usr;

extern uint16_t eo_cfg_nvsEP_mngmnt_hashfunction_id2index(uint16_t nvid);
extern const EOconstvector  s_eo_cfg_nvsEP_mngmnt_constvector_of_treenodes_EOnv_con;
extern const EOconstvector  s_eo_cfg_nvsEP_mngmnt_usr_loc_board_constvector_of_EOnv_usr;



static const eOnvscfg_EP_t s_eo_cfg_EPs_vectorof_loc_board_data[] =
{              
    {   // 00-base
        EO_INIT(.endpoint)                          EOK_cfg_nvsEP_base_endpoint,
        EO_INIT(.sizeof_endpoint_data)              sizeof(eo_cfg_nvsEP_base_t),
        EO_INIT(.hashfunction_id2index)             eo_cfg_nvsEP_base_hashfunction_id2index,
        EO_INIT(.constvector_of_treenodes_EOnv_con) &s_eo_cfg_nvsEP_base_constvector_of_treenodes_EOnv_con, //eo_cfg_nvsEP_base_constvector_of_treenodes_EOnv_con,
        EO_INIT(.constvector_of_EOnv_usr)           &s_eo_cfg_nvsEP_base_usr_loc_anydev_constvector_of_EOnv_usr, //eo_cfg_nvsEP_base_usr_loc_anydev_constvector_of_EOnv_usr,
        EO_INIT(.endpoint_data_init)                eo_cfg_nvsEP_base_usr_loc_anydev_initialise
    },
    {   // 01-mngmnt
        EO_INIT(.endpoint)                          EOK_cfg_nvsEP_mngmnt_endpoint,
        EO_INIT(.sizeof_endpoint_data)              sizeof(eo_cfg_nvsEP_mngmnt_t),
        EO_INIT(.hashfunction_id2index)             eo_cfg_nvsEP_mngmnt_hashfunction_id2index,
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

extern const eOuint16_fp_uint16_t eo_cfg_nvsEP_loc_board_fptr_hashfunction_ep2index = s_eo_cfg_nvsEP_loc_board_hashfunction_ep2index;



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static uint16_t s_hash(uint16_t ep)
{
    return(ep & 0xFF);
}

static uint16_t s_eo_cfg_nvsEP_loc_board_hashfunction_ep2index(uint16_t ep)
{
    #define EPTABLESIZE     16

    // in order to always have a hit the table s_eptable[] it must be of size equal to max{ s_hash(ep) }, thus if we
    // use an ep of value 16 and s_hash() just keeps the lsb, then the size must be 17 
    // if there are holes, they shall have EOK_uint16dummy in other entries. for example, if we have eps = {0, 7, 16}
    // then the table shall be of size 17, shall contain 0xffff everywhere but in positions 0, 7, 16 where the values
    // are ... 0, 7, 16

    static const uint16_t s_eptable[EPTABLESIZE] = 
    {   
        EOK_cfg_nvsEP_base_endpoint,                            EOK_cfg_nvsEP_mngmnt_endpoint,   
        EOK_uint16dummy,                                        EOK_uint16dummy,
        EOK_uint16dummy,                                        EOK_uint16dummy,
        EOK_uint16dummy,                                        EOK_uint16dummy,
        EOK_uint16dummy,                                        EOK_uint16dummy,
        EOK_uint16dummy,                                        EOK_uint16dummy,
        EOK_uint16dummy,                                        EOK_uint16dummy,
        EOK_uint16dummy,                                        EOK_uint16dummy                         
    };

    uint16_t index = s_hash(ep);
    
    if((index < EPTABLESIZE) && (ep == s_eptable[index]) )
    {
        return(index);
    }
    else
    {
        return(EOK_uint16dummy);
    }
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



