
/** @file       EOtheNVsCfgExample.c
    @brief      This file implements internal implementation to the configuration of the GPIOs for the MCBSTM32 board
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



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgExample.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVsCfgExample_hid.h" 


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

static void fn_before_set_fa(uint16_t id);
static void fn_after_set_fa(uint16_t id); 
static void fn_before_upd_fa(uint16_t id);
static void fn_after_upd_fa(uint16_t id);
static void fn_before_ask_fa(uint16_t id);
static void fn_after_ask_fa(uint16_t id);
static void fn_before_sig_fa(uint16_t id);
static void fn_after_sig_fa(uint16_t id);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


static const uint32_t s_var_a_def = 0xaabbccd1;
uint32_t s_var_a_vol = 0xaabbccd1;

   uint16_t                size;           // size in bytes of the data
    const void              *valuedef;      // default value of the data (in rom)
    void                    *valuevol;      // volatile value of the data (in ram)
    uint32_t                 addrperm;      // id or addr of permanent storage of value for requested cases (in eeprom).
    eOvoid_fp_uint16_t      fn_update;

static const eOnetvar_data_t nv_var_a =
{
    sizeof(uint32_t),                  
    &s_var_a_def,
    &s_var_a_vol,
    NULL,
    0
};

static const uint32_t s_var_b_def = 0xaabbccd2;
uint32_t s_var_b_vol = 0xaabbccd2;

static const eOnetvar_data_t nv_var_b =
{
    sizeof(uint32_t),                  
    &s_var_b_def,
    &s_var_b_vol,
    NULL,
    0
};

static const uint32_t s_var_c_def = 0xaabbccd3;
uint32_t s_var_c_vol = 0xaabbccd3;

static const eOnetvar_data_t nv_var_c =
{
    sizeof(uint32_t),                  
    &s_var_c_def,
    &s_var_c_vol,
    NULL,
    0
};


static const uint32_t s_var_d_def = 0xaabbccd4;
uint32_t s_var_d_vol = 0xaabbccd4;

static const eOnetvar_data_t nv_var_d =
{
    sizeof(uint32_t),                   
    &s_var_d_def,
    &s_var_d_vol,
    NULL,
    0
};


static const uint32_t s_var_e_def = 0xaabbccd5;
uint32_t s_var_e_vol = 0xaabbccd5;

static const eOnetvar_data_t nv_var_e =
{
    sizeof(uint32_t),                 
    &s_var_e_def,
    &s_var_e_vol,
    NULL,
    0
};

static const uint8_t s_var_f_def[] = {0, 1, 2, 3, 4};
uint8_t s_var_f_vol[] = {0, 1, 2, 3, 4};

static const eOnetvar_data_t nv_var_f =
{
    sizeof(s_var_f_def),                 
    &s_var_f_def,
    &s_var_f_vol,
    NULL,
    0
};



#if 0
static const EOnetvar s_thenetvars[] =
{
    // f9
    {   // 0
        eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_pkd, 0xf9),
        2,
        {1, 4, 255,255,255,255,255,255},
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        NULL
    },
    // f8
    {   // 1
        eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_pkd, 0xf8),
        2,
        {2, 3, 255,255,255,255,255,255},
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        NULL
    },
    // f1
    {   // 2
        eo_nv_getID(eo_nv_FUN_out, eo_nv_TYP_u32, 0xf1),
        0,
        {255,255,255,255,255,255,255,255},
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        &nv_var_a
    },
    // f2
    {   // 3
        eo_nv_getID(eo_nv_FUN_out, eo_nv_TYP_u32, 0xf2),
        0,
        {255,255,255,255,255,255,255,255},
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        &nv_var_b
    },
    // f7
    {   // 4
        eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_pkd, 0xf7),
        3,
        {5, 6, 9,255,255,255,255,255},
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        NULL
    },
    // f3
    {   // 5
        eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u32, 0xf3),
        0,
        {255,255,255,255,255,255,255,255},
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        &nv_var_c
    },    
    // f6
    {   // 6
        eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_pkd, 0xf6),
        2,
        {7, 8, 255,255,255,255,255,255},
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        NULL
    },
    // f4
    {   // 7
        eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 0xf4),
        0,
        {255,255,255,255,255,255,255,255},
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        &nv_var_d
    },    
    // f5
    {   // 8
        eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 0xf5),
        0,
        {255,255,255,255,255,255,255,255},
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        &nv_var_e
    },
    // fa
    {   // 9
        eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u32, 0xfa),
        0,
        {255,255,255,255,255,255,255,255},
        fn_before_set_fa, fn_after_set_fa, 
        fn_before_upd_fa, fn_after_upd_fa, 
        fn_before_ask_fa, fn_after_ask_fa, 
        fn_before_sig_fa, fn_after_sig_fa,
        &nv_var_f
    }      
}; 

#else

static const EOnetvar s_thenetvars[] =
{
    // f9
    {   // 0
        eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_pkd, 0xf9),
        3,
        {1, 4, 10,255,255,255,255,255},
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        NULL
    },
    // f8
    {   // 1
        eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_pkd, 0xf8),
        2,
        {2, 3, 255,255,255,255,255,255},
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        NULL
    },
    // f1
    {   // 2
        eo_nv_getID(eo_nv_FUN_out, eo_nv_TYP_u32, 0xf1),
        0,
        {255,255,255,255,255,255,255,255},
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        {NULL, NULL, NULL, NULL},  {NULL, NULL, NULL, NULL},
        &nv_var_a
    },
    // f2
    {   // 3
        eo_nv_getID(eo_nv_FUN_out, eo_nv_TYP_u32, 0xf2),
        0,
        {255,255,255,255,255,255,255,255},
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        {NULL, NULL, NULL, NULL},  {NULL, NULL, NULL, NULL},
        &nv_var_b
    },
    // f7
    {   // 4
        eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_pkd, 0xf7),
        3,
        {5, 6, 9,255,255,255,255,255},
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        {NULL, NULL, NULL, NULL},  {NULL, NULL, NULL, NULL},
        NULL
    },
    // f3
    {   // 5
        eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_u32, 0xf3),
        0,
        {255,255,255,255,255,255,255,255},
        fn_before_set_fa, fn_after_set_fa, 
        fn_before_upd_fa, fn_after_upd_fa, 
        fn_before_ask_fa, fn_after_ask_fa, 
        fn_before_sig_fa, fn_after_sig_fa,
        {NULL, NULL, NULL, NULL},  {NULL, NULL, NULL, NULL},
        &nv_var_c
    },    
    // f6
    {   // 6
        eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_pkd, 0xf6),
        2,
        {7, 8, 255,255,255,255,255,255},
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        {NULL, NULL, NULL, NULL},  {NULL, NULL, NULL, NULL},
        NULL
    },
    // f4
    {   // 7
        eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 0xf4),
        0,
        {255,255,255,255,255,255,255,255},
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        {NULL, NULL, NULL, NULL},  {NULL, NULL, NULL, NULL},
        &nv_var_d
    },    
    // f5
    {   // 8
        eo_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 0xf5),
        0,
        {255,255,255,255,255,255,255,255},
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        {NULL, NULL, NULL, NULL},  {NULL, NULL, NULL, NULL},
        &nv_var_e
    },
    // fa
    {   // 9
        eo_nv_getID(eo_nv_FUN_inp, eo_nv_TYP_pkd, 0xfa),
        0,
        {255,255,255,255,255,255,255,255},
        fn_before_set_fa, fn_after_set_fa, 
        fn_before_upd_fa, fn_after_upd_fa, 
        fn_before_ask_fa, fn_after_ask_fa, 
        fn_before_sig_fa, fn_after_sig_fa,
        {NULL, NULL, NULL, NULL},  {NULL, NULL, NULL, NULL},
        &nv_var_f
    },
    // a0
    {   // 10
        eo_nv_getID(eo_nv_FUN_out, eo_nv_TYP_pkd, 0xa0),
        2,
        {5,9,255,255,255,255,255,255},
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        {NULL, NULL, NULL, NULL},  {NULL, NULL, NULL, NULL},
        NULL
    }
                    
};               

#endif

/** @var        s_theconfiguration
    @brief      It is the required configuration, which is the collection of previously defined 
                const variables.
    @details    Use the name s_theconfiguration.
 **/
static const EOVtheNVsCfg s_theconfiguration =    
{ 
    // number of netvars
    sizeof(s_thenetvars)/sizeof(EOnetvar),
    // netvars
    s_thenetvars
}; 

  ggg
static const EOtheNVsCfgExample xxx =
{
    EOIDNVSCONFIG,          // eoidentifier
    &s_theconfiguration     // theconfig
};

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------



extern const EOtheNVsCfgExample * eo_nvscfg_example_GetHandle(void)
{
    return(&xxx);
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void fn_before_set_fa(uint16_t id)
{
    printf("before set of id 0x%x\n", id);
}

static void fn_after_set_fa(uint16_t id)
{
    printf("after set of id 0x%x\n", id);
}
 
static void fn_before_upd_fa(uint16_t id)
{
    printf("before upd of id 0x%x\n", id);
}

static void fn_after_upd_fa(uint16_t id)
{
    printf("after upd of id 0x%x\n", id);
}

static void fn_before_ask_fa(uint16_t id)
{
    printf("before ask of id 0x%x\n", id);
}

static void fn_after_ask_fa(uint16_t id)
{
    printf("after ask of id 0x%x\n", id);
}

static void fn_before_sig_fa(uint16_t id)
{
    printf("before sig of id 0x%x\n", id);
}

static void fn_after_sig_fa(uint16_t id)
{
    printf("after sig of id 0x%x\n", id);
}

 

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



