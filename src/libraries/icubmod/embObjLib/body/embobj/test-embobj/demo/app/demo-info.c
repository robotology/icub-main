
/* @file       demo-info.c
	@brief      This file keeps the module info of the updatre
	@author     marco.accame@iit.it
    @date       01/11/2012
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "eEcommon.h"
#include "eEmemorymap.h"

#include "hal.h"
#include "osal.h"
#include "ipal.h"

#include "EOMtheSystem.h"


extern const hal_cfg_t     s_hal_cfg;
extern const osal_cfg_t    s_osal_cfg;
extern const ipal_cfg_t    s_ipal_cfg;


// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "demo-info.h"

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

extern const eEboardInfo_t demoinfo_boardinfo =                        
{
    .info           =
    {
        .entity     =
        {
            .type       = ee_entity_board,
            .signature  = 0x11,
            .version    = 
            { 
                .major = 1, 
                .minor = 0
            },  
            .builddate  = 
            {
                .year  = 2010,
                .month = 1,
                .day   = 1,
                .hour  = 0,
                .min   = 0
            }
        },
        .rom        = 
        {   
            .addr   = EENV_ROMSTART,
            .size   = EENV_ROMSIZE
        },
        .ram        = 
        {   
            .addr   = EENV_RAMSTART,
            .size   = EENV_RAMSIZE
        },
        .storage    = 
        {
            .type   = ee_strg_eeprom,
            .size   = EENV_STGSIZE,
            .addr   = EENV_STGSTART
        },
        .communication  = ee_commtype_eth | ee_commtype_can1 | ee_commtype_can2,
        .name           = "mcbstm32c"
    },
    .uniqueid       = 0,
    .extra          = {0}
};

extern const eEmoduleInfo_t demoinfo_modinfo = 
{
    .info           =
    {
        .entity     =
        {
            .type       = ee_entity_process,
            .signature  = ee_procApplication,
            .version    = 
            { 
                .major = 1, 
                .minor = 0
            },  
            .builddate  = 
            {
                .year  = 2012,
                .month = 1,
                .day   = 11,
                .hour  = 18,
                .min   = 0
            }
        },
        .rom        = 
        {   
            .addr   = EENV_MEMMAP_EUPDATER_ROMADDR,
            .size   = EENV_MEMMAP_EUPDATER_ROMSIZE
        },
        .ram        = 
        {   
            .addr   = EENV_MEMMAP_EUPDATER_RAMADDR,
            .size   = EENV_MEMMAP_EUPDATER_RAMSIZE
        },
        .storage    = 
        {
            .type   = ee_strg_none,
            .size   = 0,
            .addr   = 0
        },
        .communication  = ee_commtype_eth,  // later on we may also add can1 and can2
        .name           = "EOMeApplication"
    },
    .protocols  =
    {
        .udpprotversion  = { .major = 0, .minor = 1},
        .can1protversion = { .major = 0, .minor = 0},
        .can2protversion = { .major = 0, .minor = 0},
        .gtwprotversion  = { .major = 0, .minor = 0}
    },
    .extra      = {0}
};

extern const eOmsystem_cfg_t demoinfo_syscfg =
{
    .codespaceoffset    = (EENV_ROMSTART-EENV_ROMSTART),
    .halcfg             = &s_hal_cfg,
    .osalcfg            = &s_osal_cfg,
    .fsalcfg            = NULL
};

extern const ipal_cfg_t* demoinfo_ipal_cfg = NULL;//&s_ipal_cfg;


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
// empty-section

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



