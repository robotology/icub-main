
/* @file       eupdater.c
	@brief      This file implements a test for abslayer with a telnet server, an ftp server (hal + ipal + fsal), a 
                blinking led facility (hal) done w/ a sw timer (osal). wow.
	@author     marco.accame@iit.it
    @date       06/16/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdint.h"
#include "stdlib.h"

#include "hal.h"
#include "osal.h"
#include "ipal.h"
//#include "fsal.h"

#include "eEcommon.h"
#include "shalPART.h" 
#include "shalBASE.h" 
#include "shalINFO.h" 

#include "app-ipnet.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------

extern hal_cfg_t *hal_cfgMINE;
//extern fsal_params_cfg_t *fsal_params_cfgMINE;
extern osal_cfg_t *osal_cfgMINE;
 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------





// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------

typedef struct
{
    hal_gpio_port_t port;
    hal_gpio_pin_t  pin;
} mygpio_t;

// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_nomemory_anymore(void);

static void s_initialise_shal(void);
static void s_initialise_hal(void);
//static void s_initialise_fsal(void);
static void s_initialise_osal(void);

static void s_task_init(void);

static void s_toggle_led_cbk(osal_timer_t *tmr, void *arg);

static void s_error_mode(void);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------



static const eEmoduleInfo_t s_updater_info __attribute__((at(EENV_MEMMAP_EUPDATER_ROMADDR+EENV_MODULEINFO_OFFSET))) = 
{
    .info           =
    {
        .entity     =
        {
            .type       = ee_entity_process,
            .signature  = ee_procUpdater,
            .version    = 
            { 
                .major = 1, 
                .minor = 0
            },  
            .builddate  = 
            {
                .year  = 2011,
                .month = 11,
                .day   = 4,
                .hour  = 12,
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
        .name           = "eUpdater"
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



static const mygpio_t s_gpiotmr = {hal_gpio_portE, hal_gpio_pin15};



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

int main(void)
{

    s_initialise_hal();

    s_initialise_shal();

    hal_trace_init();
    
    s_initialise_osal();
    
    for(;;);
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------





// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

// give hal memory, initialise sys, leds of mcbstm32c all off, display 
static void s_initialise_hal(void)
{
    uint32_t ram04size = 0;
    uint32_t *ram04data = NULL;
    
    hal_base_memory_getsize(hal_cfgMINE, &ram04size);
    
    if(0 != ram04size)
    {
        ram04data = (uint32_t*)calloc(ram04size/4, sizeof(uint32_t));
        
        if(NULL == ram04data)
        {
            s_nomemory_anymore();
        }
    }

    hal_base_initialise(hal_cfgMINE, ram04data);

   
    // system
    hal_sys_systeminit();

    hal_sys_vectortable_relocate(EENV_MEMMAP_EUPDATER_ROMADDR-EENV_ROMSTART);

    // gpio out: leds
    hal_gpio_init(hal_gpio_portE, hal_gpio_pin15, hal_gpio_dirOUT, hal_gpio_speed_low);
    // gpio in: button user
    hal_gpio_init(hal_gpio_portB, hal_gpio_pin7, hal_gpio_dirINP, hal_gpio_speed_low);

    hal_trace_init();

    hal_trace_puts("trace: hal initted");   
}


static void s_initialise_shal(void)
{

    if(ee_res_OK == shalbase_isvalid())
    {
        shalbase_init(1);
    }
    else
    {
        s_error_mode();
    }

    // put signature info partition table
    if(ee_res_OK == shalpart_isvalid())
    {
        shalpart_init();
        shalpart_proc_synchronise(ee_procUpdater, &s_updater_info);
    }
    else
    {
        // consider to enter in an error mode.
        s_error_mode();
    }
    
    if(ee_res_OK == shalinfo_isvalid())
    {
        shalinfo_init();
    }
    else
    {
        // consider to enter in an error mode.
        s_error_mode();
    }      
   
}

// give osal memory, create one task 
static void s_initialise_osal(void)
{
    uint32_t ram08size = 0;
    uint64_t *ram08data = NULL;
    
    osal_base_memory_getsize(osal_cfgMINE, &ram08size);
    
    if(0 != ram08size)
    {
        ram08data = (uint64_t*)calloc(ram08size/8, sizeof(uint64_t));
        
        if(NULL == ram08data)
        {
            s_nomemory_anymore();
        }
    }

    osal_base_initialise(osal_cfgMINE, ram08data);
    
    osal_system_start(s_task_init);
    
}


static void s_task_init(void)
{
    const osal_time_t tmr_period = 500*1000; // 500 milli
    osal_timer_t *tmr;
    osal_timer_timing_t timing = { .startat = osal_abstimeNONE, .count = tmr_period, .mode = osal_tmrmodeFOREVER };
    osal_timer_onexpiry_t onexpiry = { .cbk = s_toggle_led_cbk, .par = (void*)&s_gpiotmr};

    // create a timer with a callback function which toggles a led.
    tmr = osal_timer_new();

    osal_timer_start(tmr, &timing, &onexpiry, osal_callerTSK);

    appinet_start_ipal();
}




static void s_toggle_led_cbk(osal_timer_t *tmr, void *arg)
{
    const mygpio_t *gp = (const mygpio_t *)arg;
    static hal_gpio_val_t val = hal_gpio_valLOW;

    val = (hal_gpio_valHIGH == val) ? (hal_gpio_valLOW) : (hal_gpio_valHIGH);

    hal_gpio_setval(gp->port, gp->pin, val);
}


static void s_nomemory_anymore(void)
{
    static uint8_t nomem = 0;

    hal_trace_puts("trace: no heap anymore");

    for(;;)
    {
        nomem ++;
        nomem = nomem;
    }
}

static void s_error_mode(void)
{
    volatile uint32_t n = 0;
    volatile uint32_t a = 5;

    //s_initialise_hal();

    hal_trace_puts("trace: entered error mode");

    for(;;)
    {
        for(n=1; n< 0x00100000; n++) { a = n / (n-1); }
        s_toggle_led_cbk(NULL, (void*)&s_gpiotmr);
    }
}




// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



