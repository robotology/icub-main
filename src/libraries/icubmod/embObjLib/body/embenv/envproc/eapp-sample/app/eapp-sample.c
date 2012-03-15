
/* @file       eapp-sample.c
    @brief      This  file implements a sample e-application process for a stm32f107 onto the mcbstm32c keil board.
    @author     marco.accame@iit.it
    @date       04/05/2011
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "hal.h"
#include "osal.h"



#include "string.h"
#include "stdlib.h"
#include "stdio.h"

#include "eEcommon.h"
#include "shalPART.h"
#include "shalBASE.h"
#include "shalINFO.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------

typedef struct
{
    hal_gpio_port_t port;
    hal_gpio_pin_t  pin;
} mygpio_t;

// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_initialise_hal(void);

static void s_initialise_shal(void);

static void s_initialise_osal(void);

static void s_nomemory_anymore(void);

static void s_error_mode(void);

static void s_task_init(void);

static void s_task_work(void *arg);

static void s_toggle_led_cbk(osal_timer_t *tmr, void *arg);
static void s_check_inp_cbk(osal_timer_t *tmr, void *arg);

static void s_attempt_goto_eupdater(void);

//static void s_loader_Delay(uint32_t dlyTicks);
//static void s_loader_manage_error(uint32_t onmilli, uint32_t offmilli);
//static void s_loader_exec_loader(void);
//static void s_loader_eval_user_request(void);
//static void s_loader_eval_jump_request(void);
//static void s_loader_eval_def_from_partition_table(void);
//static void s_loader_eval_default(void);
//static void s_loader_attempt_jump(eEprocess_t proc, uint32_t adr);
//
//static void s_loader_HW_init(void);
//static void s_loader_HW_SYSTICK_Config(void);
//static void s_loader_HW_INP_Config(void);
//static uint32_t s_loader_HW_INP_IsPushed(void);
//static void s_loader_HW_LED_Config(void);
//static void s_loader_HW_LED_On(uint32_t led);
//static void s_loader_HW_LED_Off(uint32_t led);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const eEmoduleInfo_t s_eapp_info __attribute__((at(EENV_MEMMAP_EAPPLICATION_ROMADDR+EENV_MODULEINFO_OFFSET))) = 
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
                .year  = 2011,
                .month = 11,
                .day   = 3,
                .hour  = 18,
                .min   = 0
            }
        },
        .rom        = 
        {   
            .addr   = EENV_MEMMAP_EAPPLICATION_ROMADDR,
            .size   = EENV_MEMMAP_EAPPLICATION_ROMSIZE
        },
        .ram        = 
        {   
            .addr   = EENV_MEMMAP_EAPPLICATION_RAMADDR,
            .size   = EENV_MEMMAP_EAPPLICATION_RAMSIZE
        },
        .storage    = 
        {
            .type   = ee_strg_eeprom,
            .size   = EENV_MEMMAP_EAPPLICATION_STGSIZE,
            .addr   = EENV_MEMMAP_EAPPLICATION_STGADDR
        },
        .communication  = ee_commtype_none,
        .name           = "eApplication"
    },
    .protocols  =
    {
        .udpprotversion  = { .major = 0, .minor = 0},
        .can1protversion = { .major = 0, .minor = 0},
        .can2protversion = { .major = 0, .minor = 0},
        .gtwprotversion  = { .major = 0, .minor = 0}
    },
    .extra      = {0}
};


static const mygpio_t s_gpioled = {hal_gpio_portE, hal_gpio_pin14}; 
static const mygpio_t s_gpioinp = {hal_gpio_portB, hal_gpio_pin7}; 

const osal_time_t tmr_period_led = 500*1000; // 500 milli
const osal_time_t tmr_period_led_wild = 100*1000; // 200 milli
const osal_time_t tmr_period_inp = 100*1000; // 100 milli

static osal_task_t *s_tskid_work = NULL;

static osal_timer_t * s_tmr_led = NULL;


                        

// --------------------------------------------------------------------------------------------------------------------
// - definition of main 
// --------------------------------------------------------------------------------------------------------------------

int main(void) 
{

    s_initialise_hal();

    s_initialise_shal();

    s_initialise_osal();


    // just for ...
    for(;;);

}




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------



static void s_initialise_hal(void)
{
    uint32_t ram04size = 0;
    uint32_t *ram04data = NULL;
    extern hal_cfg_t *hal_cfgMINE;

    
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

    // relocate vectors
    hal_sys_vectortable_relocate(EENV_MEMMAP_EAPPLICATION_ROMADDR-EENV_ROMSTART);

    // gpio out: leds
    hal_gpio_init(s_gpioled.port, s_gpioled.pin, hal_gpio_dirOUT, hal_gpio_speed_low);
    // gpio in: button user
    hal_gpio_init(s_gpioinp.port, s_gpioinp.pin, hal_gpio_dirINP, hal_gpio_speed_low);

    hal_trace_init();

    hal_trace_puts("initted hal");
    
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
        shalpart_proc_synchronise(ee_procApplication, &s_eapp_info);
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
    
    hal_trace_puts("initted sys-shals");      
   
}


// give osal memory, create one task 
static void s_initialise_osal(void)
{
    uint32_t ram08size = 0;
    uint64_t *ram08data = NULL;
    extern osal_cfg_t *osal_cfgMINE;
    
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
    osal_timer_timing_t timing = { .startat = OSAL_abstimeNONE, .count = tmr_period_led, .mode = osal_tmrmodeFOREVER };
    osal_timer_onexpiry_t onexpiry = { .cbk = s_toggle_led_cbk, .par = (void*)&s_gpioled };
    osal_timer_t *tmr = NULL;


    // create a timer with a callback function which toggle a led.
    s_tmr_led = osal_timer_new();
    osal_timer_start(s_tmr_led, &timing, &onexpiry, osal_callerTSK);


    // create a timer with a callback which monitor the input button
    tmr = osal_timer_new();

    timing.startat = OSAL_abstimeNONE; timing.count = tmr_period_inp; timing.mode = osal_tmrmodeFOREVER;
    onexpiry.cbk = s_check_inp_cbk; onexpiry.par = (void*)&s_gpioinp;

    osal_timer_start(tmr, &timing, &onexpiry, osal_callerTSK);
    
    s_tskid_work = osal_task_new(s_task_work, NULL, 50, 1024);    
   

}

static void s_toggle_led_cbk(osal_timer_t *tmr, void *arg)
{
    const mygpio_t *gp = (const mygpio_t *)arg;
    static hal_gpio_val_t val = hal_gpio_valLOW;
    static uint32_t number = 0;
    char mystring[64];

    val = (hal_gpio_valHIGH == val) ? (hal_gpio_valLOW) : (hal_gpio_valHIGH);

    hal_gpio_setval(gp->port, gp->pin, val);

    sprintf(mystring, "timer callback has toggled the led for the %d-th time", ++number); 

    hal_trace_puts(mystring);

}


static void s_check_inp_cbk(osal_timer_t *tmr, void *arg)
{
    const mygpio_t *gp = (const mygpio_t *)arg;

    if(hal_gpio_valLOW == hal_gpio_getval(gp->port, gp->pin))
    {
        // send evt to task
        osal_eventflag_set(0x1, s_tskid_work, osal_callerISR);
    }

}



static void s_task_work(void *arg)
{
    osal_result_t res = osal_res_NOK_generic;
    osal_eventflag_t rxmsk = 0;
    uint32_t cnt = 0;

    for(;;)
    {
        res = osal_eventflag_get(0xFFFFFFFF, osal_waitANYflag, &rxmsk, OSAL_reltimeINFINITE); 
        
        if(osal_res_OK == res)
        {
            if(0x1 == (rxmsk & 0x1))
            {
                hal_trace_puts("pushed the button. in three secs i will attempt to jump to updater");

                // do whatever you like.
                // for instance .... go to eupdater
                cnt ++;
                cnt = cnt;


                s_attempt_goto_eupdater();

                // it resets any pending flags set during the time spent in the loop.
                osal_eventflag_get(0xFFFFFFFF, osal_waitANYflag, &rxmsk, OSAL_reltimeZERO);

            }
        
        }     

    }

}




static void s_nomemory_anymore(void)
{
    static uint8_t nomem = 0;
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


    for(;;)
    {
        for(n=1; n< 0x00100000; n++) { a = n / (n-1); }
        s_toggle_led_cbk(NULL, (void*)&s_gpioled);
    }
}

static void s_attempt_goto_eupdater(void)
{
    osal_timer_timing_t timing = { .startat = OSAL_abstimeNONE, .count = tmr_period_led_wild, .mode = osal_tmrmodeFOREVER };
    osal_timer_onexpiry_t onexpiry = { .cbk = s_toggle_led_cbk, .par = (void*)&s_gpioled };

    osal_timer_stop(s_tmr_led, osal_callerTSK);
    osal_timer_start(s_tmr_led, &timing, &onexpiry, osal_callerTSK);


    osal_task_wait(3*1000*1000);


    // ok, use shalbase to set teh gotoflag and to restart.

    shalbase_ipc_gotoproc_set(ee_procUpdater);

    shalbase_system_restart();


    // we should never be in here.
    osal_timer_stop(s_tmr_led, osal_callerTSK);
    timing.count = tmr_period_led;
    osal_timer_start(s_tmr_led, &timing, &onexpiry, osal_callerTSK);
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------


 




