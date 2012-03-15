
/* @file       loader_mcbstm32c.c
    @brief      This  file implements the eLoader process for a stm32f107 onto the keil board.
    @author     marco.accame@iit.it
    @date       03/11/2011
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "hal.h"
#include "hal_arch.h"
#include "hal_switch.h"



#include "string.h"
#include "stdlib.h"

#include "eEcommon.h"

#include "shalBASE.h" 
#include "shalPART.h"
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
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

//#define EENV_UPDT_ROMSTART_DEF     (EENV_MEMMAP_EUPDATER_ROMADDR) 
//#define EENV_APPL_ROMSTART_DEF     (EENV_MEMMAP_EAPPLICATION_ROMADDR) 

#define LOADER_ADR_INVALID         0xffffffff
#define LOADER_ADR_DEF_UPD         EENV_MEMMAP_EUPDATER_ROMADDR 
#define LOADER_ADR_DEF_APP         EENV_MEMMAP_EAPPLICATION_ROMADDR

#define LOADER_LED                 0x100




// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------


static void s_loader_hardware_init(void);

static void s_loader_shalibs_init(void);

static void s_loader_Delay(uint32_t dlyTicks);
static void s_loader_manage_error(uint32_t onmilli, uint32_t offmilli);
static void s_loader_exec_loader(void);
static void s_loader_eval_jump_request_from_an_eproc(void);
static void s_loader_eval_def_from_partition_table(void);
static void s_loader_eval_default(void);
static void s_loader_attempt_jump(eEprocess_t proc, uint32_t adr);

static void s_loader_HW_init(void);
static void s_loader_HW_SYSTICK_Config(void);
static void s_loader_HW_LED_Config(void);
static void s_loader_HW_LED_On(uint32_t led);
static void s_loader_HW_LED_Off(uint32_t led);

#ifndef EMS001
static void s_loader_eval_user_request(void);
static void s_loader_HW_INP_Config(void);
static uint32_t s_loader_HW_INP_IsPushed(void);
#endif


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const eEmoduleInfo_t s_loader_info __attribute__((at(EENV_MEMMAP_ELOADER_ROMADDR+EENV_MODULEINFO_OFFSET))) = 
{
    .info           =
    {
        .entity     =
        {
            .type       = ee_entity_process,
            .signature  = ee_procLoader,
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
            .addr   = EENV_MEMMAP_ELOADER_ROMADDR,
            .size   = EENV_MEMMAP_ELOADER_ROMSIZE
        },
        .ram        = 
        {   
            .addr   = EENV_MEMMAP_ELOADER_RAMADDR,
            .size   = EENV_MEMMAP_ELOADER_RAMSIZE
        },
        .storage    = 
        {
            .type   = ee_strg_none,
            .size   = 0,
            .addr   = 0
        },
        .communication  = ee_commtype_none,
        .name           = "eLoader"
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

#ifdef EMS001
static eEboardInfo_t s_loader_boardinfo =                        
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
                .year  = 2011,
                .month = 11,
                .day   = 3,
                .hour  = 18,
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
        .name           = "ems001"
    },
    .uniqueid       = 0,
    .extra          = {0}
};
#else
static eEboardInfo_t s_loader_boardinfo =                        
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
#endif


static volatile uint32_t s_loader_msTicks;  

static uint8_t hw_initted = 0; 

#ifndef EMS001
static const hal_gpio_port_t s_led_loader_port = hal_gpio_portE;
static const hal_gpio_pin_t  s_led_loader_pin  = hal_gpio_pin8;
static const hal_gpio_val_t  s_led_on  = hal_gpio_valHIGH;
static const hal_gpio_val_t  s_led_off = hal_gpio_valLOW;
#else
static const hal_gpio_port_t s_led_loader_port = hal_gpio_portE;
static const hal_gpio_pin_t  s_led_loader_pin  = hal_gpio_pin8;
static const hal_gpio_val_t  s_led_on  = hal_gpio_valLOW;
static const hal_gpio_val_t  s_led_off = hal_gpio_valHIGH;
#endif


                        

// --------------------------------------------------------------------------------------------------------------------
// - definition of main 
// --------------------------------------------------------------------------------------------------------------------

int main(void) 
{
    static volatile hal_result_t reshal = hal_res_NOK_generic;
//    static const eEmoduleInfo_t *mii;

    s_loader_hardware_init();


    s_loader_shalibs_init();



#ifndef EMS001
    // if any user request, it either jumps or manage error. else, it returns.
    s_loader_eval_user_request();
#endif

    // if any, it either jumps or manage errors. else, it returns.
    s_loader_eval_jump_request_from_an_eproc();

    // if any, it either jumps or manage errors. else, it returns.
    s_loader_eval_def_from_partition_table();

    // if in here ... try last time
    s_loader_eval_default();

    // nothing to do ...
    s_loader_manage_error(400, 100);

    // just for ...
    for(;;);

}




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static void s_loader_shalibs_init(void)
{
    static volatile hal_result_t reshal = hal_res_NOK_generic;
//    static const eEmoduleInfo_t *mii;

    if(ee_res_OK != shalbase_isvalid())
    {
        s_loader_manage_error(50, 50);
    }
    else
    {
        // init shalbase
        if(ee_res_OK != shalbase_init(1))
        {
            s_loader_manage_error(50, 100);
        }
    }


    if(ee_res_OK != shalpart_isvalid())
    {
        s_loader_manage_error(100, 100);
    }
    else
    {
        // init shalpart
        if(ee_res_OK != shalpart_init())
        {
            s_loader_manage_error(50, 100);
        }
        // put signature in partition table using shalpart
        shalpart_proc_synchronise(ee_procLoader, &s_loader_info);
    }

//    shalpart_shal_get(ee_shalPART, &mii);
//    mii = mii;
//    shalpart_proc_get(ee_procLoader, &mii);
//    mii = mii;

    if(ee_res_OK != shalinfo_isvalid())
    {
        s_loader_manage_error(200, 200);
    }
    else
    {
        if(ee_res_OK != shalinfo_init())
        {
            s_loader_manage_error(50, 100);
        }
        s_loader_boardinfo.uniqueid = hal_arch_arm_uniqueid64_get();
        shalinfo_boardinfo_synchronise(&s_loader_boardinfo);
    }

}



static void s_loader_Delay(uint32_t dlyTicks) 
{
    uint32_t curTicks;
    curTicks = s_loader_msTicks;
    while((s_loader_msTicks - curTicks) < dlyTicks);
}




static void s_loader_manage_error(uint32_t onmilli, uint32_t offmilli)
{
    s_loader_HW_init();

    s_loader_HW_LED_Config();
    
    s_loader_HW_SYSTICK_Config(); 


    while(1) 
    {
        s_loader_HW_LED_On (LOADER_LED);                            
        s_loader_Delay (onmilli);                                
        s_loader_HW_LED_Off (LOADER_LED);                            
        s_loader_Delay (offmilli);                               
    }

}


void s_loader_exec_loader(void)
{
    uint32_t cnt = 10;
    uint8_t data[8];
    uint8_t size = 0;
    eEresult_t res;

    uint8_t factor_on  = 1;
    uint8_t factor_off = 9;


    s_loader_HW_SYSTICK_Config(); 


    s_loader_HW_LED_Config(); 


    // i could see if anybody has given me any message on shared data.

    res = shalbase_ipc_volatiledata_get(data, &size, 8);

    if(ee_res_OK == res)
    {
        factor_on  = data[0];
        factor_off = data[1]; 
        
        shalbase_ipc_volatiledata_clr();  
    }

    while(1) 
    {
        if(0 == cnt)
        {
            // try to go back to default applications
            s_loader_eval_default();
            // nothing to do ...
            s_loader_manage_error(200, 50);
        }

        cnt --;

        s_loader_HW_LED_On (LOADER_LED);                            
        s_loader_Delay (100*factor_on);                                
        s_loader_HW_LED_Off (LOADER_LED);                            
        s_loader_Delay (100*factor_off);                               
    }

}

static void s_loader_attempt_jump(eEprocess_t proc, uint32_t adr)
{
    uint32_t address = LOADER_ADR_INVALID;

    if(ee_procNone != proc)
    {

        if(ee_procLoader == proc)
        {
            // avoid recursive bootstraps
            s_loader_exec_loader();
        }
     
        // attempt with the proc
        if(ee_res_OK == shalpart_proc_runaddress_get(proc, &address))
        {
            if(ee_res_OK == shalbase_system_canjump(address))
            {
                shalbase_system_jumpnow(address);
            }
        }

    }

    if(LOADER_ADR_INVALID != adr)
    {
        // attempt with adr 
        if(ee_res_OK == shalbase_system_canjump(adr))
        {
            shalbase_system_jumpnow(adr);
        }
    }

    // if i am in here we cannot jump ...
    return;
}

#ifndef EMS001
static void s_loader_eval_user_request(void)
{
    // we see if the user wants to go to the updater by pushing the button
    if(1 == s_loader_HW_INP_IsPushed())
    {
        s_loader_attempt_jump(ee_procUpdater, LOADER_ADR_DEF_UPD);
        // if in here ... the jump failed, thus i go to error.
        s_loader_manage_error(200, 800); 
    }

    // if hw can tell none, updater, one, another etc. put a switch-case and if the chosen fails ... manage error.
}
#endif// EMS001

static void s_loader_eval_jump_request_from_an_eproc(void)
{
    eEprocess_t pr = ee_procNone;

    if(ee_res_OK == shalbase_ipc_gotoproc_get(&pr))
    {
        shalbase_ipc_gotoproc_clr();
        s_loader_attempt_jump(pr, LOADER_ADR_INVALID);
        // if in here ... the jump failed, thus ... i dont go to error i just go on with other choices ... part table and default
        //s_loader_manage_error(800, 200);
    }
}


static void s_loader_eval_def_from_partition_table(void)
{
    eEprocess_t pr = ee_procNone;

    if(ee_res_OK == shalpart_proc_def2run_get(&pr))
    {
        s_loader_attempt_jump(pr, LOADER_ADR_INVALID);
        // but if jump fails, go on ...    
    }
}

static void s_loader_eval_default(void)
{
    s_loader_attempt_jump(ee_procNone, LOADER_ADR_DEF_APP);
    s_loader_attempt_jump(ee_procNone, LOADER_ADR_DEF_UPD);
}


// - hw dependant functions -------------------------------------------------------------------------------------------

static void s_loader_hardware_init(void)
{
    // init the hw ...
    s_loader_HW_init();
}


void systickhandler(void) 
{
    s_loader_msTicks++;                      
}

static void s_loader_HW_init(void)
{
    extern const hal_cfg_t *hal_cfgMINE;
    uint32_t size04aligned;
    uint32_t *data32aligned = NULL;

    if(1 == hw_initted)
    {
        return;
    }
    else
    {
        hw_initted = 1;
    }

    hal_base_memory_getsize(hal_cfgMINE, &size04aligned); 

    if(0 != size04aligned)
    {
        data32aligned = (uint32_t*)calloc(size04aligned/4, sizeof(uint32_t));   
    }

    hal_base_initialise(hal_cfgMINE, data32aligned);

    hal_sys_systeminit();

    hal_eeprom_init(hal_eeprom_i2c_01, NULL); 

#ifndef EMS001
    s_loader_HW_INP_Config();
#endif

#ifdef EMS001
    if(hal_false == hal_switch_initted_is())
    {
        hal_switch_init(NULL);
    } 
#endif

}


static void s_loader_HW_SYSTICK_Config(void)
{
    hal_sys_systick_sethandler(systickhandler, 1000, hal_int_priority15); // 1 milli
}

#ifndef EMS001
static void s_loader_HW_INP_Config(void)
{
    hal_gpio_init(hal_gpio_portB, hal_gpio_pin7, hal_gpio_dirINP, hal_gpio_speed_low);
}

static uint32_t s_loader_HW_INP_IsPushed(void)
{
    return((hal_gpio_valLOW == hal_gpio_getval(hal_gpio_portB, hal_gpio_pin7)) ? (1) : (0));
}
#endif//EMS001

static void s_loader_HW_LED_Config(void) 
{
    // assume port E. the LOADER_LED is led 8
    hal_gpio_init(s_led_loader_port, s_led_loader_pin, hal_gpio_dirOUT, hal_gpio_speed_low);
}


static void s_loader_HW_LED_On(uint32_t led) 
{
    hal_gpio_setval(s_led_loader_port, s_led_loader_pin, s_led_on);
}


static void s_loader_HW_LED_Off(uint32_t led) 
{
     hal_gpio_setval(s_led_loader_port, s_led_loader_pin, s_led_off);
}


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------


 




