
/* @file       simple-app.c
	@brief      This file implements a test for embobj with the use of abslayer under multitasking environment
	@author     marco.accame@iit.it
    @date       06/21/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdint.h"
#include "stdlib.h"

// abslayer 
#include "hal.h"
#include "osal.h"
#include "ipal.h"
#include "fsal.h"

// embobj
#include "EOMtheSystem.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrormanager.h"
#include "EoCommon.h"
#include "EOdeque.h"
#include "EOaction.h"

// test
#include "test-eocontainers.h"
#include "test-eogpio.h"
#include "test-eomultitask.h"
#include "test-eosystem.h"
#include "test-eostatemachine.h"







// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------

extern const hal_params_cfg_t *hal_params_cfgMINE;
extern const fsal_params_cfg_t *fsal_params_cfgMINE;
extern const osal_params_cfg_t *osal_params_cfgMINE;
 
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

static void s_errman_OnError(eOerrmanErrorType_t errtype, eOid08_t taskid, const char *eobjstr, const char *info);

extern const eOerrman_cfg_t errman_cfg = 
{
    .extfn = 
    {
        .usr_on_error   = s_errman_OnError
    }
};

extern const eOmempool_cfg_t mempool_cfg = 
{
    eo_mempool_alloc_dynamic,
    0, NULL,
    0, NULL,
    0, NULL,
    0, NULL
};


extern const eOmtimerman_cfg_t tmrman_cfg =
{
    240,    // priority
    512,    // stacksize
    8       // messagequeuesize
};


extern const eOmcallbackman_cfg_t cbkman_cfg =
{
    200,    // priority
    512,    // stacksize
    8       // messagequeuesize
};


extern const hal_params_cfg_t *hal_params_cfgMINE;
extern const fsal_params_cfg_t *fsal_params_cfgMINE;
extern const osal_params_cfg_t *osal_params_cfgMINE;

extern eOmsystem_cfg_t sys_cfg =
{
    .halcfg     = NULL,
    .osalcfg    = NULL,
    .fsalcfg    = NULL
};

// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------


//static void s_initialise_hal(void);
//static void s_initialise_fsal(void);
//static void s_initialise_osal_plus_test(void);

//static void s_nomemory_anymore(void);


void s_task_init(void);

void s_task_test_embobj(void *p);

static void s_testembobj_onstart(void);
static void s_testembobj_oncycle(osal_time_t per);
static void s_testembobj_onbuttonpressedreleased(void);



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static osal_task_t *s_tskid_test = NULL;


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

#undef _OSAL_WITH_ONE_TASK_

#if 1

#include "EOtheNVsCfgDevice.h"

int main(void)
{
            sys_cfg.halcfg     = hal_params_cfgMINE;
            sys_cfg.osalcfg    = osal_params_cfgMINE;
            sys_cfg.fsalcfg    = fsal_params_cfgMINE;

    
              eom_sys_Initialise(   &sys_cfg,
                                    NULL,                   // mempool
                                    &errman_cfg,            // errman
                                    &eom_timerman_DefaultCfg,
                                    &eom_callbackman_DefaultCfg
                                );  
    
    eom_sys_Start(eom_sys_GetHandle(), test_EOmultitask_init00);


}



#else
int main(void)
{

    static EOaction * act0;
    static EOaction * act1;
    static EOaction * act2;



    s_initialise_hal();

    s_initialise_fsal();


    test_eosm_Init();


//    test_eonetvar_Init();

    act0 = eo_action_New();
    eo_action_SetEvent(act0, 0x1234, (void*)0x9abcdef0);

    act1 = eo_action_New();
    eo_action_SetMessage(act1, 0x12345678, (void*)0x9abcdef1);

    act2 = eo_action_New();
    eo_action_SetCallback(act2, s_initialise_hal, NULL);


    


#ifdef _OSAL_WITH_ONE_TASK_
    // one single task 
    s_initialise_osal_plus_test();
#else
    // many tasks, started in teh proper way ....
    test_EOmultitask_start();
#endif  
    
    for(;;);


}
#endif

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

  


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

// give hal memory, initialise sys, leds of mcbstm32c all off, display 
//static void s_initialise_hal(void)
//{
//    uint32_t ram04size = 0;
//    uint32_t *ram04data = NULL;
//    
//    hal_memory_getsize(hal_params_cfgMINE, &ram04size);
//    
//    if(0 != ram04size)
//    {
//        ram04data = (uint32_t*)calloc(ram04size/4, sizeof(uint32_t));
//        
//        if(NULL == ram04data)
//        {
//            s_nomemory_anymore();
//        }
//    }
//
//    hal_initialise(hal_params_cfgMINE, ram04data);
//    
//    // system
//    hal_sys_systeminit();
//
//    // gpio out: leds
//    hal_gpio_init(hal_gpio_portE, hal_gpio_pin8,  hal_gpio_dirOUT);
//    hal_gpio_init(hal_gpio_portE, hal_gpio_pin9,  hal_gpio_dirOUT);
//    hal_gpio_init(hal_gpio_portE, hal_gpio_pin10, hal_gpio_dirOUT);
//    hal_gpio_init(hal_gpio_portE, hal_gpio_pin11, hal_gpio_dirOUT);
//    hal_gpio_init(hal_gpio_portE, hal_gpio_pin12, hal_gpio_dirOUT);
//    hal_gpio_init(hal_gpio_portE, hal_gpio_pin13, hal_gpio_dirOUT);
//    hal_gpio_init(hal_gpio_portE, hal_gpio_pin14, hal_gpio_dirOUT);
//    hal_gpio_init(hal_gpio_portE, hal_gpio_pin15, hal_gpio_dirOUT);
//    // gpio in: button user
//    hal_gpio_init(hal_gpio_portB, hal_gpio_pin7, hal_gpio_dirINP);
//    // led8 on
//    hal_gpio_setval(hal_gpio_portE, hal_gpio_pin8,  hal_gpio_valHIGH);
//    
//    // display
//    hal_display_init();
//    hal_display_clear(hal_col_white);
//    hal_display_settext(hal_font_24x16, hal_col_darkgrey, hal_col_yellow);
//    hal_display_putstring(1, "testing embobj");    
//    
//}


//// give fsal memory, format if there isnt file acemor.sig, printf on itm 
//static void s_initialise_fsal(void)
//{
//    uint32_t ram04size = 0;
//    uint32_t *ram04data = NULL;
//    fsal_info_search_t infosearch;
//    FILE *fp = NULL;
//    const uint32_t data[] = {0, 0, 7};
//    uint32_t len = 0;
//    
//    fsal_memory_getsize(fsal_params_cfgMINE, &ram04size);
//
//    
//    if(0 != ram04size)
//    {
//        ram04data = (uint32_t*)calloc(ram04size/4, sizeof(uint32_t));
//        
//        if(NULL == ram04data)
//        {
//            s_nomemory_anymore();
//        }
//    }
//
//    fsal_initialise(fsal_params_cfgMINE, ram04data);
//
//    printf("fsal initialisation done\n");
//    printf("used: stdio_enable = %d and eflash_enable = %d\n", fsal_params_cfgMINE->stdio_enable, 
//                                                               fsal_params_cfgMINE->eflash_enable);
//
//
//    if(1 == fsal_params_cfgMINE->eflash_enable)
//    {
//        infosearch.fileid = 0;
//
//        if(fsal_res_NOK_generic == fsal_find("acemor.sig", &infosearch))
//        {
//            fsal_format("");
//            printf("formatted the drive\n");
//    
//            fp = fopen("acemor.sig", "w");
//            if(NULL != fp)
//            {
//                len = fwrite(data, 1, sizeof(data), fp);
//                fclose(fp);
//                printf("created file acemor.sig. it has %d bytes\n", len);
//            }
//            else
//            {
//                printf("failed to create file acemor.sig\n");
//            }
//    
//        }
//        else
//        {
//            fsal_defrag("");
//            printf("file acemor.sig is already present. just defragged the drive\n");
//        }
//    }
//   
//  
//}


// this test runs the osal which launches an init task whcih in turns:
// a. starts the EOMtheSystem to define the memorypool and the error manager. 
// b. starts a periodic task s_task_test_embobj whcih blinks led8 and change 
//    value to led9 when button user is pressed or released. moreover, there are
//    two functions,  onstart() and oncycle(), whcih can be used to do things.

//static void s_initialise_osal_plus_test(void)
//{
//    uint32_t ram08size = 0;
//    uint64_t *ram08data = NULL;
//    
//    osal_memory_getsize(osal_params_cfgMINE, &ram08size);
//    
//    if(0 != ram08size)
//    {
//        ram08data = (uint64_t*)calloc(ram08size/8, sizeof(uint64_t));
//        
//        if(NULL == ram08data)
//        {
//            s_nomemory_anymore();
//        }
//    }
//
//    osal_initialise(osal_params_cfgMINE, ram08data);
//    
//    osal_start(s_task_init);
//    
//}


void s_task_init(void)
{
    const osal_time_t period  = 500*1000; // 500 msec 

//    // first thing .... initialise the EOMtheSystem. i dont start it however.
//    eom_sys_Initialise(&mempool_cfg);


    // create test embobj task, low priority and periodic
    s_tskid_test = osal_task_new(s_task_test_embobj, (void*)period, 10, 512);
    s_tskid_test = s_tskid_test;
}


void s_task_test_embobj(void *p)
{
    osal_time_t per = (uint32_t)p;
    hal_gpio_val_t curval = hal_gpio_valHIGH;
    hal_gpio_val_t currinp = hal_gpio_valNONE;
    hal_gpio_val_t previnp = hal_gpio_valNONE;



    s_testembobj_onstart(); 


    osal_system_task_period_set(per);

    for(;;)
    {
        osal_system_task_period_wait();


        s_testembobj_oncycle(per);
        
        // do action .... toggle a led
        curval = (hal_gpio_valHIGH == curval) ? (hal_gpio_valLOW) : (hal_gpio_valHIGH);
        hal_gpio_setval(hal_gpio_portE, hal_gpio_pin8, curval);

        // and get value of input button
        currinp = hal_gpio_getval(hal_gpio_portB, hal_gpio_pin7);

        if(previnp == hal_gpio_valNONE)
        {
            previnp = currinp;
        }
 
        if(currinp != previnp)
        {
            s_testembobj_onbuttonpressedreleased();
        }

        previnp = currinp;
        currinp = hal_gpio_valNONE;

    }
}


static void s_testembobj_onbuttonpressedreleased(void)
{
    hal_gpio_val_t curval = hal_gpio_getval(hal_gpio_portE, hal_gpio_pin9);
    curval = (hal_gpio_valHIGH == curval) ? (hal_gpio_valLOW) : (hal_gpio_valHIGH);
    hal_gpio_setval(hal_gpio_portE, hal_gpio_pin9, curval);
}


//static void s_nomemory_anymore(void)
//{
//    for(;;);
//}


void osal_on_idle(void)
{
    static uint32_t cnt = 0;

    for(;;)
    {
        cnt++;
    }
}


static void s_errman_OnError(eOerrmanErrorType_t errtype, eOid08_t taskid, const char *eobjstr, const char *info)
{
    const char err[4][16] = {"info", "warning", "weak error", "fatal error"};

    printf("[eobj: %s, tsk: %d] %s: %s\n\r", eobjstr, taskid, err[(uint8_t)errtype], info);

    if(errtype <= eo_errortype_warning)
    {
        return;
    }

    for(;;);
}



static void s_testembobj_onstart(void)
{
    // they simply call mempool and errormanager. they printf
    test_eosys_EOmemory();
    test_eosys_EOerror();

    // they simply call the containers. follow with debugger
    test_eocont_EOfifoWord();
    test_eocont_EOfifoByte();
    test_eocont_EOfifo();
    test_eocont_EOlist();
    test_eocont_EOdeque();

    // initialise gpio manager to add a waveform on led15 and a 3-sec trigger on button wakeup
    // which toggles led 10
    test_eogpio_EOgpioInit();

//    test_EOmultitask_run();




}

static void s_testembobj_oncycle(osal_time_t per)
{
    test_eogpio_EOgpioTick(per);
}




// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



