
/* @file       test-basic-mee-appl.c
	@brief      This file implements a test for embobj
	@author     marco.accame@iit.it
    @date       06/21/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"

#include "hal.h"
#include "osal.h"
#include "fsal.h"


// embobj
#include "EoCommon.h"
#include "EOaction.h"

#include "EOMmutex.h"
#include "EOVmutex.h"

#include "EOMtask.h"
#include "EOVtask.h"

#include "EOMtheSystem.h"
#include "EOVtheSystem.h"

#include "EOtimer.h"
#include "EOMtheTimerManager.h"
#include "EOMtheCallbackManager.h"
#include "EOVtheCallbackManager.h"



#include "test-eonetvar.h"

#include "test-eocontainers.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------


 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "test-basic-mee-appl.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

// must be extern to be visible in uv4
extern void task_example00(void *p);
extern void task_example01(void *p);


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------



static void s_mytper_startup(EOMtask *p, uint32_t);
static void s_mytper_run(EOMtask *p, uint32_t);


static void s_mytmes_startup(EOMtask *p, uint32_t t);
static void s_mytmes_run(EOMtask *p, uint32_t t);

static void s_testeom_callback(void *arg);

//static void s_callback(void);




     

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


static EOtimer *s_tmr0 = 0;
static EOtimer *s_tmr1 = 0;

static EOaction * s_action;


EOMmutex *mmutex = NULL;
EOMtask *mytask_periodic = NULL;
EOMtask *mytask_message = NULL;


eOnanotime_t nanotime;

static uint32_t s_myu32 = 0;


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------





extern void test_basic_mee_appl_init00(void)
{
 
    mytask_periodic = eom_task_New(eom_mtask_Periodic, 70, 4*1024, s_mytper_startup, s_mytper_run, 0, 1*1000*1000, task_example00, "example00 per");

    mytask_message = eom_task_New(eom_mtask_MessageDriven, 68, 3*1024, s_mytmes_startup, s_mytmes_run,  4, eok_reltimeINFINITE, task_example01, "example01 msg");


    eov_sys_NanoTimeGet(eov_sys_GetHandle(), &nanotime);


    ///////////////////////////////////////////////////////////////////////////


    s_tmr0 = eo_timer_New();

    s_action = eo_action_New();


    eo_action_SetMessage(s_action, 0x12345678,  mytask_message);

 
    eo_timer_Start(s_tmr0, eok_abstimeNOW, 2*1000*1000, eo_tmrmode_FOREVER, s_action);
 

    s_tmr1 = eo_timer_New();


    eo_action_SetCallback(s_action, s_testeom_callback, &s_myu32, eom_callbackman_GetTask(eom_callbackman_GetHandle()));
    //eo_action_SetCallback(s_action, s_testeom_callback, &s_myu32, eov_callbackman_GetTask(eov_callbackman_GetHandle()));
    eo_timer_Start(s_tmr1, eok_abstimeNOW, 3*1000*1000, eo_tmrmode_FOREVER, s_action);

}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

  


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------



static void s_test_mutex_step_two(void)
{
    static eOresult_t res = eores_OK;


    res = eom_mutex_Take(mmutex, 1*1000*1000);
    res = res;
    // shall wait for 1 sec

    res = eom_mutex_Release(mmutex);
    // shall fail to release if another task took it.

    res = eov_mutex_Take(mmutex, 1*1000*1000);
    // shall still wait for 1 sec
 
}


static void s_mytper_startup(EOMtask *p, uint32_t t)
{
    void *tsk = NULL;
    static uint8_t taskid = 0;
    tsk = osal_system_task_get();
    taskid = *((uint8_t*)tsk);
    taskid = taskid;
    s_test_mutex_step_two();
}

static void s_mytper_run(EOMtask *p, uint32_t t)
{
    static uint32_t aaa = 0;
    eOmessage_t msg = 0x10000001;

    if(100 == ++aaa)
    {
        aaa = 0;
    }

    printf("the periodic task ah sent msg 0x%x to task message\n\r", msg);
    eom_task_SendMessage(mytask_message, msg, eok_reltimeINFINITE);
}


extern void task_example00(void *p)
{
    // do here whatever you like before startup() is executed and then forever()
    eom_task_Start(p);
}


static void s_mytmes_startup(EOMtask *p, uint32_t t)
{
       
}

static void s_mytmes_run(EOMtask *p, uint32_t t)
{
    eOmessage_t msg = (eOmessage_t)t;

    printf("task message has received msg = 0x%x\n\r", msg);

    eov_sys_NanoTimeGet(eov_sys_GetHandle(), &nanotime);
 

}

extern void task_example01(void *p)
{
    // do here whatever you like before startup() is executed and then forever()
    eom_task_Start(p);
}





static void s_testeom_callback(void *arg)
{
    static uint32_t nn = 0;
    static eOmessage_t msg = 0x10001000;
    hal_gpio_val_t curval;
    uint32_t *myarg32 = (uint32_t*)arg;

    if(NULL != myarg32)
    {
        (*myarg32) ++;
    }

    if(0 == nn)
    {
         hal_gpio_init(hal_gpio_portE, hal_gpio_pin14, hal_gpio_dirOUT, hal_gpio_speed_2MHz);
    }

    nn++;

    curval = hal_gpio_getval(hal_gpio_portE, hal_gpio_pin14);
    curval = (hal_gpio_valHIGH == curval) ? (hal_gpio_valLOW) : (hal_gpio_valHIGH);
    hal_gpio_setval(hal_gpio_portE, hal_gpio_pin14, curval);

    printf("callback called for the %d-th time\n\r", nn);

    printf("teh callback has sent msg %d to task message\n\r", msg);

    eom_task_SendMessage(mytask_message, msg, eok_reltimeINFINITE);
}



//static void s_callback(void)
//{
//    static volatile uint32_t aa = 0;
//
//    if(10 < ++aa)
//    {
//        aa = 0;
//    }
//    printf("gpio callback on WKUP\n");
//}






// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



