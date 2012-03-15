
/* @file       nvs-app.c
	@brief      This file implements a test for embobj with the use of nvs
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

#include "test-nvs.h"
#include "test-nvs-node00.h"









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

extern const eOmsystem_cfg_t sys_cfg =
{
    .halcfg     = hal_params_cfgMINE,
    .osalcfg    = osal_params_cfgMINE,
    .fsalcfg    = fsal_params_cfgMINE
};

// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------


static void s_task_init(void);

static void s_task_test_embobj(void *p);

static void s_testembobj_onstart(void);
static void s_testembobj_oncycle(osal_time_t per);




// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


static osal_task_t *s_tskid_test = NULL;


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


#include "EOtheNVsCfgDevice.h"

int main(void)
{

    
              eom_sys_Initialise(   sys_cfg,
                                    NULL,                   // mempool
                                    NULL,                   // errman
                                    &eom_timerman_DefaultCfg,
                                    &eom_callbackman_DefaultCfg
                                );  
    
            eom_sys_Start(eom_sys_GetHandle(), s_task_init);


}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


extern void eo_errman_OnError(eOerrmanErrorType_t errtype, uint32_t taskid, const char *eobjstr, const char *info)
{
    const char err[4][16] = {"info", "warning", "weak error", "fatal error"};

    printf("[eobj: %s, tsk: %d] %s: %s\n\r", eobjstr, taskid, err[(uint8_t)errtype], info);

    if(errtype <= eo_errortype_warning)
    {
        return;
    }

    for(;;);
}

 


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------



static void s_task_init(void)
{
    const osal_time_t period  = 500*1000; // 500 msec 

    // create test embobj task, low priority and periodic
    s_tskid_test = osal_task_new(s_task_test_embobj, (void*)period, 10, 512);
    s_tskid_test = s_tskid_test;
}


static void s_task_test_embobj(void *p)
{
    osal_time_t per = (uint32_t)p;


    s_testembobj_onstart(); 


    osal_system_task_period_set(per);

    for(;;)
    {
        osal_system_task_period_wait();
        s_testembobj_oncycle(per);
    }
}





static void s_testembobj_onstart(void)
{
#if 0
    test_nvs_Init();
#else
    //test_nvs_somemore_Init();
    test_nvs_node00_Init();
#endif
    
}

static void s_testembobj_oncycle(osal_time_t per)
{
#if 0
    test_nvs_Do();
#else
    //test_nvs_somemore_Init();
    test_nvs_node00_Do();
#endif
}




// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



