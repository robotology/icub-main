
/* @file       demo-mee-core-gpio-sm-main.c
	@brief      This file implements a test for BASE/GPIO/SM in embOBJ built upon to its basic distribution for MEE
	@author     marco.accame@iit.it
    @date       09/02/2011
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------


#include "stdint.h"
#include "stdlib.h"
#include "string.h"

// abslayer 
#include "hal.h"
#include "osal.h"
#include "fsal.h"

// embobj
//#include "EoCommon.h"
#include "EOMtheSystem.h"
//#include "EOtheMemoryPool.h"
//#include "EOtheErrormanager.h"


#include "demo-mee-core-nvs-appl.h"
#include "demo-info.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



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

static void s_test_mee_core_gpio_errman_OnError(eOerrmanErrorType_t errtype, eOid08_t taskid, const char *eobjstr, const char *info);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


static const eOerrman_cfg_t errman_cfg = 
{
    .extfn = 
    {
        .usr_on_error   = s_test_mee_core_gpio_errman_OnError
    }
};


//static const eOmtimerman_cfg_t tmrman_cfg = 
//{   // change it as u like
//    .priority           = 240, 
//    .stacksize          = 512, 
//    .messagequeuesize   = 8
//};


//static const eOmcallbackman_cfg_t cbkman_cfg = 
//{   // change it as u like
//    .priority   = 202, 
//    .stacksize  = 512, 
//    .queuesize  = 8
//};


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


int main(void)
{

    // init the system
    eom_sys_Initialise  (   &demoinfo_syscfg,                       // sys
                            NULL,                           // mempool
                            &errman_cfg,                    // errman
                            &eom_timerman_DefaultCfg,       // timerman
                            &eom_callbackman_DefaultCfg     // callbackman
                        );  
    
    // start the system with a user-defined function
    eom_sys_Start(eom_sys_GetHandle(), demo_mee_core_nvs_appl_init00);

    return(0);  // never in here.
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

  


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static void s_test_mee_core_gpio_errman_OnError(eOerrmanErrorType_t errtype, eOid08_t taskid, const char *eobjstr, const char *info)
{
    const char err[4][16] = {"info", "warning", "weak error", "fatal error"};
    char str[128];

    snprintf(str, sizeof(str)-1, "[eobj: %s, tsk: %d] %s: %s\n\r", eobjstr, taskid, err[(uint8_t)errtype], info);
    hal_trace_puts(str);

    if(errtype <= eo_errortype_warning)
    {
        return;
    }

    for(;;);
}





// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



