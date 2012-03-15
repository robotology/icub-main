
/* @file       osal_cfg.c
	@brief      This file keeps internal implementation of the osal.
	@author     marco.accame@iit.it
    @date       11/27/2009
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "osal.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "osal_cfg.h"

static void s_osal_cfg_on_fatal_error(void* task, uint32_t errorcode, const char * errormsg);
static void s_osal_cfg_on_idle(void);


static const osal_params_cfg_t s_cfg = 
{   
    .rtostype               = OSAL_RTOSTYPE,          // uint8_t         rtostype;
    .memorymodel            = OSAL_MEMMODEL,
    .cpufam                 = OSAL_CPUFAM,            // uint8_t         cpufam;                                 
    .cpufreq                = OSAL_CPUFREQ,           // uint32_t        cpufreq;                                
    .tick                   = OSAL_TICK,              // uint32_t        tick;                                   
    .launcherstacksize      = OSAL_LAUNSTKSIZE,       // uint16_t        launcherstacksize;                     
    .idlestacksize          = OSAL_IDLESTKSIZE,       // uint16_t        idlestacksize;
    .globalstacksize        = OSAL_GLOBSTKSIZE,       // uint16_t        globalstacksize;
    .roundrobin             = OSAL_RROBIN,            // uint8_t         roundrobin;
    .roundrobintick         = OSAL_RROBINTICK,        // uint32_t        roundrobintick;
    .tasknum                = OSAL_TASKNUM,           // uint8_t         tasknum;
    .timernum               = OSAL_TIMERNUM,          // uint8_t         timernum;
    .mutexnum               = OSAL_MUTEXNUM,          // uint8_t         mutexnum;
    .semaphorenum           = OSAL_SEMAPHORENUM,      // uint8_t         semaphorenum;
    .mqueuenum              = OSAL_MQUEUENUM,         
    .mqueueelemnum          = OSAL_MQUEUEELEMNUM,       
    .extfn                  = 
    {
        .usr_on_fatal_error     = s_osal_cfg_on_fatal_error, 
        .usr_on_idle            = s_osal_cfg_on_idle
    }
};


extern const osal_params_cfg_t *osal_params_cfgMINE = &s_cfg;


static void s_osal_cfg_on_fatal_error(void* task, uint32_t errorcode, const char * errormsg)
{
    for(;;);
}

static void s_osal_cfg_on_idle(void)
{
    for(;;);
}


// -- redefinition of ...

// required by the arm c stdlib: gives a different memory space for the stdlib to each thread in the arm compiler
void * __user_perthread_libspace(void) 
{ 
    static volatile uint8_t fc = 1; 
    void *ret = osal_archdep_armc99stdlib_getlibspace(fc);
    fc = 0; 
    return(ret);
}

// required by the arm c stdlib: initialises a mutex
int _mutex_initialize(void *m) 
{ 
    return(osal_archdep_armc99stdlib_mutex_initialize(m)); 
}

// required by the arm c stdlib: takes a mutex
void _mutex_acquire(void *m) 
{ 
    osal_archdep_armc99stdlib_mutex_acquire(m); 
} 

// required by the arm c stdlib: releases a mutex
void _mutex_release(void *m) 
{ 
    osal_archdep_armc99stdlib_mutex_release(m); 
}  





// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



