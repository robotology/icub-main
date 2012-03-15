
// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"


#include "EOtheErrorManager.h"
#include "EOVtheSystem_hid.h" 
#include "EOSmutex.h"
#include "EOStheTimerManager_hid.h"

#include "EOStheFOOP_hid.h" 



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOStheSystem.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOStheSystem_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

//#define EOS_SYS_FLAG_MESSAGE_AVAL  0x00000001
//#define EOS_SYS_FLAG_CALLBACK_AVAL 0x00000002


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


static eOresult_t s_eos_sys_start(void (*init_fn)(void));
static EOVtaskDerived * s_eos_sys_gettask(void);

static eOabstime_t s_eos_sys_timelifeget(void);
static void s_eos_sys_timelifeset(eOabstime_t ltime);
static uint64_t s_eos_sys_nanotimeget(void);

static void s_eos_sys_tick(void);

static void s_eos_sys_tick_timermanager(void);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


static const char s_eobj_ownname[] = "EOStheSystem";

static EOStheSystem s_eos_the_system = 
{
    .thevsys        = NULL,
    .syscfg         = NULL,
    .thefoop        = NULL,
    .tickoflife     = 0
};

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern EOStheSystem * eos_sys_Initialise(const eOssystem_cfg_t *syscfg,
                                         const eOmempool_cfg_t *mpoolcfg, 
                                         const eOerrman_cfg_t *errmancfg,
                                         const eOstimerman_cfg_t *tmrmancfg,
                                         const eOscallbackman_cfg_t *cbkmancfg,
                                         const eOsfoop_cfg_t *foopcfg)
{
    eOsfoop_cfg_t localfoopcfg;

    if(NULL != s_eos_the_system.thevsys) 
    {
        // already initialised
        return(&s_eos_the_system);
    }
    

    // mpoolcfg can be NULL: in such a case we use eo_mempool_alloc_dynamic mode
    // errmancfg can be NULL

    // mempool and error manager initialised inside here.
    s_eos_the_system.thevsys = eov_sys_hid_Initialise(  mpoolcfg,
                                                        errmancfg,
                                                        s_eos_sys_start, 
                                                        s_eos_sys_gettask, 
                                                        s_eos_sys_timelifeget, 
                                                        s_eos_sys_timelifeset, 
                                                        s_eos_sys_nanotimeget,
                                                        syscfg->hal_fns.hal_sys_irq_disable
                                                     );
                                                  
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != syscfg), s_eobj_ownname, "eos_sys_Start() uses a NULL syscfg"); 
    
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != syscfg->hal_fns.hal_base_init), s_eobj_ownname, "eos_sys_Start() uses a NULL hal_init()"); 
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != syscfg->hal_fns.hal_sys_systeminit), s_eobj_ownname, "eos_sys_Start() uses a NULL hal_sys_systeminit()");
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != syscfg->hal_fns.hal_sys_systick_sethandler), s_eobj_ownname, "eos_sys_Start() uses a NULL hal_sys_systick_sethandler()");
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != syscfg->hal_fns.hal_sys_atomic_bitwiseAND), s_eobj_ownname, "eos_sys_Start() uses a NULL hal_sys_atomic_bitwiseAND()");
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != syscfg->hal_fns.hal_sys_atomic_bitwiseOR), s_eobj_ownname, "eos_sys_Start() uses a NULL hal_sys_atomic_bitwiseOR()");
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != syscfg->hal_fns.hal_sys_criticalsection_take), s_eobj_ownname, "eos_sys_Start() uses a NULL hal_sys_criticalsection_take()");
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != syscfg->hal_fns.hal_sys_criticalsection_release), s_eobj_ownname, "eos_sys_Start() uses a NULL hal_sys_criticalsection_release()");
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != syscfg->hal_fns.hal_sys_irq_disable), s_eobj_ownname, "eos_sys_Start() uses a NULL hal_sys_irq_disable()");
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != syscfg->hal_fns.hal_sys_irq_enable), s_eobj_ownname, "eos_sys_Start() uses a NULL hal_sys_irq_enable()");
    
    // we can have a null fsal_init
    // eo_errman_Assert(eo_errman_GetHandle(), (NULL != syscfg->fsal_fns.fsal_init), s_eobj_ownname, "eos_sys_Start() uses a NULL fsal_init()");

   
    eo_errman_Assert(eo_errman_GetHandle(), (syscfg->userdef.systickperiod >= EOSSYS_min_systickperiod), s_eobj_ownname, "eos_sys_Start() uses systickperiod too small");
    eo_errman_Assert(eo_errman_GetHandle(), (syscfg->userdef.systickperiod <= EOSSYS_max_systickperiod), s_eobj_ownname, "eos_sys_Start() uses systickperiod too high");
    
    // after the previous assert() we are now sure that syscfg and foopcfg contain valid values. thus copy them
    
    // the syscfg
    s_eos_the_system.syscfg = syscfg;  

    // we adjust the various configurations, so that a NULL pointer means the default.

    if(NULL == foopcfg)
    {
        foopcfg = (eOsfoop_cfg_t*)&eos_foop_DefaultCfg;
    }
    memcpy(&localfoopcfg, foopcfg, sizeof(eOsfoop_cfg_t));
    foopcfg = &localfoopcfg;

    if(NULL == tmrmancfg)
    {
        tmrmancfg = (eOstimerman_cfg_t*)&eos_timerman_DefaultCfg;
    }

    if(NULL == cbkmancfg)
    {
        cbkmancfg = (eOscallbackman_cfg_t*)&eos_callbackman_DefaultCfg;
    }

    // then we adjust the foop cfg to have a queue size for callbacks as reported in the cfg of the cbk manager
    localfoopcfg.callbackfifosize = cbkmancfg->queuesize;
  
    // we always initialise the foop.                                                  
    s_eos_the_system.thefoop = eos_foop_Initialise(&localfoopcfg, (eObasicabstr_hal_sys_fn_t*)&s_eos_the_system.syscfg->hal_fns);

    // and reset the tickoflife
    s_eos_the_system.tickoflife = 0;

    // finally we initialise the timerman and the callbackman but only if ... they have non-zero values in their cfg
    // they are independent. one can use a timermanager but does not want execution of callbacks

    if(0 != tmrmancfg->timernum)
    {
        eos_timerman_Initialise(tmrmancfg);
        eos_foop_hid_SetOnTick(eos_foop_GetHandle(), s_eos_sys_tick_timermanager);
    }

    if(0 != cbkmancfg->queuesize)
    { 
        eos_callbackman_Initialise(cbkmancfg);
    }

    

    
    
    // init hal using external references
    s_eos_the_system.syscfg->hal_fns.hal_base_init();
    s_eos_the_system.syscfg->hal_fns.hal_sys_systeminit();

    // init fsal using external references
    if(NULL != s_eos_the_system.syscfg->fsal_fns.fsal_init)
    {
        s_eos_the_system.syscfg->fsal_fns.fsal_init();  
    }



    return(&s_eos_the_system);
}



extern EOStheSystem* eos_sys_GetHandle(void)
{
    if(NULL == s_eos_the_system.thevsys)
    {
        return(NULL);
    }

    return(&s_eos_the_system);
}    


extern void eos_sys_Start(EOStheSystem *p, eOvoid_fp_void_t userinit_fn)
{
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != p), s_eobj_ownname, "eos_sys_Start() uses a NULL handle");

    s_eos_sys_start(userinit_fn);
} 


extern eOabstime_t eos_sys_LifeTimeGet(EOStheSystem *p)
{
    return(s_eos_sys_timelifeget());
}

extern eOresult_t eos_sys_LifeTimeSet(EOStheSystem *p, eOabstime_t ltime)
{
    s_eos_sys_timelifeset(ltime);
    return(eores_OK);
}

extern eOresult_t eos_sys_NanoTimeGet(EOStheSystem *p, eOnanotime_t *nt)
{
    if(NULL == nt)
    {   
        return(eores_NOK_nullpointer);
    }
    
    *nt = s_eos_sys_nanotimeget();
    return(eores_OK);
}


//extern eOresult_t eos_sys_SetEvent(EOStheSystem *p, eOevent_t evt)
//{
//    return(s_eos_sys_isr_set_evt(s_eos_the_system.thefoop, evt));
//} 
//
//extern eOresult_t eos_sys_SendMessage(EOStheSystem *p, eOmessage_t msg, eOreltime_t dummytout)
//{
//    return(s_eos_sys_isr_send_msg(s_eos_the_system.thefoop, msg));
//}
//
//extern eOresult_t eos_sys_CallbackRequest(EOStheSystem *p, eOcallback_t cbk, eOreltime_t dummytout)
//{
//    return(s_eos_sys_isr_request_cbk(s_eos_the_system.thefoop, cbk));
//}

 





// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


extern uint64_t eos_sys_hid_tickoflifeget(void)
{
    return(s_eos_the_system.tickoflife);
}


extern uint64_t eos_sys_hid_tickperiodget(void)
{
    return(s_eos_the_system.syscfg->userdef.systickperiod);
}
// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static EOVtaskDerived * s_eos_sys_gettask(void)
{
    return(&s_eos_the_system.thefoop);
}

static eOabstime_t s_eos_sys_timelifeget(void)
{
    return(s_eos_the_system.tickoflife * s_eos_the_system.syscfg->userdef.systickperiod);
}

static void s_eos_sys_timelifeset(eOabstime_t ltime)
{
    EOStheTimerManager* tmrman = eos_timerman_GetHandle();
    uint64_t oldtick = s_eos_the_system.tickoflife;
    uint64_t newtick = ltime / (s_eos_the_system.syscfg->userdef.systickperiod);

    if(oldtick == newtick)
    {
        return;
    }


    s_eos_the_system.syscfg->hal_fns.hal_sys_criticalsection_take(NULL, 0);

    // set the new tickoflife 
    s_eos_the_system.tickoflife = newtick;

    // synchronise the timer manager
    if(NULL != tmrman)
    {
        eos_hid_timerman_Synch(tmrman, oldtick, newtick);
    }

    s_eos_the_system.syscfg->hal_fns.hal_sys_criticalsection_release(NULL);
}

static uint64_t s_eos_sys_nanotimeget(void)
{
    uint64_t timeoflife = s_eos_the_system.tickoflife * s_eos_the_system.syscfg->userdef.systickperiod;
//    nt->secs = timeoflife / 1000000;
//    nt->nano = 1000*(timeoflife % 1000000);
    return(1000*timeoflife);
}


static eOresult_t s_eos_sys_start(void (*init_fn)(void))
{
    // exec the init belonging to the system object: do it before any tick is started
    if(NULL != init_fn)
    {
        init_fn();
    }

    // set the handler, thus start the ticks
    s_eos_the_system.syscfg->hal_fns.hal_sys_systick_sethandler(s_eos_sys_tick, 
                                                                s_eos_the_system.syscfg->userdef.systickperiod,
                                                                s_eos_the_system.syscfg->userdef.systickpriority);


    // exec the foop on-start-up
    eov_task_hid_StartUp(s_eos_the_system.thefoop, 0);
    
    // exec the foop run
    eov_task_hid_Run(s_eos_the_system.thefoop, 0);
 
    return(eores_OK);
}

// executed inside the sys-tick-handler
static void s_eos_sys_tick(void)
{
    
    s_eos_the_system.tickoflife ++;

    // tick the foop
    eos_foop_hid_Tick(NULL); // can use it to speed up things
    
    // execute user-defined function on tick
    if(NULL != s_eos_the_system.syscfg->userdef.on_systick)
    {
        s_eos_the_system.syscfg->userdef.on_systick();
    }

}



static void s_eos_sys_tick_timermanager(void)
{
    eos_timerman_Tick(eos_timerman_GetHandle());
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




