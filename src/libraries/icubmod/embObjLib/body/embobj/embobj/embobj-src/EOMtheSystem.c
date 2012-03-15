
// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"

#include "osal.h"

#include "EOtheErrorManager.h"
#include "EOVtheSystem_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOMtheSystem.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOMtheSystem_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
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
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static eOresult_t s_eom_sys_start(void (*init_fn)(void));

static void s_eom_thecreation(void);

static EOVtaskDerived* s_eom_gettask(void);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


static const char s_eobj_ownname[] = "EOMtheSystem";

static EOMtheSystem s_eom_system = 
{
    .thevsys        = NULL,               // thevsys
    .osalcfg        = NULL,               // osalcfg
    .tmrmancfg      = NULL,               // tmrmancfg
    .cbkmancfg      = NULL,               // cbkmancfg
    .user_init_fn   = NULL                // user_init_fn
};

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern EOMtheSystem * eom_sys_Initialise(const eOmsystem_cfg_t *syscfg, 
                                         const eOmempool_cfg_t *mpoolcfg, 
                                         const eOerrman_cfg_t *errmancfg,
                                         const eOmtimerman_cfg_t *tmrmancfg,
                                         const eOmcallbackman_cfg_t *cbkmancfg)
{
    uint32_t ram04size = 0;
    uint32_t *ram04data = NULL;
    uint32_t ram08size = 0;
    uint64_t *ram08data = NULL;

    if(NULL != s_eom_system.thevsys) 
    {
        // already initialised
        return(&s_eom_system);
    }


    eo_errman_Assert(eo_errman_GetHandle(), (NULL != syscfg), s_eobj_ownname, "syscfg is NULL");
    // verify that we have a valid osalcfg and halcfg. fsalcfg can be NULL
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != syscfg->halcfg), s_eobj_ownname, "syscfg->halcfg is NULL");
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != syscfg->osalcfg), s_eobj_ownname, "syscfg->osalcfg is NULL");

    // mpoolcfg can be NULL: in such a case we use eo_mempool_alloc_dynamic mode
    // errmancfg can be NULL
    // tmrmancfg can be NULL: in such a case we use default values
    // cbkmancfg can be NULL: in such a case we use default values

    
    // mempool and error manager initialised inside here.
    s_eom_system.thevsys = eov_sys_hid_Initialise(mpoolcfg,
                                                  errmancfg,        // error man 
                                                  (eOres_fp_voidfpvoid_t)s_eom_sys_start, s_eom_gettask, 
                                                  osal_system_abstime_get, osal_system_abstime_set, 
                                                  (eOuint64_fp_void_t)osal_system_nanotime_get,
                                                  hal_sys_irq_disable);

    s_eom_system.osalcfg    = syscfg->osalcfg;
    s_eom_system.tmrmancfg  = tmrmancfg;
    s_eom_system.cbkmancfg  = cbkmancfg;


    // initialise hal
    hal_base_memory_getsize(syscfg->halcfg, &ram04size);
    
    if(0 != ram04size)
    {
        ram04data = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, ram04size, 1);
    }

    hal_base_initialise(syscfg->halcfg, ram04data);
    hal_sys_systeminit();

    if(0 != syscfg->codespaceoffset)
    {
        hal_sys_vectortable_relocate(syscfg->codespaceoffset);
    }


//    // initialise fsal
//    if(NULL != syscfg->fsalcfg)
//    {
//        fsal_memory_getsize(syscfg->fsalcfg, &ram04size);
//        
//        if(0 != ram04size)
//        {
//            ram04data = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, ram04size, 1);
//        }
//    
//        fsal_initialise(syscfg->fsalcfg, ram04data);
//    }

 
    // initialise osal
    osal_base_memory_getsize(s_eom_system.osalcfg, &ram08size);
    if(0 != ram08size)
    {
        ram08data = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_64bit, ram08size, 1);
    }

    osal_base_initialise(s_eom_system.osalcfg, ram08data);


    return(&s_eom_system);
   
}


extern EOMtheSystem* eom_sys_GetHandle(void)
{
    if(NULL == s_eom_system.thevsys)
    {
        return(NULL);
    }

    return(&s_eom_system);
}    


extern void eom_sys_Start(EOMtheSystem *p, eOvoid_fp_void_t userinit_fn)
{

    eo_errman_Assert(eo_errman_GetHandle(), (NULL != p), s_eobj_ownname, "eom_sys_Start() uses a NULL handle");


    s_eom_sys_start(userinit_fn);
}    



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static eOresult_t s_eom_sys_start(eOvoid_fp_void_t userinit_fn)
{
    // start osal
    s_eom_system.user_init_fn = userinit_fn;
    osal_system_start(s_eom_thecreation);
 
    return(eores_OK);
}


static void s_eom_thecreation(void)
{
    const uint16_t *myused = NULL;
    const uint16_t *myfree = NULL;
    char str[128];
    // we are in osal, now

    // start the services offered by embobj: timer manager and callback manager

    eom_timerman_Initialise(s_eom_system.tmrmancfg);

    eom_callbackman_Initialise(s_eom_system.cbkmancfg);

    osal_info_entities_get_stats(&myused, &myfree);

    snprintf(str, sizeof(str)-1, "uses %d task, %d stack, %d timers, %d mutexes, %d semaphores, %d messageboxes, %d messages", 
                                    myused[osal_info_entity_task], 
                                    myused[osal_info_entity_globalstack], 
                                    myused[osal_info_entity_timer], 
                                    myused[osal_info_entity_mutex], 
                                    myused[osal_info_entity_semaphore], 
                                    myused[osal_info_entity_messagequeue],
                                    myused[osal_info_entity_message]);

    eo_errman_Info(eo_errman_GetHandle(), s_eobj_ownname, str);
    
    // run user defined initialisation ...
    s_eom_system.user_init_fn();
}


static EOVtaskDerived* s_eom_gettask(void)
{
    osal_task_t *p;
    
    p = osal_task_get(osal_callerTSK);

    if(NULL == p)
    {
        return(NULL);
    }

    return(osal_task_extdata_get(p));
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




