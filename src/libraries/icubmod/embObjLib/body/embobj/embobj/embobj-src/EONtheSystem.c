
// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"


#include "EOtheErrorManager.h"
#include "EOVtheSystem_hid.h" 



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EONtheSystem.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EONtheSystem_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------


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


static eOresult_t s_eon_sys_start(void (*init_fn)(void));
static EOVtaskDerived * s_eon_sys_gettask(void);

static eOabstime_t s_eon_sys_timelifeget(void);
static void s_eon_sys_timelifeset(eOabstime_t ltime);
static eOnanotime_t s_eon_sys_nanotimeget(void);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


static const char s_eobj_ownname[] = "EONtheSystem";

static EONtheSystem s_eos_the_system = 
{
    EO_INIT(.thevsys)       NULL,
    EO_INIT(.lifetime)      0
};

extern const eOnsystem_cfg_t eon_system_cfg_default = {0}; 

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern EONtheSystem * eon_sys_Initialise(const eOnsystem_cfg_t *syscfg,
                                         const eOmempool_cfg_t *mpoolcfg, 
                                         const eOerrman_cfg_t *errmancfg)
{

    if(NULL != s_eos_the_system.thevsys) 
    {
        // already initialised
        return(&s_eos_the_system);
    }
    
    if(NULL == syscfg)
    {
        syscfg = &eon_system_cfg_default;
    }
    
    
    // mpoolcfg can be NULL: in such a case we use eo_mempool_alloc_dynamic mode
    // errmancfg can be NULL

    // mempool and error manager initialised inside here.
    s_eos_the_system.thevsys = eov_sys_hid_Initialise(  mpoolcfg,
                                                        errmancfg,
                                                        s_eon_sys_start, 
                                                        s_eon_sys_gettask, 
                                                        s_eon_sys_timelifeget, 
                                                        s_eon_sys_timelifeset, 
                                                        s_eon_sys_nanotimeget,
                                                        NULL
                                                     );
                                                  


    // and reset the lifetime
    s_eos_the_system.lifetime = 0;


    return(&s_eos_the_system);
}



extern EONtheSystem* eon_sys_GetHandle(void)
{
    if(NULL == s_eos_the_system.thevsys)
    {
        return(NULL);
    }

    return(&s_eos_the_system);
}    


extern void eon_sys_Start(EONtheSystem *p, eOvoid_fp_void_t userinit_fn)
{
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != p), s_eobj_ownname, "eon_sys_Start() uses a NULL handle");

    s_eon_sys_start(userinit_fn);
} 


extern eOabstime_t eon_sys_LifeTimeGet(EONtheSystem *p)
{
    return(s_eon_sys_timelifeget());
}

extern eOresult_t eon_sys_LifeTimeSet(EONtheSystem *p, eOabstime_t ltime)
{
    s_eon_sys_timelifeset(ltime);
    return(eores_OK);
}

extern eOresult_t eon_sys_NanoTimeGet(EONtheSystem *p, eOnanotime_t *nt)
{
    if(NULL == nt)
    {   
        return(eores_NOK_nullpointer);
    }
    
    *nt  = s_eon_sys_nanotimeget();
    return(eores_OK);
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


extern uint64_t eon_sys_hid_tickoflifeget(void)
{
    return(0);
}


extern uint64_t eon_sys_hid_tickperiodget(void)
{
    return(0);
}
// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static EOVtaskDerived * s_eon_sys_gettask(void)
{
    return(NULL);
}

static eOabstime_t s_eon_sys_timelifeget(void)
{
    return(s_eos_the_system.lifetime);
}

static void s_eon_sys_timelifeset(eOabstime_t ltime)
{
    s_eos_the_system.lifetime = ltime;
}

static eOnanotime_t s_eon_sys_nanotimeget(void)
{
    return(1000*s_eos_the_system.lifetime);
}


static eOresult_t s_eon_sys_start(void (*init_fn)(void))
{
    // exec the init belonging to the system object: do it before any tick is started
    if(NULL != init_fn)
    {
        init_fn();
    }

 
    return(eores_OK);
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




