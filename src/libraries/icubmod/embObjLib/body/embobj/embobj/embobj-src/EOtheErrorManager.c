
// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------


#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOVtheSystem.h"
#include "EOVtask.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheErrorManager.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheErrorManager_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------

const eOerrman_cfg_t eom_errman_DefaultCfg = 
{
    EO_INIT(.extfn)
    { 
        EO_INIT(.usr_on_error)      NULL
    }
};


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_eo_errman_OnError(eOerrmanErrorType_t errtype, eOid08_t taskid, const char *eobjstr, const char *info);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

//static const char s_eobj_ownname[] = "EOtheErrorManager";

 
static EOtheErrorManager s_errman_singleton = 
{
	EO_INIT(.cfg) 
    {   
        EO_INIT(.extfn.usr_on_error)    NULL
    }
};


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern EOtheErrorManager * eo_errman_Initialise(const eOerrman_cfg_t *errmancfg)
{
#ifndef EODEF_DONT_USE_THE_ERRORMAN
    if(NULL == errmancfg)
    {
        errmancfg = &eom_errman_DefaultCfg;    
    }

    memcpy(&s_errman_singleton.cfg, errmancfg, sizeof(eOerrman_cfg_t));

    return(&s_errman_singleton);
#else
    return(NULL);
#endif
}


extern EOtheErrorManager* eo_errman_GetHandle(void) 
{
#ifndef EODEF_DONT_USE_THE_ERRORMAN	
    return(&s_errman_singleton);
#else
    return(NULL);
#endif
}

 
extern void eo_errman_Assert(EOtheErrorManager *p, uint32_t cond, const char *eobjstr, const char *info) 
{
#ifndef EODEF_DONT_USE_THE_ERRORMAN    
    if(0 == cond) 
    {
        eo_errman_Error(p, eo_errortype_fatal, eobjstr, info);
    }

#else
    ;
#endif
}


extern void eo_errman_Error(EOtheErrorManager *p, eOerrmanErrorType_t errtype, const char *eobjstr, const char *info) 
{
#ifndef EODEF_DONT_USE_THE_ERRORMAN
    EOVtaskDerived *task = NULL;
    task = eov_sys_GetRunningTask(eov_sys_GetHandle());

    s_eo_errman_OnError(errtype, eov_task_GetID(task), eobjstr, info);
#else
    ;
#endif
}


extern void eo_errman_Info(EOtheErrorManager *p, const char *eobjstr, const char *info) 
{
#ifndef EODEF_DONT_USE_THE_ERRORMAN
    eo_errman_Error(p, eo_errortype_info, eobjstr, info);
#else
    ;
#endif
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void s_eo_errman_OnError(eOerrmanErrorType_t errtype, eOid08_t taskid, const char *eobjstr, const char *info)
{
#ifndef EODEF_DONT_USE_THE_ERRORMAN
    if(NULL != s_errman_singleton.cfg.extfn.usr_on_error)
    {
        s_errman_singleton.cfg.extfn.usr_on_error(errtype, taskid, eobjstr, info);
    }

    if(errtype <= eo_errortype_weak)
    {
        return;
    }

    eov_sys_Stop(eov_sys_GetHandle());

    for(;;);
#else
#endif
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------





