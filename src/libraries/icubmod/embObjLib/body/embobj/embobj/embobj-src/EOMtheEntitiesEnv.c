// --------------------------------------------------------------------------------------------------------------------
// - doxy
// --------------------------------------------------------------------------------------------------------------------

/* @file       EOMentity_env.c
    @brief      This file contains internal implementation for the utiliteis a SW entity.
    @author     valentina.gaggero@iit.it
    @date       06/12/20011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------
#include "stdlib.h"     // to see NULL, calloc etc.
#include "string.h"
#include "EOMmutex.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------
#include "EOMtheEntitiesEnv.h"
#include "EOtheErrorManager.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
#include "EOMtheEntitiesEnv_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
#define START_CMD   0
#define STOP_CMD    1


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables. deprecated: better using _get(), _set() on static variables 
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
static eOresult_t s_eom_entitiesEnv_sendCmd_all_task(EOMtheEntitiesEnv *handler, uint8_t cmd);
static eOresult_t s_eom_entitiesEnv_start_all_eotimer(EOMtheEntitiesEnv *handler);
static eOresult_t s_eom_entitiesEnv_stop_all_eotimer(EOMtheEntitiesEnv *handler);
static eOresult_t s_eom_entitiesEnv_start_all_haltimer(EOMtheEntitiesEnv *handler);
static eOresult_t s_eom_entitiesEnv_stop_all_haltimer(EOMtheEntitiesEnv *handler);
static eOresult_t s_eom_entitiesEnv_sendCmd_all_module(EOMtheEntitiesEnv *handler, uint8_t cmd);
static void       s_eom_entitiesEnv_check_registration(EOMtheEntitiesEnv_type_t entity_task, void *arg);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
static const char s_eobj_ownname[] = "EOMtheEntitiesEnv";

static EOMtheEntitiesEnv                s_eom_entitiesEnv;
static EOMtheEntitiesEnv_signals_cfg_t  s_entities_signals;

static eObool_t entity_env_isInitted = eobool_false;


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern eOresult_t EOMtheEntitiesEnv_Initialise(EOMtheEntitiesEnv_cfg_t *env_cfg, EOMtask *sysController_task)
{

    eo_errman_Assert(eo_errman_GetHandle(), (NULL != env_cfg), s_eobj_ownname, "cfg is NULL");
    

    memset(&s_eom_entitiesEnv, 0, sizeof(EOMtheEntitiesEnv));

    if(env_cfg->task_num > 0)
    {
        s_eom_entitiesEnv.task_list = eo_vectorMutex_New(sizeof(EOMtheEntitiesEnv_task_info_t),
                                                            env_cfg->task_num,
                                                            NULL/*item_init*/,  0/*init_par*/, 
                                                            NULL/*item_copy*/, NULL/*item_clear*/,
                                                            eom_mutex_New() );    
    }

    if(env_cfg->eotimer_num > 0)
    {
        s_eom_entitiesEnv.eotimer_list = eo_vectorMutex_New(sizeof(EOMtheEntitiesEnv_eotimer_info_t),
                                                            env_cfg->eotimer_num,
                                                            NULL/*item_init*/,  0/*init_par*/, 
                                                            NULL/*item_copy*/, NULL/*item_clear*/,
                                                            eom_mutex_New() ); 
    }

    if(env_cfg->haltimer_num > 0)
    {
        s_eom_entitiesEnv.haltimer_list = eo_vectorMutex_New(sizeof(EOMtheEntitiesEnv_haltimer_info_t),
                                                            env_cfg->haltimer_num,
                                                            NULL/*item_init*/,  0/*init_par*/, 
                                                            NULL/*item_copy*/, NULL/*item_clear*/,
                                                            eom_mutex_New() ); 
    }

    if(env_cfg->module_num > 0)
    {
        s_eom_entitiesEnv.module_list = eo_vectorMutex_New(sizeof(EOMtheEntitiesEnv_module_info_t),
                                                            env_cfg->module_num,
                                                            NULL/*item_init*/,  0/*init_par*/, 
                                                            NULL/*item_copy*/, NULL/*item_clear*/,
                                                            eom_mutex_New() ); 
    }


//    if(NULL != sysController_task)
//    {
//        s_eom_entitiesEnv.sysController_ptr = sysController_task;
//    }
    
    memcpy(&s_entities_signals, &env_cfg->signals, sizeof(EOMtheEntitiesEnv_signals_cfg_t));
    memcpy(&s_eom_entitiesEnv.sysController_stuff, &env_cfg->syscntl_cfg, sizeof(EOMtheEntitiesEnv_systemControllercfg_t));

//    s_eom_entitiesEnv.callback_allEntities_registered = env_cfg->callback_allEntities_registered;
//    s_eom_entitiesEnv.callback_etity_regitered = env_cfg->callback_etity_regitered;    

    entity_env_isInitted = eobool_true;

    return(eores_OK);

}


extern eOresult_t EOMtheEntitiesEnv_Deinitialise(void)
{
    // cannot free a NULL
    if(eobool_false == entity_env_isInitted)
    {
        return(eores_OK); //already de-initted
    }
    
    eo_vectorMutex_Clear(s_eom_entitiesEnv.task_list, eok_reltimeINFINITE);    
    eo_vectorMutex_Clear(s_eom_entitiesEnv.eotimer_list, eok_reltimeINFINITE);
    eo_vectorMutex_Clear(s_eom_entitiesEnv.haltimer_list, eok_reltimeINFINITE);
  
    entity_env_isInitted = eobool_false;

    return(eores_OK);
}
    

extern EOMtheEntitiesEnv* EOMtheEntitiesEnv_GetHandle(void)
{
    if(eobool_false == entity_env_isInitted)
    {
        return(NULL);
    }
    else
    {
        return(&s_eom_entitiesEnv);
    }

}



extern eOresult_t EOMtheEntitiesEnv_register_sysController(EOMtheEntitiesEnv *handler, EOMtheEntitiesEnv_systemControllercfg_t *sysCntl_cfg)
{

    if((NULL == handler)|| (NULL == sysCntl_cfg))
    {
        return(eores_NOK_nullpointer);
    }   
        
//    handler->sysController_ptr = sysController_task;
    memcpy(&handler->sysController_stuff, sysCntl_cfg, sizeof(EOMtheEntitiesEnv_systemControllercfg_t));

    return(eores_OK);
    
}    


extern eOresult_t EOMtheEntitiesEnv_register_task(EOMtheEntitiesEnv *handler, EOMtheEntitiesEnv_task_info_t *taskentity_info)
{

    eOresult_t res; 

    if((NULL == handler)|| (NULL == taskentity_info))
    {
        return(eores_NOK_nullpointer);
    }

    if((eom_mtask_EventDriven != taskentity_info->type) && (eom_mtask_MessageDriven != taskentity_info->type) )
    {
        return(eores_NOK_unsupported);
    }    

    res = eo_vectorMutex_PushBack(handler->task_list, taskentity_info, eok_reltimeINFINITE);
    if(eores_OK !=res)
    {
        /*NOTE: the only error that eo_vectorMutex_PushBack can return is eores_NOK_busy(vector is full) */
        eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal , s_eobj_ownname, "I can NOT register task");
    
    }

    
    s_eom_entitiesEnv_check_registration(entity_task, taskentity_info->task_ptr);

    return(res);
}


extern eOresult_t EOMtheEntitiesEnv_register_eotimer(EOMtheEntitiesEnv *handler, EOMtheEntitiesEnv_eotimer_info_t *eotimerentity_info)
{
    eOresult_t res; 

    if((NULL == handler)|| (NULL == eotimerentity_info))
    {
        return(eores_NOK_nullpointer);
    }


    res = eo_vectorMutex_PushBack(handler->eotimer_list, eotimerentity_info, eok_reltimeINFINITE);
    if(eores_OK !=res)
    {
        /*NOTE: the only error that eo_vectorMutex_PushBack can return is eores_NOK_busy(vector is full) */
        eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal , s_eobj_ownname, "I can NOT register eotimer");
    
    }
    
    s_eom_entitiesEnv_check_registration(entity_eotimer, eotimerentity_info->timer_ptr);

    return(res);
}


extern eOresult_t EOMtheEntitiesEnv_register_module(EOMtheEntitiesEnv *handler, EOMtheEntitiesEnv_module_info_t *moduleentity_info)
{
    eOresult_t res;

    if((NULL == handler)|| (NULL == moduleentity_info))
    {
        return(eores_NOK_nullpointer);
    }

    if( (NULL == moduleentity_info->funcStartEntity) || (NULL == moduleentity_info->funcStopEntity) )
    {
        return(eores_NOK_nodata);
    }

    
    res = eo_vectorMutex_PushBack(handler->module_list, moduleentity_info, eok_reltimeINFINITE);
    if(eores_OK !=res)
    {
        /*NOTE: the only error that eo_vectorMutex_PushBack can return is eores_NOK_busy(vector is full) */
        eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal , s_eobj_ownname, "I can NOT register module");
    }
    
    s_eom_entitiesEnv_check_registration(entity_module, moduleentity_info->entity);

    return(eores_OK);


}



extern eOresult_t EOMtheEntitiesEnv_start_all(EOMtheEntitiesEnv *handler)
{
    
    eOresult_t res;

   if(NULL == handler)
    {
        return(eores_NOK_nullpointer);
    } 

    res = s_eom_entitiesEnv_sendCmd_all_task(handler, START_CMD);
    if(eores_OK != res)
    {
        return(res);
    }

    res = s_eom_entitiesEnv_start_all_eotimer(handler);
    if(eores_OK != res)
    {
        return(res);
    }

    res = s_eom_entitiesEnv_start_all_haltimer(handler);  //not implemented (return ok)
    if(eores_OK != res)
    {
        return(res);
    }


    res = s_eom_entitiesEnv_sendCmd_all_module(handler, START_CMD);
    if(eores_OK != res)
    {
        return(res);
    }


    return(eores_OK);
   
}


extern eOresult_t EOMtheEntitiesEnv_stop_all(EOMtheEntitiesEnv *handler)
{
    eOresult_t res;

   if(NULL == handler)
    {
        return(eores_NOK_nullpointer);
    } 
    
    s_eom_entitiesEnv_sendCmd_all_task(handler, STOP_CMD);
    if(eores_OK != res)
    {
        return(res);
    }

    res = s_eom_entitiesEnv_stop_all_eotimer(handler);
    if(eores_OK != res)
    {
        return(res);
    }

    s_eom_entitiesEnv_stop_all_haltimer(handler); //not implemented (return ok)
    if(eores_OK != res)
    {
        return(res);
    }
 
    res = s_eom_entitiesEnv_sendCmd_all_module(handler, STOP_CMD);
    if(eores_OK != res)
    {
        return(res);
    }

    return(eores_OK);
   
    
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
static eOresult_t s_eom_entitiesEnv_sendCmd_all_task(EOMtheEntitiesEnv *handler, uint8_t cmd)
{
    eOsizecntnr_t list_size, index;
    eOresult_t res;
    EOMtheEntitiesEnv_task_info_t *task_info_ptr;


    res = eo_vectorMutex_Size(handler->task_list, &list_size, eok_reltimeINFINITE);
    if(eores_OK != res)
    {
        return(eores_OK); //The list is empty. it is not a problem.
    }



    for(index = 0; index <list_size; index++)
    {
        eo_vectorMutex_At(handler->task_list, (eOsizecntnr_t)index, (void*)&task_info_ptr, eok_reltimeINFINITE);

        switch(task_info_ptr->type)
        {
            case eom_mtask_MessageDriven:
            {
                res = eom_task_SendMessage(task_info_ptr->task_ptr,
                                           ((START_CMD == cmd)? s_entities_signals.task.startMsg : s_entities_signals.task.stopMsg),
                                           eok_reltimeINFINITE);    
            }break;

            case eom_mtask_EventDriven:
            {
                res = eom_task_SetEvent(task_info_ptr->task_ptr, 
                                        ((START_CMD == cmd)? s_entities_signals.task.startEvt : s_entities_signals.task.stopEvt));    
            }break;
        
            default:
            {
                break; //currently only evt and msg based task can register itself
            }
        }

        if(eores_OK !=res)
        {
            eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal , s_eobj_ownname, "I can NOT start a task");
        }
          
    }
    return(eores_OK);
}


static eOresult_t s_eom_entitiesEnv_start_all_eotimer(EOMtheEntitiesEnv *handler)
{
    eOsizecntnr_t list_size, index;
    eOresult_t res;
    EOMtheEntitiesEnv_eotimer_info_t *eotimer_info_ptr;

    res = eo_vectorMutex_Size(handler->eotimer_list, &list_size, eok_reltimeINFINITE);
    if(eores_OK != res)
    {
        return(eores_OK); //The list is empty. it is not a problem.
    }

        
    for(index = 0; index <list_size; index++)
    {
        eo_vectorMutex_At(handler->eotimer_list, (eOsizecntnr_t)index, (void*)&eotimer_info_ptr, eok_reltimeINFINITE);

        res = eo_timer_Start(eotimer_info_ptr->timer_ptr, eotimer_info_ptr->startat, eotimer_info_ptr->countdown, 
                             eotimer_info_ptr->mode, eotimer_info_ptr->action);
        if(eores_OK != res)
        {
            eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal , s_eobj_ownname, "I can NOT start a task");
        }
    }
    return(eores_OK);
}


static eOresult_t s_eom_entitiesEnv_stop_all_haltimer(EOMtheEntitiesEnv *handler)
{
    //eOsizecntnr_t list_size, index;
    eOresult_t res = eores_OK ;
    
   // Not implemented
    return(res);
}

static eOresult_t s_eom_entitiesEnv_start_all_haltimer(EOMtheEntitiesEnv *handler)
{
    //eOsizecntnr_t list_size, index;
    eOresult_t res = eores_OK ;
    
   // Not implemented
    return(res);
}


static eOresult_t s_eom_entitiesEnv_sendCmd_all_module(EOMtheEntitiesEnv *handler, uint8_t cmd)
{
    eOsizecntnr_t list_size, index;
    eOresult_t res;
    EOMtheEntitiesEnv_module_info_t *module_info_ptr;


    res = eo_vectorMutex_Size(handler->module_list, &list_size, eok_reltimeINFINITE);
    if(eores_OK != res)
    {
        return(eores_OK); //The list is empty. it is not a problem.
    }



    for(index = 0; index <list_size; index++)
    {
        //here i'm sure eo_vectorMutex_At doesn't get error
        eo_vectorMutex_At(handler->module_list, (eOsizecntnr_t)index, (void*)&module_info_ptr, eok_reltimeINFINITE);
        if(START_CMD == cmd)
        {
            module_info_ptr->funcStartEntity(module_info_ptr->argStart);     
        }
        else
        {
            module_info_ptr->funcStopEntity(module_info_ptr->argStop);
        }
    }
     return(eores_OK);
}


static eOresult_t s_eom_entitiesEnv_stop_all_eotimer(EOMtheEntitiesEnv *handler)
{
    eOsizecntnr_t list_size, index;
    eOresult_t res;
    EOMtheEntitiesEnv_eotimer_info_t *eotimer_info_ptr;

    res = eo_vectorMutex_Size(handler->eotimer_list, &list_size, eok_reltimeINFINITE);
    if(eores_OK != res)
    {
        return(res); //The list is empty. it is not a problem.
    }

        
    for(index = 0; index <list_size; index++)
    {
        eo_vectorMutex_At(handler->eotimer_list, (eOsizecntnr_t)index, (void*)&eotimer_info_ptr, eok_reltimeINFINITE);

        res = eo_timer_Stop(eotimer_info_ptr->timer_ptr);
        if(eores_OK != res)
        {
            eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal , s_eobj_ownname, "I can NOT stop a timer");
        }
    }
    return(eores_OK);
}

static void s_eom_entitiesEnv_check_registration(EOMtheEntitiesEnv_type_t entity_task, void*arg)
{
    eObool_t tasklist_isFull = eobool_false;
    eObool_t eotimerlist_isFull = eobool_false;
    eObool_t haltimerlist_isFull = eobool_false;
    eObool_t modulelist_isFull = eobool_false;

    eOresult_t tasklist_res, eotimer_res, haltimer_res, module_res;
    if(NULL != s_eom_entitiesEnv.sysController_stuff.callback_etity_regitered)
    {
        s_eom_entitiesEnv.sysController_stuff.callback_etity_regitered(entity_task, arg);    
    }

    if(NULL == s_eom_entitiesEnv.sysController_stuff.callback_allEntities_registered)
    {
        return;
    }

    tasklist_res = eo_vectorMutex_Full(s_eom_entitiesEnv.task_list, &tasklist_isFull, eok_reltimeZERO);
    eotimer_res =  eo_vectorMutex_Full(s_eom_entitiesEnv.eotimer_list, &eotimerlist_isFull, eok_reltimeZERO);
    haltimer_res = eo_vectorMutex_Full(s_eom_entitiesEnv.haltimer_list, &haltimerlist_isFull, eok_reltimeZERO);
    module_res = eo_vectorMutex_Full(s_eom_entitiesEnv.module_list, &modulelist_isFull, eok_reltimeZERO);
    /* All entities are registered if for each list:
        - the list is not null but its full.
        - the list is null. (any entities must register)
    */

    if( ( ((eores_OK == tasklist_res) && tasklist_isFull    ) || (eores_NOK_nullpointer == tasklist_res) ) &&
        ( ((eores_OK == eotimer_res)  && eotimerlist_isFull ) || (eores_NOK_nullpointer == eotimer_res)  ) &&
        ( ((eores_OK == haltimer_res) && haltimerlist_isFull) || (eores_NOK_nullpointer == haltimer_res) ) &&
        ( ((eores_OK == module_res)   && modulelist_isFull  ) || (eores_NOK_nullpointer == module_res)   ) 
      )
      {
            s_eom_entitiesEnv.sysController_stuff.callback_allEntities_registered();    
      }
}

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



