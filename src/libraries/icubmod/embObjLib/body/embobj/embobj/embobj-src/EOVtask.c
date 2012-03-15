
/* @file       EOVtask.c
    @brief      This file implements a virtual task.
    @author     marco.accame@iit.it
    @date       08/24/2011
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOVtask.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOVtask_hid.h" 


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

static void s_eov_task_dummy(void *tsk, uint32_t v);
static eOresult_t s_eov_task_dummy_isr_set_evt (void *tsk, eOevent_t evt);
static eOresult_t s_eov_task_dummy_tsk_set_evt (void *tsk, eOevent_t evt);
static eOresult_t s_eov_task_dummy_isr_send_msg (void *tsk, eOmessage_t msg);
static eOresult_t s_eov_task_dummy_tsk_send_msg (void *tsk, eOmessage_t msg, eOreltime_t tim);
static eOresult_t s_eov_task_dummy_isr_exec_cbk (void *tsk, eOcallback_t cbk, void *arg);
static eOresult_t s_eov_task_dummy_tsk_exec_cbk (void *tsk, eOcallback_t cbk, void *arg, eOreltime_t tim);
static eOid08_t s_eov_task_dummy_get_id(void *tsk);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

//static const char s_eobj_ownname[] = "EOVtask";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern eOresult_t eov_task_isrSetEvent(EOVtaskDerived *t, eOevent_t evt) 
{
    EOVtask *task;
    eOres_fp_voidp_evt_t fptr;

    
    task = eo_common_getbaseobject(t);
	
	if(NULL == task) 
	{
		return(eores_NOK_nullpointer); 
	}

    // get set evt function
    fptr = (eOres_fp_voidp_evt_t)task->vtable[VF02_isr_set_evt]; 
	
//    if(NULL == fptr)
//	{
//		return(eores_NOK_nullpointer);     
//	}

    // call funtion of derived object. fptr is always non-NULL
    return(fptr(t, evt));
}


extern eOresult_t eov_task_tskSetEvent(EOVtaskDerived *t, eOevent_t evt) 
{
    EOVtask *task;
    eOres_fp_voidp_evt_t fptr;

    
    task = eo_common_getbaseobject(t);
	
	if(NULL == task) 
	{
		return(eores_NOK_nullpointer); 
	}

    // get set evt function
    fptr = (eOres_fp_voidp_evt_t)task->vtable[VF03_tsk_set_evt]; 
	
//    if(NULL == fptr)
//	{
//		return(eores_NOK_nullpointer);     
//	}

    // call funtion of derived object. fptr is always non-NULL
    return(fptr(t, evt));
}


extern eOresult_t eov_task_isrSendMessage(EOVtaskDerived *t, eOmessage_t msg) 
{
    EOVtask *task;
    eOres_fp_voidp_msg_t fptr;

    
    task = eo_common_getbaseobject(t);
	
	if(NULL == task) 
	{
		return(eores_NOK_nullpointer); 
	}

    // get send msg function
    fptr = (eOres_fp_voidp_msg_t)task->vtable[VF04_isr_send_msg]; 
	
//    if(NULL == fptr)
//	{
//		return(eores_NOK_nullpointer);     
//	}

    // call funtion of derived object. fptr is always non-NULL
    return(fptr(t, msg));
}


extern eOresult_t eov_task_tskSendMessage(EOVtaskDerived *t, eOmessage_t msg, eOreltime_t tout) 
{
    EOVtask *task;
    eOres_fp_voidp_msg_tim_t fptr;

    
    task = eo_common_getbaseobject(t);
	
	if(NULL == task) 
	{
		return(eores_NOK_nullpointer); 
	}

    // get send msg function
    fptr = (eOres_fp_voidp_msg_tim_t)task->vtable[VF05_tsk_send_msg]; 
	
//    if(NULL == fptr)
//	{
//		return(eores_NOK_nullpointer);     
//	}

    // call funtion of derived object. fptr is always non-NULL
    return(fptr(t, msg, tout));
}


extern eOresult_t eov_task_isrExecCallback(EOVtaskDerived *t, eOcallback_t cbk, void *arg) 
{
    EOVtask *task;
    eOres_fp_voidp_cbk_voidp_t fptr;

    
    task = eo_common_getbaseobject(t);
	
	if(NULL == task) 
	{
		return(eores_NOK_nullpointer); 
	}

    // exec callback function
    fptr = (eOres_fp_voidp_cbk_voidp_t)task->vtable[VF06_isr_exec_cbk]; 
	
//    if(NULL == fptr)
//	{
//		return(eores_NOK_nullpointer);     
//	}

    // call funtion of derived object. fptr is always non-NULL
    return(fptr(t, cbk, arg));
}


extern eOresult_t eov_task_tskExecCallback(EOVtaskDerived *t, eOcallback_t cbk, void *arg, eOreltime_t tout) 
{
    EOVtask *task;
    eOres_fp_voidp_cbk_voidp_tim_t fptr;

    
    task = eo_common_getbaseobject(t);
	
	if(NULL == task) 
	{
		return(eores_NOK_nullpointer); 
	}

    // exec callback function
    fptr = (eOres_fp_voidp_cbk_voidp_tim_t)task->vtable[VF07_tsk_exec_cbk]; 
	
//    if(NULL == fptr)
//	{
//		return(eores_NOK_nullpointer);     
//	}

    // call funtion of derived object. fptr is always non-NULL
    return(fptr(t, cbk, arg, tout));
}


extern eOid08_t eov_task_GetID(EOVtaskDerived *t) 
{
    EOVtask *task;
    eOuint8_fp_voidp_t fptr;

    
    task = eo_common_getbaseobject(t);
	
	if(NULL == task) 
	{
		return(0); 
	}

    // get function
    fptr = (eOuint8_fp_voidp_t)task->vtable[VF08_get_id]; 
	
//    if(NULL == fptr)
//	{
//		return(eores_NOK_nullpointer);     
//	}

    // call funtion of derived object. fptr is always non-NULL
    return(fptr(t));
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


extern EOVtask* eov_task_hid_New(void) 
{
	EOVtask *retptr = NULL;	

	// i get the memory for the object
	retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOVtask), 1);

	// now the obj has valid memory. i need to initialise it with user-defined data
    
    // set vtable to safe dummy values
    eov_task_hid_SetVTABLE(retptr, 
                                s_eov_task_dummy, s_eov_task_dummy,
                                s_eov_task_dummy_isr_set_evt, s_eov_task_dummy_tsk_set_evt,
                                s_eov_task_dummy_isr_send_msg, s_eov_task_dummy_tsk_send_msg,
                                s_eov_task_dummy_isr_exec_cbk, s_eov_task_dummy_tsk_exec_cbk,
                                s_eov_task_dummy_get_id
                          );

    // other stuff


	return(retptr);	
}


extern eOresult_t eov_task_hid_SetVTABLE(EOVtask *p, 
                                         eOvoid_fp_voidp_uint32_t v_startup, eOvoid_fp_voidp_uint32_t v_run,
                                         eOres_fp_voidp_evt_t v_isr_set_evt, eOres_fp_voidp_evt_t v_tsk_set_evt,
                                         eOres_fp_voidp_msg_t v_isr_send_msg, eOres_fp_voidp_msg_tim_t v_tsk_send_msg,
                                         eOres_fp_voidp_cbk_voidp_t v_isr_exec_cbk, eOres_fp_voidp_cbk_voidp_tim_t v_tsk_exec_cbk,
                                         eOuint8_fp_voidp_t v_get_id 
                                        )
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }

    p->vtable[VF00_startup]        = (NULL != v_startup) ? (v_startup) : (s_eov_task_dummy);
    p->vtable[VF01_run]            = (NULL != v_run) ? (v_run) : (s_eov_task_dummy);

    p->vtable[VF02_isr_set_evt]    = (NULL != v_isr_set_evt) ? (v_isr_set_evt) : (s_eov_task_dummy_isr_set_evt);
    p->vtable[VF03_tsk_set_evt]    = (NULL != v_tsk_set_evt) ? (v_tsk_set_evt) : (s_eov_task_dummy_tsk_set_evt);;
    p->vtable[VF04_isr_send_msg]   = (NULL != v_isr_send_msg) ? (v_isr_send_msg) : (s_eov_task_dummy_isr_send_msg);
    p->vtable[VF05_tsk_send_msg]   = (NULL != v_tsk_send_msg) ? (v_tsk_send_msg) : (s_eov_task_dummy_tsk_send_msg);
    p->vtable[VF06_isr_exec_cbk]   = (NULL != v_isr_exec_cbk) ? (v_isr_exec_cbk) : (s_eov_task_dummy_isr_exec_cbk);
    p->vtable[VF07_tsk_exec_cbk]   = (NULL != v_tsk_exec_cbk) ? (v_tsk_exec_cbk) : (s_eov_task_dummy_tsk_exec_cbk);
    p->vtable[VF08_get_id]         = (NULL != v_get_id) ? (v_get_id) : (s_eov_task_dummy_get_id);

    return(eores_OK);
}


extern void eov_task_hid_StartUp(EOVtaskDerived *t, uint32_t u)
{
    EOVtask *task;
    eOvoid_fp_voidp_uint32_t fptr;

    
    task = eo_common_getbaseobject(t);
	
	if(NULL == task) 
	{
		return; 
	}

    // get set evt function
    fptr = (eOvoid_fp_voidp_uint32_t)task->vtable[VF00_startup]; 

// never NULL. it can be dummy, however	
//    if(NULL == fptr)
//	{
//		return;     
//	}

    fptr(t, u);
}


extern void eov_task_hid_Run(EOVtaskDerived *t, uint32_t u)
{
    EOVtask *task;
    eOvoid_fp_voidp_uint32_t fptr;

    
    task = eo_common_getbaseobject(t);
	
	if(NULL == task) 
	{
		return; 
	}

    // get set evt function
    fptr = (eOvoid_fp_voidp_uint32_t)task->vtable[VF01_run]; 

// never NULL. it can be dummy, however		
//    if(NULL == fptr)
//	{
//		return(eores_NOK_nullpointer);     
//	}

    fptr(t, u);
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void s_eov_task_dummy(void *tsk, uint32_t v)
{
    tsk = tsk;
    v   = v;
}


static eOresult_t s_eov_task_dummy_isr_set_evt(void *tsk, eOevent_t evt)
{
    return(eores_NOK_generic);
}


static eOresult_t s_eov_task_dummy_tsk_set_evt(void *tsk, eOevent_t evt)
{
    return(eores_NOK_generic);
}


static eOresult_t s_eov_task_dummy_isr_send_msg(void *tsk, eOmessage_t msg)
{
    return(eores_NOK_generic);
}


static eOresult_t s_eov_task_dummy_tsk_send_msg(void *tsk, eOmessage_t msg, eOreltime_t tim)
{
    return(eores_NOK_generic);
}


static eOresult_t s_eov_task_dummy_isr_exec_cbk(void *tsk, eOcallback_t cbk, void *arg)
{
    return(eores_NOK_generic);
}


static eOresult_t s_eov_task_dummy_tsk_exec_cbk(void *tsk, eOcallback_t cbk, void *arg, eOreltime_t tim)
{
    return(eores_NOK_generic);
} 


static eOid08_t s_eov_task_dummy_get_id(void *tsk)
{
    return(0);
} 


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------





