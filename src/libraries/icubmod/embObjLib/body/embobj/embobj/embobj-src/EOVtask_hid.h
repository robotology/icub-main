
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOVTASK_HID_H_
#define _EOVTASK_HID_H_


/** @file       EOVtask_hid.h
    @brief      This header file implements hidden interface to a task object.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"


// - declaration of extern public interface ---------------------------------------------------------------------------

#include "EOVtask.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
#define VF00_startup                    0
#define VF01_run                        1
#define VF02_isr_set_evt                2
#define VF03_tsk_set_evt                3
#define VF04_isr_send_msg               4
#define VF05_tsk_send_msg               5
#define VF06_isr_exec_cbk               6
#define VF07_tsk_exec_cbk               7
#define VF08_get_id                     8
#define VTABLESIZE_task                 9


// - declaration of extern public interface ---------------------------------------------------------------------------
 
/** @struct     EOVtask_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/ 
 
struct EOVtask_hid 
{
    // - vtable: must be on top of the struct
    void * vtable[VTABLESIZE_task];

    // - other stuff
    // empty-section
}; 

// - declaration of extern hidden functions ---------------------------------------------------------------------------
 
/** @fn         extern EOVtask* eov_task_hid_New(void)
    @brief      Creates a new task object. 
    @return     The pointer to the required object. The pointer is guaranteed to be always valid and will 
                never be NULL, because failure is managed by the memory pool. 
 **/ 
extern EOVtask* eov_task_hid_New(void);

/** @fn         extern eOresult_t eov_task_hid_SetVTABLE(EOVtask *p, 
                                                         eOvoid_fp_voidp_uint32_t v_startup, eOvoid_fp_voidp_uint32_t v_run
                                                         eOres_fp_voidp_evt_t v_isr_set_evt, eOres_fp_voidp_evt_t v_tsk_set_evt,
                                                         eOres_fp_voidp_msg_t v_isr_send_msg, eOres_fp_voidp_msg_tim_t v_tsk_send_msg,
                                                         eOres_fp_voidp_cbk_voidp_t v_isr_exec_cbk, eOres_fp_voidp_cbk_voidp_tim_t v_tsk_exec_cbk,
                                                         eOuint8_fp_voidp_t v_get_id 
                                                        )
    @brief      Specialise the virtual functions of the abstract object
    @return     eores_OK.
 **/
extern eOresult_t eov_task_hid_SetVTABLE(EOVtask *p, 
                                         eOvoid_fp_voidp_uint32_t v_startup, eOvoid_fp_voidp_uint32_t v_run,
                                         eOres_fp_voidp_evt_t v_isr_set_evt, eOres_fp_voidp_evt_t v_tsk_set_evt,
                                         eOres_fp_voidp_msg_t v_isr_send_msg, eOres_fp_voidp_msg_tim_t v_tsk_send_msg,
                                         eOres_fp_voidp_cbk_voidp_t v_isr_exec_cbk, eOres_fp_voidp_cbk_voidp_tim_t v_tsk_exec_cbk,
                                         eOuint8_fp_voidp_t v_get_id 
                                        );

extern void eov_task_hid_StartUp(EOVtaskDerived *t, uint32_t u);

extern void eov_task_hid_Run(EOVtaskDerived *t, uint32_t u);

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




