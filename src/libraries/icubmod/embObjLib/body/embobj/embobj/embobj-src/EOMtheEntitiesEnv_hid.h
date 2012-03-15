// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMTHEENTITIESENV_HID_H_
#define _EOMTHEENTITIESENV_HID_H_

// - doxy -------------------------------------------------------------------------------------------------------------

/* @file       EOMtheEntitiesEnv_hid.h
    @brief      This header file implements hidden interface to entity environment.
    @author     valentina.gaggero@iit.it
    @date       12/11/2009
 **/


// - external dependencies --------------------------------------------------------------------------------------------
#include "EoCommon.h"
#include "EOVmutex.h"
#include "EOvectorMutex.h"
#include "EOaction.h"
#include "EOtimer.h"
#include "EOMtask.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
#include "EOMtheEntitiesEnv.h"


// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     Object_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOMtheEntitiesEnv_hid
{
//    EOMtask                 *sysController_ptr;     /**< pointer to singleto task, the manager of this object.*/
    EOvectorMutex           *task_list;             /**< pointer to task list */
    EOvectorMutex           *eotimer_list;          /**< pointer to eotimer list */
    EOvectorMutex           *haltimer_list;         /**< pointer to hal-timer list */
    EOvectorMutex           *module_list;           /**< pointer to module list */
    EOMtheEntitiesEnv_systemControllercfg_t sysController_stuff;
//    void (*callback_allEntities_registered)(void);
//    void (*callback_etity_regitered)(EOMtheEntitiesEnv_type_t type, void *arg);
};


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



