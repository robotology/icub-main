// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _SYSTEMCONTROLLER_H_
#define _SYSTEMCONTROLLER_H_

// - doxy begin -------------------------------------------------------------------------------------------------------
/* @file       systemController.h
	@brief      This file provides interface to systemController task.
                It starts the system. (Here the system is meant like the set of tasks and
                objects that work together to get objectives of ems-application)
                
                The systemController task waits message from PC104 by control-socket and makes the system
                running or idle depending on received message(START_CMD or STOP_CMD respectively.)
	@author     valentina.gaggero@iit.it
    @date       12/09/2010
**/


// - external dependencies --------------------------------------------------------------------------------------------
#include "EOMtask.h"

// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
 
// - declaration of public user-defined types ------------------------------------------------------------------------- 
// empty-section
    
// - declaration of extern public variables, ...deprecated: better using use _get/_set instead ------------------------
// empty-section

// - declaration of extern public functions ---------------------------------------------------------------------------
extern void sysController_startup(EOMtask *p, uint32_t t);

extern void sysController_run(EOMtask *tsk, uint32_t evtmsgper);

extern void sysController_task(void *p);


// - doxy end ---------------------------------------------------------------------------------------------------------
// empty-section

#endif  // include-guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


