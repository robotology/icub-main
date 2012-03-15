    // - include guard ----------------------------------------------------------------------------------------------------
#ifndef _ICUBRUNNINGMSG_MANAGER_H_
#define _ICUBRUNNINGMSG_MANAGER_H_

// - doxy begin -------------------------------------------------------------------------------------------------------
// empty-section

// - external dependencies --------------------------------------------------------------------------------------------
#include "EOcommon.h"
#include "EOMtask.h"

// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
 
// - declaration of public user-defined types ------------------------------------------------------------------------- 
// empty-section
    
// - declaration of extern public variables, ...deprecated: better using use _get/_set instead ------------------------
// empty-section

// - declaration of extern public functions ---------------------------------------------------------------------------
extern void iCubRunningMsg_manager_startup(EOMtask *p, uint32_t t);
extern void iCubRunningMsg_manager_run(EOMtask *tsk, uint32_t evtmsgper);
extern void iCubRunningMsg_manager_task(void *p);

// - doxy end ---------------------------------------------------------------------------------------------------------
// empty-section

#endif  // include-guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


