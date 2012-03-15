// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _CAN1MNG_H_
#define _CAN1MNG_H_

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
extern void can1mng_startup(EOMtask *p, uint32_t t);
extern void can1mng_run(EOMtask *tsk, uint32_t evtmsgper);
extern void can1mng_task(void *p);


// - doxy end ---------------------------------------------------------------------------------------------------------
// empty-section

#endif  // include-guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


