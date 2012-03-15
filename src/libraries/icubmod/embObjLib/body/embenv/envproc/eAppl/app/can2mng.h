// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _CAN2MNG_H_
#define _CAN2MNG_H_

// - doxy begin -------------------------------------------------------------------------------------------------------
// empty-section

// - external dependencies --------------------------------------------------------------------------------------------
// empty-section

// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
 
// - declaration of public user-defined types ------------------------------------------------------------------------- 
// empty-section
    
// - declaration of extern public variables, ...deprecated: better using use _get/_set instead ------------------------
// empty-section

// - declaration of extern public functions ---------------------------------------------------------------------------
extern void can2mng_startup(EOMtask *p, uint32_t t);
extern void can2mng_run(EOMtask *tsk, uint32_t evtmsgper);
extern void can2mng_task(void *p);

// - doxy end ---------------------------------------------------------------------------------------------------------
// empty-section

#endif  // include-guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


