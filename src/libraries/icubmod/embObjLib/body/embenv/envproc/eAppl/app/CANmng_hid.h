// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _CANMNG_HID_H_
#define _CANMNG_HID_H_

// - doxy begin -------------------------------------------------------------------------------------------------------
//#include "stdint.h"
#include "hal.h"

// - external dependencies --------------------------------------------------------------------------------------------
// empty-section

// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
 
// - declaration of public user-defined types ------------------------------------------------------------------------- 
// empty-section
    
// - declaration of extern public variables, ...deprecated: better using use _get/_set instead ------------------------
// empty-section

// - declaration of extern public functions ---------------------------------------------------------------------------
extern void CANmng_startup(hal_can_port_t port, EOMtask *p, uint32_t *idcan);
extern void CANmng_run(hal_can_port_t port, EOMtask *tsk, eOevent_t evt, uint32_t idcan);
extern void CANmng_task(void *p);
extern void CANmng_parserAndSend_TEST(hal_can_port_t port, uint8_t *buff, uint32_t idcan);
// - doxy end ---------------------------------------------------------------------------------------------------------
// empty-section

#endif  // include-guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


