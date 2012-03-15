#ifndef __UPDATER_CORE_H__
#define __UPDATER_CORE_H__

#include "osal.h"
#include "stdint.h"

extern void upd_core_init(void);
extern uint8_t upd_core_manage_cmd(uint8_t *pktin, uint8_t *pktout, uint16_t *sizeout);

#endif
