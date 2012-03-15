#ifndef __UPDATER_CORE_H__
#define __UPDATER_CORE_H__

#include "osal.h"
#include "app-ipnet.h"

extern void upd_core_init(osal_messagequeue_t* txpktqueue, osal_task_t* tskid_ip);
extern void upd_core_manage_cmd(datagram_t *rxdgram);

#endif
