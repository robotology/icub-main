

// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _APPIPNET_H_
#define _APPIPNET_H_

#include "osal.h"
#include "ipal.h"

typedef struct
{
    ipal_ipv4addr_t     ipaddr;
    ipal_port_t         port;
    uint16_t            size;
} datagram_header_t;

typedef struct
{
    datagram_header_t   head;
    uint8_t             data[1];
} datagram_t;

typedef struct
{
    osal_mutex_t    *mtx;
    uint8_t         first;
    uint8_t         last;
    uint8_t         size;
    datagram_t      q[8];
} datagram_fifo_t;

extern void appinet_start_ipal(void);

extern void appipnet_task_launcher(void *p);

extern void appipnet_task_wakeup(void);

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




