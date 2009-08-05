#ifndef DRIVER_H
#define DRIVER_H

#include "ntcan.h"

#ifndef NTCAN_HANDLE
#define NTCAN_HANDLE HANDLE
#endif

#define MAX_READ_MSG 64

class can_parameters_type
{
 public:
 int    p_net;
 long   p_txbufsize;
 long   p_rxbufsize;
 long   p_txtout;
 long   p_rxtout;
 int    p_baudrate;
 unsigned long p_mode;
 can_parameters_type();
};

class cDriver
{
private:
NTCAN_HANDLE m_h0;
int    m_net;
long   m_txbufsize;
long   m_rxbufsize;
long   m_txtout;
long   m_rxtout;
int    m_baudrate;
unsigned long m_mode;

public:
cDriver ();
~cDriver () {}
int init(can_parameters_type* p);
int receive_message(CMSG* messages);
int send_message(CMSG& message);
};

#endif
