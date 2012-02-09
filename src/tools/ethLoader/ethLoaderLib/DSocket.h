/*
 * Copyright (C) 2012 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __ETHUPDATER_DSOCKET_H__
#define __ETHUPDATER_DSOCKET_H__

#include <string>

#include <ace/ACE.h>
#include <ace/SOCK_Dgram.h>

class DSocket
{
public:
     DSocket();
    ~DSocket();

    bool Create(ACE_UINT16 port,std::string& address);
    bool Create(ACE_UINT16 port,ACE_UINT32 address);

    void SendTo(void* data,size_t len,ACE_UINT16 port,std::string& address);
    void SendTo(void* data,size_t len,ACE_UINT16 port,ACE_UINT32 address);

    ssize_t ReceiveFrom(void* data,size_t len,std::string &address,ACE_UINT16 &port,int wait_msec);
    ssize_t ReceiveFrom(void* data,size_t len,ACE_UINT32 &address,ACE_UINT16 &port,int wait_msec);

    void Close();

protected:
    ACE_SOCK_Dgram* mSocket;
};

#endif
