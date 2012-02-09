/*
* Copyright (C) 2012 RobotCub Consortium
* Author: Alessandro Scalzo alessandro.scalzo@iit.it
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#include "DSocket.h"

#include <ace/ACE.h>
#include <ace/Time_Value.h>
#include <ace/OS_NS_sys_socket.h>

DSocket::DSocket()
{ 
    ACE_OS::socket_init(2,2);
    mSocket=NULL; 
}
DSocket::~DSocket()
{ 
    Close(); 
    ACE_OS::socket_fini();
}

bool DSocket::Create(ACE_UINT16 port,std::string& address)
{
    ACE_INET_Addr ace_addr(port,address.c_str());
    mSocket=new ACE_SOCK_Dgram_Bcast(ace_addr);

    return mSocket!=NULL;
}

bool DSocket::Create(ACE_UINT16 port,ACE_UINT32 address)
{
    ACE_INET_Addr ace_addr(port,address);
    mSocket=new ACE_SOCK_Dgram_Bcast(ace_addr);

    return mSocket!=NULL;
}

void DSocket::SendTo(void* data,size_t len,ACE_UINT16 port,std::string& address)
{
    ACE_INET_Addr ace_addr(port,address.c_str());
    mSocket->send(data,len,ace_addr);
}

void DSocket::SendTo(void* data,size_t len,ACE_UINT16 port,ACE_UINT32 address)
{
    ACE_INET_Addr ace_addr(port,address);
    mSocket->send(data,len,ace_addr);
}

void DSocket::SendBroad(void* data,size_t len,ACE_UINT16 port)
{
    mSocket->send(data,len,port);
}

ssize_t DSocket::ReceiveFrom(void* data,size_t len,std::string &address,ACE_UINT16 &port,int wait_msec)
{
    ACE_Time_Value tv(wait_msec/1000,(wait_msec%1000)*1000);
    ACE_INET_Addr ace_addr;
    ssize_t nrec=mSocket->recv(data,len,ace_addr,0,&tv);
    address=ace_addr.get_host_addr();
    port=ace_addr.get_port_number();
    return nrec;
}

ssize_t DSocket::ReceiveFrom(void* data,size_t len,ACE_UINT32 &address,ACE_UINT16 &port,int wait_msec)
{
    ACE_Time_Value tv(wait_msec/1000,(wait_msec%1000)*1000);
    ACE_INET_Addr ace_addr;
    ssize_t nrec=mSocket->recv(data,len,ace_addr,0,&tv);
    address=ace_addr.get_ip_address();
    port=ace_addr.get_port_number();
    return nrec;
}

void DSocket::Close()
{
    if (mSocket)
    {
        mSocket->close();
        delete mSocket;
        mSocket=NULL;
    }
}

