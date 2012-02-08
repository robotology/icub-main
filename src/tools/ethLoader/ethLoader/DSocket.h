/*
 * Copyright (C) 2012 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __ETHUPDATER_DSOCKET_H__
#define __ETHUPDATER_DSOCKET_H__

#include <string>
#include <ace/SOCK_Dgram.h>

class DSocket
{
public:
     DSocket()
     { 
         ACE_OS::socket_init(2,2);
         mSocket=NULL; 
     }
    ~DSocket()
    { 
        Close(); 
        ACE_OS::socket_fini();
    }

    bool Create(ACE_UINT16 port,std::string& address)
    {
        ACE_INET_Addr ace_addr(port,address.c_str());
        mSocket=new ACE_SOCK_Dgram(ace_addr);

        return mSocket!=NULL;
    }

    bool Create(ACE_UINT16 port,ACE_UINT32 address)
    {
        ACE_INET_Addr ace_addr(port,address);
        mSocket=new ACE_SOCK_Dgram(ace_addr);

        return mSocket!=NULL;
    }

    void SendTo(void* data,size_t len,ACE_UINT16 port,std::string& address)
    {
        ACE_INET_Addr ace_addr(port,address.c_str());
        mSocket->send(data,len,ace_addr);
    }

    void SendTo(void* data,size_t len,ACE_UINT16 port,ACE_UINT32 address)
    {
        ACE_INET_Addr ace_addr(port,address);
        mSocket->send(data,len,ace_addr);
    }

    ssize_t ReceiveFrom(void* data,size_t len,std::string &address,ACE_UINT16 &port,int wait_msec)
    {
        ACE_Time_Value tv(wait_msec/1000,(wait_msec%1000)*1000);
        ACE_INET_Addr ace_addr;
        ssize_t nrec=mSocket->recv(data,len,ace_addr,0,&tv);
        address=ace_addr.get_host_addr();
        port=ace_addr.get_port_number();
        return nrec;
    }

    ssize_t ReceiveFrom(void* data,size_t len,ACE_UINT32 &address,ACE_UINT16 &port,int wait_msec)
    {
        ACE_Time_Value tv(wait_msec/1000,(wait_msec%1000)*1000);
        ACE_INET_Addr ace_addr;
        ssize_t nrec=mSocket->recv(data,len,ace_addr,0,&tv);
        address=ace_addr.get_ip_address();
        port=ace_addr.get_port_number();
        return nrec;
    }

    void Close()
    {
        if (mSocket)
        {
            mSocket->close();
            delete mSocket;
            mSocket=NULL;
        }
    }

protected:
    ACE_SOCK_Dgram* mSocket;
};

#endif
