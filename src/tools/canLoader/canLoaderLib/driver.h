
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Marco Maggiali, Marco Randazzo, Alessandro Scalzo, Marco Accame
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef DRIVER_H
#define DRIVER_H



#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <ace/ACE.h>
#include <ace/SOCK_Dgram_Bcast.h>
#include <ace/Time_Value.h>
#include <ace/OS_NS_sys_socket.h>

#include "stdint.h"

#include <cstring>
#include <vector>
using namespace std;


// if defined:      it keeps the definition of the legacy driver for backward compatibility use (and tests).
// if undefined:    it removes the code from compilation
#define DRIVER_KEEP_LEGACY_IDRIVER


#define MAX_READ_MSG 64
#define MAX_WRITE_MSG 8

// - class CanPacket contains a CAN packet as yarp::dev::CanMessage does but with additional info of the canbus.
//   it is used by all interfaces. as such it needs conversion capabilities to/from formats which are specific
//   of the interface.

class CanPacket
{

public:
    enum { everyCANbus = 255 };

private:

    // the complete definition of a can packet
    typedef struct
    {
        int canbus;
        int id;
        int len;
        unsigned char data[8];
    } CANpkt_t;

    CANpkt_t canpkt;

public:
    CanPacket(){ canpkt.canbus = 0; canpkt.id = 0; canpkt.len = 0; memset(canpkt.data, 0, sizeof(canpkt.data)); }
    ~CanPacket(){}


    CanPacket operator = (const yarp::dev::CanMessage &canmsg)
    {
        CanPacket pkt;
        pkt.canpkt.id = canmsg.getId();
        pkt.canpkt.len = canmsg.getLen();
        memcpy(pkt.canpkt.data, canmsg.getData(), sizeof(pkt.canpkt.data));

        return pkt;
    }

    unsigned int getId() const { return canpkt.id; }
    unsigned char getLen() const { return canpkt.len; }
    void setLen(unsigned char len) { canpkt.len=len; }
    void setId(unsigned int id){ canpkt.id=id; }
    const unsigned char *getData() const { return canpkt.data; }
    unsigned char *getData(){ return canpkt.data; }
    int getCanBus() const { return canpkt.canbus; }
    void setCanBus(unsigned int bus){ canpkt.canbus=bus; }
};


// - class iDriver2 offers interface to sending and receiving CAN packets over many media.
//   we use vector<CanPacket> ...
class iDriver2
{
public:
    typedef enum { can_driver2 = 0, eth_driver2 = 1 } iDriver2Type;
public:
    virtual ~iDriver2(){}
    virtual int init(yarp::os::Searchable &config, bool verbose = true)=0;
    virtual int uninit()=0;
    // using legacy rule: canpackets must be of size able to keep howMany messages. however, i put inside the vector up to howmany. returns the numer of read.
    // much better: howMany keeps the max messages and canpackets can be resized with the read ones.
    virtual int receive_message(vector<CanPacket> &canpackets, int howMany = MAX_READ_MSG, double TIMEOUT = 1)=0;
    // using legacy rule: only n are sent. canpackets must be of size able to keep n messages. returns number of sent
    // much better: remove n and use canpackets.size() instead.
    virtual int send_message(vector<CanPacket> &canpackets, int n)=0;
    virtual iDriver2Type type()=0;
};


// - class cDriver2 is the specific class for sending CAN packet with YARP.
//
class cDriver2 : public iDriver2
{
public:
    cDriver2();
    ~cDriver2(){}
    int init(yarp::os::Searchable &config, bool verbose = true);
    int uninit();
    int receive_message(vector<CanPacket> &canpackets, int howMany = MAX_READ_MSG, double TIMEOUT = 1);
    int send_message(vector<CanPacket> &canpackets, int n);
    iDriver2Type type() { return can_driver2; }

private:
    yarp::dev::PolyDriver dd;
    yarp::dev::ICanBus *iCanBus;
    yarp::dev::ICanBufferFactory *iFactory;

    yarp::dev::CanBuffer canTXbuffer;
    yarp::dev::CanBuffer canRXbuffer;

    bool _verbose;

private:
    // used to create buffers for tx/rx with YARP
    yarp::dev::CanBuffer createCanBuffer(int m);
    void destroyCanBuffer(yarp::dev::CanBuffer &buff);
};


// - class eDriver2 is the specific class for sending CAN packets over UDP
//

class CanSocket;

class eDriver2 : public iDriver2
{
public:
    eDriver2();
    ~eDriver2();
    int init(yarp::os::Searchable &config, bool verbose = true);
    int uninit();
    int receive_message(vector<CanPacket> &canpackets, int howMany = MAX_READ_MSG, double TIMEOUT = 1);
    int send_message(vector<CanPacket> &canpackets, int n);
    iDriver2Type type() { return eth_driver2; }

    bool set_verbose(bool v);
private:
    CanSocket *mSocket;
    ACE_UINT32 mBoardAddr;
    double timestart;
    bool _verbose;
};


// - definition of the UDP packet used to transport CAN frames to/from the eUpdater (see specs in TSD-ICUBUNIT-arm-system-behaviour.docx)

// marco.accame
// actually the use of a progressive id is not in the specs (but does not harm).
// the data structure used by the eUpdater is defined in eupdater_cangtw.h of icub-firmware. TODO: move it to icub-firmware-shared

#define USE_PROG_ID

struct CanPktHeader_t
{
    unsigned char signature;
    unsigned char canFrameNumOf;
#ifdef USE_PROG_ID
    ACE_UINT8         dummy[2];
    ACE_UINT32        progressive;
#else
    ACE_UINT8         dummy[6];
#endif
};

struct CanPktFrame_t
{
    unsigned char  canBus;
    unsigned char  len;
    unsigned short canId;
    unsigned char  dummy[4];
    unsigned char  data[8];
};

struct CanPkt_t
{
    CanPktHeader_t header;
    CanPktFrame_t frames[1];
};


// - class CanSocket
//   it is used to transmit / receive a UDP packet for communication with the eUpdater

class CanSocket
{
public:
     CanSocket()
    {
        ACE_OS::socket_init(2,2);
        mSocket=NULL;
    }

    ~CanSocket()
    {
        close();
        ACE_OS::socket_fini();
    }

    bool create(ACE_UINT16 port,ACE_UINT32 address)
    {
        mSocket=new ACE_SOCK_Dgram_Bcast(ACE_INET_Addr(port,address));
        return mSocket!=NULL;
    }

    void sendTo(void* data,size_t len, ACE_UINT16 port, ACE_UINT32 address)
    {
        // uncomment until yDebug() for printing
        //CanPkt_t * pkt = (CanPkt_t*) data;
        //uint8_t * dd = (uint8_t*)data;
        //yDebug ("sending UDP pkt of %d bytes to port %d adr %d with data[0] = %d and opc = %x\n", (int) len, port, address, dd[0], pkt->frames[0].data[0]);
        mSocket->send(data,len,ACE_INET_Addr(port,address));
    }

    ssize_t receiveFrom(void* data, size_t len, ACE_UINT32 &address, ACE_UINT16 &port, int wait_msec)
    {
        ACE_Time_Value tv(wait_msec/1000,(wait_msec%1000)*1000);
        ACE_INET_Addr ace_addr;
        ssize_t nrec=mSocket->recv(data,len,ace_addr,0,&tv);

        if (nrec>0)
        {
            // uncomment until yDebug() for printing
            //CanPkt_t * pkt = (CanPkt_t*) data;
            //yDebug ("received pkt w/ %d bytes. the pkt has opc = %x\n", (int) nrec, pkt->frames[0].data[0]);
            address=ace_addr.get_ip_address();
            port=ace_addr.get_port_number();
        }
        else
        {
            address=0;
            port=0;
        }

        return nrec;
    }

    void close()
    {
        if (mSocket)
        {
            mSocket->close();
            delete mSocket;
            mSocket=NULL;
        }
    }

protected:
    ACE_SOCK_Dgram_Bcast* mSocket;
};



// - old classes cDriver and iDriver, kept for backward compatibility in case someone needs them.
//   canLoader and canLoader-console dont use them

#if     defined(DRIVER_KEEP_LEGACY_IDRIVER)

class iDriver
{
public:
    virtual ~iDriver(){}
    virtual int init(yarp::os::Searchable &config)=0;
    virtual int uninit()=0;
    virtual int receive_message(yarp::dev::CanBuffer &messages, int howMany = MAX_READ_MSG, double TIMEOUT = 1)=0;
    virtual int send_message(yarp::dev::CanBuffer &message, int n)=0;

    virtual yarp::dev::CanBuffer createBuffer(int m)=0;
    virtual void destroyBuffer(yarp::dev::CanBuffer &buff)=0;
};


class cDriver : public iDriver
{
private:
    yarp::dev::PolyDriver dd;
    yarp::dev::ICanBus *iCanBus;
    yarp::dev::ICanBufferFactory *iFactory;
public:
    cDriver();
    ~cDriver(){}
    int init(yarp::os::Searchable &config);
    int uninit();
    int receive_message(yarp::dev::CanBuffer &messages, int howMany = MAX_READ_MSG, double TIMEOUT = 1);
    int send_message(yarp::dev::CanBuffer &message, int n);

    yarp::dev::CanBuffer createBuffer(int m);
    void destroyBuffer(yarp::dev::CanBuffer &buff);
};


struct ECMSG
{
    unsigned int canbus;
    int id;
    unsigned char data[8];
    int len;
};

class EthCanMessage : public yarp::dev::CanMessage
{
public:
    ECMSG *msg;

public:
    EthCanMessage(){ msg=0; }
    virtual ~EthCanMessage(){}

    virtual CanMessage &operator=(const CanMessage &l)
    {
        const EthCanMessage &tmp=dynamic_cast<const EthCanMessage &>(l);
        memcpy(msg, tmp.msg, sizeof(ECMSG));
        return *this;
    }

    virtual unsigned int getId() const { return msg->id; }
    virtual unsigned char getLen() const { return msg->len; }
    virtual void setLen(unsigned char len) { msg->len=len; }
    virtual void setId(unsigned int id){ msg->id=id; }
    virtual const unsigned char *getData() const { return msg->data; }
    virtual unsigned char *getData(){ return msg->data; }
    virtual unsigned char *getPointer(){ return (unsigned char *) msg; }
    virtual const unsigned char *getPointer() const { return (const unsigned char *) msg; }
    virtual void setBuffer(unsigned char *b){ if (b) msg=(ECMSG *)(b); }
    unsigned int getCanBus() const { return msg->canbus; }
    void setCanBus(unsigned int bus){ msg->canbus=bus; }
};



class eDriver : public yarp::dev::ImplementCanBufferFactory<EthCanMessage,ECMSG>, public iDriver
{
private:
    CanSocket mSocket;

    char mCanBusId;
    ACE_UINT32 mBoardAddr;

    double timestart;

    yarp::dev::ICanBufferFactory *iFactory;
public:
    eDriver(){}
   ~eDriver(){}

    int init(yarp::os::Searchable &config);
    int uninit();
    int receive_message(yarp::dev::CanBuffer &messages, int howMany = MAX_READ_MSG, double TIMEOUT = 1.0);
    int send_message(yarp::dev::CanBuffer &message, int n);
    yarp::dev::CanBuffer createBuffer(int m);
    void destroyBuffer(yarp::dev::CanBuffer &buff);
};

#endif//defined(KEEP_LEGACY_DRIVER)

#endif
