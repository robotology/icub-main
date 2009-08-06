/**
 * @ingroup icub_hardware_modules 
 * \defgroup fakecan fakecan
 *
 * Implements ICanBus interface for a software (fake) can bus board.
 * It accepts the same parameter file that can be passed to the real hw 
 * (you just need to replace ecan or pcan with fakecan). Useful for debugging 
 * robot code in absence of real hw.
 * 
 * The behavior of the fake boards is very simplified, this module 
 * is not simulating a real robot. 
 *
 * Copyright (C) 2008 RobotCub Consortium.
 *
 * Author: Lorenzo Natale
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __FAKECAN__
#define __FAKECAN__

#include "fbCanBusMessage.h"
#include "fakeBoard.h"
#include "msgList.h"

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>

#include <memory.h>
#include <list>

namespace yarp
{
    namespace dev
    {
        class FakeCanMessage;
        class FakeCan;
    }
}

class yarp::dev::FakeCanMessage:public yarp::dev::CanMessage
{
public:
    FCMSG *msg;

public:
    FakeCanMessage()
    {
        msg=0;
    }

    virtual ~FakeCanMessage()
    {
    }

    virtual CanMessage &operator=(const CanMessage &l)
    {
        const FakeCanMessage &tmp=dynamic_cast<const FakeCanMessage &>(l);
        memcpy(msg, tmp.msg, sizeof(FCMSG));
        return *this;
    }

    virtual unsigned int getId() const
    { return msg->id;}

    virtual unsigned char getLen() const
    { return msg->len;}

    virtual void setLen(unsigned char len)
    { msg->len=len;}

    virtual void setId(unsigned int id)
    { msg->id=id;}

    virtual const unsigned char *getData() const
    { return msg->data; }

    virtual unsigned char *getData()
    { return msg->data; }

    virtual unsigned char *getPointer()
    { return (unsigned char *) msg; }

    virtual const unsigned char *getPointer() const
    { return (const unsigned char *) msg; }

    virtual void setBuffer(unsigned char *b)
    { 
        if (b!=0)
            msg=(FCMSG *)(b);
    }
};

class Boards: public std::list<FakeBoard *>
{};

typedef std::list<FakeBoard *>::iterator BoardsIt;
typedef std::list<FakeBoard *>::const_iterator BoardsConstIt;

class yarp::dev::FakeCan: public ImplementCanBufferFactory<FakeCanMessage, FCMSG>,
    public ICanBus, 
    public DeviceDriver
{
private:
    Boards boardList;
    MsgList replies;
public:
    FakeCan();
    ~FakeCan();

    /* ICanBus */
    virtual bool canSetBaudRate(unsigned int rate);
    virtual bool canGetBaudRate(unsigned int *rate);
    virtual bool canIdAdd(unsigned int id);
    virtual bool canIdDelete(unsigned int id);

    virtual bool canRead(CanBuffer &msgs, 
        unsigned int size, 
        unsigned int *read,
        bool wait=false);

    virtual bool canWrite(const CanBuffer &msgs,
        unsigned int size,
        unsigned int *sent,
        bool wait=false);

    /*Device Driver*/
    virtual bool open(yarp::os::Searchable &par);
    virtual bool close();

};

#endif
