// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup socketcan socketcan
 *
 * Implements <a href="http://eris.liralab.it/yarpdoc/d3/d5b/classyarp_1_1dev_1_1ICanBus.html" ICanBus interface <\a> for a linux socketcan. 
 *
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

///
/// $Id: SocketCan.h,v 0.9 2010/12/05 18:00:00 randaz Exp $
///

#ifndef __SOCKETCANH__
#define __SOCKETCANH__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>

#include "memory.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace yarp{
    namespace dev{
        class SocketCan;
        class SocketCanMessage;
    }
}

class yarp::dev::SocketCanMessage:public yarp::dev::CanMessage
{
public:
    struct can_frame *msg;

public:
    SocketCanMessage()
    {
        msg=0;
    }

    virtual ~SocketCanMessage()
    {
    }

    virtual CanMessage &operator=(const CanMessage &l)
    {
        const SocketCanMessage &tmp=dynamic_cast<const SocketCanMessage &>(l);
        memcpy(msg, tmp.msg, sizeof(struct can_frame));
        return *this;
    }

    virtual unsigned int getId() const
    { return msg->can_id;}

    virtual unsigned char getLen() const
    { return msg->can_dlc;}

    virtual void setLen(unsigned char len)
    { msg->can_dlc=len;}

    virtual void setId(unsigned int id)
    { msg->can_id=id;}

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
            msg=(can_frame *)(b);
    }
};

class yarp::dev::SocketCan: public ImplementCanBufferFactory<SocketCanMessage, can_frame>,
    public ICanBus, 
    public DeviceDriver
{
private:
    int skt;
public:
    SocketCan();
    ~SocketCan();

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
