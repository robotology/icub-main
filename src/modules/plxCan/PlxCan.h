// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup pcan pcan
 *
 * Implements ICanBus interface for a "plx based"
 * can bus device (cfw pc104 card). This is the pcan
 * device.
 *
 * Copyright (C) 2008 RobotCub Consortium.
 *
 * Author: Lorenzo Natale
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/modules/plxcan/PlxCan.h
 *
 */

//
// $Id: PlxCan.h,v 1.5 2008/06/25 22:33:53 nat Exp $
//
//

#ifndef __PlxCanh__
#define __PlxCanh__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>

namespace yarp{
    namespace dev{
        class PlxCan;
        class PlxCanMessage;
    }
}

//struct __PLXCAN_HANDLE; //fw decl
//typedef struct __PLXCAN_HANDLE PLXCAN_HANDLE;
#include "PexApi.h"

class yarp::dev::PlxCanMessage:public yarp::dev::CanMessage
{
 public:
    PLXCAN_MSG *msg;

 public:
    PlxCanMessage()
    {
        msg=0;
    }
    
    virtual ~PlxCanMessage()
    {
    }

    virtual CanMessage &operator=(const CanMessage &l)
    {
        const PlxCanMessage &tmp=dynamic_cast<const PlxCanMessage &>(l);
        memcpy(msg, tmp.msg, sizeof(PLXCAN_MSG));
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
            msg=(PLXCAN_MSG *)(b);
    }
};

class yarp::dev::PlxCan: public ImplementCanBufferFactory<PlxCanMessage, PLXCAN_MSG>,
            public ICanBus, 
            public ICanBusErrors,
            public DeviceDriver
{
private:
    PLXCAN_HANDLE *handle;
public:
    PlxCan();
    ~PlxCan();

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

    /*ICanBusErrors*/
    virtual bool canGetErrors(CanErrors &errs);

    /*Device Driver*/
    virtual bool open(yarp::os::Searchable &par);
    virtual bool close();

#if 0
    virtual CanBuffer createBuffer(int elem)
    {
        return CanBufferFactoryImpl<PlxCanMessage, PLXCAN_MSG>::createBuffer(elem);
    }

    virtual void destroyBuffer(CanBuffer &buffer)
    {

        CanBufferFactoryImpl<PlxCanMessage, PLXCAN_MSG>::destroyBuffer(buffer);
    }
#endif
};

#endif
