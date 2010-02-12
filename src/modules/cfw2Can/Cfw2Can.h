// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup cfw2can cfw2can
 *
 * Implements <a href="http://eris.liralab.it/yarpdoc/d3/d5b/classyarp_1_1dev_1_1ICanBus.html"> ICanBus interface <\a> for a cfw2 can bus device (cfw2 pc104 card). This is the cfw2can module
 * device.
 *
 * Copyright (C) 2010 RobotCub Consortium.
 *
 * Author: Lorenzo Natale
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/modules/cfw2can/Cfw2Can.h
 *
 */

//
// $Id: Cfw2Can.h,v 1.5 2008/06/25 22:33:53 nat Exp $
//
//

#ifndef __Cfw2Canh__
#define __Cfw2Canh__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>

namespace yarp{
    namespace dev{
        class Cfw2Can;
        class Cfw2CanMessage;
    }
}

#include "libcfw002.h"

class yarp::dev::Cfw2CanMessage:public yarp::dev::CanMessage
{
 public:
    CFWCAN_MSG *msg;

 public:
    Cfw2CanMessage()
    {
        msg=0;
    }
    
    virtual ~Cfw2CanMessage()
    {
    }

    virtual CanMessage &operator=(const CanMessage &l);

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
            msg=(CFWCAN_MSG *)(b);
    }
};

class yarp::dev::Cfw2Can: public ImplementCanBufferFactory<Cfw2CanMessage, CFWCAN_MSG>,
            public ICanBus, 
           /* public ICanBusErrors, */
            public DeviceDriver
{
private:
    CFWCAN_HANDLE *handle;
public:
    Cfw2Can();
    ~Cfw2Can();

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
    // virtual bool canGetErrors(CanErrors &errs);
    

    /*Device Driver*/
    virtual bool open(yarp::os::Searchable &par);
    virtual bool close();
};

#endif
