// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
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
     
    /**
      * This operator is defined in a general way, but it will crash if you 
      * try to assign to a Cfw2CanMessage a CanMessage of a different type. 
      * For more information, check https://github.com/robotology/icub-main/pull/82
      */
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

/**
*  @ingroup icub_hardware_modules
*  @brief `cfw2can` : driver implementing the yarp::dev::ICanBus interface for a cfw2 can bus device (cfw2 pc104 card).
*
* | YARP device name |
* |:-----------------:|
* | `cfw2can` |
*/
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
