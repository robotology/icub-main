/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Lorenzo Natale
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __MSGLIST__
#define __MSGLIST__

#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Semaphore.h>
#include "fbCanBusMessage.h"

#include <list>

typedef std::list<FCMSG>::iterator MsgIt;
typedef std::list<FCMSG>::const_iterator MsgConstIt;

class MsgList: public std::list<FCMSG>
{
    yarp::os::Semaphore _mutex;
public:
    void lock() {_mutex.wait(); }
    void unlock() {_mutex.post(); }
};

#endif
