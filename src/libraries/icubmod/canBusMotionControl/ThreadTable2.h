// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2008 The RobotCub Consortium
 * Author: Lorenzo Natale
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

#ifndef __THREADTABLE2__
#define __THREADTABLE2__

#include <ace/Thread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/CanBusInterface.h>

#include "canControlConstants.h"
#include "canControlUtils.h"

class ThreadTable2
{
private:
    int _pending;
    int _timedOut;
    yarp::dev::CanBuffer _replies;
    yarp::dev::ICanBufferFactory *ic;
    yarp::os::Semaphore _synch;
    int _replied;
    ACE_thread_t _handle;
    yarp::os::Semaphore _mutex;

    inline void lock()
    { _mutex.wait(); }

    inline void unlock()
    { _mutex.post(); }

public:
    ThreadTable2();
    ~ThreadTable2();

    void clear();

    // initialize, need factory to create internal buffer of 
    // messages which will store replies
    void init(yarp::dev::ICanBufferFactory *i);

    // set number of pending requests, reset
    inline void setPending(int pend);

    // wait on semaphore, usually thread sleeps here after
    // has issued a list of requests to the can 
    void synch()
    { _synch.wait(); }

    // true if there are pending requests
    bool pending()
    {
        lock();
        bool ret=(_pending != 0) ? true:false;
        unlock();
        return ret;
    }

    // true if at least one time out occurred
    inline bool timedOut()
        {
            lock();
            bool ret=(_timedOut!=0)?true:false;
            unlock();
            return ret;
        }

    // notify that one of the requests timed out
    // if no other requests are pending the waiting thread
    // is released
    inline bool timeout();

    // push a reply, thread safe
    // wake up wating thread when all messages are received
    inline bool push(const yarp::dev::CanMessage &m);

    ACE_thread_t &handle()
    { return _handle; }


    //get can message from joint number
    inline yarp::dev::CanMessage *getByJoint(int j, const unsigned char *destInv);

    //get n-nth message in the list of replies
    inline yarp::dev::CanMessage *get(int n);
};

yarp::dev::CanMessage *ThreadTable2::get(int n)
{
    if (n<0 || n>=_replied)
        return 0;

    return &_replies[n];
}

yarp::dev::CanMessage *ThreadTable2::getByJoint(int j, const unsigned char *destInv)
{
    for(int k=0;k<_replied;k++)
        if (getJoint(_replies[k], destInv)==j)
            return &_replies[k];
    return 0;
}

bool ThreadTable2::push(const yarp::dev::CanMessage &m)
{
    lock();
    if (_replied>=BUF_SIZE)
        {
            unlock();
            return false; //full, should not happen
        }

    _replies[_replied]=m;

    _replied++;
    _pending--;
    if (_pending==0)
        _synch.post();
    
    unlock();
    return true;
}

bool ThreadTable2::timeout()
{
    lock();
    _replied++;
    _pending--;
    _timedOut++;
    if (_pending==0)
        {
            _synch.post();
        }
    unlock();
    return true;
}

void ThreadTable2::setPending(int pend)
{
    lock();
    _pending=pend;
    _replied=0;
    _timedOut=0;
    unlock();
}

#endif
