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

#ifndef __THREADTABLE__
#define __THREADTABLE__

#include <ace/Thread.h>
#include <yarp/os/Semaphore.h>

#include "canControlConstants.h"
#include "canControlUtils.h"

template<class T>
class ThreadTable
{
private:
    int _pending;
    T _replies[BUF_SIZE];
    yarp::os::Semaphore _synch;
    int _replied;
    ACE_thread_t _handle;
    yarp::os::Semaphore _mutex;

    inline void lock()
    { _mutex.wait(); }

    inline void unlock()
    { _mutex.post(); }

public:
 ThreadTable():_synch(0)
    {
        clear();
    }

    void clear()
    {
        lock();
        _pending=BUF_SIZE;
        _replied=0;
        unlock();
    }

    // this call is thread safe, not required
    void setPending(int pend)
    {
        lock();
        _pending=pend;
        _replied=0;
        unlock();
    }

    void synch()
    {
        _synch.wait();
    }

    // thread safe
    bool pending()
    {
        lock();
        bool ret=(_pending != 0) ? true:false;
        unlock();
        return ret;
    }

    // push a reply, thread safe
    bool push(const T &m)
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
        {
            _synch.post();
        }

        unlock();
        return true;
    }

    ACE_thread_t &handle()
    { return _handle; }

    // not thread safe, but it is "probably" ok
    T *getByJoint(int j, const unsigned char *destInv)
    {
        for(int k=0;k<_replied;k++)
            if (getJoint(_replies[k], destInv)==j)
                return _replies+k;
        return 0;
    }

    // not thread safe, but it is "probably" ok
    T *get(int n)
    {
        if (n<0 || n>=_replied)
            return 0;

        return _replies+n;
    }
};

#endif
