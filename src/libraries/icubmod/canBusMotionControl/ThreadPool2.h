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

#include <ace/config.h>
#include <ace/Thread.h>
#include "canControlConstants.h"
#include "ThreadTable2.h"

/*
 * This list contains the association between each thread and a consecutive id. */
class ThreadPool2
{
private:
    inline bool getNew(const ACE_thread_t &s, int &i)
    {
        fprintf(stderr, "Registering new thread %d out of %d\n", index, CANCONTROL_MAX_THREADS);
        if (index>=CANCONTROL_MAX_THREADS)
            {
                fprintf(stderr, "ThreadPool: ERROR reached max number of threads\n");
                i=-1;
                return false;
            }

        i=index;
        index++;
        pool[i].handle()=s;
        return true;
    }

    inline int checkExists(const ACE_thread_t &s)
    {
        for(int i=0; i<index; i++)
            {
                if (pool[i].handle()==s)
                    return i;
            }
        return -1;

    }

    ThreadTable2 *pool;
    int index;

public:
    ThreadPool2(yarp::dev::ICanBufferFactory *ic);

    ~ThreadPool2();

    inline void reset()
    {
        index=0;
    }

    inline bool getId(int &i)
    {
        ACE_thread_t self=ACE_Thread::self();
        int id;
        bool ret=true;
        id=checkExists(self);

        if (id==-1)
            {
                ret=getNew(self,id);
            }
        i=id;
        return ret;
    }

    inline ThreadTable2 *getThreadTable(int id)
    {
        if ((id<0) || (id>CANCONTROL_MAX_THREADS))
            return 0;

        return pool+id;
    }
};
