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

#ifndef __THREADPOOL__
#define __THREADPOOL__

/*
 * This list contains the association.between each thread and a consecutive id.
 */
template <class T>
class ThreadPool
{
private:
    bool getNew(const ACE_thread_t &s, int &i)
    {
        if (index>=MAX_THREADS)
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

    int checkExists(const ACE_thread_t &s)
    {
        for(int i=0; i<index; i++)
            {
                if (pool[i].handle()==s)
                    return i;
            }
        return -1;
    }

    ThreadTable<T> pool[MAX_THREADS];
    int index;

public:
    ThreadPool()
    {
        index=0;
    }

    void reset()
    {
        index=0;
    }

    bool getId(int &i)
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

    ThreadTable<T> *getThreadTable(int id)
    {
        if ((id<0) || (id>MAX_THREADS))
            return 0;

        return pool+id;
    }
};

#endif
