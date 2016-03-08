// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include "eoRequestsQueue.hpp"
#include <yarp/os/LogStream.h>

/////////////////////  eoThreadList  ///////////////////
eoThreadArray::eoThreadArray()
{
    index=0;
    pool=new eoThreadEntry[EO_THREADARRAY_MAX_THREADS];
}

eoThreadArray::~eoThreadArray()
{
    delete [] pool;
}

bool eoThreadArray::getId(int *i)
{
    ACE_thread_t self=ACE_Thread::self();
    int id;
    bool ret=true;
    id=checkExists(self);

    if(id==-1)
    {
        ret=getNew(self,id);
    }

    *i=id;
    return ret;
}

/////////////////////  eoThreadEntry  ///////////////////
eoThreadEntry::eoThreadEntry(): _synch(0),
                                _mutex(1)
{
    id = -1;
    _pending = 0;
    _timeout = 0.3f;
}

eoThreadEntry::~eoThreadEntry() { }


void eoThreadEntry::setTimeout(double to)
{
    _timeout = to;
}

int eoThreadEntry::synch()
{
    int ret;
    _synch.waitWithTimeout(_timeout) ? ret = 0 : ret = -1;
    return ret;
}

bool eoThreadEntry::push(void)
{
    lock();

    _pending--;

    if(_pending==0)
        _synch.post();

    unlock();
    return true;
}

// DO NOT post mutex here!! If timeout occour, at this time the thread is already wake!!
bool eoThreadEntry::timeout()
{
    lock();
    // no need to keep counting
    _pending=0;

    unlock();
    return true;
}



/////////////////////  eoThreadFifo  ///////////////////

// A pop function; read and destroy from front, just get thread id
eoThreadId eoThreadFifo::pop( )
{
    _mutex.wait();
    eoThreadId ret = -1;

    if(!empty())
    {
        ret=front();            // se la lista è vuota cosa ritorna? non si sa, per cui c'è il check prima del front
        pop_front();                  // erase the first element of the queue, it means the oldest
    }
    else
        yError() << "Received an answer message nobody is waiting for (eoThreadFifo::pop)";

    _mutex.post();
    return ret;
}

// Push a thread id from back. waitTime is initialized to zero
bool eoThreadFifo::push(eoThreadId id)      // add an element at the end of the queue
{
    _mutex.wait();
    push_back(id);
    _mutex.post();
    return true;
}



/////////////////////  eoRequestsQueue  ///////////////////

eoRequestsQueue::eoRequestsQueue(int num_msgs)
{
    yTrace() << "num msg=" << num_msgs;
    num_of_messages = num_msgs;
    threadPool = new eoThreadArray;
    requests   = new eoThreadFifo[num_of_messages];
    whole_pendings = 0;
}

eoRequestsQueue::~eoRequestsQueue()
{
    delete threadPool;
    delete [] requests;
}

eoThreadFifo *eoRequestsQueue::getFifo(int nv_index)
{
    if(nv_index < num_of_messages)
        return &requests[nv_index];
    else
        return 0;
}


// append requests
void eoRequestsQueue::append(const eoRequest &rqst)
{
    yTrace() << "th_id=" << rqst.threadId << " nv_prog_num=" << rqst.prognum << " joint=" << rqst.joint;
    eoThreadFifo *fifo=getFifo(rqst.prognum);

    if(!fifo)
    {
        yError() << "eoRequestsQueue::append: fifo is null for th_id=" << rqst.threadId << " nvid=" << rqst.prognum << " joint=" << rqst.joint;
        return;
    }
    fifo->push(rqst.threadId);
    whole_pendings++;
}

bool eoRequestsQueue::cleanTimeouts(eoThreadId id)
{
    for(int i=0; i<getNMessages(); i++)
    {
        eoThreadFifo *fifo=getFifo(i);
        std::list<eoThreadId>::iterator it=fifo->begin();
        std::list<eoThreadId>::iterator end=fifo->end();

        while(it!=end)
        {
            if((*it) == id)
            {
//                printf("Cleaning timeouts for thread Id %d, req %d\n", id, i);

                eoThreadEntry *th = threadPool->getThreadTable(id);
                th->timeout();
                it=fifo->erase(it);  //it now points to the next element
            }
            else
                it++;
        }
    }

    // anything can be wrong here?? maybe if I don't find any requests issued by this thread, this means there is some mess here
    // ... but what can I do about it?
    return true;
}
