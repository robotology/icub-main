// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include "eoRequestsQueue.hpp"
#include "Debug.h"

/////////////////////  eoThreadList  ///////////////////
eoThreadArray::eoThreadArray()
{
    index=0;
    //pool=new eoThreadEntry;
    pool=new eoThreadEntry[EO_THREADARRAY_MAX_THREADS];
    //    for(int k=0;k<EO_THREADLIST_MAX_THREADS;k++)
    //        pool[k].init(ic);
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
    _timeout = 1.1f;
    clear();
}

eoThreadEntry::~eoThreadEntry()
{
    clear();
}

void eoThreadEntry::clear()
{
    lock();
    _pending=0;
    _replied=0;
    unlock();
}

void eoThreadEntry::init(double to)
{
    double _timeout = to;
}

int eoThreadEntry::synch()
{
    if(false == _synch.waitWithTimeout(_timeout))
    {
        // printf("Semaphore timed out!!\n");
        return -1;
    }

    return 0;
}

bool eoThreadEntry::push(void)
{
    lock();

//  _replied++;
    _pending--;

    if(_pending==0)
        _synch.post();

    unlock();
    return true;
}

bool eoThreadEntry::timeout()
{
    lock();
    _replied++;
    _pending--;
    _timedOut++;

    if(_pending==0)
    {
        _synch.post();
    }

    unlock();
    return true;
}

//inline void eoThreadEntry::setPending(int pend)


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
//  elements = num_msgs;
//  njoints  = joints;
    threadPool = new eoThreadArray;//[EO_THREADARRAY_MAX_THREADS];
    num_of_messages = num_msgs;
    requests   = new eoThreadFifo[num_of_messages];
    whole_pendings = 0;
}

eoRequestsQueue::~eoRequestsQueue()
{
    delete [] requests;
}

eoThreadFifo *eoRequestsQueue::getFifo(int nv_index)
{
    if(nv_index < num_of_messages)
        return &requests[nv_index];
    else
        return 0;
}

// // pop a request
// int eoRequestsQueue::pop(int nv_index)
// {
//     if(whole_pendings<=0)
//     {
//         yError() << "Error, queue of requests empty";
//         return -1;
//     }
// 
//     int ret;
//     eoThreadFifo *fifo=getFifo(nv_index);
// 
//     if(!fifo)
//         return -1;
// 
//     if( (ret=fifo->pop()) < 0)
//     {
//         yError() << "Received an answer message nobody is waiting for";
//         return -1;
//     }
// 
//     whole_pendings--;
//     return ret;
// }


// append requests
void eoRequestsQueue::append(const eoRequest &rqst)
{
    eoThreadFifo *fifo=getFifo(rqst.nvid);

    if(!fifo)
        return;

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
                printf("Cleaning timeouts for thread Id %d, req %d\n", id, i);
                // to wake threads sleeping here ... is it correct this way??
                eoThreadEntry *th = threadPool->getThreadTable(id);
                th->push();
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
