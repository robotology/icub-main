// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include "eoRequestsQueue.hpp"

/////////////////////  eoThreadList  ///////////////////
eoThreadArray::eoThreadArray()
{
	index=0;
	pool=new eoThreadEntry[EO_THREADARRAY_MAX_THREADS];
	//    for(int k=0;k<EO_THREADLIST_MAX_THREADS;k++)
	//        pool[k].init(ic);
}

eoThreadArray::~eoThreadArray()
{
	delete [] pool;
}

bool eoThreadArray::getId(int &i)
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

/////////////////////  eoThreadEntry  ///////////////////
eoThreadEntry::eoThreadEntry():_synch(0)
{
	clear();
}

eoThreadEntry::~eoThreadEntry()
{


}

void eoThreadEntry::clear()
{
	lock();
	_pending=0;
	_replied=0;
	unlock();
}

void eoThreadEntry::init(void)
{

}


/* Non dovrebbero più servire

yarp::dev::CanMessage *eoThreadEntry::get(int n)
{
    if (n<0 || n>=_replied)
        return 0;

    return &_replies[n];
}

yarp::dev::CanMessage *eoThreadEntry::getByJoint(int j, const unsigned char *destInv)
{
    for(int k=0;k<_replied;k++)
        if (getJoint(_replies[k], destInv)==j)
            return &_replies[k];
    return 0;
}
 */

bool eoThreadEntry::push(void)
{
	lock();

//	_replied++;
	_pending--;
	if (_pending==0)
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
	if (_pending==0)
	{
		_synch.post();
	}
	unlock();
	return true;
}

//inline void eoThreadEntry::setPending(int pend)


/////////////////////  eoThreadFifo  ///////////////////

// A pop function; get and destroy from front, just get thread id
bool eoThreadFifo::pop(eoThreadId &ret)
{
	eoThreadId tmp =-1;
	if (empty())
		return false;

	tmp=front();
	pop_front();

	ret=tmp;
	return true;
}

// Push a thread id from back. waitTime is initialized to zero
bool eoThreadFifo::push(eoThreadId id)
{
//	eoThreadId tmp = id;
//	tmp.waitTime=0;
	push_back(id);
	return true;
}



/////////////////////  eoRequestsQueue  ///////////////////

eoRequestsQueue::eoRequestsQueue(int num_msgs)
{
	print_debug(AC_debug_file, "Allocating %d\n", num_msgs);
//	elements = num_msgs;
//	njoints  = joints;
	threadPool = new eoThreadArray[EO_THREADARRAY_MAX_THREADS];
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

// pop a request
int eoRequestsQueue::pop(int nv_index)
{
	if (whole_pendings<=0)
	{
		print_debug(AC_debug_file, "Error, queue of requests empty");
		return -1;
	}

	int ret;
	eoThreadFifo *fifo=getFifo(nv_index);
	if (!fifo)
		return -1;

	if (!fifo->pop(ret))
		return -1;
	whole_pendings--;
	return ret;
}


// append requests
void eoRequestsQueue::append(const eoRequest &rqst)
{
	eoThreadFifo *fifo=getFifo(rqst.nvid);
	if (!fifo)
		return;

	fifo->push(rqst.threadId);
	whole_pendings++;
}
