// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules
 * \defgroup TheEthManager TheEthManager
 *
*/
/* Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Alberto Cardellino
 * email: alberto.cardellino@iit.it
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

#ifndef eoRequestsQueue_HPP_
#define eoRequestsQueue_HPP_

/* This class will handle the get(ask) requests from iCubInterface to the EMS boards.
 * It is based on a table [N_joints * N_messages] wide where each element is a fifo list of (pointer to) waiting threads
 *
 * The waiting thread is represented by a structure called eoThreadEntry
 *
 * Since a thread can start more requests at time and then wait for all replies to arrive, it needs to store the number of
 * requests and will be waked up only when it gets all replies ot a timeout is reached.
 */


#include <ace/config.h>
#include <ace/Thread.h>
#include <yarp/os/Semaphore.h>
#include <list>
#include "ace/Semaphore.h"
#include "stdint.h"


#define EO_THREADARRAY_MAX_THREADS 500

typedef int eoThreadId;

class eoThreadArray;
class eoThreadEntry;
class eoRequestsQueue;

// A structure to hold a request :joint (not so useful), nvid and waiting thread
typedef struct
{
    int joint;
    //uint32_t nvid;
    uint32_t prognum;
    int threadId;
}eoRequest;


/////////////////////  eoThreadEntry  ///////////////////

// This class represent a thread waiting for something.

// era ThreadTable2 NEL CAN
class eoThreadEntry
{
private:
    // Semaphore where the thread will sleep onto -> it supports timeout
    yarp::os::Semaphore _synch;
    ACE_thread_t _handle;

    int _pending;

    double _timeout;

    // internal mutex, to avoid concurrent operations
    yarp::os::Semaphore _mutex;

    inline void lock()
    { _mutex.wait(); }

    inline void unlock()
    { _mutex.post(); }

public:
    eoThreadEntry();
    ~eoThreadEntry();

    eoThreadId id;

    void setTimeout(double to);

    // set number of pending requests, reset
    inline void setPending(int pend)
    {
        lock();
        _pending+=pend;
        unlock();
    };

    // wait on semaphore, usually thread sleeps here after
    // has issued a list of requests to the EMS
    // returns 0 if correctly waked by received messages, -1 if quit for timeout
    int synch();

    // true if there are pending requests
    bool pending()
    {
        lock();
        bool ret=(_pending != 0) ? true:false;
        unlock();
        return ret;
    }

    // notify that one of the requests timed out
    bool timeout();

    // push a reply, thread safe
    // wake up wating thread when all messages are received
    bool push(void);

    ACE_thread_t &handle()
    { return _handle; }

};



/////////////////////  eoThreadArray  ///////////////////

//  This list contains the association between each thread and a consecutive id.

class eoThreadArray
{
private:
    inline bool getNew(const ACE_thread_t &s, int &i)
    {
        fprintf(stderr, "Registering new thread %d out of %d\n", index, EO_THREADARRAY_MAX_THREADS);
        if (index>=EO_THREADARRAY_MAX_THREADS)
        {
            fprintf(stderr, "ThreadPool: ERROR reached max number of threads\n");
            i=-1;
            return false;
        }

        i=index;
        pool[i].handle()= s;
        pool[i].id = index;
        index++;
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

    eoThreadEntry *pool;
    int index;

public:
    eoThreadArray();

    ~eoThreadArray();

    inline void reset()
    {
        index=0;
    }

    bool getId(int *i);

    inline eoThreadEntry *getThreadTable(int id)
    {
        if ((id<0) || (id>EO_THREADARRAY_MAX_THREADS))
            return NULL;

        return pool+id;
    }
};


/////////////////////  eoThreadFifo  ///////////////////

// A fifo of threads. There is one on each entry in the RequestsQueue.
// Each elements actually points to the corresponding eoThreadEntry

class eoThreadFifo: public std::list<eoThreadId>
{
 public:
    yarp::os::Semaphore _mutex;
    eoThreadFifo():_mutex(1) {}

    // A pop function; read and destroy from front, just get thread id
    eoThreadId pop();

    // Push a thread id from back. waitTime is initialized to zero
    bool push(eoThreadId id);
};

/////////////////////  eoRequestsQueue  ///////////////////

// A table, the index is a given NVID for the specific joint.
// Each entry stores a list of waiting threads.
// At the moment the size of this table is statically determined (
// maximum size, given the number of joints and the number of messages,
// but it could be allocated at runtime, when requests arrive).

class eoRequestsQueue
{
private:
    eoThreadFifo *requests;
    int num_of_messages;
    int whole_pendings;

//    int njoints;
//    int elements;

public:

    eoRequestsQueue(int NV_num);
    ~eoRequestsQueue();

    eoThreadFifo *getFifo(int nv_index);
    eoThreadArray *threadPool;

    // pop a request
//     int pop(int nv_index);


    // append requests
    void append(const eoRequest &rqst);
    bool cleanTimeouts(eoThreadId id);

    inline int getNMessages()       {return num_of_messages;}
    inline int getPending()      	 {return whole_pendings;}

    //    inline int getNJoints()
    //        {return njoints;}
};

#endif /* eoRequestsQueue_HPP_ */
