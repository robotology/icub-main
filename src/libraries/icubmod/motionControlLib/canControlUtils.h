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


#ifndef __CANUTILS__
#define __CANUTILS__

#include <list>

#include "yarp/dev/CanBusInterface.h"

/**
 * Extract least significative 4 bits, from the id of a can msg.
 * In our protocol this is the recipient of a message.
 * Message id: 0000 0XXX SSSS RRRR
 */
inline int getRcp(const yarp::dev::CanMessage &m)
{
    return (m.getId()&0x0f);
}

/**
 * Extract most significative 4 bits of the least significative byte from the id of a can msg.
 * Message id: 0000 0XXX SSSS RRRR
 * In our protocol this is the sender of a message
 */
inline int getSender(const yarp::dev::CanMessage &m)
{
    return (m.getId()&0xf0)>>4;
}

/**
 * Extract message class, three bit least significative of the first byte of the message ID.
 * Message id: 0000 0CCC SSSS RRRR
 */
inline int getClass(const yarp::dev::CanMessage &m)
{
    return (m.getId()&0x700);
}

/**
 * Extract 7 lsb of first byte; this is the message type in our protocol.
 */
inline unsigned char getMessageType(const yarp::dev::CanMessage &m)
{
    return m.getData()[0]&0x7f;
}

/**
 * Extract the joint number to which a RECEIVED message is referring to.
 * This is the conversion message -> joint (uses can id and first bit of 
 * first data byte in the can message)
 */
inline int getJoint(const yarp::dev::CanMessage &m, const unsigned char *invM)
{
    int sender=getSender(m);
    int odd=((m.getData()[0]&0x80)==0) ? 0:1;
    return invM[sender]+odd;
}


/**
 * Extract least significative 4 bits, from the id of a can msg.
 * In our protocol this is the recipient of a message.
 * Message id: 0000 0XXX SSSS RRRR
 */
template<class T>
long getRcp(const T &m)
{
    return (m.id&0x0f);
}

/**
 * Extract most significative 4 bits of the least significative byte from the id of a can msg.
 * Message id: 0000 0XXX SSSS RRRR
 * In our protocol this is the sender of a message
 */
template<class T>
long getSender(const T &m)
{
    return (m.id&0xf0)>>4;
}

/**
 * Extract message class, three bit least significative of the first byte of the message ID.
 * Message id: 0000 0CCC SSSS RRRR
 */
template<class T>
long getClass(const T &m)
{
    return (m.id&0x700);
}

/**
 * Extract 7 lsb of first byte; this is the message type in our protocol.
 */
template<class T>
unsigned char getMessageType(const T &m)
{
    return m.data[0]&0x7f;
}

/**
 * Extract the joint number to which a RECEIVED message is referring to.
 * This is the conversion message -> joint (uses can id and first bit of 
 * first data byte in the can message)
 */
template<class T>
int getJoint(const T &m, const unsigned char *invM)
{
    int sender=getSender(m);
    int odd=((m.data[0]&0x80)==0) ? 0:1;
    return invM[sender]+odd;
}

const int DEBUG_PRINTF_BUFFER_LENGTH=255;

inline void DEBUG(const char *fmt, ...)
{
#ifdef CAN_DEBUG
    va_list ap; 
    va_start(ap, fmt);
    char buffer[DEBUG_PRINTF_BUFFER_LENGTH];
#ifdef WIN32
    _vsnprintf(buffer, DEBUG_PRINTF_BUFFER_LENGTH, fmt, ap); 
#else
    vsnprintf(buffer, DEBUG_PRINTF_BUFFER_LENGTH, fmt, ap); 
#endif
    fprintf(stderr, "%s", buffer);
    va_end(ap);
#endif
}


struct ThreadId
{
    int id;
    unsigned int waitTime;  //ms
};

// A fifo of threads. There is one on each entry in the RequestsQueue.
class ThreadFifo: public std::list<ThreadId>
{
 public:
    ThreadFifo(){}

    // A pop function; get and destroy from front, just get thread id
    inline bool pop(int &ret)
    {
        ThreadId tmp;
        if (empty())
            return false;

        tmp=front();
        pop_front();
        
        ret=tmp.id;
        return true;
    }

    // Push a thread id from back. waitTime is initialized to zero
    inline bool push(int id)
    {
        ThreadId tmp;
        tmp.id=id;
        tmp.waitTime=0;
        push_back(tmp);
        return true;
    }
};

// A structure to hold a request (joint, msg and waiting thread)
struct CanRequest
{
    int joint;
    int msg;
    int threadId;
};

// A table, the index is a given can message type+the joint number.
// Each entry stores a list of waiting threads.
// At the moment the size of this table is statically determined (
// maximum size, given the number of joints and the number of messages,
// but it could be allocated at runtime, when requests arrive).
class RequestsQueue
{
private:
    ThreadFifo *requests;
    int njoints;
    int num_of_messages;
    int pendings;
    int elements;
public:
    RequestsQueue(int joints, int num_msgs)
    {
        DEBUG("Allocating %d x %d\n", joints, num_msgs);
        elements=joints*num_msgs;
        requests=new ThreadFifo[elements];
        num_of_messages=num_msgs;
        njoints=joints;
        pendings=0;
    }

    ~RequestsQueue()
    {
        delete [] requests;
    }

    ThreadFifo *getFifo(int j, int msg)
    {
        unsigned int i=msg&0x7F;
        // fprintf(stderr, "Asking FIFO for joint:%d msg:%d\n", j, i);
        int index=j*num_of_messages+i;
        DEBUG("%d %d\n", index, elements);

        if ((index>=0)&&(index<elements))
            return requests+index;
        else
            return 0;
    }

    // pop a request
    int pop(int j, int msg)
    {
        if (pendings<=0)
            {
                DEBUG("Error, queue of requests empty");
                return -1;
            }

        int ret;
        ThreadFifo *fifo=getFifo(j, msg);
        if (!fifo)
            return -1;

        if (!fifo->pop(ret))
            return -1;
        pendings--;
        return ret;
    }

    int getPending()
    {
        return pendings;
    }

    // append requests
    void append(const CanRequest &rqst)
    {
        ThreadFifo *fifo=getFifo(rqst.joint, rqst.msg);
        if (!fifo)
            return;

        fifo->push(rqst.threadId);
        pendings++;
    }

    inline int getNJoints()
        {return njoints;}
    inline int getNMessages()
        {return num_of_messages;}
};

#endif
