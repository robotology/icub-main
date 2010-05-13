#include "fakeCan.h"
#include <iostream>

#include <yarp/os/Bottle.h>

using namespace std;
using namespace yarp::dev;
using namespace yarp::os;

FakeCan::FakeCan()
{}

FakeCan::~FakeCan()
{}

/*ICan*/
bool FakeCan::canSetBaudRate(unsigned int rate)
{
    return true;
}

bool FakeCan::canGetBaudRate(unsigned int *rate)
{
    return true;
}

bool FakeCan::canIdAdd(unsigned int id)
{
    return true;
}

bool FakeCan::canIdDelete(unsigned int id)
{
    return true;
}

bool FakeCan::canRead(CanBuffer &msgs, 
        unsigned int size, 
        unsigned int *read,
        bool wait)
{
    replies.lock();
    unsigned int l=replies.size();

    if (size<l)
        l=size;

    *read=l;

    MsgIt it=replies.begin();
    int k=0;
    while(it!=replies.end())
    {    
        FCMSG *r=reinterpret_cast<FCMSG *>(msgs[k].getPointer());
        *r=*it;
        it++;
        k++;
    }

    replies.clear();
    replies.unlock();
    return true;
}

bool FakeCan::canWrite(const CanBuffer &msgs,
        unsigned int size,
        unsigned int *sent,
        bool wait)
{
    BoardsIt it=boardList.begin();

    while(it!=boardList.end())
    {
        FakeBoard *tmp=(*it);
        for(unsigned int k=0;k<size;k++)
        {
            CanMessage &m=const_cast<CanBuffer &>(msgs)[k];
            FCMSG *canM=reinterpret_cast<FCMSG *>(m.getPointer());
            tmp->pushMessage(*canM);
        }
        it++;
    }

    *sent=size;

    return true;
}

/*Device Driver*/
bool FakeCan::open(yarp::os::Searchable &par)
{
    cerr<<"Opening FakeCan network" << endl;

    //fprintf(stderr, "%s", par.toString().c_str());

    int njoints=par.findGroup("GENERAL").find("Joints").asInt();
    Bottle &can = par.findGroup("CAN");
    Bottle ids=can.findGroup("CanAddresses");

    if (ids.size()<njoints/2)
    {
        fprintf(stderr, "Check ini file, wrong number of board ids or joints\n");
        return false;
    }
    
    for(int i=1;i<=njoints/2;i++)
    {
        FakeBoard *tmp=new FakeBoard;
        int id=ids.get(i).asInt();
        tmp->setId(id);   //just as a test
        tmp->setReplyFifo(&replies);
        tmp->start();
        boardList.push_back(tmp);
    }
        
    return true;
}

bool FakeCan::close()
{
    cerr<<"Closing FakeCan network" << endl;

    BoardsIt it=boardList.begin();

    while(it!=boardList.end())
    {
        FakeBoard *tmp=(*it);
        tmp->stop();
        delete tmp;
        it++;
    }

    boardList.clear();

    return true;
}
