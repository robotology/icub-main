#include "ThreadTable2.h"
#include "canControlConstants.h"

ThreadTable2::ThreadTable2():_synch(0)
{
    ic=0;
    clear();
}

ThreadTable2::~ThreadTable2()
{
    if(ic!=0)
        ic->destroyBuffer(_replies);
    ic=0;
}

void ThreadTable2::clear()
{
    lock();
    _pending=BUF_SIZE;
    _replied=0;
    unlock();
}

void ThreadTable2::init(yarp::dev::ICanBufferFactory *i)
{
    ic=i;
    _replies=ic->createBuffer(BUF_SIZE);
}

