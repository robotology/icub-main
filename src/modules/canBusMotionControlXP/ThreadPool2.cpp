#include "ThreadPool2.h"

ThreadPool2::ThreadPool2(yarp::dev::ICanBufferFactory *ic)
{
    index=0;
    pool=new ThreadTable2[CANCONTROL_MAX_THREADS];
    for(int k=0;k<CANCONTROL_MAX_THREADS;k++)
        pool[k].init(ic);
}

ThreadPool2::~ThreadPool2()
{
    delete [] pool;
}
