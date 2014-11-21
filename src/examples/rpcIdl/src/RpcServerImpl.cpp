#include <yarp/os/all.h>
#include <RpcServerImpl.h>

#include <iostream>

RpcServerImpl::RpcServerImpl() 
{ 
    answer=-1; 
    running=true;
}

int32_t RpcServerImpl::get_answer() 
{ 
    std::cout<<"The answer is "<<answer<<std::endl; 
    return answer;
}

bool RpcServerImpl::set_answer(const int32_t rightAnswer) 
{ 
    if (running)
    {
        answer=rightAnswer; 
        return true;
    }
    else
        return false;
}

int32_t RpcServerImpl::add_int(const int32_t x) 
{ 
    if (running)
        answer+=x;

    return answer;
}

bool RpcServerImpl::start() 
{ 
    running=true; 
    return true;
}

bool RpcServerImpl::stop() 
{ 
    running=false; 
    return true;
}

bool RpcServerImpl::is_running() 
{ 
    return running;
}

