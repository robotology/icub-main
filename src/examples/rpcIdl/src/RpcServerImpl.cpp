#include <yarp/os/all.h>
#include <RpcServerImpl.h>

#include <iostream>

RpcServerImpl::RpcServerImpl()
{
  answer=-1;
}

int32_t RpcServerImpl::get_answer()
{
  std::cout << "The answer is "<< answer << std::endl;   
  return answer;
}

bool RpcServerImpl::set_answer(const int32_t rightAnswer)
{
  answer=rightAnswer;
  return true;
}

int32_t RpcServerImpl::add_one(const int32_t x)
{
  return false;
}

bool RpcServerImpl::start()
{
  return true;
}

bool RpcServerImpl::stop()
{
  return true;
}


bool RpcServerImpl::is_running()
{
  return true;
}

