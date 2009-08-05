#include "webots_common.h"
#include <yarp/os/ConstString.h>
#include <yarp/os/all.h>

using namespace yarp::os;

WebotsCommon *WebotsCommon::singleton = new WebotsCommon;

WebotsCommon::WebotsCommon()
{
  for(int i = 0;i<SEMAPHORE_NUM;i++)
    {
      sem_webots[i].wait();
      sem_control[i].wait();
    }
}

WebotsCommon::~WebotsCommon()
{
}


WebotsCommon *WebotsCommon::getInstance()
{
  //to be sure there is no concurrent creation of the instance
  //WebotsCommon::sem_creation.wait();

  if(singleton==NULL)
    singleton = new WebotsCommon();

  //WebotsCommon::sem_creation.post();
  
  return singleton;
}

void WebotsCommon::sem_control_post()
{
  for(int i = 0;i<SEMAPHORE_NUM;i++)
    {
      sem_control[i].post();
    }
}

void WebotsCommon::sem_webots_wait()
{
  for(int i = 0;i<SEMAPHORE_NUM;i++)
    {
      sem_webots[i].wait();
    }
}

void WebotsCommon::sem_control_wait(int sem_num)
{
  if(sem_num<SEMAPHORE_NUM)
    sem_control[sem_num].wait();
}

void WebotsCommon::sem_webots_post(int sem_num)
{
  if(sem_num<SEMAPHORE_NUM)
    sem_webots[sem_num].post();
}
